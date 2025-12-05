#include <OneWire.h>
#include <DallasTemperature.h>
// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

// DS18B20 data pin
constexpr uint8_t ONE_WIRE_BUS_PIN{ 2 };

// Relay control pin
constexpr uint8_t RELAY_PIN{ 8 };

constexpr bool CONNECT_TO_PC{false};

// Target temperature and hysteresis
constexpr float TARGET_TEMP_C{ 24.0f };  // desired temperature
constexpr float TEMP_MAX{ 28.0f };

constexpr float TEMP_ALLOWANCE { 0.25f};

// Relay logic level.
// DollaTek-style modules are usually "active LOW":
//   IN = LOW  -> relay energised (heater ON)
//   IN = HIGH -> relay off (heater OFF)
constexpr uint8_t RELAY_ACTIVE_STATE{ HIGH };
constexpr uint8_t RELAY_INACTIVE_STATE{ LOW };

// How often to read temperature (ms)
constexpr unsigned long READ_INTERVAL_MS{ 2000UL };  // 2 seconds

// -----------------------------------------------------------------------------
// Logger
// -----------------------------------------------------------------------------


template<bool PC_CON>
struct Logger;

template<>
struct Logger<true>
{
  static void begin(unsigned long baud)
  {
    Serial.begin(baud);
    while(!Serial);
  }

  template<typename... T>
  static void print(T... v) { Serial.print(v...); }

  template<typename... T>
  static void println(T... v) { Serial.println(v...); }

  static void flush() { Serial.flush(); }
};

template<>
struct Logger<false>
{
  static void begin(unsigned long) {}
  
  template<typename... T>
  static void print(T...) {}

  template<typename... T>
  static void println(T...) {}

  static void flush() {}
};


using Log = Logger<CONNECT_TO_PC>;

// -----------------------------------------------------------------------------
// Panic Handler
// -----------------------------------------------------------------------------
class Panic
{
public:
  using Callback = void (*)();
  static constexpr unsigned char MAX_CALLBACKS = 4U;

  static bool IsPanic()
  {
    if(is_panic_)
      Log::println(F("PANIC ACTIVE"));
    return is_panic_;
    // return false;
  }
  static void StartPanic() 
  {
    if(is_panic_) return;

    is_panic_ = true;
    Log::println(F("PANIC START"));

    for (unsigned char i = 0U; i < callback_count_; ++i)
    {
      if (callbacks_[i])
      {
        callbacks_[i]();
      }
    }
  }
  static void RegisterCallback(Callback cb)
  {
    if(callback_count_ < MAX_CALLBACKS)
    {
      callbacks_[callback_count_++] = cb;
    }
    else
    {
      Log::println(F("Panic callback list full"));
      StartPanic();
    }
  }
private:
  static bool is_panic_;
  static Callback callbacks_[MAX_CALLBACKS];
  static unsigned char callback_count_;
};

// -----------------------------------------------------------------------------
// LED Man
// -----------------------------------------------------------------------------
// LEDMan: template-based LED pattern controller.
// Each template argument is one state (mode).
// For state i, LED is ON for DurationsMs[i] ms, then OFF for DurationsMs[i] ms, repeating.
//
// Example usage:
//   using HeaterLED = LEDMan<10000UL, 1000UL, 50UL>;
//   // index 0: slow blink (10 s on, 10 s off)
//   // index 1: fast blink (1 s on, 1 s off)
//   // index 2: rapid blink (50 ms on, 50 ms off)

// LEDMan: each template argument is one logical LED state (mode).
// For state i, LED is ON for DurationsMs[i] ms, then OFF for DurationsMs[i] ms, repeat.
//
// Public API:
//   LEDMan<...>::SetState(stateIndex);   // 0 <= stateIndex < N; else Panic::StartPanic()
//   LEDMan<...>::Update();               // call regularly from loop() to advance timing and drive LED
//
// Assumes:
//   - Arduino environment (millis(), digitalWrite, LED_BUILTIN, HIGH/LOW).
//   - Panic::StartPanic() exists for error handling.

template<unsigned long... DurationsMs>
class LEDMan
{
public:

  // Change current state (mode). Panics and forces LED off on out-of-range.
  static void SetState(unsigned int state)
  {
    if (state >= NumStates)
    {
      // Invalid state request: panic and force LED off
      Panic::StartPanic();
      ledOn_ = false;
      digitalWrite(LED_BUILTIN, LOW);
      return;
    }

    if (state != currentStateIndex_)
    {
      currentStateIndex_ = state;
      ledOn_ = true;            // start new state in ON phase
      lastToggleMs_ = millis(); // reset phase timer
    }
  }

  // Advance timing in the current state and drive the LED.
  static void Update()
  {
    const unsigned long now     = millis();
    const unsigned long elapsed = now - lastToggleMs_;

    const unsigned long halfPeriod = durations_[currentStateIndex_];

    if (elapsed >= halfPeriod)
    {
      ledOn_ = !ledOn_;
      lastToggleMs_ = now;
    }

    digitalWrite(LED_BUILTIN, ledOn_ ? HIGH : LOW);
  }

private:
    // Number of states (modes)
    static const unsigned int NumStates = sizeof...(DurationsMs);

    // Durations array built from template parameter pack
    static const unsigned long durations_[NumStates];

    // Current state index, LED phase, and last toggle time
    static unsigned int    currentStateIndex_;
    static bool          ledOn_;
    static unsigned long lastToggleMs_;
};

// ---------------- Specialisation for 0 states: LED always OFF ----------------

template<>
class LEDMan<>
{
public:
  static void SetState(unsigned int) { Panic::StartPanic(); }
  static void Update() { }
};


using LEDIndicator = LEDMan<10000UL, 1000UL, 50UL>;
namespace LEDState
{
  enum
  {
    COOLING,
    HEATING,
    PANIC
  };
}
// -----------------------------------------------------------------------------
// Temp Controller
// -----------------------------------------------------------------------------

class TempController
{
private:
  enum State
  {
    HEATING,
    COOLING,
    OFF
  };
public:
  //anything should be able to turn it off but not on

  static void PrintState()
  {
    switch(st_)
    {
      case HEATING:
        Log::print(F("\t ST: HEATING"));
        break;
      case COOLING:
        Log::print(F("\t ST: COOLING"));
        break;
      case OFF:
        Log::print(F("\t ST: OFF"));
        break;
      default:
        break;
    }
  }
  static inline void Off()
  {
    if(heater_is_off_ == true) return;

    digitalWrite(RELAY_PIN, RELAY_INACTIVE_STATE);
    if(Panic::IsPanic())
    {
      st_ = OFF;
    }
    heater_is_off_ = true;
  }
  static void Update(float const current_temp_c)
  {
    if(st_ == OFF) return;
    if(current_temp_c > Max)
    {
      Panic::StartPanic();
      Off();
      return;
    }

    if(st_ == HEATING && current_temp_c >= (Target + TEMP_ALLOWANCE))
    {
      Off();
      st_ = COOLING;
      LEDIndicator::SetState(LEDState::COOLING);
    }
    else if(st_ == COOLING && current_temp_c <= (Target - TEMP_ALLOWANCE))
    {
      On();
      st_ = HEATING;
      LEDIndicator::SetState(LEDState::HEATING);
    }
  }

private:
  static inline void On()
  {
    if(heater_is_off_ == false) return;

    digitalWrite(RELAY_PIN, RELAY_ACTIVE_STATE);
    heater_is_off_ = false;
  }
private:
  static State st_;
  static bool heater_is_off_;
  static constexpr float Target = TARGET_TEMP_C;
  static constexpr float Max = TEMP_MAX;
};


// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

template<unsigned long... DurationsMs>
const unsigned long LEDMan<DurationsMs...>::durations_[LEDMan<DurationsMs...>::NumStates] =
{
  DurationsMs...
};

template<unsigned long... DurationsMs>
unsigned int LEDMan<DurationsMs...>::currentStateIndex_ = 0U;

template<unsigned long... DurationsMs>
bool LEDMan<DurationsMs...>::ledOn_ = false;

template<unsigned long... DurationsMs>
unsigned long LEDMan<DurationsMs...>::lastToggleMs_ = 0UL;

bool Panic::is_panic_ = false;
Panic::Callback Panic::callbacks_[Panic::MAX_CALLBACKS] = { 0 };
unsigned char Panic::callback_count_ = 0U;

TempController::State TempController::st_ = TempController::COOLING;
bool TempController::heater_is_off_ = true;

OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature sensors(&oneWire);

unsigned long GLastReadMs{ 0UL };
unsigned int GUndetectedCount { 0 };

// -----------------------------------------------------------------------------
// Arduino setup / loop
// -----------------------------------------------------------------------------

void setup()
{

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_INACTIVE_STATE);

  pinMode(LED_BUILTIN, OUTPUT);
  LEDIndicator::SetState(LEDState::COOLING); // start in cooling/idle pattern

  Log::begin(9600);

  Log::println(F("\nNico temp controller starting..."));
  Log::println(F("Target: 26 C, hysteresis: +/-0.5 C"));

  Panic::RegisterCallback([](){LEDIndicator::SetState(LEDState::PANIC);});
  Panic::RegisterCallback([](){TempController::Off();});

  sensors.begin();
  Log::print(F("Found DS18B20 devices: "));
  Log::println(sensors.getDeviceCount());
}

void loop()
{
  LEDIndicator::Update();
  const unsigned long now{ millis() };
  if (now - GLastReadMs < READ_INTERVAL_MS) return;
  GLastReadMs = now;
  if (Panic::IsPanic())
  {
    TempController::Off();
    return;
  }


  sensors.requestTemperatures();
  const float tempC{ sensors.getTempCByIndex(0) };

  if (tempC == DEVICE_DISCONNECTED_C)
  {
    Log::println(F("Error: DS18B20 not detected or disconnected!"));
    
    // Safety: turn heater OFF if sensor is missing for 10 seconds
    if(GUndetectedCount >= 5)
    {
      Panic::StartPanic();
      TempController::Off();
      Log::println(F("Heater -> OFF (fail-safe)"));
      return;
    }
    ++GUndetectedCount;
    
  }
  else
  {
    Log::print(F("Temperature: "));
    Log::print(tempC, 2);
    Log::print(F(" C"));
    TempController::PrintState();
    Log::println(F(" "));

    GUndetectedCount = 0;
    TempController::Update(tempC);
  }
}
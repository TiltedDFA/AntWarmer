#include <OneWire.h>
#include <DallasTemperature.h>
#include <Arduino.h>
// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

constexpr bool CONNECT_TO_PC{ true };

constexpr float TEMP_ALLOWANCE { 0.25f };

// Relay logic level.
// DollaTek-style modules are usually "active LOW":
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
    // while(!Serial);
  }

  template<typename... T>
  static void print(T const&... v)
  {
    //janky code expansion due to being on c++11
    int const dummy[] = { 0, ((void)Serial.print(v), 0)... };
    (void)dummy;
  }

  template<typename... T>
  static void println(T const&... v)
  {
    print(v...);
    Serial.println();
  }

  static void flush() { Serial.flush(); }
};

template<>
struct Logger<false>
{
  static void begin(unsigned long) {}
  
  template<typename... T>
  static void print(T const&...) {}

  template<typename... T>
  static void println(T const&...) {}

  static void flush() {}
};


using Log = Logger<CONNECT_TO_PC>;

// -----------------------------------------------------------------------------
// Panic Handler
// -----------------------------------------------------------------------------
enum class PanicReason : uint8_t
{
  None = 0,
  SensorDisconnected,
  OverMax,
  DesyncNoRise,
  LEDRegisterFail,
  Other
};
struct PanicInfo
{
  uint32_t ms;
  uint16_t line;
  uint8_t  uid;
  PanicReason reason;
};
const __FlashStringHelper* PanicReasonStr(PanicReason r)
{
  switch (r)
  {
    case PanicReason::SensorDisconnected: return F("SensorDisconnected");
    case PanicReason::OverMax:            return F("OverMax");
    case PanicReason::DesyncNoRise:       return F("DesyncNoRise");
    case PanicReason::LEDRegisterFail:    return F("LEDRegisterFail");
    case PanicReason::Other:              return F("Other");
    default:                              return F("None");
  }
}
class Panic
{
public:
  using Callback = void (*)();
  static constexpr unsigned char MAX_CALLBACKS = 4U;

  template<unsigned int N>
  static void Init(Callback const (&cbs)[N])
  {
    static_assert(N <= MAX_CALLBACKS, "Too many panic callbacks"); // pick capacity
    callback_count_ = static_cast<uint8_t>(N);
    for (uint8_t i = 0; i < callback_count_; ++i)
      callbacks_[i] = cbs[i];
  }
  static bool IsPanic()
  {
    return is_panic_;
  }

  static void StartPanic(PanicReason reason, uint8_t uid, uint16_t line) 
  {
    if(is_panic_) return;

    is_panic_ = true;

    panic_info_.ms = millis();
    panic_info_.line = line;
    panic_info_.uid = uid;
    panic_info_.reason = reason;
    

    for (unsigned char i = 0U; i < callback_count_; ++i)
    {
      if (callbacks_[i])
      {
        callbacks_[i]();
      }
    }
    
    Log::println(F("PANIC START"));
    PrintPanic();
  }
  static void PrintPanic()
  {
    if (panic_info_.reason == PanicReason::None)
    {
      Log::println(F("Panic: <none>"));
      return;
    }

    Log::println(F("Panic (latched):"));
    Log::print(F("  Reason: ")); Log::println(PanicReasonStr(panic_info_.reason));
    Log::print(F("  UID: "));    Log::println(panic_info_.uid);
    Log::print(F("  Line: "));   Log::println(panic_info_.line);
    Log::print(F("  Millis: ")); Log::println(panic_info_.ms);
  }

private:
  static bool is_panic_;
  static Callback callbacks_[MAX_CALLBACKS];
  static uint8_t callback_count_;
  static PanicInfo panic_info_;
};

//I usually do not like macros but __LINE__ is nice to have
#define PANIC(uid, reason) Panic::StartPanic((reason), (uid), static_cast<uint16_t>(__LINE__))


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


//fwd declaration
class TempController;


class LEDMan
{
private:
  static constexpr unsigned int MAX_CONTROLLERS = 4U;
public:
  static void Register(TempController const * tc)
  {
    if(tc == nullptr || num_controllers_ >= MAX_CONTROLLERS)
    {
      PANIC(0, PanicReason::LEDRegisterFail);
      return;
    }
    controllers_[num_controllers_++] = tc;
  }
  static unsigned long HalfPeriodForState(unsigned int const state)
  {
    switch(state)
    {
      case 0U: return 50UL;
      case 1U: return 1000UL;
      default: return 10000UL;
    }
  }

  static void UpdateState();

  // Advance timing in the current state and drive the LED.
  static void Update()
  {
    UpdateState();
    const unsigned long now     = millis();
    const unsigned long elapsed = now - lastToggleMs_;

    const unsigned long halfPeriod = HalfPeriodForState(currentStateIndex_);

    if (elapsed >= halfPeriod)
    {
      ledOn_ = !ledOn_;
      lastToggleMs_ = now;
    }

    digitalWrite(LED_BUILTIN, ledOn_ ? HIGH : LOW);
  }

private:

    static TempController const * controllers_[MAX_CONTROLLERS];
    static unsigned int num_controllers_;

    // Current state index, LED phase, and last toggle time
    static unsigned int    currentStateIndex_;
    static bool          ledOn_;
    static unsigned long lastToggleMs_;
};
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
  class DesyncMan
  {
  private:
    static constexpr float NEEDED_TEMP_CHANGE = 0.25f;
    static constexpr unsigned long TIME_TO_WAIT = 180000UL;
  public:
    DesyncMan(): 
      start_time_(),
      start_temp_(),
      not_inited_(true)
      {}

    void Reset() { not_inited_ = true; }
    bool Update(float const temp_c)
    {
      if(not_inited_) Begin(temp_c);
      return (millis() - start_time_ >= TIME_TO_WAIT && temp_c - start_temp_ < NEEDED_TEMP_CHANGE);
    }
  private:
    void Begin(float const temp_c)
    {
      start_time_ = millis();
      start_temp_ = temp_c;
      not_inited_ = false;
    }
  private:
    unsigned long start_time_;
    float start_temp_;
    bool not_inited_;
  };
public:
  //anything should be able to turn it off but not on
  TempController()=delete;
  TempController(uint8_t const uid, float const target, float const max, uint8_t sen_wire_pin, uint8_t relay_pin):
    uid_(uid),
    st_(TempController::COOLING),
    heater_is_off_(true),
    target_(target),
    max_(max),
    one_wire_(sen_wire_pin),
    sensor_(&one_wire_),
    relay_pin_(relay_pin),
    desync_man_()
  { }
  ~TempController()=default;
  void Begin()
  {
    pinMode(relay_pin_, OUTPUT);
    digitalWrite(relay_pin_, RELAY_INACTIVE_STATE);
    sensor_.begin();
  }
  void PrintState(float const temp_c)
  {
    Log::print(F("CTRL: "), uid_, F(" Temp: "), temp_c);
    switch(st_)
    {
      case HEATING:
        Log::print(F(" ST: HEATING"));
        break;
      case COOLING:
        Log::print(F(" ST: COOLING"));
        break;
      case OFF:
        Log::print(F(" ST: OFF"));
        break;
      default:
        break;
    }
    Log::print(F("\n"));
  }
  void Off()
  {
    //apparently bad
    // if(heater_is_off_ == true) return;

    digitalWrite(relay_pin_, RELAY_INACTIVE_STATE);
    if(Panic::IsPanic())
    {
      st_ = OFF;
    }
    heater_is_off_ = true;
  }
  bool IsHeating() const
  {
    return !heater_is_off_;
  }
  void Update(float const current_temp_c)
  {
    if(st_ == OFF) return;
    if(current_temp_c >= max_)
    {
      PANIC(uid_, PanicReason::OverMax);
      Off();
      return;
    }

    if(st_ == HEATING)
    {
      if(desync_man_.Update(current_temp_c))
      {
        PANIC(uid_, PanicReason::DesyncNoRise);
        Off();
        return;
      }
      if(current_temp_c >= (target_ + TEMP_ALLOWANCE))
      {
        Off();
        st_ = COOLING;
      }
    }
    else if(st_ == COOLING && current_temp_c <= (target_ - TEMP_ALLOWANCE))
    {
      desync_man_.Reset();
      On();
      st_ = HEATING;
    }
  }

  void Loop()
  {
    if(Panic::IsPanic()) return;

    sensor_.requestTemperatures();
    float const temp_c{ sensor_.getTempCByIndex(0) };

    if(temp_c == DEVICE_DISCONNECTED_C)
    {
      PANIC(uid_, PanicReason::SensorDisconnected);
      Log::println(F("CTRL: "), uid_, F("Heater -> OFF (fail-safe)"));
      return;
    }
    else
    {
      // Log::print(F("CTRL: "), uid_, F(" Temp: "), temp_c, F(" C\n"));
      this->PrintState(temp_c);
      this->Update(temp_c);
    }
  }

  private:
  inline void On()
  {
    if(heater_is_off_ == false) return;

    digitalWrite(relay_pin_, RELAY_ACTIVE_STATE);
    heater_is_off_ = false;
  }
private:
  uint8_t const uid_;
  State st_;
  bool heater_is_off_;
  float const target_;
  float const max_;
  OneWire one_wire_;
  DallasTemperature sensor_;
  uint8_t const relay_pin_;
  DesyncMan desync_man_;
};


// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

bool Panic::is_panic_ = false;
Panic::Callback Panic::callbacks_[Panic::MAX_CALLBACKS] = { 0 };
unsigned char Panic::callback_count_ = 0U;
PanicInfo Panic::panic_info_ = { 0UL, 0U, 0U, PanicReason::None };


TempController const * LEDMan::controllers_[LEDMan::MAX_CONTROLLERS] = { nullptr };
unsigned int LEDMan::num_controllers_ = 0U;

unsigned int LEDMan::currentStateIndex_ = 0U;
bool LEDMan::ledOn_ = false;
unsigned long LEDMan::lastToggleMs_ = 0UL;

void LEDMan::UpdateState()
{
  //thematically, setting the unset value to something invalid, impossible to use
  unsigned int new_state { 100 };

  if(Panic::IsPanic())
  {
    new_state = 0;
  }
  else
  {
    bool any_heating { false };

    for(unsigned int i{}; i < num_controllers_; ++i)
    {
      any_heating |= controllers_[i]->IsHeating();
    }
    new_state = any_heating ? 1U : 2U;
  }

  if(new_state != currentStateIndex_)
  {
    currentStateIndex_ = new_state;
    ledOn_ = true;
    lastToggleMs_ = millis();
  }
}




//
TempController nico(1u, 24.0f, 28.0f, 2u, 8u);
TempController trap(2u, 25.0f, 28.0f, 4u, 12u);

//



unsigned long GLastReadMs{ 0UL };

// -----------------------------------------------------------------------------
// Arduino setup / loop
// -----------------------------------------------------------------------------

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  nico.Begin();
  trap.Begin();

  Log::begin(9600);

  Log::println(F("\nNico temp controller starting..."));
  Log::println(F("Target: 26 C, hysteresis: +/-0.5 C"));


  Panic::Callback const cbs[] = 
  {
    [](){nico.Off();},
    [](){trap.Off();}
  };
  Panic::Init(cbs);

  LEDMan::Register(&nico);
  LEDMan::Register(&trap);

}

void loop()
{
  LEDMan::Update();
  unsigned long const now{ millis() };
  if (now - GLastReadMs < READ_INTERVAL_MS) return;
  GLastReadMs = now;

  if (Panic::IsPanic())
  {
    Panic::PrintPanic();
    return;
  }
  

  nico.Loop();
  trap.Loop();
}
This project is basically a temp controller for my ants.
I recently got a few exotic ants and they needed to be warmed so instead of buying a temp controller I figured to make one.


This project uses an Arduino Rev 3 SMD board for control, and this repo has the code for the board.
Note that this projects code focus was primarily heater safety (I do not want to burn house down) so no serious optimisations have been done.

Currently the code controls 2 heater circuits, each featuring a temp probe (DS18B20) and a relay (DollaTek 5V 1-Channel) for the heat mats.

The software include a lot of safety guards such as over heating control, panic system, sensor disconnect and sensor which has not disconnected
but is not measuring the heating area (in the event it gets physically detached from the enclosure but remains conncted which could of lead to
temperature run away).

Please note that since the Arduino IDE uses C++11, there are parts of the code that had to be down quite awkwardly but it was a good oppertunity
to experiment with an older standard.

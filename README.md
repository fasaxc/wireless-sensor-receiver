Wireless Sensor Node Receiver
=============================

This repo contains the source code for the Arduino reciever that decodes 
incoming data from my wireless sensor node project and outputs it overe serial.

The sensor nodes themselves use smaller and cheaper ATTiny microcontrollers.
Their code can be found <a href="https://github.com/fasaxc/wireless-sensor-node">here</a>.

Code
----

The code is in main.cpp.  I develop in Eclipse rather than the Arduino IDE and
unfortunately the code doesn't seem to compile as a sketch.

The code also relies on the definitions from sensor_node.h from the 
wireless-sensor-node project.  If you're working in Eclipse then you can add
that project as a dependency so it gets picked up.

The code has 3 parts:

*  *setup()* the standard Arduino setup routine, run once at start of day.
*  *loop()* the standard Arduino loop function, does nothing by default.
*  *ISR(TIMER1_CAPT_vect)* the interrupt service routine for the timer 1 
   capture interrupt.  Called whenever the input level changes.

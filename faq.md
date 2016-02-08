---
layout: default
title: Roboshield FAQ
---

### How do I control more motors?

Here are some options to controlling more motors with the Roboshield:

* Use the additional PWM pins on the Arduino Mega.  Digital pins 44, 45, and 46 are hardware PWM pins tied to Timer5 on the Arduino.  

* Another option is to use the PWM that drive motor 0/1 and use those pins to drive a different H-bridge that has higher current.

---

### How do I control more servos?

You can use some of the digital pins on the Arduino as the signal pins for your servos.  Power for your extra servos can be obtained by soldering a wire to the external servo power hole located near the S0 servo port.  

---

### Why does the Arduino Mega get warm?

The Roboshield uses the on board 5V regulator on the Arduino Mega.  This regulator will get warm with voltages above 10V.    

---

### Why does the 5V power cutoff when there are a lot of sensors connected?

There is a resettable fuse (500mA) connected between the Arduino Mega and the Roboshield 5V line.  If too much power is drawn through this fuse, then it will trip.  You can pull more current from the servo regulator if necessary.

---

### Can the servo voltage be increased?

Yes.  By default, the regulator on the Roboshield provides 5 volts to the servos.  Most servos are capable of running on 6V and some can run at 7.4V.  A higher voltage will provide more torque and speed.

The servo voltage regulator can be adjusted by soldering a resistor on top of the resistor shown in the picture.  More information will be posted regarding this modification.

---
layout: default
title: Roboshield User Guide
---


### Roboshield Pinout

---

### 6-pin Motor Headers

The Roboshield support 6-pin Pololu motor headers.  The motor connector should be plugged in with the Red pin towards the left side of the board (connected to pin A).  The white pin should be connected to pin EB.  

---

### External Power

Main power for the Roboshield is applied through the main power terminals.  7-15V may be applied and by default, this voltage is sent directly to the motor drivers and is regulated to 5V for the servos.

#### External Servo Power

The on-board servo power regulator can be disabled by cutting the left trace of the solder bridge labeled 5V_S_EN.

A wire plugged into the hole near the S0 label will apply power to the servos.

#### External Motor Power


---

### I2C Functionality

Digital pins 3 and 4 are shared with the SCL and SDA pins.  These pins are labeled D3-SCL and D4-SDA.
`enableEncoders()`

`disableEncoders()`

`readEncoder(uint8_t)`

`resetEncoder(uint8_t)`

---

### GY-521 




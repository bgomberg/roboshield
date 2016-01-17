---
layout: default
title: Roboshield Library Reference Page
---


### Library Reference

`uint8_t buttonPressed()`

`void setLED(bool)`

`uint8_t setPinMode(uint8_t pin, uint8_t mode)`

`readPin(uint8_t pin)`

`setPin(uint8_t pin, bool state)`

`getAnalog(uint8_t pin)`

`setMotor(uint8_t motor_num, int8_t speed)` - motor control function

`setServo(uint8_t servo_num, int8_t position)` - servo control function

`float batteryVoltage()`

---

#### LCD Functions

`lcdSetCursor(col,row)`

`lcdPrintf()`

`lcdClear()`

---

#### Encoders

`enableEncoders()`

`disableEncoders()`

`readEncoder(uint8_t)`

`resetEncoder(uint8_t)`

---

#### GY-521 

`initMPU6050()`

`readAccelX()`
`readAccelY()`
`readAccelZ()`

`readGyroX()`
`readGyroY()`
`readGyroZ()`




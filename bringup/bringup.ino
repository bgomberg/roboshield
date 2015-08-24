#include "RoboShield.h"

RoboShield roboShield;

void setup() {
}

void loop() {
  roboShield.setLED(roboShield.buttonPressed());
  roboShield.lcdClear();
  roboShield.lcdPrintf("HELLO\nTIME: %lu", millis());
  roboShield.setMotor(0,128);
  roboShield.setMotor(1,128);
  roboShield.setMotor(2,128);
  roboShield.setMotor(3,128);
  delay(50);
}

#include "RoboShield.h"

RoboShield roboShield;

void setup() {
}

void loop() {
  roboShield.setLED(roboShield.buttonPressed());
  roboShield.lcdClear();
  roboShield.lcdPrintf("HELLO\nTIME: %lu", millis());
  roboShield.setMotor(0,127);
  roboShield.setMotor(1,127);
  roboShield.setMotor(2,127);
  roboShield.setMotor(3,127);
  roboShield.debuggingMode();
  delay(50);
}

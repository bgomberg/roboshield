#include "RoboShield.h"

RoboShield roboShield;

void setup() {
}

void loop() {
  roboShield.setLED(roboShield.buttonPressed());
  roboShield.lcdClear();
  roboShield.lcdPrintf("HELLO\nTIME: %d", millis());
  delay(50);
}
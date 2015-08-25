#include "RoboShield.h"

RoboShield roboShield;

void setup() {
}

void loop() {
  roboShield.setLED(roboShield.buttonPressed());
  roboShield.lcdClear();
  roboShield.lcdPrintf("HELLO\nTIME: %lu", millis());
  roboShield.debuggingMode();
  delay(50);
}

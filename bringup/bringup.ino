#include "RoboShield.h"

RoboShield roboShield;

void setup() {
}

void loop() {
  roboShield.setLED(roboShield.buttonPressed());
  roboShield.lcdClear();
  //roboShield.lcdPrintf("HELLO\nTIME: %lu", millis());
  roboShield.setMotor(0,0);
  roboShield.lcdPrintf("%d", roboShield.readEncoder(0));
  
  //roboShield.debuggingMode();
  delay(50);
}

#include "RoboShield.h"

RoboShield roboShield;

void setup() {
}

void loop() {
  roboShield.setLED(roboShield.buttonPressed());
  roboShield.lcdClear();
  //roboShield.lcdPrintf("HELLO\nTIME: %lu", millis());
  roboShield.setMotor(0,0);
  //roboShield.lcdPrintf("%d %d", roboShield.readEncoder(0), analogRead(14));
  
  
  roboShield.debuggingMode();
  delay(50);
}

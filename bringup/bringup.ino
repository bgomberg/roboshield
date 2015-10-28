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

  while(1==0) {
  for (uint8_t i = 0; i < 8; i++) {
    roboShield.setServo(i, 0);
  }

  delay(1000);

  for (uint8_t i = 0; i < 8; i++) {
    roboShield.setServo(i, 100);
  }

  delay(1000);

  }
  
  roboShield.debuggingMode();
  delay(50);
}

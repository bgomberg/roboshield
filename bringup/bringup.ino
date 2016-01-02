#include <Wire.h>
#include "RoboShield.h"

RoboShield roboShield;

void setup() {
  if (roboShield.buttonPressed())
    roboShield.debuggingMode();
}

void loop() {
  roboShield.setLED(roboShield.buttonPressed());
  roboShield.lcdClear();
  //roboShield.lcdPrintf("HELLO\nTIME: %lu", millis());
  roboShield.setMotor(0,0);
  //roboShield.lcdPrintf("%d %d", roboShield.readEncoder(0), analogRead(14));

  /*roboShield.initMPU6050();

  while(1==1) {
    roboShield.lcdClear();
    roboShield.readMPU();
    roboShield.lcdPrintf ("%d", roboShield.readAccelX());
    roboShield.lcdSetCursor(0,1);
    roboShield.lcdPrintf ("%d", roboShield.readGyroZ());
    delay(100);
  }*/

  uint8_t i=0;

  for (i=0;i<7;i++) {
    roboShield.setServo(i,100);
  }
  
  while(1==1) {
    //roboShield.setMotor(0,50);
    //roboShield.setMotor(1,-50);

    if (i==0) {
      roboShield.setMotor(3,-50);
      roboShield.setMotor(2,-50);
      roboShield.setMotor(1,50);
      roboShield.setMotor(0,50);
      i++;
    } else {
      roboShield.setMotor(3,50);
      roboShield.setMotor(2,50);
      roboShield.setMotor(1,-50);
      roboShield.setMotor(0,-50);
      i=0;
    }

    //roboShield.setMotor(3,50);
    delay(2000);
  }

  roboShield.debuggingMode();
  delay(50);
}

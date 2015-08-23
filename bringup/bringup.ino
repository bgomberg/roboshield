#include "RoboShield.h"

RoboShield roboShield;

void setup() {
}

void loop() {
  roboShield.setLED(roboShield.buttonPressed());
  roboShield.println("HELLO");
  roboShield.println(millis() / 1000);
  delay(100);
}
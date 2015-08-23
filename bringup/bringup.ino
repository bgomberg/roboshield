#include "RoboShield.h"

RoboShield roboShield;

void setup() {
  roboShield.lcdPrintChar('H');
  roboShield.lcdPrintChar('E');
  roboShield.lcdPrintChar('L');
  roboShield.lcdPrintChar('L');
  roboShield.lcdPrintChar('O');
}

void loop() {
  roboShield.setLED(roboShield.buttonPressed());
  const uint8_t second = (millis() / 1000) % 10;
  roboShield.lcdSetCursor(0, 1);
  roboShield.lcdPrintChar('0' + second);
  delay(100);
}
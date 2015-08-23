#ifndef __ROBO_SHIELD_H__
#define __ROBO_SHIELD_H__

#include <inttypes.h>


class RoboShield {
public:
  RoboShield() { init(); }
  
  bool buttonPressed(void);
  void setLED(bool on);
  float batteryVoltage(void);
  
  void setPinMode(uint8_t pin, uint8_t mode);
  bool readPin(uint8_t pin);
  void setPin(uint8_t pin, bool set_high);
  int getAnalog(uint8_t pin);

  void setServos(uint8_t data);
  void lcdSetCursor(uint8_t col, uint8_t row);
  void lcdPrintChar(char c);
  
private:
  void init(void);
  void setDataBus(uint8_t data);
  void lcdControl(uint8_t data);
};

#endif
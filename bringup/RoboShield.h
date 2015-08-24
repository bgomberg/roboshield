#ifndef __ROBO_SHIELD_H__
#define __ROBO_SHIELD_H__

#include <inttypes.h>
#include "Print.h"


class RoboShield : public Print {
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
  
  // LCD functions
  void lcdSetCursor(uint8_t col, uint8_t row);
  void lcdClear(void);
  void lcdPrintf(const char *format, ...);
  virtual size_t write(uint8_t);
  using Print::write;

private:
  void init(void);
  void lcdInit(void);
  void lcdWrite(uint8_t data, bool is_control);
  void motorInit(void);
  
  uint8_t _lcd_line;
};

#endif

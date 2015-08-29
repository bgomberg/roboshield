#ifndef __ROBO_SHIELD_H__
#define __ROBO_SHIELD_H__

#include <inttypes.h>
#include "Print.h"


class RoboShield : public Print {
public:
  // constructor
  RoboShield() { init(); }
  
  // general methods
  bool buttonPressed(void);
  void setLED(bool on);
  
  // digital pin methods
  void setPinMode(uint8_t pin, uint8_t mode);
  bool readPin(uint8_t pin);
  void setPin(uint8_t pin, bool set_high);
  
  // analog input methods
  int getAnalog(uint8_t pin);
  float batteryVoltage(void);

  // servo methods
  void setServo(uint8_t num, int8_t pos);

  // motor methods
  void setMotor(uint8_t num, int8_t speed);
  uint32_t readEncoder(uint8_t num);
  void resetEncoder(uint8_t num);

  void debuggingMode(void);
  
  // LCD methods
  void lcdSetCursor(uint8_t col, uint8_t row);
  void lcdClear(void);
  void lcdPrintf(const char *format, ...);
  void printFloat( float val, uint8_t precision);
  virtual size_t write(uint8_t);
  using Print::write;

private:
  // private methods
  void init(void);
  void lcdInit(void);
  void lcdWrite(uint8_t data, bool is_control);
  void motorInit(void);
  
  // class variables
  uint8_t _lcd_line;
  bool _servo_init;
  bool _motor_init;

};

#endif

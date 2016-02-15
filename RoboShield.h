#pragma once

#include <inttypes.h>
#include "Print.h"


typedef struct {
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t temp;
} MPU6050Reading;


class RoboShield : public Print {
public:
  // constructor
  RoboShield(uint8_t options=0) { init(options); }

  // general methods
  bool buttonPressed(void);
  void setLED(bool on);

  // digital pin methods
  void setPinMode(uint8_t pin, uint8_t mode);
  bool readPin(uint8_t pin);
  void writePin(uint8_t pin, bool set_high);

  // analog input methods
  uint16_t getAnalog(uint8_t pin);
  uint16_t batteryVoltage(void);

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

  void initMPU6050(void);
  MPU6050Reading readMPU6050(void);

private:
  // private methods
  void init(uint8_t options);
  void lcdInit(void);
  void lcdWrite(uint8_t data, bool is_control);
  void lcdWrite4Bits(uint8_t value, bool is_control);
  void motorInit(void);

};

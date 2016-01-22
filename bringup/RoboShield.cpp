#include "RoboShield.h"
#include "RoboShield_Defines.h"
#include <Arduino.h>
#include <Wire.h>

static RoboShield *s_active_object = NULL;

// Public methods
////////////////////////////////////////////////////////////////////////////////

bool RoboShield::buttonPressed(void) {
  return !digitalRead(BUTTON_PIN);
}

void RoboShield::setLED(bool on) {
  digitalWrite(LED_PIN, on);
}

void RoboShield::setPinMode(uint8_t pin, uint8_t mode) {
  if (pin < NUM_DIGITAL_PINS) {
    // this is a pin on the shield
    pinMode(DIGITAL_PIN_MAPPING[pin], mode);
  } else if (pin > MAX_USED_DIGITAL_PIN) {
    // this is a pin on the Arduino itself
    pinMode(pin, mode);
  } else {
    // this is an pin which is used internally by the shield
  }
}

bool RoboShield::readPin(uint8_t pin) {
  if (pin < NUM_DIGITAL_PINS) {
    // this is a pin on the shield
    return digitalRead(DIGITAL_PIN_MAPPING[pin]);
  } else if (pin > MAX_USED_DIGITAL_PIN) {
    // this is a pin on the Arduino itself
    return digitalRead(pin);
  } else {
    // this is an pin which is used internally by the shield
    return false;
  }
}

void RoboShield::setPin(uint8_t pin, bool set_high) {
  if (pin < NUM_DIGITAL_PINS) {
    // this is a pin on the shield
    digitalWrite(DIGITAL_PIN_MAPPING[pin], set_high);
  } else if (pin > MAX_USED_DIGITAL_PIN) {
    // this is a pin on the Arduino itself
    digitalWrite(pin, set_high);
  } else {
    // this is an pin which is used internally by the shield
  }
}

int RoboShield::getAnalog(uint8_t pin) {
  return analogRead(pin);
}

float RoboShield::batteryVoltage(void) {
  return (float)(analogRead(BATTERY_VOLTAGE_PIN) * 5.0 * 9.5 / 2.0 / 1012.0);
}

static uint8_t motor_value = 0;
static uint8_t old_motor_value = 0;
void RoboShield::setMotor(uint8_t num, int8_t speed) {
  if (!_motor_init) {
    motorInit();
    _motor_init = true;
  }

  motor_value &= ~(0x03 << (2 * num)); // clear the direction bits for the given motor

  // speed should be between -100 and +100
  if (speed > 0) {
    motor_value |= (0x01 << (2 * num));
  } else if (speed < 0) {
    speed = -speed;
    motor_value |= (0x02 << (2 * num));
  }

  switch (num) {
    case 0:
      OCR2B = 255 * speed / 100;
      break;
    case 1:
      OCR2A = 255 * speed / 100;
      break;
    case 2:
      OCR4A = 255 * speed / 100;
      break;
    case 3:
      OCR4B = 255 * speed / 100;
      break;
  }

  if (motor_value != old_motor_value) {
    cli();
    SHIFT_OUT_BYTE(motor_value);
    sei();
  }

  old_motor_value = motor_value;
}

static volatile uint16_t servo_value[NUM_SERVOS];
static volatile uint8_t servo_enabled[NUM_SERVOS];
void RoboShield::setServo(uint8_t num, int8_t pos) {
  if (num >= NUM_SERVOS) {
    return;
  }
  if (!_servo_init) {
    INIT_TIMER();
    _servo_init = true;
  }
  servo_value[num] = (uint16_t)map((int16_t)pos, -100, 100, 0, 2000);
  servo_enabled[num] = (1 << num);
}

volatile uint32_t encoder0 = 0;
volatile uint32_t encoder1 = 0;
uint32_t RoboShield::readEncoder(uint8_t num) {
  if (num)
    return encoder1;
  else
    return encoder0;
}

void RoboShield::resetEncoder(uint8_t num) {
  if (num)
    encoder1 = 0;
  else
    encoder0 = 0;
}

const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
void RoboShield::initMPU6050(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void RoboShield::readMPU(void) { 
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

int16_t RoboShield::readAccelX(void) {
  return AcX;
}

int16_t RoboShield::readAccelY(void) {
  return AcY;
}

int16_t RoboShield::readAccelZ(void) {
  return AcZ;
}

int16_t RoboShield::readGyroX(void) {
  return GyX;
}

int16_t RoboShield::readGyroY(void) {
  return GyY;
}

int16_t RoboShield::readGyroZ(void) {
  return GyZ;
}

// Private methods
////////////////////////////////////////////////////////////////////////////////

void RoboShield::init(void) {
  if (s_active_object) {
    // we've already run this
    return;
  }
  s_active_object = this;

  // initialize class variables
  _lcd_line = 0;
  _servo_init = false;
  _motor_init = false;

  // configure any pins we don't want to be INPUT (the default)
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PWM0_PIN, OUTPUT);
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  pinMode(PWM3_PIN, OUTPUT);
  
  pinMode(CLK_S_PIN, OUTPUT);
  digitalWrite(CLK_S_PIN, LOW);
  pinMode(DATA_S_PIN, OUTPUT);

  pinMode(CLK_L_PIN, OUTPUT);
  digitalWrite(CLK_L_PIN, LOW);
  pinMode(DATA_L_PIN, OUTPUT);
  digitalWrite(DATA_L_PIN,LOW);

  pinMode(CLK_M_PIN, OUTPUT);
  digitalWrite(CLK_M_PIN, LOW);
  pinMode(DATA_M_PIN, OUTPUT);
  digitalWrite(DATA_M_PIN, LOW);
  
  pinMode(LCD_EN_PIN, OUTPUT);
  digitalWrite(LCD_EN_PIN, LOW);

  lcdInit();
}

void RoboShield::lcdInit(void) {
  // // NOTE: Using delay() here doesn't work for some reason, so we use delayMicroseconds instead
  delayMicroseconds(15000);
  // we start in 8bit mode, try to set 4 bit mode
  lcdWrite4Bits(0x03, true);
  delayMicroseconds(4500); // wait min 4.1ms
  // second try
  lcdWrite4Bits(0x03, true);
  delayMicroseconds(4500); // wait min 4.1ms
  // third go!
  lcdWrite4Bits(0x03, true);
  delayMicroseconds(150);
  // finally, set to 4-bit interface
  lcdWrite4Bits(0x02, true);
  // finally, set # lines, font size, etc.
  lcdWrite(0x28, true);
  // turn the display on with no cursor or blinking default
  lcdWrite(0x0C, true);
  // clear it off
  lcdClear();
  // set the entry mode
  lcdWrite(0x06, true);
}

void RoboShield::lcdWrite4Bits(uint8_t data, bool is_control) {
  const uint8_t oV = PORTB & ~(0x60);
  const uint8_t dV = oV | 0x40;
  const uint8_t cV = oV | 0x20;
    
  if (data & _BV(3)) PORTB = dV;
    PORTB = cV;
    PORTB = oV;
  if (data & _BV(2)) PORTB = dV;
    PORTB = cV;
    PORTB = oV;
  if (data & _BV(1)) PORTB = dV;
    PORTB = cV;
    PORTB = oV;
  if (data & _BV(0)) PORTB = dV;
    PORTB = cV;
    PORTB = oV;

  // set the LCD_RS pin
  if (is_control) {
  } else {
    PORTB = dV;
  }
  PORTB = cV;
  PORTB = oV;
  
  // toggle the LCD_E pin
  PORTE |= _BV(PE3);
  PORTE &= ~_BV(PE3);
  delayMicroseconds(100);
}

void RoboShield::lcdWrite(uint8_t data, bool is_control) {
  if (data == '\n') {
    _lcd_line = (_lcd_line + 1) % 2;
    lcdSetCursor(0, _lcd_line);
    return;
  } else if (data == '\r') {
    return;
  }
  lcdWrite4Bits(data>>4, is_control);
  lcdWrite4Bits(data, is_control);
}


void RoboShield::lcdSetCursor(uint8_t col, uint8_t row) {
  if (col >= 16 || row >= 2) {
    return;
  }
  const uint8_t addr = 0x80 + row * 0x40 + col;
  lcdWrite(addr, true);
}

void RoboShield::lcdClear(void) {
  // clear the display
  lcdWrite(0x01, true);
  delayMicroseconds(3300);
  _lcd_line = 0;
}

void RoboShield::lcdPrintf(const char *format, ...) {
  char buf[34];
  va_list ap;
  va_start(ap, format);
  vsnprintf(buf, sizeof(buf), format, ap);
  for (char *p = &buf[0]; *p; p++) { // emulate cooked mode for newlines
    if (*p == '\n')
      write('\r');
    write(*p);
  }
  va_end(ap);
}

void RoboShield::printFloat( float val, uint8_t precision) {
  // prints val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)

  lcdPrintf("%d", int(val));  //prints the int part
  if ( precision > 0) {
    lcdPrintf("."); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    uint8_t padding = precision - 1;
    while (precision--)
      mult *= 10;

    if (val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val) - val ) * mult;
    unsigned long frac1 = frac;
    while ( frac1 /= 10 )
      padding--;
    while (  padding--)
      lcdPrintf("0");
    lcdPrintf("%d", frac);
  }
}

size_t RoboShield::write(uint8_t character) {
  lcdWrite(character, false);
  return 1;
}

void RoboShield::motorInit(void) {
  // motor 0
  TCCR2A |= _BV(WGM21) | _BV(WGM20) | _BV(COM2B1); //fast PWM, non-inverting
  TCCR2B |= _BV(CS22) | _BV(CS21) | _BV(CS20); // clk/1024 prescalar

  // motor 1
  TCCR2A |= _BV(COM2A1); //fast PWM, non-inverting

  // motor 2
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4A |= _BV(COM4B1);

  // motor 3
  TCCR4A |= _BV(WGM40) | _BV(COM4A1); // fast PWM, 8-bit, non-inverting
  TCCR4B |= _BV(CS42) | _BV(CS40) | _BV(WGM42); // clk/1024 prescalar

  // enable encoder interrupts
  EICRA = 0;
  EICRA |= _BV(ISC21) | _BV(ISC31); // set INT2 and INT3 to trigger on falling edges
  EIMSK = 0;
  EIMSK |= _BV(INT2) | _BV(INT3); // enable INT2 and INT3;

  // enable pullups on INT4 and INT5
  PORTD |= _BV(PD2) | _BV(PD3);
}


// ISRs
////////////////////////////////////////////////////////////////////////////////

ISR(TIMER1_COMPA_vect, ISR_BLOCK) {
  static volatile uint8_t servo_num = 0;
  uint8_t new_servo_data;
  if (servo_num == NUM_SERVOS) {
    // set all servos low
    OCR1A = 40000;
    new_servo_data = 0;
    servo_num = 0;
  } else {
    if (servo_num == 0) {
      TCNT1 = 0;
      OCR1A = 0;
      PORTE |= _BV(5);
    } else {
      PORTE &= ~_BV(5);
    }
    // set the next servo high (and all other servos low)
    OCR1A += servo_value[servo_num] + 2000;
    new_servo_data = servo_enabled[servo_num];
    servo_num++;
  }

  PORTE |= _BV(4);  //CLK_S_PIN, HIGH
  PORTE &= ~_BV(4); //CLK_S_PIN, LOW
}

// encoder 0
ISR(INT3_vect, ISR_NOBLOCK) {
  encoder0++;
}

//encoder 1
ISR(INT2_vect, ISR_NOBLOCK) {
  encoder1++;
}

// Debugging Mode
////////////////////////////////////////////////////////////////////////////////

void RoboShield::debuggingMode(void) {
  uint8_t selector = 0;
  while (1) {
    lcdClear();
    lcdSetCursor(0, 1);
    lcdPrintf("hold to select");
    lcdSetCursor(0, 0);
    switch (selector) {
      case 0:
        lcdPrintf("Digital 1/6");
        break;
      case 1:
        lcdPrintf("Analog 2/6");
        break;
      case 2:
        lcdPrintf("Servo 3/6");
        break;
      case 3:
        lcdPrintf("Motor 4/6");
        break;
      case 4:
        lcdPrintf("Battery 5/6");
        break;
      case 5:
        lcdPrintf("IMU test 6/6");
        break;
    }

    while(!buttonPressed()) {}
    
    if (buttonPressed()) {
      uint32_t start_time = millis();
      uint8_t hold = 0;

      // check if the button is held down
      while (buttonPressed()) {
        if ((millis() - start_time) > 800) {
          hold = 1;
          break;
        }
      }

      if (hold == 1) {
        lcdClear();
        resetEncoder(0);
        resetEncoder(1);

        // loop while the button is held down
        while (buttonPressed()) {}
        delayMicroseconds(5000);

        uint8_t mcounter = 0;
        uint8_t analog_display = 0;
        while (!buttonPressed()) {  // loop until button is pressed
          lcdClear();
          switch (selector) {
            case 0:
              lcdPrintf("%d", readPin(0));
              lcdPrintf("%d", readPin(1));
              lcdPrintf("%d", readPin(2));
              lcdPrintf("%d", readPin(3));
              lcdPrintf("%d", readPin(4));
              lcdPrintf("%d", readPin(5));
              lcdPrintf("%d", readPin(6));
              break;
            case 1:
              //while(buttonPressed()) {} // loop until button is released
              //delayMicroseconds(5000);
              hold = 0;

              while (hold == 0) {
                lcdClear();

                if (analog_display == 0) { // display analog 0 through 7
                  for (uint8_t i = 0; i < 4; i++) {
                    lcdPrintf("%4d", getAnalog(i));
                  }

                  lcdSetCursor(0, 1);

                  for (uint8_t i = 4; i < 8; i++) {
                    lcdPrintf("%4d", getAnalog(i));
                  }
                } else { // display analog 8 through 14
                  for (uint8_t i = 8; i < 11; i++) {
                    lcdPrintf("%4d", getAnalog(i));
                  }

                  lcdPrintf("%4d", getAnalog(12));

                  lcdSetCursor(0, 1);

                  for (uint8_t i = 13; i < 15; i++) {
                    lcdPrintf("%4d", getAnalog(i));
                  }
                }

                if (buttonPressed()) {           // check if the button is held down
                  delayMicroseconds(5000);
                  start_time = millis();
                  while (buttonPressed()) {
                    if ((millis() - start_time) > 800) {
                      hold = 1;
                      break;
                    }
                  }

                  if (hold == 0)
                    analog_display ^= 1;
                }

                delay(100);
              }

              lcdClear();

              break;
            case 2:
              lcdPrintf("Servo test");
              while(1) {
                for (uint8_t i = 0; i < 8; i++) {
                  setServo(i, 100);
                  delay(300);
                }
                for (uint8_t i = 0; i < 8; i++) {
                  setServo(i, 0);
                  delay(300);
                }
              }

              break;
            case 3:
              lcdPrintf("Motor ");
              setMotor(0, -2 * mcounter);
              setMotor(1, -2 * mcounter);
              setMotor(2, 2 * mcounter);
              setMotor(3, 2 * mcounter);

              mcounter++;
              if (mcounter > 50) {
                mcounter = 0;
              }

              lcdPrintf("E0:%lu", readEncoder(0));
              lcdSetCursor(0, 1);
              lcdPrintf("test  E1:%lu", readEncoder(1));
              break;
            case 4:
              printFloat(batteryVoltage(), 2);
              lcdPrintf("V");
              break;
            case 5:
              initMPU6050();
              
              while(1==1) {
                lcdClear();
                readMPU();
                lcdPrintf ("%d", readAccelX());
                lcdSetCursor(0,1);
                lcdPrintf ("%d", readGyroZ());
                delay(100);
              }
              break;
          }

          delay(100);
        }
        delayMicroseconds(5000);

        // turn off all motors
        setMotor(0, 0);
        setMotor(1, 0);
        setMotor(2, 0);
        setMotor(3, 0);

        // loop until button is released
        while (buttonPressed()) {}
        delayMicroseconds(5000);
      } else {  // no hold detected, so go to next menu option
        selector++;
        selector %= 6;

        // loop while button is pressed
        while (buttonPressed()) {}
        delayMicroseconds(5000);
      }
    }

    delay(50);
  }
}


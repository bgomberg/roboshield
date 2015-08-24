#include "RoboShield.h"
#include "RoboShield_Defines.h"
#include <Arduino.h>

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
  return (float)(analogRead(BATTERY_VOLTAGE_PIN) * MAX_BATTERY_VOLTAGE / 1024.0);
}

static uint8_t motor_value = 0x55;
void RoboShield::setMotor(uint8_t num, int8_t speed) {
  SHIFT_OUT_BYTE(motor_value);
  digitalWrite(MOTOR_LATCH_EN_PIN, HIGH);
  digitalWrite(MOTOR_LATCH_EN_PIN, LOW);
  
  switch (num) {
    case 0:
      OCR2B = speed;
      break;
    case 1:
      OCR2A = speed;
      break;
    case 2:
      OCR3A = speed;
      break;
    case 3:
      OCR4A = speed;
      break;
  }
}

static uint16_t servo_value = 0;
void RoboShield::setServo(uint8_t num, uint8_t pos) {
  if (!_servo_init) {
    INIT_TIMER();
    _servo_init = true;
  }
  servo_value = (uint16_t)pos * 10;
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
  for(char *p = &buf[0]; *p; p++) { // emulate cooked mode for newlines
    if(*p == '\n')
      write('\r');
    write(*p);
  }
  va_end(ap);
}

size_t RoboShield::write(uint8_t character) {
  lcdWrite(character, false);
  return 1;
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
  pinMode(SHIFT_REG_CLK_PIN, OUTPUT);
  digitalWrite(SHIFT_REG_CLK_PIN, LOW);
  pinMode(SHIFT_REG_DATA_PIN, OUTPUT);
  pinMode(SERVO_LATCH_EN_PIN, OUTPUT);
  pinMode(MOTOR_LATCH_EN_PIN, OUTPUT);
  pinMode(LCD_RS_PIN, OUTPUT);
  digitalWrite(LCD_RS_PIN, LOW);
  pinMode(LCD_EN_PIN, OUTPUT);
  digitalWrite(LCD_EN_PIN, LOW);
  
  lcdInit();
  motorInit();
}

void RoboShield::lcdInit(void) {
  // NOTE: Using delay() here doesn't work for some reason, so we use delayMicroseconds instead
  delayMicroseconds(15000);
  // send function set command sequence
  lcdWrite(0x38, true);
  delayMicroseconds(4500);
  // second try
  lcdWrite(0x38, true);
  delayMicroseconds(150);
  // set # lines, font size, etc
  lcdWrite(0x38, true);
  // turn display on with no cursor or blinking default
  lcdWrite(0x0C, true);
  // clear the display
  lcdClear();
  // set entry mode
  lcdWrite(0x06, true);
}

void RoboShield::lcdWrite(uint8_t data, bool is_control) {
  if (data == '\n') {
    _lcd_line = (_lcd_line + 1) % 2;
    lcdSetCursor(0, _lcd_line);
    return;
  } else if (data == '\r') {
    return;
  }
  cli();
  digitalWrite(LCD_RS_PIN, is_control ? LOW : HIGH);
  SHIFT_OUT_BYTE(data);
  digitalWrite(LCD_EN_PIN, HIGH);
  digitalWrite(LCD_EN_PIN, LOW);
  sei();
  delayMicroseconds(100);
}

void RoboShield::motorInit(void) {
  // motor 0
  TCCR2A |= _BV(WGM21) | _BV(WGM20) | _BV(COM2B1); //fast PWM, non-inverting
  TCCR2B |= _BV(CS22) | _BV(CS21) | _BV(CS20); // clk/1024 prescalar

  // motor 1
  TCCR2A |= _BV(COM2A1); //fast PWM, non-inverting
  
  // motor 2
  TCCR3A |= _BV(WGM30) | _BV(COM3A1); // fast PWM, 8-bit, non-inverting
  TCCR3B |= _BV(CS32) | _BV(CS30) | _BV(WGM32); // clk/1024 prescalar

  // motor 3
  TCCR4A |= _BV(WGM40) | _BV(COM4A1); // fast PWM, 8-bit, non-inverting
  TCCR4B |= _BV(CS42) | _BV(CS40) | _BV(WGM42); // clk/1024 prescalar

  _motor_init = true;
}


// ISRs
////////////////////////////////////////////////////////////////////////////////

ISR(TIMER_ISR) {
  cli();
  static volatile uint8_t stage = 0;
  if (stage == 0) {
    SHIFT_OUT_BYTE(0x00);
    digitalWrite(SERVO_LATCH_EN_PIN, HIGH);
    digitalWrite(SERVO_LATCH_EN_PIN, LOW);
    OCR1A = 40000;
  } else {
    SHIFT_OUT_BYTE(0xFF);
    digitalWrite(SERVO_LATCH_EN_PIN, HIGH);
    digitalWrite(SERVO_LATCH_EN_PIN, LOW);
    OCR1A = servo_value + 1000;
    TCNT1 = 0;
  }
  stage++;
  if (stage > 1) {
    stage = 0;
  }
  sei();
}

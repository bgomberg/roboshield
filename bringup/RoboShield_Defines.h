#ifndef __ROBO_SHIELD_DEFINES_H__
#define __ROBO_SHIELD_DEFINES_H__

#include <inttypes.h>


/* Board-specific defines */
#if defined(__AVR_ATmega2560__) // Arduino Mega
#define MAX_USED_DIGITAL_PIN      21
#define SHIFT_OUT_BYTE(data) \
  do { \
    cli(); \
    const uint8_t oV = PORTB & ~(0x60); \
    const uint8_t dV = oV | 0x40; \
    const uint8_t cV = oV | 0x20; \
    if (data & _BV(7)) PORTB = dV; \
    PORTB = cV; \
    PORTB = oV; \
    if (data & _BV(6)) PORTB = dV; \
    PORTB = cV; \
    PORTB = oV; \
    if (data & _BV(5)) PORTB = dV; \
    PORTB = cV; \
    PORTB = oV; \
    if (data & _BV(4)) PORTB = dV; \
    PORTB = cV; \
    PORTB = oV; \
    if (data & _BV(3)) PORTB = dV; \
    PORTB = cV; \
    PORTB = oV; \
    if (data & _BV(2)) PORTB = dV; \
    PORTB = cV; \
    PORTB = oV; \
    if (data & _BV(1)) PORTB = dV; \
    PORTB = cV; \
    PORTB = oV; \
    if (data & _BV(0)) PORTB = dV; \
    PORTB = cV; \
    PORTB = oV; \
    sei(); \
  } while (0)

#else
#erorr "This board is not yet supported!"
#endif


/* General */
#define MAX_BATTERY_VOLTAGE       15.0


/* Digital pins */
#define BUTTON_PIN                0
#define LED_PIN                   1
#define DIGITAL_0_PIN             2
#define DIGITAL_1_PIN             3
#define LCD_RS_PIN                4
#define PWM2_PIN                  5
#define PWM3_PIN                  6
#define LCD_EN_PIN                7
#define MOTOR_LATCH_EN_PIN        8
#define PWM0_PIN                  9
#define PWM1_PIN                  10
#define SHIFT_REG_CLK_PIN         11
#define SHIFT_REG_DATA_PIN        12
#define SERVO_LATCH_EN_PIN        13
#define DIGITAL_2_PIN             14
#define DIGITAL_3_PIN             15
#define DIGITAL_4_PIN             16
#define DIGITAL_5_PIN             17
#define DIGITAL_6_PIN             18
#define DIGITAL_7_PIN             19
#define DIGITAL_8_PIN             20
#define DIGITAL_9_PIN             21
#define ENCODER_0_PIN             DIGITAL_0_PIN
#define ENCODER_1_PIN             DIGITAL_1_PIN
static const uint8_t DIGITAL_PIN_MAPPING[] = {
  DIGITAL_0_PIN,
  DIGITAL_1_PIN,
  DIGITAL_2_PIN,
  DIGITAL_3_PIN,
  DIGITAL_4_PIN,
  DIGITAL_6_PIN,
  DIGITAL_7_PIN,
  DIGITAL_8_PIN,
  DIGITAL_9_PIN
};
#define NUM_DIGITAL_PINS (sizeof(DIGITAL_PIN_MAPPING)/sizeof(DIGITAL_PIN_MAPPING[0]))


/* Analog pins */
#define BATTERY_VOLTAGE_PIN       14


#endif // __ROBO_SHIELD_DEFINES_H__

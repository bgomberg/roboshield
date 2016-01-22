#ifndef __ROBO_SHIELD_DEFINES_H__
#define __ROBO_SHIELD_DEFINES_H__

#include <inttypes.h>


#ifndef __AVR_ATmega2560__
#error "This board is not yet supported for the RoboShield!"
#endif


/* General */
#define NUM_SERVOS                8
#define MAX_BATTERY_VOLTAGE       15.0
#define MAX_USED_DIGITAL_PIN      21
#define TIMER_ISR                 TIMER1_COMPA_vect


/* Macros */
#define INIT_TIMER() \
  do { \
    cli(); \
    TCCR1A = 0; \
    TCCR1B = 0x02; \
    TCNT1 = 0; \
    OCR1A = 40000; \
    TIMSK1 = 0x02; \
    sei(); \
  } while (0)
#define SHIFT_OUT_BYTE(data) \
  do { \
    const uint8_t oV = PORTH & ~(0x03); \
    const uint8_t dV = oV | 0x01; \
    const uint8_t cV = oV | 0x02; \
    if (data & _BV(7)) PORTH = dV; \
    PORTH = cV; \
    delayMicroseconds(1);\
    PORTH = oV; \
    if (data & _BV(6)) PORTH = dV; \
    PORTH = cV; \
    delayMicroseconds(1);\
    PORTH = oV; \
    if (data & _BV(5)) PORTH = dV; \
    PORTH = cV; \
    delayMicroseconds(1);\
    PORTH = oV; \
    if (data & _BV(4)) PORTH = dV; \
    PORTH = cV; \
    delayMicroseconds(1);\
    PORTH = oV; \
    if (data & _BV(3)) PORTH = dV; \
    PORTH = cV; \
    delayMicroseconds(1);\
    PORTH = oV; \
    if (data & _BV(2)) PORTH = dV; \
    PORTH = cV; \
    delayMicroseconds(1);\
    PORTH = oV; \
    if (data & _BV(1)) PORTH = dV; \
    PORTH = cV; \
    delayMicroseconds(1);\
    PORTH = oV; \
    if (data & _BV(0)) PORTH = dV; \
    PORTH = cV; \
    delayMicroseconds(1);\
    PORTH = oV; \
  } while (0)


/* Digital pins */
#define CLK_S_PIN                 2  //PE4
#define DATA_S_PIN                3  //PE5
#define BUTTON_PIN                4  //PG5
#define LCD_EN_PIN                5  //PE3
#define PWM2_PIN                  6  //PH3 - OC4A
#define PWM3_PIN                  7  //PH4 - OC4B
#define DIGITAL_0_PIN             8  //PH5
#define PWM0_PIN                  9  //PH6 - OC2B
#define PWM1_PIN                  10  //PB4 - OC2A
#define CLK_L_PIN                 11  //PB5
#define DATA_L_PIN                12  //PB6
#define LED_PIN                   13  //PB7
#define DIGITAL_1_PIN             14  //PJ1
#define DIGITAL_2_PIN             15  //PJ0
#define CLK_M_PIN                 16  //PH1
#define DATA_M_PIN                17  //PH0
#define DIGITAL_5_PIN             18  //PD3 - INT3
#define DIGITAL_6_PIN             19  //PD2 - INT2
#define DIGITAL_3_PIN             20  //PD1
#define DIGITAL_4_PIN             21  //PD0
#define ENCODER_0_PIN             DIGITAL_5_PIN
#define ENCODER_1_PIN             DIGITAL_6_PIN
static const uint8_t DIGITAL_PIN_MAPPING[] = {
  DIGITAL_0_PIN,
  DIGITAL_1_PIN,
  DIGITAL_2_PIN,
  DIGITAL_3_PIN,
  DIGITAL_4_PIN,  
  DIGITAL_5_PIN,
  DIGITAL_6_PIN
};
#define NUM_DIGITAL_PINS (sizeof(DIGITAL_PIN_MAPPING)/sizeof(DIGITAL_PIN_MAPPING[0]))


/* Analog pins */
#define BATTERY_VOLTAGE_PIN       11


#endif // __ROBO_SHIELD_DEFINES_H__

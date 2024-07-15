#ifndef PIN_H
#define PIN_H

#include <Arduino.h>

// Pins Motors - Drivers

#define stepPinM1 12
#define dirPinM1  11

#define stepPinM2 10
#define dirPinM2  9

#define stepPinM3 8
#define dirPinM3  7


// Use with Stepper-Motor driver, such as TMC2209

#define _PWM_LOGLEVEL_        4

#include "megaAVR_PWM.h"

//#define USING_TIMERB        true

#if USING_TIMERB
  // Pins tested OK in Nano Every ATmega4809
  #define STEP_PIN      3            // TimerB1, for higher frequencies, up to 100KHz
  //#define STEP_PIN      6            // TimerB0, for higher frequencies, up to 100KHz
#elif USING_ARDUINO_MEGA_AVR_CORE
  // Pins tested OK in Nano Every ATmega4809 using Arduino megaAVR core
  // TimerA0 somehow can't be used with MegaCoreX
  #define STEP_PIN      5            // TimerA0, only accurate @ low frequencies (< 1KHz) because of low 250KHz clock
  //#define STEP_PIN      9            // TimerA0, only accurate @ low frequencies (< 1KHz) because of low 250KHz clock
  //#define STEP_PIN     10            // TimerA0, only accurate @ low frequencies (< 1KHz) because of low 250KHz clock
#else
  #error TimerA0 to be used with Arduino megaAVR Core
#endif

#define DIR_PIN       4



#endif

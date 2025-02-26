#ifndef PIN_H
#define PIN_H

#include <pins_arduino.h>
#include <HardwareSerial.h>

//******************** Pins Undefined
#define PIN_10  10
#define PIN_11  11

//******************** Pins UART - Serial
#undef SOC_RX0
#define SOC_RX0 44
#undef SOC_TX0
#define SOC_TX0 43

#undef RX1
#define RX1 18
#undef TX1
#define TX1 17

//******************** Pins Motors - Drivers
#define stepPinM1 7
#define dirPinM1  6

#define stepPinM2 5
#define dirPinM2  4

#define stepPinM3 2
#define dirPinM3  1

#define EN_MCU    3

//******************** Pins TwoWire I²C - Otos
#undef SDA
#define SDA 8
#undef SCL
#define SCL 9

//******************** Pins LED - RGB
#define PIN_RGB_LED 38
#define WS2812_LED  12

#ifdef RGB_BUILTIN
#undef RGB_BUILTIN
#endif
#define RGB_BUILTIN PIN_RGB_LED

#ifdef RGB_BRIGHTNESS
#undef RGB_BRIGHTNESS
#endif
#define RGB_BRIGHTNESS 64

//******************** Pins IN
#define MODE_PIN    14
#define TEAM_PIN    13
#define BAU_PIN     15
#define START_PIN   16


#endif

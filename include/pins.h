#ifndef PIN_H
#define PIN_H

#include <pins_arduino.h>
#include <HardwareSerial.h>

//******************** Pins Undefined
constexpr size_t PIN_10 = 10;
constexpr size_t PIN_11 = 11;

//******************** Pins UART - Serial

// Serial over USB
#define SERIAL_DEBUG Serial

// Serial 0
//  We don't need this redefinition of pin, it's just for information
#undef SOC_RX0
#define SOC_RX0 44
#undef SOC_TX0
#define SOC_TX0 43

#define SERIAL_LIDAR Serial0

// Serial 1
#ifdef RX1
#undef RX1
#endif
#define RX1 18

#ifdef TX1
#undef TX1
#endif
#define TX1 17

#define SERIAL_XX Serial1

// Serial 2
// #ifdef RX2
// #undef RX2
// #endif
// #define RX2 xx

// #ifdef TX2
// #undef TX2
// #endif
// #define TX2 xx

// #define SERIAL_XX Serial2

//******************** Pins Motors - Drivers
constexpr size_t PIN_STEP_M1 = 7;
constexpr size_t PIN_DIR_M1 = 6;

constexpr size_t PIN_STEP_M2 = 5;
constexpr size_t PIN_DIR_M2 = 4;

constexpr size_t PIN_STEP_M3 = 2;
constexpr size_t PIN_DIR_M3 = 1;

constexpr size_t PIN_EN_MCU = 3;

//******************** Pins TwoWire I²C - Otos
/*
#undef SDA
#define SDA 8
#undef SCL
#define SCL 9
*/
//******************** Pins LED - RGB
constexpr size_t PIN_RGB_LED = 38;
constexpr size_t PIN_WS2812_LED = 12;

//******************** Pins IHM
constexpr size_t PIN_SWITCH = 14;
constexpr size_t PIN_TEAM = 13;
constexpr size_t PIN_BAU = 15;
constexpr size_t PIN_START = 16;

#endif

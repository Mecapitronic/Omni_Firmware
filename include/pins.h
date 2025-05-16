#ifndef PIN_H
#define PIN_H

#include <pins_arduino.h>
#include <HardwareSerial.h>

//******************** Pins Undefined
constexpr size_t PIN_11 = 11;

//******************** Pins UART - Serial USB Debug
// https://community.platformio.org/t/esp32-s3-native-usb-interface-and-serial-monitor-missing-first-messages/40377/10
// Serial used for USB CDC
#undef SERIAL_DEBUG
#if (ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE) || WOKWI
// HWCDC Serial;
#define SERIAL_DEBUG Serial
#else
#error "USB Serial not working"
#endif

//******************** Pins UART - Serial 0 Lidar LD06
//  We don't need this redefinition of pin, it's just for information
#undef SOC_RX0
#undef SOC_TX0
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE && !WOKWI
constexpr size_t RX_LIDAR = 44;
constexpr size_t TX_LIDAR = 43;
// HardwareSerial Serial0(0);
#define SERIAL_LIDAR Serial0
#elif WOKWI
#undef RX2
#undef TX2
constexpr size_t RX_LIDAR = 19;
constexpr size_t TX_LIDAR = 20;
#define SERIAL_LIDAR Serial2
#else
#error "USB Serial not working"
#endif

//******************** Pins UART - Serial 1 Servo Dxl
#undef RX1
#undef TX1
constexpr size_t RX_SERVO = 18;
constexpr size_t TX_SERVO = 17;
#define SERIAL_SERVO Serial1

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

//******************** Pins SERVO
constexpr size_t PIN_SERVO_DIR = 10;

#endif

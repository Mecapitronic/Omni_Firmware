#ifndef PIN_H
#define PIN_H

#include <pins_arduino.h>
#include <HardwareSerial.h>

//******************** Pins UART - Serial
#define UART_RX0 SOC_RX0
#define UART_TX0 SOC_TX0

#if SOC_UART_NUM > 1
#define UART_RX1 RX1
#define UART_TX1 TX1
#endif

#if SOC_UART_NUM > 2
#define UART_RX2 RX2
#define UART_TX2 TX2
#endif

//******************** Pins Motors - Drivers
#define stepPinM1 18
#define dirPinM1  19

#define stepPinM2 16
#define dirPinM2  17

#define stepPinM3 2
#define dirPinM3  4


//******************** Pins TwoWire I²C - Otos
#define I2C_SDA 21
#define I2C_SCL 22

#endif

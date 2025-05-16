#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

#include "pins.h"
#include "ESP32_Helper.h"

namespace ServoAX12
{
// https://github.com/ROBOTIS-GIT/Dynamixel2Arduino/tree/master
// Please modify it to suit your hardware.
//#define DXL_SERIAL Serial2  // Serial 2 : U2TX = GPIO17; U2RX = GPIO16
//#define DXL_DIR_PIN 5
//const uint8_t DXL_ID = 5;
//const float DXL_PROTOCOL_VERSION = 1.0;

constexpr size_t MAX_BAUD = 5;
const BaudRate dxlBaud[MAX_BAUD] = {
    BaudRate::BAUD_RATE_57600,
    BaudRate::BAUD_RATE_115200,
    BaudRate::BAUD_RATE_1000000,
    BaudRate::BAUD_RATE_2000000,
    BaudRate::BAUD_RATE_3000000};

enum class DxlProtocolVersion
{
    PROTOCOL_1 = 1,
    PROTOCOL_2 = 2
};

constexpr size_t MAX_PROTOCOL = 2;
const DxlProtocolVersion dxlProtocol[MAX_PROTOCOL] = {
    DxlProtocolVersion::PROTOCOL_1,
    DxlProtocolVersion::PROTOCOL_2};

    void Initialisation();
    void Update();
    void HandleCommand(Command cmd);
    const void PrintCommandHelp();
    int16_t Scan();
    int16_t Scan(DxlProtocolVersion _protocol, BaudRate _dxlBaud);
    void PrintDxlInfo(uint8_t id = DXL_BROADCAST_ID);
}
#endif
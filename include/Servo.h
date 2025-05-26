#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

#include "pins.h"
#include "ESP32_Helper.h"

namespace ServoAX12
{
    // https://github.com/ROBOTIS-GIT/Dynamixel2Arduino/tree/master

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

    // id vitesse acceleration position command_position ledState
    struct ServoMotion
    {
        uint8_t id;
        float vitesse;
        float acceleration;
        float position;
        float command_position;
        bool ledState;
    };

    extern ServoMotion Servo_Up;
    // extern ServoMotion Servo_Down;
    extern ServoMotion Servo_Left;
    extern ServoMotion Servo_Right;

    void Initialisation();
    void InitServo(ServoMotion &servo);
    void Update();
    void UpdateServo(ServoMotion &servo);
    void SetServoPosition(ServoMotion &servo, float position);

    void HandleCommand(Command cmd);
    const void PrintCommandHelp();
    int16_t Scan();
    int16_t Scan(DxlProtocolVersion _protocol, BaudRate _dxlBaud);
    void PrintDxlInfo(uint8_t id = DXL_BROADCAST_ID);
}
#endif
#ifndef STEPPER_H
#define STEPPER_H

#include <ESP32_FastPWM.h>

#include "pin.h"
#include "ESP32_Helper.h"
using namespace Printer;

class Stepper
{
public:
    void Initialisation();
    void Update();
    void HandleCommand(Command cmd);
    void PrintCommandHelp();
    
    void SetMotorsSpeed(float speed_1_mms,float speed_2_mms,float speed_3_mms);
    void SetMotorSpeed(int motor_ID, float speed_mms);
};

#endif// STEPPER_H

#ifndef OTOS_H
#define OTOS_H
#ifdef SPARKFUN_OTOS

#include <Arduino.h>
#include "pin.h"
#include "ESP32_Helper.h"
using namespace Printer;

#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include <Wire.h>


class OpticalTrackingOdometrySensor
{
public:
    void Initialisation();
    void Update();
    void HandleCommand(Command cmd);
    void PrintCommandHelp();
    void Teleplot();

private:
    sfe_otos_pose2d_t myPosition;
    sfe_otos_pose2d_t myVelocity;
    sfe_otos_pose2d_t myAcceleration;

    QwiicOTOS myOtos;
};

#endif
#endif

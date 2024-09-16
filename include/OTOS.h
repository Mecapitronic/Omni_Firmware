#ifndef OTOS_H
#define OTOS_H
#ifdef SPARKFUN_OTOS

#include <Arduino.h>
#include "ESP32_Helper.h"

#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include <Wire.h>

using namespace Printer;

class OpticalTrackingOdometrySensor
{
public:
    void Initialisation();
    void Update();
    void HandleCommand(Command cmd);
    void Teleplot();

    sfe_otos_pose2d_t myPosition;
    sfe_otos_pose2d_t myVelocity;
    sfe_otos_pose2d_t myAcceleration;

private:
    QwiicOTOS myOtos;
};

#endif
#endif

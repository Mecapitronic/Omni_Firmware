#ifndef OTOS_H
#define OTOS_H

#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include <Arduino.h>
#include <Wire.h>

#include "ESP32_Helper.h"
#include "pins.h"

typedef sfeTkError_t sfTkError_t;

class OpticalTrackingOdometrySensor
{
public:
    void Initialisation(bool simulation = false);
    bool IsConnected();
    void Update();
    void HandleCommand(Command cmd);
    const void PrintCommandHelp();

    void SetPose(float x, float y, float h);
    void Teleplot();

    PoseF position;
    PoseF velocity;
    PoseF acceleration;

private:
    bool connected = false;
    QwiicOTOS myOtos;
};

#endif

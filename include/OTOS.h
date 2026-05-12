#ifndef OTOS_H
#define OTOS_H

#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include <Arduino.h>

#include "ESP32_Helper.h"
#include "main.h"
#include "pins.h"

typedef sfeTkError_t sfTkError_t;

namespace OTOS
{
    void Initialisation();
    void Update();

    void SetPose(float x, float y, float h);
    void Teleplot();

    extern PoseF position;
    extern PoseF velocity;
    extern PoseF acceleration;
} // namespace OTOS

#endif

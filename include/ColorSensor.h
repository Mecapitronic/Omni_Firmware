#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include "ESP32_Helper.h"
#include <Adafruit_TCS34725.h>
#include <FastLED.h>
#include <Wire.h>

// https://www.luisllamas.es/en/arduino-rgb-color-sensor-tcs34725/

namespace ColorSensor
{
    void Initialisation(void);
    void Update(void);
    CRGB GetColor();
    void PrintDebug(bool _printDebug);

    void printColorName(CRGB c);
} // namespace ColorSensor
#endif
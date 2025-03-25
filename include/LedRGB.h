#ifndef LEDRGB_H
#define LEDRGB_H

#include <FastLED.h>

#include "ESP32_Helper.h"
#include "pins.h"
#include "Structure.h"

#define NUM_LEDS 24
#define DATA_PIN 12

using namespace Printer;

struct LEDState
{
  uint8_t time;
  uint16_t color;
  std::vector<float> obstacles;
  std::vector<float> adversaries;
};
class LedRGB
{

public:
  void init(uint8_t numPixels, uint8_t data_pin);
  void update();
  std::vector<uint32_t> color = {0xFF0000, 0x00FF00, 0x0000FF}; //{0x0F0000,0x000F00,0x00000F,0x000000};

private:
  // Private member variables and methods
  int current_hue = 0;
  LEDState current_state;
  uint8_t m_numPixels;

  CRGB leds[NUM_LEDS];
};

#endif
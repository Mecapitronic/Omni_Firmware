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
  void Initialisation(uint8_t numPixels, uint8_t data_pin);
  /**
   * @brief update led ring display according to current robot state.
   * @description Takes into account: enemies position, obstacles position, robot position, robot state.
   * The informaitons are stored in robot_state private variables and translated into colors
   * and leds positions.
   *
   */
  void update();

  /**
   * @brief display a loading circle on the RGB LEDs ring
   * turns all leds on one by one, then turns them off one by one, cycling through colors.
   *
   * @return * void
   */
  void loader();
  int lidarPositionToLedNumber(float position, float min, float max);

private:
  std::vector<uint32_t> color = {0xFF0000, 0x00FF00, 0x0000FF}; //{0x0F0000,0x000F00,0x00000F,0x000000};
  CRGB leds[NUM_LEDS];

  int current_hue = 0;
  LEDState current_state;
  uint8_t m_numPixels;
};

#endif
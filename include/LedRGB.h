#ifndef LEDRGB_H
#define LEDRGB_H

#include "Structure.h"
#include "ESP32_Helper.h"
using namespace Printer;

#include <Adafruit_NeoPixel.h>
#include "pins.h"

struct LEDState
{
  uint8_t time;
  uint16_t color;
  vector<float32> obstacles;
  vector<float32> adversaries;
};
class LedRGB
{

public:
  Adafruit_NeoPixel pixels;
  std::vector<uint32_t> color = {0xFF0000, 0x00FF00, 0x0000FF}; //{0x0F0000,0x000F00,0x00000F,0x000000};

  void Initialisation(uint8_t numPixels = 1, uint8_t pin = PIN_WS2812_LED, uint8_t brightness = RGB_BRIGHTNESS);
  void Update();
  void HandleCommand(Command cmd);
  void PrintCommandHelp();

  /**
   * @brief display rolling a rainbow
   *
   */

  void rainbowRing();

  /**
   * @brief turn on one pixel that turns around the ring with a little fade out tail
   * change color after each turn
   * hold the current pixel in current_state.time
   * display a fade out tail on the 4 pixels behind the current pixel
   */
  void rollingColors();
  /*
   * set led color in function of its position on the ring.
   */
  void setColorWithPosition(float32 angle_rad, uint32_t color);
  std::vector<float32> getAdversaryPositions();
  void displayEnemies();

private:
  // Private member variables and methods
  int current_hue = 0;
  LEDState current_state;
};

#endif
#ifndef LEDRGB_H
#define LEDRGB_H

/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Structure.h"
#include "ESP32_Helper.h"
using namespace Printer;

#include <Adafruit_NeoPixel.h>
#include "pins.h"

class LedRGB
{

public:
  /****************************************************************************************
   * Variables
   ****************************************************************************************/
  Adafruit_NeoPixel pixels;
  std::vector<uint32_t> color = {0xFF0000, 0x00FF00, 0x0000FF}; //{0x0F0000,0x000F00,0x00000F,0x000000};

  /****************************************************************************************
   * Prototypes fonctions
   ****************************************************************************************/
  void Initialisation(uint8_t numPixels = 1, uint8_t pin = PIN_RGB_LED, uint8_t brightness = RGB_BRIGHTNESS);
  void Update();
  void HandleCommand(Command cmd);
  void PrintCommandHelp();

private:
  // Private member variables and methods
};

#endif
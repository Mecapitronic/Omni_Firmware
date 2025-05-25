#ifndef LEDRGB_H
#define LEDRGB_H

#include <FastLED.h>

#include "ESP32_Helper.h"
#include "GeoMathTools.h"
#include "Match.h"
#include "pins.h"
#include "Structure.h"

#define NUM_LEDS 24

using namespace Printer;


class LedRGB
{

public:
  void Initialisation();

  /**
   * @brief update the current robot state from value in other modules
   * get the adversaries and obstacles positions, robot position, robot state.
   * This function is called by the update() function.
   */
  void updateState(PoseF position, std::array<Circle, 10> obstacles);

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

  void emergencyStop();
  int lidarPositionToLedNumber(float position, float min, float max);
  int obstacleRelativePosition(PoseF robotPosition, Point obstaclePosition);

private:
  //{0x0F0000,0x000F00,0x00000F,0x000000};
  std::vector<uint32_t> color = {0xFF0000, 0x00FF00, 0x0000FF};
  CRGB leds[NUM_LEDS];

  int current_hue = 0;

  long time_led = 0;
  // uint16_t color;
  std::vector<float> obstacles;
  std::vector<float> adversaries;
};

#endif
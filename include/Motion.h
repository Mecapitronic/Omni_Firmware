#ifndef MOTION_H
#define MOTION_H

/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Structure.h"
#include "ESP32_Helper.h"
#include "GeoMathTools.h"
using namespace Printer;

class Motion
{
  /****************************************************************************************
   * Variables
   ****************************************************************************************/
public:
  // delta time used for integration from acceleration to velocity, or velocity to position => period of robot motion asserv
  static constexpr float dt_motion = 0.005;

  float accel_max;        // absolute maximum acceleration or deceleration (always positive)
  float speed_max;        // absolute maximum speed (always positive)
  float speed_limit;      // maximum speed allowed during this motion (always positive)
  float speed_final;      // final speed at the end of this motion (always positive or zero)
  float velocity_actual;  // current real velocity (positive or negative speed)
  float velocity_command; // velocity command send to the system
  float position_error;   // distance from current to end position (positive or negative distance)
  float position_margin;  // final position tolerance at the end of this motion

  float direction;        // direction of the linear motion vector in radian

  bool isRunning; // flag = 1 dans update() ; = 0 dans resetRamp()

public:
  /****************************************************************************************
   * Prototypes fonctions
   ****************************************************************************************/
  void Initialisation(float speedMax, float accelMax);
  void Update();
  void Stop();
  void TrapezoidalProfile();
  void AntiOverspeed();
  void SetMargin(float mm_or_rad);
  void Teleplot(String name);
};

#endif

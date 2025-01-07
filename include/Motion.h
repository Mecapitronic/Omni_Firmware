#ifndef MOTION_H
#define MOTION_H

/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Structure.h"
#include "ESP32_Helper.h"
using namespace Printer;

class Motion
{

public:
  /****************************************************************************************
   * Variables
   ****************************************************************************************/
  t_control position;     // déplacement
  t_control velocity;     // vitesse
  t_control acceleration; // acceleration
  t_control jerk;         // à-coup

public:
  /****************************************************************************************
   * Prototypes fonctions
   ****************************************************************************************/
  void Initialisation(int32 speedMax, int32 accelMax, int32 jerkMax);
  void Update();
  void Reset_Ramp();
  void Setpoint_Position(int32 position);
  bool Check_Position();
  void Teleplot(String name);
};

#endif

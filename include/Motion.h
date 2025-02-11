#ifndef MOTION_H
#define MOTION_H

/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Structure.h"
#include "ESP32_Helper.h"
#include "MATH_module.h"
using namespace Printer;

/****************************************************************************************
 * Variables
 ****************************************************************************************/
// famous "dt" used to pass from acceleration to velocity, or velocity to position => period of robot motion asserv
const float dt_asserv = 0.005; 
// paramètres de déplacement max pour le trapèze => directement en unité robot
const float speed_lin_mms_max = 1500.0; // vitesse linéaire max en mm/s
const float speed_ang_rads_max = radians(600.0); // vitesse angulaire max en rad/s (deg converti en rad)
// l'acceleration max est typiquement entre 50% et 200% de la vitesse max
const float accel_lin_mms2_max = 1000.0;  // acceleration linéaire max en mm/s2 
const float accel_ang_rads2_max = radians(600.0); // acceleration angulaire max en rad/s (deg converti en rad)

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
  float tolerance; // tolérance de position pour arrêter le déplacement
  bool isRunning; // flag = 1 dans update() ; = 0 dans resetRamp()

public:
  /****************************************************************************************
   * Prototypes fonctions
   ****************************************************************************************/
  void Initialisation(float speedMax, float accelMax, float jerkMax);
  void Update();
  void Stop();
  void Setpoint_Position(float position);
  void SetTolerance(float mm_or_rad);
  bool Check_Position();
  void Teleplot(String name);
};

#endif

#ifndef MAIN_H
#define MAIN_H

using namespace std;

#include <math.h>
#include <ESP32_FastPWM.h>
#include "pin.h"
#include "ESP32_Helper.h"
using namespace Printer;

// résolution moteur : 200*8=1600 microstep/tour, périmètre roue environ 185 mm, soit 8.65 step/mm
// vitesse max : 1 m/s = 8.65 kHz (step/s)
#define STEP_PER_MM 8.65


void setMotorSpeed(int motor_ID, float speed_mms);

#endif

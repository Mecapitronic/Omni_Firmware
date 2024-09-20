#ifndef MAIN_H
#define MAIN_H

using namespace std;

#include <math.h>
#include <ESP32_FastPWM.h>
#include "pin.h"
#include "ESP32_Helper.h"
using namespace Printer;
#include "OTOS.h"

// résolution moteur : 200*8=1600 microstep/tour, périmètre roue environ 185 mm, soit 8.65 step/mm
// vitesse max : 1 m/s = 8.65 kHz (step/s)
#define STEP_PER_MM 8.65

// coefficient "grossissant" de conversion mm => unité interne robot (pour calculs plus efficace en entier)
#define UNIT_PER_MM 512
// fréquence timer asserv en Hz
#define TIMER_ASSERV_FREQ 200

// distance du centre du robot au centre de la roue en mm
#define WHEEL_DISTANCE 100

void setMotorSpeed(int motor_ID, float speed_mms);
void SetRobotSpeed(float Vx, float Vy, float omega);
void setRobotPosition(float Vx, float Vy, float omega);
void updateOdometry();

void functionChrono(int nbrLoop);

#endif

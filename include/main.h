#ifndef MAIN_H
#define MAIN_H

using namespace std;

#include <math.h>
#include "pins.h"
#include "ESP32_Helper.h"
using namespace Printer;
#include "Structure.h"
#include "GeoMathTools.h"
#include "OTOS.h"
#include "Motor.h"
#include "Motion.h"
#include "Timer.h"
#include "Trajectory.h"
#include "LedRGB.h"
#include "ihm.h"
#include "match.h"
#include "Servo.h"
#include "Lidar.h"

#include "PathPlanning/PathFinding.h"

#ifndef ARDUINO_USB_MODE
#warning /*error*/ This ESP32 SoC has no Native USB interface
#elif ARDUINO_USB_MODE == 1
// #warning USB is in device mode
#else
#warning USB is in OTG mode
#include "USB.h"
#endif

#ifdef ARDUINO_USB_CDC_ON_BOOT

#endif

#ifdef SIMULATOR
const bool simulation = true;
#else
const bool simulation = false;
#endif

// distance between center of robot and center of wheel in mm => equal to mm/radian
#define CENTER_WHEEL_DISTANCE 115.0
#define MM_PER_RAD CENTER_WHEEL_DISTANCE

extern TimerThread timerMotion;
void timerMotionCallback(TimerHandle_t xTimer);

void TaskMatch(void *pvParameters);

void functionChrono(int nbrLoop = 1);
#endif

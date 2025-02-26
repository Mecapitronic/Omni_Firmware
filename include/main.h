#ifndef MAIN_H
#define MAIN_H

using namespace std;

#include <math.h>
#include "pin.h"
#include "ESP32_Helper.h"
using namespace Printer;
#include "Structure.h"
#include "GeoMathTools.h"
#include "OTOS.h"
#include "Motor.h"
#include "Motion.h"
#include "Trajectory.h"
#include "LedRGB.h"

#include "PathPlanning/Path_finding.h"

#ifndef ARDUINO_USB_MODE
    #warning /*error*/ This ESP32 SoC has no Native USB interface
#elif  ARDUINO_USB_MODE == 1
    //#warning USB is in device mode
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


/****************************************************************************************
 * Variables
 ****************************************************************************************/

// distance between center of robot and center of wheel in mm => equal to mm/radian
#define CENTER_WHEEL_DISTANCE 115.0
#define MM_PER_RAD  CENTER_WHEEL_DISTANCE

// Timer Settings
static const TickType_t timer_delay_1 = (1000 * Motion::dt_motion) / portTICK_PERIOD_MS; // period of robot motion asserv
static TimerHandle_t timer_handle_1 = NULL;
static bool timer_enable_1 = false;
#define DisableTimerMotion()    {timer_enable_1 = false;}
#define EnableTimerMotion()     {timer_enable_1 = true;}
#define TimerMotionIsEnable()   timer_enable_1

void timerCallback1(TimerHandle_t xTimer);

#endif

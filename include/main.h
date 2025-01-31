#ifndef MAIN_H
#define MAIN_H

#define ARDUINO_USB_MODE 1
#define ARDUINO_USB_CDC_ON_BOOT 1

using namespace std;

#include <math.h>
#include "pin.h"
#include "ESP32_Helper.h"
using namespace Printer;
#include "Structure.h"
#include "MATH_module.h"
#include "OTOS.h"
#include "MotorController.h"
#include "Motion.h"
#include "LedRGB.h"


#ifndef ARDUINO_USB_MODE
    #warning /*error*/ This ESP32 SoC has no Native USB interface
#elif  ARDUINO_USB_MODE == 1
    #warning USB is in device mode
#else
    #warning USB is in OTG mode
    #include "USB.h"
#endif

#ifdef ARDUINO_USB_CDC_ON_BOOT

#endif

// Motion parameters
// https://poivron-robotique.fr/Robot-holonome-localisation-partie-1.html
// 2  Y  1
//    o  X (angle ref X+)
//    3
// float theta1 = PI / 6;     // Angle for motor 1 : 30° PI/6
// float theta2 = PI * 5 / 6; // Angle for motor 2 : 150° PI*5/6
// float theta3 = PI * 3 / 2; // Angle for motor 3 : 270° PI*3/2

// coefficient multiplicateur de conversion mm => unité interne robot (pour calculs plus précis en entier)
// valeur arbitraire, une puissance de 2 permettrait d'optimiser les calculs avec des décalages de bits... todo
#define UNIT_PER_MM 512

// fréquence timer asserv en Hz
#define TIMER_ASSERV_FREQ        200
#define TIMER_ASSERV_FREQ_SQUARE (TIMER_ASSERV_FREQ * TIMER_ASSERV_FREQ)

// distance du centre du robot au centre de la roue en mm => équivaut à mm/radian
// TODO: calibrer la valeur en faisant plusieurs rotation et/ou comparer avec le capteur OTOS
#define CENTER_WHEEL_DISTANCE 100
#define MM_PER_RAD  CENTER_WHEEL_DISTANCE
#define UNIT_PER_RAD (UNIT_PER_MM * MM_PER_RAD)


// Anti Lock Speed // TODO: régler le décalage lors de la rotation => ok avec delay entre
#define ANTI_LOCK_SPEED_LIN 200
#define ANTI_LOCK_SPEED_ANG 300

//**** Macro de conversion en unité robot ****/
// Position => unit
#define MM_TO_UNIT(mm)      ((mm) * UNIT_PER_MM)
#define RAD_TO_UNIT(rad)    ((rad) * UNIT_PER_RAD)
#define DEG_TO_UNIT(deg)    ((deg) * UNIT_PER_RAD * DEG_TO_RAD) //(RAD_TO_UNIT(radians(deg)))
// Unit => position
#define UNIT_TO_MM(unit)    ((unit) / UNIT_PER_MM)
#define UNIT_TO_RAD(unit)   ((unit) / UNIT_PER_RAD)
#define UNIT_TO_DEG(unit)   (UNIT_TO_RAD(radians(deg)))
// Vitesse => speed unit
#define SPEED_LIN_MMS_TO_UNIT(mm_s)     (MM_TO_UNIT(mm_s) / TIMER_ASSERV_FREQ)          // [mm/s] to [unit/period]
#define SPEED_ANG_RADS_TO_UNIT(rad_s)   (RAD_TO_UNIT(rad_s) / TIMER_ASSERV_FREQ)        // [rad/s] to [unit/period]
#define SPEED_ANG_DEGS_TO_UNIT(deg_s)   (DEG_TO_UNIT(deg_s) / TIMER_ASSERV_FREQ)        // [deg/s] to [unit/period]
// Speed unit => vitesse
#define SPEED_LIN_UNIT_TO_MMS(unit)     (UNIT_TO_MM(unit) * TIMER_ASSERV_FREQ)          // [unit/period] to [mm/s]
#define SPEED_ANG_UNIT_TO_RADS(unit)    (UNIT_TO_RAD(unit) * TIMER_ASSERV_FREQ)         // [unit/period] to [rad/s]
#define SPEED_ANG_UNIT_TO_DEGS(unit)    (UNIT_TO_DEG(unit) * TIMER_ASSERV_FREQ)         // [unit/period] to [deg/s]
// Acceleration => accel unit
#define ACCEL_LIN_MMS2_TO_UNIT(mm_s2)   (MM_TO_UNIT(mm_s2) / TIMER_ASSERV_FREQ_SQUARE)  // [mm/s^2] to [unit/period]
#define ACCEL_ANG_RADS2_TO_UNIT(rad_s2) (RAD_TO_UNIT(rad_s2) / TIMER_ASSERV_FREQ_SQUARE)// [rad/s^2] to [unit/period]
#define ACCEL_ANG_DEGS2_TO_UNIT(deg_s2) (DEG_TO_UNIT(deg_s2) / TIMER_ASSERV_FREQ_SQUARE)// [deg/s^2] to [unit/period]
// Accel unit => acceleration
#define ACCEL_LIN_UNIT_TO_MMS2(unit)    (UNIT_TO_MM(unit) * TIMER_ASSERV_FREQ_SQUARE)   // [unit/period] to [mm/s^2]
#define ACCEL_ANG_UNIT_TO_RADS2(unit)   (UNIT_TO_RAD(unit) * TIMER_ASSERV_FREQ_SQUARE)  // [unit/period] to [rad/s^2]
#define ACCEL_ANG_UNIT_TO_DEGS2(unit)   (UNIT_TO_DEG(unit) * TIMER_ASSERV_FREQ_SQUARE)  // [unit/period] to [deg/s^2]

// paramètres de déplacement max pour le trapèze => directement en unité robot
const float speed_lin_max = SPEED_LIN_MMS_TO_UNIT(2000.0); //1000 // vitesse linéaire max en mm/s = x * 2.56
const float speed_ang_max = SPEED_ANG_DEGS_TO_UNIT(200.0); //200 // vitesse angulaire max en °/s = x * 4.46804288511

const float accel_lin_max = ACCEL_LIN_MMS2_TO_UNIT(200.0);  //80 // acceleration linéaire max en mm/s2 = x * 0.0128
const float accel_ang_max = ACCEL_ANG_DEGS2_TO_UNIT(45.0); //45 // acceleration angulaire max en °/s2 = x * 0.02234021442

const float jerk_lin = 20; // jerk linéaire en unité robot
const float jerk_ang = 20; // jerk angulaire en unité robot

const int anticipation_mm = 1;
const int anticipation_unit = MM_TO_UNIT(anticipation_mm);
const int anticipation_deg = 1;
const int anticipation_deg_unit = DEG_TO_UNIT(anticipation_deg);

// Timer Settings
static const TickType_t timer_delay_1 = (1000 / TIMER_ASSERV_FREQ) / portTICK_PERIOD_MS; // period of robot motion asserv = TIMER_ASSERV_FREQ Hz
static TimerHandle_t timer_handle_1 = NULL;
static bool timer_enable_1 = false;

void timerCallback1(TimerHandle_t xTimer);
void SetRobotPosition(float x, float y, float theta);

#endif

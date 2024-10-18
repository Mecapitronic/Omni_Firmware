#ifndef STEPPER_H
#define STEPPER_H

#include <ESP32_FastPWM.h>

#include "pin.h"
#include "ESP32_Helper.h"
using namespace Printer;

// résolution moteur : 200*8=1600 micro-step/tour, périmètre roue environ 185 mm, soit 8.65 step/mm
// vitesse max : 1 m/s = 8.65 kHz (step/s)
#define MOTOR_STEP_PER_MM 8.65
const float MM_PER_STEP_MOTOR =1/MOTOR_STEP_PER_MM ;//0.11560693641

#define BIT_RESOLUTION 13

// Bit resolution | Min Frequency [Hz] | Max Frequency [Hz]
//              1 |                489 |         40 078 277
//              2 |                245 |         20 039 138
//              3 |                123 |         10 019 569
//              4 |                 62 |          5 009 784
//              5 |                 31 |          2 504 892
//              6 |                 16 |          1 252 446
//              7 |                  8 |            626 223
//              8 |                  4 |            313 111
//              9 |                  2 |            156 555
//             10 |                  1 |             78 277
//             11 |                  1 |             39 138
//             12 |                  1 |             19 569
//             13 |                  1 |              9 784
//             14 |                  1 |              4 892
//             15 |                  2 |              2 446
//             16 |                  1 |              1 223

#if BIT_RESOLUTION == 1
#define FREQ_MAX_STEPPER 40078277
#define FREQ_MIN_STEPPER 489
#elif BIT_RESOLUTION == 13
#define FREQ_MAX_STEPPER 9784
#define FREQ_MIN_STEPPER 10 //! TODO To Be Defined
#elif BIT_RESOLUTION == 16
#define FREQ_MAX_STEPPER 1223
#define FREQ_MIN_STEPPER 1
#else
#error "Incorrect Bit Resolution value !"
#endif

const int SPEED_MAX_STEPPER = FREQ_MAX_STEPPER/MOTOR_STEP_PER_MM;

class Stepper
{
public:
    void Initialisation();
    void Update();
    void HandleCommand(Command cmd);
    void PrintCommandHelp();
    
    void SetMotorsSpeed(float speed_1_mms,float speed_2_mms,float speed_3_mms);
    void SetMotorSpeed(int motor_ID, float speed_mms);
    float GetMotorSpeed(int motor_ID);
};

#endif

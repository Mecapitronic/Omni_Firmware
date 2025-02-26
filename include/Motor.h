#ifndef MOTOR_H
#define MOTOR_H

#include <ESP32_FastPWM.h>

#include "pin.h"
#include "ESP32_Helper.h"
using namespace Printer;
#include "GeoMathTools.h"

// résolution moteur : 200*8=1600 micro-step/tour, périmètre roue environ 185 mm, soit 8.65 step/mm
// vitesse max : 1 m/s = 8.65 kHz (step/s)
#define MOTOR_STEP_PER_MM 8.65
const float MM_PER_STEP_MOTOR =1/MOTOR_STEP_PER_MM ;//0.11560693641

#define BIT_RESOLUTION 13

//Esp32 Wroom
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

/* Esp32 S3
Bit resolution | Min Frequency [Hz] | Max Frequency [Hz]
             1 |              19532 |           20039138
             2 |               9766 |           10019569
             3 |               4883 |            5009784
             4 |               2442 |            2504892
             5 |               1221 |            1252446
             6 |                611 |             626223
             7 |                306 |             313111
             8 |                153 |             156555
             9 |                 77 |              78277
            10 |                 39 |              39138
            11 |                 20 |              19569
            12 |                 10 |               9784
            13 |                  5 |               4892
            14 |                  3 |               2446
            15 |                  0 |                  0
            16 |                  0 |                  0
*/
#if BIT_RESOLUTION == 1
#define FREQ_MAX_STEPPER 40078277
#define FREQ_MIN_STEPPER 489
#elif BIT_RESOLUTION == 11
#define FREQ_MAX_STEPPER 19569
#define FREQ_MIN_STEPPER 20
#elif BIT_RESOLUTION == 12
#define FREQ_MAX_STEPPER 9784
#define FREQ_MIN_STEPPER 10 //! does not work under 10
#elif BIT_RESOLUTION == 13
#define FREQ_MAX_STEPPER 4892
#define FREQ_MIN_STEPPER 5
#elif BIT_RESOLUTION == 14
#define FREQ_MAX_STEPPER 2446
#define FREQ_MIN_STEPPER 3
#else
#error "Incorrect Bit Resolution value !"
#endif

const int SPEED_MAX_STEPPER = FREQ_MAX_STEPPER/MOTOR_STEP_PER_MM;
// const int var_duty_resolution =  (int) log2 (APB_CLK_FREQ / FREQ_MAX_STEPPER); //test

class Motor
{
public:

enum MotorBaseType
{
    NONE_0_MOTOR,
    UNICYCLE_1_MOTOR,
    DIFFERENTIAL_2_MOTORS,
    OMNIDIRECTIONAL_3_MOTORS,
};
    MotorBaseType motorBaseType = NONE_0_MOTOR;
    float centerToWheel = 0;

    void Initialisation(MotorBaseType _motorBaseType, float _centerToWheel);
    void Update(float linear_speed_mms, float linear_direction_rad, float angular_speed_rad);
    void HandleCommand(Command cmd);
    void PrintCommandHelp();
    
    //void SetMotorsSpeed(float speed_1_mms,float speed_2_mms,float speed_3_mms);
    void SetMotorSpeed(int motor_ID, float speed_mms);
    float GetMotorSpeed(int motor_ID);
    void test_ledc(void);
    void test_ledc2(void);
};

#endif

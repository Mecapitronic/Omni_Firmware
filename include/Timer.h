#ifndef TIMER_H
#define TIMER_H

#include "Motion.h"
#include "ESP32_Helper.h"

// Timer Settings
static const TickType_t timer_delay_1 = (1000 * Motion::dt_motion) / portTICK_PERIOD_MS; // period of robot motion asserv
static TimerHandle_t timer_handle_1 = NULL;
static bool timer_enable_1 = false;

#define DisableTimerMotion()    \
    {                           \
        timer_enable_1 = false; \
    }
#define EnableTimerMotion()    \
    {                          \
        timer_enable_1 = true; \
    }
#define TimerMotionIsEnable() timer_enable_1

void timerCallback1(TimerHandle_t xTimer);

#endif // TIMER_H
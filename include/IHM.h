#ifndef IHM_H
#define IHM_H

#include "ESP32_Helper.h"
#include "pins.h"
#include "Match.h"
#include <FastLED.h>
#include "Structure.h"

namespace IHM
{
    extern Team team;
    extern Enable tirettePresent;
    extern int switchMode;
    extern int bauReady;
    
    extern CRGB led[1];
    
    extern bool useBlink;
    
    void InitIHM();
    void UpdateHMI();
    void UpdateBAU();

    void Blink();

    void PrintTeam();
    void PrintSwitch();
    void PrintBAU();
    void PrintStart();

}
#endif
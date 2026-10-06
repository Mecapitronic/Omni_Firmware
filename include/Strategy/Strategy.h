#ifndef STRATEGY_H
#define STRATEGY_H

#include "Action.h"
#include "ESP32_Helper.h"

namespace Strategy
{
    void Initialisation();
    bool Execute_Action(uint8_t ID);
    void Print();
}

#endif // STRATEGY_H
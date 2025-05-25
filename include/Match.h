#ifndef MATCH_H
#define MATCH_H

#include "ESP32_Helper.h"
#include "Structure.h"

namespace Match
{

    // Time in ms
    static constexpr int time_start_match = 0;
    static constexpr int time_start_train = 0;
    static constexpr int time_end_train = time_start_train + 999000;
    static constexpr int time_end_match = time_start_match + 100000;

    
    extern State matchState;
    extern Enable matchMode;

    void startMatch();
    void stopMatch();
    long getMatchTimeSec();
    long getMatchTimeMs();
    void updateMatch();
    void printMatch();
}
#endif

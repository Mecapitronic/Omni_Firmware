#ifndef MATCH_H
#define MATCH_H

#include "ESP32_Helper.h"
#include "IHM.h"
#include "Structure.h"

namespace Match
{

    // Time in ms
    static constexpr int time_start_match = 0;
    static constexpr int time_start_train = 0;
    static constexpr int time_end_train = time_start_train + 999000;
    static constexpr int time_end_match = time_start_match + 100000;

    extern State matchState;

    void startMatch();
    void stopMatch();
    long getMatchTimeSec();
    long getMatchTimeMs();
    long getStartTime();
    void setStartTime(long start_time);

    void updateMatch();
    void printMatch();
} // namespace Match
#endif

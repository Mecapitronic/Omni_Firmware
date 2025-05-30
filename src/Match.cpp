#include "Match.h"

using namespace Printer;
using namespace std;

namespace Match
{
    namespace
    {
        long elapsedTime = 0;
        long startTime = 0;
    } // namespace

    // Get the current state of the match
    State matchState = State::MATCH_WAIT;

    void startMatch()
    {
        startTime = millis();
        matchState = State::MATCH_BEGIN;
        printMatch();
    }

    void stopMatch()
    {
        matchState = State::MATCH_STOP;
        printMatch();
    }

    long getMatchTimeSec()
    {
        return (getMatchTimeMs()) / 1000;
    }

    long getMatchTimeMs()
    {
        if (matchState != State::MATCH_WAIT)
            return millis() - startTime;
        else
            return 0; // Match not started yet
    }

    long getStartTime()
    {
        return startTime;
    }

    void setStartTime(long start_time)
    {
        startTime = start_time;
    }

    void updateMatch()
    {
        if (matchState == State::MATCH_WAIT)
        {
            // Wait start of match
        }
        else if (matchState == State::MATCH_BEGIN)
        {
            // In TaskMatch init map and robot position then go into run mode
        }
        else if ((matchState == State::MATCH_RUN) || (matchState == State::MATCH_STOP))
        {
            // PAMI still running or waiting for end of match
            elapsedTime = millis() - startTime;

            if ((elapsedTime >= time_end_match && IHM::switchMode == 1)
                || (elapsedTime >= time_end_train && IHM::switchMode != 1))
            {
                matchState = State::MATCH_END;
            }
        }
        else if (matchState == State::MATCH_END)
        {
            // End of match
        }
        else
        {
            // Not possible
        }
    }

    void printMatch()
    {
        print("Match State : ");
        switch (matchState)
        {
            ENUM_PRINT(State::MATCH_WAIT);
            ENUM_PRINT(State::MATCH_BEGIN);
            ENUM_PRINT(State::MATCH_RUN);
            ENUM_PRINT(State::MATCH_STOP);
            ENUM_PRINT(State::MATCH_END);
        }
    }
} // namespace Match
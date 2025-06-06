#include "IHM.h"

using namespace Printer;
using namespace std;

namespace IHM
{
    namespace
    {
        int bauReadyPrev = -1;

        int ledState = LOW;
        Timeout ledTimeOut;
    } // namespace

    Team team = Team::None;

    Enable tirettePresent = Enable::ENABLE_NONE;
    // Match = 1, TEST = 0, None = -1
    // BUG
    int switchMode = 1;
    // int switchMode = -1;
    //  Retiré (OK) = 1, Enclenché (NOK) = 0, None = -1
    int bauReady = -1;

    CLEDController *LEDcontroller;
    CRGB builtin_led;

    bool useBlink = true;

    void InitIHM()
    {
        // Tirette
        pinMode(PIN_START, INPUT_PULLUP);

        // Switch Color
        pinMode(PIN_TEAM, INPUT_PULLUP);

        // Switch Switch
        pinMode(PIN_SWITCH, INPUT_PULLUP);

        // Boutton Arret d'Urgence
        pinMode(PIN_BAU, INPUT);

        ledTimeOut.Start(1000);

        tirettePresent = (Enable)!digitalRead(PIN_START);
        if (tirettePresent == Enable::ENABLE_TRUE)
        {
            println("Tirette : Présente au démarrage");
            ledTimeOut.timeOut = 500;
        }
        else if (tirettePresent == Enable::ENABLE_FALSE)
        {
            println("Tirette : Absente au démarrage");
            ledTimeOut.timeOut = 500;
        }
        Match::printMatch();
        UpdateHMI();
        UpdateBAU();

        LEDcontroller = &FastLED.addLeds<WS2812, PIN_RGB_LED, RGB>(&builtin_led, 1);
        builtin_led = CRGB::Black;
        LEDcontroller->showLeds(BUILTIN_BRIGHTNESS);
    }

    void UpdateHMI()
    {
        // Lecture du bouton Team Yellow / Blue
        Team teamTmp = (Team)digitalRead(PIN_TEAM);
        if (teamTmp != team)
        {
            team = teamTmp;
            PrintTeam();
        }

        // Lecture du bouton Switch TEST / OK
        // int switchTmp = digitalRead(PIN_SWITCH);
        // if (switchTmp != switchMode)
        //{
        //    switchMode = switchTmp;
        //    PrintSwitch();
        //}

        // Lecture de la tirette
        Enable tiretteTmp = (Enable)!digitalRead(PIN_START);
        if (tiretteTmp != tirettePresent)
        {
            if (tiretteTmp == Enable::ENABLE_TRUE)
            {
                println("Tirette : insérée");
                // ledTimeOut.timeOut = 500;
            }
            else if (tiretteTmp == Enable::ENABLE_FALSE)
            {
                // ledTimeOut.timeOut = 1000;
                Match::startMatch();
            }
            tirettePresent = tiretteTmp;
            PrintStart();
        }

        // if ((tirettePresent == Enable::ENABLE_FALSE && switchMode == 1)
        //     || (tirettePresent == Enable::ENABLE_TRUE && switchMode != 1))
        // {
        //     // If tirette is not present and switch is OK, start the match
        //     Match::startMatch();
        // }

        if (bauReadyPrev != bauReady)
        {
            bauReadyPrev = bauReady;
            PrintBAU();
        }
    }

    void UpdateBAU()
    {
        // Lecture du BAU
        int bauTmp = digitalRead(PIN_BAU);
        if (bauTmp != bauReady)
        {
            bauReady = bauTmp;
        }
    }

    void Blink()
    {
        CRGB teamColor;
        if (team == Team::Jaune)
        {
            teamColor = CRGB::Gold;
        }
        if (team == Team::Bleu)
        {
            teamColor = CRGB::Blue;
        }

        // no blink behavior
        if (!bauReady)
        {
            builtin_led = CRGB::Red;
        }
        else
        {
            builtin_led = teamColor;
        }

        if (useBlink)
        {
            if (ledTimeOut.IsTimeOut())
            {
                ledState = !ledState;
            }

            if (ledState)
                builtin_led = teamColor;
            else
            {
                builtin_led = bauReady ? CRGB::Black : CRGB::Red;
            }
        }
        LEDcontroller->showLeds(BUILTIN_BRIGHTNESS);
    }

    void PrintAll()
    {
        PrintTeam();
        PrintSwitch();
        PrintBAU();
        PrintStart();
    }

    void PrintTeam()
    {
        print("Team    : ");
        if (team == Team::Bleu)
            println("Bleu");
        else if (team == Team::Jaune)
            println("Jaune");
    }

    void PrintSwitch()
    {
        print("Switch  : ");
        if (switchMode == 1)
            println("OK");
        else
            println("TEST");
    }

    void PrintBAU()
    {
        print("BAU     : ");
        if (bauReady == 1)
            println("Retiré");
        else
            println("Enclenché");
    }

    void PrintStart()
    {
        print("Tirette : ");
        if (tirettePresent == Enable::ENABLE_TRUE)
            println("Insérée");
        else if (tirettePresent == Enable::ENABLE_FALSE)
            println("Enlevée");
    }
} // namespace IHM
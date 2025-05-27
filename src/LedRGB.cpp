#include "LedRGB.h"

using namespace Printer;

void LedRGB::Initialisation()
{
    print("RGB initialisation of ", NUM_LEDS, " pixels");
    println(" on pin ", PIN_WS2812_LED);

    ring_controller = &FastLED.addLeds<NEOPIXEL, PIN_WS2812_LED>(leds, NUM_LEDS);
    FastLED.setBrightness(RING_BRIGHTNESS);

    changeColorTimer.Start(TRANSITION_DELAY_MS / TRANSITION_STEPS);
}

// This function would update the current state based on the robot's state
void LedRGB::updateState(PoseF position, std::array<Circle, 10> obstacles_list)
{
    IHM::team == Team::Jaune ? team_color = CRGB::Gold : team_color = CRGB::DodgerBlue;

    m_obstacles_list.clear();
    for (const auto &obstacle : obstacles_list)
    {
        m_obstacles_list.emplace_back(obstacle.p.x, obstacle.p.y);
    }
}

// pour faire un clignotement on stock 2 couleurs pour alterner
void LedRGB::update()
{
    if (Match::matchState == State::MATCH_END)
    {
        rainbow();
        return;
    }

    // si le bouton d'arrêt d'urgence est enclenché on voit rouge
    if (IHM::bauReady == 0)
    {
        filling_color = CRGB::Red;
    }
    else
    {
        if (Match::matchState == State::MATCH_WAIT || Match::matchState == State::MATCH_BEGIN)
        {
            filling_color = team_color; // Set the team color when waiting for the match to start
        }
        else
        {
            // Blend the team color with black for a lighter shade
            filling_color = team_color.lerp8(CRGB::Black, 128);
        }
    }
    fill_solid(leds, NUM_LEDS, filling_color); // Clear all LEDs

    // update led ring display according to current robot state
    if (Match::matchState == State::MATCH_BEGIN && Match::matchState == State::MATCH_RUN)
    {
        match_time_led = (NUM_LEDS * Match::getMatchTimeMs()) / Match::time_end_match;
        // Ensure we don't go out of bounds
        if (match_time_led <= NUM_LEDS)
        {
            leds[match_time_led] = CRGB::Green; // Set the time in green
        }
    }
    if (Match::matchState == State::MATCH_WAIT)
    {
        if (current_hue >= NUM_LEDS)
        {
            current_hue = 0; // Reset hue to avoid overflow
        }
        leds[current_hue++] = CRGB::Green;
    }

    // // calculate obstacles orientation relative to the robot position and orientation
    // for (size_t i = 0; i < m_obstacles_list.size(); i++)
    // {
    //     // get the led number corresponding to the obstacle position
    //     int ledNumber = lidarPositionToLedNumber(m_obstacles_list[i].x, -200, 200);
    //     if (ledNumber >= 0 && ledNumber < NUM_LEDS)
    //     {
    //         leds[ledNumber] = CRGB::Violet; // Example: Violet for obstacles
    //     }
    //     else
    //     {
    //         leds[0] = CRGB::Violet;
    //     }
    // }

    // else if (i < adversaries.size() + obstacles.size())
    // {
    //     // Set color based on adversary position
    //     leds[i] = CRGB::Red; // Example: Blue for adversaries
    // }

    // si le match n'est pas démarré on affiche la couleur de l'équipe
    // if (Match::matchState == State::MATCH_WAIT || Match::matchState == State::MATCH_BEGIN)
    // {
    //     color_team = team_color;
    // }
    // else
    // {
    //     color_team = CRGB::Black;
    // }

    ring_controller->showLeds(RING_BRIGHTNESS);
}

int LedRGB::obstacleRelativePosition(PoseF robotPosition, Point obstaclePosition)
{
    // get angle between h and the vector formed by robotposition x,y and obstaclePosition x,y

    //     DirectionFromPoint(robotPosition.x, robotPosition.y, obstaclePosition.x, obstaclePosition.y)

    //     atan2((object 1 Y - object 2 Y) / (object 1 X - object 2 X)) + 180}

    return 0;
}

int LedRGB::lidarPositionToLedNumber(float position, float min, float max)
{
    // Convert the position to a value between 0 and NUM_LEDS
    auto ledNumber = static_cast<int>((position - min) / (max - min) * NUM_LEDS);
    // Ensure the ledNumber is within bounds
    if (ledNumber < 0)
        ledNumber = 0;
    else if (ledNumber >= NUM_LEDS)
        ledNumber = NUM_LEDS - 1;
    return ledNumber;
}

// ANIMATIONS

void LedRGB::rainbow()
{
    if (changeColorTimer.IsTimeOut())
    {
        fill_rainbow_circular(leds, NUM_LEDS, current_hue, false);
        ring_controller->showLeds(RING_BRIGHTNESS); // Show the current color
        current_hue += 2;                           // Increment hue for the next cycle
    }
}

inline void LedRGB::TwoColorsTransition(CRGB color1, CRGB color2)
{
    if (changeColorTimer.IsTimeOut())
    {
        // uint8_t blendAmount = map(i, 0, steps, 0, 255); // 0 → 255
        CRGB color = blend(color1, color2, blendAmount);
        fill_solid(leds, NUM_LEDS, color);
        ring_controller->showLeds(RING_BRIGHTNESS); // Show the current color

        blendAmount++;
        if (blendAmount > 255)
        {
            blendAmount = 0; // Reset blend amount after full transition
        }
    }
}

inline void LedRGB::glowTwoColors(CRGB color1, CRGB color2)
{
    TwoColorsTransition(color1, color2);
    // TwoColorsTransition(color2, color1);
}

inline void LedRGB::glowOneColor(CRGB color)
{
    LedRGB::glowTwoColors(CRGB::Black, color);
}

inline void LedRGB::emergencyStop()
{
    fill_solid(leds, NUM_LEDS, CRGB::Red);
    for (uint8_t i = RING_BRIGHTNESS; i > 0; i--)
    {
        ring_controller->showLeds(i);
        delay(30);
    }
    ring_controller->showLeds(0); // Turn off all LEDs
    for (uint8_t i = 0; i < RING_BRIGHTNESS; i++)
    {
        ring_controller->showLeds(i);
        delay(30);
    }
}
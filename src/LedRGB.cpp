#include "LedRGB.h"

using namespace Printer;

void LedRGB::Initialisation(Robot *robotPosition)
{
    // Initialize timers for color transitions and rotation
    print("RGB initialisation of ", NUM_LEDS, " pixels");
    println(" on pin ", PIN_WS2812_LED);

    // Initialize the FastLED library
    ring_controller = &FastLED.addLeds<NEOPIXEL, PIN_WS2812_LED>(leds, NUM_LEDS);
    FastLED.setBrightness(RING_BRIGHTNESS);
    // Set the initial color of the LEDs to black
    fill_solid(leds, NUM_LEDS, CRGB::DarkCyan);
    ring_controller->showLeds(RING_BRIGHTNESS);

    // Initialize the robot position pointer
    robot_position = robotPosition;

    // Initialize the timers
    changeColorTimer.Start(TRANSITION_DELAY_MS / TRANSITION_STEPS);
    rotationTimer.Start(50);
}

// pour faire un clignotement on stock 2 couleurs pour alterner
void LedRGB::update()
{
    // Update data
    IHM::team == Team::Jaune ? team_color = CRGB::Gold : team_color = CRGB::DodgerBlue;

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
        if (Match::matchState == State::MATCH_WAIT
            || Match::matchState == State::MATCH_BEGIN)
        {
            // Set the team color when waiting for the match to start
            filling_color = team_color;
        }
        else
        {
            // Blend the team color with black for a lighter shade
            filling_color = team_color.lerp8(CRGB::Black, 200);
        }
    }
    fill_solid(leds, NUM_LEDS, filling_color); // Clear all LEDs

    if (Match::matchState == State::MATCH_WAIT)
    {
        if (current_hue >= NUM_LEDS)
        {
            current_hue = 0; // Reset hue to avoid overflow
        }
        leds[current_hue] = CRGB::ForestGreen;
        if (rotationTimer.IsTimeOut())
        {
            current_hue++; // Increment hue for the next cycle
        }
    }

    // TODO add red leds for errors (like otos not connected)
    // we could have an error counters and increment or decrement outside ledrgb
    // and turn on the number of leds equivalent to the number of errors

    // update led ring display according to current robot state
    if (Match::matchState == State::MATCH_BEGIN || Match::matchState == State::MATCH_RUN)
    {
        match_time_led = (NUM_LEDS * Match::getMatchTimeMs()) / Match::time_end_match;
        // Ensure we don't go out of bounds
        if (match_time_led <= NUM_LEDS)
        {
            leds[match_time_led] = CRGB::ForestGreen; // Set the time in green
        }
    }

    // display obstacles around the robot
    PoseF robot_current_position = robot_position->GetPoseF();
    for (const auto &obstacle : Obstacle::obstacle)
    {
        if (obstacle.r == 0)
        {
            continue; // Skip if the obstacle radius is invalid
        }

        // calculate obstacles orientation relative to the robot position and orientation
        leds[directionToLedNumber(RelativeDirection(obstacle.p))] = CRGB::White;
    }

    ring_controller->showLeds(RING_BRIGHTNESS);
}

void LedRGB::rainbow()
{
    if (rotationTimer.IsTimeOut())
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

float LedRGB::RelativeDirection(Point point)
{
    return atan2(point.y - robot_position->y, point.x - robot_position->x);
}

uint8_t LedRGB::directionToLedNumber(float angle)
{
    // Normalize the angle to be in the range [0, 2π]
    while (angle < 0)
        angle += 2 * M_PI;
    while (angle >= 2 * M_PI)
        angle -= 2 * M_PI;

    // shift the angle from 90
    angle -= 90;

    // Convert the angle to a value between 0 and NUM_LEDS
    int led_number = static_cast<int>(angle / (2 * M_PI) * NUM_LEDS);

    // Adjust for clockwise direction
    led_number = (NUM_LEDS - led_number) % NUM_LEDS;

    // println("received angle: ", angle);
    // println("calculated led number: ", led_number);

    if (led_number >= NUM_LEDS || led_number < 0)
    {
        // If the angle is out of bounds, return a default value
        return 12; // Default LED number, can be adjusted as needed
    }

    return static_cast<uint8_t>(led_number);
}

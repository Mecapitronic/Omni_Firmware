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
    fill_solid(leds, NUM_LEDS, CRGB::Purple);
    ring_controller->showLeds(RING_BRIGHTNESS);

    // Initialize the robot position pointer
    robot_position = robotPosition;

    // Initialize the timers
    // we need a rotation of led every 2 seconds, so 2s/24
    rotationTimer.Start(2000 / NUM_LEDS);
}

void LedRGB::robotIsStarting()
{
    fill_solid(leds, NUM_LEDS, CRGB::Purple);
    ring_controller->showLeds(RING_BRIGHTNESS);
}

// pour faire un clignotement on stock 2 couleurs pour alterner
void LedRGB::update()
{
    // Update data
    // select right team color
    IHM::team == Team::Jaune ? team_color = CRGB::Gold : team_color = CRGB::DodgerBlue;
    // set time led to green if match mode, violet in test mode
    IHM::switchMode == 1 ? clock_color = CRGB::ForestGreen : clock_color = CRGB::Purple;

    if (Match::matchState == State::MATCH_END)
    {
        rainbow();
        return;
    }

    // si le bouton d'arrêt d'urgence est enclenché on voit rouge
    if (IHM::bauReady != 1)
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

    displayTime();

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

// comme une led représente 4 secondes, on peut faire un tour complet en 1 secondes
// faire 4 tours avant d'incrémenter la led
// quand on est pas en match on peut faire une petite navette en allumant la led avant et
// la led après un peu moins fort
void LedRGB::displayTime()
{
    // display seconds counter in wait, begin and run states
    if (Match::matchState != State::MATCH_END && Match::matchState != State::MATCH_STOP)
    {
        leds[secondsCounter] = clock_color;
        if (rotationTimer.IsTimeOut())
        {
            secondsCounter++;
        }
    }

    // update led ring display according to current robot state
    if (Match::matchState == State::MATCH_BEGIN || Match::matchState == State::MATCH_RUN)
    {
        // get time instead of using timer
        // match_time_led = (NUM_LEDS * Match::getMatchTimeMs()) / Match::time_end_match;

        if (!matchClockTimer.isRunning)
        {
            // match lasts 100 seconds
            matchClockTimer.Start(100000 / NUM_LEDS);
            match_time_led = 0;
            secondsCounter = 0;
        }

        // change of led all 4 seconds
        if (matchClockTimer.IsTimeOut())
        {
            match_time_led++;
        }

        // Ensure we don't go out of bounds
        if (match_time_led <= NUM_LEDS)
        {
            leds[match_time_led] = clock_color;
        }
    }

    if (secondsCounter >= NUM_LEDS)
    {
        secondsCounter = 0; // avoid overflow
    }
}

void LedRGB::rainbow()
{
    if (rotationTimer.IsTimeOut())
    {
        fill_rainbow_circular(leds, NUM_LEDS, secondsCounter, false);
        ring_controller->showLeds(RING_BRIGHTNESS); // Show the current color
        secondsCounter += 2;                        // Increment hue for the next cycle
    }
}

inline void LedRGB::TwoColorsTransition(CRGB color1, CRGB color2)
{
    if (rotationTimer.IsTimeOut())
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

void LedRGB::emergencyStop()
{
    fill_solid(leds, NUM_LEDS, CRGB::Red);
    for (uint8_t i = RING_BRIGHTNESS; i > 0; i--)
    {
        ring_controller->showLeds(i);
        delay(1);
    }
    ring_controller->showLeds(0); // Turn off all LEDs
    for (uint8_t i = 0; i < RING_BRIGHTNESS; i++)
    {
        ring_controller->showLeds(i);
        delay(1);
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

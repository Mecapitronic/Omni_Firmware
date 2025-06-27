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
    rotationTimer.Start(50);
    blendTimeOut.Start(5);
    blendCoef = 5;
}

void LedRGB::robotIsStarting()
{
    fill_solid(leds, NUM_LEDS, CRGB::Purple);
    ring_controller->showLeds(RING_BRIGHTNESS);
}

void LedRGB::update()
{
    if (Match::matchState == State::MATCH_END)
    {
        rainbow();
        return;
    }
    CRGB team_color = (IHM::team == Team::Jaune ? CRGB::Gold : CRGB::DodgerBlue);

    displayTime();

    // si le bouton d'arrêt d'urgence est enclenché on voit rouge
    if (IHM::bauReady != 1)
    {
        CRGB filling_color = CRGB::Red;
        filling_color = glowTwoColors(filling_color, team_color);
        fill_solid(leds, NUM_LEDS, filling_color);
    }

    displayObstacle();

    ring_controller->showLeds(RING_BRIGHTNESS);
}

void LedRGB::displayTime()
{
    // select right team color
    CRGB team_color = (IHM::team == Team::Jaune ? CRGB::Gold : CRGB::DodgerBlue);

    fill_solid(leds, NUM_LEDS, team_color);

    if (Match::matchState == State::MATCH_WAIT)
    {
        // 1 turn in 1 sec
        float ratio = 1000.0 / NUM_LEDS;
        long time = 0;
        if (Match::matchState == State::MATCH_WAIT)
            time = millis();
        else
            time = Match::getMatchTimeMs();
        float index = (time % 1000) / ratio;
        if (index >= NUM_LEDS)
            index = 0;

        leds[(int)index] = CRGB::Black;
    }
    else
    {
        // slowly decrease the leds from team color to black according to match time left
        uint8_t match_time_index_led =
            (NUM_LEDS * Match::getMatchTimeMs()) / Match::time_end_match;

        leds[match_time_index_led] = team_color.lerp8(
            CRGB::Black,
            map(Match::getMatchTimeMs() % (Match::time_end_match / NUM_LEDS),
                0,
                Match::time_end_match / NUM_LEDS,
                0,
                255));
        for (int i = 0; i < match_time_index_led; i++)
        {
            leds[i] = CRGB::Black;
        }
    }
}

void LedRGB::displayObstacle()
{
    // display obstacles around the robot
    for (const auto &obstacle : Obstacle::obstacle)
    {
        if (obstacle.r == 0)
        {
            continue; // Skip if the obstacle radius is invalid
        }

        Point robot = robot_position->GetPoint();
        // calculate obstacles orientation relative to the robot position and orientation
        float relativeDirection =
            DirectionFromPoints(robot, obstacle.p) - robot_position->h;
        // calculate the distance between robot and obstacle
        float relativeDistance = DistanceBetweenPoints(robot, obstacle.p);

        if (relativeDistance > 375)
            leds[directionToLedNumber(relativeDirection)] =
                (IHM::team != Team::Jaune ? CRGB::Gold : CRGB::DodgerBlue);
        else
            leds[directionToLedNumber(relativeDirection)] = CRGB::DarkRed;
    }
}

void LedRGB::rainbow()
{
    static uint8_t secondsCounter = 0;
    if (rotationTimer.IsTimeOut())
    {
        fill_rainbow_circular(leds, NUM_LEDS, secondsCounter++, false);
        ring_controller->showLeds(RING_BRIGHTNESS);
    }
}

inline CRGB LedRGB::TwoColorsTransition(CRGB color1, CRGB color2)
{

    // uint8_t blendAmount = map(i, 0, steps, 0, 255); // 0 → 255
    CRGB color = blend(color1, color2, blendAmount);
    // fill_solid(leds, NUM_LEDS, color);
    // ring_controller->showLeds(RING_BRIGHTNESS); // Show the current color
    static int8_t direction = 1; // 1 for forward, -1 for backward
    if (blendTimeOut.IsTimeOut())
    {
        blendAmount += direction;
        if (blendAmount >= 255)
        {
            blendAmount = 255;
            direction = -blendCoef; // Reverse direction
        }
        else if (blendAmount <= 0)
        {
            blendAmount = 0;
            direction = blendCoef; // Reverse direction
        }
    }
    return color;
}

inline CRGB LedRGB::glowTwoColors(CRGB color1, CRGB color2)
{
    return TwoColorsTransition(color1, color2);
}

inline CRGB LedRGB::glowOneColor(CRGB color)
{
    return LedRGB::glowTwoColors(CRGB::Black, color);
}

void LedRGB::emergencyStopAtStart()
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

uint8_t LedRGB::directionToLedNumber(float angle)
{
    // shift the angle of -90 because the 0 led in the ring face the 90° of the robot
    angle -= HALF_PI;

    // Normalize the angle to be in the range [0, 2π]
    angle = NormalizeAngle2PI(angle);


    // Convert the angle to a value between 0 and NUM_LEDS
    int led_number = static_cast<int>(angle / TWO_PI * NUM_LEDS);

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

#include "LedRGB.h"

using namespace Printer;

void LedRGB::Initialisation(uint8_t numPixels, uint8_t data_pin)
{
    print("RGB initialisation of ", numPixels, " pixels");
    println(" on pin ", data_pin);
    m_numPixels = numPixels;

    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, numPixels);
    FastLED.setBrightness(RGB_BRIGHTNESS);
}

// This function would update the current_state based on the robot's state
void LedRGB::updateState(PoseF position, std::array<Circle, 10> obstacles)
{   
    long match_time =(NUM_LEDS  * Match::getMatchTimeSec()) / Match::time_end_match;
    if (match_time == 0 || match_time > NUM_LEDS) {
        match_time = 1;
    }
    current_state.time_led = match_time;

    println("LED: Updating state with time_led: ", current_state.time_led);
}


void LedRGB::update()
{
    // update led ring display according to current robot state
    for (int i = 0; i < NUM_LEDS; i++)
    {
        // set the time as filled green leds from start to current_state.time
        if (i == current_state.time_led)
        {
            leds[i] = CRGB::Green;
        }
        else
        {
            // Default color for remaining LEDs
            leds[i] = CRGB::Black;
        }
        // if (i < current_state.obstacles.size())
        // {
        //     // Set color based on obstacle position
        //     leds[i] = CRGB::Violet;
        // }
        // else if (i < current_state.adversaries.size() + current_state.obstacles.size())
        // {
        //     // Set color based on adversary position
        //     leds[i] = CRGB::Red; // Example: Blue for adversaries
        // }

    }
    FastLED.show(); // Update the LEDs to reflect the changes   
    vTaskDelay(100); // Add a small delay to avoid overwhelming the LED updates
}

int LedRGB::obstacleRelativePosition(PoseF robotPosition, Point obstaclePosition) {
    // get angle between h and the vector formed by robotposition x,y and obstaclePosition x,y

//     DirectionFromPoint(robotPosition.x, robotPosition.y, obstaclePosition.x, obstaclePosition.y)

//     atan2((object 1 Y - object 2 Y) / (object 1 X - object 2 X)) + 180}

}

int LedRGB::lidarPositionToLedNumber(float position, float min, float max)
{
    // Convert the position to a value between 0 and m_numPixels
    auto ledNumber = static_cast<int>((position - min) / (max - min) * m_numPixels);
    // Ensure the ledNumber is within bounds
    if (ledNumber < 0)
        ledNumber = 0;
    else if (ledNumber >= m_numPixels)
        ledNumber = m_numPixels - 1;
    return ledNumber;
}

// ANIMATIONS

void LedRGB::loader()
{
    for (int i = 0; i < m_numPixels; i++)
    {
        leds[i] = CRGB::Black; // Clear all LEDs
        FastLED.show();
        delay(50);
    }
    for (int i = 0; i < m_numPixels; i++)
    {
        leds[i] = CHSV(current_hue++, 255, 255); // Cycle through colors
        FastLED.show();
        delay(50);
    }
}

void LedRGB::emergencyStop()
{
    // set all leds glowing red smoothly
    for (int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = CRGB::Red; // Set all LEDs to red
    }
    FastLED.show();
    // going proegressively to black
    for (int i = 255; i >= 0; i--)
    {
        FastLED.setBrightness(i); // Decrease brightness
        FastLED.show();
        delay(10);
    }
    FastLED.clear(); // Clear the LEDs
    FastLED.show(); // Update the LEDs to turn them off

}
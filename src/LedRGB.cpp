#include "LedRGB.h"
#include "GeoMathTools.h"

void LedRGB::Initialisation(uint8_t numPixels, uint8_t data_pin)
{
    print("RGB initialisation of ", numPixels, " pixels");
    println(" on pin ", data_pin);
    m_numPixels = numPixels;

    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, numPixels);
    FastLED.setBrightness(RGB_BRIGHTNESS);
}

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

void LedRGB::update()
{
    // update led ring display according to current robot state
    updateState(); // Update the current_state based on robot's state and other conditions
    for (int i = 0; i < m_numPixels; i++)
    {
        // set the time as filled green leds from start to current_state.time
        if (i < current_state.time)
        {
            leds[i] = CRGB::Green; // Example: Green for time
        }

        if (i < current_state.obstacles.size())
        {
            // Set color based on obstacle position
            leds[i] = CRGB::Violet; // Example: Red for obstacles
        }
        else if (i < current_state.adversaries.size() + current_state.obstacles.size())
        {
            // Set color based on adversary position
            leds[i] = CRGB::Red; // Example: Blue for adversaries
        }
        else
        {
            // Default color for remaining LEDs
            leds[i] = CRGB::Black;
        }
    }
}

int LedRGB::obstacleRelativePosition(PoseF robotPosition, Point obstaclePosition){
    // get angle between h and the vector formed by robotposition x,y and obstaclePosition x,y

    DirectionToPoint(robotPosition.x, robotPosition.y, obstaclePosition.x, obstaclePosition.y)

        ATan((object 1 Y - object 2 Y) / (object 1 X - object 2 X)) +
    180}

void LedRGB::updateState(PoseF position, std::array<Circle> obstacles)
{
    // This function would update the current_state based on the robot's state

    // represent obstacles as radis or led position to be displayed
    size_t list_size = sizeof(obstacles) / sizeof(obstacles[0]);

    for (int i = 0; i < list_size; i++)
    {
        obstacles[i].x
            obstacles[i]
                .y
    }
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
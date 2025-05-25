#include "LedRGB.h"

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

void LedRGB::updateState()
{
    // This function would update the current_state based on the robot's state
    // and other conditions. It is a placeholder for the actual implementation.
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
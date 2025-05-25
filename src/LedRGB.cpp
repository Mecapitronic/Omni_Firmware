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
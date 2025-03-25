#include "LedRGB.h"

// uint8_t brightness)
void LedRGB::init(uint8_t numPixels, uint8_t data_pin)
{
    print("RGB initialisation of ", numPixels, " pixels");
    println(" on pin ", data_pin);
    m_numPixels = numPixels;

    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, numPixels);
    FastLED.setBrightness(RGB_BRIGHTNESS);
}

void LedRGB::update()
{
    leds[0] = CRGB::White;
    FastLED.show();
    delay(30);
    leds[0] = CRGB::Black;
    FastLED.show();
    delay(30);
}

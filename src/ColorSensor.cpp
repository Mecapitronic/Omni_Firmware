#include "ColorSensor.h"

using namespace Printer;

namespace ColorSensor
{
    namespace
    {
        byte gammatable[256];
        CRGB color;
        bool printDebug = false;
    } // namespace

    /* Initialise with default values (int time = 2.4ms, gain = 1x) */
    // Adafruit_TCS34725 tcs = Adafruit_TCS34725();

    /* Initialise with specific int time and gain values */
    Adafruit_TCS34725 tcs =
        Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

    void Initialisation(void)
    {
        if (tcs.begin())
        {
            println("Found sensor");
        }
        else
        {
            println("No TCS34725 found ... check your connections");
        }

        for (int i = 0; i < 256; i++)
        {
            float x = i;
            x /= 255;
            x = pow(x, 2.5);
            x *= 255;
            gammatable[i] = x;
        }
    }

    void Update(void)
    {
        uint16_t clear, red, green, blue;
        float r, g, b;

        tcs.setInterrupt(false); // turn on LED
        tcs.getRawData(&red, &green, &blue, &clear);
        tcs.setInterrupt(true); // turn off LED

        // Make rgb measurement relative
        uint32_t sum = clear;
        r = red;
        r /= sum;
        g = green;
        g /= sum;
        b = blue;
        b /= sum;
        // Scale rgb to bytes
        r *= 256;
        g *= 256;
        b *= 256;

        color = CRGB(r, g, b);
        if (printDebug)
        {
            // Show color name
            printColorName(color);
            println("R: %d G: %d B: %d\n", int(r), int(g), int(b));
        }
    }

    CRGB GetColor()
    {
        return color;
    }

    void PrintDebug(bool _printDebug)
    {
        printDebug = _printDebug;
    }

    void printColorName(CRGB c)
    {
        // Do not use this as it is time-consuming !
        CHSV hsv = rgb2hsv_approximate(c);
        if (hsv.hue < 15)
        {
            println("Red");
        }
        else if (hsv.hue < 45)
        {
            println("Orange");
        }
        else if (hsv.hue < 90)
        {
            println("Yellow");
        }
        else if (hsv.hue < 150)
        {
            println("Green");
        }
        else if (hsv.hue < 210)
        {
            println("Cyan");
        }
        else if (hsv.hue < 270)
        {
            println("Blue");
        }
        else if (hsv.hue < 330)
        {
            println("Magenta");
        }
        else
        {
            println("Red");
        }
    }
} // namespace ColorSensor

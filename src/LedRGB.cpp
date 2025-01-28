#include "LedRGB.h"

void LedRGB::Initialisation()
{
    println("RGB initialisation");
    pixels = Adafruit_NeoPixel(numPixels, PIN_RGB_LED, NEO_GRB + NEO_KHZ800);
    pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
    pixels.setBrightness(RGB_BRIGHTNESS);
    pixels.clear();
}

void LedRGB::Update()
{
    static int i = 0;
    if (i > color.capacity())
        i = 0;
    pixels.fill(color[i++]);
    pixels.show();
    // println("RGB Fill");
}

void LedRGB::HandleCommand(Command cmd)
{
    // HandleCommand implementation
}

void LedRGB::PrintCommandHelp()
{
    // PrintCommandHelp implementation
}

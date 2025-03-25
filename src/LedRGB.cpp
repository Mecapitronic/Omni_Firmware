#include "LedRGB.h"

void LedRGB::Initialisation(uint8_t numPixels, uint8_t pin, uint8_t brightness)
{
    print("RGB initialisation of ", numPixels, " pixels");
    println(" on pin ", pin);
    pixels = Adafruit_NeoPixel(numPixels, pin, NEO_GRB + NEO_KHZ800);
    pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
    pixels.setBrightness(brightness);
    pixels.clear();
}

void LedRGB::Update()
{
    int step = 65535 / pixels.numPixels();
    current_hue += step;
    if (current_hue > 65535)
    {
        current_hue = 0;
    }
    pixels.rainbow(current_hue);
    pixels.show();
    println("LED Current hue ", current_hue);
}

void LedRGB::HandleCommand(Command cmd)
{
    // if (cmd.cmd == "RGB")
    //{
    //  RGB:0;0;0
    //    print("RGB : ", cmd.data[0]);
    //    print(" : ", cmd.data[1]);
    //    print(" : ", cmd.data[2]);
    //    println();
    //}
}

void LedRGB::PrintCommandHelp()
{
    Printer::println("RGB Command Help :");
    Printer::println("! No Command yet !");
}

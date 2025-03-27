#include "LedRGB.h"

void LedRGB::Initialisation(uint8_t numPixels, uint8_t pin, uint8_t brightness)
{
    print("RGB initialisation of ", numPixels, " pixels");
    println(" on pin ", pin);
    pixels = Adafruit_NeoPixel(numPixels, pin, NEO_GRB + NEO_KHZ800);
    pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
    pixels.setBrightness(brightness);
    pixels.clear();

    // turn off all pixels
    for (size_t i = 0; i < numPixels; i++)
    {
        pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    }
    pixels.show(); // Update the strip with the new pixel colors
}

void LedRGB::rainbowRing()
{
    int step = 65535 / pixels.numPixels();
    current_hue += step;
    if (current_hue > 65535)
    {
        current_hue = 0;
    }
    pixels.rainbow(current_hue);
    // println("LED Current hue ", current_hue);
}

void LedRGB::rollingColors()
{
    pixels.setPixelColor(current_state.time, pixels.ColorHSV(current_state.color));
    for (int i = 1; i < 5; i++)
    {
        // HSV: H is the color, S is the saturation, V is the brightness
        // we reduce the brightness and change a little bit the saturation
        // (which changes the percieved color) for the tail
        pixels.setPixelColor((current_state.time - i + pixels.numPixels()) % pixels.numPixels(), pixels.ColorHSV(current_state.color, 255 - i * 8, 255 - i * (255 / 5)));
    }
    // set the 6th pixel to black
    pixels.setPixelColor((current_state.time - 5 + pixels.numPixels()) % pixels.numPixels(), pixels.Color(0, 0, 0));

    current_state.time++;
    if (current_state.time >= pixels.numPixels())
    {
        current_state.time = 0;
        // change the color with a big gap. Reduce the number for a smoother transition
        current_state.color += 4096;
    }
}

void LedRGB::displayEnemies()
{
    // get the adversary positions
    std::vector<float32> adversaryPositions = getAdversaryPositions();
    // display the adversary positions
    for (int i = 0; i < adversaryPositions.size(); i++)
    {
        setColorWithPosition(adversaryPositions[i], 0xFF0000);
    }
}

void LedRGB::Update()
{
    // rainbow_ring();
    // displayEnemies();
    rollingColors();
    pixels.show();
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

void LedRGB::setColorWithPosition(float32 angle_rad, uint32_t color)
{
    // get the led number depending of the angle in rad as we have a led ring
    int led_num = int(angle_rad * pixels.numPixels() / (2 * PI));
    pixels.setPixelColor(led_num, color);
    print("LED ", led_num);
    println(" set to color ", color);
}

std::vector<float32> LedRGB::getAdversaryPositions()
{
    std::vector<float32> adversaryPositions;

    // generate fake enemy positions
    for (int i = 0; i < 3; i++)
    {
        float32 position = i * PI / 3;
        print("Enemy position ", i);
        println(" : ", position);
        adversaryPositions.push_back(position);
    }

    return adversaryPositions;
}
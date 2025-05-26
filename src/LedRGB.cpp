#include "LedRGB.h"

using namespace Printer;

void LedRGB::Initialisation()
{
    print("RGB initialisation of ", NUM_LEDS, " pixels");
    println(" on pin ", PIN_WS2812_LED);

    ring_controller = &FastLED.addLeds<NEOPIXEL, PIN_WS2812_LED>(leds, NUM_LEDS);
    FastLED.setBrightness(RGB_BRIGHTNESS);
}

// This function would update the current state based on the robot's state
void LedRGB::updateState(PoseF position, std::array<Circle, 10> obstacles)
{   
    IHM::team == Team::Jaune ? team_color = CRGB::Gold : team_color = CRGB::DodgerBlue;

    if (Match::matchState == State::MATCH_RUN) {
        time_led = (NUM_LEDS  * Match::getMatchTimeSec()) / Match::time_end_match;
    }
    else
    {
        time_led++;
        if (time_led >= NUM_LEDS)
        {
            time_led = 0;
        }   
    }
    println("LED: Updating state with time_led: ",time_led );
}


void LedRGB::update()
{

    // si le bouton d'arrêt d'urgence est enclenché on voit rouge
    if (IHM::bauReady == 0) {
        emergencyStop();
        return;
    }

    // si le match n'est pas démarré on affiche la couleur de l'équipe
    if (Match::matchState == State::MATCH_WAIT)
    {
        for (int i = 0; i < NUM_LEDS; i++)
        {
            leds[i] = team_color; // Set all LEDs to team color
        }
        FastLED.show(); // Update the LEDs to reflect the changes
        return;
    }
    // // diminue l'intensité des LEDs si le match n'est pas en cours
    // if (Match::matchState != State::MATCH_RUN)
    // {
    //     FastLED.setBrightness(RGB_BRIGHTNESS / 2); // Dim the LEDs
    // }
    // else
    // {
    //     FastLED.setBrightness(RGB_BRIGHTNESS); // Set brightness to normal
    // }
    
    // update led ring display according to current robot state
    for (int i = 0; i < NUM_LEDS; i++)
    {
        // set the time as filled green leds from start to time_led
        if (i == time_led)
        {
            leds[i] = CRGB::Green;
        }
        else
        {
            // Default color for remaining LEDs
            leds[i] = CRGB::Black;
        }
        // if (i < obstacles.size())
        // {
        //     // Set color based on obstacle position
        //     leds[i] = CRGB::Violet;
        // }
        // else if (i < adversaries.size() + obstacles.size())
        // {
        //     // Set color based on adversary position
        //     leds[i] = CRGB::Red; // Example: Blue for adversaries
        // }

    }
    FastLED.show(); // Update the LEDs to reflect the changes   
}

int LedRGB::obstacleRelativePosition(PoseF robotPosition, Point obstaclePosition) {
    // get angle between h and the vector formed by robotposition x,y and obstaclePosition x,y

//     DirectionFromPoint(robotPosition.x, robotPosition.y, obstaclePosition.x, obstaclePosition.y)

//     atan2((object 1 Y - object 2 Y) / (object 1 X - object 2 X)) + 180}

}

int LedRGB::lidarPositionToLedNumber(float position, float min, float max)
{
    // Convert the position to a value between 0 and NUM_LEDS
    auto ledNumber = static_cast<int>((position - min) / (max - min) * NUM_LEDS);
    // Ensure the ledNumber is within bounds
    if (ledNumber < 0)
        ledNumber = 0;
    else if (ledNumber >= NUM_LEDS)
        ledNumber = NUM_LEDS - 1;
    return ledNumber;
}

// ANIMATIONS

void LedRGB::loader()
{
    for (int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = CRGB::Black; // Clear all LEDs
        FastLED.show();
        delay(50);
    }
    for (int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = CHSV(current_hue++, 255, 255); // Cycle through colors
        FastLED.show();
        delay(50);
    }
}

void LedRGB::emergencyStop()
{
    fill_solid(leds, NUM_LEDS, CRGB::Red);
    ring_controller->showLeds(RGB_BRIGHTNESS);

    for (uint8_t i = RGB_BRIGHTNESS ; i > 0; i--)
    {
        ring_controller->showLeds(i);
        delay(30);
    }
    ring_controller->showLeds(0); // Turn off all LEDs
}
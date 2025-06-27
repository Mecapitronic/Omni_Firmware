#ifndef LEDRGB_H
#define LEDRGB_H

#include <FastLED.h>

#include "ESP32_Helper.h"
#include "GeoMathTools.h"
#include "IHM.h"
#include "Match.h"
#include "PathPlanning/Obstacle.h"
#include "Structure.h"
#include "pins.h"

#define NUM_LEDS 24

#define TRANSITION_DELAY_MS 500
#define TRANSITION_STEPS 100
#define RING_BRIGHTNESS 255 // Brightness of the LED ring

using namespace Printer;

/**
 * @brief Gère l'affichage des LEDs RGB du robot sur un anneau de 24 LEDs.
 *
 * affiche la couleur de l'équipe en fond très léger (bleu ou jaune)
 * affiche la position des adversaires en rouge
 * affiche la position des obstacles en violet
 * affiche le temps écoulé depuis le début du match en vert
 * lors d'un arrêt d'urgence, affiche toutes les LEDs en rouge clignotant
 *
 * L'ensemble des fonctions sont implémentées de manière non-bloquante
 * pour permettre une utilisation en temps réel dans le robot.
 * Les fonctions d'affichage sont appelées par la fonction update() qui est
 * appelée régulièrement par le task Led.
 * Les fonctions d'initialisation sont appelées par la fonction Initialisation()
 * qui est appelée au démarrage du robot.
 *
 * Pour l'orientation
 * la LED 0 est devant le robot. Les LED sont numérotées dans le sens horaire.
 * La LED 1 est sa droite et la LED 24 est à gauche
 *
 * Le 0 du robot et à droite à 90°, c'est le 0 trigonométrique. Les angles sont
 * compris dans [ -pi; pi ] en partant de ce 0.
 * Il faut inverser le sens de rotations.
 *
 * @todo TODO add red leds for errors (like otos not connected)
 * we could have an error counters and increment or decrement outside ledrgb
 * and turn on the number of leds equivalent to the number of errors
 */
class LedRGB
{

public:
    /**
     * @brief initialise the RGB LED ring, sets the brightness and prepares the ring
     * controller.
     *
     * @param robotPosition pointer to robot position in order to display the obstacles
     * and adversaries positions relative to the robot
     */
    void Initialisation(Robot *robotPosition);

    /**
     * @brief update led ring display according to current robot state.
     * @details update the current robot state from value in other modules
     * get the adversaries and obstacles positions, robot position, robot state.
     * Takes into account: enemies position, obstacles position, robot position, robot
     * state. The informaitons are stored in robot_state private variables and translated
     * into colors and leds positions.
     *
     */
    void update();

    /**
     * @brief display a circular rainbow as a loading circle on the RGB LEDs ring
     * the rainbow is circling
     *
     * @return * void
     */
    void rainbow();


    void robotIsStarting();
    void displayTime();
    void displayObstacle();

    /**
     * @brief set all leds to red and glow smoothly
     * inline function
     */
    inline CRGB TwoColorsTransition(CRGB color1, CRGB color2);
    inline CRGB glowTwoColors(CRGB color1, CRGB color2);
    inline CRGB glowOneColor(CRGB color);

    void emergencyStopAtStart();

    /**
     * @brief convert a polar point to the corresponding LED number
     * @details takes the angle in radians starting from 0 trigo (on the right) and going
     * counter clockwise and gives the corresponding led number, starting from 0 in front
     * of the robot and going clockwise.
     *
     * @note l'angle 0 est à droite du robot, on doit donc le convertir pour que 0 soit en
     * haut on doit aussi inverser le sens pour suivre le sens horaire et non
     * trigonométrique
     *
     * @param angle the angle in radians starting from 0 trigo
     * @return uint8_t the LED number corresponding to the angle
     * @note The angle is normalized to be in the range [0, 2π] before conversion.
     */
    uint8_t directionToLedNumber(float angle);

private:
    // CRGB::HTMLColorCode timerMatchColor;
    CLEDController *ring_controller;  // Pointer to the FastLED controller for the ring
    // CRGB filling_color = CRGB::Black; // Couleur de fond
    CRGB leds[NUM_LEDS];              // Array to hold the colors of the LEDs
    // CRGB team_color = CRGB::Black;    // Default color for the team
    //  CRGB clock_color = CRGB::ForestGreen; // Color for the clock
    Robot *robot_position;                // Pointer to the robot position
    // Timeout matchClockTimer;              // Timer to display match time
    Timeout rotationTimer;                // Timer to rotate colors
    Timeout blendTimeOut;
    uint8_t blendAmount; // Amount of blending between two colors,
                         // that changes over time
    int8_t blendCoef;
    // uint8_t secondsCounter = 0; // permet d'afficher les secondes (1/2 rpm) pour la
    //  rotation de couleur ou d'arc en ciel
    // uint8_t match_time_led = 0; // numero de la led à allumer
    //  pour indiquer le temps de match écoulé
};

#endif
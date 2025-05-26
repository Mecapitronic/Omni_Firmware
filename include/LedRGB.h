#ifndef LEDRGB_H
#define LEDRGB_H

#include <FastLED.h>

#include "ESP32_Helper.h"
#include "GeoMathTools.h"
#include "IHM.h"
#include "Match.h"
#include "pins.h"
#include "Structure.h"

#define NUM_LEDS 24

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
 */
class LedRGB
{

public:
  void Initialisation();

  /**
   * @brief update the current robot state from value in other modules
   * get the adversaries and obstacles positions, robot position, robot state.
   * This function is called by the update() function.
   */
  void updateState(PoseF position, std::array<Circle, 10> obstacles);

  /**
   * @brief update led ring display according to current robot state.
   * @description Takes into account: enemies position, obstacles position, robot position, robot state.
   * The informaitons are stored in robot_state private variables and translated into colors
   * and leds positions.
   *
   */
  void update();

  /**
   * @brief display a loading circle on the RGB LEDs ring
   * turns all leds on one by one, then turns them off one by one, cycling through colors.
   *
   * @return * void
   */
  void loader();

  /**
   * @brief set all leds to red and glow smoothly
   * inline function
   */
  inline void emergencyStop();

  int lidarPositionToLedNumber(float position, float min, float max);
  int obstacleRelativePosition(PoseF robotPosition, Point obstaclePosition);

  inline void TwoColorsTransition(CRGB color1, CRGB color2);
  inline void glowTwoColors(CRGB color1, CRGB color2);
  inline void glowOneColor(CRGB color);
  bool isItTimeToChangeColor(uint32_t *time_led, uint32_t delta_ms = 1000);

private:
  CLEDController *ring_controller;
  CRGB leds[NUM_LEDS];
  uint32_t switch_color_timer = 0;
  uint8_t blendAmount = 0; // Amount of blending between two colors

  uint8_t current_hue = 0;
  CRGB team_color = CRGB::Black; // Default color for the team
  long time_led = 0;
  std::vector<float> obstacles;
  std::vector<float> adversaries;
};

#endif
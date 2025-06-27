#ifndef GEOMATHTOOLS_H
#define GEOMATHTOOLS_H

#include "ESP32_Helper.h"
#include "Structure.h"

#ifndef SQRT2
#define SQRT2 (1.41421356237) // SQRT(2) racine carrée de 2
#endif
#ifndef SQRT3_2
#define SQRT3_2 (0.86602540378) // SQRT(3)/2
#endif
#ifndef SQRT3
#define SQRT3 (1.73205080757) // SQRT(3)
#endif
#ifndef INV_SQRT3
#define INV_SQRT3 (0.57735026919) // 1/SQRT(3)
#endif

/**
 * @brief Approximation de distance euclidienne. Attention: dx et dy doivent être positifs
 *
 */
#define Approx_Distance(dx, dy)                                                          \
    ((dy < dx) ? (dx + (dy >> 2) + (dy >> 3)) : (dy + (dx >> 2) + (dx >> 3)))

/**
 * @brief  Norm of 2D vector given by dx and dy => magnitude or Euclidean distance
 *
 * @param dx
 * @param dy
 * @return float
 */
float Norm2D(float dx, float dy);

/**
 * @brief Direction in radians from position xA, yA towards position xB, yB
 *
 * @param xA
 * @param yA
 * @param xB
 * @param yB
 * @return float
 */
float DirectionFromPositions(float xA, float yA, float xB, float yB);

/**
 * @brief Direction en radians depuis le point A vers point B
 *
 * @param pA point A: origine
 * @param pB point B: point duquel on veut la direction
 * @return float direction en radian
 */
float DirectionFromPoints(PointF pA, PointF pB);

float DirectionFromPoints(Point pA, Point pB);

/**
 * @brief Euclidean distance from position xA, yA and position xB, yB
 *
 * @param xA
 * @param yA
 * @param xB
 * @param yB
 * @return float
 */
float DistanceBetweenPositions(float xA, float yA, float xB, float yB);

/**
 * @brief Euclidean distance between point A and point B
 *
 * @param pA point A
 * @param pB point B
 * @return float distance between A and B
 */
float DistanceBetweenPoints(PointF pA, PointF pB);

float DistanceBetweenPoints(Point pA, Point pB);

/**
 * @brief Normalisation of angle in radians between [-π, π]
 *
 * @param a_rad angle en radian
 * @return float angle normalisé entre -π et π
 */
float NormalizeAngle(float a_rad);

/**
 * @brief Normalisation of angle in radians between [0, 2*π]
 *
 * @param a_rad angle en radian
 * @return float angle normalisé entre 0 et 2*π
 */
float NormalizeAngle2PI(float a_rad);

#endif

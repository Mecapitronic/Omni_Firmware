#ifndef GEOMATHTOOLS_H
#define	GEOMATHTOOLS_H

/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Structure.h"
#include "ESP32_Helper.h"

/****************************************************************************************
 * Constantes
 ****************************************************************************************/
#ifndef SQRT2
#define SQRT2       (1.41421356237)  // SQRT(2) racine carrée de 2
#endif
#ifndef SQRT3_2
#define SQRT3_2     (0.86602540378)  // SQRT(3)/2
#endif
#ifndef SQRT3
#define SQRT3       (1.73205080757)  // SQRT(3)
#endif
#ifndef INV_SQRT3
#define INV_SQRT3   (0.57735026919)  // 1/SQRT(3)
#endif

/****************************************************************************************
 * Fonctions
 ****************************************************************************************/

// Approximation de distance euclidienne /!\ dx et dy doivent être positif
#define Approx_Distance(dx,dy)  ( (dy<dx) ? (dx+(dy>>2)+(dy>>3)) : (dy+(dx>>2)+(dx>>3)) )

float Norm2D(float dx, float dy);
float DirectionToPosition(float xA, float yA, float xB, float yB);
float DirectionToPoint(PointF pA, PointF pB);
float DistanceToPosition(float xA, float yA, float xB, float yB);
float DistanceToPoint(PointF pA, PointF pB);
float NormalizeAngle(float a_rad);

#endif


/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "GeoMathTools.h"

/****************************************************************************************
 * Norm of 2D vector given by dx and dy => magnitude or Euclidean distance
 ****************************************************************************************/
float Norm2D(float dx, float dy)
{
  return sqrtf(dx*dx + dy*dy);
}

/****************************************************************************************
 * Direction in radians from position xA, yA towards position xB, yB
 ****************************************************************************************/
float DirectionToPosition(float xA, float yA, float xB, float yB)
{
  return atan2f((yB - yA), (xB - xA));
}

/****************************************************************************************
 * Direction in radians from point A towards point B
 ****************************************************************************************/
float DirectionToPoint(PointF pA, PointF pB)
{
  return atan2f((pB.y - pA.y), (pB.x - pA.x));
}

/****************************************************************************************
 * Euclidean distance from position xA, yA and position xB, yB
 ****************************************************************************************/
float DistanceToPosition(float xA, float yA, float xB, float yB)
{
  return Norm2D((xB - xA), (yB - yA));
}

/****************************************************************************************
 * Euclidean distance from point A and point B
 ****************************************************************************************/
float DistanceToPoint(PointF pA, PointF pB)
{
  return Norm2D((pB.x - pA.x), (pB.y - pA.y));
}

/****************************************************************************************
 * Normalisation of angle in radians between [-π, π] 
 ****************************************************************************************/
float NormalizeAngle(float a_rad)
{
  while(a_rad > PI ) a_rad -= TWO_PI;
  while(a_rad < -PI ) a_rad += TWO_PI;
  return a_rad;
}

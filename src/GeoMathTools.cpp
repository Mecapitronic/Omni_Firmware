#include "GeoMathTools.h"

float Norm2D(float dx, float dy)
{
    return sqrtf(dx * dx + dy * dy);
}

float DirectionFromPositions(float xA, float yA, float xB, float yB)
{
    return atan2f((yB - yA), (xB - xA));
}

float DirectionFromPoints(PointF pA, PointF pB)
{
    return atan2f((pB.y - pA.y), (pB.x - pA.x));
}

float DistanceBetweenPositions(float xA, float yA, float xB, float yB)
{
    return Norm2D((xB - xA), (yB - yA));
}

float DistanceBetweenPoints(PointF pA, PointF pB)
{
    return Norm2D((pB.x - pA.x), (pB.y - pA.y));
}

float DistanceBetweenPoints(Point pA, Point pB)
{
    return Norm2D((pB.x - pA.x), (pB.y - pA.y));
}

float NormalizeAngle(float a_rad)
{
    while (a_rad > PI)
        a_rad -= TWO_PI;
    while (a_rad < -PI)
        a_rad += TWO_PI;
    return a_rad;
}

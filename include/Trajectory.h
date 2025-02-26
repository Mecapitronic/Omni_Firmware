#ifndef TRAJECTORY_H
#define TRAJECTORY_H

/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Structure.h"
#include "ESP32_Helper.h"
#include "GeoMathTools.h"
#include "Motion.h"
#include "main.h"

namespace Trajectory
{
    /****************************************************************************************
     * Functions
     ****************************************************************************************/
    void Initialisation(Motion *_linear, Motion *_angular, Robot *_robot);
    void Update();

    void TranslateToPosition(float x, float y, float speed_limit, float speed_final);
    void TranslateToPoint(PointF point, float speed_limit, float speed_final);

    void RotateToOrientation(float h_rad, float speed_limit, float speed_final);
    void RotateTowardsPosition(float x, float y, float speed_limit, float speed_final);
    void RotateTowardsPoint(PointF point, float speed_limit, float speed_final);

    void GoToPose(float x, float y, float h, float speed_limit, float speed_final);
}
#endif

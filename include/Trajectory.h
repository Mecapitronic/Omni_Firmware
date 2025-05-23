#ifndef TRAJECTORY_H
#define TRAJECTORY_H

/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Structure.h"
#include "ESP32_Helper.h"
#include "GeoMathTools.h"
#include "Motion.h"
#include "Timer.h"
namespace Trajectory
{
    /****************************************************************************************
     * Functions
     ****************************************************************************************/
    void Initialisation(Motion *_linear, Motion *_angular, Robot *_robot);
    void Update();
    void Reset();

    /**
     * @brief Pure translation to position x and y, with limit and final speed
     * @details Don't forget to disable the motion timer before calling this function and enable it after updating the motion parameters.
     *
     * @param x
     * @param y
     * @param speed_limit
     * @param speed_final
     */
    void TranslateToPosition(float x, float y, float speed_limit, float speed_final);
    void TranslateToPoint(PointF point, float speed_limit, float speed_final);

    /**
     * @brief Pure rotation to orientation h (in global field reference), with limit and final speed
     * @details Don't forget to disable the motion timer before calling this function and enable it after updating the motion parameters.
     *
     * @param h_rad
     * @param speed_limit
     * @param speed_final
     */
    void RotateToOrientation(float h_rad, float speed_limit, float speed_final);
    void RotateTowardsPosition(float x, float y, float speed_limit, float speed_final);
    void RotateTowardsPoint(PointF point, float speed_limit, float speed_final);

    /**
     * @brief Go to pose x, y and h, with limit and final speed
     * @details Don't forget to disable the motion timer before calling this function and enable it after updating the motion parameters.
     *
     * @param x
     * @param y
     * @param h
     * @param speed_limit
     * @param speed_final
     */
    void GoToPose(float x, float y, float h, float speed_limit, float speed_final);
    void GoToVertex(t_vertexID id, float speed_limit, float speed_final);
    void Navigate_To_Vertex(t_vertexID id, float speed_limit, float speed_final);
}
#endif

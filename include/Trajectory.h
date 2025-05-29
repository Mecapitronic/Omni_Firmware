#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "ESP32_Helper.h"
#include "GeoMathTools.h"
#include "Motion.h"
#include "PathPlanning/Mapping.h"
#include "PathPlanning/PathFinding.h"
#include "Structure.h"
#include "Timer.h"

namespace Trajectory
{
    // distance seuil à partir de laquelle on considère qu'on est arrivé à un point donné
    // (tolérance)
    constexpr float ArrivalTriggerDistance = 20.0;
    // angle seuil à partir de laquelle on considère qu'on est arrivé à un point donné
    // (tolérance)
    constexpr float ArrivalTriggerAngle = radians(5.0);

    constexpr uint16_t OBSTACLE_TOO_CLOSE =
        20; // distance en dessous de laquelle on robot adversaire est considéré trop prêt
            // de nous = risque de collision


    PoseF GetTarget();


    /**
     * @brief Give the polar position of an object relative to the robot position and
     * direction
     * @details la direction est donnée en radians compris entre -pi et +pi
     * L'angle est donne la position de l'obstacle relative au robot. l'origine de l'angle
     * est le 0 trigonométrique.
     *
     * @param obstacle the object we want to relative position from
     * @return PolarPoint the relative (to the robot) position and
     * direction of the object in degrees from 0 trigo and between [-pi, pi]
     */
    PolarPoint CartesianToRelativePolar(Point obstacle);
    PolarPoint CartesianToRelativePolar(float x, float y);

    /**
     * @brief TODO
     *
     * @param obstacle
     * @return true
     * @return false
     */
    bool isTheObstacleToClose(Circle obstacle);

    /**
     * @brief detecte si on obstacle se trouve devant la direction actuelle du robot
     * @details défini un cone de 30° autour de la direction (cone de tolérance)
     * ~ les calculs sont effectués en [-180;180] ~
     *
     * @param current_direction la direction du robot (h) en radian
     * @return true si on obstacle se trouve sur le chemin (dans le cone de tolérance)
     * @return false si aucun obstacle n'est détecté devant le robot
     */
    bool isThereAnObstacleInFrontOfMe(float current_direction);
    void FallBackIfWeAreWithinObstacleLimits();

    /**
     * @brief Trajectory initialisation
     *
     * @param _linear
     * @param _angular
     * @param _robot
     */
    void Initialisation(Motion *_linear, Motion *_angular, Robot *_robot);

    /**
     * @brief Update linear direction and positions errors: Must be called on every speed
     * control loop => motion timer
     *
     */
    void Update();
    void Reset();

    /**
     * @brief Pure translation to position x and y, with limit and final speed
     * @details Don't forget to disable the motion timer before calling this function and
     * enable it after updating the motion parameters.
     *
     * @param x
     * @param y
     * @param speed_limit
     * @param speed_final
     */
    void TranslateToPosition(float x, float y, float speed_limit, float speed_final);

    /**
     * @brief Pure translation to point, with limit and final speed
     *
     * @param point
     * @param speed_limit
     * @param speed_final
     */
    void TranslateToPoint(PointF point, float speed_limit, float speed_final);

    /**
     * @brief Pure rotation to orientation h (in global field reference), with limit and
     * final speed
     * @details Don't forget to disable the motion timer before calling this function and
     * enable it after updating the motion parameters.
     *
     * @param h_rad
     * @param speed_limit
     * @param speed_final
     */
    void RotateToOrientation(float h_rad, float speed_limit, float speed_final);

    /**
     * @brief Pure rotation in direction of a position x and y, with limit and final speed
     *
     * @param x
     * @param y
     * @param speed_limit
     * @param speed_final
     */
    void RotateTowardsPosition(float x, float y, float speed_limit, float speed_final);

    /**
     * @brief Pure rotation in direction of a point, with limit and final speed
     *
     * @param point
     * @param speed_limit
     * @param speed_final
     */
    void RotateTowardsPoint(PointF point, float speed_limit, float speed_final);

    /**
     * @brief Go to pose x, y and h, with limit and final speed
     *
     * @param x
     * @param y
     * @param h
     * @param speed_limit
     * @param speed_final
     */
    void GoToPose(float x, float y, float h, float speed_limit, float speed_final);
    /**
     * @brief Go to given vertex id with speed limit and final speed
     *
     * @param id
     * @param speed_limit
     * @param speed_final
     */
    void GoToVertex(t_vertexID id, float speed_limit, float speed_final);

    /**
     * @brief Go to vertex id with speed limit and final speed using a Path planning
     * algorithm
     *
     * @param id
     * @param speed_limit
     * @param speed_final
     */
    void Navigate_To_Vertex(t_vertexID id, float speed_limit, float speed_final);

    bool WaitRobotArrived();
} // namespace Trajectory
#endif

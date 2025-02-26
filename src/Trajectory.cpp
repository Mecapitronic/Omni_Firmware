/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Trajectory.h"
using namespace Printer;

namespace Trajectory
{  
  /****************************************************************************************
   * Variables
   ****************************************************************************************/
  namespace
  {
    Motion * linear;
    Motion * angular;
    Robot * robot;

    // current target pose to go
    PoseF target;
  }

  /****************************************************************************************
   * Initialisation 
   ****************************************************************************************/
  void Initialisation(Motion *_linear, Motion *_angular, Robot *_robot)
  {
    // get a reference to the original object
    linear = _linear;
    angular = _angular;
    robot = _robot;

    target.x = robot->x;
    target.y = robot->y;
    target.h = robot->h;
  }

  /****************************************************************************************
   * Update linear direction and positions errors : Must be called on every speed control loop => motion timer 
   ****************************************************************************************/
  void Update()
  {
    // Direction of the linear motion vector, in local robot reference
    linear->direction = NormalizeAngle( DirectionToPosition(robot->x, robot->y, target.x, target.y) - robot->h);

    // Magnitude of the linear motion vector => Distance error
    linear->position_error = DistanceToPosition(robot->x, robot->y, target.x, target.y);

    // Orientation error
    angular->position_error = NormalizeAngle(target.h - robot->h);
  }

  /****************************************************************************************
   * Pure translation to position x and y, with limit and final speed
   ****************************************************************************************/
  void TranslateToPosition(float x, float y, float speed_limit, float speed_final)
  {
    // Disable motion timer before updating motion parameters
    DisableTimerMotion();

    // Update target position
    target.x = x;
    target.y = y;

    // Update linear speeds
    linear->speed_limit = speed_limit;
    linear->speed_final = speed_final;
    
    EnableTimerMotion();
  }

  /****************************************************************************************
   * Pure translation to point, with limit and final speed
   ****************************************************************************************/
  void TranslateToPoint(PointF point, float speed_limit, float speed_final)
  {
    TranslateToPosition(point.x, point.y, speed_limit, speed_final);
  }

  /****************************************************************************************
   * Pure rotation to orientation h (in global field reference), with limit and final speed
   ****************************************************************************************/
  void RotateToOrientation(float h_rad, float speed_limit, float speed_final)
  {
    // Disable motion timer before updating motion parameters
    DisableTimerMotion();

    // Update target orientation
    target.h = h_rad;

    // Update linear speeds
    angular->speed_limit = speed_limit;
    angular->speed_final = speed_final;
    
    EnableTimerMotion();
  }

  /****************************************************************************************
   * Pure rotation in direction of a position x and y, with limit and final speed
   ****************************************************************************************/
  void RotateTowardsPosition(float x, float y, float speed_limit, float speed_final)
  {
    RotateToOrientation( DirectionToPosition(robot->x, robot->y, x, y), speed_limit, speed_final);
  }

  /****************************************************************************************
   * Pure rotation in direction of a point, with limit and final speed
   ****************************************************************************************/
  void RotateTowardsPoint(PointF point, float speed_limit, float speed_final)
  {
    RotateTowardsPosition(point.x, point.y, speed_limit, speed_final);
  }

  /****************************************************************************************
   * Go to pose x, y and h, with limit and final speed
   ****************************************************************************************/
  void GoToPose(float x, float y, float h, float speed_limit, float speed_final)
  {
    // Disable motion timer before updating motion parameters
    DisableTimerMotion();

    // Update target position
    target.x = x;
    target.y = y;
    target.h = h;

  // TODO: adapter vitesse de rotation selon distance : vitesse angular = angle * Vitesse linear / distance
      // if (linear.position_error != 0)
      // {
      //   angular.speed_max = fmin((fabsf(angular.position_error) * linear.speed_max) / fabsf(linear.position_error), speed_ang_rads_max);
      // }

    // Update linear speeds
    linear->speed_limit = speed_limit;
    linear->speed_final = speed_final;
    
    EnableTimerMotion();
  }
}
#include "Trajectory.h"
using namespace Printer;

namespace Trajectory
{
  // private members
  namespace
  {
    Motion *linear;
    Motion *angular;
    Robot *robot;

    // current target pose to go
    PoseF target;
    // current target vertex ID to go
    t_vertexID target_vertex = 0;
  }

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

  void Update()
  {
    if (target_vertex != 0)
    {
      Point p = Mapping::Get_Vertex_Point(target_vertex);
      if (DistanceBetweenPositions(robot->x, robot->y, p.x, p.y) < 10)
      {
        // If we are close enough to the vertex, stop the trajectory
        target_vertex = 0;
      }
      else
      {
        // If we are navigating to a vertex, update the target pose to the vertex position
        target.x = p.x;
        target.y = p.y;
        // target.h = robot->h; // Keep current orientation
      }
    }
    // Direction of the linear motion vector, in local robot reference
    linear->direction = NormalizeAngle(DirectionFromPositions(robot->x, robot->y, target.x, target.y) - robot->h);

    // Magnitude of the linear motion vector => Distance error
    linear->position_error = DistanceBetweenPositions(robot->x, robot->y, target.x, target.y);

    // Orientation error
    angular->position_error = NormalizeAngle(target.h - robot->h);
  }

  void Reset()
  {
    // Reset target pose to current robot pose
    target.x = robot->x;
    target.y = robot->y;
    target.h = robot->h;
    target_vertex = 0;

    // Reset linear and angular motion parameters
    linear->position_error = 0;
    angular->position_error = 0;

    linear->velocity_command = 0;
    angular->velocity_command = 0;

    linear->Stop();
    angular->Stop();
  }

  void TranslateToPosition(float x, float y, float speed_limit, float speed_final)
  {
    // Update target position
    target.x = x;
    target.y = y;
    target_vertex = 0;

    // Update linear speeds
    linear->speed_limit = speed_limit;
    linear->speed_final = speed_final;
  }

  void TranslateToPoint(PointF point, float speed_limit, float speed_final)
  {
    TranslateToPosition(point.x, point.y, speed_limit, speed_final);
  }

  void RotateToOrientation(float h_rad, float speed_limit, float speed_final)
  {
    // Update target orientation
    target.h = h_rad;
    target_vertex = 0;

    // Update linear speeds
    angular->speed_limit = speed_limit;
    angular->speed_final = speed_final;
  }

  void RotateTowardsPosition(float x, float y, float speed_limit, float speed_final)
  {
    RotateToOrientation(DirectionFromPositions(robot->x, robot->y, x, y), speed_limit, speed_final);
  }

  void RotateTowardsPoint(PointF point, float speed_limit, float speed_final)
  {
    RotateTowardsPosition(point.x, point.y, speed_limit, speed_final);
  }

  void GoToPose(float x, float y, float h, float speed_limit, float speed_final)
  {
    // Update target position
    target.x = x;
    target.y = y;
    target.h = h;
    target_vertex = 0;
    // TODO: adapter vitesse de rotation selon distance : vitesse angular = angle * Vitesse linear / distance
    // if (linear.position_error != 0)
    // {
    //   angular.speed_max = fmin((fabsf(angular.position_error) * linear.speed_max) / fabsf(linear.position_error), speed_ang_rads_max);
    // }

    // Update linear speeds
    linear->speed_limit = speed_limit;
    linear->speed_final = speed_final;  
  }

  void GoToVertex(t_vertexID id, float speed_limit, float speed_final)
  {
    target_vertex = id;
    linear->speed_limit = speed_limit;
    linear->speed_final = speed_final;
  }

  void Navigate_To_Vertex(t_vertexID id, float speed_limit, float speed_final)
  {
    if (PathFinding::PathFinding((int16_t)robot->x, (int16_t)robot->y, id) && PathFinding::solution.size() > 0)
    {
      if (target_vertex != PathFinding::solution.front() || target_vertex == 0)
      {
        print("Path to vertex ", id);
        println(" found with ", PathFinding::solution.size(), " points.");
        PathFinding::ListVertexPrint(PathFinding::solution, "solution");
        println("Next point: ", PathFinding::solution.front());
        target_vertex = PathFinding::solution.front();
      }
    }
    else
    {
      println("Path to vertex ", id, " not found.");
      target_vertex = 0;
    }
    linear->speed_limit = speed_limit;
    linear->speed_final = speed_final;
  }
}
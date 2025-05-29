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
        // timer d'attente lorsque l'on detecte un obstacle
        // avant de reprendre notre course
        Timeout collision_prevention_timer;
        // permet d'arrêter le robot lors de la detection d'un obstacle
        // sans flinguer le path finding
        bool pathFindingOnHold = false;

    } // namespace

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

    PoseF GetTarget()
    {
        return target;
    }

    PolarPoint CartesianToPolar(Point point)
    {
        return CartesianToPolar(point.x, point.y);
    }

    PolarPoint CartesianToPolar(float x, float y)
    {
        float distance =
            sqrt((x - robot->x) * (x - robot->x) + (y - robot->y) * (y - robot->y));
        float angle = atan2(y - robot->y, x - robot->x);

        return PolarPoint(angle, distance, 0);
    }

    PolarPoint CartesianToRelativePolar(Point obstacle)
    {
        return CartesianToRelativePolar(obstacle.x, obstacle.y);
    }

    PolarPoint CartesianToRelativePolar(float x, float y)
    {
        PolarPoint point = CartesianToPolar(x, y);
        point.angle = NormalizeAngle(point.angle - robot->h);
        return point;
    }

    bool isTheObstacleToClose(Circle obstacle)
    {
        return DistanceBetweenPoints(robot->GetPoint(), obstacle.p)
               < OBSTACLE_TOO_CLOSE + ROBOT_RADIUS + ROBOT_MARGIN + OBSTACLE_RADIUS
                     + OBSTACLE_MARGIN;
    }

    bool isThereAnObstacleInFrontOfMe()
    {
        for (auto obstacle : Obstacle::obstacle)
        {
            if (isTheObstacleToClose(obstacle))
            {
                PolarPoint adversary = CartesianToPolar(obstacle.p);
                if (abs(adversary.angle - linear->direction) < radians(20.0))
                {
                    return true;
                }
            }
        }
        return false;
    }

    void putOnHold()
    {
        pathFindingOnHold = true;
    }

    void resumePathFinding()
    {
        pathFindingOnHold = false;
    }

    void Update()
    {
        if (isThereAnObstacleInFrontOfMe())
        {
            // si on détecte un obstacle (surprise) devant nous lorsque l'on
            // avance on attend 2 secondes collision_prevention_timer.Start(2000);
            // on peut pas reset directement parce que le path finding croira qu'on est
            // arrivé et continuera le reste de la stratégie
            putOnHold();
        }
        else
        {
            resumePathFinding();
        }


        // Direction of the linear motion vector, in local robot reference
        linear->direction = CartesianToRelativePolar(target.x, target.y).angle;

        // Magnitude of the linear motion vector => Distance error
        linear->position_error =
            DistanceBetweenPositions(robot->x, robot->y, target.x, target.y);

        // Orientation error
        angular->position_error = NormalizeAngle(target.h - robot->h);
    }

    void Reset()
    {
        // Reset target pose to current robot pose
        target.x = robot->x;
        target.y = robot->y;
        target.h = robot->h;

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
        target.h = robot->h;

        // Update linear speeds
        linear->speed_limit = speed_limit;
        linear->speed_final = speed_final;
        WaitRobotArrived();
    }

    void TranslateToPoint(PointF point, float speed_limit, float speed_final)
    {
        TranslateToPosition(point.x, point.y, speed_limit, speed_final);
    }

    void RotateToOrientation(float h_rad, float speed_limit, float speed_final)
    {
        // Update target orientation
        target.x = robot->x;
        target.y = robot->y;
        target.h = h_rad;

        // Update linear speeds
        angular->speed_limit = speed_limit;
        angular->speed_final = speed_final;
        WaitRobotArrived();
    }

    void RotateTowardsPosition(float x, float y, float speed_limit, float speed_final)
    {
        RotateToOrientation(
            DirectionFromPositions(robot->x, robot->y, x, y), speed_limit, speed_final);
    }

    void RotateTowardsPoint(PointF point, float speed_limit, float speed_final)
    {
        RotateTowardsPosition(point.x, point.y, speed_limit, speed_final);
    }

    void GoToPose(float x, float y, float h)
    {
        // Update target position
        target.x = x;
        target.y = y;
        target.h = h;
        WaitRobotArrived();
    }

    void GoToPose(float x, float y, float h, float speed_limit, float speed_final)
    {
        // Update target position
        target.x = x;
        target.y = y;
        target.h = h;
        // TODO: adapter vitesse de rotation selon distance : vitesse angular = angle
        // * Vitesse linear / distance if (linear.position_error != 0)
        // {
        //   angular.speed_max = fmin((fabsf(angular.position_error) *
        //   linear.speed_max) / fabsf(linear.position_error), speed_ang_rads_max);
        // }

        // Update linear speeds
        linear->speed_limit = speed_limit;
        linear->speed_final = speed_final;
        WaitRobotArrived();
    }

    void GoToVertex(t_vertexID id, float speed_limit, float speed_final)
    {
        Point p = Mapping::Get_Vertex_Point(id);
        target.x = p.x;
        target.y = p.y;
        target.h = robot->h;
        linear->speed_limit = speed_limit;
        linear->speed_final = speed_final;
        WaitRobotArrived();
    }

    void FallBackIfWeAreWithinObstacleLimits()
    {
        for (auto obstacle : Obstacle::obstacle)
        {
            // si on est "dans" l'obstacle,  on recule de 10cm
            if (DistanceBetweenPoints(robot->GetPoint(), obstacle.p) < obstacle.r)
            {
                println("Obstacle too close, going back");
                // on recule de 10cm dans la direction opposée de l'obstacle
                PoseF linear_target = robot->GetPoseF();
                PolarPoint adversary = CartesianToRelativePolar(obstacle.p);
                linear_target.x -= 100 * cos(radians(adversary.angle + 180));
                linear_target.y -= 100 * sin(radians(adversary.angle + 180));

                // TODO: checker si il n'y a pas d'obstacle dans la direction opposée
                GoToPose(linear_target.x, linear_target.y, robot->h);
                return;
            }
        }
    }

    // Does not return until arrived
    void Navigate_To_Vertex(t_vertexID id, float speed_limit, float speed_final)
    {
        t_vertexID target_vertex = 0;
        Point p = Mapping::Get_Vertex_Point(id);
        // TODO: dans l'idéal on regarde uniquement les obstacles dans la moitié du
        // terrain où on est pour éviter le flicker de la trajectoire
        // TODO: checker si on est dans un obstacle avant path planning et reculer
        // dans la direction opposée de l'obstacle
        while (DistanceBetweenPositions(robot->x, robot->y, p.x, p.y)
               > ArrivalTriggerDistance)
        {
            PathFinding::PathFinding((int16_t)robot->x, (int16_t)robot->y, id);
            if (target_vertex != PathFinding::solution.front())
            {
                print("Path to vertex ", id);
                println(" found with ", PathFinding::solution.size(), " points.");
                PathFinding::ListVertexPrint(PathFinding::solution, "solution");
                println("Next point: ", PathFinding::solution.front());

                target_vertex = PathFinding::solution.front();
                GoToVertex(PathFinding::solution.front(), speed_limit, speed_final);
            }
            vTaskDelay(5);
        }
        println("End of Path Finding");
    }

    bool WaitRobotArrived()
    {
        while (pathFindingOnHold
               || DistanceBetweenPositions(robot->x, robot->y, target.x, target.y)
                      > ArrivalTriggerDistance
               || (NormalizeAngle(abs(robot->h - target.h)) > ArrivalTriggerAngle))
        {
            println("distance : ",
                    DistanceBetweenPositions(robot->x, robot->y, target.x, target.y));
            println("angle : ",
                    (float)(degrees(NormalizeAngle(abs(robot->h - target.h)))));
            delay(100);
        }

        // while (linear->isRunning || angular->isRunning)
        //{
        //     delay(1);
        // }
        return true;
    }
} // namespace Trajectory

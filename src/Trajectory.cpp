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

        // backup de la target courrante lorsqu'on s'arrête pour
        // éviter un robot (car on reset la target)
        PoseF pending_target;

        // permet d'arrêter le robot lors de la detection d'un obstacle
        // sans flinguer la traj
        bool trajOnHold = false;

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
        return DistanceBetweenPoints(robot->GetPoint(), obstacle.p) < OBSTACLE_TOO_CLOSE;
    }

    void UpdateAdversary()
    {
        // first update the adversary list
        for (size_t i = 0; i < MAX_OBSTACLE; i++)
        {
            if (Mapping::Is_NotNull_Circle(&Obstacle::obstacle[i]))
                Obstacle::adversary[i] = CartesianToPolar(Obstacle::obstacle[i].p);
            else
                Obstacle::adversary[i] = PolarPoint();
        }
    }

    bool isThereAnObstacleInFrontOfMe()
    {
        PoseF target_to_consider;
        if (trajOnHold)
        {
            target_to_consider = pending_target;
        }
        else
        {
            target_to_consider = Trajectory::GetTarget();
        }

        // check list
        for (size_t i = 0; i < MAX_OBSTACLE; i++)
        {
            if (Obstacle::adversary[i].distance != 0
                && isTheObstacleToClose(Obstacle::obstacle[i]))
            {
                float targetDirection = Trajectory::CartesianToPolar(target_to_consider.x,
                                                                     target_to_consider.y)
                                            .angle;
                if (abs(Obstacle::adversary[i].angle - targetDirection) < radians(60.0))
                {
                    return true;
                }
            }
        }
        return false;
    }

    void putOnHold()
    {
        trajOnHold = true;
    }

    void resumeTraj()
    {
        trajOnHold = false;
    }

    bool IsOnHold()
    {
        return trajOnHold;
    }

    void UpdateTrajectory()
    {
        UpdateAdversary();
        if (isThereAnObstacleInFrontOfMe())
        {
            putOnHold();
            if (pending_target == PoseF())
            {
                // si on détecte un obstacle (surprise) devant nous lorsque l'on
                // avance on attend 2 secondes collision_prevention_timer.Start(2000);
                pending_target = target;
            }
            Reset();
        }
        else
        {
            if (pending_target != PoseF())
            {
                // restore target
                target = pending_target;
                pending_target = PoseF();
                // collision_prevention_timer.Stop();
                resumeTraj();
            }
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

    void GoToPose(float x, float y, float h)
    {
        // Update target position
        target.x = x;
        target.y = y;
        target.h = h;
        WaitRobotArrived();
    }

    void TranslateToPosition(float x, float y, float speed_limit, float speed_final)
    {
        // Update linear speeds
        linear->speed_limit = speed_limit;
        linear->speed_final = speed_final;

        GoToPose(x, y, robot->h);
    }

    void TranslateToPositionWithoutWaiting(float x,
                                           float y,
                                           float speed_limit,
                                           float speed_final)
    {
        // Update linear speeds
        linear->speed_limit = speed_limit;
        linear->speed_final = speed_final;

        GoToPose(x, y, robot->h);
    }

    void TranslateToPoint(PointF point, float speed_limit, float speed_final)
    {
        TranslateToPosition(point.x, point.y, speed_limit, speed_final);
    }

    void RotateToOrientation(float h_rad, float speed_limit, float speed_final)
    {
        // Update linear speeds
        angular->speed_limit = speed_limit;
        angular->speed_final = speed_final;

        GoToPose(robot->x, robot->y, h_rad);
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

    void GoToPose(float x, float y, float h, float speed_limit, float speed_final)
    {
        // Update linear speeds
        linear->speed_limit = speed_limit;
        linear->speed_final = speed_final;

        GoToPose(x, y, h);
    }

    void GoToVertex(t_vertexID id, float speed_limit, float speed_final)
    {
        linear->speed_limit = speed_limit;
        linear->speed_final = speed_final;

        Point p = Mapping::Get_Vertex_Point(id);
        GoToPose(p.x, p.y, robot->h);
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
                linear_target.x -= 100 * cos(adversary.angle + PI);
                linear_target.y -= 100 * sin(adversary.angle + PI);

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

            if (PathFinding::PathFinding((int16_t)robot->x, (int16_t)robot->y, id)
                && PathFinding::solution.size() > 0
                && target_vertex != PathFinding::solution.front())
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
        while (trajOnHold
               || DistanceBetweenPositions(robot->x, robot->y, target.x, target.y)
                      > ArrivalTriggerDistance
               || (NormalizeAngle(abs(robot->h - target.h)) > ArrivalTriggerAngle))
        {
            // println("distance : ",
            //         DistanceBetweenPositions(robot->x, robot->y, target.x, target.y));
            // println("angle : ",
            //         (float)(degrees(NormalizeAngle(abs(robot->h - target.h)))));
            delay(100);
        }

        Debugger::WaitForAvailableSteps();
        // while (linear->isRunning || angular->isRunning)
        //{
        //     delay(1);
        // }
        return true;
    }
} // namespace Trajectory

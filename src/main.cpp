#include "main.h"

using namespace Printer;
using namespace IHM;
using namespace Hardware_Config;

/****************************************************************************************
 * Variables
 ****************************************************************************************/
Robot robot;
TimerThread timerMotion;

// LedRGB led_ring;
Motor motor;

Motion linear;
Motion angular;

void setup()
{
    // display state as soon as possible to show it is starting
    // led_ring.Initialisation();
    // delay(500); // display for 1/2 second

    ESP32_Helper::Initialisation();
    println("Robot Holonome Firmware");

    Hardware::Initialisation(false);
    Power::EnablePower();

    Match::SetNumPami(10);
    /*
    println("Board : ", String(ARDUINO_BOARD));
    print("Arduino Version : ", ESP_ARDUINO_VERSION_MAJOR);
    print(".", ESP_ARDUINO_VERSION_MINOR);
    println(".", ESP_ARDUINO_VERSION_PATCH);
    println("ESP IDF Version : ", String(esp_get_idf_version()));
    println("Temperature : ", temperatureRead(), " deg Celsius");
    println("Frequency CPU : ", getCpuFrequencyMhz(), " MHz");
    println();*/

    // ColorSensor::Initialisation();
    ServoAX12::ServoConfig servoConfig;
    servoConfig.ax12Id = 18;
    servoConfig.AddPosition(58, Hardware_Config::ServoPosition::Min);   // position départ
    servoConfig.AddPosition(58, Hardware_Config::ServoPosition::Pos1);  // Position basse
    servoConfig.AddPosition(110, Hardware_Config::ServoPosition::Pos2); // position juste au dessus des caisses
    servoConfig.AddPosition(150, Hardware_Config::ServoPosition::Pos3); // position au dessus d'une caisse sur la tranche
    servoConfig.AddPosition(290, Hardware_Config::ServoPosition::Max);  // Tout en haut
    ServoAX12::AddServo(Hardware_Config::ServoID::Up, "Up", servoConfig);

    servoConfig.ax12Id = 17;
    servoConfig.AddPosition(48, Hardware_Config::ServoPosition::Min);   // position départ
    servoConfig.AddPosition(60, Hardware_Config::ServoPosition::Pos1);   // position 
    servoConfig.AddPosition(200, Hardware_Config::ServoPosition::Pos2);  // position retournement
    servoConfig.AddPosition(240, Hardware_Config::ServoPosition::Pos3); // position de maintient des caisses
    servoConfig.AddPosition(270, Hardware_Config::ServoPosition::Max);  // position de prise des caisses
    ServoAX12::AddServo(Hardware_Config::ServoID::Front, "Fwd", servoConfig);

    Lidar::Initialisation(&robot);

    // led_ring.emergencyStopAtStart();
    //OTOS::Initialisation();

    // Init motors
    motor.Initialisation(Motor::OMNIDIRECTIONAL_3_MOTORS, CENTER_WHEEL_DISTANCE);

    // Init Motion
    // Linear max speed and acceleration
    linear.Initialisation(1500, 500);
    // Angular max speed and acceleration
    angular.Initialisation(radians(500), radians(500));
    // Init end position tolerance
    linear.SetMargin(1);           // 1 mm
    angular.SetMargin(radians(1)); // 1 deg

    // Init trajectory
    Trajectory::Initialisation(&linear, &angular, &robot);

    // Init Path Planning
    Mapping::Initialize_Map(IHM::team);
    Obstacle::Initialize_Obstacle();
    Mapping::Initialize_Passability_Graph();

    // Start Point
    Point start = Mapping::Get_Vertex_Point(1);
    println("Start Point: x=%d y=%d", start.x, start.y);
    // Initial pose
    OTOS::SetPose(start.x, start.y, radians(180));
    robot.SetPose(start.x, start.y, radians(180));
    Screen::SetPose(robot.GetPose());
    Trajectory::Reset();
    
    Mapping::Update_Start_Vertex((int16_t)robot.x, (int16_t)robot.y);
    Mapping::Update_Passability_Graph();

    // Create a timer => for motion
    timerMotion = TimerThread(timerMotionCallback,
                              "Timer Motion",
                              (1000 * Motion::dt_motion) / portTICK_PERIOD_MS);
    // we will start it after match begin
    //timerMotion.Start();

    // Put at least the 1 Tick delay, this is needed so the watchdog doesn't trigger
    TaskThread(TaskTeleplot, "TaskTeleplot", 10000, 5, 0);
    // TaskThread(TaskUpdate, "TaskUpdate", 10000, 15, 0);
    TaskThread(TaskHandleCommand, "TaskHandleCommand", 20000, 5, 0);
    TaskThread(TaskMatch, "TaskMatch", 20000, 15, 1);

    // Send to PC all the mapping data
    // ESP32_Helper::HandleCommand(Command("UpdateMapping"));
}

void loop()
{
    // HACK Vérifier qu'on n'utilise pas les serialEvent !!!
    // C:\Users\xxx\.platformio\packages\framework-arduinoespressif32\cores\esp32\main.cpp
    // https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/main.cpp
    vTaskDelete(NULL); // Supprime immédiatement le task Arduino "loop"
}

// TIMER 5ms => MOTION
// do NON BLOCKING stuff
void timerMotionCallback(TimerHandle_t xTimer)
{
    if (timerMotion.IsEnable())
    {
        timerMotion.Running(true);
        // Odometry is updated from Hardware task callback to avoid I2C conflicts.
        if (simulation)
        {
            float motor1_speed = motor.GetMotorSpeed(1);
            float motor2_speed = motor.GetMotorSpeed(2);
            float motor3_speed = motor.GetMotorSpeed(3);

            // Simulation Calcul Vitesse
            // Calcul des composantes x, y et angular à partir des vitesses des
            // moteurs
            float v_x_relatif = (motor1_speed + motor2_speed - 2 * motor3_speed) / 3;
            float v_y_relatif = (motor2_speed - motor1_speed) * INV_SQRT3;
            float v_ang = (-(motor1_speed + motor2_speed + motor3_speed)
                           / (3 * CENTER_WHEEL_DISTANCE));

            float theta = OTOS::position.h; // orientation actuelle du robot
            float v_x_global = v_x_relatif * cos(theta) - v_y_relatif * sin(theta);
            float v_y_global = v_x_relatif * sin(theta) + v_y_relatif * cos(theta);

            OTOS::acceleration.x = v_x_global - OTOS::velocity.x;
            OTOS::acceleration.y = v_y_global - OTOS::velocity.y;
            OTOS::acceleration.h = v_ang - OTOS::velocity.h;

            OTOS::velocity.x = v_x_global;
            OTOS::velocity.y = v_y_global;
            OTOS::velocity.h = v_ang;

            // Mise à jour des positions en fonction des vitesses
            OTOS::position.x += v_x_global * timerMotion.Period() / 1000;
            OTOS::position.y += v_y_global * timerMotion.Period() / 1000;
            OTOS::position.h += v_ang * timerMotion.Period() / 1000;
        }

        // Actual position update
        robot.SetPose(OTOS::position.x, OTOS::position.y, OTOS::position.h);
        
        // Update Screen
        Screen::SetPose(robot.GetPose());
        PoseF targetScreenF = Trajectory::GetTarget();
        Pose targetScreen = Pose(targetScreenF.x, targetScreenF.y, targetScreenF.h);
        Screen::SetTarget(targetScreen);

        // Actual velocity update, in global field reference
        linear.velocity_actual = Norm2D(OTOS::velocity.x, OTOS::velocity.y);
        angular.velocity_actual = OTOS::velocity.h;

        // Trajectory update => error update
        Trajectory::UpdateTrajectory();

        // TODO: adapter vitesse de rotation selon distance : vitesse angular = angle
        // * Vitesse linear / distance if (linear.position_error != 0)
        // {
        //   angular.speed_max = fmin((fabsf(angular.position_error) *
        //   linear.speed_max) / fabsf(linear.position_error), speed_ang_rads_max);
        // }

        // Temps pour chaque mouvement
        // éviter div/0
        // float t_lin = linear.position_error / fmax(linear.speed_max, 1e-3f);
        // float t_ang = angular.position_error / fmax(angular.speed_max, 1e-3f);

        // // Synchronisation : on limite la vitesse du plus rapide
        // if (t_lin > t_ang && t_lin > 0)
        //     angular.speed_limit =
        //         fmax(angular.position_error / t_lin, radians(5)); // min 5°/s
        // else if (t_ang > t_lin && t_ang > 0)
        //     linear.speed_limit =
        //         fmax(linear.position_error / t_ang, 10.0f); // min 10 mm/s

        // Motion update
        linear.UpdateMotion();
        angular.UpdateMotion();

        // Motor update => in local robot reference
        motor.Update(linear.velocity_command, linear.direction, angular.velocity_command);
    }
    timerMotion.Running(false);
}

void TaskTeleplot(void *pvParameters)
{
    int lastMatchTime = 0;
    println("Start TaskTeleplot");
    Timeout robotPosTimeOut, mapTimeOut;
    robotPosTimeOut.Start(300);
    mapTimeOut.Start(300);
    Chrono chrono("Teleplot", 1000);

    while (true)
    {
        chrono.Start();
        try
        {
            // ServoAX12::TeleplotAllPosition();
            if (robotPosTimeOut.IsTimeOut())
            {
                teleplot("Position", robot);
                teleplot("Orient", degrees(robot.h));
                // teleplot("Target", Trajectory::GetTarget());
                // teleplot("TargetOrient", degrees(Trajectory::GetTarget().h));
                // teleplot("linear.direction", degrees(linear.direction));
                // println(">linear.isRunning:", linear.isRunning);
                // println(">angular.isRunning:", angular.isRunning);

                // teleplot("Direction",
                //          degrees(Trajectory::CartesianToPolar(Trajectory::GetTarget().x,
                //                                               Trajectory::GetTarget().y)
                //                      .angle));

                //  println(">fixeScale:0:0;0:2000;3000:2000;3000:0;|xy");
                //  otos.Teleplot();
                //  linear.Teleplot("linear");
                //  angular.Teleplot("angular");
            }
            if (mapTimeOut.IsTimeOut())
            {
                Obstacle::PrintObstacleList();
                // println(">OnHold:", Trajectory::IsOnHold());
                // ServoAX12::TeleplotPosition();
                // Obstacle::PrintAdversaryList();
            }

            // Countdown
            // if (lastMatchTime != (int)(Match::getMatchTimeSec()))
            //{
            //    // println("Match Time : ", (int)(Match::getMatchTimeSec()));
            //    lastMatchTime = (int)(Match::getMatchTimeSec());
            //}
        }
        catch (const std::exception &e)
        {
            printError(e.what());
        }
        if (chrono.Check() && Chrono::print)
        {
            printChrono(chrono);
        }
        vTaskDelay(10);
    }
}

void TaskUpdate(void *pvParameters)
{
    println("Start TaskUpdate");
    Chrono chrono("Update", 1000);
    while (true)
    {
        chrono.Start();
        try
        {
            // led_ring.update();
            //  ColorSensor::Update();
        }
        catch (const std::exception &e)
        {
            printError(e.what());
        }
        if (chrono.Check() && Chrono::print)
        {
            printChrono(chrono);
        }
        vTaskDelay(10);
    }
}

void TaskHandleCommand(void *pvParameters)
{
    println("Start TaskHandleCommand");
    Chrono chrono("HandleCommand", 1000);
    while (true)
    {
        chrono.Start();
        try
        {
            if (ESP32_Helper::HasWaitingCommand())
            {
                Command cmd = ESP32_Helper::GetCommand();
                println("Received Command: %s", cmd.ToString());
                motor.HandleCommand(cmd);

                if (cmd.cmdEquals("Help"))
                {
                    motor.PrintCommandHelp();
                }
                else if(cmd.cmdEquals("Restart"))
                {
                    println("Restart !");
                    Match::matchState = Match::State::MATCH_BOOT;
                }
                else if (cmd.cmdEquals("GoToPose") && cmd.size == 3)
                {
                    // GoToPose:500;500;90
                    // GoToPose:500;500;0
                    // GoToPose:50;50;0
                    // GoToPose:0;0;45
                    // GoToPose:0;0;0
                    PoseF goTo = PoseF(cmd.data[0], cmd.data[1], radians(cmd.data[2]));
                    print("Robot go to x= %f", goTo.x);
                    print(" y= %f", goTo.y);
                    print(" h= %f", goTo.h);
                    println();
                    Trajectory::GoToPose(goTo.x, goTo.y, goTo.h, linear.speed_max, 0);
                }
                else if (cmd.cmdEquals("SetPose") && cmd.size == 3)
                {
                    println("Set Robot Pose");
                    // SetPose:2000:200:9000
                    // SetPose:500;500;0
                    // SetPose:1500;1000;0
                    Point otosInit = Point(cmd.data[0], cmd.data[1]);
                    int angle = cmd.data[2];
                    print("Robot set to x= %d y=%d h=%d", otosInit.x, otosInit.y, angle);
                    InitRobotOTOS(otosInit, radians(angle));
                }
                else if (cmd.cmdEquals("GetPose"))
                {
                    Pose pose = robot.GetPose();
                    println("Robot Pose: x= %d y= %d h= %d", pose.x, pose.y, degrees(pose.h));
                }
                else if (cmd.cmdEquals("UpdateMapping"))
                {
                    Mapping::Update_Start_Vertex((int16_t)robot.x, (int16_t)robot.y);
                    Mapping::Update_Passability_Graph();
                    Mapping::PrintVertexList();
                    Mapping::PrintSegmentList();
                    Mapping::PrintCircleList();
                    Obstacle::PrintObstacleList();
                    println("RobotRadius: %d", ROBOT_RADIUS);
                    println("RobotMargin: %d", ROBOT_MARGIN);
                }
                else if (cmd.cmdEquals("PF"))
                {
                    bool result = false;
                    // PathFinding
                    // PF:5
                    // PF:500:1000:5
                    if (cmd.size == 1)
                    {
                        result = PathFinding::PathFinding(
                            (int16_t)robot.x, (int16_t)robot.y, cmd.data[0]);
                    }
                    else if (cmd.size == 3)
                    {
                        result = PathFinding::PathFinding(
                            cmd.data[0], cmd.data[1], cmd.data[2]);
                    }
                    if (result)
                    {
                        println("PF Found");
                        for (auto &v : PathFinding::solution)
                        {
                            println("Vertex id: %d", v);
                        }
                    }
                    else
                    {
                        println("PF Not Found");
                    }
                }
                else if (cmd.cmdEquals("Nav") && cmd.size == 1)
                {
                    Trajectory::Navigate_To_Vertex(cmd.data[0], linear.speed_max, 0);
                }
                else if (cmd.cmdEquals("VertexList"))
                {
                    Mapping::PrintVertexList();
                }
                else if (cmd.cmdEquals("SegmentList"))
                {
                    Mapping::PrintSegmentList();
                }
                else if (cmd.cmdEquals("CircleList"))
                {
                    Mapping::PrintCircleList();
                }
                else if (cmd.cmdEquals("MappingList"))
                {
                    Mapping::PrintVertexList();
                    Mapping::PrintSegmentList();
                    Mapping::PrintCircleList();
                }
                else if (cmd.cmdEquals("ObstacleList"))
                {
                    Obstacle::PrintObstacleList();
                }
                else if (cmd.cmdEquals("AddObs") && cmd.size == 3)
                {
                    // AddObs:0:500:1000
                    int num = cmd.data[0];
                    Point p;
                    p.x = cmd.data[1];
                    p.y = cmd.data[2];
                    Obstacle::Add_Obstacle(num, p);
                    Mapping::Update_Passability_Obstacle();
                    Obstacle::PrintObstacleList();
                }
                else if (cmd.cmdEquals("RemoveObstacle") && cmd.size == 1)
                {
                    int num = cmd.data[0];
                    Obstacle::Add_Obstacle(num, {0, 0});
                    Mapping::Update_Passability_Obstacle();
                    Obstacle::PrintObstacleList();
                }
                else if (cmd.cmdEquals("ColorSensor") && cmd.size == 1)
                {
                    ColorSensor::PrintDebug(cmd.data[0]);
                }
            }
        }
        catch (const std::exception &e)
        {
            printError(e.what());
        }
        if (chrono.Check() && Chrono::print)
        {
            printChrono(chrono);
        }
        vTaskDelay(10); // Allow other tasks to run
    }
}

void InitRobotOTOS(Point pInit, float angle)
{

                timerMotion.WaitForDisable();
                // Initial pose
                OTOS::SetPose(pInit.x, pInit.y, angle);
                robot.SetPose(pInit.x, pInit.y, angle);
                Screen::SetPose(robot.GetPose());
                // Reset odometry
                Trajectory::Reset();
                //  Enable Motor & Servo Power
                Power::EnablePower();
                delay(100);
                timerMotion.Enable();
                timerMotion.Start();
}

void TaskMatch(void *pvParameters)
{
    println("Start TaskMatch");
    Chrono chrono("MainMatch", 10000);
    while (true)
    {
        chrono.Start();
        try
        {
            // Attente insertion de la tirette de démarrage
            if (Match::matchState == Match::State::MATCH_BOOT)
            {
                Power::EnablePower();
                ServoAX12::RepliHaut();
            }

            // En attente de retrait de la tirette pour démarrer le match
            if (Match::matchState == Match::State::MATCH_WAIT)
            {
                // Disable Motor & Servo Power
                Power::DisablePower();
                
                Mapping::Initialize_Map(IHM::team);
            }

            // Match en cours
            if (Match::matchState == Match::State::MATCH_RUN)
            {
                InitRobotOTOS(Mapping::Get_Vertex_Point(1), radians(180));

                Point p;
                

                // --------  1ere prise --------
                // prise vertex 2
                ServoAX12::PrePrise();
                
                p = Mapping::Get_Vertex_Point(2);
                Trajectory::GoToPose(p.x, p.y+50, radians(180), linear.speed_max, 0);
                Trajectory::GoToVertex(2, linear.speed_max, 0);

                // Tourner vers la prise
                // angle = 180;
                Trajectory::RotateToOrientation(radians(180), angular.speed_max, 0);

                // Prise
                p = Mapping::Get_Vertex_Point(2);
                Trajectory::GoToPose(p.x, p.y-100, radians(180), linear.speed_max, 0);

                ServoAX12::Prise();

                // --------  1ere dépose --------
                // dépose vertex 3
                // Aller à la dépose
                p = Mapping::Get_Vertex_Point(3);
                if(IHM::team == IHM::Team::Jaune)
                    Trajectory::GoToPose(p.x, p.y, radians(-90), linear.speed_max, 0);
                else
                    Trajectory::GoToPose(p.x, p.y, radians(90), linear.speed_max, 0);
                
                // on pousse les elements dans la case d'après
                if(IHM::team == IHM::Team::Jaune)
                    Trajectory::GoToPoseTimeout(p.x+500, p.y, radians(-90), linear.speed_max, 0, 5000);
                else
                    Trajectory::GoToPoseTimeout(p.x-500, p.y, radians(90), linear.speed_max, 0, 5000);

                // on revient à la 1ère dépose
                p = Mapping::Get_Vertex_Point(3);
                if(IHM::team == IHM::Team::Jaune)
                    Trajectory::GoToPose(p.x, p.y, radians(-90), linear.speed_max, 0);
                else
                    Trajectory::GoToPose(p.x, p.y, radians(90), linear.speed_max, 0);

                delay(1000);
                ServoAX12::Retourne();
                ServoAX12::Depose();
                delay(1000);
                
                
                // --------  2eme prise --------
                Trajectory::GoToVertex(5, linear.speed_max, 0);
                Trajectory::RotateToOrientation(radians(180), linear.speed_max, 0);
                p = Mapping::Get_Vertex_Point(4);
                Trajectory::GoToPose(p.x, p.y+50, radians(180), linear.speed_max, 0);
                Trajectory::GoToVertex(4, linear.speed_max, 0);
                
                // Tourner vers la prise
                // angle = 180;
                Trajectory::RotateToOrientation(radians(180), angular.speed_max, 0);
                
                // Prise
                p = Mapping::Get_Vertex_Point(4);
                Trajectory::GoToPose(p.x, p.y-100, radians(180), linear.speed_max, 0);

                ServoAX12::Prise();
                delay(1000);
/*
                if(IHM::team == IHM::Team::Jaune)
                Trajectory::GoToPoseTimeout(p.x-100, p.y-500, radians(180), linear.speed_max, 0,3000);
                else
                Trajectory::GoToPoseTimeout(p.x+100, p.y-500, radians(180), linear.speed_max, 0,3000);

                
                Point pInit1;
                if(IHM::team == IHM::Team::Jaune)
                    pInit1 = Point(126, 310);
                else
                    pInit1 = Point(3000-126, 310);

                InitRobotOTOS(pInit1, radians(180));
*/
                
                // sortie de la dépose
                if(IHM::team == IHM::Team::Jaune)
                    Trajectory::GoToPose(p.x+250, p.y-100, radians(180), linear.speed_max, 0);
                else
                    Trajectory::GoToPose(p.x-250, p.y-100, radians(180), linear.speed_max, 0);
                
                // --------  2eme dépose --------
                // dépose vertex 5
                // Aller à la dépose
                p = Mapping::Get_Vertex_Point(5);
                if(IHM::team == IHM::Team::Jaune)
                    Trajectory::GoToPose(p.x+100, p.y, radians(90), linear.speed_max, 0);
                else
                    Trajectory::GoToPose(p.x-100, p.y, radians(-90), linear.speed_max, 0);
                
                ServoAX12::Retourne();
                ServoAX12::Depose();
                // Monte au max pour etre au dessus de la bordure
                ServoAX12::SetServoPosition(Hardware_Config::ServoID::Up, Hardware_Config::ServoPosition::Max);
                while (ServoAX12::AreAllServoMoving())
                {
                    delay(10);
                }

                // on pousse legerement les caisses
                if(IHM::team == IHM::Team::Jaune)
                    Trajectory::GoToPoseTimeout(p.x+50, p.y, radians(90), linear.speed_max, 0, 2000);
                else
                    Trajectory::GoToPoseTimeout(p.x-50, p.y, radians(-90), linear.speed_max, 0, 2000);

                if(IHM::team == IHM::Team::Jaune)
                    Trajectory::GoToPose(p.x+150, p.y, radians(90), linear.speed_max, 0);
                else
                    Trajectory::GoToPose(p.x-150, p.y, radians(-90), linear.speed_max, 0);


                // --------  3eme prise --------
                // vertex 7
                p = Mapping::Get_Vertex_Point(7);
                if(IHM::team == IHM::Team::Jaune)
                    Trajectory::GoToPose(p.x-50, p.y, radians(-90), linear.speed_max, 0);
                else
                    Trajectory::GoToPose(p.x+50, p.y, radians(90), linear.speed_max, 0);
                
                ServoAX12::PrePrise();

                if(IHM::team == IHM::Team::Jaune)
                    Trajectory::GoToPose(p.x+100, p.y, radians(-90), linear.speed_max, 0);
                else
                    Trajectory::GoToPose(p.x-100, p.y, radians(90), linear.speed_max, 0);
                    

                ServoAX12::Prise();

                // ---------  3eme dépose --------
                // vertex 8                
                p = Mapping::Get_Vertex_Point(8);
                
                if(IHM::team == IHM::Team::Jaune)
                    Trajectory::GoToPose(p.x, p.y, radians(-90), linear.speed_max, 0);
                else
                    Trajectory::GoToPose(p.x, p.y, radians(90), linear.speed_max, 0);

                    ServoAX12::Retourne();
                ServoAX12::Depose();
                // Monte au max pour etre au dessus de la bordure
                ServoAX12::SetServoPosition(Hardware_Config::ServoID::Up, Hardware_Config::ServoPosition::Max);
                ServoAX12::WaitAllServo();

                if(IHM::team == IHM::Team::Jaune)
                    Trajectory::GoToPose(p.x-100, p.y, radians(-90), linear.speed_max, 0);
                else
                    Trajectory::GoToPose(p.x+100, p.y, radians(90), linear.speed_max, 0);


                // Curseur
                Trajectory::RotateToOrientation(radians(180), angular.speed_max/2, 0);
                delay(2000);
                ServoAX12::PrePrise();
                 ServoAX12::SetServoPosition(Hardware_Config::ServoID::Up, Hardware_Config::ServoPosition::Min);
               ServoAX12::WaitAllServo();
                delay(1000);
                p = Mapping::Get_Vertex_Point(9);
                    Trajectory::GoToPose(p.x, p.y-100, radians(180), linear.speed_max/2, 0);

                    ServoAX12::SetServoPosition(Hardware_Config::ServoID::Up, Hardware_Config::ServoPosition::Max);
               ServoAX12::WaitAllServo();
               delay(1000);
                // retour en arrière
                p = Mapping::Get_Vertex_Point(9);
                
                    Trajectory::GoToPose(p.x, p.y+200, radians(180), linear.speed_max, 0);

                    ServoAX12::RepliHaut();



                // --------  Retour en zone de départ --------

                // point d'attente vertex 6

                p = Mapping::Get_Vertex_Point(6);
                Trajectory::GoToPose(p.x, p.y, radians(0), linear.speed_max, 0);

                // Pos repli sans attendre
                ServoAX12::SetServoPosition(Hardware_Config::ServoID::Up, Hardware_Config::ServoPosition::Pos3);
                ServoAX12::SetServoPosition(Hardware_Config::ServoID::Front, Hardware_Config::ServoPosition::Pos1);

                // attente sortie des PAMI
                while(Match::getMatchTimeMs() < Match::time_start_match+8500)
                {
                    delay(10);
                }

                // rentrer devant la zone
                p = Mapping::Get_Vertex_Point(1);
                Trajectory::GoToPose(p.x, p.y-250, radians(0), linear.speed_max, 0);

                // rentrer dans la zone
                Trajectory::GoToVertex(1, linear.speed_max, 0);

                //rentrer encore plus dans la zone
                p = Mapping::Get_Vertex_Point(1);
                Trajectory::GoToPose(p.x, p.y+50, radians(0), linear.speed_max, 0);

                // Fin des actions
                Match::matchState = Match::State::MATCH_STOP;
            }

            // Arrêt du robot
            if (Match::matchState == Match::State::MATCH_STOP)
            {
                // Wait for end of match
            }

            // Fin du match
            if (Match::matchState == Match::State::MATCH_END)
            {
                // Disable Motor & Servo Power
                Power::DisablePower();
                IHM::useBlink = false;
                // Disable Motion timer
                timerMotion.WaitForDisable();
                motor.Update(0, 0, 0);
                // Wait for reset
                //if (IHM::switchMode == 0 && IHM::tirettePresent == 0)
                //    Match::matchState = Match::State::MATCH_BOOT;
            }
        }
        catch (const std::exception &e)
        {
            printError(e.what());
        }
        if (chrono.Check() && Chrono::print)
        {
            printChrono(chrono);
        }
        vTaskDelay(1);
    }
}

//**************************************************************************************************************************/
void functionChrono(int nbrLoop)
{
    unsigned long startChrono = micros();
    for (int i = 0; i < nbrLoop; i++)
    {
        // function or code to loop
    }
    unsigned long endChrono = micros();
    unsigned long deltaChrono = endChrono - startChrono;

    unsigned long chrono = deltaChrono / nbrLoop;
    print("Chrono from ", nbrLoop, " loop");
    print(" is : ", deltaChrono, " µs total");
    print(" = ", deltaChrono / 1000, " ms total.");
    print(" or ", chrono, " µs/func ");
    print(" = ", chrono / 1000, " ms/func.");
    println();
}

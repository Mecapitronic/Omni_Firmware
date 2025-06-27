#include "main.h"

/****************************************************************************************
 * Variables
 ****************************************************************************************/
TimerThread timerMotion;

LedRGB led_ring;
Motor motor;

Motion linear;
Motion angular;

Robot robot;

OpticalTrackingOdometrySensor otos;

void setup()
{
    // display state as soon as possible to show it is starting
    led_ring.Initialisation(&robot);
    delay(500); // display for 1/2 second

    pinMode(PIN_EN_MCU, OUTPUT);
    // digitalWrite(PIN_EN_MCU, LOW);
    //  We need it to init the servos
    digitalWrite(PIN_EN_MCU, HIGH);

    ESP32_Helper::Initialisation();
    if (!simulation)
        delay(3000);
    println("Board : ", String(ARDUINO_BOARD));
    print("Arduino Version : ", ESP_ARDUINO_VERSION_MAJOR);
    print(".", ESP_ARDUINO_VERSION_MINOR);
    println(".", ESP_ARDUINO_VERSION_PATCH);
    println("ESP IDF Version : ", String(esp_get_idf_version()));
    println("Temperature : ", temperatureRead(), " deg Celsius");
    println("Frequency CPU : ", getCpuFrequencyMhz(), " MHz");
    println();
    println("Robot Holonome Firmware");

    // Init IHM
    IHM::InitIHM();

    while (IHM::bauReady != 1)
    {
        IHM::UpdateBAU();
        led_ring.emergencyStopAtStart();
        vTaskDelay(1);
    }

    ServoAX12::Initialisation(simulation);

    Lidar::Initialisation(&robot);

    otos.Initialisation(simulation);
    // Init sensors
    while (!otos.IsConnected() && !simulation)
    {
        otos.Initialisation(simulation);
        led_ring.emergencyStopAtStart();
        vTaskDelay(1);
    }

    // Init motors
    motor.Initialisation(Motor::OMNIDIRECTIONAL_3_MOTORS, CENTER_WHEEL_DISTANCE);

    // Init Motion
    // Linear max speed 1500 mm/s and acceleration 1000 mm/s²
    linear.Initialisation(1000, 500);
    // Angular max speed 500 °/s and acceleration 1000 °/s²
    angular.Initialisation(radians(200), radians(500));
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
    Point start = Mapping::Get_Vertex_Point(3);
    // Initial pose
    otos.SetPose(start.x, start.y, radians(0));
    robot.SetPose(start.x, start.y, radians(0));
    Trajectory::Reset();
    
    Mapping::Update_Start_Vertex((int16_t)robot.x, (int16_t)robot.y);
    Mapping::Update_Passability_Graph();

    // Create a timer => for motion
    timerMotion = TimerThread(timerMotionCallback,
                              "Timer Motion",
                              (1000 * Motion::dt_motion) / portTICK_PERIOD_MS);
    timerMotion.Start();

    // Put at least the 1 Tick delay, this is needed so the watchdog doesn't trigger
    TaskThread(TaskTeleplot, "TaskTeleplot", 10000, 10, 0);
    TaskThread(TaskUpdate, "TaskUpdate", 10000, 15, 1);
    TaskThread(TaskHandleCommand, "TaskHandleCommand", 20000, 5, 1);
    TaskThread(TaskMatch, "TaskMatch", 20000, 10, 1);

    // Send to PC all the mapping data
    ESP32_Helper::HandleCommand(Command("UpdateMapping"));
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
        // Odometry Update
        if (!simulation)
        {
            otos.Update();
        }
        else
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

            float theta = otos.position.h; // orientation actuelle du robot
            float v_x_global = v_x_relatif * cos(theta) - v_y_relatif * sin(theta);
            float v_y_global = v_x_relatif * sin(theta) + v_y_relatif * cos(theta);

            otos.acceleration.x = v_x_global - otos.velocity.x;
            otos.acceleration.y = v_y_global - otos.velocity.y;
            otos.acceleration.h = v_ang - otos.velocity.h;

            otos.velocity.x = v_x_global;
            otos.velocity.y = v_y_global;
            otos.velocity.h = v_ang;

            // Mise à jour des positions en fonction des vitesses
            otos.position.x += v_x_global * timerMotion.Period() / 1000;
            otos.position.y += v_y_global * timerMotion.Period() / 1000;
            otos.position.h += v_ang * timerMotion.Period() / 1000;
        }

        // Actual position update
        robot.SetPose(otos.position.x, otos.position.y, otos.position.h);

        // Actual velocity update, in global field reference
        linear.velocity_actual = Norm2D(otos.velocity.x, otos.velocity.y);
        angular.velocity_actual = otos.velocity.h;

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
    Timeout robotPosTimeOut, mapTimeOut, ihmTimeOut;
    robotPosTimeOut.Start(50);
    mapTimeOut.Start(500);
    ihmTimeOut.Start(5000);
    Chrono chrono("Teleplot", 1000);

    while (true)
    {
        chrono.Start();
        try
        {
            if (robotPosTimeOut.IsTimeOut())
            {
                teleplot("Position", robot);
                teleplot("Orient", degrees(robot.h));
                teleplot("Target", Trajectory::GetTarget());
                teleplot("TargetOrient", degrees(Trajectory::GetTarget().h));
                teleplot("linear.direction", degrees(linear.direction));
                println(">linear.isRunning:", linear.isRunning);
                println(">angular.isRunning:", angular.isRunning);

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
                println(">OnHold:", Trajectory::IsOnHold());
                // ServoAX12::TeleplotPosition();
                // Obstacle::PrintAdversaryList();
            }
            if (ihmTimeOut.IsTimeOut())
            {
                //             Match::printMatch();
                //             IHM::PrintAll();
            }

            // Countdown
            if (lastMatchTime != (int)(Match::getMatchTimeSec()))
            {
                println("Match Time : ", (int)(Match::getMatchTimeSec()));
                lastMatchTime = (int)(Match::getMatchTimeSec());
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
            Match::updateMatch();
            IHM::UpdateBAU();
            IHM::Blink();
            led_ring.update();

            // take some time to update the servo, maybe move it elsewhere
            ServoAX12::Update();
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

                otos.HandleCommand(cmd);
                motor.HandleCommand(cmd);
                if (cmd.cmd.startsWith("AX12"))
                {
                    ServoAX12::HandleCommand(cmd);
                }

                if (cmd.cmd == "Help")
                {
                    otos.PrintCommandHelp();
                    motor.PrintCommandHelp();
                }
                else if (cmd.cmd == "GoToPose" && cmd.size == 3)
                {
                    // GoToPose:500;500;90
                    // GoToPose:500;500;0
                    // GoToPose:50;50;0
                    // GoToPose:0;0;45
                    // GoToPose:0;0;0
                    PoseF goTo = PoseF(cmd.data[0], cmd.data[1], radians(cmd.data[2]));
                    print("Robot go to x=", goTo.x);
                    print(" y=", goTo.y);
                    print(" h=", goTo.h);
                    println();
                    Trajectory::GoToPose(goTo.x, goTo.y, goTo.h, linear.speed_max, 0);
                }
                else if (cmd.cmd == "SetPose" && cmd.size == 3)
                {
                    // SetPose:2000:200:9000
                    // SetPose:500;500;0
                    // SetPose:1500;1000;0
                    timerMotion.WaitForDisable();
                    PoseF goTo = PoseF(cmd.data[0], cmd.data[1], radians(cmd.data[2]));
                    print("Robot set to x=", goTo.x);
                    print(" y=", goTo.y);
                    print(" h=", goTo.h);
                    println();
                    robot.SetPose(goTo.x, goTo.y, goTo.h);
                    otos.SetPose(robot.x, robot.y, robot.h);
                    Trajectory::Reset();
                    timerMotion.Enable();
                }
                else if (cmd.cmd == "UpdateMapping")
                {
                    Mapping::Update_Start_Vertex((int16_t)robot.x, (int16_t)robot.y);
                    Mapping::Update_Passability_Graph();
                    Mapping::PrintVertexList();
                    Mapping::PrintSegmentList();
                    Mapping::PrintCircleList();
                    Obstacle::PrintObstacleList();
                    println("RobotRadius:", ROBOT_RADIUS);
                    println("RobotMargin:", ROBOT_MARGIN);
                }
                else if (cmd.cmd == "PF")
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
                            println("Vertex id:", v);
                        }
                    }
                    else
                    {
                        print("PF Not Found");
                    }
                }
                else if (cmd.cmd == "Nav" && cmd.size == 1)
                {
                    Trajectory::Navigate_To_Vertex(cmd.data[0], linear.speed_max, 0);
                }
                else if (cmd.cmd == "VertexList")
                {
                    Mapping::PrintVertexList();
                }
                else if (cmd.cmd == "SegmentList")
                {
                    Mapping::PrintSegmentList();
                }
                else if (cmd.cmd == "CircleList")
                {
                    Mapping::PrintCircleList();
                }
                else if (cmd.cmd == "MappingList")
                {
                    Mapping::PrintVertexList();
                    Mapping::PrintSegmentList();
                    Mapping::PrintCircleList();
                }
                else if (cmd.cmd == "ObstacleList")
                {
                    Obstacle::PrintObstacleList();
                }
                else if (cmd.cmd == "AddObs" && cmd.size == 3)
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
                else if (cmd.cmd == "RemoveObstacle" && cmd.size == 1)
                {
                    int num = cmd.data[0];
                    Obstacle::Add_Obstacle(num, {0, 0});
                    Mapping::Update_Passability_Obstacle();
                    Obstacle::PrintObstacleList();
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

void TaskMatch(void *pvParameters)
{
    println("Start TaskMatch");
    Chrono chrono("Match", 1000);
    while (true)
    {
        chrono.Start();
        try
        {
            // Attente du démarrage du match par la tirette
            if (Match::matchState == State::MATCH_WAIT)
            {
                IHM::UpdateHMI();
                // Disable Motor & Servo Power in Match mode during waiting
                // if (IHM::switchMode == 1)
                // {
                //     // digitalWrite(PIN_EN_MCU, LOW);
                // }
                // else
                // {
                //     // digitalWrite(PIN_EN_MCU, HIGH);
                // }
                digitalWrite(PIN_EN_MCU, HIGH);
            }

            // Match en cours
            if (Match::matchState == State::MATCH_BEGIN)
            {
                // Enable Motor & Servo Power
                digitalWrite(PIN_EN_MCU, HIGH);
                timerMotion.WaitForDisable();
                Mapping::Initialize_Map(IHM::team);
                // Start Point
                Point p = Mapping::Get_Vertex_Point(3);
                // Initial pose
                otos.SetPose(p.x, p.y, radians(0));
                robot.SetPose(p.x, p.y, radians(0));
                // Reset odometry
                Trajectory::Reset();
                timerMotion.Enable();

                Match::matchState = State::MATCH_RUN;
                Match::printMatch();
            }

            // Démarrage du robot
            if (Match::matchState == State::MATCH_RUN)
            {
                Point p;
                // float angle = 0;
                //  Enable Motor & Servo Power
                digitalWrite(PIN_EN_MCU, HIGH);
                ServoAX12::Bas();
                ServoAX12::Prise();
                while (ServoAX12::AreAllServoMoving())
                {
                    delay(1);
                }


                // --------  1ere prise --------
                // Nav prise vertex 25
                Trajectory::Navigate_To_Vertex(25, linear.speed_max, 0);

                // Tourner vers la prise
                // angle = 0; //-88;
                Trajectory::RotateToOrientation(radians(0), angular.speed_max, 0);

                // Prise
                p = Mapping::Get_Vertex_Point(25);
                Trajectory::GoToPose(p.x, p.y + 100, radians(0), linear.speed_max, 0);
                Trajectory::GoToPose(
                    p.x + 30, p.y + 100, radians(0), linear.speed_max, 0);
                Trajectory::GoToPose(
                    p.x - 30, p.y + 100, radians(0), linear.speed_max, 0);


                // --------  1ere dépose --------
                // Nav dépose vertex 8
                Trajectory::Navigate_To_Vertex(8, linear.speed_max / 4, 0);

                // Tourner vers la dépose
                Trajectory::RotateToOrientation(radians(178), angular.speed_max / 5, 0);

                ServoAX12::Bas();
                while (ServoAX12::AreAllServoMoving())
                {
                    delay(1);
                }

                // Avancer vers la dépose
                p = Mapping::Get_Vertex_Point(3);
                Trajectory::TranslateToPosition(p.x, p.y, linear.speed_max / 4, 0);

                // Baisser le bras
                ServoAX12::Bas();
                while (ServoAX12::AreAllServoMoving())
                {
                    delay(1);
                }

                // Déposer les boites
                ServoAX12::Depose();
                while (ServoAX12::AreAllServoMoving())
                {
                    delay(1);
                }

                // reculer de la dépose
                Trajectory::TranslateToPosition(p.x, p.y + 150, linear.speed_max, 0);


                // --------  2eme prise --------
                ServoAX12::Prise();
                while (ServoAX12::AreAllServoMoving())
                {
                    delay(1);
                }
                ServoAX12::Bas();
                while (ServoAX12::AreAllServoMoving())
                {
                    delay(1);
                }

                // Pré rotation vers la prise
                // angle = 0; // 88;
                Trajectory::RotateToOrientation(radians(0), angular.speed_max, 0);

                // Nav prise vertex 21
                Trajectory::Navigate_To_Vertex(21, linear.speed_max, 0);

                // Tourner vers la prise
                Trajectory::RotateToOrientation(radians(0), angular.speed_max, 0);

                // Prise
                p = Mapping::Get_Vertex_Point(21);
                Trajectory::TranslateToPosition(p.x, p.y + 100, linear.speed_max / 4, 0);
                Trajectory::TranslateToPosition(
                    p.x + 30, p.y + 100, linear.speed_max / 4, 0);
                Trajectory::TranslateToPosition(
                    p.x - 30, p.y + 100, linear.speed_max / 4, 0);


                // --------  2eme dépose --------
                // Nav dépose vertex 8
                Trajectory::Navigate_To_Vertex(8, linear.speed_max / 4, 0);

                // Tourner vers la dépose
                Trajectory::RotateToOrientation(radians(178), angular.speed_max / 5, 0);

                // Monter le bras
                ServoAX12::Haut();
                while (ServoAX12::AreAllServoMoving())
                {
                    delay(1);
                }

                // Avancer vers la dépose
                p = Mapping::Get_Vertex_Point(3);
                Trajectory::TranslateToPosition(p.x, p.y, linear.speed_max / 4, 0);

                // Poser le bras
                ServoAX12::Mid();
                while (ServoAX12::AreAllServoMoving())
                {
                    delay(1);
                }

                // Déposer les boites
                ServoAX12::Depose();
                while (ServoAX12::AreAllServoMoving())
                {
                    delay(1);
                }

                // reculer de la dépose
                Trajectory::TranslateToPosition(p.x, p.y + 150, linear.speed_max, 0);

                // Descendre le bras
                ServoAX12::Bas();
                while (ServoAX12::AreAllServoMoving())
                {
                    delay(1);
                }
                ServoAX12::Prise();
                while (ServoAX12::AreAllServoMoving())
                {
                    delay(1);
                }

                // Tourner devant soi
                Trajectory::RotateToOrientation(radians(0), angular.speed_max, 0);

                // Restart in waiting State
                Match::resetMatch();
            }

            // Arrêt du robot
            if (Match::matchState == State::MATCH_STOP)
            {
                // Wait for end of match
            }

            // Fin du match
            if (Match::matchState == State::MATCH_END)
            {
                // Disable Motor & Servo Power
                digitalWrite(PIN_EN_MCU, LOW);
                IHM::useBlink = false;
                // Disable Motion timer
                timerMotion.WaitForDisable();
                motor.Update(0, 0, 0);
                ServoAX12::StopAllServo();
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
        vTaskDelay(10);
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

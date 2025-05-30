#include "Lidar.h"

using namespace Printer;

namespace Lidar
{
    namespace
    {
        unsigned char trame[7];
        uint16_t cursor = 0;
        Robot *robot;
    } // namespace

    void Initialisation(Robot *_robot)
    {
        println("Init Lidar");
        robot = _robot;
        SERIAL_LIDAR.setPins(RX_LIDAR, TX_LIDAR);
        SERIAL_LIDAR.setRxBufferSize(1024);
        SERIAL_LIDAR.setTxBufferSize(1024);
        SERIAL_LIDAR.begin(230400);
        TaskThread Task = TaskThread(TaskLidar, "TaskLidar", 20000, 10, 1);
    }

    void sendMatchStartTime(int startTime)
    {
        SERIAL_LIDAR.write('?');
        SERIAL_LIDAR.write((int)startTime % 256);
        SERIAL_LIDAR.write((int)startTime >> 8);
        // Ending char : '\n'
        SERIAL_LIDAR.write(0x0A);
    }

    void TaskLidar(void *pvParameters)
    {
        println("Start TaskLidar");
        Chrono chrono("Lidar", 1000);
        Timeout sendDataToLidarTimer;
        sendDataToLidarTimer.Start(20);
        while (true)
        {
            if (Match::matchState == State::MATCH_BEGIN)
            {
                // peut être qu'on va l'envoyer plusieurs fois,
                // on devrait checker si on l'a déjà envoyé avec un bool dédié
                sendMatchStartTime((int)Match::getStartTime());
            }

            chrono.Start();
            try
            {
                if (sendDataToLidarTimer.IsTimeOut())
                {
                    PoseF p = robot->GetPoseF();
                    // Starting char : '!'
                    SERIAL_LIDAR.write(0x21);

                    // Robot X
                    SERIAL_LIDAR.write((int)p.x % 256);
                    SERIAL_LIDAR.write((int)p.x >> 8);

                    // Robot Y
                    SERIAL_LIDAR.write((int)p.y % 256);
                    SERIAL_LIDAR.write((int)p.y >> 8);

                    // Robot Angle * 100
                    // Robot is -180 to +180 deg, lidar is 0 to 360 deg
                    int angle = (int)(degrees(p.h) * 100);
                    if (angle < 0)
                        angle += 360 * 100;
                    SERIAL_LIDAR.write(angle % 256);
                    SERIAL_LIDAR.write(angle >> 8);

                    // Ending char : '\n'
                    SERIAL_LIDAR.write(0x0A);
                    // println("Lidar sent : ", p);
                }

                while (SERIAL_LIDAR.available())
                {
                    char data = SERIAL_LIDAR.read();

                    // safety feature: check match start time recorded in lidar esp
                    // in case of reset
                    if (data == '?' && cursor == 0)
                    {
                        trame[cursor++] = data;
                    }

                    // classic lidar data
                    else if (data == 0x21 && cursor == 0) // start char ! for lidar data
                    {
                        trame[cursor++] = data;
                    }

                    // match start time received
                    else if (cursor > 0 && trame[0] == '?')
                    {
                        trame[cursor++] = data;
                        if (cursor >= 3)
                        {
                            // full packet receveid
                            if (data == 0x0A)
                            {
                                // we need to check the match start time received against
                                // ours
                                int received_match_start_time =
                                    trame[1] | (trame[2] << 8);
                                // 300 ms if difference is enough to detect a reset, 300
                                // is to take communication time into account
                                if (Match::getStartTime() - received_match_start_time
                                    > 300)
                                {

                                    // reset start time and go into match mode
                                    Match::startMatch();
                                    Match::setStartTime(received_match_start_time);

                                    print("Lidar match start time mismatch: ");
                                    print(received_match_start_time);
                                    print(" ours is: ");
                                    print(" and lidars is : ");
                                    println(Match::getStartTime());
                                }
                                else
                                {
                                    println("Lidar match start time ok: ",
                                            received_match_start_time);
                                }
                                cursor = 0;
                            }
                        }
                    }
                    // classic lidar data
                    else if (cursor > 0 && trame[0] == '!')
                    {
                        trame[cursor++] = data;
                        if (cursor >= 7)
                        {
                            if (data == 0x0A)
                            {
                                Point p;
                                int header = trame[0];
                                int num = trame[1];
                                p.x = trame[3] << 8 | trame[2];
                                p.y = trame[5] << 8 | trame[4];
                                int footer = trame[6];
                                // I use the radius as the id number
                                Obstacle::Add_Obstacle(num, p);
                                cursor = 0;
                                // break;
                            }
                            cursor = 0;
                        }
                    }
                }
            }
            catch (const std::exception &e)
            {
                printError(e.what());
            }
            if (chrono.Check())
            {
                // printChrono(chrono);
            }
            vTaskDelay(5);
        }

        println("End TaskLidar");
    }

}; // namespace Lidar

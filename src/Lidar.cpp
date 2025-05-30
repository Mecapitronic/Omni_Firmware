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

    void TaskLidar(void *pvParameters)
    {
        println("Start TaskLidar");
        Chrono chrono("Lidar", 1000);
        Timeout sendDataToLidarTimer;
        sendDataToLidarTimer.Start(20);
        while (true)
        {
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

                    if (data == 0x21 && cursor == 0)
                    {
                        trame[cursor++] = data;
                    }
                    else if (cursor > 0)
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

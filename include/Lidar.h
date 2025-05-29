#ifndef LIDAR_H
#define LIDAR_H

#include "ESP32_Helper.h"
#include "PathPlanning/Obstacle.h"
#include "Structure.h"
#include "pins.h"

namespace Lidar
{
    void Initialisation(Robot *_robot);
    void TaskLidar(void *pvParameters);
}; // namespace Lidar
#endif
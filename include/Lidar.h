#ifndef LIDAR_H
#define LIDAR_H

#include "pins.h"
#include "ESP32_Helper.h"
#include "Structure.h"
#include "PathPlanning/Obstacle.h"

namespace Lidar
{    
    extern bool enableComLidar;
    extern bool disableComLidar;

    void Initialisation(Robot *_robot);
    void TaskLidar(void *pvParameters);
};
#endif
#ifndef OBSTACLE_H
#define OBSTACLE_H

/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Structure.h"
#include "Mapping.h"
#include "NodeItem.h"

/****************************************************************************************
 * Parameters
 ****************************************************************************************/
constexpr size_t OBSTACLE_RADIUS = 150; // mm
constexpr size_t OBSTACLE_MARGIN = 40;  // mm for false obstacle checking

constexpr size_t MAX_OBSTACLE = 10;       // normally 1 robot for other team
constexpr size_t MAX_FALSE_OBSTACLE = 10; // potential false obstacle

namespace Obstacle
{
    /****************************************************************************************
     * Variables Globales
     ****************************************************************************************/
    extern std::array<Circle, MAX_OBSTACLE> obstacle;
    extern std::array<Circle, MAX_FALSE_OBSTACLE> false_obstacle;

    /****************************************************************************************
     * Functions Prototypes
     ****************************************************************************************/
    void Initialize_Obstacle(void);
    boolean Is_Valid_Obstacle(uint8_t obstacleID);
    boolean Is_False_Obstacle(Circle circle_obstacle);
    boolean Is_In_Map(Point p);
    void Add_Obstacle(uint8_t id, Point p);
    void PrintObstacleList();
}
#endif

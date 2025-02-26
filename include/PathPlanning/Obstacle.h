#ifndef OBSTACLE_H
#define	OBSTACLE_H

/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Structure.h"
#include "Mapping.h"
#include "Node.h"

/****************************************************************************************
 * Parameters
 ****************************************************************************************/
constexpr size_t OBSTACLE_RADIUS = 150;     // mm
constexpr size_t OBSTACLE_MARGIN = 40;      // mm for false obstacle checking

constexpr size_t MAX_OBSTACLE = 10;         // = LIDAR_MAX_SENSOR (one obstacle per sensor)
constexpr size_t MAX_FALSE_OBSTACLE = 6;    // potential false obstacle


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
Circle Get_Obstacle(uint8 obstacleID);
boolean Is_Valid_Obstacle(uint8 obstacleID);
void Initialize_Obstacle(void);
boolean Is_False_Obstacle(Circle circle_obstacle);
boolean IsInMap(Point p);
Circle Circle_Obstacle_Polar(float32 angle_rad, float32 distance_mm);
Circle Circle_Obstacle_Cart(int x, int y);
void Update_Obstacles(void);
void Add_Obstacle_Polar(uint8 id);
void Add_Obstacle_Cart(uint8 id, int x, int y);
}
#endif


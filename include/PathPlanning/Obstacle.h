#ifndef OBSTACLE_H
#define	OBSTACLE_H

/****************************************************************************************
 * Parameters
 ****************************************************************************************/
#define OBSTACLE_RADIUS    150  // mm
#define OBSTACLE_MARGIN    40  // mm for false obstacle checking

#define MAX_OBSTACLE       10 // = LIDAR_MAX_SENSOR (one obstacle per sensor)
#define MAX_FALSE_OBSTACLE 6  // potential false obstacle


/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "Structure.h"
#include "Mapping.h"
#include "Node.h"

namespace Obstacle
{
/****************************************************************************************
 * Variables Globales
 ****************************************************************************************/
extern Circle obstacle[MAX_OBSTACLE];
extern Circle false_obstacle[MAX_FALSE_OBSTACLE];

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


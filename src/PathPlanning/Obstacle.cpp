/****************************************************************************************
 * Includes
 ****************************************************************************************/
#include "PathPlanning/Obstacle.h"
using namespace Mapping;

namespace Obstacle
{
	/****************************************************************************************
	 * Variables
	 ****************************************************************************************/
	std::array<Circle, MAX_OBSTACLE> obstacle;
	boolean obstacle_enable = true;

	std::array<Circle, MAX_FALSE_OBSTACLE> false_obstacle;
	boolean false_obstacle_enable = false;

	/****************************************************************************************
	 * Initialize obstacle
	 ****************************************************************************************/
	void Initialize_Obstacle(void)
	{
		// Define false obstacle
		int i = 0;
		// false_obstacle[i++] = Circle(0, 0, 0 + OBSTACLE_MARGIN);
		// false_obstacle_enable = true;

		// enable obstacle detection
		obstacle_enable = true;
	}

	/****************************************************************************************
	 * Return true if the obstacle is valid (radius NOT zero)
	 ****************************************************************************************/
	boolean Is_Valid_Obstacle(uint8_t obstacleID)
	{
		return (obstacle[obstacleID].r != 0 ? true : false);
	}

	/****************************************************************************************
	 * Return true if the circle obstacle is considered false
	 ****************************************************************************************/
	boolean Is_False_Obstacle(Circle circle_obstacle)
	{
		if (false_obstacle_enable)
		{
			uint8_t i;
			// check map border limits
			if (circle_obstacle.p.x < OBSTACLE_MARGIN)
				return true;
			if (circle_obstacle.p.x > (MAP_X_MAX - OBSTACLE_MARGIN))
				return true;
			if (circle_obstacle.p.y < OBSTACLE_MARGIN)
				return true;
			if (circle_obstacle.p.y > (MAP_Y_MAX - OBSTACLE_MARGIN))
				return true;
			// check some special point
			for (i = 0; i < MAX_FALSE_OBSTACLE; i++)
			{
				if (Get_Distance_Point(&circle_obstacle.p, &false_obstacle[i].p) <= false_obstacle[i].r)
					return true;
			}
		}
		return false;
	}

	boolean Is_In_Map(Point p)
	{
		// check map border limits
		if (p.x < 0)
			return false;
		if (p.x > MAP_X_MAX)
			return false;
		if (p.y < 0)
			return false;
		if (p.y > MAP_Y_MAX)
			return false;
		return true;
	}

	/****************************************************************************************
	 * Add circle obstacle for graph
	 ****************************************************************************************/
	void Add_Obstacle(uint8_t id, Point p)
	{
		if (obstacle_enable)
		{
			Circle circle_obstacle;
			if (p.x == 0 && p.y == 0 || Is_False_Obstacle(circle_obstacle))
				circle_obstacle = Circle(0, 0, 0);
			else
			{
				circle_obstacle.p.x = p.x;
				circle_obstacle.p.y = p.y;
				circle_obstacle.r = OBSTACLE_RADIUS;
			}
			if (Is_In_Map(circle_obstacle.p) && id > 0 && id < MAX_OBSTACLE)
				obstacle[id] = circle_obstacle;
		}
	}
}
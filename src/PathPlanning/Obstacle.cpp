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
	boolean Is_False_Obstacle(Point obstacle_point)
	{
		if (false_obstacle_enable)
		{
			uint8_t i;
			// check map border limits
			if (obstacle_point.x < OBSTACLE_MARGIN)
				return true;
			if (obstacle_point.x > (MAP_X_MAX - OBSTACLE_MARGIN))
				return true;
			if (obstacle_point.y < OBSTACLE_MARGIN)
				return true;
			if (obstacle_point.y > (MAP_Y_MAX - OBSTACLE_MARGIN))
				return true;
			// check some special point
			for (i = 0; i < MAX_FALSE_OBSTACLE; i++)
			{
				if (Get_Distance_Point(&obstacle_point, &false_obstacle[i].p) <= false_obstacle[i].r)
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
			if (p.x != 0 && p.y != 0 && Is_In_Map(p) && id >= 0 && id < MAX_OBSTACLE && !Is_False_Obstacle(p))
			{
				obstacle[id].p.x = p.x;
				obstacle[id].p.y = p.y;
				obstacle[id].r = OBSTACLE_RADIUS;
			}
			else
			{
				obstacle[id] = Circle(0, 0, 0);
			}
		}
	}

	void PrintObstacleList()
	{
		String str = "ObstacleList:";
		String str01 = String(obstacle[0].p.x) + ":" + String(obstacle[0].p.y);
		String str02 = ":" + String(obstacle[0].r);
		str = str + str01 + str02;

		for (size_t i = 1; i < Max_Circle; i++)
		{
			if (obstacle[i].p.x != 0 || obstacle[i].p.y != 0 || obstacle[i].r != 0)
			{
				String stri1 = String(obstacle[i].p.x) + ":" + String(obstacle[i].p.y);
				String stri2 = ":" + String(obstacle[i].r);
				str = str + ";" + stri1 + stri2;
			}
		}
		// str = str + "|xy";
		Printer::println(str);
	}
}
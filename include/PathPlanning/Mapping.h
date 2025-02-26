#ifndef MAPPING_H
#define	MAPPING_H

/****************************************************************************************
* Includes
****************************************************************************************/
#include "Structure.h"
#include "Obstacle.h"
#include "GeoMathTools.h"
#include "Node.h"
#include "NodeList.h"

//using namespace Node;
//using namespace NodeList;
//using namespace Obstacle;


namespace Mapping
{
/****************************************************************************************
* Parameters
****************************************************************************************/
//#define MAX_SEGMENT    8
//#define MAX_CIRCLE     12
//#define MAX_VERTEX     24

#define ROBOT_RADIUS       135  // mm
#define ROBOT_MARGIN       30   // mm

#define MAP_X_MAX 3000 // Attention: format paysage
#define MAP_Y_MAX 2000

#define MAP_FILE_TEAM_A      "Map2024Jaune.cpp"
#define MAP_FILE_TEAM_B      "Map2024Bleue.cpp"

/****************************************************************************************
* Global Variables
****************************************************************************************/
/*extern*/ //Segment segment[MAX_SEGMENT];
/*extern*/ //Circle circle[MAX_CIRCLE];
/*extern*/ //Vertex vertex[MAX_VERTEX];

/****************************************************************************************
* Prototypes
****************************************************************************************/
void Initialize_Map(uint8 team);

void Set_Adjacent(t_vertexID id1, t_vertexID id2);
void Clear_Adjacent(t_vertexID id1, t_vertexID id2);
boolean Is_Adjacent(t_vertexID id1, t_vertexID id2);
boolean Is_NotAdjacent(t_vertexID id1, t_vertexID id2);

boolean Is_Intersection_Segment(Segment *s1, Segment *s2);
boolean Is_Point_On_Segment(Point *p, Segment *s);
uint32 Get_Distance_Point(Point *p1, Point *p2);
int32 Dot_Product(Point *v1, Point *v2);
Point GePoint_ProjectionOn_Segment(Point *p, Segment *s);
boolean Is_Equal_Point(Point *p1, Point *p2);
boolean Is_DifferenPoint(Point *p1, Point *p2);
boolean Is_Point_CloseTo_Segment(Point *p, Segment *s, uint16 d);
boolean Is_Circle_CloseTo_Segment(Circle *c, Segment *s, uint16 d);
boolean Is_Segment_CloseTo_Segment(Segment *s1, Segment *s2, uint16 d);
boolean Is_NotNull_Circle(Circle *c);

boolean Is_Passable_Point(Point *source, Point *target, uint16 margin);
boolean Is_Passable_Robot(Point *target, uint16 margin);
void Initialize_Passability_Graph(void);
void Update_Passability_Robot(void);
void Update_Passability_Graph(void);
void Update_Passability_Obstacle(void);
void Update_Passability_Element(void);

void Set_End_Vertex(t_vertexID id);
t_vertexID Get_End_Vertex(void);
boolean Is_Equal_Vertex(t_vertexID id1, t_vertexID id2);
boolean Is_Valid_Vertex(uint8 vertexID);
uint32 Get_Distance_Vertex(t_vertexID id1, t_vertexID id2);
void Update_Start_Vertex(int16_t x, int16_t y);
void PrintVertexList();
void PrintSegmentList();
void PrintCircleList();

}
#endif

/****************************************************************************************
* Includes
****************************************************************************************/
#include "PathPlanning/Mapping.h"
using namespace Obstacle;

namespace Mapping
{
/****************************************************************************************
* Variables
****************************************************************************************/

std::array<Segment, Max_Segment> segment;
std::array<Circle, Max_Circle> circle;
std::array<Vertex, Max_Vertex> vertex;

t_vertexID end_vertex_ID;

/****************************************************************************************
* Initialize all map with defined segment, circle, vertex and potential false obstacle
****************************************************************************************/
void Initialize_Map(Team team)
{
  if (team == Team::Jaune)
  {
    // Segment
    segment[0] = Segment(Point(650, 2000), Point(650, 1800));
    segment[1] = Segment(Point(650, 1800), Point(1050, 1800));
    segment[2] = Segment(Point(1050, 1800), Point(1050, 1550));
    segment[3] = Segment(Point(1050, 1550), Point(1950, 1550));
    segment[4] = Segment(Point(1950, 1550), Point(1950, 1800));
    segment[5] = Segment(Point(1950, 1800), Point(2350, 1800));
    segment[6] = Segment(Point(2350, 1800), Point(2350, 2000));
    // Circle
    circle[0] = Circle(950, 950, 100);
    circle[1] = Circle(1050, 950, 100);
    circle[2] = Circle(1150, 950, 100);
    circle[3] = Circle(1250, 950, 100);
    circle[4] = Circle(1750, 950, 100);
    circle[5] = Circle(1850, 950, 100);
    circle[6] = Circle(1950, 950, 100);
    circle[7] = Circle(2050, 950, 100);
    circle[8] = Circle(75, 1325, 400);
    circle[9] = Circle(75, 400, 400);
    circle[10] = Circle(2925, 1325, 400);
    circle[11] = Circle(2925, 400, 400);
    circle[12] = Circle(775, 250, 400);
    circle[13] = Circle(2225, 250, 400);
    circle[14] = Circle(825, 1725, 400);
    circle[15] = Circle(2175, 1725, 400);
    // Vertex
    vertex[0].point = Point(0, 0);
    vertex[1].point = Point(1225, 225);
    vertex[2].point = Point(775, 400);
    vertex[3].point = Point(775, 175);
    vertex[4].point = Point(250, 400);
    vertex[5].point = Point(600, 875);
    vertex[6].point = Point(1100, 800);
    vertex[7].point = Point(1100, 1100);
    vertex[8].point = Point(250, 1325);
    vertex[9].point = Point(825, 1600);
    vertex[10].point = Point(375, 1400);
    vertex[11].point = Point(375, 1750);
    vertex[12].point = Point(1500, 550);
    vertex[13].point = Point(1500, 1350);
    vertex[14].point = Point(1775, 400);
    vertex[15].point = Point(2225, 400);
    vertex[16].point = Point(2225, 175);
    vertex[17].point = Point(2750, 400);
    vertex[18].point = Point(2400, 875);
    vertex[19].point = Point(1900, 800);
    vertex[20].point = Point(1900, 1100);
    vertex[21].point = Point(2750, 1325);
    vertex[22].point = Point(2775, 875);
  }
  else if (team == Team::Bleu)
  {
    // Segment
    segment[0] = Segment(Point(2350, 2000), Point(2350, 1800));
    segment[1] = Segment(Point(2350, 1800), Point(1950, 1800));
    segment[2] = Segment(Point(1950, 1800), Point(1950, 1550));
    segment[3] = Segment(Point(1950, 1550), Point(1050, 1550));
    segment[4] = Segment(Point(1050, 1550), Point(1050, 1800));
    segment[5] = Segment(Point(1050, 1800), Point(650, 1800));
    segment[6] = Segment(Point(650, 1800), Point(650, 2000));
    // Circle
    circle[0] = Circle(2050, 950, 100);
    circle[1] = Circle(1950, 950, 100);
    circle[2] = Circle(1850, 950, 100);
    circle[3] = Circle(1750, 950, 100);
    circle[4] = Circle(1250, 950, 100);
    circle[5] = Circle(1150, 950, 100);
    circle[6] = Circle(1050, 950, 100);
    circle[7] = Circle(950, 950, 100);
    circle[8] = Circle(2925, 1325, 400);
    circle[9] = Circle(2925, 400, 400);
    circle[10] = Circle(75, 1325, 400);
    circle[11] = Circle(75, 400, 400);
    circle[12] = Circle(2225, 250, 400);
    circle[13] = Circle(775, 250, 400);
    circle[14] = Circle(2175, 1725, 400);
    circle[15] = Circle(825, 1725, 400);
    // Vertex
    vertex[0].point = Point(0, 0);
    vertex[1].point = Point(1775, 225);
    vertex[2].point = Point(2225, 400);
    vertex[3].point = Point(2225, 175);
    vertex[4].point = Point(2750, 400);
    vertex[5].point = Point(2400, 875);
    vertex[6].point = Point(1900, 800);
    vertex[7].point = Point(1900, 1100);
    vertex[8].point = Point(2750, 1325);
    vertex[9].point = Point(2175, 1600);
    vertex[10].point = Point(2625, 1400);
    vertex[11].point = Point(2625, 1750);
    vertex[12].point = Point(1500, 550);
    vertex[13].point = Point(1500, 1350);
    vertex[14].point = Point(1225, 400);
    vertex[15].point = Point(775, 400);
    vertex[16].point = Point(775, 175);
    vertex[17].point = Point(250, 400);
    vertex[18].point = Point(600, 875);
    vertex[19].point = Point(1100, 800);
    vertex[20].point = Point(1100, 1100);
    vertex[21].point = Point(250, 1325);
    vertex[22].point = Point(225, 875);
  }
}

/****************************************************************************************
* Set adjacent the vertices #id1 and #id2 (Vertex vertex[] must be declared)
****************************************************************************************/
void Set_Adjacent(t_vertexID id1, t_vertexID id2)
{
  vertex[id1].adjacency_active |= ((t_adjacency)1u << id2);
  vertex[id2].adjacency_active |= ((t_adjacency)1u << id1);
}

/****************************************************************************************
* Set NOT adjacent the vertices #id1 and #id2 (Vertex vertex[] must be declared)
****************************************************************************************/
void Clear_Adjacent(t_vertexID id1, t_vertexID id2)
{
  vertex[id1].adjacency_active &= ~((t_adjacency)1u << id2);
  vertex[id2].adjacency_active &= ~((t_adjacency)1u << id1);
}

/****************************************************************************************
* Return 1 if vertices id1 and id2 are adjacent
****************************************************************************************/
boolean Is_Adjacent(t_vertexID id1, t_vertexID id2)
{ 
  if ((vertex[id1].adjacency_active & ((t_adjacency)1u << id2)) 
   || (vertex[id2].adjacency_active & ((t_adjacency)1u << id1))) 
    return true;

  return false;
}

/****************************************************************************************
* Return 1 if vertices id1 and id2 are NOT adjacent
****************************************************************************************/
boolean Is_NotAdjacent(t_vertexID id1, t_vertexID id2)
{
  return !Is_Adjacent(id1, id2);
}

/****************************************************************************************
* Return 1 if an intersection exists between segments s1 and s2
****************************************************************************************/
boolean Is_Intersection_Segment(Segment *s1, Segment *s2)
{
	Point commun;

    if ((s1->a == s2->a) && (s1->b == s2->b)) // segments are one the same line (parallel but not distinct)
        if (Is_Point_On_Segment(&s2->p1, s1) && Is_Point_On_Segment(&s2->p2, s1))
            return true; 

    if ((s1->a == s2->a) && (s1->b != s2->b)) // segments are parallel and distinct
        return false;

    if (s1->a == 9999) // segment 1 is vertical
    {
        commun.x = s1->p1.x;
        commun.y = (s2->a * commun.x) + s2->b;
    }
    else if (s2->a == 9999) // segment 2 is vertical
    {
        commun.x = s2->p1.x;
        commun.y = (s1->a * commun.x) + s1->b;
    }
    else
    {
        commun.x = (s2->b - s1->b) / (s1->a - s2->a);
        commun.y = (s1->a * commun.x) + s1->b;
    }
  
  if (Is_Point_On_Segment(&commun, s1) && Is_Point_On_Segment(&commun, s2))
    return true;

  return false;
}

/****************************************************************************************
* Return 1 if point p is between end points of segment s (/!\ it's assume that p is on the line)
****************************************************************************************/
boolean Is_Point_On_Segment(Point * p, Segment *s)
{
  if ((p->x <= max(s->p1.x,s->p2.x)) && (p->x >= min(s->p1.x,s->p2.x))
   && (p->y <= max(s->p1.y,s->p2.y)) && (p->y >= min(s->p1.y,s->p2.y)))
    return true;

  return false;
}

/****************************************************************************************
* Return the distance (approximated) between points p1 and p2
****************************************************************************************/
uint32_t Get_Distance_Point(Point *p1, Point *p2)
{
  uint32_t dx = abs(p1->x - p2->x);
  uint32_t dy = abs(p1->y - p2->y);
  return Approx_Distance(dx,dy);
}

/****************************************************************************************
* Return the dot (scalar) product of vectors (points) v1 and v2
****************************************************************************************/
int32_t Dot_Product(Point *v1, Point *v2)
{
  return ((v1->x * v2->x) + (v1->y * v2->y));
}

/****************************************************************************************
* Return the orthogonal projected point of p on segment s
****************************************************************************************/
Point GePoint_ProjectionOn_Segment(Point *p, Segment *s)
{
  Point u, v;
  float projection;
  
  u.x = p->x - s->p1.x;
  u.y = p->y - s->p1.y;
  v.x = s->p2.x - s->p1.x;
  v.y = s->p2.y - s->p1.y;

  if ((v.x == 0) && (v.y == 0)) // check if segment length is null
    return s->p1;
  
  projection = Dot_Product(&u, &v);
  projection /= Dot_Product(&v, &v);
  u.x = (projection * v.x) + s->p1.x;
  u.y = (projection * v.y) + s->p1.y;

  return u;
}

/****************************************************************************************
* Return 1 if point p1 is the same point than p2
****************************************************************************************/
boolean Is_Equal_Point(Point *p1, Point *p2)
{
  if ((p1->x == p2->x) && (p1->y == p2->y)) 
    return true;
  
  return false;
}

/****************************************************************************************
* Return 1 if point p1 is different from point p2
****************************************************************************************/
boolean Is_DifferenPoint(Point *p1, Point *p2)
{
  return !Is_Equal_Point(p1, p2);
}

/****************************************************************************************
* Return 1 if distance (orthogonal) between point p and segment s is less than d
****************************************************************************************/
boolean Is_Point_CloseTo_Segment(Point *p, Segment *s, uint16_t d)
{
	Point projected;

//  if (Get_Distance_Point(p, s.p1) <= d) // check distance from first end point
//    return true;
//
//  if (Get_Distance_Point(p, s.p2) <= d) // check distance from second end point
//    return true;
    
  projected = GePoint_ProjectionOn_Segment(p, s);
  
  if (Is_Point_On_Segment(&projected, s))
  {
    if (Get_Distance_Point(p, &projected) <= d)  // check distance from line
      return true;
  }
  
  return false;
}

/****************************************************************************************
* Return 1 if distance between circle c and segment s is less than d
****************************************************************************************/
boolean Is_Circle_CloseTo_Segment(Circle *c, Segment *s, uint16_t d)
{
  d = d + c->r;
  // check distance from each end point to circle
  if (Get_Distance_Point(&s->p1, &c->p) <= d) return true;
  if (Get_Distance_Point(&s->p2, &c->p) <= d) return true;
  
  return Is_Point_CloseTo_Segment(&c->p, s, d);
}

/****************************************************************************************
* Return 1 if distance between segment s1 and segment s2 is less than d
****************************************************************************************/
boolean Is_Segment_CloseTo_Segment(Segment *s1, Segment *s2, uint16_t d)
{
  // check segment intersection
  if (Is_Intersection_Segment(s1, s2)) return true;
      
//  // check distance from each end point to each other       // manually done in excel map
//  if (Get_Distance_Point(&s1->p1, &s2->p1) <= d) return true;
//  if (Get_Distance_Point(&s1->p1, &s2->p2) <= d) return true;
//  if (Get_Distance_Point(&s1->p2, &s2->p1) <= d) return true;
//  if (Get_Distance_Point(&s1->p2, &s2->p2) <= d) return true;
    
//  // check distance orthogonal from end point               // manually done in excel map
//  if (Is_Point_CloseTo_Segment(&s1->p1, s2, d)) return true;
//  if (Is_Point_CloseTo_Segment(&s1->p2, s2, d)) return true;
//  if (Is_Point_CloseTo_Segment(&s2->p1, s1, d)) return true;
//  if (Is_Point_CloseTo_Segment(&s2->p2, s1, d)) return true;

  return false;
}

/****************************************************************************************
* Return 1 if radius of circle is NOT zero
****************************************************************************************/
boolean Is_NotNull_Circle(Circle *c)
{
  return (c->r != 0 ? 1 : 0);
}

/****************************************************************************************
* Return 1 if point target is passable from point source, according to distance margin
****************************************************************************************/
boolean Is_Passable_Point(Point *source, Point *target, uint16_t margin)
{
  uint8_t i;
  Segment edge;

//  if (Is_Equal_Point(source, target)) // check segment existence
//    return false;
  
  edge = Segment(*source, *target);

  for (i=0; i<Max_Segment; i++)
  {
    if (Is_Segment_CloseTo_Segment(&segment[i], &edge, margin)) // check segment proximity
      return false;
  }

  for (i=0; i<Max_Circle; i++)
  {
    if (Is_NotNull_Circle(&circle[i]))
    {
      if (Is_Circle_CloseTo_Segment(&circle[i], &edge, margin)) // check circle proximity
        return false;
    }
  }
  
  return true; // after all checking
}

/****************************************************************************************
* Return 1 if point target is passable from robot (vertex[0]), according to distance margin
****************************************************************************************/
boolean Is_Passable_Robot(Point *target, uint16_t margin)
{
  uint8_t i;
  Segment edge;
  uint16_t margin_temp;
  
  edge = Segment(vertex[0].point, *target);

  for (i=0; i<Max_Segment; i++) // check segment proximity
  {
    // check segment intersection
    if (Is_Intersection_Segment(&segment[i], &edge)) return false;
    
//    // check distance from each end point to each other
//    //if (Get_Distance_Point(&edge.p1, &segment[i].p1) <= margin) return false; // ignore edge.p1 = vertex[0] = robot
//    //if (Get_Distance_Point(&edge.p1, &segment[i].p2) <= margin) return false;
//    if (Get_Distance_Point(&edge.p2, &segment[i].p1) <= margin) return false;
//    if (Get_Distance_Point(&edge.p2, &segment[i].p2) <= margin) return false;
      
//    // check distance orthogonal from end point
//    if (Is_Point_CloseTo_Segment(&segment[i].p1, &edge, margin)) return false;
//    if (Is_Point_CloseTo_Segment(&segment[i].p2, &edge, margin)) return false;
//    //if (Is_Point_CloseTo_Segment(edge.p1, segment[i], margin)) return false;
//    if (Is_Point_CloseTo_Segment(&edge.p2, &segment[i], margin)) return false;  // manually done in excel map
  }

  for (i=0; i<Max_Circle; i++) // check circle proximity
  {
    if (Is_NotNull_Circle(&circle[i]))
    {
      margin_temp = margin + circle[i].r;
      // check distance from end point to circle
      //if (Get_Distance_Point(&edge.p1, &circle[i].p) <= margin_temp) return false;
      if (Get_Distance_Point(&edge.p2, &circle[i].p) <= margin_temp) return false;  // usefull for vertex in circle

      if (Is_Point_CloseTo_Segment(&circle[i].p, &edge, margin_temp)) return false; // most important
    }
  }
  
  margin_temp = margin + OBSTACLE_RADIUS;
  
  for (i=0; i<MAX_OBSTACLE; i++) // check obstacle proximity
  {
    if (Is_Valid_Obstacle(i))
    {
      if (Get_Distance_Point(&edge.p2, &obstacle[i].p) < margin_temp) 
        return false;

      if (Is_Point_CloseTo_Segment(&obstacle[i].p, &edge, margin_temp)) 
        return false;
    }
  }
  
  return true; // after all checking
}

/****************************************************************************************
* Compute all the passability graph and set the initial adjacency
****************************************************************************************/
void Initialize_Passability_Graph(void)
{ 
  uint16_t i, j;
  uint16_t margin = ROBOT_RADIUS + ROBOT_MARGIN;
  
  for (i=0; i<Max_Vertex; i++)
  {
    vertex[i].adjacency_active = 0;  // clear all list
    vertex[i].adjacency_passive = 0;
    vertex[i].adjacency_static = 0;
  }
  
  for (i=1; i<Max_Vertex; i++)
  { 
    for (j=(i+1); j<Max_Vertex; j++) // ignore vertex couple already checked (i,j)=(j,i)
    {
      //if (i != j) // if not the same point  // no more possible
      {
        //if (Is_NotAdjacent(i, j))  // if not already adjacent
        {
          if (Is_Passable_Point(&vertex[i].point, &vertex[j].point, margin))
        {
          Set_Adjacent(i, j); // set list
        }
      }
    }
    }
    vertex[i].adjacency_static = vertex[i].adjacency_active; // backup list
    vertex[i].adjacency_passive = vertex[i].adjacency_active;
  }
  
  //Update_Passability_Robot(); // vertex #0
}

/****************************************************************************************
* Update the passability graph from robot (vertex #0)
****************************************************************************************/
void Update_Passability_Robot(void)
{ 
  uint8_t i;
  uint16_t margin = ROBOT_RADIUS + ROBOT_MARGIN;
  //TODO
  //vertex[0].point = robot.mm;
  vertex[0].adjacency_active = 0;  // clear list
  
  for (i=1; i<Max_Vertex; i++)
  {
    if (Is_Passable_Robot(&vertex[i].point, margin))
    {
      Set_Adjacent(0, i);
    }
    else 
    {
      Clear_Adjacent(0, i);
    }
  }
  vertex[0].adjacency_passive = vertex[0].adjacency_active; // backup list
}

/****************************************************************************************
* Update the passability graph
****************************************************************************************/
void Update_Passability_Graph(void)
{ 
  Update_Passability_Obstacle();
  Update_Passability_Robot();
}

/****************************************************************************************
* Update the passability graph considering obstacle only
****************************************************************************************/
void Update_Passability_Obstacle(void)
{
  uint16_t i, j, k;
  uint16_t margin = ROBOT_RADIUS + ROBOT_MARGIN; 
  Segment edge;

  for (i=1; i<Max_Vertex; i++)
  {
    vertex[i].adjacency_active = vertex[i].adjacency_passive;  // retrieve list without obstacle
  }
  
  for (k=0; k<MAX_OBSTACLE; k++)  
  {
    if (Is_Valid_Obstacle(k))  // for all obstacle detected
    {
      for (i=1; i<Max_Vertex; i++)
      { 
          if (Is_Valid_Vertex(i))
          {
              for (j = (i + 1); j < Max_Vertex; j++) // ignore vertex couple already checked (i,j)=(j,i)
              {
                  if (Is_Valid_Vertex(j))
                  {
                      if (Is_Adjacent(i, j))  // for all couple passabled
                      {
                          edge = Segment(vertex[i].point, vertex[j].point);

                              if (Is_Circle_CloseTo_Segment(&obstacle[k], &edge, margin)) // check circle proximity
                              {
                                  Clear_Adjacent(i, j);  // Temporarily not passabled 
                              }
                      }
                  }
              }
          }
      }
    }
  }

}

/****************************************************************************************
* Update the passability graph considering playing element only
****************************************************************************************/
//void Update_Passability_Element(void)
//{ 
//  uint16_t i, j, k;
//  uint16_t margin = ROBOT_RADIUS + ROBOT_MARGIN + OBSTACLE_RADIUS; 
//  Segment edge;
//  
//  for (i=1; i<MAX_VERTEX; i++)
//  {
//    vertex[i].adjacency_active = vertex[i].adjacency_static;  // retrieve list without element
//  }
//  
//  for (k=0; k<MAX_ELEMENT; k++)  
//  {
//    if (Is_Obstacle_Element(k))  // for all element still present
//    {
//      for (i=1; i<MAX_VERTEX; i++)
//      { 
//        for (j=(i+1); j<MAX_VERTEX; j++) // ignore vertex couple already checked (i,j)=(j,i)
//        {
//          if (Is_Adjacent(i, j))  // for all couple passabled
//          {
//            edge = Segment(vertex[i].point, vertex[j].point);
//        
//            if (Is_Circle_CloseTo_Segment(&element[k], &edge, margin)) // check circle proximity
//            {
//              Clear_Adjacent(i,j);  // Temporarily not passabled 
//            }
//          }
//        }
//        vertex[i].adjacency_passive = vertex[i].adjacency_active; // backup list
//      }
//    }
//  }
//  
//}

/****************************************************************************************
* Set end vertex for PathFinding
****************************************************************************************/
void Set_End_Vertex(t_vertexID id)
{
  end_vertex_ID = id;
}

/****************************************************************************************
* Get end vertex for PathFinding
****************************************************************************************/
t_vertexID Get_End_Vertex(void)
{
  return end_vertex_ID;
}

/****************************************************************************************
* Return 1 if vertex id1 is the same vertex than id2
****************************************************************************************/
boolean Is_Equal_Vertex(t_vertexID id1, t_vertexID id2)
{
  return Is_Equal_Point(&vertex[id1].point, &vertex[id2].point);
}

/****************************************************************************************
* Return 1 if the vertex is valid
****************************************************************************************/
boolean Is_Valid_Vertex(uint8_t vertexID)
{
    return (&vertex[vertexID].point.x !=0 || &vertex[vertexID].point.y != 0);
}

/****************************************************************************************
* Return the distance (approximated) between vertex id1 and id2
****************************************************************************************/
uint32_t Get_Distance_Vertex(t_vertexID id1, t_vertexID id2)
{
  return Get_Distance_Point(&vertex[id1].point, &vertex[id2].point);
}

/****************************************************************************************
* Update vertex[0] to robot position
****************************************************************************************/
void Update_Start_Vertex(int16_t x, int16_t y)
{
  vertex[0].point.x = x; //robot.mm.x;
  vertex[0].point.y = y; //robot.mm.y;
}

void PrintVertex0()
{
  String str = "VertexList:";
  String str01 = String(vertex[0].point.x) + ":" + String(vertex[0].point.y);
  String str02 = ":" + String((uint64_t)(vertex[0].adjacency_active));
  str = str + str01 + str02;
  Printer::println(str);
}

void PrintVertexList()
{
  String str = "VertexList:";
  String str01 = String(vertex[0].point.x) + ":" + String(vertex[0].point.y);
  String str02 = ":" + String((uint64_t)(vertex[0].adjacency_active));
  str = str + str01 + str02;
  
  for (size_t i = 1; i < Max_Vertex; i++)
  {
    if(vertex[i].point.x != 0 || vertex[i].point.y != 0)
    {
      String stri1 = String(vertex[i].point.x) + ":" + String(vertex[i].point.y);
      String stri2 = ":" + String((uint64_t)(vertex[i].adjacency_active));
      str = str + ";" + stri1 + stri2;  
    }
  }
  //str = str + "|xy";
  Printer::println(str);
}

void PrintSegmentList()
{
  String str = "SegmentList:";
  String str01 = String(segment[0].p1.x) + ":" + String(segment[0].p1.y);
  String str02 =  ":" + String(segment[0].p2.x) + ":" + String(segment[0].p2.y);
  str = str + str01 + str02;
  
  for (size_t i = 1; i < Max_Segment; i++)
  {
    if(segment[i].p1.x != 0 || segment[i].p1.y != 0 || segment[i].p2.x != 0 || segment[i].p2.y != 0)
    {
      String stri1 = String(segment[i].p1.x) + ":" + String(segment[i].p1.y);
      String stri2 = ":" + String(segment[i].p2.x) + ":" + String(segment[i].p2.y);
      str = str + ";" + stri1 + stri2;
    }
  }
  //str = str + "|xy";
  Printer::println(str);
}

void PrintCircleList()
{
  String str = "CircleList:";
  String str01 = String(circle[0].p.x) + ":" + String(circle[0].p.y);
  String str02 =  ":" + String(circle[0].r);
  str = str + str01 + str02;
  
  for (size_t i = 1; i < Max_Circle; i++)
  {
    if(circle[i].p.x != 0 || circle[i].p.y != 0 || circle[i].r != 0)
    {
      String stri1 = String(circle[i].p.x) + ":" + String(circle[i].p.y);
      String stri2 = ":" + String(circle[i].r);
      str = str + ";" + stri1 + stri2;
    }
  }
  //str = str + "|xy";
  Printer::println(str);
}

//void Print_Visibility_Graph()
//{
//    uint16_t i;
//    for (i = 0; i < MAX_VERTEX; i++)
//    {
//        printf("vertex[%d].adjacency = %ld",i ,vertex[i].adjacency);        printf(";\n");
//        printf("vertex[%d].adjacency_static = %ld",i ,vertex[i].adjacency_static);        printf(";\n");
//        printf("vertex[%d].point.x = %d",i ,vertex[i].point.x);        printf(";\n");
//        printf("vertex[%d].point.y = %d",i ,vertex[i].point.y);        printf(";\n");
//    }
//}

}
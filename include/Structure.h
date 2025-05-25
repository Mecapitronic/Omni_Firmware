#ifndef STRUCTURE_H
#define STRUCTURE_H

#include <Arduino.h>
#include "Structure_Helper.h"

/****************************************************************************************
 * Enum Class
 ****************************************************************************************/
// Bleu = 0
// Jaune = 1
enum class Team
{
  Bleu,
  Jaune,
  None
};

// 

/**
 * @brief WAIT BEGIN RUN STOP END
 *
 * WAIT attend que le match démarre, BEGIN le match démarre avec la tirette (état temporaire très court)
 * RUN le match est en cours, STOP les actions sont terminées, on attend la fin du timer, END le match et le timer sont terminés
 * 
 */
enum class State
{
  MATCH_WAIT,
  MATCH_BEGIN,
  MATCH_RUN,
  MATCH_STOP,
  MATCH_END
};

/*
#define CMD_FREE		0
#define CMD_BUSY		1
#define CMD_DONE		2
#define CMD_FAIL		3
*/

/****************************************************************************************
 * MAPPING
 ****************************************************************************************/

typedef uint64_t t_adjacency; // 64 vertex max

struct Vertex
{
  Point point;
  t_adjacency adjacency_active;  // high level graph with active obstacle => working graph
  t_adjacency adjacency_passive; // medium level graph with passive play element
  t_adjacency adjacency_static;  // low level graph with only static map element
};

struct Segment
{
  Point p1;
  Point p2;
  float a;   // slope of the segment : rise/run = (Y2 - Y1) / (X2 - X1)
  int32_t b; // intercept the axis : y = a*x + b
  Segment()
  {
    p1.x = 0;
    p1.y = 0;
    p2.x = 0;
    p2.y = 0;
    a = 0;
    b = 0;
  }
  Segment(Point _p1, Point _p2)
  {
    p1 = _p1;
    p2 = _p2;
    if (p1.x == p2.x)
    {
      a = 9999; // infinite slope (vertical line)
      b = p1.x;
    }
    else
    {
      a = (p2.y - p1.y); // slope
      a /= (p2.x - p1.x);
      b = (p1.y - (a * p1.x)); // intercept
    }
  }
};

struct Circle
{
  Point p;
  uint16_t r;
  Circle() : p{0, 0}, r(0) {}
  Circle(int16_t _x, int16_t _y, uint16_t _r = 0) : p{_x, _y}, r(_r) {}
  Circle(Point _p, uint16_t _r = 0) : p(_p), r(_r) {}
};

typedef uint8_t t_vertexID;

/****************************************************************************************
* PathFinding
****************************************************************************************/
/*
struct t_node{
  uint32_t currentCost;
  uint32_t parentCost;
  t_vertexID parentID;
  t_vertexID currentID;
};
*/
/****************************************************************************************
 * STRATEGY
 ****************************************************************************************/
/*typedef struct {
  t_vertexID vertexID;
  int8_t mission;
  bool possible;
  bool done;
  uint8_t iteration;
  Point point;
  bool (* function)(void);
} t_action;
*/

/****************************************************************************************
 * UART COMMAND
 ****************************************************************************************/
/*
typedef struct {
  char cmd;
  t_vertexID actionID;
  t_vertexID vertexID;
  //boolean available;
  Point point;
  float angle;
  int32_t distance;
} t_uartCMD;
*/
// Exemple overriding operator == and !=
/*
typedef struct T_t
{
    /// @brief X value
    float x;

    /// @brief Y value
    float y;

    /// @brief Heading value
    float h;

    bool operator==(const T_t &other) const
    {
        return this->x == other.x
            && this->y == other.y
            && this->h == other.h;
    }
    bool operator!=(const T_t &other) const
    {
        return !(*this == other);
    }
};
*/

struct Robot : PoseF
{
  /****************************************************************************************
   * Variables
   ****************************************************************************************/
  // inherit of PoseF : x, y, h

  /****************************************************************************************
   * Prototypes fonctions
   ****************************************************************************************/
  void SetPose(float x_mm, float y_mm, float h_rad)
  {
    x = x_mm;
    y = y_mm;
    h = h_rad;
  }
  PoseF GetPoseF()
  {
    return {x, y, h};
  }
  Pose GetPose()
  {
    return {(int16_t)(x), (int16_t)(y), (int32_t)(h)};
  }
  PointF GetPointF()
  {
    return {x, y};
  }
  Point GetPoint()
  {
    return {(int16_t)(x), (int16_t)(y)};
  }
};

#endif

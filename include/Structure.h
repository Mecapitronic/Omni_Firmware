#ifndef STRUCTURE_H
#define STRUCTURE_H

#define SERIAL_PRINT // TODO : remove this line

#include <Arduino.h>
#include "Structure_Helper.h"

/****************************************************************************************
 * Types de données
 ****************************************************************************************/
// // boolean (1 byte)
//typedef unsigned char boolean;
typedef bool boolean;

// // char (1 byte)
// #define INT8_MIN 		(-128)
// #define INT8_MAX 		(127)
typedef signed char int8;
// #define UINT8_MAX 		(255)
typedef unsigned char uint8;

// // int (2 bytes)
// #define INT16_MIN 		(-32768)
// #define INT16_MAX 		(32767)
typedef signed int int16;
// #define UINT16_MAX 		(65535U)
typedef unsigned int uint16;

// // long (4 bytes)
// #define INT32_MIN 		(-2 147 483 648L)
// #define INT32_MAX 		(2147483647L)
typedef signed long int int32;
// #define UINT32_MAX 		(4294967295UL)
typedef unsigned long int uint32;

// // long long (8 bytes)
// #define INT64_MIN               (-9223372036854775808LL)
// #define INT64_MAX               (9223372036854775808LL)
typedef signed long long int int64;
// #define UINT64_MAX              (18446744073709551615ULL)
typedef unsigned long long int uint64;

// // float (4 bytes)
// #define FLOAT32_MIN             (1.175494351E?38)           // 2e-126
// #define FLOAT32_MAX             (3.402823466E38)            // 2e128
typedef float float32; // about 7 decimal digits

// // double (8 bytes)
// #define FLOAT64_MIN             (2.2250738585072014E?308)   // 2e-1022
// #define FLOAT64_MAX             (1.7976931348623158E308)    // 2e1024
typedef double float64; // about 16 decimal digits

// /* SUFFIXES :
//  * u or U : unsigned
//  * l or L : long
//  * f or F or . : float
//  * E : exponent
//  */

/****************************************************************************************
 * Constantes Génériques
 ****************************************************************************************/
enum class Team {Jaune, Bleue};
enum class Mode {Match, Test};

 /*
#define CMD_FREE		0
#define CMD_BUSY		1
#define CMD_DONE		2
#define CMD_FAIL		3
*/

/****************************************************************************************
* MAPPING
****************************************************************************************/

typedef uint64 t_adjacency; // 64 vertex max

struct Vertex{
  Point point;
  t_adjacency adjacency_active;   // high level graph with active obstacle => working graph
  t_adjacency adjacency_passive;  // medium level graph with passive play element
  t_adjacency adjacency_static;   // low level graph with only static map element
};

struct Segment{
  Point p1;
  Point p2;
  float32 a;	// slope of the segment : rise/run = (Y2 - Y1) / (X2 - X1)
  int32 b;		// intercept the axis : y = a*x + b
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
      a = 9999;  // infinite slope (vertical line)
      b = p1.x;
    }
    else
    {
      a = (p2.y - p1.y);  // slope
      a /= (p2.x - p1.x);
      b = (p1.y - (a * p1.x));  // intercept
    }
  }
};

struct Circle{
  Point p;
  uint16 r;
  Circle()
  {
    p.x = 0;
    p.y = 0;
    r = 0;
  }
  Circle(int32 _x, int32 _y, uint16 _r)
  {
    p.x = _x;
    p.y = _y;
    r = _r;
  }
};

typedef uint8 t_vertexID;

/****************************************************************************************
* PathFinding
****************************************************************************************/
/*
struct t_node{
  uint32 currentCost;
	uint32 parentCost;
	t_vertexID parentID;
  t_vertexID currentID;
};
*/
/****************************************************************************************
* STRATEGY
****************************************************************************************/
/*typedef struct {
  t_vertexID vertexID;
  int8 mission;
  bool possible;
  bool done;
  uint8 iteration;
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
	float32 angle;
	int32 distance;
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
  PoseF Pose()
  {
    return {x, y, h};
  }
  PointF Position()
  {
    return {x, y};
  }
};

#endif

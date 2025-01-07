#ifndef STRUCTURE_H
#define STRUCTURE_H

#include <Arduino.h>

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
#define OK          1
#define NOK         0

#define	ON          1
#define	OFF         0

#define YES         1
#define NO          0

//#define HIGH        1
//#define LOW         0

#define TRUE        1
#define FALSE       0

#define FOREVER     1

#define EXIT_OK     0

#ifndef NULL
#define	NULL	   (0)
#endif

#define CMD_FREE		0
#define CMD_BUSY		1
#define CMD_DONE		2
#define CMD_FAIL		3

/****************************************************************************************
* MAPPING
****************************************************************************************/
typedef struct {
  int32 x;
  int32 y;
} t_point;

typedef struct {
  float32 x;
  float32 y;
} t_pointF;

typedef uint64 t_adjacency; // 64 vertex max

typedef struct {
  t_point point;
  t_adjacency adjacency_active;   // high level graph with active obstacle => working graph
  t_adjacency adjacency_passive;  // medium level graph with passive play element
  t_adjacency adjacency_static;   // low level graph with only static map element
} t_vertex;

typedef struct {
  t_point p1;
  t_point p2;
  float32 a;	// slope of the segment : rise/run = (Y2 - Y1) / (X2 - X1)
  int32 b;		// intercept the axis : y = a*x + b
} t_segment;

typedef struct {
  t_point p;
  uint16 r;
} t_circle;

typedef uint8 t_vertexID;


/****************************************************************************************
* Path_Finding
****************************************************************************************/
typedef struct {
  uint32 currentCost;
	uint32 parentCost;
	t_vertexID parentID;
  t_vertexID currentID;
} t_node;


/****************************************************************************************
* CONTROL SYSTEM
****************************************************************************************/
// paramètres d'un régulateur PID
typedef struct {
  //int32 command;
  int32 error;
  int32 last_error;
  int32 sum_error;
  float32 kP;
  float32 kI;
  float32 kD;
} t_PID;

//                                                 value 
//                                                   V
// Paramètres génériques de contrôle : setpoint -> (PID) -> command -> (SYSTEM)
typedef struct {
  int32 setpoint;   // consigne, valeur à atteindre définie par l'utilisateur OU relatif pour la position = distance restante
  int32 command;    // commande, valeur envoyée par le contrôleur au système
  int32 real;       // valeur actuelle, réelle, mesurée ou estimée
  int32 pivot;      
} t_control;

/****************************************************************************************
* MOTION
****************************************************************************************/
// paramètres d'un mouvement avec asservissement PID sur une des composantes (par exemple vitesse)
/*typedef struct {
  t_control position;      // déplacement
  t_control velocity;      // vitesse
  t_control acceleration;  // acceleration
  t_control jerk;          // à-coup
  //t_PID PID;               // régulateur PID
} t_motion;
*/
// paramètres de cinématique du robot (ajout de la direction (linéaire) pour un holonome)
typedef struct {
  t_pointF location;         // coordonnées actuelles x et y en mm
  float32 direction;        // direction du vecteur déplacement linéaire en radians
  float32 orientation;      // orientation (du déplacement angulaire) actuelle en degrés => ang.position en radians
  /*String teleplot()
  {
    String ret1 = String() + ">robot:" + (int)location.x;
    String ret2 = ":" + (int)location.y;
    String ret3 = ":" + (int)(orientation*100);
    String ret4 = "|xy";
    return ret1+ret2+ret3+ret4;
  }*/
} t_robot;


/****************************************************************************************
* STRATEGY
****************************************************************************************/
typedef struct {
  t_vertexID vertexID;
  int8 mission;
  bool possible;
  bool done;
  uint8 iteration;
  t_point point;
  bool (* function)(void);
} t_action;


/****************************************************************************************
* UART COMMAND
****************************************************************************************/
typedef struct {
	char cmd;
	t_vertexID actionID;
	t_vertexID vertexID;
	//boolean available;
	t_point point;
	float32 angle;
	int32 distance;
} t_uartCMD;

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
#endif

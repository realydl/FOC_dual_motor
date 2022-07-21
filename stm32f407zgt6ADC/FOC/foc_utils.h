#ifndef FOCUTILS_LIB_H
#define FOCUTILS_LIB_H

#include "arm_math.h"
//#include "main.h"
/******************************************************************************/
// sign function
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#define _round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _sqrt(a) (_sqrtApprox(a))
#define _isset(a) ( (a) != (NOT_SET) )

// utility defines
#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f

#define NOT_SET -12345
/******************************************************************************/

// dq current structure 
typedef struct 
{
	float d;
	float q;
} DQCurrent_s;
// phase current structure 
typedef struct 
{
	float a;
	float b;
	float c;
} PhaseCurrent_s;
// dq voltage structs
typedef struct 
{
	float d;
	float q;
} DQVoltage_s;

typedef struct
{
	float alpha;
	float beta;
} Ialphabeta;

/******************************************************************************/
float _sin(float a);
float _cos(float a);
float _normalizeAngle(float angle);
float _electricalAngle(float shaft_angle, int pole_pairs);
float _sqrtApprox(float number);



/******************************************************************************/


#endif


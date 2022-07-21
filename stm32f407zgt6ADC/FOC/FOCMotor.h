#ifndef FOCMOTOR_H
#define FOCMOTOR_H


#include "lowpass_filter.h"
//#include "arm_math.h"
#include "MagneticSensorI2C.h"
#include "MagneticSensorI2C2.h"

/******************************************************************************/
/**
 *  Motiron control type
 */
typedef enum
{
	Type_torque,//!< Torque control
	Type_velocity,//!< Velocity motion control
	Type_angle,//!< Position/angle motion control
	Type_velocity_openloop,
	Type_angle_openloop
} MotionControlType;

/**
 *  Motiron control type
 */
typedef enum
{
	Type_voltage, //!< Torque control using voltage
	Type_dc_current, //!< Torque control using DC current (one current magnitude)
	Type_foc_current //!< torque control using dq currents
} TorqueControlType;

extern TorqueControlType torque_controller;
extern MotionControlType controller;
/******************************************************************************/
extern float shaft_angle;//!< current motor angle
extern float shaft_angle2;//!< current motor angle

extern float electrical_angle;
extern float electrical_angle2;


extern float shaft_velocity;

extern float current_sp;
extern float current_sp2;

extern float shaft_velocity_sp;
extern float shaft_angle_sp;

extern DQVoltage_s voltage;
extern DQVoltage_s voltage2;

extern DQCurrent_s current;
//extern DQCurrent_s current2;

//extern float sensor_offset;
extern float zero_electric_angle;
/******************************************************************************/
float shaftAngle(void);
float shaftAngle2(void);

float shaftVelocity(void);
float shaftVelocity2(void);


float electricalAngle(void);
float electricalAngle2(void);


/******************************************************************************/

#endif


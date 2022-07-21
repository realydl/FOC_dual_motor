#ifndef PID_H
#define PID_H

#include "tim.h"
/******************************************************************************/
typedef struct 
{
    float P; //!< Proportional gain 
    float I; //!< Integral gain 
    float D; //!< Derivative gain 
    float output_ramp; //!< Maximum speed of change of the output value
    float limit; //!< Maximum output value
    float error_prev; //!< last tracking error value
    float output_prev;  //!< last pid output value
    float integral_prev; //!< last integral component value
    unsigned long timestamp_prev; //!< Last execution timestamp
	
		float err_last;//上上次误差记录
	} PIDController;

extern PIDController  PID_current_q,PID_current_d,PID_velocity,P_angle;
extern PIDController  PID_current_q2,PID_current_d2,PID_velocity2,P_angle2;	
	
/******************************************************************************/
void PID_init(void);
void PID_init2(void);
	
	
float PIDoperator(PIDController* PID,float error);//simplefoc
float PID_incremental(PIDController* PID,float error);//增量式
	
/******************************************************************************/

#endif

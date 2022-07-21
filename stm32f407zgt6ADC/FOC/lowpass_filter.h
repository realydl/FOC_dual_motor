#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include "tim.h"
/******************************************************************************/
typedef struct 
{
	float Tf; //!< Low pass filter time constant
	float y_prev; //!< filtered value in previous execution step 
	unsigned long timestamp_prev;  //!< Last execution timestamp
} LowPassFilter;

extern LowPassFilter  LPF_current_q,LPF_current_d,LPF_velocity;
extern LowPassFilter  LPF_current_q2,LPF_current_d2,LPF_velocity2;

extern LowPassFilter  testIU;
/******************************************************************************/
void LPF_init(void);
void LPF_init2(void);

float LPFoperator(LowPassFilter* LPF,float x);
/******************************************************************************/

#endif

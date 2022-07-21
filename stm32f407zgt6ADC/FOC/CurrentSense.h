#ifndef CURRENTSENSE_H
#define CURRENTSENSE_H

/******************************************************************************/
//#include "main.h"
//#include "foc_utils.h"

#include "InlineCurrentSense.h"

/******************************************************************************/
float getDCCurrent(float motor_electrical_angle);

//DQCurrent_s getFOCCurrents(float angle_el,uint8_t sec);
DQCurrent_s getFOCCurrents(float angle_el);
DQCurrent_s getFOCCurrents2(float angle_el);

Ialphabeta clarkTransform(float angle_el,uint8_t sec);

/******************************************************************************/


#endif

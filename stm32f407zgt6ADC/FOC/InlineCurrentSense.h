#ifndef INLINE_CS_LIB_H
#define INLINE_CS_LIB_H

#include "foc_utils.h" 
#include "stm32f4xx_hal.h"
//#include "main.h"

/******************************************************************************/
void InlineCurrentSense(float _shunt_resistor, float _gain);
void InlineCurrentSense2(float _shunt_resistor, float _gain);

//void InlineCurrentSense_Init(void);
PhaseCurrent_s getSelectionPhaseCurrents(uint8_t sector);


void Current_calibrateOffsets(void);
void Current_calibrateOffsets2(void);

PhaseCurrent_s getPhaseCurrents(void);
PhaseCurrent_s getPhaseCurrents2(void);

/******************************************************************************/


#endif

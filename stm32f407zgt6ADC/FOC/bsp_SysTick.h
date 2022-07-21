#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f4xx.h"

typedef uint32_t  u32;

void SysTick_Init(void);
void delay_us(__IO u32 nTime);

//#define delay_ms(x) delay_us(1000*x)	 //µ¥Î»ms

#endif /* __SYSTICK_H */

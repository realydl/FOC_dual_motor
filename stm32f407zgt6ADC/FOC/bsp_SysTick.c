/**
  ******************************************************************************
  * @file    bsp_SysTick.c
  * @author  fire
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   SysTick ϵͳ�δ�ʱ��10us�жϺ�����,�ж�ʱ����������ã�
  *          ���õ��� 1us 10us 1ms �жϡ�     
  ******************************************************************************
  */
  
#include "bsp_SysTick.h"

static __IO u32 TimingDelay;
 
/**
  * @brief  ����ϵͳ�δ�ʱ�� SysTick
  * @param  ��
  * @retval ��
  */
void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms�ж�һ��
	 * SystemFrequency / 100000	 10us�ж�һ��
	 * SystemFrequency / 1000000 1us�ж�һ��
	 */
	if (HAL_SYSTICK_Config(SystemCoreClock / 1000000))
	{ 
		/* Capture error */ 
		while (1);
	}
}

//void SysTick_Init_ms(void)
//{
//	/* SystemFrequency / 1000    1ms�ж�һ��
//	 * SystemFrequency / 100000	 10us�ж�һ��
//	 * SystemFrequency / 1000000 1us�ж�һ��
//	 */
//	if (HAL_SYSTICK_Config(SystemCoreClock / 1000))
//	{ 
//		/* Capture error */ 
//		while (1);
//	}
//}



///**
//  * @brief   us��ʱ����,1usΪһ����λ
//  * @param  
//  *		@arg nTime: Delay_us( 1 ) ��ʵ�ֵ���ʱΪ 1 * 1us = 1us
//  * @retval  ��
//  */
//void delay_us(__IO u32 nTime)
//{ 
//	TimingDelay = nTime;	

//	while(TimingDelay != 0);
//}

///**
//  * @brief  ��ȡ���ĳ���
//  * @param  ��
//  * @retval ��
//  * @attention  �� SysTick �жϺ��� SysTick_Handler()����
//  */
//void TimingDelay_Decrement(void)
//{
//	if (TimingDelay != 0x00)
//	{ 
//		TimingDelay--;
//	}
//}

//void Delay(__IO uint32_t nCount)
//{
//  for(; nCount != 0; nCount--);
//}



/*********************************************END OF FILE**********************/

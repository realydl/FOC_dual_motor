#include "mydelay.h"

////查询定时器方法产生延迟
////延时nus
//void my_delay_us(unsigned long nus)
//{
//	unsigned long temp;
//	
////	RCC_ClocksTypeDef clocks;
////	RCC_GetClocksFreq(&clocks);
//	
////	SysTick->LOAD = nus*9;   //9=针对72MHz
//	SysTick->LOAD = nus*21;   //168=针对168MHz
//	SysTick->VAL =0; 
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;   //HCLK/8
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&!(temp&(1<<16)));

//	SysTick->CTRL=0;//SysTick控制及状态寄存器
//	SysTick->VAL =0;//SysTick当前数值寄存器
//}

////延时nms
////F103最大延时时间=0xFFFFFF/9MHz=1864ms
////F407最大延时时间=0xFFFFFF/168MHz/8 = 0.798915s
//void my_delay_ms(unsigned short nms)
//{
//	unsigned long temp;

////	SysTick->LOAD=(u32)nms*9000;   //9=针对72MHz
//	SysTick->LOAD=(uint32_t)nms*168000000; //168000000/8=针对168MHz
//	SysTick->VAL =0;	
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;  //HCLK/8
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&!(temp&(1<<16)));

//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;
//	SysTick->VAL =0;
//}


//防止一直开关定时器
void my_delay_us(uint16_t nus)	//TIM2定时频率84MHz/4/65535   
{
		uint32_t Count = 0;
		__HAL_TIM_SetCounter(&htim2, 0);//htim1

  	__HAL_TIM_ENABLE(&htim2);
//		HAL_TIM_Base_Start(&htim2);
	
		while(__HAL_TIM_GetCounter(&htim2) < (21 * nus));
//		while(Count < nus){
//			if(__HAL_TIM_GetCounter(&htim2)==3)
//			Count++;
//		}
		/* Disable the Peripheral */
		__HAL_TIM_DISABLE(&htim2);
//		HAL_TIM_Base_Stop(&htim2);
}

void my_delay_ms(uint16_t nms)	//TIM3定时频率84MHz/4000/65535   
{
		uint32_t Count = 0;
		__HAL_TIM_SetCounter(&htim3, 0);//htim1

  	__HAL_TIM_ENABLE(&htim3);
//		HAL_TIM_Base_Start(&htim2);
	
		while(__HAL_TIM_GetCounter(&htim3) < (21 * nms));
//		while(Count < nms){
//			if(__HAL_TIM_GetCounter(&htim2)==3)
//				Count++;
//		}
	
		/* Disable the Peripheral */
		__HAL_TIM_DISABLE(&htim3);
//		HAL_TIM_Base_Stop(&htim2);
}

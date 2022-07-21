#include "mydelay.h"

////��ѯ��ʱ�����������ӳ�
////��ʱnus
//void my_delay_us(unsigned long nus)
//{
//	unsigned long temp;
//	
////	RCC_ClocksTypeDef clocks;
////	RCC_GetClocksFreq(&clocks);
//	
////	SysTick->LOAD = nus*9;   //9=���72MHz
//	SysTick->LOAD = nus*21;   //168=���168MHz
//	SysTick->VAL =0; 
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;   //HCLK/8
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&!(temp&(1<<16)));

//	SysTick->CTRL=0;//SysTick���Ƽ�״̬�Ĵ���
//	SysTick->VAL =0;//SysTick��ǰ��ֵ�Ĵ���
//}

////��ʱnms
////F103�����ʱʱ��=0xFFFFFF/9MHz=1864ms
////F407�����ʱʱ��=0xFFFFFF/168MHz/8 = 0.798915s
//void my_delay_ms(unsigned short nms)
//{
//	unsigned long temp;

////	SysTick->LOAD=(u32)nms*9000;   //9=���72MHz
//	SysTick->LOAD=(uint32_t)nms*168000000; //168000000/8=���168MHz
//	SysTick->VAL =0;	
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;  //HCLK/8
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&!(temp&(1<<16)));

//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;
//	SysTick->VAL =0;
//}


//��ֹһֱ���ض�ʱ��
void my_delay_us(uint16_t nus)	//TIM2��ʱƵ��84MHz/4/65535   
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

void my_delay_ms(uint16_t nms)	//TIM3��ʱƵ��84MHz/4000/65535   
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

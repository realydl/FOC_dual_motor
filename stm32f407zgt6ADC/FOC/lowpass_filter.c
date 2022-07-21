#include "lowpass_filter.h"

/******************************************************************************/
LowPassFilter  LPF_current_q,LPF_current_d,LPF_velocity;

LowPassFilter  LPF_current_q2,LPF_current_d2,LPF_velocity2;
//LowPassFilter  testIU;
/******************************************************************************/
void LPF_init(void)
{
	LPF_current_q.Tf=0.05;    
	LPF_current_q.y_prev=0;
	LPF_current_q.timestamp_prev=0;  //SysTick->VAL;
	
	LPF_current_d.Tf=0.05;
	LPF_current_d.y_prev=0;
	LPF_current_d.timestamp_prev=0;
	
	LPF_velocity.Tf=0.05;   //0.02 Tf太小电机容易抖动
	LPF_velocity.y_prev=0;
	LPF_velocity.timestamp_prev=0;
	
//	testIU.Tf=0.05;    
//	testIU.y_prev=0;
//	testIU.timestamp_prev=0;  //SysTick->VAL;
}

void LPF_init2(void)
{
	LPF_current_q2.Tf=0.05;    
	LPF_current_q2.y_prev=0;
	LPF_current_q2.timestamp_prev=0;  //SysTick->VAL;
	
	LPF_current_d2.Tf=0.05;
	LPF_current_d2.y_prev=0;
	LPF_current_d2.timestamp_prev=0;
	
	LPF_velocity2.Tf=0.05;   //0.02 Tf太小电机容易抖动
	LPF_velocity2.y_prev=0;
	LPF_velocity2.timestamp_prev=0;
	
//	testIU.Tf=0.05;    
//	testIU.y_prev=0;
//	testIU.timestamp_prev=0;  //SysTick->VAL;
}


/******************************************************************************/
float LPFoperator(LowPassFilter* LPF,float x)
{
	unsigned long now_us;
	float dt, alpha, y;
	
//	now_us = SysTick->VAL;
//	if(now_us < LPF->timestamp_prev)dt = (float)(LPF->timestamp_prev - now_us)/6*1e-9f;
//	else
//		dt = (float)(0xFFFFFF - now_us + LPF->timestamp_prev)/6*1e-9f;
//	LPF->timestamp_prev = now_us;
//	if(dt > 0.3f)   //时间过长，大概是程序刚启动初始化，直接返回
//	{
//		LPF->y_prev = x;
//		return x;
//	}
	
	now_us = __HAL_TIM_GetCounter(&htim4);
	if(now_us < LPF->timestamp_prev) 
		dt = (float)(LPF->timestamp_prev - now_us)*1e-6f;
	else
		dt = (float)(0x0000FFFF - now_us + LPF->timestamp_prev)*1e-6f;
	LPF->timestamp_prev = now_us;
	if(dt > 0.3f)   //时间过长，大概是程序刚启动初始化，直接返回
	{
		LPF->y_prev = x;
		return x;
	}
	
	alpha = LPF->Tf/(LPF->Tf + dt);
	y = alpha*LPF->y_prev + (1.0f - alpha)*x;
	LPF->y_prev = y;
	
	return y;
}
/******************************************************************************/



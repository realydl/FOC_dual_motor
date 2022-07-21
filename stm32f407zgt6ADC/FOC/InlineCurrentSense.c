#include "InlineCurrentSense.h"

int pinA,pinB,pinC;

float gain_a,gain_b,gain_c;
float gain_a2,gain_b2,gain_c2;

float offset_ia,offset_ib,offset_ic;
float offset_ia2,offset_ib2,offset_ic2;

extern float adc_u;
extern float adc_v;
extern float adc_w;

extern float adc_u2;
extern float adc_v2;
extern float adc_w2;


extern __IO uint16_t ChannelPulse;


void InlineCurrentSense(float _shunt_resistor, float _gain)
{
	float volts_to_amps_ratio;
	
//	pinA = _pinA;//U
//	pinB = _pinB;//V
//	pinC = _pinC;//W
	
	volts_to_amps_ratio = 1.0f /_shunt_resistor / _gain; // volts to amps
	
#if 1
	gain_a = volts_to_amps_ratio;
	gain_b = -volts_to_amps_ratio;//应该和电流放大器芯片的连接有关
	gain_c = volts_to_amps_ratio;
#endif

#if 0
	gain_a = -volts_to_amps_ratio;
	gain_b = volts_to_amps_ratio;//应该和电流放大器芯片的连接有关
	gain_c = volts_to_amps_ratio;
#endif
	
	printf("gain_a:%.2f,gain_b:%.2f,gain_c:%.2f.\r\n",gain_a,gain_b,gain_c);
}

void InlineCurrentSense2(float _shunt_resistor, float _gain)
{
	float volts_to_amps_ratio;
	
//	pinA = _pinA;//U
//	pinB = _pinB;//V
//	pinC = _pinC;//W
	
	volts_to_amps_ratio = 1.0f /_shunt_resistor / _gain; // volts to amps
	
#if 1
	gain_a2 = volts_to_amps_ratio;
	gain_b2 = -volts_to_amps_ratio;//应该和电流放大器芯片的连接有关
	gain_c2 = volts_to_amps_ratio;
#endif

#if 0
	gain_a = -volts_to_amps_ratio;
	gain_b = volts_to_amps_ratio;//应该和电流放大器芯片的连接有关
	gain_c = volts_to_amps_ratio;
#endif
	
	printf("gain_a:%.2f,gain_b:%.2f,gain_c:%.2f.\r\n",gain_a,gain_b,gain_c);
}


PhaseCurrent_s current_ab;
// read all three phase currents (if possible 2 or 3)
PhaseCurrent_s getPhaseCurrents(void)
{

	current_ab.a = (adc_u - offset_ia)*gain_a;// amps
	current_ab.b = (adc_v - offset_ib)*gain_b;// amps
	current_ab.c = -current_ab.a - current_ab.b;
	
//	//a->c gainA=sign(C)*gain(A)
//		current_ab.c = (adc_u - offset_ia)*gain_a;// amps
//		current_ab.b = (adc_v - offset_ib)*gain_b;// amps
//		current_ab.a = (-current_ab.b - current_ab.c);
	

	
//	current.c = (adc_w - offset_ic)*gain_c*1000;//
	
//	current.a = (adc_u - 1.25)*gain_a*1000;// amps
//	current.b = (adc_v - 1.25)*gain_b*1000;// amps
//	current.c = (adc_w - 1.25)*gain_c*1000;//
//	current.c = (!_isset(pinC)) ? 0 : (_readADCVoltageInline(pinC) - offset_ic)*gain_c; // amps
	
	return current_ab;
}

PhaseCurrent_s current_ab2;

PhaseCurrent_s getPhaseCurrents2(void)
{

	current_ab2.a = (adc_u2 - offset_ia2)*gain_a2;// amps
	current_ab2.b = (adc_v2 - offset_ib2)*gain_b2;// amps
	current_ab2.c = -current_ab2.a - current_ab2.b;
	return current_ab2;
}

//上电之后检测电流偏置
void Current_calibrateOffsets(void)
{
	int i;
	
	offset_ia=0;
	offset_ib=0;
	offset_ic=0;
	// read the adc voltage 1000 times ( arbitrary number )
	for(i=0; i<500; i++)
	{
		offset_ia += adc_u;
		offset_ib += adc_v;
		offset_ic += adc_w;
//		if(_isset(pinC)) offset_ic += _readADCVoltageInline(pinC);
		my_delay_ms(1);
	}
	// calculate the mean offsets
	offset_ia = offset_ia/500.0f;
	offset_ib = offset_ib/500.0f;
	offset_ic = offset_ic/500.0f;
//	if(_isset(pinC)) offset_ic = offset_ic / 1000;
	
	printf("offset_ia:%.4f,offset_ib:%.4f,offset_ic:%.4f.\r\n",offset_ia,offset_ib,offset_ic);
}


//上电之后检测电流偏置
void Current_calibrateOffsets2(void)
{
	int i;
	
	offset_ia2=0;
	offset_ib2=0;
	offset_ic2=0;
	// read the adc voltage 1000 times ( arbitrary number )
	for(i=0; i<500; i++)
	{
		offset_ia2 += adc_u2;
		offset_ib2 += adc_v2;
		offset_ic2 += adc_w2;
//		if(_isset(pinC)) offset_ic += _readADCVoltageInline(pinC);
		my_delay_ms(1);
	}
	// calculate the mean offsets
	offset_ia2 = offset_ia2/500.0f;
	offset_ib2 = offset_ib2/500.0f;
	offset_ic2 = offset_ic2/500.0f;
//	if(_isset(pinC)) offset_ic = offset_ic / 1000;
	
	printf("offset_ia2:%.4f,offset_ib2:%.4f,offset_ic2:%.4f.\r\n",offset_ia2,offset_ib2,offset_ic2);
}



// read all three phase currents (if possible 2 or 3)
//phase selection
PhaseCurrent_s getSelectionPhaseCurrents(uint8_t sector)
{
	PhaseCurrent_s current;
	//互补，下侧端检测，上通下断
	//U->W,V->V,W->U
	switch(sector)
	{
		case 1://(W>V)
			//ch1- ch3+
			current.b = (adc_v - offset_ib)*gain_b;//V
			current.c = (adc_w - offset_ic)*gain_c;//W
			current.a = -current.b-current.c;//U
			break;
		case 2://(W>U)
			//ch3+ ch2-
			current.a = (adc_u - offset_ia)*gain_a;//U
			current.c = (adc_w - offset_ic)*gain_c;//W
			current.b = -current.a-current.c;//V
			break;
		case 3://(U>W)
			//ch1+ ch2-
			current.a = (adc_u - offset_ia)*gain_a;//U
			current.c = (adc_w - offset_ic)*gain_c;//W
			current.b = -current.a-current.c;//V
			break;
		case 4://(U>V)
			//ch1+ ch3-
			current.a = (adc_u - offset_ia)*gain_a;//U
			current.b = (adc_v - offset_ib)*gain_b;//V
			current.c = -current.a-current.b;//W
			break;
		case 5://(V>U)
			//ch2+ ch3-
			current.a = (adc_u - offset_ia)*gain_a;//U
			current.b = (adc_v - offset_ib)*gain_b;//V
			current.c = -current.a-current.b;//W
			break;
		case 6://(V>W)
			//ch1- ch2+
			current.b = (adc_v - offset_ib)*gain_b;//V
			current.c = (adc_w - offset_ic)*gain_c;//W
			current.a = -current.b-current.c;//U
			break;
		default:
			printf("Error：Current sampling anomaly！！！");
	}	
//	current.a = (adc_u - offset_ia)*gain_a;// amps
//	current.b = (adc_v - offset_ib)*gain_b;// amps
//	current.c = (adc_w - offset_ic)*gain_c;//
	
//	current.c = (!_isset(pinC)) ? 0 : (_readADCVoltageInline(pinC) - offset_ic)*gain_c; // amps
	
	return current;
}






// Function aligning the current sense with motor driver
 // if all pins are connected well none of this is really necessary! - can be avoided
 // returns flag
 // 0 - fail
 // 1 - success and nothing changed
 // 2 - success but pins reconfigured
 // 3 - success but gains inverted
 // 4 - success but pins reconfigured and gains inverted
	/*
	0 - 失败
	1 - 成功了，且没有改变其他配置
	2 - 成功，但引脚重新配置
	3 - 成功，但增益相反
	4 - 成功，但引脚重新配置且增益相反
	*/
int driverAlign(TIM_HandleTypeDef* driver, float voltage){
	int exit_flag = 1;
//	if(skip_align) return exit_flag;//跳过上电检测

	// set phase A active and phases B and C down
//	driver->setPwm(voltage, 0, 0);
	
	__HAL_TIM_SetCompare(driver,TIM_CHANNEL_1,voltage);
	__HAL_TIM_SetCompare(driver,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(driver,TIM_CHANNEL_3,0);
	my_delay_ms(200); 
	PhaseCurrent_s c = getPhaseCurrents();
	// read the current 100 times ( arbitrary number )
	for (int i = 0; i < 100; i++) {
		 PhaseCurrent_s c1 = getPhaseCurrents();
		 c.a = c.a*0.6f + 0.4f*c1.a;
		 c.b = c.b*0.6f + 0.4f*c1.b;
		 c.c = c.c*0.6f + 0.4f*c1.c;
		 my_delay_ms(3);
	}
//	driver->setPwm(0, 0, 0);
	__HAL_TIM_SetCompare(driver,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(driver,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(driver,TIM_CHANNEL_3,0);
	
	// align phase A
	float ab_ratio = fabs(c.a / c.b);
	float ac_ratio = c.c ? fabs(c.a / c.c) : 0;//c>0
	if( ab_ratio > 1.5f ){ // should be ~2    
		 gain_a *= _sign(c.a);
	}else if( ab_ratio < 0.7f ){ // should be ~0.5
		 // switch phase A and B
		 int tmp_pinA = pinA;
		 pinA = pinB; 
		 pinB = tmp_pinA;
		 gain_a *= _sign(c.b);
		 exit_flag = 2; // signal that pins have been switched
//	}else if(_isset(pinC) &&  ac_ratio < 0.7f ){ // should be ~0.5
		}else if( ac_ratio < 0.7f ){ // should be ~0.5
		 // switch phase A and C
		 int tmp_pinA = pinA;
		 pinA = pinC; 
		 pinC= tmp_pinA;
		 gain_a *= _sign(c.c);
		 exit_flag = 2;// signal that pins have been switched
	}else{
		 // error in current sense - phase either not measured or bad connection
		 return 0;
	}

	// set phase B active and phases A and C down
//	driver->setPwm(0, voltage, 0);
	
	__HAL_TIM_SetCompare(driver,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(driver,TIM_CHANNEL_2,voltage);
	__HAL_TIM_SetCompare(driver,TIM_CHANNEL_3,0);
	my_delay_ms(200); 
	c = getPhaseCurrents();
	// read the current 50 times
	for (int i = 0; i < 100; i++) {
		 PhaseCurrent_s c1 = getPhaseCurrents();
		 c.a = c.a*0.6f + 0.4f*c1.a;
		 c.b = c.b*0.6f + 0.4f*c1.b;
		 c.c = c.c*0.6f + 0.4f*c1.c;
		 my_delay_ms(3);
	}
//	driver->setPwm(0, 0, 0);
	
	__HAL_TIM_SetCompare(driver,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(driver,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(driver,TIM_CHANNEL_3,0);
	
	float ba_ratio = fabs(c.b/c.a);
	float bc_ratio = c.c ? fabs(c.b / c.c) : 0;
	if( ba_ratio > 1.5f ){ // should be ~2
		 gain_b *= _sign(c.b);
	}else if( ba_ratio < 0.7f ){ // it should be ~0.5
		 // switch phase A and B
		 int tmp_pinB = pinB;
		 pinB = pinA; 
		 pinA = tmp_pinB;
		 gain_b *= _sign(c.a);
		 exit_flag = 2; // signal that pins have been switched
//	}else if(_isset(pinC) && bc_ratio < 0.7f ){ // should be ~0.5
		}else if( bc_ratio < 0.7f ){ // should be ~0.5
		 // switch phase A and C
		 int tmp_pinB = pinB;
		 pinB = pinC; 
		 pinC = tmp_pinB;
		 gain_b *= _sign(c.c);
		 exit_flag = 2; // signal that pins have been switched
	}else{
		 // error in current sense - phase either not measured or bad connection
		 return 0;
	}
/*********************************************************************************/
	// if phase C measured
	if(_isset(pinC)){
		 // set phase B active and phases A and C down
//		 driver->setPwm(0, 0, voltage);
		
		__HAL_TIM_SetCompare(driver,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(driver,TIM_CHANNEL_2,0);
		__HAL_TIM_SetCompare(driver,TIM_CHANNEL_3,voltage);
		 my_delay_ms(200); 
		 c = getPhaseCurrents();
		 // read the adc voltage 500 times ( arbitrary number )
		 for (int i = 0; i < 50; i++) {
				 PhaseCurrent_s c1 = getPhaseCurrents();
				 c.c = (c.c+c1.c)/50.0f;
		 }
//		 driver->setPwm(0, 0, 0);
			 
		__HAL_TIM_SetCompare(driver,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(driver,TIM_CHANNEL_2,0);
		__HAL_TIM_SetCompare(driver,TIM_CHANNEL_3,0);
		 gain_c *= _sign(c.c);
	}

	if(gain_a < 0 || gain_b < 0 || gain_c < 0) exit_flag +=2;
	// exit flag is either
	// 0 - fail
	// 1 - success and nothing changed
	// 2 - success but pins reconfigured
	// 3 - success but gains inverted
	// 4 - success but pins reconfigured and gains inverted

	return exit_flag;
}
  



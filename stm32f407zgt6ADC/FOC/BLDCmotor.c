#include "BLDCmotor.h"
//#include "foc_utils.h"
//#include "tim.h"

//extern __IO uint16_t ChannelPulse;
//extern __IO uint16_t ChannelPulse2;

__IO uint16_t ChannelPulse = 2100-1;//100%占空比
__IO uint16_t ChannelPulse2 = 2100-1;//100%占空比


extern float target;

extern float shaft_velocity;

extern long sensor_direction;
extern long sensor_direction2;

extern float voltage_power_supply;
extern float voltage_limit;

extern float voltage_sensor_align;
extern float voltage_sensor_align2;

extern int  pole_pairs;
//unsigned long open_loop_timestamp;
extern float velocity_limit;
extern float current_limit;

extern float targetId;
extern float targetIq;

extern float zero_angle;

extern float angle_prev2;
extern float shaft_velocity2;
extern float shaft_angle2;

extern float zero_electric_angle2;


//void Motor_init(void)
//{
//	printf("MOT: Init\r\n");
//	
//	if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;
//	PID_current_q.limit = voltage_limit;
//	PID_current_d.limit = voltage_limit;
//}

void Motor_init(void)
{
	printf("MOT: Init\r\n");
	
	if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;
	
	// current control loop controls voltage
	PID_current_q.limit = voltage_limit;
	PID_current_d.limit = voltage_limit;
	// velocity control loop controls current
	// 如果是电压模式限制电压，如果是电流模式限制电流
//	if(torque_controller == Type_voltage)PID_velocity.limit = voltage_limit;  //速度模式的电流限制
	PID_velocity.limit = current_limit;
	P_angle.limit = velocity_limit;      //角度模式的速度限制
	
	M1_Enable;
//	my_delay_ms(1);
	printf("MOT: Enable driver.\r\n");
}

void Motor_init2(void)
{
	printf("MOT2: Init\r\n");
	
	if(voltage_sensor_align2 > voltage_limit) voltage_sensor_align2 = voltage_limit;
	
	// current control loop controls voltage
	PID_current_q2.limit = voltage_limit;
	PID_current_d2.limit = voltage_limit;
	// velocity control loop controls current
	// 如果是电压模式限制电压，如果是电流模式限制电流
//	if(torque_controller == Type_voltage)PID_velocity.limit = voltage_limit;  //速度模式的电流限制
	PID_velocity2.limit = current_limit;
	P_angle2.limit = velocity_limit;      //角度模式的速度限制
	
	M2_Enable;
//	my_delay_ms(1);
	printf("MOT2: Enable driver.\r\n");
}

void Motor_initFOC(float zero_electric_offset, Direction _sensor_direction)
{
	//int exit_flag = 1;
	
	if(zero_electric_offset!=0)
	{
    // abosolute zero offset provided - no need to align
    zero_electric_angle = zero_electric_offset;
    // set the sensor direction - default CW
    sensor_direction = _sensor_direction;
  }
	alignSensor();    //检测零点偏移量和极对数
	
	//shaft_angle update
	angle_prev = getAngle();  //getVelocity(),make sure velocity=0 after power on
	my_delay_ms(50);
	shaft_velocity = shaftVelocity();  //必须调用一次，进入主循环后速度为0
	my_delay_ms(5);
	shaft_angle = shaftAngle();// shaft angle
//	if(controller==Type_angle)
//		target = shaft_angle;//角度模式，以当前的角度为目标角度，进入主循环后电机静止
	
	my_delay_ms(200);
}

void Motor_initFOC2(float zero_electric_offset, Direction _sensor_direction)
{
	//int exit_flag = 1;
	
	if(zero_electric_offset!=0)
	{
    // abosolute zero offset provided - no need to align
    zero_electric_angle2 = zero_electric_offset;
    // set the sensor direction - default CW
    sensor_direction2 = _sensor_direction;
  }
	alignSensor2();    //检测零点偏移量和极对数
	
	//shaft_angle update
	angle_prev2 = getAngle2();  //getVelocity(),make sure velocity=0 after power on
	my_delay_ms(50);
	shaft_velocity2 = shaftVelocity2();  //必须调用一次，进入主循环后速度为0
	my_delay_ms(5);
	shaft_angle2 = shaftAngle2();// shaft angle
//	if(controller==Type_angle)
//		target = shaft_angle;//角度模式，以当前的角度为目标角度，进入主循环后电机静止
	
	my_delay_ms(200);
}

int alignSensor(void)
{
	long i;
	float angle;
	float mid_angle,end_angle;
	float moved;
	
	printf("MOT: Align sensor.\r\n");
	
	if(sensor_direction == UNKNOWN)  //没有设置，需要检测
	{
		// find natural direction
		// move one electrical revolution forward
		
		setPhaseVoltage(0, voltage_sensor_align, 0);
		my_delay_ms(200);
		
		for(i = 0; i<=500; i++)
		{
//			angle = _3PI_2 + _2PI * i / 500.0f;
			
			angle = _2PI* i / 500.0f;
//			setPhaseVoltage(voltage_sensor_align, 0,  angle);
			setPhaseVoltage(0, voltage_sensor_align,  angle);
			my_delay_ms(2);
		}
		
		setPhaseVoltage(0, voltage_sensor_align, _2PI);
		my_delay_ms(200);
		
		mid_angle = getAngle();
		
		for(i = 500; i>=0; i--) 
		{
			//angle = _3PI_2 + _2PI * i / 500.0f ;
			angle = _2PI * i / 500.0f ;
//			setPhaseVoltage(voltage_sensor_align, 0,  angle);
			setPhaseVoltage(0, voltage_sensor_align,  angle);
			my_delay_ms(2);
		}
		end_angle = getAngle();
		setPhaseVoltage(0, 0, 0);
		my_delay_ms(200);
		
		printf("mid_angle=%.4f\r\n",mid_angle);
		printf("end_angle=%.4f\r\n",end_angle);
		
		moved = fabs(mid_angle - end_angle);
		if((mid_angle == end_angle)||(moved < 0.01f))  //相等或者几乎没有动
		{
			printf("MOT: Failed to notice movement\r\n");
			
			M1_Disable;    //电机1检测不正常，关闭驱动
			return 0;
		}
		else if(mid_angle < end_angle)
		{
			printf("MOT: sensor_direction==CCW\r\n");
			sensor_direction=CCW;
//			sensor_direction=CW;
		}
		else
		{
			printf("MOT: sensor_direction==CW\r\n");
			sensor_direction=CW;
//			sensor_direction=CCW;
		}
		
		printf("MOT: PP check: ");    //计算Pole_Pairs
		if( fabs(moved*pole_pairs - _2PI) > 0.5f )  // 0.5 is arbitrary number it can be lower or higher!
		{
			printf("fail - estimated pp:");
			pole_pairs = (int)(_2PI/moved + 0.5f);     //浮点数转整形，四舍五入
			printf("%d\r\n",pole_pairs);
		}
		else printf("OK!\r\n");
	}
	else
		printf("MOT: Skip dir calib.\r\n");
	
	if(zero_electric_angle == 0)  //没有设置，需要检测
	{
//		setPhaseVoltage(voltage_sensor_align, 0,  _3PI_2);  //计算零点偏移角度	
		//setPhaseVoltage(voltage_sensor_align, 0,  _PI_2);  //计算零点偏移角度
		
		setPhaseVoltage(0, voltage_sensor_align,  0);  //计算零点偏移角度
		
//		//Ud基准0
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0.4*ChannelPulse);
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);
//		my_delay_ms(500);
//	
//		SecondAngle = shaftAngle();
//		sensor_offset = shaftAngle();// shaft angle
			
		my_delay_ms(700);
		zero_electric_angle = _normalizeAngle(_electricalAngle(sensor_direction*getAngle(), pole_pairs));
		zero_angle = _normalizeAngle(sensor_direction*getAngle());
		my_delay_ms(20);
		printf("MOT: Zero elec. angle:");
		printf("%.4f\r\n",zero_electric_angle);
		setPhaseVoltage(0, 0, 0);
		my_delay_ms(200);
	}
	else
		printf("MOT: Skip offset calib.\r\n");
	
	return 1;
}

int alignSensor2(void)
{
	long i;
	float angle;
	float mid_angle,end_angle;
	float moved;
	
	printf("MOT2: Align sensor.\r\n");
	
	if(sensor_direction2 == UNKNOWN)  //没有设置，需要检测
	{
		// find natural direction
		// move one electrical revolution forward
		
		setPhaseVoltage2(0, voltage_sensor_align2, 0);
		my_delay_ms(100);
		
		for(i = 0; i<=500; i++)
		{
//			angle = _3PI_2 + _2PI * i / 500.0f;
			
			angle = _2PI* i / 500.0f;
//			setPhaseVoltage(voltage_sensor_align, 0,  angle);
			setPhaseVoltage2(0, voltage_sensor_align2,  angle);
			my_delay_ms(2);
		}
		
		setPhaseVoltage2(0, voltage_sensor_align2, _2PI);
		my_delay_ms(100);
		
		mid_angle = getAngle2();
		
		for(i = 500; i>=0; i--) 
		{
			//angle = _3PI_2 + _2PI * i / 500.0f ;
			angle = _2PI * i / 500.0f ;
//			setPhaseVoltage(voltage_sensor_align, 0,  angle);
			setPhaseVoltage2(0, voltage_sensor_align2,  angle);
			my_delay_ms(2);
		}
		end_angle = getAngle2();
		setPhaseVoltage2(0, 0, 0);
		my_delay_ms(200);
		
		printf("mid_angle2=%.4f\r\n",mid_angle);
		printf("end_angle2=%.4f\r\n",end_angle);
		
		moved = fabs(mid_angle - end_angle);
		if((mid_angle == end_angle)||(moved < 0.01f))  //相等或者几乎没有动
		{
			printf("MOT2: Failed to notice movement\r\n");
			
			M2_Disable;    //电机2检测不正常，关闭驱动
			return 0;
		}
		else if(mid_angle < end_angle)
		{
			printf("MOT2: sensor_direction==CCW\r\n");
			sensor_direction2=CCW;
//			sensor_direction=CW;
		}
		else
		{
			printf("MOT2: sensor_direction==CW\r\n");
			sensor_direction2=CW;
//			sensor_direction=CCW;
		}
		
		printf("MOT2: PP check: ");    //计算Pole_Pairs
		if( fabs(moved*pole_pairs - _2PI) > 0.5f )  // 0.5 is arbitrary number it can be lower or higher!
		{
			printf("fail - estimated pp:");
			pole_pairs = (int)(_2PI/moved + 0.5f);     //浮点数转整形，四舍五入
			printf("%d\r\n",pole_pairs);
		}
		else printf("OK!\r\n");
	}
	else
		printf("MOT2: Skip dir calib.\r\n");
	
	if(zero_electric_angle2 == 0)  //没有设置，需要检测
	{
//		setPhaseVoltage(voltage_sensor_align, 0,  _3PI_2);  //计算零点偏移角度	
		//setPhaseVoltage(voltage_sensor_align, 0,  _PI_2);  //计算零点偏移角度
		
		setPhaseVoltage2(0, voltage_sensor_align2,  0);  //计算零点偏移角度
		
//		//Ud基准0
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0.4*ChannelPulse);
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);
//		my_delay_ms(500);
//	
//		SecondAngle = shaftAngle();
//		sensor_offset = shaftAngle();// shaft angle
			
		my_delay_ms(700);
		zero_electric_angle2 = _normalizeAngle(_electricalAngle(sensor_direction2*getAngle2(), pole_pairs));
//		zero_angle = _normalizeAngle(sensor_direction*getAngle());
		my_delay_ms(20);
		printf("MOT2: Zero elec. angle:");
		printf("%.4f\r\n",zero_electric_angle2);
		setPhaseVoltage2(0, 0, 0);
		my_delay_ms(200);
	}
	else
		printf("MOT2: Skip offset calib.\r\n");
	
	return 1;
}


//设置相电压
//第三个参数为电角度 
//返回当前扇区
uint8_t setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	float Uout;
	uint8_t sector;
	uint8_t sector2;//用于电流采样
	
	float T0,T1,T2;
	float Ta,Tb,Tc;
	float a,b,c;
	
//	if(Uq < 0) 
//		angle_el += _PI;
//	
//	 Uq = fabs(Uq);
	
	if(Uq> voltage_limit)Uq= voltage_limit;
	if(Uq<-voltage_limit)Uq=-voltage_limit;
	if(Ud> voltage_limit)Ud= voltage_limit;
	if(Ud<-voltage_limit)Ud=-voltage_limit;
	
	sector2 = (angle_el / _PI_3) + 1;
	
	if(Ud) // only if Ud and Uq set 
	{// _sqrt is an approx of sqrt (3-4% error)
		Uout = _sqrt(Ud*Ud + Uq*Uq) / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));//电角度加上UqUd之间的夹角
//		angle_el = _normalizeAngle(angle_el - atan2(Uq, Ud));//电角度加上UqUd之间的夹角
	}
	else
	{// only Uq available - no need for atan2 and sqrt
		Uout = Uq / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + _PI_2);
//		angle_el = _normalizeAngle(angle_el - _PI_2);
	}
	if(Uout> 0.577f)Uout= 0.577f;
	if(Uout<-0.577f)Uout=-0.577f;
	
	sector = (angle_el / _PI_3) + 1;//直接通过换算后的角度来计算扇区
	
//	T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uout;
//	T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uout;	
	
	//F407DSP
	T1 = _SQRT3*arm_sin_f32(sector*_PI_3 - angle_el) * Uout;
	T2 = _SQRT3*arm_sin_f32(angle_el - (sector-1.0)*_PI_3) * Uout;
	
	T0 = 1 - T1 - T2;
	// calculate the duty cycles(times)
	switch(sector)
	{
		case 1://U,V
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,(Tb-T0/8)*ChannelPulse);
			break;
		case 2://U,V
			Ta = T1 + T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,(Ta-T0/8)*ChannelPulse);
			break;
		case 3://V,W
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,(Tc-T0/8)*ChannelPulse);
			break;
		case 4://V,W
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,(Tb-T0/8)*ChannelPulse);
			break;
		case 5://U,W
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,(Ta-T0)/8*ChannelPulse);
			break;
		case 6://U,W
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,(Tc-T0)/8*ChannelPulse);
			break;
		default:  // possible error state
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}
	
//	Ta = (1-Ta)/2;
//	Tb = (1-Tb)/2;
//	Tc = (1-Tc)/2;
	
	//或者可以使用串口打印波形	
//	a = Ta*10000;
//	b = Tb*10000;
//	c = Tc*10000;
	a = Ta*ChannelPulse;
	b = Tb*ChannelPulse;
	c = Tc*ChannelPulse;
	
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,Ta*ChannelPulse);//U+
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,Tb*ChannelPulse);//V+
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,Tc*ChannelPulse);//W+

	
//	TIM_SetCompare1(TIM2,Ta*PWM_Period);
//	TIM_SetCompare2(TIM2,Tb*PWM_Period);
//	TIM_SetCompare3(TIM2,Tc*PWM_Period);
	return sector2;
}


uint8_t setPhaseVoltage2(float Uq, float Ud, float angle_el)
{
	float Uout;
	uint8_t sector;
	uint8_t sector2;//用于电流采样
	
	float T0,T1,T2;
	float Ta,Tb,Tc;
	float a,b,c;
	
//	if(Uq < 0) 
//		angle_el += _PI;
//	
//	 Uq = fabs(Uq);
	
	if(Uq> voltage_limit)Uq= voltage_limit;
	if(Uq<-voltage_limit)Uq=-voltage_limit;
	if(Ud> voltage_limit)Ud= voltage_limit;
	if(Ud<-voltage_limit)Ud=-voltage_limit;
	
	sector2 = (angle_el / _PI_3) + 1;
	
	if(Ud) // only if Ud and Uq set 
	{// _sqrt is an approx of sqrt (3-4% error)
		Uout = _sqrt(Ud*Ud + Uq*Uq) / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));//电角度加上UqUd之间的夹角
//		angle_el = _normalizeAngle(angle_el - atan2(Uq, Ud));//电角度加上UqUd之间的夹角
	}
	else
	{// only Uq available - no need for atan2 and sqrt
		Uout = Uq / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + _PI_2);
//		angle_el = _normalizeAngle(angle_el - _PI_2);
	}
	if(Uout> 0.577f)Uout= 0.577f;
	if(Uout<-0.577f)Uout=-0.577f;
	
	sector = (angle_el / _PI_3) + 1;//直接通过换算后的角度来计算扇区
	
//	T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uout;
//	T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uout;	
	
	//F407DSP
	T1 = _SQRT3*arm_sin_f32(sector*_PI_3 - angle_el) * Uout;
	T2 = _SQRT3*arm_sin_f32(angle_el - (sector-1.0)*_PI_3) * Uout;
	
	T0 = 1 - T1 - T2;
	// calculate the duty cycles(times)
	switch(sector)
	{
		case 1://U,V
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,(Tb-T0/8)*ChannelPulse);
			break;
		case 2://U,V
			Ta = T1 + T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,(Ta-T0/8)*ChannelPulse);
			break;
		case 3://V,W
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,(Tc-T0/8)*ChannelPulse);
			break;
		case 4://V,W
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,(Tb-T0/8)*ChannelPulse);
			break;
		case 5://U,W
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,(Ta-T0)/8*ChannelPulse);
			break;
		case 6://U,W
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,(Tc-T0)/8*ChannelPulse);
			break;
		default:  // possible error state
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}
	
//	Ta = (1-Ta)/2;
//	Tb = (1-Tb)/2;
//	Tc = (1-Tc)/2;
	
	//或者可以使用串口打印波形	
//	a = Ta*10000;
//	b = Tb*10000;
//	c = Tc*10000;
	a = Ta*ChannelPulse2;
	b = Tb*ChannelPulse2;
	c = Tc*ChannelPulse2;
	
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,Ta*ChannelPulse2);//U
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,Tb*ChannelPulse2);//V
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,Tc*ChannelPulse2);//W

	return sector2;
}

void move(float new_target){
//	voltage.q = new_target;  // if voltage torque control
	current_sp = new_target; // if current/foc_current torque control
}

void move2(float new_target){
//	voltage.q = new_target;  // if voltage torque control
	current_sp2 = new_target; // if current/foc_current torque control
}

float mymove(float new_target){
//	voltage.q = new_target;  // if voltage torque control
	//current_sp = new_target; // if current/foc_current torque control
	current_sp = shaft_angle;
	return current_sp;
}

//位置环模式(双电机力矩互控模式)
float move_position2(float OPP_shaft_angle){
//	shaft_velocity = shaftVelocity();
	shaft_angle = getAngle();
	current_sp = OPP_shaft_angle - shaft_angle;

	return shaft_angle;
}

void move_torque1(void)
{
	current_sp = 2*(shaft_angle2 - shaft_angle);
}

void move_torque2(void)
{
	current_sp2 = 2*(shaft_angle - shaft_angle2);
}


//位置环模式
float move_position(float position_target){
	shaft_velocity = shaftVelocity();
	shaft_angle = getAngle();
	// angle set point
	shaft_angle_sp = position_target;
	// calculate velocity set point
	shaft_velocity_sp = PIDoperator(&P_angle,(shaft_angle_sp - shaft_angle));
	// calculate the torque command
	current_sp = PIDoperator(&PID_velocity,(- shaft_velocity_sp - shaft_velocity)); // if voltage torque control

	return shaft_angle;
}


//速度环模式
float move_velocity(float velocity_target){
	shaft_velocity = shaftVelocity();//测速
	// velocity set point
	shaft_velocity_sp = velocity_target;
	// calculate the torque command
	current_sp = PIDoperator(&PID_velocity,(shaft_velocity_sp - shaft_velocity)); // if current/foc_current torque control
//	current_sp = PIDoperator(&PID_velocity,(shaft_velocity - shaft_velocity_sp)); // if current/foc_current torque control
	// if torque controlled through voltage control 
//	if(torque_controller == Type_voltage)
//	{
//		voltage.q = current_sp;  // use voltage if phase-resistance not provided
//		voltage.d = 0;
//	}
	return shaft_velocity;
}



//void loopFOC(void)
//{
//	shaft_angle = shaftAngle();// shaft angle
//	electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
//	
//	// read dq currents
//	current = getFOCCurrents(electrical_angle);
//	// filter values
//	current.q = LPFoperator(&LPF_current_q,current.q);
//	current.d = LPFoperator(&LPF_current_d,current.d);
//	// calculate the phase voltages
//	voltage.q = PIDoperator(&PID_current_q,(current_sp - current.q)); 
//	voltage.d = PIDoperator(&PID_current_d, -current.d);
//			
//	// set the phase voltage - FOC heart function :)
//  setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
//}


//电流环
void loopFOCtest(void)
{
//	getPhaseCurrents();
	
	shaft_angle = shaftAngle();// shaft angle
	electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
	
	// read dq currents
	current = getFOCCurrents(electrical_angle);
	
	
	
	// filter values
//	current.q = LPFoperator(&LPF_current_q,current.q);//有bug
//	current.d = LPFoperator(&LPF_current_d,current.d);

	
	
//	// calculate the phase voltages
//	if(targetIq){
//		voltage.q = PIDoperator(&PID_current_q,(targetIq - current.q)); 
//	}else{
//		voltage.q = PIDoperator(&PID_current_q,(current_sp - current.q)); 
//	}
		
	voltage.q = PIDoperator(&PID_current_q,(current_sp - current.q)); 
	voltage.d = PIDoperator(&PID_current_d, - current.d);

			
	// set the phase voltage - FOC heart function :)
  setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
	
}

DQCurrent_s current2;
//电流环
void loopFOCtest_second(void)
{
	
	shaft_angle2 = shaftAngle2();// shaft angle
	electrical_angle2 = electricalAngle2();// electrical angle - need shaftAngle to be called first
	
	// read dq currents
//	getPhaseCurrents2();
	
	current2 = getFOCCurrents2(electrical_angle2);
	
//	getFOCCurrents2(electrical_angle2);

//	printf("%f\r\n",current2.d);
//	printf("%f\r\n",current2.q);

	voltage2.q = PIDoperator(&PID_current_q2,(current_sp2 - current2.q)); //输出为啥是0？
	voltage2.d = PIDoperator(&PID_current_d2, - current2.d);

			
	// set the phase voltage - FOC heart function :)
  setPhaseVoltage2(voltage2.q, voltage2.d, electrical_angle2);
	
}


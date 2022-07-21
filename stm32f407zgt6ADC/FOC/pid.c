#include "pid.h"
//#include "main.h"

PIDController  PID_current_q,PID_current_d,PID_velocity,P_angle;
PIDController  PID_current_q2,PID_current_d2,PID_velocity2,P_angle2;

//����PID������������̨����ʵ���һ�㣬���ں�ģ����ʵ�Сһ��
void PID_init(void)
{
	//�ٶȻ�PID
//	PID_velocity.P=0.4;  //0.5
//	PID_velocity.I=0.6;    //1
	
	PID_velocity.P=0.5;  //0.5
	PID_velocity.I=0.9;    //10

	PID_velocity.D=0;
	PID_velocity.output_ramp=1000;
	//PID_velocity.limit=0;        //Motor_init()�����Ѿ���limit��ʼ�����˴����账��
	PID_velocity.error_prev=0;
	PID_velocity.output_prev=0;
	PID_velocity.integral_prev=0;
	PID_velocity.timestamp_prev=0;
	
	//λ�û�PID
	P_angle.P=20;
	P_angle.I=0;
	P_angle.D=0;
	P_angle.output_ramp=0;
	//P_angle.limit=0;
	P_angle.error_prev=0;
	P_angle.output_prev=0;
	P_angle.integral_prev=0;
	P_angle.timestamp_prev=0;
	
	//������q��PID
//	PID_current_q.P=3;  //��ģ������ٶȱջ������ܴ���1����������ʧ��
//	PID_current_q.I=300;    //������I������̫�õ��ԣ�ֻ��P����Ҳ����

	PID_current_q.P=0.2;  //��ģ������ٶȱջ������ܴ���1����������ʧ��
	PID_current_q.I=2.5;    //������I������̫�õ��ԣ�ֻ��P����Ҳ����

//	PID_current_q.I=0;    //������I������̫�õ��ԣ�ֻ��P����Ҳ����
	PID_current_q.D=0;
	PID_current_q.output_ramp=0;
	//PID_current_q.limit=0;
	PID_current_q.error_prev=0;
	PID_current_q.output_prev=0;
	PID_current_q.integral_prev=0;
	PID_current_q.timestamp_prev=0;
	
	//������d��PID
//	PID_current_d.P=3;  //0.5
//	PID_current_d.I=300;  //0.5
	
	PID_current_d.P=0.2;  //��ģ������ٶȱջ������ܴ���1����������ʧ��
	PID_current_d.I=2.5;    //������I������̫�õ��ԣ�ֻ��P����Ҳ����

//	PID_current_d.I=0;
	PID_current_d.D=0;
	PID_current_d.output_ramp=0;
	//PID_current_d.limit=0;
	PID_current_d.error_prev=0;
	PID_current_d.output_prev=0;
	PID_current_d.integral_prev=0;
	PID_current_d.timestamp_prev=0;
}


void PID_init2(void)
{
	//�ٶȻ�PID
//	PID_velocity.P=0.4;  //0.5
//	PID_velocity.I=0.6;    //1
	
	PID_velocity2.P=0.5;  //0.5
	PID_velocity2.I=0.9;    //10

	PID_velocity2.D=0;
	PID_velocity2.output_ramp=1000;
	//PID_velocity.limit=0;        //Motor_init()�����Ѿ���limit��ʼ�����˴����账��
	PID_velocity2.error_prev=0;
	PID_velocity2.output_prev=0;
	PID_velocity2.integral_prev=0;
	PID_velocity2.timestamp_prev=0;
	
	//λ�û�PID
	P_angle2.P=20;
	P_angle2.I=0;
	P_angle2.D=0;
	P_angle2.output_ramp=0;
	//P_angle.limit=0;
	P_angle2.error_prev=0;
	P_angle2.output_prev=0;
	P_angle2.integral_prev=0;
	P_angle2.timestamp_prev=0;
	
	//������q��PID
//	PID_current_q.P=3;  //��ģ������ٶȱջ������ܴ���1����������ʧ��
//	PID_current_q.I=300;    //������I������̫�õ��ԣ�ֻ��P����Ҳ����

	PID_current_q2.P=0.2;  //��ģ������ٶȱջ������ܴ���1����������ʧ��
	PID_current_q2.I=2.5;    //������I������̫�õ��ԣ�ֻ��P����Ҳ����

//	PID_current_q.I=0;    //������I������̫�õ��ԣ�ֻ��P����Ҳ����
	PID_current_q2.D=0;
	PID_current_q2.output_ramp=0;
	//PID_current_q.limit=0;
	PID_current_q2.error_prev=0;
	PID_current_q2.output_prev=0;
	PID_current_q2.integral_prev=0;
	PID_current_q2.timestamp_prev=0;
	
	//������d��PID
//	PID_current_d.P=3;  //0.5
//	PID_current_d.I=300;  //0.5
	
	PID_current_d2.P=0.2;  //��ģ������ٶȱջ������ܴ���1����������ʧ��
	PID_current_d2.I=2.5;    //������I������̫�õ��ԣ�ֻ��P����Ҳ����

//	PID_current_d.I=0;
	PID_current_d2.D=0;
	PID_current_d2.output_ramp=0;
	//PID_current_d.limit=0;
	PID_current_d2.error_prev=0;
	PID_current_d2.output_prev=0;
	PID_current_d2.integral_prev=0;
	PID_current_d2.timestamp_prev=0;
}

float Ts;
float proportional,integral,derivative;
// PID controller function
float PIDoperator(PIDController* PID,float error)
{
	unsigned long now_us;
	float output;
	float output_rate;
	
//	now_us = SysTick->VAL;
//	if(now_us<PID->timestamp_prev)Ts = (float)(PID->timestamp_prev - now_us)/6*1e-9f;
//	else
//		Ts = (float)(0xFFFFFF - now_us + PID->timestamp_prev)/6*1e-9f;
//	PID->timestamp_prev = now_us;
//	if(Ts == 0 || Ts > 0.5f) Ts = 1e-3f;
	
	now_us = __HAL_TIM_GetCounter(&htim4);;
	if(now_us < PID->timestamp_prev)
		Ts = (float)(PID->timestamp_prev - now_us)*1e-6f;
	else
		Ts = (float)(0x0000FFFF - now_us + PID->timestamp_prev)*1e-6f;
	PID->timestamp_prev = now_us;
	if(Ts == 0 || Ts > 0.5f) Ts = 1e-3f;
	
	// u(s) = (P + I/s + Ds)e(s)
	// Discrete implementations
	// proportional part
	// u_p  = P *e(k)
	proportional = PID->P * error;
	// Tustin transform of the integral part
	// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
	integral = PID->integral_prev + PID->I*Ts*0.5f*(error + PID->error_prev);
	// antiwindup - limit the output
	integral = _constrain(integral, -PID->limit, PID->limit);
	// Discrete derivation
	// u_dk = D(ek - ek_1)/Ts
	derivative = PID->D*(error - PID->error_prev)/Ts;
	
	// sum all the components
	output = proportional + integral + derivative;
	// antiwindup - limit the output variable
	output = _constrain(output, -PID->limit, PID->limit);
	
	// if output ramp defined
	if(PID->output_ramp > 0)
	{
		// limit the acceleration by ramping the output
		//����PID����������ʺ��½��ʣ���ֹ�����ʺ��½��ʹ��������ϵͳ����
		output_rate = (output - PID->output_prev)/Ts;
		if(output_rate > PID->output_ramp)output = PID->output_prev + PID->output_ramp*Ts;
		else if(output_rate < -PID->output_ramp)output = PID->output_prev - PID->output_ramp*Ts;
	}
	
	// saving for the next pass
	PID->integral_prev = integral;
	PID->output_prev = output;
	PID->error_prev = error;
	
	return output;
}

/*************************************************************************/
/**
  * @brief  ����ʽPID�㷨ʵ��
  * @param  val		Ŀ��ֵ
	*	@note 	��
  * @retval ͨ��PID���������
  */

float PID_incremental(PIDController* PID,float error) 
{
	unsigned long now_us;
	float Ts;
	float output_rate;
	float output;
	
	now_us = SysTick->VAL;
	if(now_us<PID->timestamp_prev)Ts = (float)(PID->timestamp_prev - now_us)/9*1e-6f;
	else
		Ts = (float)(0xFFFFFF - now_us + PID->timestamp_prev)/9*1e-6f;
	PID->timestamp_prev = now_us;
	if(Ts == 0 || Ts > 0.5f) Ts = 1e-3f;
	
  
	/*PID�㷨ʵ��*/
	output +=  PID->P * (error - PID->error_prev) 
										+  PID->I *  error
										+  PID->D * (error - 2 * PID->error_prev + PID->err_last);
	/*�������*/
	
	output = _constrain(output, -PID->limit, PID->limit);
	
	// if output ramp defined
	if(PID->output_ramp > 0)
	{
		// limit the acceleration by ramping the output
		//����PID����������ʺ��½��ʣ���ֹ�����ʺ��½��ʹ��������ϵͳ����
		output_rate = (output - PID->output_prev)/Ts;
		if(output_rate > PID->output_ramp)output = PID->output_prev + PID->output_ramp*Ts;
		else if(output_rate < -PID->output_ramp)output = PID->output_prev - PID->output_ramp*Ts;
	}
	
	PID->err_last = PID->error_prev;
	PID->error_prev = error;
  PID->output_prev = output;
	
	/*���ص�ǰʵ��ֵ*/
	return output;
}



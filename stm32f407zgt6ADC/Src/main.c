/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern int16_t adc_buff[ADC_NUM_MAX];    // ADC��ѹ�ɼ�������
//extern int16_t adc_buff2[ADC_NUM_MAX];    // ADC��ѹ�ɼ�������

//extern float adc_buff[ADC_NUM_MAX];    // ADC��ѹ�ɼ�������
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern float adc_u;
extern float adc_v;

extern float adc_u2;
extern float adc_v2;

uint16_t IU,IV;

//extern __IO uint16_t ChannelPulse;


float targetId;
float targetIq;
float velocity_target;	//���趨Ŀ���ٶ�
float position_target;	//���趨Ŀ��λ��

int  pole_pairs	= 7;//���������

long sensor_direction;
long sensor_direction2;

float voltage_power_supply;
float voltage_limit;

float voltage_sensor_align;
float voltage_sensor_align2;

float velocity_limit;
float current_limit;


uint32_t upposition;
uint32_t upvelocity;
uint32_t upcurrentq;
uint32_t upcurrentd;


float now_angle;
float now_velocity;
float now_position;

float now_angle2;
float now_velocity2;
float now_position2;

float Iq_target;

extern float a,b,c;
PhaseCurrent_s current_test;
PhaseCurrent_s current_test2;

float ctt,stt,d_t,q_t;
float i_alpha_t, i_beta_t;

extern float sensor_offset;
extern float Electrical_Angele_Sensor_Offset;

float firstAngle;
float SecondAngle;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	voltage_limit = 6;          //V�����ֵ��С�� �����ѹ/sqrt(3) 12/1.732=6.9 24/sqrt(3)=13.8568
	voltage_limit = 12;          //V�����ֵ��С�� �����ѹ/sqrt(3) 12/1.732=6.9 24/sqrt(3)=13.8568
	voltage_power_supply = 12; 	//��Դ��ѹ
	
	voltage_sensor_align = 2;   //V alignSensor() use it�����ʵ�����õ�ֵСһ�����0.5-1��С������õĴ�һ�����2-3
	voltage_sensor_align2 = 4;
	
	current_limit = 30; 				//�������ƣ��ٶ�ģʽ��λ��ģʽ������
	velocity_limit= 20;         	//λ��ģʽ�ٶ�����
	
	velocity_target = 3;        //���趨Ŀ���ٶ� 3 rad/s
	position_target = 3;				//���趨Ŀ��λ�� 3 rad
	
	Iq_target = 0; 
	targetId = 0;
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	protocol_init();
	
	__HAL_TIM_SetCounter(&htim4, 0);
  __HAL_TIM_ENABLE(&htim4);
	
	IIC_Init();
	IIC2_Init();
	
		
	HAL_ADC_Start(&hadc1);
	//HAL_ADC_Start_DMA(&hadc3,(uint32_t *)&ADC_ConvertedValue,3);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&adc_buff,ADC_NUM_MAX);//ADC��������
	
//	getPhaseCurrents();
//	getPhaseCurrents2();
	
	
	LPF_init();
	LPF_init2();
	
	PID_init();
	PID_init2();
	
	InlineCurrentSense(0.01,50);    //SimpleMotor//����������ֵ���˷ű���
	InlineCurrentSense2(0.01, 50);
	
	Motor_init();
	my_delay_ms(10);
	Motor_initFOC(0,UNKNOWN);
	my_delay_ms(500);
	Current_calibrateOffsets();//����ƫ�ò���(��������µĵ���)
	
	Motor_init2();
	my_delay_ms(10);
	Motor_initFOC2(0,UNKNOWN);
	my_delay_ms(500);
	Current_calibrateOffsets2();//����ƫ�ò���(��������µĵ���)
	
//	M2_Enable;
	
	my_delay_ms(10);
	/*****************************************************************/
	//Uq90��
//	//setPhaseVoltage(voltage_sensor_align, 0, _3PI_2);
//	
//	setPhaseVoltage(voltage_sensor_align, 0, _PI_2);
//	
//	firstAngle = shaftAngle();//������Ӧ����90��
//	
////	setPhaseVoltage(0, voltage_sensor_align, 0);
////	my_delay_ms(500);
//	sensor_offset = shaftAngle();// shaft angle
//	sensor_offset = sensor_offset - _PI_2/7;
	
	/*****************************************************************/

	
	
	
//	setPhaseVoltage(0, 0, 0);
#if 0
	CALIBRATION_start();

	CALIBRATION_loop();
	my_delay_ms(1000);	
#endif

#if 0
	set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);                // ͬ����λ����������ť״̬
	set_computer_value(SEND_STOP_CMD, CURVES_CH2, NULL, 0);                // ͬ����λ����������ť״̬
	set_computer_value(SEND_STOP_CMD, CURVES_CH3, NULL, 0);                // ͬ����λ����������ť״̬
	set_computer_value(SEND_STOP_CMD, CURVES_CH4, NULL, 0);                // ͬ����λ����������ť״̬
	
//	set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &upcurrentq, 1);     // ��ͨ�� 3����Ŀ��ֵ
//	set_computer_value(SEND_TARGET_CMD, CURVES_CH2, &upcurrentd, 1);     // ��ͨ�� 3����Ŀ��ֵ
//	set_computer_value(SEND_TARGET_CMD, CURVES_CH3, &velocity_target, 1);     // ��ͨ�� 3����Ŀ��ֵ
//	set_computer_value(SEND_TARGET_CMD, CURVES_CH4, &position_target, 1);     // ��ͨ�� 3����Ŀ��ֵ
	
	set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0);               // ͬ����λ����������ť״̬
	set_computer_value(SEND_START_CMD, CURVES_CH2, NULL, 0);               // ͬ����λ����������ť״̬
	set_computer_value(SEND_START_CMD, CURVES_CH3, NULL, 0);               // ͬ����λ����������ť״̬
	set_computer_value(SEND_START_CMD, CURVES_CH4, NULL, 0);               // ͬ����λ����������ť״̬
#endif	
//	setPhaseVoltage(voltage_sensor_align,0,_PI_2);
	
//	setPhaseVoltage(0,voltage_sensor_align,0);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	
  while (1)
  {
		#if 0
			receiving_process();
		#endif
		
		//printf("%f\r\n",AS5600_ReadRawAngle2()/4096*360);

		
		#if 0
//		shaft_angle = shaftAngle();// shaft angle
		
		electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
		shaft_angle = shaftAngle();// shaft angle
		now_velocity = getVelocity();
		#endif
//		printf("%f\r\n",PIDoperator(&PID_velocity,(10 - 5)));
		
		//���1У׼
		#if 0
		//Ud��׼
		setPhaseVoltage(0,voltage_sensor_align,(_2PI * 1.f/ 12.0f));
		
		current_test = getPhaseCurrents();
		
		shaft_angle = shaftAngle();// shaft angle
		electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
		
//		i_alpha_t = current_test.a;  
//		i_beta_t = _1_SQRT3 * current_test.a + _2_SQRT3 * current_test.b;//a + b + c = 0
//		
//		ctt = arm_cos_f32(electrical_angle);
//		stt = arm_sin_f32(electrical_angle);
//		
//		//simplefoc��park�任
//		d_t = i_alpha_t * ctt + i_beta_t * stt;
//		q_t = i_beta_t * ctt - i_alpha_t * stt;
		
		#endif
		
		//���2У׼
		#if 0
		//���2 Ud��׼
		setPhaseVoltage2(0,voltage_sensor_align2,(_2PI * 1.f/ 12.0f));
		
		current_test2 = getPhaseCurrents2();
		
		shaft_angle2 = shaftAngle2();// shaft angle
		electrical_angle2 = electricalAngle2();// electrical angle - need shaftAngle to be called first
		
//		i_alpha_t = current_test2.a;  
//		i_beta_t = _1_SQRT3 * current_test2.a + _2_SQRT3 * current_test2.b;//a + b + c = 0
//		
//		ctt = arm_cos_f32(electrical_angle2);
//		stt = arm_sin_f32(electrical_angle2);
//		
//		//simplefoc��park�任
//		d_t = i_alpha_t * ctt + i_beta_t * stt;
//		q_t = i_beta_t * ctt - i_alpha_t * stt;
		
		#endif
//		printf( "angle : %.04f\r\n", ReadAngle() );

//		HAL_Delay(1000);
		
		//�������
		#if 0
		//a
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0.1*ChannelPulse);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);
		//b
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0.1*ChannelPulse);
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);
		//c
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0.1*ChannelPulse);
		
		
		current_test = getPhaseCurrents();
		#endif
		
//		setPhaseVoltage(0, 1, _2PI * 1.0f/ 12.0f);
//		setPhaseVoltage(0, 1, _2PI * 3.0f/ 12.0f);
//		setPhaseVoltage(0, 1, _2PI * 5.0f/ 12.0f);
//		setPhaseVoltage(0, 1, _2PI * 7.0f/ 12.0f);
//		setPhaseVoltage(0, 1, _2PI * 9.0f/ 12.0f);
//		setPhaseVoltage(0, 1, _2PI * 11.0f/ 12.0f);
		
//		shaft_angle = shaftAngle();// shaft angle
//		electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
//		current = getFOCCurrents(electrical_angle);
		
		
//		move(Iq_target);
//Id
#if 0
		shaft_angle = shaftAngle();// shaft angle
		electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
		
		// read dq currents
		current = getFOCCurrents(electrical_angle);
		// filter values
//		current.q = LPFoperator(&LPF_current_q,current.q);
		current.d = LPFoperator(&LPF_current_d,current.d);
		
		voltage.d = PIDoperator(&PID_current_d, (targetId - current.d));
		setPhaseVoltage(0, voltage.d, electrical_angle);
#endif		

//Iq
#if 0
//		shaft_angle = shaftAngle();// shaft angle
		electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
		
		// read dq currents
		current = getFOCCurrents(electrical_angle);
		// filter values
		current.q = LPFoperator(&LPF_current_q,current.q);
//		current.d = LPFoperator(&LPF_current_d,current.d);
		
//		voltage.d = PIDoperator(&PID_current_d, (targetId - current.d));
		voltage.q = PIDoperator(&PID_current_q, (targetIq - current.q));
		setPhaseVoltage(voltage.q , 0, electrical_angle);
#endif
//clark park �任��֤

#if 0
//	DQCurrent_s getFOCCurrents(float angle_el);
	
	float angle_t = _PI/6;
	float ctt,stt,d_t,q_t;
	float i_alpha_t, i_beta_t;
	float Ia_t = 0.5;
	float	Ib_t = -0.3;
	
	i_alpha_t = Ia_t;  
	i_beta_t = _1_SQRT3 * Ia_t + _2_SQRT3 * Ib_t;//a + b + c = 0
	
	ctt = arm_cos_f32(angle_t);
	stt = arm_sin_f32(angle_t);
	
	//simplefoc��park�任
	d_t = i_alpha_t * ctt + i_beta_t * stt;
	q_t = i_beta_t * ctt - i_alpha_t * stt;
	
	printf("d_t:%f\r\n",d_t);
	printf("q_t:%f\r\n",q_t);
#endif

//����ģʽ
#if 0
		move(0);
		loopFOCtest();
		now_velocity = shaftVelocity();//����
		now_position = getAngle();
#endif
//�ٶ�ģʽ
#if 0
		now_velocity = move_velocity(velocity_target);
		loopFOCtest();
		
//		upposition = getAngle();
		
//		upvelocity = now_velocity * 1000;
//		upcurrentq = current.q * 1000 + 1000;//����1000��ƫ��1000
//		upcurrentd = current.d * 1000 + 1000;//����1000,ƫ��1000
		
//		set_computer_value(SEND_FACT_CMD, CURVES_CH1, &upcurrentq, 1);     // ��ͨ��1��Iq
//		set_computer_value(SEND_FACT_CMD, CURVES_CH2, &upcurrentd, 1);     // ��ͨ��2��Id
//		set_computer_value(SEND_FACT_CMD, CURVES_CH3, &upvelocity, 1);     // ��ͨ��3�����ٶ�ֵ
////		set_computer_value(SEND_FACT_CMD, CURVES_CH4, &upposition, 1);     // ��ͨ��4����λ��ֵ
#endif

//λ�û�ģʽ
#if 0
		now_position = move_position(position_target);
		loopFOCtest();//������PID
		
		now_velocity = shaftVelocity();
		
//		upvelocity = now_velocity * 1000;//����1000
//		upcurrentq = current.q * 1000 + 1000;//����1000��ƫ��1000
//		upcurrentd = current.d * 1000 + 1000;//����1000,ƫ��1000
#endif

//		now_velocity= move_velocity(velocity_target);
//		loopFOCtest();

//	float A,B;
//	float d =_2PI/2048;
	
//˫������ػ���
#if 1
//		move(0);
//		move2(0);

		move_torque1();
		move_torque2();
		
		loopFOCtest();
		loopFOCtest_second();
		
//		test();
		
//		if((adc_u>0.5)&&(adc_v>0.5)&&(adc_u2>0.5)&&(adc_v2>0.5)){
//			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,GPIO_PIN_RESET);
//		}else{
//			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,GPIO_PIN_SET);
//		}
		
//	shaft_angle2 = shaftAngle2();// shaft angle
//	electrical_angle2 = electricalAngle2();// electrical angle - need shaftAngle to be called first
	
//	// read dq currents
//	current2 = getFOCCurrents2(electrical_angle2);

////		
//	voltage2.q = PIDoperator(&PID_current_q2,(current_sp - current2.q)); 
//	voltage2.d = PIDoperator(&PID_current_d2, (targetId - current2.d));

//			
//	// set the phase voltage - FOC heart function :)
//  setPhaseVoltage2(voltage2.q, voltage2.d, electrical_angle2);



//		getPhaseCurrents();
//		getPhaseCurrents2();

//		current_test = getPhaseCurrents();
//		current_test2 = getPhaseCurrents2();
		
		now_velocity = shaftVelocity();//����
		now_position = getAngle();
		
//		now_velocity2 = shaftVelocity2();//����
//		now_position2 = getAngle2();
#endif
	
	
#if 0
//��������
	for(uint32_t i = 0; i<5000;i++){
		shaft_angle = shaftAngle();// shaft angle
		electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
		setPhaseVoltage(3*arm_sin_f32(((float)i/5000.0f) * _2PI),3*arm_cos_f32(((float)i/5000.0f) * _2PI),electrical_angle);

//		my_delay_ms(10);
		
		set_computer_value(SEND_FACT_CMD, CURVES_CH1, &a, 1);     
		set_computer_value(SEND_FACT_CMD, CURVES_CH2, &b, 1);  
		set_computer_value(SEND_FACT_CMD, CURVES_CH3, &c, 1);     
		
	}
#endif

//����ͨ��ֵ
#if 0
		upposition = now_position * 1000;//����1000
		upvelocity = now_velocity * 1000;//����1000
		upcurrentq = current.q * 1000 + 1000;//����1000��ƫ��1000
		upcurrentd = current.d * 1000 + 1000;//����1000,ƫ��1000
		set_computer_value(SEND_FACT_CMD, CURVES_CH1, &upcurrentq, 1);     // ��ͨ��1��Iq
		set_computer_value(SEND_FACT_CMD, CURVES_CH2, &upcurrentd, 1);     // ��ͨ��2��Id
		set_computer_value(SEND_FACT_CMD, CURVES_CH3, &upvelocity, 1);     // ��ͨ��3�����ٶ�ֵ
		set_computer_value(SEND_FACT_CMD, CURVES_CH4, &upposition, 1);     // ��ͨ��4����λ��ֵ
#endif
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		my_delay_ms(100);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

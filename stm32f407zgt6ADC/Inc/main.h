/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mydelay.h"
#include "arm_math.h"
#include "foc_utils.h"
#include "tim.h"
#include "BLDCmotor.h"
#include "pid.h"
#include "MagneticSensorI2C.h"
#include "MagneticSensorI2C2.h"
#include "bsp_SysTick.h"
#include "lowpass_filter.h"
#include "FOCMotor.h"
#include "adc.h"
#include "CurrentSense.h"
#include "InlineCurrentSense.h"
#include "stdio.h"
#include "protocol.h"
#include "calibration.h"
#include "test.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
//#define SHUTDOWN_PIN                  GPIO_PIN_6
//#define SHUTDOWN_GPIO_PORT            GPIOE

//#define SHUTDOWN_PIN2                  GPIO_PIN_0
//#define SHUTDOWN_GPIO_PORT2            GPIOE


///* ��� SD or EN ʹ�ܽ� */
//#define M1_Enable  HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT, SHUTDOWN_PIN, GPIO_PIN_SET)      // �ߵ�ƽ��-�ߵ�ƽʹ�� 
//#define M1_Disable HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT, SHUTDOWN_PIN, GPIO_PIN_RESET)    // �͵�ƽ�ض�-�͵�ƽ����

//#define M2_Enable  HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT2, SHUTDOWN_PIN2, GPIO_PIN_SET)      // �ߵ�ƽ��-�ߵ�ƽʹ�� 
//#define M2_Disable HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT2, SHUTDOWN_PIN2, GPIO_PIN_RESET)    // �͵�ƽ�ض�-�͵�ƽ����


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

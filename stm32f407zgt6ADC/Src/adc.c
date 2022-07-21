/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
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

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
int16_t adc_buff[ADC_NUM_MAX];    // ADC电压采集缓冲区
//int16_t adc_buff2[ADC_NUM_MAX];    // ADC电压采集缓冲区

//float adc_buff[ADC_NUM_MAX];

uint32_t adc_mean_sum_u = 0;        // 平均值累加
uint32_t adc_mean_sum_v = 0;        // 平均值累加
uint32_t adc_mean_sum_w = 0;        // 平均值累加

uint32_t adc_mean_sum_u2 = 0;        // 平均值累加
uint32_t adc_mean_sum_v2 = 0;        // 平均值累加

float adc_u = 0;
float adc_v = 0;



float adc_w = 0;

float adc_u2 = 0;
float adc_v2 = 0;
float adc_w2 = 0;

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration    
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3|GPIO_PIN_4);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_1);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle)
{
	HAL_ADC_Stop_DMA(adcHandle);       // 停止 ADC 采样，处理完一次数据在继续采样

	adc_u = (uint32_t)adc_buff[0]/4096.0f*3.3f;
	adc_v = (uint32_t)adc_buff[1]/4096.0f*3.3f;
	
	adc_u2 = (uint32_t)adc_buff[2]/4096.0f*3.3f;
	adc_v2 = (uint32_t)adc_buff[3]/4096.0f*3.3f;
	
	
//	for(uint32_t count = 0; count < ADC_NUM_MAX; count+=2)
//	{
//		adc_mean_sum_u += (uint32_t)adc_buff[count];
//	}
//	adc_u = (float)adc_mean_sum_u/(ADC_NUM_MAX/2);
//	adc_u = adc_u/4096.0f*3.3f;
//	adc_mean_sum_u = 0;
//	for(uint32_t count = 1; count < ADC_NUM_MAX; count+=2)
//	{
//		adc_mean_sum_v += (uint32_t)adc_buff[count];
//	}
//	adc_v = (float)adc_mean_sum_v/(ADC_NUM_MAX/2);
//	adc_v = adc_v/4096.0f*3.3f;
//	adc_mean_sum_v = 0;
//	
///*********************************************************************/	

//	for(uint32_t count = 2; count < ADC_NUM_MAX; count+=2)
//	{
//		adc_mean_sum_u2 += (uint32_t)adc_buff[count];
//	}
//	adc_u2 = (float)adc_mean_sum_u2/(ADC_NUM_MAX/2);
//	adc_u2 = adc_u2/4096.0f*3.3f;
//	adc_mean_sum_u2 = 0;
//	for(uint32_t count = 3; count < ADC_NUM_MAX; count+=2)
//	{
//		adc_mean_sum_v2 += (uint32_t)adc_buff[count];
//	}
//	adc_v2 = (float)adc_mean_sum_v2/(ADC_NUM_MAX/2);
//	adc_v2 = adc_v2/4096.0f*3.3f;
//	adc_mean_sum_v2 = 0;
	
	HAL_ADC_Start_DMA(adcHandle,(uint32_t*)&adc_buff, ADC_NUM_MAX);       // 开始 ADC 采样
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

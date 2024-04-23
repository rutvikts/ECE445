/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM2_CH1_ME1_ChA_Pin GPIO_PIN_0
#define TIM2_CH1_ME1_ChA_GPIO_Port GPIOA
#define TIM2_CH2_ME1_ChB_Pin GPIO_PIN_1
#define TIM2_CH2_ME1_ChB_GPIO_Port GPIOA
#define SPI1_NSS_RasPi_Pin GPIO_PIN_2
#define SPI1_NSS_RasPi_GPIO_Port GPIOA
#define SPI1_NSS_ESP2_Pin GPIO_PIN_3
#define SPI1_NSS_ESP2_GPIO_Port GPIOA
#define SPI1_NSS_ESP1_Pin GPIO_PIN_4
#define SPI1_NSS_ESP1_GPIO_Port GPIOA
#define TIM1_CH1__PWM_Motor_1_Pin GPIO_PIN_8
#define TIM1_CH1__PWM_Motor_1_GPIO_Port GPIOA
#define TIM1_CH2__PWM_Motor_2_Pin GPIO_PIN_9
#define TIM1_CH2__PWM_Motor_2_GPIO_Port GPIOA
#define TIM1_CH3__PWM_Motor_3_Pin GPIO_PIN_10
#define TIM1_CH3__PWM_Motor_3_GPIO_Port GPIOA
#define TIM3_CH1_ME2_ChA_Pin GPIO_PIN_4
#define TIM3_CH1_ME2_ChA_GPIO_Port GPIOB
#define TIM3_CH2_ME2_ChB_Pin GPIO_PIN_5
#define TIM3_CH2_ME2_ChB_GPIO_Port GPIOB
#define TIM4_CH1_ME3_ChA_Pin GPIO_PIN_6
#define TIM4_CH1_ME3_ChA_GPIO_Port GPIOB
#define TIM4_CH2_ME3_ChB_Pin GPIO_PIN_7
#define TIM4_CH2_ME3_ChB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

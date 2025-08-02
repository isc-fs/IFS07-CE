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
#include "stm32h7xx_hal.h"

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
#define START_BUTTON_LED_Pin GPIO_PIN_0
#define START_BUTTON_LED_GPIO_Port GPIOC
#define RTDS_Pin GPIO_PIN_1
#define RTDS_GPIO_Port GPIOC
#define CTRL_FANS_Pin GPIO_PIN_4
#define CTRL_FANS_GPIO_Port GPIOC
#define SUSPENSION_RL_Pin GPIO_PIN_5
#define SUSPENSION_RL_GPIO_Port GPIOC
#define SUSPENSION_RR_Pin GPIO_PIN_0
#define SUSPENSION_RR_GPIO_Port GPIOB
#define CTRL_PUMP_Pin GPIO_PIN_1
#define CTRL_PUMP_GPIO_Port GPIOB
#define S_FRENO_Pin GPIO_PIN_11
#define S_FRENO_GPIO_Port GPIOF
#define SUSPENSION_FL_Pin GPIO_PIN_13
#define SUSPENSION_FL_GPIO_Port GPIOF
#define SUSPENSION_FR_Pin GPIO_PIN_14
#define SUSPENSION_FR_GPIO_Port GPIOF
#define MICROSD_DET_Pin GPIO_PIN_15
#define MICROSD_DET_GPIO_Port GPIOF
#define DS18B20_Data_Pin GPIO_PIN_8
#define DS18B20_Data_GPIO_Port GPIOE
#define PWM_FAN1_Pin GPIO_PIN_9
#define PWM_FAN1_GPIO_Port GPIOE
#define PWM_FAN2_Pin GPIO_PIN_11
#define PWM_FAN2_GPIO_Port GPIOE
#define START_BUTTON_Pin GPIO_PIN_7
#define START_BUTTON_GPIO_Port GPIOG
#define USART1_TX_GPS_Pin GPIO_PIN_9
#define USART1_TX_GPS_GPIO_Port GPIOA
#define USART1_RX_GPS_Pin GPIO_PIN_10
#define USART1_RX_GPS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

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
HAL_StatusTypeDef module_send_message_CAN1(uint32_t id, uint8_t* data, uint8_t length);
HAL_StatusTypeDef module_send_message_CAN2(uint32_t id, uint8_t* data, uint8_t length);

HAL_StatusTypeDef module_send_message_NoExtId_CAN1(uint32_t id, uint8_t* data, uint8_t length);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AMS_OK_Pin GPIO_PIN_13
#define AMS_OK_GPIO_Port GPIOF
#define DIGITAL1_Pin GPIO_PIN_9
#define DIGITAL1_GPIO_Port GPIOE
#define Charge_Button_Pin GPIO_PIN_7
#define Charge_Button_GPIO_Port GPIOG
#define RELAY_AIR_N_Pin GPIO_PIN_3
#define RELAY_AIR_N_GPIO_Port GPIOD
#define RELAY_AIR_P_Pin GPIO_PIN_4
#define RELAY_AIR_P_GPIO_Port GPIOD
#define RELAY_PRECHARGE_Pin GPIO_PIN_5
#define RELAY_PRECHARGE_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
void print(char uart_buffer[]);
void printnl(char uart_buffer[]);
void printValue(int value);
HAL_UART_StateTypeDef getUARTState();
float readAnalogValue(void);
float readCurrentValue(void);

struct CANMsg {
    uint32_t id;    // CAN ID
    uint8_t len;    // Vector length
    uint8_t buf[8]; // CAN vector
    int bus;        // From which CAN bus is received
    uint32_t time;  // When was the message received
};
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

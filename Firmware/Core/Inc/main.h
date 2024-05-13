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
#include "stm32g4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Op_LED_Pin GPIO_PIN_13
#define Op_LED_GPIO_Port GPIOC
#define Error_LED_Pin GPIO_PIN_14
#define Error_LED_GPIO_Port GPIOC
#define Rfl_V_Pin GPIO_PIN_0
#define Rfl_V_GPIO_Port GPIOA
#define Fwd_V_Pin GPIO_PIN_1
#define Fwd_V_GPIO_Port GPIOA
#define Relays_R4_Pin GPIO_PIN_2
#define Relays_R4_GPIO_Port GPIOA
#define Relays_S4_Pin GPIO_PIN_3
#define Relays_S4_GPIO_Port GPIOA
#define Relays_R3_Pin GPIO_PIN_4
#define Relays_R3_GPIO_Port GPIOA
#define Relays_S3_Pin GPIO_PIN_5
#define Relays_S3_GPIO_Port GPIOA
#define Relays_R2_Pin GPIO_PIN_6
#define Relays_R2_GPIO_Port GPIOA
#define Relays_S2_Pin GPIO_PIN_7
#define Relays_S2_GPIO_Port GPIOA
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOB
#define START_Pin GPIO_PIN_1
#define START_GPIO_Port GPIOB
#define START_EXTI_IRQn EXTI1_IRQn
#define CapRels_1_Pin GPIO_PIN_11
#define CapRels_1_GPIO_Port GPIOB
#define CapRels_2_Pin GPIO_PIN_12
#define CapRels_2_GPIO_Port GPIOB
#define IndRels_1_Pin GPIO_PIN_13
#define IndRels_1_GPIO_Port GPIOB
#define IndRels_2_Pin GPIO_PIN_14
#define IndRels_2_GPIO_Port GPIOB
#define SWR_SHDN_Pin GPIO_PIN_15
#define SWR_SHDN_GPIO_Port GPIOB
#define Relays_R1_Pin GPIO_PIN_8
#define Relays_R1_GPIO_Port GPIOA
#define Relays_S1_Pin GPIO_PIN_9
#define Relays_S1_GPIO_Port GPIOA
#define CAN_STBY_Pin GPIO_PIN_7
#define CAN_STBY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define TIM3_CH2_SERVO_L3_Pin GPIO_PIN_7
#define TIM3_CH2_SERVO_L3_GPIO_Port GPIOA
#define TIM3_CH3_POLOLU_P_Pin GPIO_PIN_0
#define TIM3_CH3_POLOLU_P_GPIO_Port GPIOB
#define TIM3_CH4_POLOLU_L_Pin GPIO_PIN_1
#define TIM3_CH4_POLOLU_L_GPIO_Port GPIOB
#define TIM1_CH1_SERVO_P1_Pin GPIO_PIN_8
#define TIM1_CH1_SERVO_P1_GPIO_Port GPIOA
#define TIM1_CH2_SERVO_L1_Pin GPIO_PIN_9
#define TIM1_CH2_SERVO_L1_GPIO_Port GPIOA
#define TIM1_CH3_SERVO_P2_Pin GPIO_PIN_10
#define TIM1_CH3_SERVO_P2_GPIO_Port GPIOA
#define TIM1_CH4_SERVO_L2_Pin GPIO_PIN_11
#define TIM1_CH4_SERVO_L2_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define TIM3_CH1_SERVO_P3_Pin GPIO_PIN_4
#define TIM3_CH1_SERVO_P3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "csb.h"
#include "delay.h"
#include <stdio.h>
#include <string.h>
#include "jy931.h"
#include "pid.h"
#include "encoder_motor.h"
#include "Emm_V5.h"
#include "odom.h"
#include "vofa.h"
#include "control.h"
#include "control_trapezoid.h"
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
#define T1_Pin GPIO_PIN_2
#define T1_GPIO_Port GPIOE
#define E2_Pin GPIO_PIN_3
#define E2_GPIO_Port GPIOE
#define E2_EXTI_IRQn EXTI3_IRQn
#define T2_Pin GPIO_PIN_4
#define T2_GPIO_Port GPIOE
#define E3_Pin GPIO_PIN_5
#define E3_GPIO_Port GPIOE
#define E3_EXTI_IRQn EXTI9_5_IRQn
#define T3_Pin GPIO_PIN_6
#define T3_GPIO_Port GPIOE
#define E4_Pin GPIO_PIN_10
#define E4_GPIO_Port GPIOD
#define E4_EXTI_IRQn EXTI15_10_IRQn
#define T4_Pin GPIO_PIN_11
#define T4_GPIO_Port GPIOD
#define E5_Pin GPIO_PIN_12
#define E5_GPIO_Port GPIOD
#define E5_EXTI_IRQn EXTI15_10_IRQn
#define T5_Pin GPIO_PIN_13
#define T5_GPIO_Port GPIOD
#define E6_Pin GPIO_PIN_14
#define E6_GPIO_Port GPIOD
#define E6_EXTI_IRQn EXTI15_10_IRQn
#define T6_Pin GPIO_PIN_15
#define T6_GPIO_Port GPIOD
#define IO1_Pin GPIO_PIN_5
#define IO1_GPIO_Port GPIOB
#define IO2_Pin GPIO_PIN_8
#define IO2_GPIO_Port GPIOB
#define E1_Pin GPIO_PIN_1
#define E1_GPIO_Port GPIOE
#define E1_EXTI_IRQn EXTI1_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

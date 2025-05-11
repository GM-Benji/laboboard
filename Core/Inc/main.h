/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define OUT1_Pin GPIO_PIN_13
#define OUT1_GPIO_Port GPIOC
#define OUT2_Pin GPIO_PIN_14
#define OUT2_GPIO_Port GPIOC
#define LIMIT_Pin GPIO_PIN_15
#define LIMIT_GPIO_Port GPIOC
#define LIMIT_EXTI_IRQn EXTI15_10_IRQn
#define Motor1_EN_Pin GPIO_PIN_0
#define Motor1_EN_GPIO_Port GPIOC
#define Motor1_D_Pin GPIO_PIN_2
#define Motor1_D_GPIO_Port GPIOC
#define Motor1_IN1_Pin GPIO_PIN_0
#define Motor1_IN1_GPIO_Port GPIOA
#define Motor1_IN2_Pin GPIO_PIN_1
#define Motor1_IN2_GPIO_Port GPIOA
#define Motor2_EN_Pin GPIO_PIN_2
#define Motor2_EN_GPIO_Port GPIOA
#define Motor2_D_Pin GPIO_PIN_4
#define Motor2_D_GPIO_Port GPIOA
#define Motor2_IN1_Pin GPIO_PIN_6
#define Motor2_IN1_GPIO_Port GPIOA
#define Motor2_IN2_Pin GPIO_PIN_7
#define Motor2_IN2_GPIO_Port GPIOA
#define Stepper1_M2_Pin GPIO_PIN_4
#define Stepper1_M2_GPIO_Port GPIOC
#define Stepper1_M1_Pin GPIO_PIN_5
#define Stepper1_M1_GPIO_Port GPIOC
#define Stepper1_M0_Pin GPIO_PIN_0
#define Stepper1_M0_GPIO_Port GPIOB
#define Stepper1_Step_Pin GPIO_PIN_1
#define Stepper1_Step_GPIO_Port GPIOB
#define Stepper1_E_Pin GPIO_PIN_2
#define Stepper1_E_GPIO_Port GPIOB
#define Stepper1_Dir_Pin GPIO_PIN_10
#define Stepper1_Dir_GPIO_Port GPIOB
#define Stepper1_R_Pin GPIO_PIN_13
#define Stepper1_R_GPIO_Port GPIOB
#define Stepper2_M2_Pin GPIO_PIN_14
#define Stepper2_M2_GPIO_Port GPIOB
#define Stepper2_M1_Pin GPIO_PIN_15
#define Stepper2_M1_GPIO_Port GPIOB
#define Stepper2_M0_Pin GPIO_PIN_6
#define Stepper2_M0_GPIO_Port GPIOC
#define Stepper2_Step_Pin GPIO_PIN_7
#define Stepper2_Step_GPIO_Port GPIOC
#define Stepper2_E_Pin GPIO_PIN_8
#define Stepper2_E_GPIO_Port GPIOC
#define Stepper2_Dir_Pin GPIO_PIN_9
#define Stepper2_Dir_GPIO_Port GPIOC
#define Stepper2_R_Pin GPIO_PIN_15
#define Stepper2_R_GPIO_Port GPIOA
#define Stepper3_M2_Pin GPIO_PIN_10
#define Stepper3_M2_GPIO_Port GPIOC
#define Stepper3_M1_Pin GPIO_PIN_11
#define Stepper3_M1_GPIO_Port GPIOC
#define Stepper3_M0_Pin GPIO_PIN_12
#define Stepper3_M0_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOD
#define Stepper3_Step_Pin GPIO_PIN_3
#define Stepper3_Step_GPIO_Port GPIOB
#define Stepper3_E_Pin GPIO_PIN_4
#define Stepper3_E_GPIO_Port GPIOB
#define Stepper3_Dir_Pin GPIO_PIN_5
#define Stepper3_Dir_GPIO_Port GPIOB
#define Stepper3_R_Pin GPIO_PIN_7
#define Stepper3_R_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "tim.h"
volatile uint32_t lastDebounceTime_Pin15 = 0;
const uint32_t debounceDelay = 50; // w ms
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OUT1_Pin|OUT2_Pin|Motor1_EN_Pin|Motor1_D_Pin
                          |Stepper1_M2_Pin|Stepper1_M1_Pin|Stepper2_M0_Pin|Stepper2_Step_Pin
                          |Stepper2_E_Pin|Stepper2_Dir_Pin|Stepper3_M2_Pin|Stepper3_M1_Pin
                          |Stepper3_M0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Motor1_IN1_Pin|Motor1_IN2_Pin|Motor2_D_Pin|Motor2_IN1_Pin
                          |Motor2_IN2_Pin|Stepper2_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Stepper1_M0_Pin|Stepper1_E_Pin|Stepper1_Dir_Pin|Stepper1_R_Pin
                          |Stepper2_M2_Pin|Stepper2_M1_Pin|Stepper3_Step_Pin|Stepper3_E_Pin
                          |Stepper3_Dir_Pin|Stepper3_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Stepper1_Step_GPIO_Port, Stepper1_Step_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OUT1_Pin OUT2_Pin Motor1_EN_Pin Motor1_D_Pin
                           Stepper1_M2_Pin Stepper1_M1_Pin Stepper2_M0_Pin Stepper2_Step_Pin
                           Stepper2_E_Pin Stepper2_Dir_Pin Stepper3_M2_Pin Stepper3_M1_Pin
                           Stepper3_M0_Pin */
  GPIO_InitStruct.Pin = OUT1_Pin|OUT2_Pin|Motor1_EN_Pin|Motor1_D_Pin
                          |Stepper1_M2_Pin|Stepper1_M1_Pin|Stepper2_M0_Pin|Stepper2_Step_Pin
                          |Stepper2_E_Pin|Stepper2_Dir_Pin|Stepper3_M2_Pin|Stepper3_M1_Pin
                          |Stepper3_M0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LIMIT_Pin */
  GPIO_InitStruct.Pin = LIMIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LIMIT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor1_IN1_Pin Motor1_IN2_Pin Motor2_D_Pin Motor2_IN1_Pin
                           Motor2_IN2_Pin Stepper2_R_Pin */
  GPIO_InitStruct.Pin = Motor1_IN1_Pin|Motor1_IN2_Pin|Motor2_D_Pin|Motor2_IN1_Pin
                          |Motor2_IN2_Pin|Stepper2_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Stepper1_M0_Pin Stepper1_Step_Pin Stepper1_E_Pin Stepper1_Dir_Pin
                           Stepper1_R_Pin Stepper2_M2_Pin Stepper2_M1_Pin Stepper3_Step_Pin
                           Stepper3_E_Pin Stepper3_Dir_Pin Stepper3_R_Pin */
  GPIO_InitStruct.Pin = Stepper1_M0_Pin|Stepper1_Step_Pin|Stepper1_E_Pin|Stepper1_Dir_Pin
                          |Stepper1_R_Pin|Stepper2_M2_Pin|Stepper2_M1_Pin|Stepper3_Step_Pin
                          |Stepper3_E_Pin|Stepper3_Dir_Pin|Stepper3_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */
void Motor_init()
{
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);

	 HAL_GPIO_WritePin(Motor1_D_GPIO_Port, Motor1_D_Pin,0);
	 HAL_GPIO_WritePin(Motor1_IN1_GPIO_Port, Motor1_IN1_Pin,0);
	 HAL_GPIO_WritePin(Motor1_IN2_GPIO_Port, Motor1_IN2_Pin,0);
	 HAL_GPIO_WritePin(Motor1_EN_GPIO_Port, Motor1_EN_Pin,0);


	 HAL_GPIO_WritePin(Motor2_D_GPIO_Port, Motor2_D_Pin,0);
	 HAL_GPIO_WritePin(Motor2_IN1_GPIO_Port, Motor2_IN1_Pin,0);
	 HAL_GPIO_WritePin(Motor2_IN2_GPIO_Port, Motor2_IN2_Pin,0);
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);


}
void Set_Motor1(int direction)
{
	if(direction==1)
	{
		 HAL_GPIO_WritePin(Motor1_IN1_GPIO_Port, Motor1_IN1_Pin,1);
		 HAL_GPIO_WritePin(Motor1_IN2_GPIO_Port, Motor1_IN2_Pin,0);
		 HAL_GPIO_WritePin(Motor1_EN_GPIO_Port, Motor1_EN_Pin,1);
	}
	else if(direction==-1)
	{
		 HAL_GPIO_WritePin(Motor1_IN1_GPIO_Port, Motor1_IN1_Pin,0);
		 HAL_GPIO_WritePin(Motor1_IN2_GPIO_Port, Motor1_IN2_Pin,1);
		 HAL_GPIO_WritePin(Motor1_EN_GPIO_Port, Motor1_EN_Pin,1);
	}
	else
	{
		 HAL_GPIO_WritePin(Motor1_IN1_GPIO_Port, Motor1_IN1_Pin,0);
		 HAL_GPIO_WritePin(Motor1_IN2_GPIO_Port, Motor1_IN2_Pin,0);
		 HAL_GPIO_WritePin(Motor1_EN_GPIO_Port, Motor1_EN_Pin,0);
	}

}
void Set_Motor2(int direction, int speed)
{
	if(direction==1)
	{
		 HAL_GPIO_WritePin(Motor2_IN1_GPIO_Port, Motor2_IN1_Pin,1);
		 HAL_GPIO_WritePin(Motor2_IN2_GPIO_Port, Motor2_IN2_Pin,0);
		 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3 ,speed);
	}
	else if(direction==-1)
	{
		 HAL_GPIO_WritePin(Motor2_IN1_GPIO_Port, Motor2_IN1_Pin,0);
		 HAL_GPIO_WritePin(Motor2_IN2_GPIO_Port, Motor2_IN2_Pin,1);
		 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,speed);
	}
	else
	{
		 HAL_GPIO_WritePin(Motor2_IN1_GPIO_Port, Motor2_IN1_Pin,0);
		 HAL_GPIO_WritePin(Motor2_IN2_GPIO_Port, Motor2_IN2_Pin,0);
		 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t currentTime = HAL_GetTick();

    if ((GPIO_Pin == GPIO_PIN_15) && (currentTime - lastDebounceTime_Pin15 > debounceDelay))
    {
        lastDebounceTime_Pin15 = currentTime;
    }

}
/* USER CODE END 2 */

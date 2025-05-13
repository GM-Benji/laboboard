/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];
volatile uint8_t globalMode=0;
volatile uint8_t stepperNr=0;
volatile uint8_t dcNr=0;
volatile uint8_t dutyCycle =120;
volatile uint8_t wchdgReset=0;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    	{
    	    Error_Handler();
    	}
    	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)
    	{
    		Error_Handler();
    	}
    	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING)!=HAL_OK)
    	{
    		Error_Handler();
      }
      if (HAL_CAN_Start(&hcan1) != HAL_OK)
    	{
    	Error_Handler ();
    	}
  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    Error_Handler();
  }

  if(RxHeader.StdId== 202)
  {
	  /*
	   * RxData[0] - nr silnika: 1-3 krokowe, 4-5 dc, 6 żarówka, 0 wyłącza wszystko
	   * RxData[1] - tryb krokowca: 0 - obrót o próbówkę, 1 - o 1 krok, 2 - o 10 kroków, 3 - o 100 kroków
	   * RxData[2] - kierunek krokowca 0/1
	   * RxData[3] - tryb wypełnienia dc: 0 - wypełnienie na 1V, 1 - custom wypełnienie, 2 wyłącza silnik
	   * RxData[4] - custom wypełnienie dc % 0-100
	   * RxData[5] - 0 żarówka off, 1 - żarówka on
	   * */
	  switch (RxData[0]) {
	  	case 0:
	  		HAL_NVIC_DisableIRQ(TIM2_IRQn);
	  		HAL_NVIC_DisableIRQ(TIM3_IRQn);
	  		HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin, GPIO_PIN_RESET);
	  		break;
		case 1:
			HAL_NVIC_EnableIRQ(TIM3_IRQn);
			stepperNr = 1;
			if(RxData[2])
			{
				HAL_GPIO_WritePin(Stepper1_Dir_GPIO_Port,Stepper1_Dir_Pin,GPIO_PIN_SET);
			}
			else {
				HAL_GPIO_WritePin(Stepper1_Dir_GPIO_Port,Stepper1_Dir_Pin,GPIO_PIN_RESET);
			}
			break;
		case 2:
			HAL_NVIC_EnableIRQ(TIM3_IRQn);
			stepperNr = 2;
			if(RxData[2])
			{
				HAL_GPIO_WritePin(Stepper2_Dir_GPIO_Port,Stepper2_Dir_Pin,GPIO_PIN_SET);
			}
			else {
				HAL_GPIO_WritePin(Stepper2_Dir_GPIO_Port,Stepper2_Dir_Pin,GPIO_PIN_RESET);
			}
			break;
		case 3:
			HAL_NVIC_EnableIRQ(TIM3_IRQn);
			stepperNr = 3;
			if(RxData[2])
			{
				HAL_GPIO_WritePin(Stepper3_Dir_GPIO_Port,Stepper3_Dir_Pin,GPIO_PIN_SET);
			}
			else {
				HAL_GPIO_WritePin(Stepper3_Dir_GPIO_Port,Stepper3_Dir_Pin,GPIO_PIN_RESET);
			}
			break;
		case 4:
			if(RxData[3] != 2)
			{
				HAL_NVIC_EnableIRQ(TIM2_IRQn);
			}
			dcNr = 1;
			break;
		case 5:
			if(RxData[3] != 2)
			{
				HAL_NVIC_EnableIRQ(TIM2_IRQn);
			}
			dcNr = 2;
			break;
		case 6:
			if(RxData[5]==1)
			{
				HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin, GPIO_PIN_SET);
			}
			if(RxData[4]==0)
			{
				HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin, GPIO_PIN_RESET);
			}
			break;
		default:
			break;
	}
	if(RxData[3] == 2)
	{
		HAL_NVIC_DisableIRQ(TIM2_IRQn);
	}
	globalMode = RxData[1];
	if(RxData[3] == 1 && RxData[4]<=100)
	{
		dutyCycle = RxData[4];
	}
	if(!RxData[3])
	{
		dutyCycle =120; // pwm na 1V do mieszania
	}
  }
  wchdgReset = 1;
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}
/* USER CODE END 1 */

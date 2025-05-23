/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void stepperConfig()
{
	HAL_Delay(10);
	//Stepper1 config
	HAL_GPIO_WritePin(Stepper1_R_GPIO_Port,Stepper1_R_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Stepper1_Dir_GPIO_Port,Stepper1_Dir_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Stepper1_E_GPIO_Port,Stepper1_E_Pin,GPIO_PIN_RESET);//0- enable
	HAL_GPIO_WritePin(Stepper1_M0_GPIO_Port,Stepper1_M0_Pin,GPIO_PIN_SET);//110 - 1/8 stepping
	HAL_GPIO_WritePin(Stepper1_M1_GPIO_Port,Stepper1_M1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Stepper1_M2_GPIO_Port,Stepper1_M2_Pin,GPIO_PIN_RESET);
	//Stepper2 config
	HAL_GPIO_WritePin(Stepper2_R_GPIO_Port,Stepper2_R_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Stepper2_Dir_GPIO_Port,Stepper2_Dir_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Stepper2_E_GPIO_Port,Stepper2_E_Pin,GPIO_PIN_RESET);//0- enable
	HAL_GPIO_WritePin(Stepper2_M0_GPIO_Port,Stepper2_M0_Pin,GPIO_PIN_SET);//110 - 1/8 stepping
	HAL_GPIO_WritePin(Stepper2_M1_GPIO_Port,Stepper2_M1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Stepper2_M2_GPIO_Port,Stepper2_M2_Pin,GPIO_PIN_RESET);
	//Stepper2 config
	HAL_GPIO_WritePin(Stepper3_R_GPIO_Port,Stepper3_R_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Stepper3_Dir_GPIO_Port,Stepper3_Dir_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Stepper3_E_GPIO_Port,Stepper3_E_Pin,GPIO_PIN_RESET);//0- enable
	HAL_GPIO_WritePin(Stepper3_M0_GPIO_Port,Stepper3_M0_Pin,GPIO_PIN_SET);//110 - 1/8 stepping
	HAL_GPIO_WritePin(Stepper3_M1_GPIO_Port,Stepper3_M1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Stepper3_M2_GPIO_Port,Stepper3_M2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_SET);
}

void dcConfig()
{
	HAL_GPIO_WritePin(Motor1_EN_GPIO_Port,Motor1_EN_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Motor2_EN_GPIO_Port,Motor2_EN_Pin,GPIO_PIN_SET);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_NVIC_DisableIRQ(TIM2_IRQn);
  HAL_NVIC_DisableIRQ(TIM3_IRQn);
  //HAL_NVIC_DisableIRQ(TIM3_IRQn);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  Motor_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  stepperConfig();
  dcConfig();

  while (1)
  {


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPEED_PORT GPIOA
#define SPEED_PIN GPIO_PIN_15
#define AH_PORT GPIOA
#define AH_PIN GPIO_PIN_12
#define AL_PORT GPIOA
#define AL_PIN GPIO_PIN_11
#define BH_PORT GPIOA
#define BH_PIN GPIO_PIN_10
#define BL_PORT GPIOA
#define BL_PIN GPIO_PIN_9
#define CH_PORT GPIOA
#define CH_PIN GPIO_PIN_8
#define CL_PORT GPIOB
#define CL_PIN GPIO_PIN_15
#define SA_PORT GPIOB
#define SA_PIN GPIO_PIN_14
#define SB_PORT GPIOB
#define SB_PIN GPIO_PIN_13
#define SC_PORT GPIOB
#define SC_PIN GPIO_PIN_12
#define Clockwise GPIO_PIN_6 //the old one is pin 5th
#define Counter_Clockwise GPIO_PIN_5 // the old one is pin 6th
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t speed = 0;
uint8_t debug;
uint8_t test;
uint8_t mode;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  void PA(int status);
  void PB(int status);
  void PC(int status);
  void rotate(unsigned char rotation);
  void rotate_CounterClockWise(unsigned char rotation_C);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  TIM2->CCR1 = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* if (HAL_GPIO_ReadPin (GPIOA, Clockwise) == 1 && HAL_GPIO_ReadPin (GPIOA, Counter_Clockwise) == 0)
     {
       speed = 150;
       rotate(0);
     }
     else if (HAL_GPIO_ReadPin (GPIOA, Clockwise) == 0 && HAL_GPIO_ReadPin (GPIOA, Counter_Clockwise) == 1)
     {
       speed = 150;
       rotate_CounterClockWise(0);
     }
     else {
     speed = 0;
     rotate(0);
     } */
    
   //TEST MODE
   if (mode == 0){
    rotate(0);
    test = debug;}
    else {
    rotate_CounterClockWise(0);
    test = debug;
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// PHASE A
void PA(int status) 
{
  // Phase high
  if (status == 1) 
  {
    HAL_GPIO_WritePin(AH_PORT, AH_PIN, 1);
    HAL_GPIO_WritePin(AL_PORT, AL_PIN, 1);
  } 
  // Phase low
  else if (status == -1) 
  {
    HAL_GPIO_WritePin(AH_PORT, AH_PIN, 0);
    HAL_GPIO_WritePin(AL_PORT, AL_PIN, 0);
  } 
  // Disconnect
  else if (status == 0) 
  {
    HAL_GPIO_WritePin(AH_PORT, AH_PIN, 1);
    HAL_GPIO_WritePin(AL_PORT, AL_PIN, 0);
  }
}
// PHASE B
void PB(int status) 
{
  // Phase high
  if (status == 1) 
  {
    HAL_GPIO_WritePin(BH_PORT, BH_PIN, 1);
    HAL_GPIO_WritePin(BL_PORT, BL_PIN, 1);
  } 
  // Phase low
  else if (status == -1) 
  {
    HAL_GPIO_WritePin(BH_PORT, BH_PIN, 0);
    HAL_GPIO_WritePin(BL_PORT, BL_PIN, 0);
  } 
  // Disconnect
  else if (status == 0) 
  {
    HAL_GPIO_WritePin(BH_PORT, BH_PIN, 1);
    HAL_GPIO_WritePin(BL_PORT, BL_PIN, 0);
  }
}
// PHASE C
void PC(int status) 
{
  // Phase high
  if (status == 1) 
  {
    HAL_GPIO_WritePin(CH_PORT, CH_PIN, 1);
    HAL_GPIO_WritePin(CL_PORT, CL_PIN, 1);
  } 
  // Phase low
  else if (status == -1) 
  {
    HAL_GPIO_WritePin(CH_PORT, CH_PIN, 0);
    HAL_GPIO_WritePin(CL_PORT, CL_PIN, 0);
  } 
  // Disconnect
  else if (status == 0) 
  {
    HAL_GPIO_WritePin(CH_PORT, CH_PIN, 1);
    HAL_GPIO_WritePin(CL_PORT, CL_PIN, 0);
  }
}
// Control motor
void rotate(unsigned char rotation)
{
  // A -> B | W -> V | Debug 1 -> 6
  if (rotation == 0)
  {
    if (HAL_GPIO_ReadPin (SA_PORT, SA_PIN) == 1 && HAL_GPIO_ReadPin (SB_PORT, SB_PIN) == 0 && HAL_GPIO_ReadPin (SC_PORT, SC_PIN) == 1)
    {
      TIM2->CCR1 = speed * TIM2->ARR/1000;
      PA(0);
      PB(-1);
      PC(1);
      debug = 1;
    }
    else if (HAL_GPIO_ReadPin (SA_PORT, SA_PIN) == 1 && HAL_GPIO_ReadPin (SB_PORT, SB_PIN) == 0 && HAL_GPIO_ReadPin (SC_PORT, SC_PIN) == 0)
    {
      TIM2->CCR1 = speed * TIM2->ARR/1000;
      PA(-1);
      PB(0);
      PC(1);
      debug = 2;
    }
    else if (HAL_GPIO_ReadPin (SA_PORT, SA_PIN) == 1 && HAL_GPIO_ReadPin (SB_PORT, SB_PIN) == 1 && HAL_GPIO_ReadPin (SC_PORT, SC_PIN) == 0)
    {
      TIM2->CCR1 = speed * TIM2->ARR/1000;
      PA(-1);
      PB(1);
      PC(0);
      debug = 3;
    }
    else if (HAL_GPIO_ReadPin (SA_PORT, SA_PIN) == 0 && HAL_GPIO_ReadPin (SB_PORT, SB_PIN) == 1 && HAL_GPIO_ReadPin (SC_PORT, SC_PIN) == 0)
    {
      TIM2->CCR1 = speed * TIM2->ARR/1000;
      PA(0);
      PB(1);
      PC(-1);
      debug = 4;
    }
    else if (HAL_GPIO_ReadPin (SA_PORT, SA_PIN) == 0 && HAL_GPIO_ReadPin (SB_PORT, SB_PIN) == 1 && HAL_GPIO_ReadPin (SC_PORT, SC_PIN) == 1)
    {
      TIM2->CCR1 = speed * TIM2->ARR/1000;
      PA(1);
      PB(0);
      PC(-1);
      debug = 5;
    }
    else if (HAL_GPIO_ReadPin (SA_PORT, SA_PIN) == 0 && HAL_GPIO_ReadPin (SB_PORT, SB_PIN) == 0 && HAL_GPIO_ReadPin (SC_PORT, SC_PIN) == 1)
    {
      TIM2->CCR1 = speed * TIM2->ARR/1000;
      PA(1);
      PB(-1);
      PC(0);
      debug = 6;
    }
    else
    {
      TIM2->CCR1 = 0;
      
      PA(-1);
      PB(-1);
      PC(-1);
    }
  }
}

void rotate_CounterClockWise(unsigned char rotation_C)
{
  // B -> A | V -> U
  if (rotation_C == 0)
  {
    if (HAL_GPIO_ReadPin (SA_PORT, SA_PIN) == 1 && HAL_GPIO_ReadPin (SB_PORT, SB_PIN) == 0 && HAL_GPIO_ReadPin (SC_PORT, SC_PIN) == 1) // Step 1
    {
      TIM2->CCR1 = speed * TIM2->ARR/1000;
      PA(0);
      PB(1);
      PC(-1);
    }
    else if (HAL_GPIO_ReadPin (SA_PORT, SA_PIN) == 1 && HAL_GPIO_ReadPin (SB_PORT, SB_PIN) == 0 && HAL_GPIO_ReadPin (SC_PORT, SC_PIN) == 0) //Step 2
    {
      TIM2->CCR1 = speed * TIM2->ARR/1000;
      PA(1);
      PB(0);
      PC(-1);
    }
    else if (HAL_GPIO_ReadPin (SA_PORT, SA_PIN) == 1 && HAL_GPIO_ReadPin (SB_PORT, SB_PIN) == 1 && HAL_GPIO_ReadPin (SC_PORT, SC_PIN) == 0) //Step 3
    {
      TIM2->CCR1 = speed * TIM2->ARR/1000;
      PA(1);
      PB(-1);
      PC(0);
    }
    else if (HAL_GPIO_ReadPin (SA_PORT, SA_PIN) == 0 && HAL_GPIO_ReadPin (SB_PORT, SB_PIN) == 1 && HAL_GPIO_ReadPin (SC_PORT, SC_PIN) == 0) //Step 4
    {
      TIM2->CCR1 = speed * TIM2->ARR/1000;
      PA(0);
      PB(-1);
      PC(1);
    }
    else if (HAL_GPIO_ReadPin (SA_PORT, SA_PIN) == 0 && HAL_GPIO_ReadPin (SB_PORT, SB_PIN) == 1 && HAL_GPIO_ReadPin (SC_PORT, SC_PIN) == 1) //Step 5
    {
      TIM2->CCR1 = speed * TIM2->ARR/1000;
      PA(-1);
      PB(0);
      PC(1);
    }
    else if (HAL_GPIO_ReadPin (SA_PORT, SA_PIN) == 0 && HAL_GPIO_ReadPin (SB_PORT, SB_PIN) == 0 && HAL_GPIO_ReadPin (SC_PORT, SC_PIN) == 1)
    {
      TIM2->CCR1 = speed * TIM2->ARR/1000;
      PA(-1);
      PB(1);
      PC(0);
    }
    else
    {
      TIM2->CCR1 = 0;
      
      PA(-1);
      PB(-1);
      PC(-1);
    }
  }
}
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

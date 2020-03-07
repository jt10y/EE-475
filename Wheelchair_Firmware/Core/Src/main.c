/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ADC
#define ADC_MAX 4096
#define ADC_MIN 0
// PWM (stick)
#define PWM_DUTY_MAX 100
#define STICK_Y_MID 2152
#define STICK_X_MID 2033
#define STICK_MARGIN 300
// PWM (voice)
#define VOICE_PWM 50
// H-Bridge GPIOs
#define BRIDGE_GPIO GPIOD
#define BRIDGE_PIN_1 GPIO_PIN_12
#define BRIDGE_PIN_2 GPIO_PIN_13
#define BRIDGE_PIN_3 GPIO_PIN_14
#define BRIDGE_PIN_4 GPIO_PIN_15
// Ultrasonic Min Distance
#define US_MIN 25
// UART
#define UART_TIMEOUT 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static uint16_t ADC_1, ADC_2; // ADC values
static uint16_t PWM_1, PWM_2; // PWM values (range 0 - 99)
static GPIO_PinState
    BRIDGE_PIN_1_State, BRIDGE_PIN_2_State, BRIDGE_PIN_3_State, BRIDGE_PIN_4_State;
GPIO_PinState button_state;
volatile uint8_t cmd[UART_CMD_LENGTH]; // UART receive cmd
volatile uint8_t voice = 0;
uint16_t count_1, count_2, count_3;
uint16_t distance_1, distance_2, distance_3;
char msg_1[50];
char msg_2[50];
char msg_3[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static uint16_t ADC_Read(ADC_HandleTypeDef*);
static void manual_driving(void);
static void voice_driving(void);

static void forward(void);
static void backward(void);
static void stop(void);
static void clockwise(void);
static void counterclockwise(void);
static void Ultrasonic_Read(GPIO_TypeDef* GPIOx, uint16_t Trig_Pin,
          uint16_t Echo_Pin, uint16_t* count, uint16_t* distance);
static void Delay_us(uint16_t);
static void Reset_Timer_us();
static uint16_t Get_Timer_us();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// FOR DEBUGGING
int CASE;
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_DAC_Init();
  MX_I2C3_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Start TIM1 (for us delay)
  HAL_TIM_Base_Start(&htim1);
  // Start PWMs
  HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start (&htim4, TIM_CHANNEL_2);
  while (1)
  {
    if (!voice)
    {
      // Read ADC value
      ADC_1 = ADC_Read(&hadc1);
      ADC_2 = ADC_Read(&hadc2);
      // Motor controlling logics
      manual_driving();
    }
    else
    {
      HAL_UART_Receive (&huart3, (uint8_t*)cmd, UART_CMD_LENGTH, UART_TIMEOUT); // receiving message from pi
      // Read distances
      Ultrasonic_Read(GPIOE, US4_Trig_Pin, US4_Echo_Pin, &count_1, &distance_1);
      Ultrasonic_Read(GPIOE, US2_Trig_Pin, US2_Echo_Pin, &count_2, &distance_2);
      Ultrasonic_Read(GPIOD, US3_Trig_Pin, US3_Echo_Pin, &count_3, &distance_3);
      if ((distance_1 < US_MIN) || (distance_2 < US_MIN) || (distance_3 < US_MIN))
      {
        cmd[0] = ' ';
      }
      // Motor controlling logics
      voice_driving();

      // debugging message
      sprintf(msg_1, "delay_1 = %d us, distance_1 = %d cm\r\n ", count_1, distance_1);
      sprintf(msg_2, "delay_2 = %d us, distance_2 = %d cm\r\n ", count_2, distance_2);
      sprintf(msg_3, "delay_3 = %d us, distance_3 = %d cm\r\n\r\n ", count_3, distance_3);
      HAL_UART_Transmit(&huart2, (uint8_t*)msg_1, strlen(msg_1), HAL_MAX_DELAY);
      HAL_UART_Transmit(&huart2, (uint8_t*)msg_2, strlen(msg_2), HAL_MAX_DELAY);
      HAL_UART_Transmit(&huart2, (uint8_t*)msg_3, strlen(msg_3), HAL_MAX_DELAY);
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 32;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 80;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 12;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|US4_Trig_Pin|US2_Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(US1_Trig_GPIO_Port, US1_Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HBridge_STBY_GPIO_Port, HBridge_STBY_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |US3_Trig_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : US4_Echo_Pin US2_Echo_Pin */
  GPIO_InitStruct.Pin = US4_Echo_Pin|US2_Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE3 US4_Trig_Pin US2_Trig_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|US4_Trig_Pin|US2_Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 US1_Echo_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|US1_Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Joystick_Button_Pin */
  GPIO_InitStruct.Pin = Joystick_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Joystick_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : US1_Trig_Pin */
  GPIO_InitStruct.Pin = US1_Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(US1_Trig_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HBridge_STBY_Pin */
  GPIO_InitStruct.Pin = HBridge_STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HBridge_STBY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           US3_Trig_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |US3_Trig_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : US3_Echo_Pin OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = US3_Echo_Pin|OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
/**
 *  @brief Read ADC value
 *  @param pointer to ADC handler
 *  @retval ADC value
 */
static uint16_t ADC_Read(ADC_HandleTypeDef* handler)
{
  HAL_ADC_Start(handler);
  HAL_ADC_PollForConversion(handler, HAL_MAX_DELAY);
  return HAL_ADC_GetValue(handler);
}

/**
 *  @brief Manual driving mode, set GPIOs and PWM
 *  @param None
 *  @retval None
 */
static void manual_driving(void)
{
  if (ADC_1 < (STICK_Y_MID - STICK_MARGIN)) // y is 0
  {
  	// Case I, II, or III
  	backward();
  }
  else if (ADC_1 > (STICK_Y_MID + STICK_MARGIN))
  {
  	// Case VII, VIII, or IX
  	forward();
  }
  else
  {
    if (ADC_2 > (STICK_X_MID + STICK_MARGIN))
    {
	    // Case IV
    	counterclockwise();
    	CASE = 4;
    }
    else if (ADC_2 < (STICK_X_MID - STICK_MARGIN))
    {
	    // Case VI
    	clockwise();
    	CASE = 6;
    }
    else
    {
      // Case V
      stop();
      CASE = 5;
    }
  }
  // updating GPIO value
  HAL_GPIO_WritePin(BRIDGE_GPIO, BRIDGE_PIN_1, BRIDGE_PIN_1_State);
  HAL_GPIO_WritePin(BRIDGE_GPIO, BRIDGE_PIN_2, BRIDGE_PIN_2_State);
  HAL_GPIO_WritePin(BRIDGE_GPIO, BRIDGE_PIN_3, BRIDGE_PIN_3_State);
  HAL_GPIO_WritePin(BRIDGE_GPIO, BRIDGE_PIN_4, BRIDGE_PIN_4_State);
  // updating PWM value
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_1);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_2);
}

/**
 *  @brief Auto driving mode, set GPIOs and PWM
 *  @param None
 *  @retval None
 */
static void voice_driving()
{
  if (cmd[0] =='f')
  {
    backward();
  }
  else if (cmd[0] == 'l')
  {
    counterclockwise();
  }
  else if (cmd[0] == 'r')
  {
    clockwise();
  }
  else
  {
    stop();
  }
  PWM_1 = VOICE_PWM;
  PWM_2 = VOICE_PWM;
  // updating GPIO value
  HAL_GPIO_WritePin(BRIDGE_GPIO, BRIDGE_PIN_1, BRIDGE_PIN_1_State);
  HAL_GPIO_WritePin(BRIDGE_GPIO, BRIDGE_PIN_2, BRIDGE_PIN_2_State);
  HAL_GPIO_WritePin(BRIDGE_GPIO, BRIDGE_PIN_3, BRIDGE_PIN_3_State);
  HAL_GPIO_WritePin(BRIDGE_GPIO, BRIDGE_PIN_4, BRIDGE_PIN_4_State);
  // updating PWM value
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_1);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_2);
  if (cmd[0] == 'l' || cmd[0] == 'r') {
    HAL_Delay(1000);
    cmd[0] = ' ';
  }
}

/**
 * 	@brief Driving control case I, IV, VII: forward and/or turn
 * 	@param None
 * 	@retval None
 */
static void backward()
{
	// motor direction
	BRIDGE_PIN_1_State = GPIO_PIN_SET;
	BRIDGE_PIN_2_State = GPIO_PIN_RESET;
	BRIDGE_PIN_3_State = GPIO_PIN_SET;
	BRIDGE_PIN_4_State = GPIO_PIN_RESET;
	// PWM Mapping
	float PosY_Percent =
	    (float)(STICK_Y_MID - ADC_1) / STICK_Y_MID * PWM_DUTY_MAX;
	if (ADC_2 > (STICK_X_MID + STICK_MARGIN))
	{
	  // Case I
    PWM_2 = PosY_Percent;
    PWM_1 = (float)(ADC_MAX - ADC_2) / (ADC_MAX - STICK_X_MID) * PWM_2;
    CASE = 1;
	}
	else if (ADC_2 < (STICK_X_MID - STICK_MARGIN))
	{
	  // Case III
	  PWM_1 = PosY_Percent;
    PWM_2 = (float)ADC_2 / STICK_X_MID * PWM_1;
    CASE = 3;
	}
	else
	{
	  // Case II
	  PWM_1 = PosY_Percent;
	  PWM_2 = PWM_1;
	  CASE = 2;
	}
}

/**
 * 	@brief Driving control case III, VI, IX: backward and/or turn
 * 	@param None
 * 	@retval None
 */
static void forward()
{
	// motor direction
	BRIDGE_PIN_1_State = GPIO_PIN_RESET;
  BRIDGE_PIN_2_State = GPIO_PIN_SET;
  BRIDGE_PIN_3_State = GPIO_PIN_RESET;
  BRIDGE_PIN_4_State = GPIO_PIN_SET;
	// PWM Mapping
	float NegY_Percent =
	    (float)(ADC_1 - STICK_Y_MID) / (ADC_MAX - STICK_Y_MID) * PWM_DUTY_MAX;
  if (ADC_2 > (STICK_X_MID + STICK_MARGIN))
  {
    // Case VII
    PWM_2 = NegY_Percent;
    PWM_1 = (float)(ADC_MAX - ADC_2) / (ADC_MAX - STICK_X_MID) * PWM_DUTY_MAX;
    CASE = 7;
  }
  else if (ADC_2 < (STICK_X_MID - STICK_MARGIN))
  {
    // Case IX
    PWM_1 = NegY_Percent;
    PWM_2 = (float)ADC_2 / STICK_X_MID * PWM_DUTY_MAX;
    CASE = 9;
  }
  else
  {
    // Case VIII
    PWM_1 = NegY_Percent;
    PWM_2 = PWM_1;
    CASE = 8;
  }
}

/**
 *  @brief Driving control case V: stop
 *  @param None
 *  @retval None
 */
static void stop()
{
	// motor direction
	BRIDGE_PIN_1_State = GPIO_PIN_RESET;
  BRIDGE_PIN_2_State = GPIO_PIN_RESET;
  BRIDGE_PIN_3_State = GPIO_PIN_RESET;
  BRIDGE_PIN_4_State = GPIO_PIN_RESET;
	// PWM mapping
	PWM_1 = 0;
	PWM_2 = 0;
}

/**
 *  @brief Driving control case VII: rotate clockwise
 *  @param None
 *  @retval None
 */
static void clockwise()
{
	// motor direction
	BRIDGE_PIN_1_State = GPIO_PIN_SET;
  BRIDGE_PIN_2_State = GPIO_PIN_RESET;
  BRIDGE_PIN_3_State = GPIO_PIN_RESET;
  BRIDGE_PIN_4_State = GPIO_PIN_SET;
	// PWM mapping
	PWM_1 = (float)(STICK_Y_MID - ADC_2) / STICK_Y_MID * PWM_DUTY_MAX;
	PWM_2 = PWM_1;
}

/**
 *  @brief Driving control case II: rotate counter-clockwise
 *  @param None
 *  @retval None
 */
static void counterclockwise()
{
	// motor direction
	BRIDGE_PIN_1_State = GPIO_PIN_RESET;
  BRIDGE_PIN_2_State = GPIO_PIN_SET;
  BRIDGE_PIN_3_State = GPIO_PIN_SET;
  BRIDGE_PIN_4_State = GPIO_PIN_RESET;
	// PWM mapping
	PWM_1 = (float)(ADC_2 - STICK_Y_MID) / (ADC_MAX - STICK_Y_MID) * PWM_DUTY_MAX;
	PWM_2 = PWM_1;
}

/**
 *  @brief Get ultrasonic distance
 *  @param GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin
 *  @retval None
 */
static void Ultrasonic_Read(GPIO_TypeDef* GPIOx, uint16_t Trig_Pin,
    uint16_t Echo_Pin, uint16_t* count, uint16_t* distance)
{
  HAL_GPIO_WritePin(GPIOx, Trig_Pin, GPIO_PIN_SET);
  Delay_us(10);
  HAL_GPIO_WritePin(GPIOx, Trig_Pin, GPIO_PIN_RESET);
  while (HAL_GPIO_ReadPin(GPIOx, Echo_Pin) == GPIO_PIN_RESET);
  Reset_Timer_us();
  while ((HAL_GPIO_ReadPin(GPIOx, Echo_Pin) == GPIO_PIN_SET) && (Get_Timer_us() < 65535));
  *count = Get_Timer_us();
  *distance = (float)(0.0343 * (*count) / 2);
}

/**
 *  @brief Microsecond delay
 *  @param None
 *  @retval None
 */
static void Delay_us(uint16_t delay)
{
  __HAL_TIM_SET_COUNTER (&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

/**
 *  @brief Reset Microsecond timer
 *  @param None
 *  @retval None
 */
static void Reset_Timer_us(){
  __HAL_TIM_SET_COUNTER (&htim1, 0);
}

/**
 *  @brief Get Microsecond timer current count
 *  @param None
 *  @retval None
 */
static uint16_t Get_Timer_us()
{
  return __HAL_TIM_GET_COUNTER(&htim1);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

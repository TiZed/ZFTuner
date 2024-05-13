/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_redirect.h"
#include "usbd_cdc_if.h"

#include "latching_relays.h"

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CORDIC_HandleTypeDef hcordic;

FDCAN_HandleTypeDef hfdcan1;

FMAC_HandleTypeDef hfmac;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
relay_t Cap_InOut = {Relays_R1_GPIO_Port, CapRels_1_GPIO_Port, Relays_S1_Pin, Relays_R1_Pin, CapRels_1_Pin} ;

relay_t Cap_K402 = {Relays_R1_GPIO_Port, CapRels_1_GPIO_Port, Relays_S2_Pin, Relays_R2_Pin, CapRels_1_Pin} ;
relay_t Cap_K403 = {Relays_R1_GPIO_Port, CapRels_1_GPIO_Port, Relays_S3_Pin, Relays_R3_Pin, CapRels_1_Pin} ;
relay_t Cap_K404 = {Relays_R1_GPIO_Port, CapRels_1_GPIO_Port, Relays_S4_Pin, Relays_R4_Pin, CapRels_1_Pin} ;
relay_t Cap_K405 = {Relays_R1_GPIO_Port, CapRels_2_GPIO_Port, Relays_S1_Pin, Relays_R1_Pin, CapRels_2_Pin} ;
relay_t Cap_K406 = {Relays_R1_GPIO_Port, CapRels_2_GPIO_Port, Relays_S2_Pin, Relays_R2_Pin, CapRels_2_Pin} ;
relay_t Cap_K407 = {Relays_R1_GPIO_Port, CapRels_2_GPIO_Port, Relays_S3_Pin, Relays_R3_Pin, CapRels_2_Pin} ;
relay_t Cap_K408 = {Relays_R1_GPIO_Port, CapRels_2_GPIO_Port, Relays_S4_Pin, Relays_R4_Pin, CapRels_2_Pin} ;

relay_t Ind_K501 = {Relays_R1_GPIO_Port, IndRels_1_GPIO_Port, Relays_S1_Pin, Relays_R1_Pin, IndRels_1_Pin} ;
relay_t Ind_K502 = {Relays_R1_GPIO_Port, IndRels_1_GPIO_Port, Relays_S2_Pin, Relays_R2_Pin, IndRels_1_Pin} ;
relay_t Ind_K503 = {Relays_R1_GPIO_Port, IndRels_1_GPIO_Port, Relays_S3_Pin, Relays_R3_Pin, IndRels_1_Pin} ;
relay_t Ind_K504 = {Relays_R1_GPIO_Port, IndRels_1_GPIO_Port, Relays_S4_Pin, Relays_R4_Pin, IndRels_1_Pin} ;
relay_t Ind_K505 = {Relays_R1_GPIO_Port, IndRels_2_GPIO_Port, Relays_S1_Pin, Relays_R1_Pin, IndRels_2_Pin} ;
relay_t Ind_K506 = {Relays_R1_GPIO_Port, IndRels_2_GPIO_Port, Relays_S2_Pin, Relays_R2_Pin, IndRels_2_Pin} ;
relay_t Ind_K507 = {Relays_R1_GPIO_Port, IndRels_2_GPIO_Port, Relays_S3_Pin, Relays_R3_Pin, IndRels_2_Pin} ;

volatile uint16_t adc_data[4] ;

uint16_t rf_fwd_mv ;
uint16_t rf_rfl_mv ; 
uint16_t ref_volt ;
uint16_t int_temp ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CORDIC_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FMAC_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t getActiveBank() {
    volatile uint32_t remap = READ_BIT(SYSCFG->MEMRMP, 0x1 << 8);
    return remap == 0 ? 1 : 2;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CORDIC_Init();
  MX_FDCAN1_Init();
  MX_FMAC_Init();
  MX_USB_Device_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) ;
  HAL_GPIO_WritePin(Op_LED_GPIO_Port, Op_LED_Pin, GPIO_PIN_RESET) ;
  HAL_GPIO_WritePin(Error_LED_GPIO_Port, Error_LED_Pin, GPIO_PIN_SET) ;

  GPIO_TypeDef * Relays_Port = Relays_R1_GPIO_Port ;
  uint16_t All_Relays = Relays_S1_Pin | Relays_R1_Pin | Relays_S2_Pin | Relays_S3_Pin | Relays_R3_Pin | Relays_S4_Pin | Relays_R4_Pin ; 

  GPIO_TypeDef * Relays_Power_Port = IndRels_1_GPIO_Port ;
  uint16_t All_Power = IndRels_1_Pin | IndRels_2_Pin | CapRels_1_Pin | CapRels_2_Pin ;

  Relays_Port->BRR = All_Relays ;
  Relays_Power_Port->BRR = All_Power ;

  HAL_GPIO_WritePin(SWR_SHDN_GPIO_Port, SWR_SHDN_Pin, GPIO_PIN_SET) ; // Shutdown SWR meter
  HAL_GPIO_WritePin(CAN_STBY_GPIO_Port, CAN_STBY_Pin, GPIO_PIN_RESET) ; // Shutdown CAN transceiver 

  HAL_GPIO_WritePin(KEY_GPIO_Port, KEY_Pin, GPIO_PIN_RESET) ; // Disable KEY signal

  uint32_t Caps[] = {12, 22, 47, 100, 220, 470, 100}  ; // pF
  uint32_t Inds[] = {505, 1000, 2200, 5000, 10000, 23000, 44700} ; // 0.1 nH

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(500) ;
  printf("Turn off 'Err' LED\n") ;
  HAL_GPIO_WritePin(Error_LED_GPIO_Port, Error_LED_Pin, GPIO_PIN_RESET) ;
  uint8_t fw_bank = getActiveBank() ;
  printf("Current FW bank: %d", fw_bank) ;

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FMAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMAC_Init(void)
{

  /* USER CODE BEGIN FMAC_Init 0 */

  /* USER CODE END FMAC_Init 0 */

  /* USER CODE BEGIN FMAC_Init 1 */

  /* USER CODE END FMAC_Init 1 */
  hfmac.Instance = FMAC;
  if (HAL_FMAC_Init(&hfmac) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMAC_Init 2 */

  /* USER CODE END FMAC_Init 2 */

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
  htim1.Init.Prescaler = 1000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim1.Init.Period = 7200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Op_LED_Pin|Error_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Relays_R4_Pin|Relays_S4_Pin|Relays_R3_Pin|Relays_S3_Pin
                          |Relays_R2_Pin|Relays_S2_Pin|Relays_R1_Pin|Relays_S1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, KEY_Pin|CapRels_1_Pin|CapRels_2_Pin|IndRels_1_Pin
                          |IndRels_2_Pin|SWR_SHDN_Pin|CAN_STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Op_LED_Pin Error_LED_Pin */
  GPIO_InitStruct.Pin = Op_LED_Pin|Error_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PG10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : Relays_R4_Pin Relays_S4_Pin Relays_R3_Pin Relays_S3_Pin
                           Relays_R2_Pin Relays_S2_Pin Relays_R1_Pin Relays_S1_Pin */
  GPIO_InitStruct.Pin = Relays_R4_Pin|Relays_S4_Pin|Relays_R3_Pin|Relays_S3_Pin
                          |Relays_R2_Pin|Relays_S2_Pin|Relays_R1_Pin|Relays_S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_Pin CapRels_1_Pin CapRels_2_Pin IndRels_1_Pin
                           IndRels_2_Pin SWR_SHDN_Pin CAN_STBY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin|CapRels_1_Pin|CapRels_2_Pin|IndRels_1_Pin
                          |IndRels_2_Pin|SWR_SHDN_Pin|CAN_STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : START_Pin */
  GPIO_InitStruct.Pin = START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(START_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB4 PB5
                           PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

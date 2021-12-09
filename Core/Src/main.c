/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
//#include <eeprom.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CCS811_I2C_ADDR 0x5A
#define CCS811_BOOTLOADER_APP_START 0xF4
#define CCS811_REG_STATUS 0x00
#define CCS811_REG_MEAS_MODE 0x01
#define CCS811_REG_ALG_RESULT_DATA 0x02
#define CCS811_REG_ERROR_ID 0xE0

#define SHT30_I2C_ADDR 0x44
#define SHT30_COMMAND_READ_ONESHOT 0x2C10
#define SHT30_COMMAND_START_PERIODIC 0x2B32
#define SHT30_COMMAND_STOP_PERIODIC 0x3093
#define SHT30_COMMAND_READ_DATA 0xE000  // followed by I2C read header, returns 6 bytes (temp MSB, temp LSB, CRC, hum MSB, hum LSB, CRC)
#define SHT30_COMMAND_SOFT_RESET 0x30A2
#define SHT30_COMMAND_START_HEATER 0x306D
#define SHT30_COMMAND_STOP_HEATER 0x3066
#define SHT30_COMMAND_READ_STATUS 0xF32D // followed by I2C read header, returns 3 bytes (register MSB, register LSB, CRC)
#define SHT30_COMMAND_CLEAR_STATUS 0x3041

#define I2C_GENERAL_CALL_RESET 0x0006

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
typedef struct
{
  uint8_t name[128];
  uint16_t air;
  uint16_t heat;
  uint16_t water;
} environment_t;

typedef struct
{
  uint8_t name[128];
  uint16_t air;
  uint16_t heat;
  uint16_t light;
  uint16_t water;
} setpoint_t;

static volatile uint8_t uart_recv[128] = {0};
static volatile environment_t conditions = {0};
static volatile setpoint_t setpoints = {0};
static volatile uint8_t enable_auto = 0;
static volatile RTC_TimeTypeDef time = {0};
static volatile RTC_DateTypeDef data = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

static uint8_t CCS811_begin(void);
static uint16_t CCS811_get_eCO2(void);
static void CCS811_check_status(void);

static void SHT30_begin(void);
static uint16_t SHT30_get_temp(void);

static uint16_t EK1940_get_moisture(void);

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
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  // initialize sensors
  HAL_ADCEx_Calibration_Start(&hadc);
  CCS811_begin();
  SHT30_begin();

  // initialize uart rx
  HAL_UART_Receive_IT(&huart1, uart_recv, 1);

//  HAL_FLASH_Unlock();
//  EE_Init();
//  uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777};
//  uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0};
//  uint16_t VarValue = 0;

  strcpy(conditions.name, "grand wizard oscar fucking mayer");

  // TODO initialize saved state
  //
  //
  //

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(1000);
    HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &data, RTC_FORMAT_BIN);
    // read sensors
    uint16_t hours = time.Hours;
    uint16_t moisture = EK1940_get_moisture();
    uint16_t co2 = CCS811_get_eCO2();
    uint16_t temp = SHT30_get_temp();

    conditions.air = co2;
    conditions.heat = temp;
    conditions.water = moisture;

    if (enable_auto)
    {
      memcpy(conditions.name, setpoints.name, 128);
      HAL_GPIO_WritePin(light_GPIO_Port,  light_Pin,  (hours < setpoints.light));
      HAL_GPIO_WritePin(air_GPIO_Port,    air_Pin,    (co2 < setpoints.air));
      HAL_GPIO_WritePin(heat_GPIO_Port,   heat_Pin,   (temp < setpoints.heat));
      HAL_GPIO_WritePin(water_GPIO_Port,  water_Pin,  (moisture < setpoints.water));
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_8B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable Calibration
  */
  if (HAL_RTCEx_SetCalibrationOutPut(&hrtc, RTC_CALIBOUTPUT_1HZ) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, heat_Pin|water_Pin|light_Pin|air_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : heat_Pin water_Pin light_Pin air_Pin */
  GPIO_InitStruct.Pin = heat_Pin|water_Pin|light_Pin|air_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static uint8_t CCS811_begin(void)
{
  uint8_t byte_buffer = CCS811_BOOTLOADER_APP_START;
  if (HAL_OK != HAL_I2C_Master_Transmit(&hi2c1, CCS811_I2C_ADDR << 1, (uint8_t *) &byte_buffer, 1, 1000))
  {
    return 1;
  }

  byte_buffer = 0b10000;
  if (HAL_OK != HAL_I2C_Mem_Write(&hi2c1, CCS811_I2C_ADDR << 1, CCS811_REG_MEAS_MODE, 1, &byte_buffer, 1, 1000))
  {
    return 1;
  }

  return 0;
}

static uint16_t CCS811_get_eCO2(void)
{
  uint8_t result[2] = {0};
  if (HAL_OK != HAL_I2C_Mem_Read(&hi2c1, CCS811_I2C_ADDR << 1, CCS811_REG_ALG_RESULT_DATA, 1, (uint8_t *) &result, 2, 1000))
    {
      while (1);
    }
  return (((uint16_t) result[0] << 8) | (uint16_t) result[1]);
}

static void CCS811_check_status(void)
{
  uint8_t status = 0;
  if (HAL_OK != HAL_I2C_Mem_Read(&hi2c1, CCS811_I2C_ADDR << 1, CCS811_REG_STATUS, 1, &status, 1, 1000))
  {
    while (1);
  }
  __asm volatile ("nop");
}

static void SHT30_begin(void)
{

}

static uint16_t SHT30_get_temp(void)
{
  uint8_t recv[6] = {0};
  uint16_t temp = 0;
  if (HAL_OK != HAL_I2C_Mem_Read(&hi2c1, SHT30_I2C_ADDR << 1, SHT30_COMMAND_READ_ONESHOT, 2, recv, 6, 1000))
  {
    while (1);
  }
  temp = (uint16_t) recv[0] << 8 | recv[1];
  return (uint16_t) ((temp / 256 + 0.5) * 315 / 256 - 49); // sensor output to degrees F equation, page 30 of SHT30 datasheet
}

static void SHT30_check_status(void)
{
  uint8_t recv[3] = {0};
  if (HAL_OK != HAL_I2C_Mem_Read(&hi2c1, SHT30_I2C_ADDR << 1, SHT30_COMMAND_READ_STATUS, 2, recv, 3, 1000))
  {
    while (1);
  }
}

static uint16_t EK1940_get_moisture(void)
{
  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, 1000);
  uint32_t raw = HAL_ADC_GetValue(&hadc);
  return 100 - ((100 * (raw - 75.0)) / (205 - 75));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (uart_recv[0] == 0xAA)
  {
    // get name and conditions
    HAL_UART_Transmit(huart, &conditions, sizeof(conditions), 500);
  }
  else if (uart_recv[0] == 0xBB)
  {
    if (HAL_OK == HAL_UART_Receive(huart, &setpoints, 136, 500))
    {
      enable_auto = 1;
    }
//    memcpy(&setpoints, uart_recv, 4);
  }
  else if (uart_recv[0] == 0xCC)
  {
    HAL_GPIO_TogglePin(air_GPIO_Port, air_Pin);
  }
  else if (uart_recv[0] == 0xDD)
  {
    HAL_GPIO_TogglePin(heat_GPIO_Port, heat_Pin);
  }
  else if (uart_recv[0] == 0xEE)
  {
    HAL_GPIO_TogglePin(light_GPIO_Port, light_Pin);
  }
  else if (uart_recv[0] == 0xFF)
  {
    HAL_GPIO_TogglePin(water_GPIO_Port, water_Pin);
  }
  HAL_UART_Receive_IT(&huart1, uart_recv, 1);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

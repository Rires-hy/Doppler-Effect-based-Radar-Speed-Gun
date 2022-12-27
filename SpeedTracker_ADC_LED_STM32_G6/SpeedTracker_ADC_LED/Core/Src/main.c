/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright EEEE2059(2020) Group 6.
 * All rights reserved.
 *
 * PB6 -> USART1_TX
 * PB7 -> USART1_RX
 * Baud rate: must be 57600 bits/s
 *
 * PA0 -> ADC_INPUT
 *
 * This program is only used for final demonstration!
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stm32l476g_discovery.h>
#include <stm32l476g_discovery_glass_lcd.h>
#include <arm_math.h>
#include <arm_const_structs.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
	USER_SPEED_MODE_KPH = 0U, USER_SPEED_MODE_MPS = 1U, USER_SPEED_MODE_MPH = 2U
} USER_SpeedModeTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/**
 ******************************************************************************
 *Comment this line if you want to show speed
 *Or it will show frequency
 *When it is in FREQUENCY MODE, speed unit switching is disabled
 ******************************************************************************
 */
//#define SHOW_FREQ
#define SHOW_NUMBER

#ifdef SHOW_NUMBER
#define SHOW_NUMBER_START_VALUE 88
#endif

#define SAMPLE_FREQ (4000)
#define FFT_SIZE (4096)
#define N_BINS FFT_SIZE
#define OUTPUT_FREQ_OFFSET (1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

LCD_HandleTypeDef hlcd;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//temp
uint8_t leftvalue = 0;
uint8_t rightvalue = 0;

//ADC
uint32_t currentADCValue = 0;
float32_t realVoltage = 0.0;

//FFT
float32_t fftMaxAmp = 0;
uint32_t fftMaxFreq = 0;
uint8_t isNewFreq = 0;	//1->enable	//0->unable
uint16_t fftIndex = 0;
float32_t fftInput[FFT_SIZE];
float32_t fftOutput[FFT_SIZE];
float32_t cmplxMagOutput[FFT_SIZE / 2];
arm_rfft_fast_instance_f32 S;

//LCD
char LCDTextBuffer[7];
uint8_t currentSpeedMode = USER_SPEED_MODE_KPH;
const uint8_t speedModeNum = 3;
float32_t realFreq = 0;
float32_t speedValue = 0;
const float32_t freqResolution = (float32_t) SAMPLE_FREQ / (float32_t) N_BINS;
const float32_t freqMax = (float32_t) 10525 / (float32_t) 9;
//const float32_t freqMin = (float32_t) 2105 / (float32_t) 27;
const float32_t freqMin = 0;
const float32_t ampMin = 34;
const float32_t factorFreqKPH = (float32_t) 108 / (float32_t) 2105 * (float32_t)100;	//times 100 for .XX
const float32_t factorFreqMPS = (float32_t) 6 / (float32_t) 421 * (float32_t)100;		//times 100 for .XX
const float32_t factorFreqMPH = (float32_t) 21600 / (float32_t) 421;
uint8_t isLCDPoint = 0;	//1->enable	//0->unable

//LED Display (through USART1 communication)
uint8_t LEDTextBuffer;
uint8_t speedValueLED = 0;
const float32_t factorFreqLEDKPH = (float32_t) 108 / (float32_t) 2105;

//Device state
uint8_t isPause = 0;	//1->pause	//0->run

//SHOW_NUMBER_VALUE
#ifdef SHOW_NUMBER
uint8_t showNumberValue = SHOW_NUMBER_START_VALUE;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LCD_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

void Initialize();
void User_ReadADC();
void User_FFT();
void RefreshLCD();

#ifndef SHOW_NUMBER
void ShowSpeedMode()
#endif

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
  MX_LCD_Init();
  MX_USART2_UART_Init();
  MX_USB_HOST_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  Initialize();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

        //Refresh LCD
        if (isNewFreq)
        {
            RefreshLCD();
            isNewFreq = 0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LCD Initialization Function
  * @param None
  * @retval None
  */
static void MX_LCD_Init(void)
{

  /* USER CODE BEGIN LCD_Init 0 */

  /* USER CODE END LCD_Init 0 */

  /* USER CODE BEGIN LCD_Init 1 */

  /* USER CODE END LCD_Init 1 */
  hlcd.Instance = LCD;
  hlcd.Init.Prescaler = LCD_PRESCALER_1;
  hlcd.Init.Divider = LCD_DIVIDER_16;
  hlcd.Init.Duty = LCD_DUTY_1_4;
  hlcd.Init.Bias = LCD_BIAS_1_4;
  hlcd.Init.VoltageSource = LCD_VOLTAGESOURCE_INTERNAL;
  hlcd.Init.Contrast = LCD_CONTRASTLEVEL_0;
  hlcd.Init.DeadTime = LCD_DEADTIME_0;
  hlcd.Init.PulseOnDuration = LCD_PULSEONDURATION_0;
  hlcd.Init.MuxSegment = LCD_MUXSEGMENT_DISABLE;
  hlcd.Init.BlinkMode = LCD_BLINKMODE_OFF;
  hlcd.Init.BlinkFrequency = LCD_BLINKFREQUENCY_DIV8;
  hlcd.Init.HighDrive = LCD_HIGHDRIVE_DISABLE;
  if (HAL_LCD_Init(&hlcd) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LCD_Init 2 */

  /* USER CODE END LCD_Init 2 */

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
  htim1.Init.Prescaler = 99;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 198;
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
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_VBUS_GPIO_Port, OTG_FS_VBUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : JOY_LEFT_Pin JOY_RIGHT_Pin JOY_UP_Pin JOY_DOWN_Pin */
  GPIO_InitStruct.Pin = JOY_LEFT_Pin|JOY_RIGHT_Pin|JOY_UP_Pin|JOY_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD_R_Pin */
  GPIO_InitStruct.Pin = LD_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LD_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD_G_Pin */
  GPIO_InitStruct.Pin = LD_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LD_G_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT_RST_Pin */
  GPIO_InitStruct.Pin = EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXT_RST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void Initialize()
{
	//Start Timer1 interrupt
	HAL_TIM_Base_Start_IT(&htim1);

	//Initiate LCD by BSP
	BSP_LCD_GLASS_Init();
	BSP_LCD_GLASS_Contrast(LCD_CONTRASTLEVEL_5);

	//Initiate LED by BSP
	BSP_LED_Init(LED_GREEN);
	BSP_LED_Init(LED_RED);

	//ADC1 Calibration
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

	//RFFT Instance initialization
	arm_rfft_fast_init_f32(&S, FFT_SIZE);
}

//Used to ADC reading and filling FFInput array
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim1)
	{
		if (!isPause)
		{
			BSP_LED_Toggle(LED_GREEN);
			User_ReadADC();
			if (fftIndex >= FFT_SIZE - 1)
			{
				User_FFT();
				isNewFreq = 1;
				BSP_LED_Toggle(LED_RED);
			}
			else
			{
				fftInput[fftIndex] = (float32_t) currentADCValue;
				fftIndex++;
			}
			BSP_LED_Toggle(LED_GREEN);
		}
	}
}

void User_ReadADC()
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	currentADCValue = HAL_ADC_GetValue(&hadc1);
	realVoltage = currentADCValue / 4095.0 * 3.3;
}

void User_FFT()
{
	arm_rfft_fast_f32(&S, fftInput, fftOutput, 0);
	arm_cmplx_mag_f32(fftOutput, cmplxMagOutput, FFT_SIZE / 2);
	arm_max_f32(cmplxMagOutput + 15, FFT_SIZE / 2 - 15, &fftMaxAmp, &fftMaxFreq);
	fftMaxFreq += 15;
	fftIndex = 0;
}

void RefreshLCD()
{
#ifndef SHOW_NUMBER

#ifndef SHOW_FREQ
	if (fftMaxAmp < ampMin)
	{
		snprintf(LCDTextBuffer, 7, "%6s", "NULL");
		isLCDPoint = 0;
		LEDTextBuffer = 0xAA;    //EP
	}
	else
	{
		realFreq = (fftMaxFreq + OUTPUT_FREQ_OFFSET) * freqResolution;
		if (realFreq > freqMax)
		{
			snprintf(LCDTextBuffer, 7, "%6s", "HIGH");
			isLCDPoint = 0;
			LEDTextBuffer = 0xBB;   //HI
		}
		else if (realFreq < freqMin)
		{
			snprintf(LCDTextBuffer, 7, "%6s", "LOW");
			isLCDPoint = 0;
			LEDTextBuffer = 0xCC;   //LO
		}
		else
		{
			switch (currentSpeedMode)
			{
				case USER_SPEED_MODE_MPH:
					speedValue = realFreq * factorFreqMPH;
					isLCDPoint = 0;
					break;
				case USER_SPEED_MODE_MPS:
					speedValue = realFreq * factorFreqMPS;
					isLCDPoint = 1;
					break;
				default:	//USER_SPEED_MODE_KPH
				    speedValue = realFreq * factorFreqKPH;
					isLCDPoint = 1;
					break;
			}
			snprintf(LCDTextBuffer, 7, "%6d", (uint16_t) speedValue);

			//LED display must show in unit: KPH
			speedValueLED = realFreq * factorFreqLEDKPH;
			speedValueLED = ((uint16_t)speedValueLED / 10) << 4 | ((uint16_t)speedValueLED % 10);
			LEDTextBuffer = speedValueLED;
		}
	}

	if(isLCDPoint)
	{
		uint16_t tempExString[6];
		for(int i = 0; i < 6; i++)
		{
			tempExString[i] = LCDTextBuffer[i];
		}
		tempExString[3] |= DOT;
		BSP_LCD_GLASS_DisplayStrDeci(tempExString);
	}
	else
	{
		BSP_LCD_GLASS_DisplayString(LCDTextBuffer);
	}

	//LED display data sending
    HAL_UART_Transmit(&huart2, &LEDTextBuffer, 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, &LEDTextBuffer, 1, HAL_MAX_DELAY);
#endif

#ifdef SHOW_FREQ
	realFreq = (fftMaxFreq + OUTPUT_FREQ_OFFSET) * freqResolution;
	snprintf(LCDTextBuffer, 7, "%6d", (uint16_t) realFreq);
	BSP_LCD_GLASS_DisplayString(LCDTextBuffer);

	//LED display show "FR"
	LEDTextBuffer = 0xDD;   //FR
    HAL_UART_Transmit(&huart2, &LEDTextBuffer, 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, &LEDTextBuffer, 1, HAL_MAX_DELAY);
#endif

#else   //SHOW_NUMBER
    speedValueLED = (showNumberValue / 10) << 4 | (showNumberValue % 10);
    LEDTextBuffer = speedValueLED;
    //LED
    HAL_UART_Transmit(&huart2, &LEDTextBuffer, 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, &LEDTextBuffer, 1, HAL_MAX_DELAY);
    //LCD
    snprintf(LCDTextBuffer, 7, "%6d", (uint16_t) showNumberValue);
    BSP_LCD_GLASS_DisplayString(LCDTextBuffer);

    showNumberValue++;
    if(showNumberValue >= 100)
    {
        showNumberValue = 0;
    }
#endif
}

//Control Speed Mode and Pause
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#ifndef SHOW_NUMBER
	switch(GPIO_Pin)
	{
#ifndef SHOW_FREQ
		case JOY_UP_Pin:
			currentSpeedMode = (currentSpeedMode == speedModeNum - 1) ? 0 : currentSpeedMode + 1;
			ShowSpeedMode();
			HAL_Delay(1500);
			fftIndex = 0;
			break;
		case JOY_DOWN_Pin:
			currentSpeedMode = (currentSpeedMode == 0) ? speedModeNum - 1 : currentSpeedMode - 1;
			ShowSpeedMode();
			HAL_Delay(1500);
			fftIndex = 0;
			break;
#endif
		case JOY_RIGHT_Pin:
		case JOY_LEFT_Pin:
			if(isPause)
			{
				snprintf(LCDTextBuffer, 7, "% 6s", "---RUN");
				BSP_LCD_GLASS_DisplayString(LCDTextBuffer);	//Why????????????
				BSP_LCD_GLASS_DisplayString(LCDTextBuffer);	//For an unknown reason, here should have two refresh
				HAL_Delay(1500);
				BSP_LED_Off(LED_GREEN);
				BSP_LED_Off(LED_RED);
				fftIndex = 0;
				isPause = 0;
				//HAL_TIM_Base_Start_IT(&htim1);
			}
			else
			{
				//HAL_TIM_Base_Stop_IT(&htim1);
				isPause = 1;
				BSP_LED_Off(LED_GREEN);
				BSP_LED_Off(LED_RED);
				snprintf(LCDTextBuffer, 7, "% 6s", "-PAUSE");
				BSP_LCD_GLASS_DisplayString(LCDTextBuffer);
				HAL_Delay(1500);
				BSP_LED_On(LED_GREEN);
				BSP_LED_On(LED_RED);
				RefreshLCD();
			}
			break;
		default:
			break;
	}
#endif
}

#ifndef SHOW_NUMBER
void ShowSpeedMode()
{
	switch (currentSpeedMode)
	{
		case USER_SPEED_MODE_MPH:
			snprintf(LCDTextBuffer, 7, "% 6s", "---MPH");
			break;
		case USER_SPEED_MODE_MPS:
			snprintf(LCDTextBuffer, 7, "% 6s", "---MPS");
			break;
		case USER_SPEED_MODE_KPH:
			snprintf(LCDTextBuffer, 7, "% 6s", "---KPH");
			break;
		default:
			break;
	}
	BSP_LCD_GLASS_DisplayString(LCDTextBuffer);	//Why????????????
	BSP_LCD_GLASS_DisplayString(LCDTextBuffer);	//For an unknown reason, here should have two refresh
}
#endif

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

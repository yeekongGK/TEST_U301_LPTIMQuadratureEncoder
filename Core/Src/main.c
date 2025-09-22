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
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRACSENS_CFG_AUTORELOAD_VALUE 0xFFFF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

LPTIM_HandleTypeDef hlptim1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

typedef enum
{
	NORMAL_CounterMode,
	INVERT_CounterMode,
}PULSER_CounterMode_t;

typedef enum
{
	UNKNOWN_CounterDirection,
	FORWARD_CounterDirection,
	BACKWARD_CounterDirection,
}PULSER_CounterDirection_t;

static PULSER_CounterMode_t eMode= NORMAL_CounterMode;
static uint16_t uwCompareValue;

static PULSER_CounterDirection_t eCounterDirection= UNKNOWN_CounterDirection;
static int32_t lCntrMultiplier= 0;
static int32_t lCntrErrorReading= 0;

typedef enum
{
	NONE_CounterErrorState= 0,
	FWD_EXPECTING_BWD_CounterErrorState,
	BWD_EXPECTING_FWD_CounterErrorState,
	FWD_EXPECTING_BWD_END_CounterErrorState,
	BWD_EXPECTING_FWD_END_CounterErrorState,
}TRACSENS_CounterErrorState_t;

typedef struct
{
	bool enableErrorPatternCheck;
	bool useCompensatedValue;
	uint16_t errorPatternConfirmationCount;

	int32_t rteOffsetValue;
	int32_t rteLastSavedValue;
	uint32_t rteErrorPatternCount;
	bool rteErrorPatternCompensationStarted;
	TRACSENS_CounterErrorState_t rteErrorPatternState;
	int32_t rteErrorPatternPreviousPulse;
	bool rteErrorPatternJustStarted;
}TRACSENS_t;

static TRACSENS_t *pConfig;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void UART_Printf(char *format, ...);
void TRACSENS_StartCounting();

void TRACSENS_DisplayInfo(void);
static int32_t TRACSENS_GetCounter(void);

void TRACSENS_CompareCallback(LPTIM_HandleTypeDef *hlptim);
void TRACSENS_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim);
void TRACSENS_CounterChangedToUpCallback(LPTIM_HandleTypeDef *hlptim);
void TRACSENS_CounterChangedToDownCallback(LPTIM_HandleTypeDef *hlptim);
void TRACSENS_ChangedToUpErrorHandling(void);
void TRACSENS_ChangedToDownErrorHandling(void);
static void TRACSENS_Power(bool _enable);
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
  MX_USART2_UART_Init();
  MX_LPTIM1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  TRACSENS_StartCounting();

	if (HAL_LPTIM_Counter_Start_IT(&hlptim1) != HAL_OK)
	{
	  Error_Handler();
	}

	/* Disable autoreload write complete interrupt */
	__HAL_LPTIM_DISABLE_IT(&hlptim1, LPTIM_IT_ARROK);

	uint32_t value = LL_LPTIM_OC_GetCompareCH1(LPTIM1);

	UART_Printf("Main start %d\n\r", value);
	TRACSENS_DisplayInfo();
	HAL_Delay(1000);

	int32_t reading = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  reading = TRACSENS_GetCounter();
  //	  UART_Printf("reading %d %d\r\n",reading, lCntrMultiplier);
	  TRACSENS_DisplayInfo();
	  HAL_Delay(2000);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 6;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.UltraLowPowerClock.Polarity = LPTIM_CLOCKPOLARITY_RISING;
  hlptim1.Init.UltraLowPowerClock.SampleTime = LPTIM_CLOCKSAMPLETIME_DIRECTTRANSITION;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.Period = 100;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_EXTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  hlptim1.Init.RepetitionCounter = 0;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

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

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x25;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x10;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void TRACSENS_DisplayInfo(void)
{
	static int32_t _prevPulse= 0;
	int32_t _currPulse= TRACSENS_GetCounter();
//	if(_prevPulse!= _currPulse)
//	{
//		_prevPulse= _currPulse;

		UART_Printf("#stat:mag2: display > curr: %d errCnt: %d errState: %d errPulse: %d\r\n",
				_currPulse, pConfig->rteErrorPatternCount, pConfig->rteErrorPatternState, pConfig->rteErrorPatternPreviousPulse);
//	}
}

static int32_t TRACSENS_GetCounter(void)
{
	int32_t _value;
    do/* 2 consecutive readings need to be the same*/
    {
    	_value= LL_LPTIM_GetCounter(LPTIM1);
    }while(LL_LPTIM_GetCounter(LPTIM1)!= _value);

//	if(INVERT_CounterMode== eMode)
//	{
//		_value= 0xFFFF& ((TRACSENS_CFG_AUTORELOAD_VALUE+ 1)- _value);
//	}
//
//    _value+= (lCntrMultiplier* (TRACSENS_CFG_AUTORELOAD_VALUE+ 1));

    return _value;
}

void TRACSENS_CompareCallback(LPTIM_HandleTypeDef *hlptim)
{
	UART_Printf("TRACSENS_CompareCallback\n\r");
	if(UNKNOWN_CounterDirection== eCounterDirection)/*initially we don't know. we choose forward cos this compare confirming it forward*/
	{
		eCounterDirection= FORWARD_CounterDirection;
	}

	//DBG_Print("CompareCallback:%d, multiplier:%d \r\n", eCounterDirection, lCntrMultiplier);
}

void TRACSENS_CounterChangedToUpCallback(LPTIM_HandleTypeDef *hlptim)
{
	UART_Printf("TRACSENS_CounterChangedToUpCallback\n\r");
	eCounterDirection= FORWARD_CounterDirection;

	if(true== pConfig->enableErrorPatternCheck)
	{
		TRACSENS_ChangedToUpErrorHandling();
	}

	//DBG_Print("CounterChangedToUpCallback:%d, multiplier:%d \r\n", eCounterDirection, lCntrMultiplier);
}

void TRACSENS_CounterChangedToDownCallback(LPTIM_HandleTypeDef *hlptim)
{
	UART_Printf("TRACSENS_CounterChangedToDownCallback\n\r");
	eCounterDirection= BACKWARD_CounterDirection;

	if(true== pConfig->enableErrorPatternCheck)
	{
		TRACSENS_ChangedToDownErrorHandling();
	}

	//DBG_Print("CounterChangedToDownCallback:%d, multiplier:%d \r\n", eCounterDirection, lCntrMultiplier);
}

void TRACSENS_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
	UART_Printf("TRACSENS_AutoReloadMatchCallback\n\r");
	if(UNKNOWN_CounterDirection== eCounterDirection)/*initially we don't know. we choose backward cos we have a compare int to choose forward*/
	{
		eCounterDirection= BACKWARD_CounterDirection;
	}

	/*check and set cos sometimes direction interrupt will occur simultaneously with ARR interrupt, but served after ARR*/
	if(LL_LPTIM_IsActiveFlag_UP(LPTIM1))
	{
		if(NORMAL_CounterMode== eMode)
		{
			eCounterDirection= FORWARD_CounterDirection;
		}
		else if(INVERT_CounterMode== eMode)
		{
			eCounterDirection= BACKWARD_CounterDirection;
		}
	}
	else if(LL_LPTIM_IsActiveFlag_DOWN(LPTIM1))
	{
		if(NORMAL_CounterMode== eMode)
		{
			eCounterDirection= BACKWARD_CounterDirection;
		}
		else if(INVERT_CounterMode== eMode)
		{
			eCounterDirection= FORWARD_CounterDirection;
		}
	}

	if(FORWARD_CounterDirection== eCounterDirection)
	{
		lCntrMultiplier++;
	}
	else if(BACKWARD_CounterDirection== eCounterDirection)
	{
		lCntrMultiplier--;
	}

	//DBG_Print("AutoReloadMatchCallback:%d, multiplier:%d \r\n", eCounterDirection, lCntrMultiplier);
}
void TRACSENS_StartCounting()
{
	TRACSENS_Power(true);

	if(NORMAL_CounterMode== eMode)
	{
		HAL_LPTIM_RegisterCallback(&hlptim1, HAL_LPTIM_COMPARE_MATCH_CB_ID, TRACSENS_CompareCallback);
		HAL_LPTIM_RegisterCallback(&hlptim1, HAL_LPTIM_AUTORELOAD_MATCH_CB_ID, TRACSENS_AutoReloadMatchCallback);
		HAL_LPTIM_RegisterCallback(&hlptim1, HAL_LPTIM_DIRECTION_UP_CB_ID, TRACSENS_CounterChangedToUpCallback);
		HAL_LPTIM_RegisterCallback(&hlptim1, HAL_LPTIM_DIRECTION_DOWN_CB_ID, TRACSENS_CounterChangedToDownCallback);
	}
	else if(INVERT_CounterMode== eMode)
	{
		HAL_LPTIM_RegisterCallback(&hlptim1, HAL_LPTIM_COMPARE_MATCH_CB_ID, TRACSENS_CompareCallback);
		HAL_LPTIM_RegisterCallback(&hlptim1, HAL_LPTIM_AUTORELOAD_MATCH_CB_ID, TRACSENS_AutoReloadMatchCallback);
		HAL_LPTIM_RegisterCallback(&hlptim1, HAL_LPTIM_DIRECTION_UP_CB_ID, TRACSENS_CounterChangedToDownCallback);
		HAL_LPTIM_RegisterCallback(&hlptim1, HAL_LPTIM_DIRECTION_DOWN_CB_ID, TRACSENS_CounterChangedToUpCallback);
	}

	uwCompareValue=0x01;/*used once to detect direction*/

//	LL_LPTIM_EnableIT_CC1(LPTIM1);		/*Enable the Compare Match interrupt for Channel 1 */
//	LL_LPTIM_EnableIT_ARRM(LPTIM1); 	/*Enable autoreload match interrupt (ARRMIE).*/
//	LL_LPTIM_EnableIT_UP(LPTIM1);		/*Enable direction change to up interrupt (UPIE).*/
//	LL_LPTIM_EnableIT_DOWN(LPTIM1);		/*Enable direction change to down interrupt (DOWNIE).*/

	LL_LPTIM_OC_SetCompareCH1(LPTIM1, 11);
//	LL_LPTIM_SetCompare(LPTIM1, uwCompareValue);/*we need this to know the initial pulse direction(which we don't know after reset)*/

	LL_LPTIM_SetEncoderMode(LPTIM1, LL_LPTIM_ENCODER_MODE_RISING_FALLING);
    LL_LPTIM_EnableEncoderMode(LPTIM1);
    LL_LPTIM_Enable(LPTIM1);
	LL_LPTIM_SetAutoReload(LPTIM1, TRACSENS_CFG_AUTORELOAD_VALUE);
    LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_CONTINUOUS);
    //LPTIM_FeedExternalClock();/*needed when using external clock in counter mode*/

    /*this is needed during power up as we always get extra pulse a bit while after start counting*/
	HAL_Delay(1);	/*when reboot, we get extra pulse*/

	do/* 2 consecutive readings need to be the same*/
    {
    	lCntrErrorReading= LL_LPTIM_GetCounter(LPTIM1);
    }while(LL_LPTIM_GetCounter(LPTIM1)!= lCntrErrorReading);

	//DBG_Print("lCntrErrorReading: %d.\r\n", lCntrErrorReading);
}

void TRACSENS_ChangedToUpErrorHandling(void)
{
	if(BWD_EXPECTING_FWD_CounterErrorState== pConfig->rteErrorPatternState)
	{
		pConfig->rteErrorPatternState= FWD_EXPECTING_BWD_END_CounterErrorState;
	}

}

void TRACSENS_ChangedToDownErrorHandling(void)
{
	int32_t _curr= TRACSENS_GetCounter();
	if(0!= (_curr- pConfig->rteErrorPatternPreviousPulse))
	{
		pConfig->rteErrorPatternState= NONE_CounterErrorState; /*reset if we get real pulse inbetween error pattern*/
		pConfig->rteErrorPatternJustStarted= false;
		if(false== pConfig->rteErrorPatternCompensationStarted)
		{
			pConfig->rteErrorPatternCount= 0;/*we need consecutive error pattern to mark the meter as erroneous that we can handle*/
			//DBG_Print("#stat:mag2: errorPatternCountCleared >\r\n");
		}
	}
	if(NONE_CounterErrorState== pConfig->rteErrorPatternState)
	{
		pConfig->rteErrorPatternJustStarted= true;
		pConfig->rteErrorPatternState= BWD_EXPECTING_FWD_CounterErrorState;
	}
	else if(FWD_EXPECTING_BWD_CounterErrorState== pConfig->rteErrorPatternState)
	{
		pConfig->rteErrorPatternState= BWD_EXPECTING_FWD_END_CounterErrorState;
	}
	else if(FWD_EXPECTING_BWD_END_CounterErrorState== pConfig->rteErrorPatternState)
	{
		pConfig->rteErrorPatternCount++;
		if(true== pConfig->rteErrorPatternJustStarted)
		{
			pConfig->rteErrorPatternJustStarted= false;
			pConfig->rteErrorPatternCount++;
		}

		if((false== pConfig->rteErrorPatternCompensationStarted)&& (pConfig->errorPatternConfirmationCount<= pConfig->rteErrorPatternCount))
		{
			pConfig->rteErrorPatternCompensationStarted= true;
			//DBG_Print("#stat:mag2: errorPatternCompensationStarted > \r\n");
		}
		pConfig->rteErrorPatternState= BWD_EXPECTING_FWD_CounterErrorState;
	}
	pConfig->rteErrorPatternPreviousPulse= _curr;
}

static void TRACSENS_Power(bool _enable)
{
}

void UART_Printf(char *format, ...)
{
  char str[256];
  va_list args;
  va_start(args, format);
  vsnprintf(str, sizeof(str), format, args);
  va_end(args);

  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
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
#ifdef USE_FULL_ASSERT
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

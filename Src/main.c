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
#include "communication.h"
#include "utils.h"
#include "interface.h"
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

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_ADC_Init();
	MX_TIM1_Init();
	MX_TIM16_Init();
	MX_TIM2_Init();
	MX_USART1_UART_Init();
	MX_MotorControl_Init();
	MX_TIM3_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */

	Interface_Init();
	Communication_TimerInit(PWM, &htim3);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		Communication_SetThrottle();
		Interface_ProcessData();
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1) {
	}
	LL_RCC_HSI_Enable();

	/* Wait till HSI is ready */
	while (LL_RCC_HSI_IsReady() != 1) {

	}
	LL_RCC_HSI_SetCalibTrimming(16);
	LL_RCC_HSI14_Enable();

	/* Wait till HSI14 is ready */
	while (LL_RCC_HSI14_IsReady() != 1) {

	}
	LL_RCC_HSI14_SetCalibTrimming(16);
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while (LL_RCC_PLL_IsReady() != 1) {

	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {

	}
	LL_SetSystemCoreClock(48000000);

	/* Update the time base */
	if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
		Error_Handler();
	}
	LL_RCC_HSI14_EnableADCControl();
	LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
	/* USART1_IRQn interrupt configuration */
	NVIC_SetPriority(USART1_IRQn, 3);
	NVIC_EnableIRQ(USART1_IRQn);
	/* DMA1_Channel2_3_IRQn interrupt configuration */
	NVIC_SetPriority(DMA1_Channel2_3_IRQn, 3);
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	/* TIM2_IRQn interrupt configuration */
	NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC_EnableIRQ(TIM2_IRQn);
	/* TIM1_BRK_UP_TRG_COM_IRQn interrupt configuration */
	NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 1);
	NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
	/* ADC1_IRQn interrupt configuration */
	NVIC_SetPriority(ADC1_IRQn, 0);
	NVIC_EnableIRQ(ADC1_IRQn);
	/* EXTI0_1_IRQn interrupt configuration */
	NVIC_SetPriority(EXTI0_1_IRQn, 3);
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	LL_ADC_InitTypeDef ADC_InitStruct = { 0 };
	LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	/**ADC GPIO Configuration
	 PA0   ------> ADC_IN0
	 PA1   ------> ADC_IN1
	 PA2   ------> ADC_IN2
	 PB1   ------> ADC_IN9
	 */
	GPIO_InitStruct.Pin = M1_BEMF_U_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(M1_BEMF_U_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = M1_BEMF_V_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(M1_BEMF_V_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = M1_BEMF_W_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(M1_BEMF_W_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = M1_BUS_VOLTAGE_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(M1_BUS_VOLTAGE_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */

	/** Configure Regular Channel
	 */
	LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_0);

	/** Configure Regular Channel
	 */
	LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_1);

	/** Configure Regular Channel
	 */
	LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_2);

	/** Configure Regular Channel
	 */
	LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_9);

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	ADC_InitStruct.Clock = LL_ADC_CLOCK_ASYNC;
	ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
	ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_LEFT;
	ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
	LL_ADC_Init(ADC1, &ADC_InitStruct);
	ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM1_TRGO;
	ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
	ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
	ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
	ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
	LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
	LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
	LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_7CYCLES_5);
	LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_FALLING);

	/** Configure Analog WatchDog
	 */
	LL_ADC_SetAnalogWDMonitChannels(ADC1, LL_ADC_AWD_CHANNEL_0_REG);
	LL_ADC_ConfigAnalogWDThresholds(ADC1, 0, 0);
	LL_ADC_DisableIT_AWD1(ADC1);
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct = { 0 };
	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = { 0 };
	LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	/**TIM1 GPIO Configuration
	 PA12   ------> TIM1_ETR
	 */
	GPIO_InitStruct.Pin = M1_ETR_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
	LL_GPIO_Init(M1_ETR_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	TIM_InitStruct.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = (PWM_PERIOD_CYCLES );
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	TIM_InitStruct.RepetitionCounter = (REP_COUNTER );
	LL_TIM_Init(TIM1, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM1);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.CompareValue = 0;
	TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
	TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
	TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);
	LL_TIM_SetOCRefClearInputSource(TIM1, LL_TIM_OCREF_CLR_INT_ETR);
	LL_TIM_DisableExternalClock(TIM1);
	LL_TIM_ConfigETR(TIM1, LL_TIM_ETR_POLARITY_NONINVERTED, LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV1);
	LL_TIM_OC_EnableClear(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_EnableClear(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_EnableClear(TIM1, LL_TIM_CHANNEL_CH3);
	LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC4REF);
	LL_TIM_EnableMasterSlaveMode(TIM1);
	TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_ENABLE;
	TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_ENABLE;
	TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
	TIM_BDTRInitStruct.DeadTime = ((DEAD_TIME_COUNTS ) / 2);
	TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
	TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_LOW;
	TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
	LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	/**TIM1 GPIO Configuration
	 PB13   ------> TIM1_CH1N
	 PB14   ------> TIM1_CH2N
	 PB15   ------> TIM1_CH3N
	 PA8   ------> TIM1_CH1
	 PA9   ------> TIM1_CH2
	 PA10   ------> TIM1_CH3
	 */
	GPIO_InitStruct.Pin = M1_PWM_UL_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
	LL_GPIO_Init(M1_PWM_UL_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = M1_PWM_VL_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
	LL_GPIO_Init(M1_PWM_VL_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = M1_PWM_WL_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
	LL_GPIO_Init(M1_PWM_WL_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = M1_PWM_UH_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
	LL_GPIO_Init(M1_PWM_UH_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = M1_PWM_VH_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
	LL_GPIO_Init(M1_PWM_VH_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = M1_PWM_WH_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
	LL_GPIO_Init(M1_PWM_WH_GPIO_Port, &GPIO_InitStruct);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	TIM_InitStruct.Prescaler = LF_TIMER_PSC;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = LF_TIMER_ARR;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM2, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM2);
	LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_ENABLE);
	LL_TIM_EnableMasterSlaveMode(TIM2);
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct = { 0 };
	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = { 0 };
	LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM16);

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	TIM_InitStruct.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = PWM_PERIOD_CYCLES_REF;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	TIM_InitStruct.RepetitionCounter = 0;
	LL_TIM_Init(TIM16, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM16);
	LL_TIM_OC_EnablePreload(TIM16, LL_TIM_CHANNEL_CH1);
	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.CompareValue = 0;
	TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
	TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
	TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
	LL_TIM_OC_Init(TIM16, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM16, LL_TIM_CHANNEL_CH1);
	TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
	TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
	TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
	TIM_BDTRInitStruct.DeadTime = 0;
	TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
	TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
	TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
	LL_TIM_BDTR_Init(TIM16, &TIM_BDTRInitStruct);
	/* USER CODE BEGIN TIM16_Init 2 */

	/* USER CODE END TIM16_Init 2 */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	/**TIM16 GPIO Configuration
	 PA6   ------> TIM16_CH1
	 */
	GPIO_InitStruct.Pin = M1_CURRENT_REF_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(M1_CURRENT_REF_GPIO_Port, &GPIO_InitStruct);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	LL_USART_InitTypeDef USART_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	/**USART1 GPIO Configuration
	 PB6   ------> USART1_TX
	 PB7   ------> USART1_RX
	 */
	GPIO_InitStruct.Pin = UART_TX_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
	LL_GPIO_Init(UART_TX_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = UART_RX_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
	LL_GPIO_Init(UART_RX_GPIO_Port, &GPIO_InitStruct);

	/* USART1 DMA Init */

	/* USART1_RX Init */
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);

	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

	/* USART1_TX Init */
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	USART_InitStruct.BaudRate = 230400;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART1, &USART_InitStruct);
	LL_USART_DisableIT_CTS(USART1);
	LL_USART_ConfigAsyncMode(USART1);
	LL_USART_Enable(USART1);
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* Init with LL driver */
	/* DMA controller clock enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	LL_EXTI_InitTypeDef EXTI_InitStruct = { 0 };
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	/**/
	LL_GPIO_ResetOutputPin(M1_BEMF_DIVIDER_GPIO_Port, M1_BEMF_DIVIDER_Pin);

	/**/
	LL_GPIO_ResetOutputPin(OCTH_STBY1_GPIO_Port, OCTH_STBY1_Pin);

	/**/
	LL_GPIO_SetOutputPin(OC_SEL_GPIO_Port, OC_SEL_Pin);

	/**/
	LL_GPIO_SetOutputPin(OCTH_STBY2_GPIO_Port, OCTH_STBY2_Pin);

	/**/
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTF, LL_SYSCFG_EXTI_LINE0);

	/**/
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTF, LL_SYSCFG_EXTI_LINE1);

	/**/
	LL_GPIO_SetPinPull(Start_Stop_GPIO_Port, Start_Stop_Pin, LL_GPIO_PULL_UP);

	/**/
	LL_GPIO_SetPinPull(PWM_DSHOT_GPIO_Port, PWM_DSHOT_Pin, LL_GPIO_PULL_DOWN);

	/**/
	LL_GPIO_SetPinMode(Start_Stop_GPIO_Port, Start_Stop_Pin, LL_GPIO_MODE_INPUT);

	/**/
	LL_GPIO_SetPinMode(PWM_DSHOT_GPIO_Port, PWM_DSHOT_Pin, LL_GPIO_MODE_INPUT);

	/**/
	EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
	EXTI_InitStruct.LineCommand = ENABLE;
	EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
	LL_EXTI_Init(&EXTI_InitStruct);

	/**/
	EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
	EXTI_InitStruct.LineCommand = ENABLE;
	EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
	LL_EXTI_Init(&EXTI_InitStruct);

	/**/
	GPIO_InitStruct.Pin = M1_BEMF_DIVIDER_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(M1_BEMF_DIVIDER_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = OC_SEL_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(OC_SEL_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = OCTH_STBY2_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(OCTH_STBY2_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = OCTH_STBY1_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(OCTH_STBY1_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	// Enable RXNE interrupt to receive data without DMA
//	LL_USART_EnableIT_RXNE(USART1);
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

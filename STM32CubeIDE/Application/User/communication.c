/*
 * communication.c
 *
 *  Created on: Apr 10, 2025
 *      Author: Michał
 */

#include "communication.h"
#include "dshot.h"
#include "mc_api.h"

#define PWM_BUFFER_SIZE 2
#define DSHOT_BUFFER_SIZE 32
#define DSHOT_US_FACTOR 4

static uint16_t pwmCounterBuffer[PWM_BUFFER_SIZE] = {0};
static uint16_t dshotCounterBuffer[DSHOT_BUFFER_SIZE] = {0};

static CommunicationHandler commHandler = {0};

static volatile uint16_t pwmPulseWidth;
static volatile uint8_t pwmRxFlag = 0;
static volatile uint8_t dshotRxFlag = 0;

void Communication_TimerInit(CommunicationType commType, TIM_HandleTypeDef *htim){
	commHandler.Timer = htim;
	commHandler.Type = commType;
	uint32_t prescaler = 0;
	switch(commType){
	case PWM:
		prescaler = (SystemCoreClock / 1000000) - 1;
		break;
	case DSHOT:
		prescaler = (SystemCoreClock / (DSHOT_US_FACTOR * 1000000)) - 1;
		break;
	default:
		break;
	}
	__HAL_TIM_SET_PRESCALER(htim, prescaler);
	HAL_TIM_Base_Start_IT(htim);
}

void PWM_WriteBuffer(uint16_t counterValue){
	static uint8_t head = 0;
	static uint8_t risingEdgeFlag = 0;

	if (!risingEdgeFlag && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET){
		risingEdgeFlag = 1;
		pwmCounterBuffer[head] = counterValue;
		++head;
	}
	else if (risingEdgeFlag && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET){
		risingEdgeFlag = 0;
		pwmCounterBuffer[head] = counterValue;
		++head;
	}
	if (head >= PWM_BUFFER_SIZE){
		head = 0;
		pwmPulseWidth = pwmCounterBuffer[1] - pwmCounterBuffer[0];
		pwmRxFlag = 1;
	}
}

void PWM_SetThrottle(void){
	static uint16_t lastPulseWidth = 0;
	uint16_t currentPulseWidth = pwmPulseWidth;

	if (lastPulseWidth != pwmPulseWidth){
		lastPulseWidth = currentPulseWidth;
//		UART_Printf("%d\n", pwmPulseWidth);
		if (currentPulseWidth <= 2000 && currentPulseWidth >= 1500){
			int16_t targetSpeedUnit = ((int16_t)(currentPulseWidth-1500) * MAX_APPLICATION_SPEED_UNIT) / 500;
			MC_ProgramSpeedRampMotor1(targetSpeedUnit, 1);
		}
	}
}

void Dshot_WriteBuffer(uint16_t counterValue){
	static uint8_t head = 0;
	static uint8_t risingEdgeFlag = 0;

	if (dshotRxFlag){
		return;
	}

	if (!risingEdgeFlag && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET){
		risingEdgeFlag = 1;
		dshotCounterBuffer[head] = counterValue;
		++head;
	}
	else if (risingEdgeFlag && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET){
		risingEdgeFlag = 0;
		dshotCounterBuffer[head] = counterValue;
		++head;
	}
	if (head >= DSHOT_BUFFER_SIZE){
		head = 0;
		dshotRxFlag = 1;
	}
}

void Dshot_SetThrottle(void){
	uint16_t pulseWidthUs[DSHOT_BUFFER_SIZE/2];
	for (uint8_t i=0;i<DSHOT_BUFFER_SIZE/2;++i){
		pulseWidthUs[i] = (dshotCounterBuffer[2*i+1]-dshotCounterBuffer[2*i])/DSHOT_US_FACTOR;
	}
	DShotPacket dshotPacket = {0};
	DShot_DeserializePulseWidthUs(&dshotPacket, pulseWidthUs, DSHOT_BUFFER_SIZE/2, DSHOT150);
//	UART_Printf("%d\n", dshotPacket.throttle);
	if (!DShot_ValidateCrc(&dshotPacket)){
		return;
	}

	if (dshotPacket.throttle >= 48){
		int16_t targetSpeedUnit = ((int16_t)(dshotPacket.throttle-48) * MAX_APPLICATION_SPEED_UNIT) / 2000;
		MC_ProgramSpeedRampMotor1(targetSpeedUnit, 1);
	}
}

void Communication_SetThrottle(void){
	if (commHandler.Type == PWM){
		if (pwmRxFlag){
			pwmRxFlag = 0;
			PWM_SetThrottle();
		}
	}
	else if (commHandler.Type == DSHOT){
		if (dshotRxFlag){
			dshotRxFlag = 0;
			Dshot_SetThrottle();
		}
	}
}

void Communication_InputCaptureHandler(void){
	if (commHandler.Type == PWM){
		PWM_WriteBuffer((uint16_t)__HAL_TIM_GET_COUNTER(commHandler.Timer));
	}
	else if (commHandler.Type == DSHOT){
		Dshot_WriteBuffer((uint16_t)__HAL_TIM_GET_COUNTER(commHandler.Timer));
	}
}

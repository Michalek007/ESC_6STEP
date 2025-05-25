/*
 * communication.h
 *
 *  Created on: Apr 10, 2025
 *      Author: Michał
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_

#include "main.h"

typedef enum {
	PWM = 0,
	DSHOT = 1
} CommunicationType;

typedef struct {
	CommunicationType Type;
	TIM_HandleTypeDef *Timer;
} CommunicationHandler;

void Communication_TimerInit(CommunicationType commType, TIM_HandleTypeDef *htim);
void Communication_SetThrottle(void);
void Communication_InputCaptureHandler(void);

#endif /* INC_COMMUNICATION_H_ */

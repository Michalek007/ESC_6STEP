/*
 * communication.h
 *
 *  Created on: Apr 10, 2025
 *      Author: Michał
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_

#include "main.h"

#define MIN_APPLICATION_SPEED_RPM 4000
#define MIN_APPLICATION_SPEED_UNIT ((MIN_APPLICATION_SPEED_RPM * SPEED_UNIT) / U_RPM)

typedef enum {
	PWM = 0, DSHOT = 1, UART = 2
} CommunicationType;

typedef struct {
	CommunicationType Type;
	TIM_HandleTypeDef *Timer;
} CommunicationHandler;

void Communication_TimerInit(CommunicationType commType, TIM_HandleTypeDef *htim);
void Communication_ChangeType(CommunicationType commType);
void Communication_SetThrottle(void);
void Communication_InputCaptureHandler(void);

#endif /* INC_COMMUNICATION_H_ */

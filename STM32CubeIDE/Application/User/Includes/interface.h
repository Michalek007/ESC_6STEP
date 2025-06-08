/*
 * interface.h
 *
 *  Created on: Jun 3, 2025
 *      Author: Michał
 */

#ifndef APPLICATION_USER_INCLUDES_INTERFACE_H_
#define APPLICATION_USER_INCLUDES_INTERFACE_H_

#include "main.h"

void Interface_Init(void);
void Interface_UartRxHandler(void);
void Interface_ProcessData(void);
void Interface_SendTelemetry(void);

#endif /* APPLICATION_USER_INCLUDES_INTERFACE_H_ */

/*
 * utils.h
 *
 *  Created on: Jun 3, 2025
 *      Author: Michał
 */

#ifndef APPLICATION_USER_INCLUDES_UTILS_H_
#define APPLICATION_USER_INCLUDES_UTILS_H_

#include "main.h"

#define MAX_PRINTF_LEN 100

void UART_Print(const char *string);
void UART_Printf(const char *string, ...);

#endif /* APPLICATION_USER_INCLUDES_UTILS_H_ */

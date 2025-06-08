/*
 * utils.c
 *
 *  Created on: Jun 3, 2025
 *      Author: Michał
 */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "utils.h"

inline void UART_Print(const char *string) {
	size_t strLen = strlen(string);
	for (size_t i = 0; i < strLen; i++) {
		while (!LL_USART_IsActiveFlag_TXE(USART1))
			;
		LL_USART_TransmitData8(USART1, string[i]);
	}
}

void UART_Printf(const char *string, ...) {
	va_list argp;
	va_start(argp, string);
	char stringf[MAX_PRINTF_LEN];
	if (vsprintf(stringf, string, argp) > 0) {
		UART_Print(stringf);
	}
	va_end(argp);
}

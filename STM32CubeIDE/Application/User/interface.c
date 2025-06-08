/*
 * interface.c
 *
 *  Created on: Jun 8, 2025
 *      Author: Michał
 */
#include "interface.h"
#include "mc_api.h"
#include "mc_packet.h"
#include "communication.h"

#define UART_RX_BUFFER_SIZE 4

static uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE];
static volatile uint8_t uartRxFlag = 0;

static MCPacket mcPacket = { 0 };
static MCTelemetryPacket mcTelemetryPacket = { 0 };

//void Interface_Echo(void) {
//	for (uint16_t i = 0; i < UART_RX_BUFFER_SIZE; i++) {
//		while (!LL_USART_IsActiveFlag_TXE(USART1))
//			; // Wait for TX buffer
//		LL_USART_TransmitData8(USART1, uartRxBuffer[i]);
//	}
//}

void Interface_Init(void) {
	LL_USART_EnableIT_RXNE(USART1);
}

void Interface_UartRxHandler(void) {
	static uint8_t head = 0;
	if (LL_USART_IsActiveFlag_RXNE(USART1) && !uartRxFlag) {
		uint8_t receivedData = LL_USART_ReceiveData8(USART1);
		uartRxBuffer[head] = receivedData;
		head++;
		if (head >= UART_RX_BUFFER_SIZE) {
			head = 0;
			uartRxFlag = 1;
		}
	}
}

void Interface_ProcessData(void) {
	if (uartRxFlag) {
		uartRxFlag = 0;
		McPacket_Deserialize(&mcPacket, uartRxBuffer, UART_RX_BUFFER_SIZE);
		if (!McPacket_ValidateCrc(&mcPacket)) {
			return;
		}
		switch (mcPacket.Type) {
		case SET_COMMUNICATION_TYPE:
//			UART_Print("comm");
			uint8_t commType = mcPacket.Data[0];
			switch (commType) {
			case 0:
				Communication_ChangeType(PWM);
				break;
			case 1:
				Communication_ChangeType(DSHOT);
				break;
			default:
				Communication_ChangeType(UART);
				break;
			}
			break;
		case SET_SPEED_RPM:
//			UART_Print("speed");
			uint16_t speedRPM = (mcPacket.Data[0] & 0xFF) | ((uint16_t) mcPacket.Data[1] << 8);
			if (speedRPM >= MIN_APPLICATION_SPEED_RPM && speedRPM <= MAX_APPLICATION_SPEED_RPM) {

				int16_t targetSpeedUnit = ((int16_t) speedRPM * SPEED_UNIT) / U_RPM;
				MC_ProgramSpeedRampMotor1(targetSpeedUnit, 1);
				if (IDLE == MC_GetSTMStateMotor1()) {
					MC_StartMotor1();
				}
			} else if (speedRPM == 0) {
				MC_StopMotor1();
			}
			break;
		case TELEMETRY_REQUEST:
//			UART_Print("tell");
			mcTelemetryPacket.DutyCycle = (uint16_t) MCI_GetDutyCycleRefMotor1();
			mcTelemetryPacket.ReferenceSpeed = (uint16_t) MC_GetMecSpeedReferenceMotor1_F();
			mcTelemetryPacket.AverageSpeed = (uint16_t) MC_GetAverageMecSpeedMotor1_F();
			mcTelemetryPacket.MotorState = (uint8_t) MC_GetSTMStateMotor1();
			McTelemetryPacket_SetCrc(&mcTelemetryPacket);

			uint8_t txBuffer[8];
			McTelemetryPacket_Serialize(&mcTelemetryPacket, txBuffer, 8);
			for (size_t i = 0; i < 8; i++) {
				while (!LL_USART_IsActiveFlag_TXE(USART1))
					;
				LL_USART_TransmitData8(USART1, txBuffer[i]);
			}
			break;
		default:
			break;
		}
	}
}


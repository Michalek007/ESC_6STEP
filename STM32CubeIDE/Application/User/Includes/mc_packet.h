/*
 * mc_packet.h
 *
 *  Created on: Jun 8, 2025
 *      Author: Michał
 */

#ifndef APPLICATION_USER_INCLUDES_MC_PACKET_H_
#define APPLICATION_USER_INCLUDES_MC_PACKET_H_

#include "main.h"

typedef struct {
	uint16_t DutyCycle;
	uint16_t ReferenceSpeed;
	uint16_t AverageSpeed;
	uint8_t MotorState;
	uint8_t Crc;
} MCTelemetryPacket;

typedef enum {
	SET_COMMUNICATION_TYPE, SET_SPEED_RPM, TELEMETRY_REQUEST
} MCPacketType;

typedef struct {
	MCPacketType Type;
	uint8_t Data[2];
	uint8_t Crc;
} MCPacket;

#endif /* APPLICATION_USER_INCLUDES_MC_PACKET_H_ */

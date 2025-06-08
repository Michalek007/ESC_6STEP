/*
 * mc_packet.h
 *
 *  Created on: Jun 3, 2025
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

void McTelemetryPacket_Serialize(MCTelemetryPacket *mcPacket, uint8_t *buffer, uint8_t size);
void McTelemetryPacket_Deserialize(MCTelemetryPacket *mcPacket, uint8_t *data, uint8_t size);
void McTelemetryPacket_SetCrc(MCTelemetryPacket *mcPacket);
uint8_t McTelemetryPacket_ValidateCrc(MCTelemetryPacket *mcPacket);

void McPacket_Serialize(MCPacket *mcPacket, uint8_t *buffer, uint8_t size);
void McPacket_Deserialize(MCPacket *mcPacket, uint8_t *data, uint8_t size);
void McPacket_SetCrc(MCPacket *mcPacket);
uint8_t McPacket_ValidateCrc(MCPacket *mcPacket);

#endif /* APPLICATION_USER_INCLUDES_MC_PACKET_H_ */

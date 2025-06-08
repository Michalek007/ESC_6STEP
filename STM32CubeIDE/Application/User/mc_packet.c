/*
 * mc_packet.c
 *
 *  Created on: Jun 8, 2025
 *      Author: Michał
 */
#include "mc_packet.h"
#include "assert.h"
#include "string.h"

void McTelemetryPacket_Serialize(MCTelemetryPacket *mcPacket, uint8_t *buffer, uint8_t size) {
//	assert(size == sizeof(*mcPacket));
	memcpy(buffer, &mcPacket->DutyCycle, sizeof(mcPacket->DutyCycle));
	memcpy(buffer + 2, &mcPacket->ReferenceSpeed, sizeof(mcPacket->DutyCycle));
	memcpy(buffer + 4, &mcPacket->AverageSpeed, sizeof(mcPacket->DutyCycle));
	buffer[6] = mcPacket->MotorState;
	buffer[7] = mcPacket->Crc;
}

void McTelemetryPacket_SetCrc(MCPacket *mcPacket) {
	mcPacket->Crc = 0;
}

void McPacket_Deserialize(MCPacket *mcPacket, uint8_t *data, uint8_t size) {
//	assert(size == sizeof(*mcPacket));
	mcPacket->Type = data[0];
	mcPacket->Data[0] = data[1];
	mcPacket->Data[1] = data[2];
	mcPacket->Crc = data[3];
}

uint8_t McPacket_ValidateCrc(MCPacket *mcPacket) {
	return 1;
}

static uint8_t McPacket_CRC8ATM(uint8_t *data, size_t size) {
	uint8_t crc = 0x00;
	for (size_t i = 0; i < size; ++i) {
		crc ^= data[i];
		for (uint8_t b = 0; b < 8; ++b) {
			if (crc & 0x80)
				crc = (crc << 1) ^ 0x07;
			else
				crc <<= 1;
		}
	}
	return crc;
}

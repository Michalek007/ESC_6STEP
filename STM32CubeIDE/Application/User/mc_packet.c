/*
 * mc_packet.c
 *
 *  Created on: Jun 3, 2025
 *      Author: Michał
 */
#include "mc_packet.h"
#include "assert.h"
#include "string.h"

static uint8_t McPacket_CRC8ATM(uint8_t *data, uint8_t size);

void McTelemetryPacket_Serialize(MCTelemetryPacket *mcPacket, uint8_t *buffer, uint8_t size) {
//	assert(size == sizeof(*mcPacket));
	memcpy(buffer, &mcPacket->DutyCycle, sizeof(mcPacket->DutyCycle));
	memcpy(buffer + 2, &mcPacket->ReferenceSpeed, sizeof(mcPacket->DutyCycle));
	memcpy(buffer + 4, &mcPacket->AverageSpeed, sizeof(mcPacket->DutyCycle));
	buffer[6] = mcPacket->MotorState;
	buffer[7] = mcPacket->Crc;
}

void McTelemetryPacket_Deserialize(MCTelemetryPacket *mcPacket, uint8_t *data, uint8_t size) {
//	assert(size == sizeof(*mcPacket));
	mcPacket->DutyCycle = (data[0] & 0xFF) | ((uint16_t) data[1] << 8);
	mcPacket->ReferenceSpeed = (data[2] & 0xFF) | ((uint16_t) data[3] << 8);
	mcPacket->AverageSpeed = (data[4] & 0xFF) | ((uint16_t) data[5] << 8);
	mcPacket->MotorState = data[6];
	mcPacket->Crc = data[7];
}

void McTelemetryPacket_SetCrc(MCTelemetryPacket *mcPacket) {
	uint8_t data[8];
	McTelemetryPacket_Serialize(mcPacket, data, 8);
	mcPacket->Crc = McPacket_CRC8ATM(data, 7);
}

uint8_t McTelemetryPacket_ValidateCrc(MCTelemetryPacket *mcPacket) {
	uint8_t data[8];
	McTelemetryPacket_Serialize(mcPacket, data, 8);
	if (mcPacket->Crc == McPacket_CRC8ATM(data, 7)) {
		return 1;
	}
	return 0;
}

void McPacket_Serialize(MCPacket *mcPacket, uint8_t *buffer, uint8_t size) {
//	assert(size == sizeof(*mcPacket));
	buffer[0] = mcPacket->Type;
	buffer[1] = mcPacket->Data[0];
	buffer[2] = mcPacket->Data[1];
	buffer[3] = mcPacket->Crc;
}

void McPacket_Deserialize(MCPacket *mcPacket, uint8_t *data, uint8_t size) {
//	assert(size == sizeof(*mcPacket));
	mcPacket->Type = data[0];
	mcPacket->Data[0] = data[1];
	mcPacket->Data[1] = data[2];
	mcPacket->Crc = data[3];
}

void McPacket_SetCrc(MCPacket *mcPacket) {
	uint8_t data[4];
	McPacket_Serialize(mcPacket, data, 4);
	mcPacket->Crc = McPacket_CRC8ATM(data, 3);
}

uint8_t McPacket_ValidateCrc(MCPacket *mcPacket) {
	uint8_t data[4];
	McPacket_Serialize(mcPacket, data, 4);
	if (mcPacket->Crc == McPacket_CRC8ATM(data, 3)) {
		return 1;
	}
	return 0;
}

static uint8_t McPacket_CRC8ATM(uint8_t *data, uint8_t size) {
	uint8_t crc = 0;
	while (size--) {
		uint8_t inbyte = *data++;
		for (uint8_t i = 8; i--;) {
			uint8_t mix = (crc ^ inbyte) & 0x80;
			crc <<= 1;
			if (mix) {
				crc ^= 0x07;
			}
			inbyte <<= 1;
		}
	}
	return crc;
}

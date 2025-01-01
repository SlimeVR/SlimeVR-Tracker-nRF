/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RETAINED_H_
#define RETAINED_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Example of validatable retained data. */
struct retained_data {
	/* The uptime from the current session the last time the
	 * retained data was updated.
	 */
	uint64_t uptime_latest;

	/* Cumulative uptime from all previous sessions up through
	 * uptime_latest of this session.
	 */
	uint64_t uptime_sum;

	/* Number of times the application has started. */
	uint32_t boots;

	/* Number of times the application has gone into system off. */
	uint32_t off_count;

	uint8_t reboot_counter;
	uint8_t paired_addr[8];

	float accelBias[3];
	float gyroBias[3];
	float magBias[3];
	float magBAinv[4][3];
	float accBAinv[4][3];

	uint8_t fusion_id; // fusion_data_stored
	uint8_t fusion_data[512];

	uint16_t imu_addr;
	uint16_t mag_addr;

	uint8_t imu_reg;
	uint8_t mag_reg;

	/* CRC used to validate the retained data.  This must be
	 * stored little-endian, and covers everything up to but not
	 * including this field.
	 */
	uint32_t crc;
};

/* For simplicity in the sample just allow anybody to see and
 * manipulate the retained state.
 */
extern struct retained_data retained;

int ram_range_retain(const void *ptr, size_t len, bool enable);

/* Check whether the retained data is valid, and if not reset it.
 *
 * @return true if and only if the data was valid and reflects state
 * from previous sessions.
 */
bool retained_validate(void);

/* Update any generic retained state and recalculate its checksum so
 * subsequent boots can verify the retained state.
 */
void retained_update(void);

#endif /* RETAINED_H_ */
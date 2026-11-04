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

struct retained_data {
	/* The build version of the firmware that last updated the
	 * retained data.
	 */
	uint32_t build_timestamp;

	/* The uptime from the current session the last time the
	 * retained data was updated.
	 */
	uint64_t uptime_latest;

	/* Cumulative uptime from all previous sessions up through
	 * uptime_latest of this session.
	 */
	uint64_t uptime_sum;

	/* Battery statistics.  Tracking for discharge curve only begins
	 * after ~3% discharged.  If battery_pptt has changed significantly
	 * compared to min_battery_pptt since the last update,
	 * the statistics are cleared and may be saved to NVS.
	 */
	int16_t max_battery_pptt;
	int16_t min_battery_pptt;

	/* Battery uptime from last retained update */
	uint64_t battery_uptime_latest;

	/* Cumulative runtime */
	uint64_t battery_runtime_sum;

	/* Last interval stored in NVS */
	int16_t battery_pptt_saved;
	uint64_t battery_runtime_saved;

	/* Calibrated discharge curve */
	int16_t battery_pptt_curve[18];

	uint8_t reboot_counter;
	uint8_t paired_addr[8];
	uint8_t last_dongle_channel;
	uint8_t tdma_window;

	uint8_t sensor_data[128];

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

	uint8_t settings[128];

	/* CRC used to validate the retained data.  This must be
	 * stored little-endian, and covers everything up to but not
	 * including this field.
	 */
	uint32_t crc;
};

/* Up to 1 KB of retained data allowed right now.
 */
#define RETAINED_SIZE (sizeof(struct retained_data))

/* For simplicity in the sample just allow anybody to see and
 * manipulate the retained state.
 */
extern struct retained_data *retained;

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
/*
Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.

BSD-3-Clause

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include <math.h>

#include <zephyr/logging/log.h>

#include "BMM150.h"
#include "sensor/sensor_none.h"

// Annoyingly, chip id cannot be read unless the BMM150 power control is enabled
// In sensor_scan power control is enabled first to read chip id
// taken from boschsensortec BMM150_SensorAPI
static int8_t dig_x1;
static int8_t dig_y1;
static int8_t dig_x2;
static int8_t dig_y2;
static uint16_t dig_z1;
static int16_t dig_z2;
static int16_t dig_z3;
static int16_t dig_z4;
static uint8_t dig_xy1;
static int8_t dig_xy2;
static uint16_t dig_xyz1;

static uint8_t last_odr = 0xff;
static float last_real_time = 0.0f;

LOG_MODULE_REGISTER(BMM150, LOG_LEVEL_DBG);

static int read_trim_registers(void);
static float compensate_x(int16_t mag_data_x, uint16_t data_rhall);
static float compensate_y(int16_t mag_data_y, uint16_t data_rhall);
static float compensate_z(int16_t mag_data_z, uint16_t data_rhall);

int bmm1_init(float time, float *actual_time)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM150_POWER_CTRL, 0x01);
	k_msleep(2); // BMM150 start-up
	err |= read_trim_registers();
	if (err)
		LOG_ERR("Communication error");
	last_odr = 0xff; // reset last odr
	err |= bmm1_update_odr(time, actual_time);
	return (err < 0 ? err : 0);
}

void bmm1_shutdown(void)
{
	last_odr = 0xff; // reset last odr
//	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM150_POWER_CTRL, 0x82); // soft reset
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM150_POWER_CTRL, 0x00);
	if (err)
		LOG_ERR("Communication error");
}

float bmm1_get_odr(void)
{
	return last_real_time;
}

int bmm1_update_odr(float time, float *actual_time)
{
	int ODR;
	uint8_t DR;
	uint8_t OPMODE;
	uint8_t REP_XY; // 1+2(REP_XY)
	uint8_t REP_Z; // 1+REP_Z

	if (time <= 0 || time == INFINITY) // sleep and forced mode both use sleep mode
	{
		OPMODE = OPMODE_SLEEP;
		ODR = 0;
	}
	else
	{
		OPMODE = OPMODE_NORMAL;
		// Regular preset, 0.6uT RMS Noise or 6mG
		REP_XY = 4; // 9
		REP_Z = 14; // 15
		ODR = 1 / time;
	}

	if (time <= 0)
	{
		DR = 0;
		REP_XY = 0;
		REP_Z = 0;
		time = 0; // off
	}
	else if (ODR > 25) // TODO: this sucks
	{
		DR = DR_ODR_30Hz;
		time = 1.0 / 30;
	}
	else if (ODR > 20)
	{
		DR = DR_ODR_25Hz;
		time = 1.0 / 25;
	}
	else if (ODR > 15)
	{
		DR = DR_ODR_20Hz;
		time = 1.0 / 20;
	}
	else if (ODR > 10)
	{
		DR = DR_ODR_15Hz;
		time = 1.0 / 15;
	}
	else if (ODR > 8)
	{
		DR = DR_ODR_10Hz;
		time = 1.0 / 10;
	}
	else if (ODR > 6)
	{
		DR = DR_ODR_8Hz;
		time = 1.0 / 8;
	}
	else if (ODR > 2)
	{
		DR = DR_ODR_6Hz;
		time = 1.0 / 6;
	}
	else if (ODR > 0)
	{
		DR = DR_ODR_2Hz;
		time = 1.0 / 2;
	}
	else
	{
		DR = DR_ODR_30Hz;
		// Low power preset, 1.0uT RMS Noise or 10mG
		REP_XY = 1; // 3
		REP_Z = 4; // 5
		time = INFINITY;
	}

	uint8_t OP_SET = DR << 2 | OPMODE;
	if (last_odr == OP_SET)
		return 1;
	else
		last_odr = OP_SET;

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM150_OP_CTRL, OP_SET << 1);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM150_REP_XY, REP_XY);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM150_REP_Z, REP_Z);
	if (err)
		LOG_ERR("Communication error");

	*actual_time = time;
	last_real_time = time;
	return err;
}

void bmm1_mag_oneshot(void)
{
	int err = ssi_reg_update_byte(SENSOR_INTERFACE_DEV_MAG, BMM150_OP_CTRL, 0x06, OPMODE_FORCED << 1);
	if (err)
		LOG_ERR("Communication error");
}

void bmm1_mag_read(float m[3])
{
	int err = 0;
	uint8_t status;
	while ((status & 0x06) == 0x02) // wait for forced mode to complete
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, BMM150_OP_CTRL, &status);
	uint8_t rawData[8];
	err |= ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, BMM150_DATAX_LSB, &rawData[0], 8);
	if (err)
		LOG_ERR("Communication error");
	bmm1_mag_process(rawData, m);
}

void bmm1_mag_process(uint8_t *raw_m, float m[3])
{
	int16_t mag[3] = {0};
	for (int i = 0; i < 3; i++) // x, y, z
		mag[i] = (int16_t)((((uint16_t)raw_m[(i * 2) + 1]) << 8) | (raw_m[i * 2] & 0xFE)) / (i < 2 ? 8 : 2);
	uint16_t rhall = (uint16_t)((((uint16_t)raw_m[7]) << 6) | (raw_m[6] >> 2));
	m[0] = compensate_x(mag[0], rhall) / 100; // uT to gauss
	m[1] = compensate_y(mag[1], rhall) / 100;
	m[2] = compensate_z(mag[2], rhall) / 100;
}

// from boschsensortec BMM150_SensorAPI
static int read_trim_registers(void)
{
	uint8_t trim_x1y1[2] = {0};
	uint8_t trim_xyz_data[4] = {0};
	uint8_t trim_xy1xy2[10] = {0};

	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, BMM150_DIG_X1, trim_x1y1, 2);
	err |= ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, BMM150_DIG_Z4_LSB, trim_xyz_data, 4);
	err |= ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, BMM150_DIG_Z2_LSB, trim_xy1xy2, 10);
	if (err)
		LOG_ERR("Communication error");

	dig_x1 = (int8_t)trim_x1y1[0];
	dig_y1 = (int8_t)trim_x1y1[1];
	dig_x2 = (int8_t)trim_xyz_data[2];
	dig_y2 = (int8_t)trim_xyz_data[3];
	dig_z1 = (uint16_t)((((uint16_t)trim_xy1xy2[3]) << 8) | trim_xy1xy2[2]);
	dig_z2 = (int16_t)((((uint16_t)trim_xy1xy2[1]) << 8) | trim_xy1xy2[0]);
	dig_z3 = (int16_t)((((uint16_t)trim_xy1xy2[7]) << 8) | trim_xy1xy2[6]);
	dig_z4 = (int16_t)((((uint16_t)trim_xyz_data[1]) << 8) | trim_xyz_data[0]);
	dig_xy1 = trim_xy1xy2[9];
	dig_xy2 = (int8_t)trim_xy1xy2[8];
	dig_xyz1 = (uint16_t)((((uint16_t)(trim_xy1xy2[5] & 0x7F)) << 8) | trim_xy1xy2[4]);

	return err;
}

// from boschsensortec BMM150_SensorAPI
// returns value in microtesla
static float compensate_x(int16_t mag_data_x, uint16_t data_rhall)
{
    float retval = 0;

    /* Overflow condition check */
    if ((mag_data_x != -4096) && (data_rhall != 0) && (dig_xyz1 != 0))
    {
        /* Processing compensation equations */
        float x0 = (((float)dig_xyz1) * 16384.0f / data_rhall);
        retval = (x0 - 16384.0f);
        float x1 = ((float)dig_xy2) * (retval * retval / 268435456.0f);
        float x2 = x1 + retval * ((float)dig_xy1) / 16384.0f;
        float x3 = ((float)dig_x2) + 160.0f;
        float x4 = mag_data_x * ((x2 + 256.0f) * x3);
        retval = ((x4 / 8192.0f) + (((float)dig_x1) * 8.0f)) / 16.0f;
    }

    return retval;
}

static float compensate_y(int16_t mag_data_y, uint16_t data_rhall)
{
    float retval = 0;

    /* Overflow condition check */
    if ((mag_data_y != -4096) && (data_rhall != 0) && (dig_xyz1 != 0))
    {
        /* Processing compensation equations */
        float y0 = ((float)dig_xyz1) * 16384.0f / data_rhall;
        retval = y0 - 16384.0f;
        float y1 = ((float)dig_xy2) * (retval * retval / 268435456.0f);
        float y2 = y1 + retval * ((float)dig_xy1) / 16384.0f;
        float y3 = ((float)dig_y2) + 160.0f;
        float y4 = mag_data_y * (((y2) + 256.0f) * y3);
        retval = ((y4 / 8192.0f) + (((float)dig_y1) * 8.0f)) / 16.0f;
    }

    return retval;
}

static float compensate_z(int16_t mag_data_z, uint16_t data_rhall)
{
    float retval = 0;

    /* Overflow condition check */
    if ((mag_data_z != -16384) && (dig_z2 != 0) && (dig_z1 != 0) && (dig_xyz1 != 0) && (data_rhall != 0))
    {
        /* Processing compensation equations */
        float z0 = ((float)mag_data_z) - ((float)dig_z4);
        float z1 = ((float)data_rhall) - ((float)dig_xyz1);
        float z2 = (((float)dig_z3) * z1);
        float z3 = ((float)dig_z1) * ((float)data_rhall) / 32768.0f;
        float z4 = ((float)dig_z2) + z3;
        float z5 = (z0 * 131072.0f) - z2;
        retval = (z5 / ((z4) * 4.0f)) / 16.0f;
    }

    return retval;
}

const sensor_mag_t sensor_mag_bmm150 = {
	*bmm1_init,
	*bmm1_shutdown,

	*bmm1_update_odr,
	*bmm1_get_odr,

	*bmm1_mag_oneshot,
	*bmm1_mag_read,
	*mag_none_temp_read,

	*bmm1_mag_process,
	6, 8, // rhall does not get read by limited external interface
	0, BMM150_DATAX_LSB
};

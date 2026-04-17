/*
Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.

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

#include "BMM350.h"

// the datasheet says almost nothing about sensitivities, these are taken from boschsensortec BMM350_SensorAPI
// Data registers for X,Y and Z magnetic channel and temperature channel provide data in 24bit registers in 21bit signed-integer format (21LSBs used)
// currently the data read is uncompensated, there are compensation values in OTP which is not used at the moment
#define bxy_sens 14.55f
#define bz_sens 9.0f
#define temp_sens 0.00204f
#define ina_xy_gain_trgt 19.46f
#define ina_z_gain_trgt 31.0f
#define adc_gain (1 / 1.5f)
#define lut_gain 0.714607238769531f
#define power (float)(1000000.0 / 1048576.0)

static const float sensitivity_xy = (power / (bxy_sens * ina_xy_gain_trgt * adc_gain * lut_gain)); // uT/LSB
static const float sensitivity_z = (power / (bz_sens * ina_z_gain_trgt * adc_gain * lut_gain));
static const float sensitivity_temp = 1 / (temp_sens * adc_gain * lut_gain * 1048576); // C/LSB

static uint8_t last_odr = 0xff;

LOG_MODULE_REGISTER(BMM350, LOG_LEVEL_DBG);

int bmm3_init(float time, float *actual_time)
{
	last_odr = 0xff; // reset last odr
	int err = bmm3_update_odr(time, actual_time);
	return (err < 0 ? err : 0);
}

void bmm3_shutdown(void)
{
	last_odr = 0xff; // reset last odr
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM350_OTP_CMD_REG, 0x80); // PWR_OFF_OTP
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM350_PMU_CMD, PMU_CMD_SUS); // enter suspend (amogus,)
	if (err)
		LOG_ERR("Communication error");
}

int bmm3_update_odr(float time, float *actual_time)
{
	int ODR;
	uint8_t AGGR;
	uint8_t AGGR_AVG;
	uint8_t PMU_CMD;

	if (time <= 0 || time == INFINITY) // suspend and forced mode both use suspend mode
	{
		PMU_CMD = PMU_CMD_SUS; // amogus
		ODR = 0;
	}
	else
	{
		PMU_CMD = PMU_CMD_NM;
		ODR = 1 / time;
	}

	if (time <= 0)
	{
		AGGR = 0;
		AGGR_AVG = AGGR_NO_AVG;
		time = 0; // off
	}
	else if (ODR > 200) // TODO: this sucks
	{
		AGGR = AGGR_ODR_400Hz;
		AGGR_AVG = AGGR_NO_AVG;
		time = 1.0 / 400;
	}
	else if (ODR > 100)
	{
		AGGR = AGGR_ODR_200Hz;
		AGGR_AVG = AGGR_AVG_2;
		time = 1.0 / 200;
	}
	else if (ODR > 50)
	{
		AGGR = AGGR_ODR_100Hz;
		AGGR_AVG = AGGR_AVG_4;
		time = 1.0 / 100;
	}
	else if (ODR > 25)
	{
		AGGR = AGGR_ODR_50Hz;
		AGGR_AVG = AGGR_AVG_8;
		time = 1.0 / 50;
	}
	else if (ODR > 12.5)
	{
		AGGR = AGGR_ODR_25Hz;
		AGGR_AVG = AGGR_AVG_8;
		time = 1.0 / 25;
	}
	else if (ODR > 6.25)
	{
		AGGR = AGGR_ODR_12_5Hz;
		AGGR_AVG = AGGR_AVG_8;
		time = 1.0 / 12.5;
	}
	else if (ODR > 3.125)
	{
		AGGR = AGGR_ODR_6_25Hz;
		AGGR_AVG = AGGR_AVG_8;
		time = 1.0 / 6.25;
	}
	else if (ODR > 1.5625)
	{
		AGGR = AGGR_ODR_3_125Hz;
		AGGR_AVG = AGGR_AVG_8;
		time = 1.0 / 3.125;
	}
	else if (ODR > 0)
	{
		AGGR = AGGR_ODR_1_5625Hz;
		AGGR_AVG = AGGR_AVG_8;
		time = 1.0 / 1.5625;
	}
	else
	{
		AGGR = AGGR_ODR_200Hz;
		AGGR_AVG = AGGR_NO_AVG;
		time = INFINITY;
	}

	uint8_t AGGR_SET = AGGR_AVG << 4 | AGGR;
	if (last_odr == AGGR_SET)
		return 1;
	else
		last_odr = AGGR_SET;

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM350_PMU_CMD_AGGR_SET, AGGR_SET);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM350_PMU_CMD, PMU_CMD);
	if (err)
		LOG_ERR("Communication error");

	*actual_time = time;
	return err;
}

void bmm3_mag_oneshot(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, BMM350_PMU_CMD, PMU_CMD_FM_FAST);
	if (err)
		LOG_ERR("Communication error");
}

void bmm3_mag_read(float m[3])
{
	int err = 0;
	uint8_t status;
	while ((status & 0x01) == 0x01) // wait for forced mode to complete
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, BMM350_PMU_CMD_STATUS_0, &status);
	uint8_t rawData[11];
	err |= ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, BMM350_MAG_X_XLSB, &rawData[0], 11); // two dummy bytes on read
	if (err)
		LOG_ERR("Communication error");
	bmm3_mag_process(rawData + 2, m);
}

float bmm3_temp_read(float bias[3])
{
	uint8_t rawTemp[5];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, BMM350_TEMP_XLSB, &rawTemp[0], 5); // two dummy bytes on read
	if (err)
		LOG_ERR("Communication error");
	float temp = (int32_t)((((int32_t)rawTemp[4]) << 24) | (((int32_t)rawTemp[3]) << 16) | (((int32_t)rawTemp[2]) << 8)) / 256;
	temp *= sensitivity_temp;
	// taken from boschsensortec BMM350_SensorAPI, why is this needed?
	if (temp > 0)
		temp -= 25.49f;
	else if (temp < 0)
		temp += 25.49f;
	return temp;
}

void bmm3_mag_process(uint8_t *raw_m, float m[3])
{
	for (int i = 0; i < 3; i++) // x, y, z
	{
		m[i] = (int32_t)((((int32_t)raw_m[(i * 3) + 2]) << 24) | (((int32_t)raw_m[(i * 3) + 1]) << 16) | (((int32_t)raw_m[i * 3]) << 8)) / 256;
		m[i] *= i < 2 ? sensitivity_xy : sensitivity_z;
		m[i] /= 100; // uT to gauss
	}
}

// TODO: from BMM350_SensorAPI, add otp_dump_after_boot and update_mag_off_sens

const sensor_mag_t sensor_mag_bmm350 = {
	*bmm3_init,
	*bmm3_shutdown,

	*bmm3_update_odr,

	*bmm3_mag_oneshot,
	*bmm3_mag_read,
	*bmm3_temp_read,

	*bmm3_mag_process,
	9, 9
};

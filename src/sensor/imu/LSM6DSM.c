#include <math.h>

#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#include "LSM6DSM.h"
#include "LSM6DSV.h" // Common functions
#include "sensor/sensor_none.h"

#define PACKET_SIZE 7 // first byte is pattern, only 6 actual sample bytes

static uint8_t accel_fs = DSM_FS_XL_16G;
static uint8_t gyro_fs = DSM_FS_G_2000DPS;

static uint8_t fifo_pattern_length;
static bool fifo_pattern_gyro_dominant; // if more samples are gyro than accel in a pattern

LOG_MODULE_REGISTER(LSM6DSM, LOG_LEVEL_DBG);

int lsm6dsm_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	// setup interface for SPI
	sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(10), 0);
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_CTRL3, 0x74); // freeze register until done reading, increment register address during multi-byte access (BDU, IF_INC), INT H_LACTIVE active low, PP_OD open-drain
	if (err)
		LOG_ERR("Communication error");
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	err |= lsm6dsm_update_odr(accel_time, gyro_time, accel_actual_time, gyro_actual_time);
	if (err)
		LOG_ERR("Communication error");
	return (err < 0 ? err : 0);
}

void lsm6dsm_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
	if (accel_range > 8)
	{
		accel_fs = DSM_FS_XL_16G;
		accel_range = 16;
	}
	else if (accel_range > 4)
	{
		accel_fs = DSM_FS_XL_8G;
		accel_range = 8;
	}
	else if (accel_range > 2)
	{
		accel_fs = DSM_FS_XL_4G;
		accel_range = 4;
	}
	else
	{
		accel_fs = DSM_FS_XL_2G;
		accel_range = 2;
	}

	if (gyro_range > 1000)
	{
		gyro_fs = DSM_FS_G_2000DPS;
		gyro_range = 2000;
	}
	else if (gyro_range > 500)
	{
		gyro_fs = DSM_FS_G_1000DPS;
		gyro_range = 1000;
	}
	else if (gyro_range > 250)
	{
		gyro_fs = DSM_FS_G_500DPS;
		gyro_range = 500;
	}
	else
	{
		gyro_fs = DSM_FS_G_250DPS;
		gyro_range = 250;
	}

	accel_sensitivity = accel_range / 32768.0f;
	gyro_sensitivity = 35.0f * gyro_range / 1000000.0f;

	*accel_actual_range = accel_range;
	*gyro_actual_range = gyro_range;
}

int lsm6dsm_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	int ODR;
	uint8_t OP_MODE_XL;
	uint8_t OP_MODE_G;
	uint8_t ODR_XL;
	uint8_t ODR_G;
	uint8_t GYRO_SLEEP = DSM_OP_MODE_G_AWAKE;

	// Calculate accel
	if (accel_time <= 0 || accel_time == INFINITY) // off, standby interpreted as off
	{
		// set High perf mode and off odr on XL
		OP_MODE_XL = DSM_OP_MODE_XL_HP;
		ODR_XL = DSM_ODR_OFF;
		ODR = 0;
	}
	else
	{
		// set High perf mode and select odr on XL
		OP_MODE_XL = DSM_OP_MODE_XL_HP;
		ODR = 1 / accel_time;
	}

	if (ODR == 0)
	{
		accel_time = 0; // off
		ODR_XL = DSM_ODR_OFF;
	}
	else if (accel_time < 0.3f / 1000) // in this case it seems better to compare accel_time
	{
		ODR_XL = DSM_ODR_6_66kHz; // TODO: this is absolutely awful
		accel_time = 0.15 / 1000;
	}
	else if (accel_time < 0.6f / 1000)
	{
		ODR_XL = DSM_ODR_3_33kHz;
		accel_time = 0.3 / 1000;
	}
	else if (accel_time < 1.2f / 1000)
	{
		ODR_XL = DSM_ODR_1_66kHz;
		accel_time = 0.6 / 1000;
	}
	else if (accel_time < 2.4f / 1000)
	{
		ODR_XL = DSM_ODR_833Hz;
		accel_time = 1.2 / 1000;
	}
	else if (accel_time < 4.8f / 1000)
	{
		ODR_XL = DSM_ODR_416Hz;
		accel_time = 2.4 / 1000;
	}
	else if (accel_time < 9.6f / 1000)
	{
		ODR_XL = DSM_ODR_208Hz;
		accel_time = 4.8 / 1000;
	}
	else if (accel_time < 19.2f / 1000)
	{
		ODR_XL = DSM_ODR_104Hz;
		accel_time = 9.6 / 1000;
	}
	else if (accel_time < 38.4f / 1000)
	{
		ODR_XL = DSM_ODR_52Hz;
		accel_time = 19.2 / 1000;
	}
	else if (ODR > 12.5)
	{
		ODR_XL = DSM_ODR_26Hz;
		accel_time = 38.4 / 1000;
	}
	else
	{
		ODR_XL = DSM_ODR_12_5Hz;
		accel_time = 1.0 / 12.5; // 13Hz -> 76.8 / 1000
	}

	// Calculate gyro
	if (gyro_time <= 0) // off
	{
		OP_MODE_G = DSM_OP_MODE_G_HP;
		ODR_G = DSM_ODR_OFF;
		ODR = 0;
	}
	else if (gyro_time == INFINITY) // sleep
	{
		OP_MODE_G = DSM_OP_MODE_G_NP;
		GYRO_SLEEP = DSM_OP_MODE_G_SLEEP;
		ODR_G = last_gyro_odr; // using last ODR
		ODR = 0;
	}
	else
	{
		OP_MODE_G = DSM_OP_MODE_G_HP;
		ODR_G = 0; // the compiler complains unless I do this
		ODR = 1 / gyro_time;
	}

	if (ODR == 0)
	{
		gyro_time = 0; // off
		ODR_G = DSM_ODR_OFF;
	}
	else if (gyro_time < 0.3f / 1000) // in this case it seems better to compare gyro_time
	{
		ODR_G = DSM_ODR_6_66kHz; // TODO: this is absolutely awful
		gyro_time = 1.0 / 6660;
	}
	else if (gyro_time < 0.6f / 1000)
	{
		ODR_G = DSM_ODR_3_33kHz;
		gyro_time = 0.3 / 1000;
	}
	else if (gyro_time < 1.2f / 1000)
	{
		ODR_G = DSM_ODR_1_66kHz;
		gyro_time = 0.6 / 1000;
	}
	else if (gyro_time < 2.4f / 1000)
	{
		ODR_G = DSM_ODR_833Hz;
		gyro_time = 1.2 / 1000;
	}
	else if (gyro_time < 4.8f / 1000)
	{
		ODR_G = DSM_ODR_416Hz;
		gyro_time = 2.4 / 1000;
	}
	else if (gyro_time < 9.6f / 1000)
	{
		ODR_G = DSM_ODR_208Hz;
		gyro_time = 4.8 / 1000;
	}
	else if (gyro_time < 19.2f / 1000)
	{
		ODR_G = DSM_ODR_104Hz;
		gyro_time = 9.6 / 1000;
	}
	else if (gyro_time < 38.4f / 1000)
	{
		ODR_G = DSM_ODR_52Hz;
		gyro_time = 19.2 / 1000;
	}
	else if (ODR > 12.5)
	{
		ODR_G = DSM_ODR_26Hz;
		gyro_time = 38.4 / 1000;
	}
	else
	{
		ODR_G = DSM_ODR_12_5Hz;
		gyro_time = 1.0 / 12.5; // 13Hz -> 76.8 / 1000
	}

	if (last_accel_mode == OP_MODE_XL && last_gyro_mode == OP_MODE_G && last_accel_odr == ODR_XL && last_gyro_odr == ODR_G) // if both were already configured
		return 1;

	last_accel_mode = OP_MODE_XL;
	last_gyro_mode = OP_MODE_G;
	last_accel_odr = ODR_XL;
	last_gyro_odr = ODR_G;

	uint8_t ODR_FIFO = MAX(ODR_XL, ODR_G);

	// calculate decimation configuration, see datasheet
	uint8_t DEC_G = (ODR_FIFO - ODR_G) >> 4; // difference is the decimation factor
	DEC_G = DEC_G >= 6 ? 0 : (DEC_G >= 2 ? DEC_G + 2 : DEC_G + 1);

	uint8_t DEC_XL = (ODR_FIFO - ODR_XL) >> 4;
	DEC_XL = DEC_XL >= 6 ? 0 : (DEC_XL >= 2 ? DEC_XL + 2 : DEC_XL + 1);

	if (gyro_time > accel_time)
	{
		fifo_pattern_gyro_dominant = false;
		fifo_pattern_length = accel_time == 0 ? 1 : gyro_time / accel_time + 0.5f;
	}
	else
	{
		fifo_pattern_gyro_dominant = true;
		fifo_pattern_length = gyro_time == 0 ? 1 : accel_time / gyro_time + 0.5f;
	}

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_CTRL1, ODR_XL | accel_fs); // set accel ODR and FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_CTRL6, OP_MODE_XL); // set accelerator perf mode

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_CTRL2, ODR_G | gyro_fs); // set gyro ODR and mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_CTRL7, OP_MODE_G); // set gyroscope perf mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_CTRL4, GYRO_SLEEP); // set gyroscope awake/sleep mode

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_FIFO_CTRL3, (DEC_G << 3) | DEC_XL); // set decimation
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_FIFO_CTRL5, (ODR_FIFO >> 1) | 0x06); // set FIFO ODR, FIFO Continuous mode
	if (err)
		LOG_ERR("Communication error");

	*accel_actual_time = accel_time;
	*gyro_actual_time = gyro_time;

	return 0;
}

uint16_t lsm6dsm_fifo_read(uint8_t *data, uint16_t len)
{
	int err = 0;
	uint16_t total = 0;
	uint16_t count = UINT16_MAX;
	uint16_t pattern = 0;
	while (count > 0 && len >= PACKET_SIZE)
	{

		uint8_t rawCount[4];
		err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_FIFO_STATUS1, &rawCount[0], 2);
		count = (uint16_t)((rawCount[1] & 7) << 8 | rawCount[0]); // Turn the 16 bits into a unsigned 16-bit value
		count /= PACKET_SIZE / 2; // words to "packets" (actually PACKET_SIZE - 1)
		uint16_t limit = len / PACKET_SIZE;
		if (count > limit)
		{
			LOG_WRN("FIFO read buffer limit reached, %d packets dropped", count - limit);
			count = limit;
		}
		for (int i = 0; i < count; i++)
		{
			err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_FIFO_STATUS3, &rawCount[0], 2); // reading pattern
			pattern = (uint16_t)((rawCount[1] & 3) << 8 | rawCount[0]);
			if (pattern % 3 != 0) // misaligned!
			{
				LOG_WRN("FIFO not aligned");
				err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_FIFO_DATA_OUT_L, rawCount, (pattern % 3) * 2); // read and discard misaligned axes
				count--;
			}
			data[i * PACKET_SIZE] = pattern / 3;
			err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_FIFO_DATA_OUT_L, &data[i * PACKET_SIZE + 1], PACKET_SIZE - 1);
		}
		if (err)
			LOG_ERR("Communication error");
		data += count * PACKET_SIZE;
		len -= count * PACKET_SIZE;
		total += count;
	}
	return total;
}

int lsm6dsm_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	index *= PACKET_SIZE;
	uint8_t pattern = data[index];
	if ((pattern == 0 && (fifo_pattern_length != 1 || fifo_pattern_gyro_dominant)) || (pattern > 1 && fifo_pattern_gyro_dominant))
	{
		for (int i = 0; i < 3; i++) // x, y, z
		{
			g[i] = (int16_t)((((uint16_t)data[index + 2 + (i * 2)]) << 8) | data[index + 1 + (i * 2)]);
			g[i] *= gyro_sensitivity;
		}
		return 0;
	}
	else
	{
		for (int i = 0; i < 3; i++) // x, y, z
		{
			a[i] = (int16_t)((((uint16_t)data[index + 2 + (i * 2)]) << 8) | data[index + 1 + (i * 2)]);
			a[i] *= accel_sensitivity;
		}
		return 0;
	}
	return 1;
}

/* LSM6DSM does not have COUNTER_BDR, FIFO threshold uses word count, or 3 words per sensor sample
 * ex. if gyro odr is higher, and pattern length is 3:
 * first timestamp will have both gyro and accel
 * second (and above) timestamp will contain only gyro for the rest of the pattern length
 * 0->(0+0)*3, 1->(1+1)*3, 2->(1+2)*3, 3->(1+3)*3, 4->(2+4)*3
 * this assumes pattern always begins at 0 on fifo read, which it will not.
 */
uint8_t lsm6dsm_setup_DRDY(uint16_t threshold)
{
	fifo_pattern_length;
	uint8_t ratio = 0;
	if (last_gyro_odr > 0 || last_accel_odr > 0)
	{
		if (fifo_pattern_gyro_dominant)
		{
			ratio = (last_gyro_odr - last_accel_odr) >> 4;
		}
		else
		{
			ratio = (last_accel_odr - last_gyro_odr) >> 4;
		}
	}
	threshold = ((threshold + fifo_pattern_length - 1) / fifo_pattern_length + threshold) * 3;
	uint8_t buf[2];
	buf[0] = threshold & 0xFF;
	buf[1] = (threshold >> 8) & 0x07;
	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_FIFO_CTRL1, buf, 2);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_INT1_CTRL, 0x08); // FIFO threshold interrupt
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

uint8_t lsm6dsm_setup_WOM(void) // TODO:
{ // TODO: should be off by the time WOM will be setup
//	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_CTRL1, ODR_OFF); // set accel off
//	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_CTRL2, ODR_OFF); // set gyro off

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_CTRL1, DSM_ODR_208Hz | DSM_FS_XL_8G); // set accel ODR and FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_CTRL6, DSM_OP_MODE_XL_NP); // set accel perf mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_CTRL8, 0x74); // set HPCF_XL to the lowest bandwidth, enable HP_REF_MODE (set HP_REF_MODE, HP_SLOPE_XL_EN, HPCF_XL nonzero)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_TAP_CFG, 0x10); // set SLOPE_FDS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_WAKE_UP_THS, 0x01); // set threshold, 1 * 31.25 mg is ~31.25 mg
	k_msleep(12); // need to wait for accel to settle

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_TAP_CFG, 0x90); // enable interrupts (keep SLOPE_FDS)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_MD1_CFG, 0x20); // route wake-up to INT1
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSM_CTRL3, 0x30); // INT H_LACTIVE active low, PP_OD open-drain
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

const sensor_imu_t sensor_imu_lsm6dsm = {
	*lsm6dsm_init,
	*lsm_shutdown,

	*lsm6dsm_update_fs,
	*lsm6dsm_update_odr,

	*lsm6dsm_fifo_read,
	*lsm6dsm_fifo_process,
	*lsm_accel_read,
	*lsm_gyro_read,
	*lsm_temp_read,

	*lsm6dsm_setup_DRDY,
	*lsm6dsm_setup_WOM,
	
	*imu_none_ext_setup,
	*lsm_ext_passthrough
};

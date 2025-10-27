#include <math.h>

#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#include "LSM6DSO.h"
#include "LSM6DSV.h" // Common functions
#include "sensor/sensor_none.h"

#define PACKET_SIZE 7

static uint8_t accel_fs = DSO_FS_XL_16G;
static uint8_t gyro_fs = DSO_FS_G_2000DPS;

static float freq_scale = 1; // ODR is scaled by INTERNAL_FREQ_FINE

LOG_MODULE_REGISTER(LSM6DSO, LOG_LEVEL_DBG);

int lsm6dso_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	// setup interface for SPI
	sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(10), 0);
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL3, 0x74); // freeze register until done reading, increment register address during multi-byte access (BDU, IF_INC), INT H_LACTIVE active low, PP_OD open-drain
	if (err)
		LOG_ERR("Communication error");
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	int8_t internal_freq_fine;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_INTERNAL_FREQ_FINE, &internal_freq_fine); // affects ODR
	freq_scale = 1.0f + 0.0015f * (float)internal_freq_fine;
	err |= lsm6dso_update_odr(accel_time, gyro_time, accel_actual_time, gyro_actual_time);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FIFO_CTRL4, 0x06); // enable Continuous mode
	if (err)
		LOG_ERR("Communication error");
	return (err < 0 ? err : 0);
}

void lsm6dso_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
	if (accel_range > 8)
	{
		accel_fs = DSO_FS_XL_16G;
		accel_range = 16;
	}
	else if (accel_range > 4)
	{
		accel_fs = DSO_FS_XL_8G;
		accel_range = 8;
	}
	else if (accel_range > 2)
	{
		accel_fs = DSO_FS_XL_4G;
		accel_range = 4;
	}
	else
	{
		accel_fs = DSO_FS_XL_2G;
		accel_range = 2;
	}

	if (gyro_range > 1000)
	{
		gyro_fs = DSO_FS_G_2000DPS;
		gyro_range = 2000;
	}
	else if (gyro_range > 500)
	{
		gyro_fs = DSO_FS_G_1000DPS;
		gyro_range = 1000;
	}
	else if (gyro_range > 250)
	{
		gyro_fs = DSO_FS_G_500DPS;
		gyro_range = 500;
	}
	else
	{
		gyro_fs = DSO_FS_G_250DPS;
		gyro_range = 250;
	}

	accel_sensitivity = accel_range / 32768.0f;
	gyro_sensitivity = 35.0f * gyro_range / 1000000.0f;

	*accel_actual_range = accel_range;
	*gyro_actual_range = gyro_range;
}

int lsm6dso_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	int ODR;
	uint8_t OP_MODE_XL;
	uint8_t OP_MODE_G;
	uint8_t ODR_XL;
	uint8_t ODR_G;
	uint8_t GYRO_SLEEP = DSO_OP_MODE_G_AWAKE;

	// Calculate accel
	if (accel_time <= 0 || accel_time == INFINITY) // off, standby interpreted as off
	{
		// set High perf mode and off odr on XL
		OP_MODE_XL = DSO_OP_MODE_XL_HP;
		ODR_XL = DSO_ODR_OFF;
		ODR = 0;
	}
	else
	{
		// set High perf mode and select odr on XL
		OP_MODE_XL = DSO_OP_MODE_XL_HP;
		ODR = 1 / accel_time;
		ODR /= freq_scale; // scale by internal freq adjustment
	}

	if (ODR == 0)
	{
		accel_time = 0; // off
		ODR_XL = DSO_ODR_OFF;
	}
	else if (accel_time < 0.3f / 1000) // in this case it seems better to compare accel_time
	{
		ODR_XL = DSO_ODR_6_66kHz; // TODO: this is absolutely awful
		accel_time = 0.15 / 1000;
	}
	else if (accel_time < 0.6f / 1000)
	{
		ODR_XL = DSO_ODR_3_33kHz;
		accel_time = 0.3 / 1000;
	}
	else if (accel_time < 1.2f / 1000)
	{
		ODR_XL = DSO_ODR_1_66kHz;
		accel_time = 0.6 / 1000;
	}
	else if (accel_time < 2.4f / 1000)
	{
		ODR_XL = DSO_ODR_833Hz;
		accel_time = 1.2 / 1000;
	}
	else if (accel_time < 4.8f / 1000)
	{
		ODR_XL = DSO_ODR_416Hz;
		accel_time = 2.4 / 1000;
	}
	else if (accel_time < 9.6f / 1000)
	{
		ODR_XL = DSO_ODR_208Hz;
		accel_time = 4.8 / 1000;
	}
	else if (accel_time < 19.2f / 1000)
	{
		ODR_XL = DSO_ODR_104Hz;
		accel_time = 9.6 / 1000;
	}
	else if (accel_time < 38.4f / 1000)
	{
		ODR_XL = DSO_ODR_52Hz;
		accel_time = 19.2 / 1000;
	}
	else if (ODR > 12.5)
	{
		ODR_XL = DSO_ODR_26Hz;
		accel_time = 38.4 / 1000;
	}
	else
	{
		ODR_XL = DSO_ODR_12_5Hz;
		accel_time = 1.0 / 12.5; // 13Hz -> 76.8 / 1000
	}
	accel_time /= freq_scale; // scale by internal freq adjustment

	// Calculate gyro
	if (gyro_time <= 0) // off
	{
		OP_MODE_G = DSO_OP_MODE_G_HP;
		ODR_G = DSO_ODR_OFF;
		ODR = 0;
	}
	else if (gyro_time == INFINITY) // sleep
	{
		OP_MODE_G = DSO_OP_MODE_G_NP;
		GYRO_SLEEP = DSO_OP_MODE_G_SLEEP;
		ODR_G = last_gyro_odr; // using last ODR
		ODR = 0;
	}
	else
	{
		OP_MODE_G = DSO_OP_MODE_G_HP;
		ODR_G = 0; // the compiler complains unless I do this
		ODR = 1 / gyro_time;
		ODR /= freq_scale; // scale by internal freq adjustment
	}

	if (ODR == 0)
	{
		gyro_time = 0; // off
		ODR_G = DSO_ODR_OFF;
	}
	else if (gyro_time < 0.3f / 1000) // in this case it seems better to compare gyro_time
	{
		ODR_G = DSO_ODR_6_66kHz; // TODO: this is absolutely awful
		gyro_time = 1.0 / 6660;
	}
	else if (gyro_time < 0.6f / 1000)
	{
		ODR_G = DSO_ODR_3_33kHz;
		gyro_time = 0.3 / 1000;
	}
	else if (gyro_time < 1.2f / 1000)
	{
		ODR_G = DSO_ODR_1_66kHz;
		gyro_time = 0.6 / 1000;
	}
	else if (gyro_time < 2.4f / 1000)
	{
		ODR_G = DSO_ODR_833Hz;
		gyro_time = 1.2 / 1000;
	}
	else if (gyro_time < 4.8f / 1000)
	{
		ODR_G = DSO_ODR_416Hz;
		gyro_time = 2.4 / 1000;
	}
	else if (gyro_time < 9.6f / 1000)
	{
		ODR_G = DSO_ODR_208Hz;
		gyro_time = 4.8 / 1000;
	}
	else if (gyro_time < 19.2f / 1000)
	{
		ODR_G = DSO_ODR_104Hz;
		gyro_time = 9.6 / 1000;
	}
	else if (gyro_time < 38.4f / 1000)
	{
		ODR_G = DSO_ODR_52Hz;
		gyro_time = 19.2 / 1000;
	}
	else if (ODR > 12.5)
	{
		ODR_G = DSO_ODR_26Hz;
		gyro_time = 38.4 / 1000;
	}
	else
	{
		ODR_G = DSO_ODR_12_5Hz;
		gyro_time = 1.0 / 12.5; // 13Hz -> 76.8 / 1000
	}
	gyro_time /= freq_scale; // scale by internal freq adjustment

	if (last_accel_mode == OP_MODE_XL && last_gyro_mode == OP_MODE_G && last_accel_odr == ODR_XL && last_gyro_odr == ODR_G) // if both were already configured
		return 1;

	last_accel_mode = OP_MODE_XL;
	last_gyro_mode = OP_MODE_G;
	last_accel_odr = ODR_XL;
	last_gyro_odr = ODR_G;

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL1, ODR_XL | accel_fs); // set accel ODR and FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL6, OP_MODE_XL); // set accelerator perf mode

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL2, ODR_G | gyro_fs); // set gyro ODR and mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL7, OP_MODE_G); // set gyroscope perf mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL4, GYRO_SLEEP); // set gyroscope awake/sleep mode

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FIFO_CTRL3, (ODR_XL >> 4) | ODR_G); // set accel and gyro batch rate
	if (err)
		LOG_ERR("Communication error");

	*accel_actual_time = accel_time;
	*gyro_actual_time = gyro_time;

	return 0;
}

uint16_t lsm6dso_fifo_read(uint8_t *data, uint16_t len)
{
	int err = 0;
	uint16_t total = 0;
	uint16_t count = UINT16_MAX;
	while (count > 0 && len >= PACKET_SIZE)
	{
		uint8_t rawCount[2];
		err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FIFO_STATUS1, &rawCount[0], 2);
		count = (uint16_t)((rawCount[1] & 3) << 8 | rawCount[0]); // Turn the 16 bits into a unsigned 16-bit value // TODO: might be 3 bits not 2
		uint16_t limit = len / PACKET_SIZE;
		if (count > limit)
		{
			LOG_WRN("FIFO read buffer limit reached, %d packets dropped", count - limit);
			count = limit;
		}
		for (int i = 0; i < count; i++)
			err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_FIFO_DATA_OUT_TAG, &data[i * PACKET_SIZE], PACKET_SIZE);
		if (err)
			LOG_ERR("Communication error");
		data += count * PACKET_SIZE;
		len -= count * PACKET_SIZE;
		total += count;
	}
	return total;
}

uint8_t lsm6dso_setup_WOM(void)
{ // TODO: should be off by the time WOM will be setup
//	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL1, ODR_OFF); // set accel off
//	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL2, ODR_OFF); // set gyro off

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL1, DSO_ODR_208Hz | DSO_FS_XL_8G); // set accel ODR and FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL6, DSO_OP_MODE_XL_NP); // set accel perf mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL5, 0x80); // enable accel ULP // TODO: for LSM6DSR/ISM330DHCX this bit may be required to be 0
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL8, 0xF4); // set HPCF_XL to the lowest bandwidth, enable HP_REF_MODE (set HP_REF_MODE_XL, HP_SLOPE_XL_EN, HPCF_XL nonzero)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_TAP_CFG0, 0x10); // set SLOPE_FDS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_WAKE_UP_THS, 0x01); // set threshold, 1 * 31.25 mg is ~31.25 mg
	k_msleep(12); // need to wait for accel to settle

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_TAP_CFG2, 0x80); // enable interrupts
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_MD1_CFG, 0x20); // route wake-up to INT1
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_CTRL3, 0x30); // INT H_LACTIVE active low, PP_OD open-drain
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

int lsm6dso_ext_setup(void)
{
	sensor_interface_ext_configure(&sensor_ext_lsm6dsv);
	return 0;
}

int lsm6dso_ext_passthrough(bool passthrough)
{
	int err = 0;
	if (passthrough)
	{
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x10); // passthrough on
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	}
	else
	{
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x08); // passthrough off
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	}
	if (err)
		LOG_ERR("Communication error");
	return 0;
}

int lsm6dso_ext_write(const uint8_t addr, const uint8_t *buf, uint32_t num_bytes)
{
	if (num_bytes != 2)
	{
		LOG_ERR("Unsupported write");
		return -1;
	}
	// Configure transaction and begin one-shot (AN5922, page 80, One-shot write routine)
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	uint8_t slv0[3] = {(addr << 1) | 0x00, buf[0], 0x00 | 0x00}; // write, SHUB_ODR = 104Hz, reading no bytes
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_ADD, slv0, 3);
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_ADD, (addr << 1) | 0x00); // write
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_SUBADD, buf[0]);
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_CONFIG, 0x00 | 0x00); // SHUB_ODR = 104Hz, reading no bytes
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_DATAWRITE_SLV0, buf[1]);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x4C); // WRITE_ONCE, SHUB_PU_EN, enable I2C master
	// Wait for transaction
	uint8_t status = 0;
	int64_t timeout = k_uptime_get() + 10;
	while ((status & 0x80) && k_uptime_get() < timeout) // WR_ONCE_DONE
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_STATUS_MASTER, &status);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x08); // SHUB_PU_EN, disable I2C master
	k_usleep(300);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	if (~status & 0x80)
	{
		LOG_ERR("Write timeout");
		return -1;
	}
	return err;
}

int lsm6dso_ext_write_read(const uint8_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read)
{
	if (num_write != 1 || num_read < 1 || num_read > 8)
	{
		LOG_ERR("Unsupported write_read");
		return -1;
	}
	// Configure transaction and begin one-shot (AN5922, page 79, One-shot read routine)
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	uint8_t slv0[3] = {(addr << 1) | 0x01, ((const uint8_t *)write_buf)[0], 0x00 | num_read}; // read, SHUB_ODR = 104Hz, reading num_read bytes
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_ADD, slv0, 3);
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_ADD, (addr << 1) | 0x01); // read
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_SUBADD, ((const uint8_t *)write_buf)[0]);
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_CONFIG, 0x00 | num_read); // SHUB_ODR = 104Hz, reading num_read bytes
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x4C); // WRITE_ONCE mandatory for read, SHUB_PU_EN, enable I2C master
	// Wait for transaction
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	uint8_t tmp;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_OUTX_H_A, &tmp); // clear XLDA
	uint8_t status = 0;
	int64_t timeout = k_uptime_get() + 10;
	while ((status & 0x01) && k_uptime_get() < timeout) // XLDA
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_STATUS_REG, &status);
	status = 0;
	timeout = k_uptime_get() + 10;
	while ((status & 0x01) && k_uptime_get() < timeout) // SENS_HUB_ENDOP
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSO_STATUS_MASTER_MAINPAGE, &status);
	// Read data
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x08); // SHUB_PU_EN, disable I2C master
	k_usleep(300);
	err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SENSOR_HUB_1, read_buf, num_read);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	return err;
}

const sensor_imu_t sensor_imu_lsm6dso = {
	*lsm6dso_init,
	*lsm_shutdown,

	*lsm6dso_update_fs,
	*lsm6dso_update_odr,

	*lsm6dso_fifo_read,
	*lsm_fifo_process,
	*lsm_accel_read,
	*lsm_gyro_read,
	*lsm_temp_read,

	*lsm_setup_DRDY,
	*lsm6dso_setup_WOM,
	
	*lsm6dso_ext_setup,
	*lsm6dso_ext_passthrough
};

const sensor_ext_ssi_t sensor_ext_lsm6dso = {
	*lsm6dso_ext_write,
	*lsm6dso_ext_write_read,
	8
};

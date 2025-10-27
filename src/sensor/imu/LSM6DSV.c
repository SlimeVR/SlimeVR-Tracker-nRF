#include <math.h>

#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#include "LSM6DSV.h"
#include "sensor/sensor_none.h"

#define PACKET_SIZE 7

// TODO: shared with LSM
float accel_sensitivity = 16.0f / 32768.0f; // Default 16G (FS = ±16 g: 0.488 mg/LSB)
float gyro_sensitivity = 0.070f; // Default 2000dps (FS = ±2000 dps: 70 mdps/LSB)

static uint8_t accel_fs = FS_XL_16G;
static uint8_t gyro_fs = FS_G_2000DPS;

// TODO: shared with LSM
uint8_t last_accel_mode = 0xff;
uint8_t last_gyro_mode = 0xff;
uint8_t last_accel_odr = 0xff;
uint8_t last_gyro_odr = 0xff;

static float freq_scale = 1; // ODR is scaled by INTERNAL_FREQ_FINE

LOG_MODULE_REGISTER(LSM6DSV, LOG_LEVEL_DBG);

int lsm_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	// setup interface for SPI
	sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(10), 0);
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL6, gyro_fs); // set gyro FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL8, accel_fs); // set accel FS
	if (err)
		LOG_ERR("Communication error");
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_IF_CFG, 0x18); // INT H_LACTIVE active low, PP_OD open-drain
	int8_t internal_freq_fine;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_INTERNAL_FREQ_FINE, &internal_freq_fine); // affects ODR
	freq_scale = 1.0f + 0.0013f * (float)internal_freq_fine;
	err |= lsm_update_odr(accel_time, gyro_time, accel_actual_time, gyro_actual_time);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL4, 0x06); // enable Continuous mode
	if (err)
		LOG_ERR("Communication error");
	return (err < 0 ? err : 0);
}

void lsm_shutdown(void)
{
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL3, 0x01); // SW_RESET
	if (err)
		LOG_ERR("Communication error");
}

void lsm_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
	if (accel_range > 8)
	{
		accel_fs = FS_XL_16G;
		accel_range = 16;
	}
	else if (accel_range > 4)
	{
		accel_fs = FS_XL_8G;
		accel_range = 8;
	}
	else if (accel_range > 2)
	{
		accel_fs = FS_XL_4G;
		accel_range = 4;
	}
	else
	{
		accel_fs = FS_XL_2G;
		accel_range = 2;
	}

	if (gyro_range > 2000)
	{
		gyro_fs = FS_G_4000DPS;
		gyro_range = 4000;
	}
	else if (gyro_range > 1000)
	{
		gyro_fs = FS_G_2000DPS;
		gyro_range = 2000;
	}
	else if (gyro_range > 500)
	{
		gyro_fs = FS_G_1000DPS;
		gyro_range = 1000;
	}
	else if (gyro_range > 250)
	{
		gyro_fs = FS_G_500DPS;
		gyro_range = 500;
	}
	else if (gyro_range > 125)
	{
		gyro_fs = FS_G_250DPS;
		gyro_range = 250;
	}
	else
	{
		gyro_fs = FS_G_125DPS;
		gyro_range = 125;
	}

	accel_sensitivity = accel_range / 32768.0f;
	gyro_sensitivity = 35.0f * gyro_range / 1000000.0f;

	*accel_actual_range = accel_range;
	*gyro_actual_range = gyro_range;
}

int lsm_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	int ODR;
	uint8_t OP_MODE_XL;
	uint8_t OP_MODE_G;
	uint8_t ODR_XL;
	uint8_t ODR_G;

	// Calculate accel
	if (accel_time <= 0 || accel_time == INFINITY) // off, standby interpreted as off
	{
		OP_MODE_XL = OP_MODE_XL_HP;
		ODR_XL = ODR_OFF;
		ODR = 0;
	}
	else
	{
		OP_MODE_XL = OP_MODE_XL_HP;
		ODR = 1 / accel_time;
		ODR /= freq_scale; // scale by internal freq adjustment
	}

	if (ODR == 0)
	{
		accel_time = 0; // off
	}
	else if (ODR > 3840) // TODO: this is absolutely awful
	{
		ODR_XL = ODR_7_68kHz;
		accel_time = 1.0 / 7680;
	}
	else if (ODR > 1920)
	{
		ODR_XL = ODR_3_84kHz;
		accel_time = 1.0 / 3840;
	}
	else if (ODR > 960)
	{
		ODR_XL = ODR_1_92kHz;
		accel_time = 1.0 / 1920;
	}
	else if (ODR > 480)
	{
		ODR_XL = ODR_960Hz;
		accel_time = 1.0 / 960;
	}
	else if (ODR > 240)
	{
		ODR_XL = ODR_480Hz;
		accel_time = 1.0 / 480;
	}
	else if (ODR > 120)
	{
		ODR_XL = ODR_240Hz;
		accel_time = 1.0 / 240;
	}
	else if (ODR > 60)
	{
		ODR_XL = ODR_120Hz;
		accel_time = 1.0 / 120;
	}
	else if (ODR > 30)
	{
		ODR_XL = ODR_60Hz;
		accel_time = 1.0 / 60;
	}
	else if (ODR > 15)
	{
		ODR_XL = ODR_30Hz;
		accel_time = 1.0 / 30;
	}
	else if (ODR > 7.5)
	{
		ODR_XL = ODR_15Hz;
		accel_time = 1.0 / 15;
	}
	else if (ODR > 1.875)
	{
		ODR_XL = ODR_7_5Hz;
		accel_time = 1.0 / 7.5;
	}
	else
	{
		ODR_XL = ODR_1_875Hz;
		accel_time = 1.0 / 1.875;
	}
	accel_time /= freq_scale; // scale by internal freq adjustment

	// Calculate gyro
	if (gyro_time <= 0) // off
	{
		OP_MODE_G = OP_MODE_G_HP;
		ODR_G = ODR_OFF;
		ODR = 0;
	}
	else if (gyro_time == INFINITY) // sleep
	{
		OP_MODE_G = OP_MODE_G_SLEEP;
		ODR_G = last_gyro_odr; // using last ODR
		ODR = 0;
	}
	else
	{
		OP_MODE_G = OP_MODE_G_HP;
		ODR_G = 0; // the compiler complains unless I do this
		ODR = 1 / gyro_time;
		ODR /= freq_scale; // scale by internal freq adjustment
	}

	if (ODR == 0)
	{
		gyro_time = 0; // off
	}
	else if (ODR > 3840) // TODO: this is absolutely awful
	{
		ODR_G = ODR_7_68kHz;
		gyro_time = 1.0 / 7680;
	}
	else if (ODR > 1920)
	{
		ODR_G = ODR_3_84kHz;
		gyro_time = 1.0 / 3840;
	}
	else if (ODR > 960)
	{
		ODR_G = ODR_1_92kHz;
		gyro_time = 1.0 / 1920;
	}
	else if (ODR > 480)
	{
		ODR_G = ODR_960Hz;
		gyro_time = 1.0 / 960;
	}
	else if (ODR > 240)
	{
		ODR_G = ODR_480Hz;
		gyro_time = 1.0 / 480;
	}
	else if (ODR > 120)
	{
		ODR_G = ODR_240Hz;
		gyro_time = 1.0 / 240;
	}
	else if (ODR > 60)
	{
		ODR_G = ODR_120Hz;
		gyro_time = 1.0 / 120;
	}
	else if (ODR > 30)
	{
		ODR_G = ODR_60Hz;
		gyro_time = 1.0 / 60;
	}
	else if (ODR > 15)
	{
		ODR_G = ODR_30Hz;
		gyro_time = 1.0 / 30;
	}
	else if (ODR > 7.5)
	{
		ODR_G = ODR_15Hz;
		gyro_time = 1.0 / 15;
	}
	else if (ODR > 1.875)
	{
		ODR_G = ODR_7_5Hz;
		gyro_time = 1.0 / 7.5;
	}
	else
	{
		ODR_G = ODR_1_875Hz;
		gyro_time = 1.0 / 1.875;
	}
	gyro_time /= freq_scale; // scale by internal freq adjustment

	if (last_accel_mode == OP_MODE_XL && last_gyro_mode == OP_MODE_G && last_accel_odr == ODR_XL && last_gyro_odr == ODR_G) // if both were already configured
		return 1;

	last_accel_mode = OP_MODE_XL;
	last_gyro_mode = OP_MODE_G;
	last_accel_odr = ODR_XL;
	last_gyro_odr = ODR_G;

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL1, OP_MODE_XL << 4 | ODR_XL); // set accel ODR and mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL2, OP_MODE_G << 4 | ODR_G); // set gyro ODR and mode

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL3, ODR_XL | (ODR_G << 4)); // set accel and gyro batch rate
	if (err)
		LOG_ERR("Communication error");

	*accel_actual_time = accel_time;
	*gyro_actual_time = gyro_time;

	return 0;
}

uint16_t lsm_fifo_read(uint8_t *data, uint16_t len)
{
	int err = 0;
	uint16_t total = 0;
	uint16_t count = UINT16_MAX;
	while (count > 0 && len >= PACKET_SIZE)
	{
		uint8_t rawCount[2];
		err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_STATUS1, &rawCount[0], 2);
		count = (uint16_t)((rawCount[1] & 3) << 8 | rawCount[0]); // Turn the 16 bits into a unsigned 16-bit value. Only LSB on FIFO_STATUS2 is used, but we mask 2nd bit too // TODO: might be 3 bits not 2
		uint16_t limit = len / PACKET_SIZE;
		if (count > limit)
		{
			LOG_WRN("FIFO read buffer limit reached, %d packets dropped", count - limit);
			count = limit;
		}
		for (int i = 0; i < count; i++)
			err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_DATA_OUT_TAG, &data[i * PACKET_SIZE], PACKET_SIZE);
		if (err)
			LOG_ERR("Communication error");
		data += count * PACKET_SIZE;
		len -= count * PACKET_SIZE;
		total += count;
	}
	return total;
}

int lsm_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	index *= PACKET_SIZE;
	if ((data[index] >> 3) == 0x02) // Accelerometer NC (Accelerometer uncompressed data)
	{
		for (int i = 0; i < 3; i++) // x, y, z
		{
			a[i] = (int16_t)((((uint16_t)data[index + 2 + (i * 2)]) << 8) | data[index + 1 + (i * 2)]);
			a[i] *= accel_sensitivity;
		}
		return 0;
	}
	if ((data[index] >> 3) == 0x01) // Gyroscope NC (Gyroscope uncompressed data)
	{
		for (int i = 0; i < 3; i++) // x, y, z
		{
			g[i] = (int16_t)((((uint16_t)data[index + 2 + (i * 2)]) << 8) | data[index + 1 + (i * 2)]);
			g[i] *= gyro_sensitivity;
		}
		return 0;
	}
	// TODO: need to skip invalid data
	return 1;
}

void lsm_accel_read(float a[3])
{
	uint8_t rawAccel[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_OUTX_L_A, &rawAccel[0], 6);
	if (err)
		LOG_ERR("Communication error");
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a[i] = (int16_t)((((uint16_t)rawAccel[1 + (i * 2)]) << 8) | rawAccel[i * 2]);
		a[i] *= accel_sensitivity;
	}
	// TODO: for ISM330BX, the accelerometer data is in ZYX order
}

void lsm_gyro_read(float g[3])
{
	uint8_t rawGyro[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_OUTX_L_G, &rawGyro[0], 6);
	if (err)
		LOG_ERR("Communication error");
	for (int i = 0; i < 3; i++) // x, y, z
	{
		g[i] = (int16_t)((((uint16_t)rawGyro[1 + (i * 2)]) << 8) | rawGyro[i * 2]);
		g[i] *= gyro_sensitivity;
	}
}

float lsm_temp_read(void)
{
	uint8_t rawTemp[2];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_OUT_TEMP_L, &rawTemp[0], 2);
	if (err)
		LOG_ERR("Communication error");
	// TSen Temperature sensitivity 256 LSB/°C
	// The output of the temperature sensor is 0 LSB (typ.) at 25°C
	float temp = (int16_t)((((uint16_t)rawTemp[1]) << 8) | rawTemp[0]);
	temp /= 256;
	temp += 25;
	return temp;
}

uint8_t lsm_setup_DRDY(uint16_t threshold)
{
	uint8_t buf[2];
	buf[0] = ((threshold >> 8) & 0x03) | (last_gyro_odr > last_accel_odr ? 0x20 : 0x00); // use gyro for BDR if gyro rate is higher // NOTE: using 0x03 for DSV, but DSO allows 0x07
	buf[1] = threshold & 0xFF;
	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_COUNTER_BDR_REG1, buf, 2);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_INT1_CTRL, 0x40); // COUNTER_BDR interrupt
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

uint8_t lsm_setup_WOM(void)
{ // TODO: should be off by the time WOM will be setup
//	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL1, ODR_OFF); // set accel off
//	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL2, ODR_OFF); // set gyro off

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL8, 0xE0 | FS_XL_8G); // set accel FS, set HP_LPF2_XL_BW to lowest bandwidth, enable HP_REF_MODE (set HP_LPF2_XL_BW)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL1, OP_MODE_XL_LP1 << 4 | ODR_240Hz); // set accel low power mode 1, set accel ODR (enable accel)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL9, 0x50); // enable HP_REF_MODE (set HP_REF_MODE_XL and HP_SLOPE_XL_EN)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_TAP_CFG0, 0x10); // set SLOPE_FDS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_WAKE_UP_THS, 0x04); // set threshold, 4 * 7.8125 mg is ~31.25 mg
	k_msleep(11); // need to wait for accel to settle

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNCTIONS_ENABLE, 0x80); // enable interrupts
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MD1_CFG, 0x20); // route wake-up to INT1
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_IF_CFG, 0x18); // INT H_LACTIVE active low, PP_OD open-drain
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

int lsm_ext_setup(void)
{
	// enable internal pull-up for auxiliary I2C
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_IF_CFG, 0x58); // SHUB_PU_EN, INT H_LACTIVE active low, PP_OD open-drain
	if (err)
		LOG_ERR("Communication error");
	sensor_interface_ext_configure(&sensor_ext_lsm6dsv);
	return 0;
}

int lsm_ext_passthrough(bool passthrough)
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
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x00); // passthrough off
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	}
	if (err)
		LOG_ERR("Communication error");
	return 0;
}

int lsm_ext_write(const uint8_t addr, const uint8_t *buf, uint32_t num_bytes)
{
	if (num_bytes != 2)
	{
		LOG_ERR("Unsupported write");
		return -1;
	}
	// Configure transaction and begin one-shot (AN5922, page 80, One-shot write routine)
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	uint8_t slv0[3] = {(addr << 1) | 0x00, buf[0], 0xC0 | 0x00}; // write, SHUB_ODR = 440Hz, reading no bytes
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_ADD, slv0, 3);
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_ADD, (addr << 1) | 0x00); // write
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_SUBADD, buf[0]);
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_CONFIG, 0xC0 | 0x00); // SHUB_ODR = 440Hz, reading no bytes
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_DATAWRITE_SLV0, buf[1]);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x44); // WRITE_ONCE, enable I2C master
	// Wait for transaction
	uint8_t status = 0;
	int64_t timeout = k_uptime_get() + 10;
	while ((status & 0x80) && k_uptime_get() < timeout) // WR_ONCE_DONE
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_STATUS_MASTER, &status);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x00); // disable I2C master
	k_usleep(300);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	if (~status & 0x80)
	{
		LOG_ERR("Write timeout");
		return -1;
	}
	return err;
}

int lsm_ext_write_read(const uint8_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read)
{
	if (num_write != 1 || num_read < 1 || num_read > 8)
	{
		LOG_ERR("Unsupported write_read");
		return -1;
	}
	// Configure transaction and begin one-shot (AN5922, page 79, One-shot read routine)
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	uint8_t slv0[3] = {(addr << 1) | 0x01, ((const uint8_t *)write_buf)[0], 0xC0 | num_read}; // read, SHUB_ODR = 440Hz, reading num_read bytes
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_ADD, slv0, 3);
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_ADD, (addr << 1) | 0x01); // read
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_SUBADD, ((const uint8_t *)write_buf)[0]);
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_CONFIG, 0xC0 | num_read); // SHUB_ODR = 440Hz, reading num_read bytes
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x44); // WRITE_ONCE mandatory for read, enable I2C master
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
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_STATUS_MASTER_MAINPAGE, &status);
	// Read data
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x00); // disable I2C master
	k_usleep(300);
	err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SENSOR_HUB_1, read_buf, num_read);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	return err;
}

const sensor_imu_t sensor_imu_lsm6dsv = {
	*lsm_init,
	*lsm_shutdown,

	*lsm_update_fs,
	*lsm_update_odr,

	*lsm_fifo_read,
	*lsm_fifo_process,
	*lsm_accel_read,
	*lsm_gyro_read,
	*lsm_temp_read,

	*lsm_setup_DRDY,
	*lsm_setup_WOM,
	
	*lsm_ext_setup,
	*lsm_ext_passthrough
};

const sensor_ext_ssi_t sensor_ext_lsm6dsv = {
	*lsm_ext_write,
	*lsm_ext_write_read,
	8
};

#include <math.h>

#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#include "BMI270.h"
#include "BMI270_firmware.h"
#include "sensor/sensor_none.h"

#define PACKET_SIZE 12

static float accel_sensitivity = 16.0f / 32768.0f; // default 16g
static float gyro_sensitivity = 2000.0f / 32768.0f; // default 2000dps

static uint8_t accel_fs = RANGE_16G;
static uint8_t gyro_fs = RANGE_2000;

static uint8_t last_accel_odr = 0xff;
static uint8_t last_gyro_odr = 0xff;
static float factor_zx;

LOG_MODULE_REGISTER(BMI270, LOG_LEVEL_DBG);

static int asic_init(void);
static int upload_config_file(void);
static int factor_zx_read(void);

int bmi270_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	// setup interface for SPI
	sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(10), 1);
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CONF, 0x00); // disable adv_power_save
	k_usleep(450);
	if (asic_init())
		return -1;
	factor_zx = factor_zx_read();
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_ACC_RANGE, accel_fs);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_GYR_RANGE, gyro_fs);
	err |= bmi270_update_odr(accel_time, gyro_time, accel_actual_time, gyro_actual_time);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_FIFO_CONFIG_0, 0x00); // do not return sensortime frame
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_FIFO_CONFIG_1, 0xC0); // enable a+g data in FIFO, don't store header
	if (err)
		LOG_ERR("Communication error");
	return (err < 0 ? err : 0);
}

void bmi270_shutdown(void) // this does not reset the device, to avoid clearing the config
{
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CTRL, 0x00); // disable all sensors
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CONF, 0x01); // enable adv_power_save (suspend)
	if (err)
		LOG_ERR("Communication error");
}

void bmi270_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
	if (accel_range > 8)
	{
		accel_fs = RANGE_16G;
		accel_range = 16;
	}
	else if (accel_range > 4)
	{
		accel_fs = RANGE_8G;
		accel_range = 8;
	}
	else if (accel_range > 2)
	{
		accel_fs = RANGE_4G;
		accel_range = 4;
	}
	else
	{
		accel_fs = RANGE_2G;
		accel_range = 2;
	}

	if (gyro_range > 1000)
	{
		gyro_fs = RANGE_2000;
		gyro_range = 2000;
	}
	else if (gyro_range > 500)
	{
		gyro_fs = RANGE_1000;
		gyro_range = 1000;
	}
	else if (gyro_range > 250)
	{
		gyro_fs = RANGE_500;
		gyro_range = 500;
	}
	else
	{
		gyro_fs = RANGE_250;
		gyro_range = 250;
	}

	accel_sensitivity = accel_range / 32768.0f;
	gyro_sensitivity = gyro_range / 32768.0f;

	*accel_actual_range = accel_range;
	*gyro_actual_range = gyro_range;
}

int bmi270_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	int ODR;
	uint8_t acc_odr;
	uint8_t gyr_odr;

	// Calculate accel
	if (accel_time <= 0 || accel_time == INFINITY) // off, standby interpreted as off
		ODR = 0;
	else
		ODR = 1 / accel_time;

	if (ODR == 0)
	{
		acc_odr = 0; // off
		accel_time = 0; // off
	}
	else if (ODR > 800) // TODO: this is absolutely awful
	{
		acc_odr = ODR_1k6;
		accel_time = 1.0 / 1600;
	}
	else if (ODR > 400)
	{
		acc_odr = ODR_800;
		accel_time = 1.0 / 800;
	}
	else if (ODR > 200)
	{
		acc_odr = ODR_400;
		accel_time = 1.0 / 400;
	}
	else if (ODR > 100)
	{
		acc_odr = ODR_200;
		accel_time = 1.0 / 200;
	}
	else if (ODR > 50)
	{
		acc_odr = ODR_100;
		accel_time = 1.0 / 100;
	}
	else if (ODR > 25)
	{
		acc_odr = ODR_50;
		accel_time = 1.0 / 50;
	}
	else if (ODR > 12.5)
	{
		acc_odr = ODR_25;
		accel_time = 1.0 / 25;
	}
	else if (ODR > 6.25)
	{
		acc_odr = ODR_12p5;
		accel_time = 2.0 / 25;
	}
	else if (ODR > 3.125)
	{
		acc_odr = ODR_6p25;
		accel_time = 4.0 / 25;
	}
	else if (ODR > 1.5625)
	{
		acc_odr = ODR_3p1;
		accel_time = 8.0 / 25;
	}
	else if (ODR > 0.78125)
	{
		acc_odr = ODR_1p5;
		accel_time = 16.0 / 25;
	}
	else
	{
		acc_odr = ODR_0p78;
		accel_time = 32.0 / 25;
	}

	// Calculate gyro
	if (gyro_time <= 0 || gyro_time == INFINITY) // off, standby interpreted as off
		ODR = 0;
	else
		ODR = 1 / gyro_time;

	if (ODR == 0)
	{
		gyr_odr = 0; // off
		gyro_time = 0; // off
	}
	else if (ODR > 1600) // TODO: this is absolutely awful
	{
		gyr_odr = ODR_3k2;
		gyro_time = 1.0 / 3200;
	}
	else if (ODR > 800)
	{
		gyr_odr = ODR_1k6;
		gyro_time = 1.0 / 1600;
	}
	else if (ODR > 400)
	{
		gyr_odr = ODR_800;
		gyro_time = 1.0 / 800;
	}
	else if (ODR > 200)
	{
		gyr_odr = ODR_400;
		gyro_time = 1.0 / 400;
	}
	else if (ODR > 100)
	{
		gyr_odr = ODR_200;
		gyro_time = 1.0 / 200;
	}
	else if (ODR > 50)
	{
		gyr_odr = ODR_100;
		gyro_time = 1.0 / 100;
	}
	else if (ODR > 25)
	{
		gyr_odr = ODR_50;
		gyro_time = 1.0 / 50;
	}
	else
	{
		gyr_odr = ODR_25;
		gyro_time = 1.0 / 25;
	}

	if (last_accel_odr == acc_odr && last_gyro_odr == gyr_odr) // if both were already configured
		return 1;

	last_accel_odr = acc_odr;
	last_gyro_odr = gyr_odr;

	int err = 0;
	if (acc_odr != 0)
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_ACC_CONF, 0xA0 | acc_odr);
	if (gyr_odr != 0)
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_GYR_CONF, 0xE0 | gyr_odr); // set performance opt. noise performance

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CTRL, 0x08 | (acc_odr != 0 ? 0x04 : 0) | (gyr_odr != 0 ? 0x02 : 0)); // enable temp, set accel and gyro power
	if (err)
		LOG_ERR("Communication error");

	*accel_actual_time = accel_time;
	*gyro_actual_time = gyro_time;

	return 0;
}

// TODO: gyro rotation data is delayed for some reason, accelerometer still responds instantly
uint16_t bmi270_fifo_read(uint8_t *data, uint16_t len)
{
	int err = 0;
	uint16_t total = 0;
	uint16_t packets = UINT16_MAX;
	while (packets > 0 && len >= PACKET_SIZE)
	{
		uint8_t rawCount[2];
		err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, BMI270_FIFO_LENGTH_0, &rawCount[0], 2);
		uint16_t count = (uint16_t)((rawCount[1] & 0x3F) << 8 | rawCount[0]); // Turn the 16 bits into a unsigned 16-bit value
		packets = count / PACKET_SIZE;
		uint16_t limit = len / PACKET_SIZE;
		if (packets > limit)
		{
			LOG_WRN("FIFO read buffer limit reached, %d packets dropped", packets - limit);
			packets = limit;
			count = packets * PACKET_SIZE;
		}
		err |= ssi_burst_read_interval(SENSOR_INTERFACE_DEV_IMU, BMI270_FIFO_DATA, data, count, PACKET_SIZE);
		if (err)
			LOG_ERR("Communication error");
		data += packets * PACKET_SIZE;
		len -= packets * PACKET_SIZE;
		total += packets;
	}
	return total;
}

static const uint8_t overread[2] = {0x00, 0x80};
static const uint8_t invalid_accel[6] = {0x01, 0x7F, 0x00, 0x80, 0x00, 0x80};
static const uint8_t invalid_gyro[6] = {0x02, 0x7F, 0x00, 0x80, 0x00, 0x80};

int bmi270_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	index *= PACKET_SIZE;
	if (!memcmp(&data[index], overread, sizeof(overread)))
		return 1; // Skip overread packets
	float a_bmi[3];
	float g_bmi[3];
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a_bmi[i] = (int16_t)((((uint16_t)data[index + 7 + (i * 2)]) << 8) | data[index + 6 + (i * 2)]);
		a_bmi[i] *= accel_sensitivity;
		g_bmi[i] = (int16_t)((((uint16_t)data[index + 1 + (i * 2)]) << 8) | data[index + (i * 2)]);
		g_bmi[i] *= gyro_sensitivity;
	}
	if (memcmp(&data[index + 6], invalid_accel, sizeof(invalid_accel))) // valid accel data
	{
		a[0] = -a_bmi[1];
		a[1] = a_bmi[0];
		a[2] = a_bmi[2];
	}
	if (memcmp(&data[index], invalid_gyro, sizeof(invalid_gyro))) // valid gyro data
	{
		// Ratex = DATA_15<<8+DATA_14 - GYR_CAS.factor_zx * (DATA_19<<8+DATA_18) / 2^9
		g_bmi[0] -= g_bmi[2] * factor_zx;
		g[0] = -g_bmi[1];
		g[1] = g_bmi[0];
		g[2] = g_bmi[2];
	}
	return 0;
}

void bmi270_accel_read(float a[3])
{
	uint8_t rawAccel[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, BMI270_DATA_8, &rawAccel[0], 6);
	if (err)
		LOG_ERR("Communication error");
	float a_bmi[3];
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a_bmi[i] = (int16_t)((((uint16_t)rawAccel[1 + (i * 2)]) << 8) | rawAccel[i * 2]);
		a_bmi[i] *= accel_sensitivity;
	}
	a[0] = -a_bmi[1];
	a[1] = a_bmi[0];
	a[2] = a_bmi[2];
}

void bmi270_gyro_read(float g[3])
{
	uint8_t rawGyro[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, BMI270_DATA_14, &rawGyro[0], 6);
	if (err)
		LOG_ERR("Communication error");
	float g_bmi[3];
	for (int i = 0; i < 3; i++) // x, y, z
	{
		g_bmi[i] = (int16_t)((((uint16_t)rawGyro[1 + (i * 2)]) << 8) | rawGyro[i * 2]);
		g_bmi[i] *= gyro_sensitivity;
	}
	// Ratex = DATA_15<<8+DATA_14 - GYR_CAS.factor_zx * (DATA_19<<8+DATA_18) / 2^9
	g_bmi[0] -= g_bmi[2] * factor_zx;
	g[0] = -g_bmi[1];
	g[1] = g_bmi[0];
	g[2] = g_bmi[2];
}

float bmi270_temp_read(void)
{
	uint8_t rawTemp[2];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, BMI270_TEMPERATURE_0, &rawTemp[0], 2);
	if (err)
		LOG_ERR("Communication error");
	if (rawTemp[0] == 0x00 && rawTemp[1] == 0x80)
		return 23; // TODO: invalid temperature, what to return?
	// 0x0000 -> 23°C
	// The resolution is 1/2^9 K/LSB
	float temp = (int16_t)((((uint16_t)rawTemp[1]) << 8) | rawTemp[0]);
	temp /= 512;
	temp += 23;
	return temp;
}

uint8_t bmi270_setup_DRDY(uint16_t threshold)
{
	uint8_t buf[2];
	buf[0] = threshold & 0xFF;
	buf[1] = (threshold >> 8) & 0x1F;
	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, BMI270_FIFO_WTM_0, buf, 2);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_INT1_MAP_FEAT, 0x00); // disable any_motion_out (interrupt)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_INT1_IO_CTRL, 0x0C); // set INT1 active low, open-drain, output enabled
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_INT_MAP_DATA, 0x02); // FIFO threshold interrupt
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

uint8_t bmi270_setup_WOM(void) // TODO: seems too sensitive? try to match icm at least // TODO: half working.
{
	uint8_t config[4] = {0};
	uint16_t *ptr = (uint16_t *)config; // bmi is little endian
	ptr[0] = 0x7 << 13 | 0x000; // enable all axes, set detection duration to 0
	ptr[1] = 0x1 << 15 | 0x7 << 11 | 0x040; // enable any_motion, set out_conf to bit 6, set threshold (1LSB equals to 0.488mg, 64 * 0.488mg is ~31.25mg)
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CONF, 0x00); // disable adv_power_save
	k_usleep(450);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_ACC_CONF, ODR_200); // disable filters, set accel ODR
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CTRL, 0x04); // enable accel
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_FEAT_PAGE, 0x01); // go to page 1
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, BMI270_ANYMO_1, config, sizeof(config)); // Start write buffer
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_INT_MAP_DATA, 0x00); // disable FIFO threshold interrupt
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_INT1_IO_CTRL, 0x0C); // set INT1 active low, open-drain, output enabled
	k_msleep(55); // wait for sensor to settle
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_INT1_MAP_FEAT, 0x40); // enable any_motion_out (interrupt)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CONF, 0x01); // enable adv_power_save (suspend)
	if (err)
		LOG_ERR("Communication error");
	// LOG_DBG("WOM setup complete");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

static int asic_init(void)
{
	uint8_t status;
	int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_INTERNAL_STATUS, &status);
	if ((status & 0x7) != 0x1) // ASIC is not initialized
	{
		LOG_DBG("ASIC not initialized, uploading config file");
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_CMD, 0xB6); // softreset
		k_msleep(2);
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CONF, 0x00); // disable adv_power_save
		k_usleep(450);
		err |= upload_config_file();
		int retry_count = 0;
		while ((status & 0x7) != 0x1)
		{
			if (retry_count > 100)
			{
				LOG_ERR("Failed to initialize ASIC");
				return -1;
			};
			k_msleep(1);
			err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_INTERNAL_STATUS, &status);
			// LOG_DBG("Status: 0x%02X", status);
			retry_count++;
		}
		LOG_DBG("ASIC initialized");
	}
	if (err)
		LOG_ERR("Communication error");
	return 0;
}

// write_config_file function from https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/sensor/bosch/bmi270/bmi270.c
// saved my ass
static int upload_config_file(void)
{
	uint16_t count = sizeof(bmi270_config_file) / sizeof(bmi270_config_file[0]);
	uint8_t init_addr[2] = {0};
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_INIT_CTRL, 0x00); // prepare config load
	for (int i = 0; i < count; i += 64)
	{
		init_addr[0] = (i / 2) & 0xF;
		init_addr[1] = (i / 2) >> 4;
		err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, BMI270_INIT_ADDR_0, init_addr, 2);
		err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, BMI270_INIT_DATA, &bmi270_config_file[i], 64); // 64 works, 128 doesn't? either way it takes forever
	}
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_INIT_CTRL, 0x01); // complete config load
	if (err)
		LOG_ERR("Communication error");
	// LOG_DBG("Completed config load");
	return 0;
}

static int factor_zx_read(void)
{
	uint8_t data;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_FEAT_PAGE, 0x00); // go to page 0
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_GYR_CAS, &data);
	// GYR_CAS.factor_zx is a 7-bit two-complement encoded signed value
	data <<= 1; // shift to 8-bit
	// Ratex = DATA_15<<8+DATA_14 - GYR_CAS.factor_zx * (DATA_19<<8+DATA_18) / 2^9
	float factor_zx = (int8_t)data / 2 / 512.0;
	if (err)
		LOG_ERR("Communication error");
	return factor_zx;
}

// from https://github.com/SlimeVR/SlimeVR-Tracker-ESP/blob/main/src/sensors/softfusion/drivers/bmi270.h
int bmi270_crt(uint8_t *data)
{
	uint8_t status;
	uint8_t acc_odr = last_accel_odr; // store last odr
	uint8_t gyr_odr = last_gyro_odr; // store last odr
	uint8_t config[2] = {0};
	uint16_t *ptr = (uint16_t *)config; // bmi is little endian
	*ptr = 0x0100; // CRT will be executed
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_CMD, 0xB6); // softreset
	k_msleep(2);
	// in case of SPI, where CS pin must trigger rising edge for BMI to enable interface
	uint8_t tmp;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x00, &tmp);
	k_usleep(200);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CONF, 0x00); // disable adv_power_save
	k_usleep(450);
	if (asic_init())
		return -1;

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_OFFSET_6, 0x80); // gyr_gain_en
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CTRL, 0x04); // enable accel
	k_msleep(2);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_GYR_CRT_CONF, 0x04); // set CRT running
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_FEAT_PAGE, 0x01); // go to page 1
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, BMI270_G_TRIG_1, config, sizeof(config)); // Start write buffer
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_CMD, 0x02); // g_trigger
	do
	{
		k_msleep(200);
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_GYR_CRT_CONF, &status);
	}
	while (status == 0x04);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_FEAT_PAGE, 0x00); // go to page 0
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_GYR_GAIN_STATUS, &status);
	if (status)
	{
		LOG_INF("Execution status: 0x%02X", status);
		if (status == 0x03)
			LOG_INF("Motion detected during CRT");
		data[0] = 0; // invalid
	}
	else
	{
		err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, BMI270_GYR_USR_GAIN, data + 1, 3);
		LOG_INF("Gain: 0x%02X, 0x%02X, 0x%02X", data[1], data[2], data[3]);
		data[0] = 1; // flag for valid gain
	}

	// CRT seems to leave some state behind which isn't persisted after
	// restart. If we continue without restarting, the gyroscope will behave
	// differently on this run compared to subsequent restarts.
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_CMD, 0xB6); // softreset
	k_msleep(2);
	// in case of SPI, where CS pin must trigger rising edge for BMI to enable interface
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x00, &tmp);
	k_usleep(200);
	bmi270_init(0, 0, 0, 0, 0);
	if (acc_odr != 0)
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_ACC_CONF, 0xA0 | acc_odr);
	if (gyr_odr != 0)
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_GYR_CONF, 0xE0 | gyr_odr); // set performance opt. noise performance
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_PWR_CTRL, 0x08 | (acc_odr != 0 ? 0x04 : 0) | (gyr_odr != 0 ? 0x02 : 0)); // enable temp, set accel and gyro power
	last_accel_odr = acc_odr;
	last_gyro_odr = gyr_odr;

	if (err)
		LOG_ERR("Communication error");

	if (data[0] == 0)
		return 1;
	return 0;
}

void bmi270_gain_apply(uint8_t *data)
{
	if (data[0] == 1) // flag for valid gain
	{
		int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, BMI270_OFFSET_6, 0x80); // gyr_gain_en
		err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, BMI270_GYR_USR_GAIN, data + 1, 3);
	}
}

const sensor_imu_t sensor_imu_bmi270 = {
	*bmi270_init,
	*bmi270_shutdown,

	*bmi270_update_fs,
	*bmi270_update_odr,

	*bmi270_fifo_read,
	*bmi270_fifo_process,
	*bmi270_accel_read,
	*bmi270_gyro_read,
	*bmi270_temp_read,

	*bmi270_setup_DRDY,
	*bmi270_setup_WOM,
	
	*imu_none_ext_setup,
	*imu_none_ext_passthrough
};

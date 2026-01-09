#include <math.h>

#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#include "ICM45686.h"
#include "sensor/sensor_none.h"

#define PACKET_SIZE 20

static const float accel_sensitivity = 16.0f / 32768.0f; // Always 16G
static const float gyro_sensitivity = 2000.0f / 32768.0f; // Always 2000dps

static const float accel_sensitivity_32 = 32.0f / ((uint32_t)2<<30); // 32G forced
static const float gyro_sensitivity_32 = 4000.0f / ((uint32_t)2<<30); // 4000dps forced

static uint8_t last_accel_odr = 0xff;
static uint8_t last_gyro_odr = 0xff;
static const float clock_reference = 32000;
static float clock_scale = 1; // ODR is scaled by clock_rate/clock_reference

#define FIFO_MULT 0.00075f // assuming i2c fast mode
#define FIFO_MULT_SPI 0.0001f // ~24MHz

static float fifo_multiplier_factor = FIFO_MULT;
static float fifo_multiplier = 0;

LOG_MODULE_REGISTER(ICM45686, LOG_LEVEL_DBG);

int icm45_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	// setup interface for SPI
	if (!sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(24), 0))
		fifo_multiplier_factor = FIFO_MULT_SPI; // SPI mode
	else
		fifo_multiplier_factor = FIFO_MULT; // I2C mode
	int err = 0;
	if (clock_rate > 0)
	{
		clock_scale = clock_rate / clock_reference;
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_OVRD, 0x06); // override pin 9 to CLKIN
		err |= ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_RTC_CONFIG, 0x20, 0x20); // enable external CLKIN
//		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_RTC_CONFIG, 0x23); // enable external CLKIN (0x20, default register value is 0x03)
	}
	uint8_t ireg_buf[3];
	ireg_buf[0] = ICM45686_IPREG_BAR; // address is a word, icm is big endian
	ireg_buf[1] = ICM45686_IPREG_BAR_REG_58;
	ireg_buf[2] = 0xD9 & ~0x48; // disable internal pull resistors for AP pins (pin 13, 12)
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
	ireg_buf[1] = ICM45686_IPREG_BAR_REG_59;
	ireg_buf[2] = 0xB6 & ~0x92; // disable internal pull resistors for AP pins (pin 7, 1, 14)
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
	ireg_buf[0] = ICM45686_IPREG_TOP1; // address is a word, icm is big endian
	ireg_buf[1] = ICM45686_SREG_CTRL;
	ireg_buf[2] = 0x02; // set big endian
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	err |= icm45_update_odr(accel_time, gyro_time, accel_actual_time, gyro_actual_time);
//	k_msleep(50); // 10ms Accel, 30ms Gyro startup
	k_msleep(1); // fuck i dont wanna wait that long
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG0, 0x80 | 0b000111); // set FIFO stop-on-full mode, set FIFO depth to 2K bytes (see AN-000364)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG3, 0x0F); // begin FIFO stream, hires, a+g
	if (err)
		LOG_ERR("Communication error");
	return (err < 0 ? err : 0);
}

void icm45_shutdown(void)
{
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_REG_MISC2, 0x02); // Don't need to wait for ICM to finish reset
//	k_msleep(1);
	// TODO: not working
//	uint8_t ireg_buf[3];
//	ireg_buf[1] = ICM45686_IPREG_BAR_REG_60;
//	ireg_buf[2] = 0x6D & ~0x05; // set internal pull down resistors for AP pins (pin 10, 7)
//	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
//	ireg_buf[1] = ICM45686_IPREG_BAR_REG_61;
//	ireg_buf[2] = 0xBB & ~0x10; // set internal pull down resistors for AP pins (pin 11)
//	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
	if (err)
		LOG_ERR("Communication error");
}

void icm45_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
	*accel_actual_range = 32; // always 32g in hires
	*gyro_actual_range = 4000; // always 4000dps in hires
}

int icm45_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	int ODR;
	uint8_t ACCEL_UI_FS_SEL = ACCEL_UI_FS_SEL_16G;
	uint8_t GYRO_UI_FS_SEL = GYRO_UI_FS_SEL_2000DPS;
	uint8_t ACCEL_MODE;
	uint8_t GYRO_MODE;
	uint8_t ACCEL_ODR;
	uint8_t GYRO_ODR;

	// Calculate accel
	if (accel_time <= 0 || accel_time == INFINITY) // off, standby interpreted as off
	{
		ACCEL_MODE = ACCEL_MODE_OFF;
		ODR = 0;
	}
	else
	{
		ACCEL_MODE = ACCEL_MODE_LN;
		ODR = 1 / accel_time;
		ODR /= clock_scale; // scale clock
	}

	if (ACCEL_MODE != ACCEL_MODE_LN)
	{
		ACCEL_ODR = 0;
		accel_time = 0; // off
	}
	else if (ODR > 3200) // TODO: this is absolutely awful
	{
		ACCEL_ODR = ACCEL_ODR_6_4kHz;
		accel_time = 1.0 / 6400;
	}
	else if (ODR > 1600)
	{
		ACCEL_ODR = ACCEL_ODR_3_2kHz;
		accel_time = 1.0 / 3200;
	}
	else if (ODR > 800)
	{
		ACCEL_ODR = ACCEL_ODR_1_6kHz;
		accel_time = 1.0 / 1600;
	}
	else if (ODR > 400)
	{
		ACCEL_ODR = ACCEL_ODR_800Hz;
		accel_time = 1.0 / 800;
	}
	else if (ODR > 200)
	{
		ACCEL_ODR = ACCEL_ODR_400Hz;
		accel_time = 1.0 / 400;
	}
	else if (ODR > 100)
	{
		ACCEL_ODR = ACCEL_ODR_200Hz;
		accel_time = 1.0 / 200;
	}
	else if (ODR > 50)
	{
		ACCEL_ODR = ACCEL_ODR_100Hz;
		accel_time = 1.0 / 100;
	}
	else if (ODR > 25)
	{
		ACCEL_ODR = ACCEL_ODR_50Hz;
		accel_time = 1.0 / 50;
	}
	else if (ODR > 12.5)
	{
		ACCEL_ODR = ACCEL_ODR_25Hz;
		accel_time = 1.0 / 25;
	}
	else
	{
		ACCEL_ODR = ACCEL_ODR_12_5Hz;
		accel_time = 1.0 / 12.5;
	}
	accel_time /= clock_scale; // scale clock

	// Calculate gyro
	if (gyro_time <= 0) // off
	{
		GYRO_MODE = GYRO_MODE_OFF;
		ODR = 0;
	}
	else if (gyro_time == INFINITY) // standby
	{
		GYRO_MODE = GYRO_MODE_STANDBY;
		ODR = 0;
	}
	else
	{
		GYRO_MODE = GYRO_MODE_LN;
		ODR = 1 / gyro_time;
		ODR /= clock_scale; // scale clock
	}

	if (GYRO_MODE != GYRO_MODE_LN)
	{
		GYRO_ODR = 0;
		gyro_time = 0; // off
	}
	else if (ODR > 3200) // TODO: this is absolutely awful
	{
		GYRO_ODR = GYRO_ODR_6_4kHz;
		gyro_time = 1.0 / 6400;
	}
	else if (ODR > 1600)
	{
		GYRO_ODR = GYRO_ODR_3_2kHz;
		gyro_time = 1.0 / 3200;
	}
	else if (ODR > 800)
	{
		GYRO_ODR = GYRO_ODR_1_6kHz;
		gyro_time = 1.0 / 1600;
	}
	else if (ODR > 400)
	{
		GYRO_ODR = GYRO_ODR_800Hz;
		gyro_time = 1.0 / 800;
	}
	else if (ODR > 200)
	{
		GYRO_ODR = GYRO_ODR_400Hz;
		gyro_time = 1.0 / 400;
	}
	else if (ODR > 100)
	{
		GYRO_ODR = GYRO_ODR_200Hz;
		gyro_time = 1.0 / 200;
	}
	else if (ODR > 50)
	{
		GYRO_ODR = GYRO_ODR_100Hz;
		gyro_time = 1.0 / 100;
	}
	else if (ODR > 25)
	{
		GYRO_ODR = GYRO_ODR_50Hz;
		gyro_time = 1.0 / 50;
	}
	else if (ODR > 12.5)
	{
		GYRO_ODR = GYRO_ODR_25Hz;
		gyro_time = 1.0 / 25;
	}
	else
	{
		GYRO_ODR = GYRO_ODR_12_5Hz;
		gyro_time = 1.0 / 12.5;
	}
	gyro_time /= clock_scale; // scale clock

	if (last_accel_odr == ACCEL_ODR && last_gyro_odr == GYRO_ODR) // if both were already configured
		return 1;

	int err = 0;
	// only if the power mode has changed
	if (last_accel_odr == 0xff || last_gyro_odr == 0xff || (last_accel_odr == 0 ? 0 : 1) != (ACCEL_ODR == 0 ? 0 : 1) || (last_gyro_odr == 0 ? 0 : 1) != (GYRO_ODR == 0 ? 0 : 1))
	{ // TODO: can't tell difference between gyro off and gyro standby
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_PWR_MGMT0, GYRO_MODE << 2 | ACCEL_MODE); // set accel and gyro modes
		k_busy_wait(250); // wait >200us // TODO: is this needed?
	}
	last_accel_odr = ACCEL_ODR;
	last_gyro_odr = GYRO_ODR;

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_ACCEL_CONFIG0, ACCEL_UI_FS_SEL << 4 | ACCEL_ODR); // set accel ODR and FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_GYRO_CONFIG0, GYRO_UI_FS_SEL << 4 | GYRO_ODR); // set gyro ODR and FS
	if (err)
		LOG_ERR("Communication error");

	*accel_actual_time = accel_time;
	*gyro_actual_time = gyro_time;

	// extra read packets by ODR time
	if (accel_time == 0 && gyro_time != 0)
		fifo_multiplier = fifo_multiplier_factor / gyro_time; 
	else if (accel_time != 0 && gyro_time == 0)
		fifo_multiplier = fifo_multiplier_factor / accel_time;
	else if (gyro_time > accel_time)
		fifo_multiplier = fifo_multiplier_factor / accel_time;
	else if (accel_time > gyro_time)
		fifo_multiplier = fifo_multiplier_factor / gyro_time;
	else
		fifo_multiplier = 0;

	return 0;
}

uint16_t icm45_fifo_read(uint8_t *data, uint16_t len) // TODO: check if working
{
	int err = 0;
	uint16_t total = 0;
	uint16_t packets = UINT16_MAX;
	while (packets > 0 && len >= PACKET_SIZE)
	{
		uint8_t rawCount[2];
		err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_COUNT_0, &rawCount[0], 2);
		packets = (uint16_t)(rawCount[0] << 8 | rawCount[1]); // Turn the 16 bits into a unsigned 16-bit value
		float extra_read_packets = packets * fifo_multiplier;
		packets += extra_read_packets;
		uint16_t count = packets * PACKET_SIZE;
		uint16_t limit = len / PACKET_SIZE;
		if (packets > limit)
		{
			LOG_WRN("FIFO read buffer limit reached, %d packets dropped", packets - limit);
			packets = limit;
			count = packets * PACKET_SIZE;
		}
		err |= ssi_burst_read_interval(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_DATA, data, count, PACKET_SIZE);
		if (err)
			LOG_ERR("Communication error");
		data += packets * PACKET_SIZE;
		len -= packets * PACKET_SIZE;
		total += packets;
	}
	return total;
}

static const uint8_t invalid[6] = {0x80, 0x00, 0x80, 0x00, 0x80, 0x00};

int icm45_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	index *= PACKET_SIZE;
	if (data[index] != 0x78) // ACCEL_EN, GYRO_EN, HIRES_EN, TMST_FIELD_EN
		return 1; // Skip invalid header
	// Empty packet is 7F filled
	// combine into 20 bit values in 32 bit int
	float a_raw[3] = {0};
	float g_raw[3] = {0};
	if (memcmp(&data[index + 1], invalid, sizeof(invalid))) // valid accel data
	{
		for (int i = 0; i < 3; i++) // accel x, y, z
			a_raw[i] = (int32_t)((((uint32_t)data[index + 1 + (i * 2)]) << 24) | (((uint32_t)data[index + 2 + (i * 2)]) << 16) | (((uint32_t)data[index + 17 + i] & 0xF0) << 8));
	}
	if (memcmp(&data[index + 7], invalid, sizeof(invalid))) // valid gyro data
	{
		for (int i = 0; i < 3; i++) // gyro x, y, z
			g_raw[i] = (int32_t)((((uint32_t)data[index + 7 + (i * 2)]) << 24) | (((uint32_t)data[index + 8 + (i * 2)]) << 16) | (((uint32_t)data[index + 17 + i] & 0x0F) << 12));
	}
	else if (!memcmp(&data[index + 1], invalid, sizeof(invalid))) // Skip invalid data
	{
		return 1;
	}
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a_raw[i] *= accel_sensitivity_32;
		g_raw[i] *= gyro_sensitivity_32;
	}
	memcpy(a, a_raw, sizeof(a_raw));
	memcpy(g, g_raw, sizeof(g_raw));
	return 0;
}

void icm45_accel_read(float a[3])
{
	uint8_t rawAccel[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_ACCEL_DATA_X1_UI, &rawAccel[0], 6);
	if (err)
		LOG_ERR("Communication error");
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a[i] = (int16_t)((((uint16_t)rawAccel[i * 2]) << 8) | rawAccel[1 + (i * 2)]);
		a[i] *= accel_sensitivity;
	}
}

void icm45_gyro_read(float g[3])
{
	uint8_t rawGyro[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_GYRO_DATA_X1_UI, &rawGyro[0], 6);
	if (err)
		LOG_ERR("Communication error");
	for (int i = 0; i < 3; i++) // x, y, z
	{
		g[i] = (int16_t)((((uint16_t)rawGyro[i * 2]) << 8) | rawGyro[1 + (i * 2)]);
		g[i] *= gyro_sensitivity;
	}
}

float icm45_temp_read(void)
{
	uint8_t rawTemp[2];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_TEMP_DATA1_UI, &rawTemp[0], 2);
	if (err)
		LOG_ERR("Communication error");
	// Temperature in Degrees Centigrade = (TEMP_DATA / 128) + 25
	float temp = (int16_t)((((uint16_t)rawTemp[0]) << 8) | rawTemp[1]);
	temp /= 128;
	temp += 25;
	return temp;
}

uint8_t icm45_setup_DRDY(uint16_t threshold)
{
	uint8_t buf[2];
	buf[0] = threshold & 0xFF;
	buf[1] = threshold >> 8;
	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG1_0, buf, 2);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_CONFIG0, 0x02); // FIFO threshold interrupt
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

uint8_t icm45_setup_WOM(void) // TODO: check if working
{
	uint8_t interrupts;
	uint8_t ireg_buf[5];
	int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_STATUS0, &interrupts); // clear reset done int flag // TODO: is this needed
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_CONFIG0, 0x00); // disable default interrupt (RESET_DONE)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_ACCEL_CONFIG0, ACCEL_UI_FS_SEL_8G << 4 | ACCEL_ODR_200Hz); // set accel ODR and FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_PWR_MGMT0, ACCEL_MODE_LP); // set accel and gyro modes
	ireg_buf[0] = ICM45686_IPREG_SYS2; // address is a word, icm is big endian
	ireg_buf[1] = ICM45686_IPREG_SYS2_REG_129;
	ireg_buf[2] = 0x00; // set ACCEL_LP_AVG_SEL to 1x
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
	// should already be defaulted to AULP
//	ireg_buf[0] = ICM45686_IPREG_TOP1;
//	ireg_buf[1] = ICM45686_SMC_CONTROL_0;
//	ireg_buf[2] = 0x60; // set ACCEL_LP_CLK_SEL to AULP
//	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
	ireg_buf[0] = ICM45686_IPREG_TOP1;
	ireg_buf[1] = ICM45686_ACCEL_WOM_X_THR;
	ireg_buf[2] = 0x08; // set wake thresholds // 8 x 3.9 mg is ~31.25 mg
	ireg_buf[3] = 0x08; // set wake thresholds
	ireg_buf[4] = 0x08; // set wake thresholds
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 5); // write buffer
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_TMST_WOM_CONFIG, 0x14); // enable WOM, enable WOM interrupt
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_CONFIG1, 0x0E); // route WOM interrupt
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

int icm45_ext_passthrough(bool passthrough)
{
	int err = 0;
	if (passthrough)
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_AUX_OVRD, 0x18); // AUX1_MODE_OVRD, AUX1 in I2CM Bypass
	else
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_AUX_OVRD, 0x00); // disable overrides
	if (err)
		LOG_ERR("Communication error");
	return 0;
}

int icm45_ext_setup() {
	int err = 0;
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_AUX_OVRD, 0x17); // AUX1_MODE_OVRD, AUX1 in I2CM Master, AUX1_ENABLE_OVRD, AUX1 enabled
	sensor_interface_ext_configure(&sensor_ext_icm45686);
	return err;
}

int icm45_bank_write(const uint8_t bank, const uint8_t reg, const uint8_t *buf, uint32_t num_bytes)
{
	if (num_bytes == 0)
	{
		LOG_ERR("Invalid bank write size");
		return -1;
	}

	int err = 0;
	uint8_t ireg_buf[3];
	ireg_buf[0] = bank;
	ireg_buf[1] = reg;
	ireg_buf[2] = buf[0];
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3);
	k_usleep(4);

	for (uint32_t i = 1; i < num_bytes; i++)
	{
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_DATA, buf[i]);
		k_usleep(4);
	}

	return err;
}

int icm45_bank_write_byte(const uint8_t bank, const uint8_t reg, const uint8_t value)
{
	return icm45_bank_write(bank, reg, &value, 1);
}

int icm45_bank_read(const uint8_t bank, const uint8_t reg, uint8_t *buf, uint32_t num_bytes)
{
	if (num_bytes == 0)
	{
		LOG_ERR("Invalid bank read size");
		return -1;
	}

	int err = 0;
	uint8_t ireg_buf[2];
	ireg_buf[0] = bank;
	ireg_buf[1] = reg;
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 2);
	k_usleep(4);

	for (uint32_t i = 0; i < num_bytes; i++)
	{
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_DATA, &buf[i]);
		k_usleep(4);
	}
	return err;
}

int icm45_bank_read_byte(const uint8_t bank, const uint8_t reg, uint8_t *value) {
	return icm45_bank_read(bank, reg, value, 1);
}

int icm45_ext_write(const uint8_t addr, const uint8_t *buf, uint32_t num_bytes) 
{
	if (num_bytes > 6) 
	{
		LOG_ERR("Unsupported write");
		return -1;
	}

	int err = 0;

	err |= icm45_bank_write(ICM45686_IPREG_TOP1, ICM45686_I2CM_WR_DATA_0, buf, num_bytes);
	err |= icm45_bank_write_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_COMMAND_0, 0x80 + num_bytes); // Last transaction, channel 0, write num_bytes bytes
	err |= icm45_bank_write_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_CONTROL, 0x01); // No restarts, fast mode, start transaction

	uint8_t last_status = 0;
	err |= icm45_bank_read_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_STATUS, &last_status);
	while (last_status & 0x01) // I2CM busy
	{
		err |= icm45_bank_read_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_STATUS, &last_status);
	}

	if (last_status != 0x02) // Not (just) "done"
	{
		LOG_ERR("I2CM error: %02x", last_status);
		return -1;
	}

	uint8_t dev_status;
	err |= icm45_bank_read_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_EXT_DEV_STATUS, &dev_status);

	if (dev_status & 0x01) {
		// Maybe log the nack?
		return -1;
	}

	return err;
}

int icm45_ext_write_read(const uint8_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read) {
	if (num_write != 1 || num_read < 1 || num_read > 15)
	{
		LOG_ERR("Unsupported write_read");
		return -1;
	}

	int err = 0;

	uint8_t dev_profile_data[2] = {((const uint8_t *)write_buf)[0], addr};
	err |= icm45_bank_write(ICM45686_IPREG_TOP1, ICM45686_DEV_PROFILE_0, dev_profile_data, 2);
	err |= icm45_bank_write_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_COMMAND_0, 0x90 + num_read); // Last transaction, channel 0, read num_read bytes with register specified
	err |= icm45_bank_write_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_CONTROL, 0x01); // No restarts, fast mode, start transaction

	uint8_t last_status = 0;
	err |= icm45_bank_read_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_STATUS, &last_status);
	while (last_status & 0x01) // I2CM busy
	{
		err |= icm45_bank_read_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_STATUS, &last_status);
	}

	if (last_status != 0x02) // Not (just) "done"
	{
		LOG_ERR("I2CM error: %02x", last_status);
		return -1;
	}

	uint8_t dev_status;
	err |= icm45_bank_read_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_EXT_DEV_STATUS, &dev_status);

	if (dev_status & 0x01) {
		// Maybe log the nack?
		return -1;
	}

	err |= icm45_bank_read(ICM45686_IPREG_TOP1, ICM45686_I2CM_RD_DATA_0, read_buf, num_read);

	return err;
}

const sensor_imu_t sensor_imu_icm45686 = {
	*icm45_init,
	*icm45_shutdown,

	*icm45_update_fs,
	*icm45_update_odr,

	*icm45_fifo_read,
	*icm45_fifo_process,
	*icm45_accel_read,
	*icm45_gyro_read,
	*icm45_temp_read,

	*icm45_setup_DRDY,
	*icm45_setup_WOM,

	*icm45_ext_setup,
	*icm45_ext_passthrough
};

const sensor_ext_ssi_t sensor_ext_icm45686 = {
	*icm45_ext_write,
	*icm45_ext_write_read,
	15
};

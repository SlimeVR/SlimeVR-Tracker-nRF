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

static const uint16_t intervals[] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 0};
static const uint8_t odrs[] = {ACCEL_ODR_6_4kHz, ACCEL_ODR_3_2kHz, ACCEL_ODR_1_6kHz, ACCEL_ODR_800Hz, ACCEL_ODR_400Hz, ACCEL_ODR_200Hz, ACCEL_ODR_100Hz, ACCEL_ODR_50Hz, ACCEL_ODR_25Hz, ACCEL_ODR_12_5Hz};

static uint8_t last_accel_odr = 0xff;
static uint8_t last_gyro_odr = 0xff;
static const float clock_reference = 32000;
static float clock_scale = 1; // ODR is scaled by clock_rate/clock_reference

#define FIFO_MULT 0.00075f // assuming i2c fast mode
#define FIFO_MULT_SPI 0.0001f // ~24MHz

static float fifo_multiplier_factor = FIFO_MULT;
static float fifo_multiplier = 0;
static uint8_t ext_mode = SENSOR_EXT_MODE_OFF;
static uint8_t ext_addr = 0;
static uint8_t ext_reg = 0;
static const sensor_mag_t *ext_mag = NULL;

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
	k_usleep(4); // Wait 4uS after writing IREG, as per datasheet
	ireg_buf[1] = ICM45686_IPREG_BAR_REG_59;
	ireg_buf[2] = 0xB6 & ~0x92; // disable internal pull resistors for AP pins (pin 7, 1, 14)
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
	k_usleep(4); // Wait 4uS after writing IREG, as per datasheet
	ireg_buf[0] = ICM45686_IPREG_TOP1; // address is a word, icm is big endian
	ireg_buf[1] = ICM45686_SREG_CTRL;
	ireg_buf[2] = 0x02; // set big endian
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
	k_usleep(4); // Wait 4uS after writing IREG, as per datasheet
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	err |= icm45_update_odr(accel_time, gyro_time, accel_actual_time, gyro_actual_time);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG0, 0x80 | 0b000111); // set FIFO stop-on-full mode, set FIFO depth to 2K bytes (see AN-000364)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG3, 0x0F); // begin FIFO stream, hires, a+g

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_CONFIG2, INT1_MODE); // latch mode, push pull

	if (err)
		LOG_ERR("Communication error");
	return (err < 0 ? err : 0);
}

void icm45_shutdown(void)
{
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_REG_MISC2, 0x02);
	ext_mode = SENSOR_EXT_MODE_OFF;
	ext_addr = 0;
	ext_reg = 0;

//	uint8_t ireg_buf[3];
//	ireg_buf[1] = ICM45686_IPREG_BAR_REG_60;
//	ireg_buf[2] = 0x6D & ~0x05; // set internal pull down resistors for AP pins (pin 10, 7)
//	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
//	ireg_buf[1] = ICM45686_IPREG_BAR_REG_61;
//	ireg_buf[2] = 0xBB & ~0x10; // set internal pull down resistors for AP pins (pin 11)
//	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, ICM45686_IREG_ADDR_15_8, ireg_buf, 3); // write buffer
	// Wait to finish reset
	uint8_t rst_state;
	while(true) {
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_STATUS0, &rst_state);
		if (err) {
			LOG_ERR("Communication error when reading reset state");
			break;
		}
		if((rst_state & 0x80) != 0x80) {
			k_usleep(10);
			LOG_DBG("IMU reset is pending (0x%02x), waiting...", rst_state);
		} else {
			break;
		}
	}
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
	int interval;
	uint8_t ACCEL_UI_FS_SEL = ACCEL_UI_FS_SEL_16G;
	uint8_t GYRO_UI_FS_SEL = GYRO_UI_FS_SEL_2000DPS;
	uint8_t ACCEL_MODE;
	uint8_t GYRO_MODE;
	uint8_t ACCEL_ODR = 0;
	uint8_t GYRO_ODR = 0;

	// Calculate accel
	if (accel_time <= 0 || accel_time == INFINITY) // off, standby interpreted as off
	{
		ACCEL_MODE = ACCEL_MODE_OFF;
		accel_time = 0; // off
	}
	else
	{
		ACCEL_MODE = ACCEL_MODE_LN;
		interval = accel_time * clock_scale * 6400; // scale clock
		for (int i = 1; i < ARRAY_SIZE(intervals); i++)
		{
			if (intervals[i] && interval >= intervals[i])
				continue;
			ACCEL_ODR = odrs[i - 1];
			accel_time = intervals[i - 1] / 6400.0f;
			break;
		}
	}
	accel_time /= clock_scale; // scale clock

	// Calculate gyro
	if (gyro_time <= 0) // off
	{
		GYRO_MODE = GYRO_MODE_OFF;
		gyro_time = 0; // off
	}
	else if (gyro_time == INFINITY) // standby
	{
		GYRO_MODE = GYRO_MODE_STANDBY;
		gyro_time = 0; // off
	}
	else
	{
		GYRO_MODE = GYRO_MODE_LN;
		interval = gyro_time * clock_scale * 6400; // scale clock
		for (int i = 1; i < ARRAY_SIZE(intervals); i++)
		{
			if (intervals[i] && interval >= intervals[i])
				continue;
			GYRO_ODR = odrs[i - 1];
			gyro_time = intervals[i - 1] / 6400.0f;
			break;
		}
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

int icm45_temp_read(float *data)
{
	uint8_t rawTemp[2];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_TEMP_DATA1_UI, &rawTemp[0], 2);
	if (err)
	{
		LOG_ERR("Communication error");
		return -1;
	}
	// Temperature in Degrees Centigrade = (TEMP_DATA / 128) + 25
	*data = (int16_t)((((uint16_t)rawTemp[0]) << 8) | rawTemp[1]);
	*data /= 128;
	*data += 25;
	return 0;
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
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_CONFIG1, INT1_STATUS_WOM_X | INT1_STATUS_WOM_Y | INT1_STATUS_WOM_Z);
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
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

int icm45_ext_addr_set(const uint8_t addr, const uint8_t reg) {
	int err = 0;
	if (ext_addr != addr) { // aux address changed, rewrite both address and register
		uint8_t dev_profile_data[2] = {reg, addr};
		err = icm45_bank_write(ICM45686_IPREG_TOP1, ICM45686_DEV_PROFILE_0, dev_profile_data, sizeof(dev_profile_data));
		if (!err) {
			ext_addr = addr;
			ext_reg = reg;
		}
	}
	else if (ext_reg != reg) { // only register changed, aux address remains
		err = icm45_bank_write_byte(ICM45686_IPREG_TOP1, ICM45686_DEV_PROFILE_0, reg);
		if (!err) {
			ext_reg = reg;
		}
	}

	return err;
}

int icm45_ext_write(const uint8_t addr, const uint8_t *buf, uint32_t num_bytes) 
{
	if (ext_mode != SENSOR_EXT_MODE_I2CM_PROXY) {
		LOG_ERR("Sensor not in correct mode to perform ext write: %d", ext_mode);
	}

	if (num_bytes > 6) 
	{
		LOG_ERR("Unsupported write");
		return -1;
	}

	int err = icm45_ext_addr_set(addr, ext_reg);

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

int icm45_ext_write_read(const uint8_t addr, const uint8_t *write_buf, size_t num_write, uint8_t *read_buf, size_t num_read) {
	if (num_write != 1 || num_read < 1 || num_read > 15)
	{
		LOG_ERR("Unsupported write_read");
		return -1;
	}

	if (ext_mode != SENSOR_EXT_MODE_I2CM_PROXY) {
		LOG_ERR("Sensor not in correct mode to perform ext write read: %d", ext_mode);
	}

	int err = icm45_ext_addr_set(addr, write_buf[0]);

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

int icm45_ext_setup(sensor_ext_mode_t mode, const sensor_mag_t *mag, uint8_t mag_addr) {
	int err = 0;
	ext_mag = NULL; // only autonomous mode will set this 

	switch (mode) {
		case SENSOR_EXT_MODE_OFF:
			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_AUX_OVRD, 0x01); // override AUX1_ENABLE=0
			break;

		case SENSOR_EXT_MODE_I2C_PASSTHROUGH:
			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_AUX_OVRD, 0x1B); // AUX1_ENABLE=1 AUX1_MODE=I2CM Bypass
			break;
			
		case SENSOR_EXT_MODE_I2CM_PROXY:
			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_AUX_OVRD, 0x17); // AUX1_ENABLE=1 AUX1_MODE=I2CM
			if (!err) {
				sensor_interface_ext_configure(&sensor_ext_icm45686);
			}
			break;
		
		case SENSOR_EXT_MODE_I2CM_AUTONOMOUS:
			ext_mag = mag;
			const uint8_t total_bytes = mag->ext_burst + mag->ext_dummy_bytes;
			if (total_bytes > 9) {
				return -1; // not supported (for now)
			}

			// we're going to do periodic burst read of length defined by mag starting from ext_burst_reg
			// not asking IMU to put that data info FIFO, as we would lose hi-res mode

			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_IOC_PAD_SCENARIO_AUX_OVRD, 0x17); // AUX1_ENABLE=1 AUX1_MODE=I2CM
			err |= icm45_ext_addr_set(mag_addr, mag->ext_burst_reg);
			// prepare transaction that will be repeated periodically
			err |= icm45_bank_write_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_COMMAND_0, 0x90 | total_bytes); // read operation with register address, ch 0, endflag_0=1
			err |= icm45_bank_write_byte(ICM45686_IPREG_TOP1, ICM45686_I2CM_CONTROL, 0x40); // allow restarts, fast mode

			// the following lines would put data in fifo, resulting in lost of hi-res mode. Also, fifo read function is not prepared for that
			//err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG4, (total_bytes > 6 ? 1 : 0)); // set FIFO_ES0_6B_9B
			//err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_CONFIG3, 0x1f); // everything like on _init, with FIFO_ES0_EN=1

			const float ext_intervals[] = {1.0f/3.125f, 1.0f/6.25f, 1.0f/12.5f, 1.0f/25, 1.0f/50, 1.0f/100, 1.0f/200, 1.0f/400};
			const uint8_t ext_odr[] = {EXT_ODR_3_125Hz, EXT_ODR_6_25Hz, EXT_ODR_12_5Hz, EXT_ODR_25Hz, EXT_ODR_50Hz, EXT_ODR_100Hz, EXT_ODR_200Hz, EXT_ODR_400Hz};

			// select odr that is just higher or equal to mag odr, we might consider higher value
			const float mag_odr = mag->get_odr();
			size_t sel_odr;

			for (sel_odr = 0; sel_odr<sizeof(ext_intervals)/sizeof(ext_intervals[0]); sel_odr++)
			{
				if (ext_intervals[sel_odr] <= mag_odr) break;
			}

			// this is apparently needed https://github.com/tdk-invn-oss/motion.arduino.ICM45686/blob/main/src/ICM45686.cpp#L859
			err |= icm45_bank_write_byte(ICM45686_IPREG_TOP1, ICM45686_INT_I2CM_SOURCE, INT_STATUS_I2CM_SMC_EXT_ODR_EN);

			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_DMP_EXT_SEN_ODR_CFG, EXT_SENSOR_EN | (ext_odr[sel_odr] << EXT_ODR_OFFSET));
			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_CONFIG1, INT1_STATUS_I2CM_DONE);

			break;
	}

	if (err) {
		LOG_ERR("Communication error: %d", err);
	}
	else {
		ext_mode = mode;
	}

	return err;
}

uint16_t icm45_data_read(uint8_t *data, uint16_t len)
{
	int err = 0;
	uint16_t total = 0;
	uint16_t packets = UINT16_MAX;

	uint8_t int1_status[2] = {0};
	err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_STATUS0, int1_status, 2);

	if (ext_mag)
	{
		// we have aux mag working in autonomous mode
		if (int1_status[1] & INT1_STATUS_I2CM_DONE)
		{
			// i2c master finished transfer, that means we have data for mag
			if (len >= PACKET_SIZE) {
				data[0] = 0x70; // magic tag for mag entry
				// put data as fifo entry
				err |= icm45_bank_read(ICM45686_IPREG_TOP1, ICM45686_I2CM_RD_DATA_0 + ext_mag->ext_dummy_bytes, &data[1], ext_mag->ext_burst);

				data += PACKET_SIZE;
				len -= PACKET_SIZE; 
				total++;
			}
			else {
				LOG_WRN("Cannot read mag data, as buffer is too small: %d bytes", len);
			}
		}

		// // set if we should maybe also read fifo
		// err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM45686_INT1_STATUS0, &int1_status);
		// int1 = int1_status;

		// if (!(int1_status & INT1_STATUS_FIFO_THS))
		// {
		// 	// fifo threshold not exceeded, skip reading it for now
		// 	if (total == 0) {
		// 		LOG_WRN("No data to read int flags 0x%x 0x%x", int0, int1);
		// 	}
		// 	return total;
		// }
	}

	if (int1_status[0] & INT1_STATUS_FIFO_THS) {

		while (packets > 0 && len >= PACKET_SIZE)
		{
			uint8_t rawCount[2];
			err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, ICM45686_FIFO_COUNT_0, &rawCount[0], 2);
			packets = (uint16_t)(rawCount[0] << 8 | rawCount[1]); // Turn the 16 bits into a unsigned 16-bit value
			if (!packets) // nothing to do
				break;
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
	}

	if (total == 0) {
		LOG_WRN("no data");
		LOG_WRN("flags 0x%x 0x%x", int1_status[0], int1_status[1]);
	}

	return total;
}

static const uint8_t invalid[6] = {0x80, 0x00, 0x80, 0x00, 0x80, 0x00};

sensor_data_attrs_t icm45_data_process(uint16_t index, uint8_t *data, float a[3], float g[3], float m[3])
{
	index *= PACKET_SIZE;

	if (data[index] == 0x70)
	{
		// our artifical mag entry we read earlier
		if (ext_mag)
		{
			ext_mag->mag_process(&data[index+1], m);
			return DATA_VALID_MAG | DATA_OUTSIDE_FIFO;

		}
		else {
			return DATA_OUTSIDE_FIFO;
		}
	}

	if (data[index] != 0x78) // ACCEL_EN, GYRO_EN, HIRES_EN, TMST_FIELD_EN
		return DATA_INVALID; // Skip invalid header

	sensor_data_attrs_t result = 0;

	// Empty packet is 7F filled
	// combine into 20 bit values in 32 bit int
	if (memcmp(&data[index + 1], invalid, sizeof(invalid))) // valid accel data
	{
		for (int i = 0; i < 3; i++) // accel x, y, z
			a[i] = (int32_t)((((uint32_t)data[index + 1 + (i * 2)]) << 24) | (((uint32_t)data[index + 2 + (i * 2)]) << 16) | (((uint32_t)data[index + 17 + i] & 0xF0) << 8));

		result |= DATA_VALID_ACCEL;
	}
	if (memcmp(&data[index + 7], invalid, sizeof(invalid))) // valid gyro data
	{
		for (int i = 0; i < 3; i++) // gyro x, y, z
			g[i] = (int32_t)((((uint32_t)data[index + 7 + (i * 2)]) << 24) | (((uint32_t)data[index + 8 + (i * 2)]) << 16) | (((uint32_t)data[index + 17 + i] & 0x0F) << 12));

		result |= DATA_VALID_GYRO;
	}
	else if (!memcmp(&data[index + 1], invalid, sizeof(invalid))) // Skip invalid data
	{
		return DATA_INVALID;
	}
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a[i] *= accel_sensitivity_32;
		g[i] *= gyro_sensitivity_32;
	}

	return result;
}

const sensor_imu_t sensor_imu_icm45686 = {
	*icm45_init,
	*icm45_shutdown,

	*icm45_update_fs,
	*icm45_update_odr,

	*icm45_data_read,
	*icm45_data_process,
	*icm45_accel_read,
	*icm45_gyro_read,
	*icm45_temp_read,

	*icm45_setup_DRDY,
	*icm45_setup_WOM,

	*icm45_ext_setup
};

const sensor_ext_ssi_t sensor_ext_icm45686 = {
	*icm45_ext_write,
	*icm45_ext_write_read,
	15
};

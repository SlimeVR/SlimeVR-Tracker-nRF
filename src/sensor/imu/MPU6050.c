#include <math.h>

#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#include "MPU6050.h"
#include "sensor/sensor_none.h"

#define PACKET_SIZE 14

static float accel_sensitivity = 2.0f / 32768.0f;  // default ±2 g
static float gyro_sensitivity  = 250.0f / 32768.0f; // default ±250°/s

static uint8_t accel_fs = MPU6050_ACCEL_FS_2;
static uint8_t gyro_fs  = MPU6050_GYRO_FS_250;

LOG_MODULE_REGISTER(MPU6050, LOG_LEVEL_DBG);

int mpu_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
    int err = 0;

    // Reset device, select PLL as clock
    err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_PWR_MGMT_1, 0x80);
    k_msleep(100);
    err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_PWR_MGMT_1, 0x01);
    err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_PWR_MGMT_2, 0x00); // enable all axes
    k_msleep(10);

    // Sampling 1kHz/(1+7)=125Hz, DLPF 42Hz
    err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_SMPLRT_DIV, 0x07);
    err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42);

    // FS ranges
    err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_GYRO_CONFIG, (gyro_fs & 0x3) << 3);
    err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_ACCEL_CONFIG, (accel_fs & 0x3) << 3);

    uint8_t user;
    ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_USER_CTRL, &user);
    user &= ~(BIT(7)|BIT(6)|BIT(5)); // DMP_EN=0, FIFO_EN=0, I2C_MST_EN=0
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_USER_CTRL, user);
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_FIFO_EN, 0x00);
    k_usleep(50);

    // Pulse FIFO_RESET
    user |= BIT(2);
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_USER_CTRL, user);
    k_usleep(50);

    // Re-enable FIFO
    user |= BIT(6); // FIFO_EN
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_USER_CTRL, user);
    // Enable TEMP + ACCEL + GYROX/Y/Z to FIFO
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_FIFO_EN, 0xF8);
    uint8_t fifo_en = 0;
    ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_FIFO_EN, &fifo_en);
    fifo_en &= 0xF8; // clear bits 0-2 (SLVx_FIFO_EN)
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_FIFO_EN, fifo_en);
    uint8_t user_ctrl = 0;
    ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_USER_CTRL, &user_ctrl);
    user_ctrl &= ~BIT(5); // I2C_MST_EN = 0
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_USER_CTRL, user_ctrl);

    // Clear I2C master configuration registers
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_I2C_MST_CTRL, 0x00); // I2C_MST_CTRL
    for (uint8_t r = MPU6050_RA_I2C_SLV0_ADDR; r <= MPU6050_RA_I2C_SLV3_CTRL; r++)  // SLV0-SLV3 + their REGs/CTRL
        ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, r, 0x00);
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_I2C_SLV4_CTRL, 0x00);
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_I2C_SLV0_DO, 0x00); // I2C_SLV4_CTRL
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00); // I2C_MST_DELAY_CTRL

    // Enable interrupt for FIFO data ready
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_INT_ENABLE, 0x01);
    mpu_ext_passthrough(true);

    uint8_t dummy;
    ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_INT_STATUS, &dummy);

    *accel_actual_time = 1.f / 125;
    *gyro_actual_time  = 1.f / 125;
    return err;
}

void mpu_shutdown(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_PWR_MGMT_1, 1 << MPU6050_PWR1_SLEEP_BIT);
	if (err)
		LOG_ERR("Communication error");
}

void mpu_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
    if (accel_range > 8)
    {
        accel_fs = MPU6050_ACCEL_FS_16;
        accel_range = 16;
    }
    else if (accel_range > 4)
    {
        accel_fs = MPU6050_ACCEL_FS_8;
        accel_range = 8;
    }
    else if (accel_range > 2)
    {
        accel_fs = MPU6050_ACCEL_FS_4;
        accel_range = 4;
    }
    else
    {
        accel_fs = MPU6050_ACCEL_FS_2;
        accel_range = 2;
    }

    if (gyro_range > 1000)
    {
        gyro_fs = MPU6050_GYRO_FS_2000;
        gyro_range = 2000;
    }
    else if (gyro_range > 500)
    {
        gyro_fs = MPU6050_GYRO_FS_1000;
        gyro_range = 1000;
    }
    else if (gyro_range > 250)
    {
        gyro_fs = MPU6050_GYRO_FS_500;
        gyro_range = 500;
    }
    else
    {
        gyro_fs = MPU6050_GYRO_FS_250;
        gyro_range = 250;
    }

    accel_sensitivity = accel_range / 32768.0f;
    gyro_sensitivity  = gyro_range / 32768.0f;

    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
                       MPU6050_RA_ACCEL_CONFIG,
                       (accel_fs << MPU6050_ACONFIG_AFS_SEL_BIT));
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_GYRO_CONFIG, (gyro_fs << MPU6050_GCONFIG_FS_SEL_BIT));

    *accel_actual_range = accel_range;
    *gyro_actual_range = gyro_range;
}

int mpu_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
    // accel_time is in seconds
    float period_ms = accel_time > 0 ? (accel_time * 1000.0f) : 8.0f; // default ~8ms (125 Hz)
    int div = (int)lrintf(period_ms - 1.0f);
    if (div < 0)
	{
		div = 0;
	}
    if (div > 255)
	{
		div = 255;
	}

    int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_SMPLRT_DIV, div);
    if (err)
	{
		LOG_ERR("Communication error");
	}

    *accel_actual_time = (div + 1) / 1000.0f;
    *gyro_actual_time  = *accel_actual_time;
    return err;
}

void mpu_fifo_reset(void)
{
    uint8_t user = 0;
    ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_USER_CTRL, &user);

    // Disable FIFO & DMP
    user &= ~(BIT(7) | BIT(6)); // DMP_EN=0, FIFO_EN=0
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_USER_CTRL, user);
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_FIFO_EN, 0x00);
    k_usleep(50);

    // Pulse FIFO_RESET
    user |= BIT(2);
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_USER_CTRL, user);
    k_usleep(50);

    // Disable I2C master engine again (in case FIFO_RESET touched it)
    user &= ~BIT(5); // I2C_MST_EN = 0
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_USER_CTRL, user);

    // Clear all I2C master configuration registers
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_I2C_MST_CTRL, 0x00);
    for (uint8_t r = MPU6050_RA_I2C_SLV0_ADDR; r <= MPU6050_RA_I2C_SLV3_CTRL; r++)
        ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, r, 0x00);
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_I2C_SLV4_CTRL, 0x00);
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);

    // Clear sticky interrupt
    uint8_t dummy;
    ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_INT_STATUS, &dummy);

    // Re-enable FIFO (TEMP + ACCEL + GYRO only, 14 bytes per packet)
    user |= BIT(6); // FIFO_EN
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_USER_CTRL, user);
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_FIFO_EN, 0xF8); // 1111 1000

    uint8_t fifo_en = 0;
    ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_FIFO_EN, &fifo_en);
    LOG_INF("FIFO reset complete, FIFO_EN=0x%02x (expected 0xF8)", fifo_en);
}

uint16_t mpu_fifo_read(uint8_t *data, uint16_t len)
{
    if (len < PACKET_SIZE)
        return 0;

    uint8_t cnt[2];
    if (ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_FIFO_COUNTH, cnt, 2))
        return 0;

    uint16_t fifo_bytes = ((uint16_t)cnt[0] << 8) | cnt[1];
    if (fifo_bytes < PACKET_SIZE)
        return 0;

    // Round down to complete packets
    uint16_t packets = fifo_bytes / PACKET_SIZE;
    if (packets * PACKET_SIZE > len)
        packets = len / PACKET_SIZE;

    uint16_t to_read = packets * PACKET_SIZE;
    if (ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_FIFO_R_W, data, to_read)) {
        LOG_ERR("FIFO read error");
        return 0;
    }

    // Drop any partial leftover bytes
    uint16_t leftover = fifo_bytes - to_read;
    if (leftover) {
        uint8_t junk[16];
        ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_FIFO_R_W, junk, leftover);
        LOG_WRN("Dropped %u leftover bytes", leftover);
    }

    return packets;
}

int mpu_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
    index *= PACKET_SIZE;
    for (int i = 0; i < 3; i++) {
        int16_t accel_raw = (data[index + i*2] << 8) | data[index + i*2 + 1];
        a[i] = accel_raw * accel_sensitivity;
    }
    for (int i = 0; i < 3; i++) {
        int16_t gyro_raw = (data[index + 6 + i*2] << 8) | data[index + 6 + i*2 + 1];
        g[i] = gyro_raw * gyro_sensitivity;
    }
    return 0;
}

void mpu_accel_read(float a[3])
{
	uint8_t raw[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_ACCEL_XOUT_H, raw, 6);
	if (err)
		LOG_ERR("Communication error");
	for (int i = 0; i < 3; i++) {
		int16_t v = (raw[i * 2] << 8) | raw[i * 2 + 1];
		a[i] = v * accel_sensitivity;
	}
}

void mpu_gyro_read(float g[3])
{
	uint8_t raw[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_GYRO_XOUT_H, raw, 6);
	if (err)
		LOG_ERR("Communication error");
	for (int i = 0; i < 3; i++) {
		int16_t v = (raw[i * 2] << 8) | raw[i * 2 + 1];
		g[i] = v * gyro_sensitivity;
	}
}

float mpu_temp_read(void)
{
	uint8_t raw[2];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_TEMP_OUT_H, raw, 2);
	if (err)
		LOG_ERR("Communication error");
	int16_t val = (raw[0] << 8) | raw[1];
	return (val / 340.0f) + 36.53f;
}

uint8_t mpu_setup_DRDY(uint16_t threshold)
{
	(void)threshold; // MPU6050 does not support programmable FIFO watermark
	int err = 0;
	// Active low, open-drain, latch until read
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_INT_PIN_CFG, 0x02); // BYPASS_EN=1
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_INT_ENABLE, 0x01);
	uint8_t dummy;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_INT_STATUS, &dummy);
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW;
}

// This has made me rethink my wonderful idea of implementing this... mess
uint8_t mpu_setup_WOM(void)
{
	int err = 0;

	// Motion detection setup (like WOM)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_PWR_MGMT_1, 0x00); // wake
	k_msleep(50);

	// Configure detection thresholds (LSB ~1 mg @ ±2g)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_MOT_THR, 0x14); // ~20 mg
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_MOT_DUR, 0x10); // 16 ms duration
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_MOT_DETECT_CTRL, 0x15); // use accel LPF
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_INT_ENABLE, (1 << MPU6050_INTERRUPT_MOT_BIT));
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_INT_PIN_CFG, 0x02); // BYPASS_EN=1
	// Cycle between sleep and sample
	uint8_t pwr2 = (1 << MPU6050_PWR2_STBY_XG_BIT) | (1 << MPU6050_PWR2_STBY_YG_BIT) | (1 << MPU6050_PWR2_STBY_ZG_BIT); // only accel active
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_PWR_MGMT_2, pwr2);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_PWR_MGMT_1, (1 << MPU6050_PWR1_CYCLE_BIT)); // enable cycle mode
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low interrupt
}

int mpu_ext_passthrough(bool enable)
{
	int err = 0;
	if (enable)
	{
        err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_PWR_MGMT_1, 0x00);
        k_msleep(10);

        uint8_t user = 0;
        ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_USER_CTRL, &user);
        user &= ~BIT(5); // I2C_MST_EN = 0
        err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_USER_CTRL, user);

        uint8_t ipcfg = 0;
        ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_INT_PIN_CFG, &ipcfg);
        ipcfg |= BIT(1); // BYPASS_EN = 1
        ipcfg &= ~BIT(0); // INT_LEVEL = 0 (not latched)
        err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_INT_PIN_CFG, ipcfg);

        err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, MPU6050_RA_PWR_MGMT_1, 0x01);
	}

	if (err)
		LOG_ERR("Communication error");

    return err;
}

const sensor_imu_t sensor_imu_mpu6050 = {
	*mpu_init,
	*mpu_shutdown,
	*mpu_update_fs,
	*mpu_update_odr,
	*mpu_fifo_read,
	*mpu_fifo_process,
	*mpu_accel_read,
	*mpu_gyro_read,
	*mpu_temp_read,
	*mpu_setup_DRDY,
	*mpu_setup_WOM,
	*imu_none_ext_setup,
	*mpu_ext_passthrough
};
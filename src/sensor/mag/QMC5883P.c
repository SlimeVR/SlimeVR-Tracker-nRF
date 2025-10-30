#include <math.h>

#include <zephyr/logging/log.h>

#include "sensor/sensor_none.h"

#include "../imu/MPU6050.h"
#include "QMC5883P.h"

LOG_MODULE_REGISTER(QMC5883P, LOG_LEVEL_INF);

#define ODR_1Hz  0b000
#define ODR_10Hz  0b001
#define ODR_50Hz  0b010
#define ODR_100Hz 0b011
#define ODR_200Hz 0b100
#define ODR_MASK(odr) ((odr) << 2)

#define OSR_OFF 0b11
#define OSR_2 0b10
#define OSR_4 0b01
#define OSR_8 0b00
#define OSR_MASK(osr) ((osr) << 6)

#define RNG_2G 0b00
#define RNG_8G 0b01
#define RNG_MASK(x) (((x) & 0x03) << 4)

// ±8 G mode: 3750 LSB/G ~0.267 mG/LSB
static const float sensitivity = 1.0f / 3750.0f;

static uint8_t last_ctrl1 = 0xFF;
static uint8_t last_ctrl2 = 0xFF;
static bool overflow_latched = false;

int qmc5883p_init(float time, float *actual_time)
{
    mpu_ext_passthrough(true);

    uint8_t id = 0;
	int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_CHIP_ID_REG, &id);
    if (id != 0x80) {
        LOG_ERR("QMC5883P: invalid WHOAMI=0x%02X", id);
        return -ENODEV;
    }

    // Define axis sign and range
    err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_AXIS_SIGN_REG, 0x06);
    k_msleep(5);

    uint8_t ctrl2 = QMC5883P_RNG_8G | QMC5883P_SET_RESET_ON;
    err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_CTRL2_REG, ctrl2);
    last_ctrl2 = ctrl2;
    k_msleep(5);

    err |= qmc5883p_update_odr(time, actual_time);
    overflow_latched = false;

	if (err)
		LOG_ERR("Communication error");

    return 0;
}

void qmc5883p_shutdown(void)
{
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_CTRL1_REG, QMC5883P_MODE_SUSPEND);
    k_msleep(5);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_CTRL2_REG, QMC5883P_SOFT_RST);
	if (err)
		LOG_ERR("Communication error");
}

int qmc5883p_update_odr(float time, float *actual_time)
{
	int ODR;
	uint8_t MODR;
	uint8_t MD;

	if (time <= 0 || time == INFINITY) // power down mode or single measurement mode
	{
		MD = QMC5883P_MODE_SUSPEND; // oneshot will set SINGLE after
		ODR = 0;
	}
	else
	{
		MD = QMC5883P_MODE_CONT;
		ODR = 1 / time;
	}

	if (MD == QMC5883P_MODE_SUSPEND)
	{
		MODR = ODR_200Hz; // for oneshot
		time = 0; // off
	}
	else if (ODR > 100)
	{
		MODR = ODR_200Hz;
		time = 1.f / 200;
	}
	else if (ODR > 50)
	{
		MODR = ODR_100Hz;
		time = 1.f / 100;
	}
	else if (ODR > 25)
	{
		MODR = ODR_50Hz;
		time = 1.f / 50;
	}
	else if (ODR > 5)
	{
		MODR = ODR_10Hz;
		time = 1.f / 10;
	}
	else
	{
		MODR = ODR_1Hz;
		time = 1.f;
	}

    *actual_time = time;

    uint8_t ctrl1 = OSR_MASK(OSR_2) | ODR_MASK(MODR) | RNG_MASK(RNG_8G) | QMC5883P_MODE_NORMAL;
    if (ctrl1 == last_ctrl1)
        return 0;

    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_CTRL1_REG, ctrl1);

    last_ctrl1 = ctrl1;
    return 0;
}

void qmc5883p_mag_oneshot(void)
{
    // Single measurement, automatically returns to suspend
    int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_CTRL1_REG, OSR_MASK(OSR_8) | ODR_MASK(ODR_200Hz) | RNG_MASK(RNG_8G) | QMC5883P_MODE_SINGLE);
	if (err)
		LOG_ERR("Communication error");
    k_usleep(5000);
}

void qmc5883p_mag_read(float m[3])
{
    uint8_t status;
    int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_STATUS_REG, &status);
	if (err)
        return;

    if (!(status & QMC5883P_STAT_DRDY))
        return;

    if (status & QMC5883P_STAT_OVFL && !overflow_latched)
    {
        LOG_WRN("QMC5883P: magnetic overflow detected");
        overflow_latched = true;
    }

    uint8_t rawData[6];
    for (int i = 0; i < 6; i++)
    {
        ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, QMC5883P_OUTX_L_REG + i, &rawData[i]);
    }

    qmc5883p_mag_process(rawData, m);
}

void qmc5883p_mag_process(uint8_t *raw_m, float m[3])
{
    int16_t *s16 = (int16_t *)raw_m;
    for (int i = 0; i < 3; i++)
        m[i] = s16[i] * sensitivity; // in Gauss
}

const sensor_mag_t sensor_mag_qmc5883p = {
    *qmc5883p_init,
    *qmc5883p_shutdown,
    *qmc5883p_update_odr,
    *qmc5883p_mag_oneshot,
    *qmc5883p_mag_read,
    *mag_none_temp_read,
    *qmc5883p_mag_process,
    6, 6
};
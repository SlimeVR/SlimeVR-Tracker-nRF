#include <zephyr/logging/log.h>

#include "ISM330BX.h"
#include "LSM6DSV.h" // Common functions
#include "sensor/sensor_none.h"

#define PACKET_SIZE 7

LOG_MODULE_REGISTER(ISM330BX, LOG_LEVEL_DBG);

int ism_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	index *= PACKET_SIZE;
	switch (data[index] >> 3)
	{
	case 0x02: // Accelerometer NC (Accelerometer uncompressed data)
		for (int i = 0; i < 3; i++) // z, y, x
		{
			a[2 - i] = (int16_t)((((uint16_t)data[index + 2 + (i * 2)]) << 8) | data[index + 1 + (i * 2)]);
			a[2 - i] *= accel_sensitivity;
		}
		return 0;
	case 0x01: // Gyroscope NC (Gyroscope uncompressed data)
		for (int i = 0; i < 3; i++) // x, y, z
		{
			g[i] = (int16_t)((((uint16_t)data[index + 2 + (i * 2)]) << 8) | data[index + 1 + (i * 2)]);
			g[i] *= gyro_sensitivity;
		}
		return 0;
	default:
	}
	// TODO: need to skip invalid data
	return 1;
}

void ism_accel_read(float a[3])
{
	uint8_t rawAccel[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_OUTX_L_A, &rawAccel[0], 6);
	if (err)
		LOG_ERR("Communication error");
	for (int i = 0; i < 3; i++) // z, y, x
	{
		a[2 - i] = (int16_t)((((uint16_t)rawAccel[1 + (i * 2)]) << 8) | rawAccel[i * 2]);
		a[2 - i] *= accel_sensitivity;
	}
}

const sensor_imu_t sensor_imu_ism330bx = {
	*lsm_init,
	*lsm_shutdown,

	*lsm_update_fs,
	*lsm_update_odr,

	*lsm_fifo_read,
	*ism_fifo_process,
	*ism_accel_read,
	*lsm_gyro_read,
	*lsm_temp_read,

	*lsm_setup_DRDY,
	*lsm_setup_WOM,

	*lsm_ext_setup,
	*lsm_ext_passthrough
};

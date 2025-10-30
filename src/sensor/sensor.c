/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"
#include "system/system.h"
#include "util.h"
#include "connection/connection.h"
#include "calibration.h"

#include <math.h>
#include <hal/nrf_gpio.h>

#include "fusion/fusions.h"
#include "sensors.h"

#include "sensor.h"

#define SPI_OP SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8)

#if DT_NODE_HAS_STATUS(DT_NODELABEL(imu_spi), okay)
#define SENSOR_IMU_SPI_EXISTS true
#define SENSOR_IMU_SPI_NODE DT_NODELABEL(imu_spi)
static struct spi_dt_spec sensor_imu_spi_dev = SPI_DT_SPEC_GET(SENSOR_IMU_SPI_NODE, SPI_OP, 0);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(imu), okay)
#define SENSOR_IMU_EXISTS true
#define SENSOR_IMU_NODE DT_NODELABEL(imu)
static struct i2c_dt_spec sensor_imu_dev = I2C_DT_SPEC_GET(SENSOR_IMU_NODE);
#else
static struct i2c_dt_spec sensor_imu_dev = {0};
#endif
#if !SENSOR_IMU_SPI_EXISTS && !SENSOR_IMU_EXISTS
#error "IMU node does not exist"
#endif
static uint8_t sensor_imu_dev_reg = 0xFF;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(mag_spi), okay)
#define SENSOR_MAG_SPI_EXISTS true
#define SENSOR_MAG_SPI_NODE DT_NODELABEL(mag_spi)
static struct spi_dt_spec sensor_mag_spi_dev = SPI_DT_SPEC_GET(SENSOR_MAG_SPI_NODE, SPI_OP, 0);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(mag), okay)
#define SENSOR_MAG_EXISTS true
#define SENSOR_MAG_NODE DT_NODELABEL(mag)
static struct i2c_dt_spec sensor_mag_dev = I2C_DT_SPEC_GET(SENSOR_MAG_NODE);
#else
static struct i2c_dt_spec sensor_mag_dev = {0};
#endif
#if SENSOR_IMU_SPI_EXISTS // might exist
#define SENSOR_MAG_EXT_EXISTS true
#endif
#if !SENSOR_MAG_SPI_EXISTS && !SENSOR_MAG_EXISTS && !SENSOR_MAG_EXT_EXISTS
#warning "Magnetometer node does not exist"
#endif
static uint8_t sensor_mag_dev_reg = 0xFF;

static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion
static float last_q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion

static float q3[4] = {SENSOR_QUATERNION_CORRECTION}; // correction quaternion

static float last_lin_a[3] = {0}; // vector to hold last linear accelerometer

static int64_t last_suspend_attempt_time = 0;
static int64_t last_data_time;

static float max_gyro_speed_square;
static bool mag_use_oneshot;
static bool mag_skip_oneshot;

static float accel_actual_time;
static float gyro_actual_time;
static float mag_actual_time;

static float sensor_actual_time;
static int16_t sensor_fifo_threshold;
static int64_t sensor_data_time; // ticks

static bool sensor_fusion_init;
static bool sensor_sensor_init;

static bool sensor_sensor_scanning;

static bool main_suspended;

static bool mag_available;
#if MAG_ENABLED
static bool mag_enabled = true; // TODO: toggle from server
#else
static bool mag_enabled = false;
#endif

#if CONFIG_SENSOR_USE_XIOFUSION
static const sensor_fusion_t *sensor_fusion = &sensor_fusion_fusion; // TODO: change from server
int fusion_id = FUSION_FUSION;
#elif CONFIG_SENSOR_USE_NXPSENSORFUSION
static const sensor_fusion_t *sensor_fusion = &sensor_fusion_motionsense; // TODO: change from server
int fusion_id = FUSION_MOTIONSENSE;
#elif CONFIG_SENSOR_USE_VQF
static const sensor_fusion_t *sensor_fusion = &sensor_fusion_vqf; // TODO: change from server
int fusion_id = FUSION_VQF;
#endif

static int sensor_imu_id = -1;
static int sensor_mag_id = -1;
static const sensor_imu_t *sensor_imu = &sensor_imu_none;
static const sensor_mag_t *sensor_mag = &sensor_mag_none;

//#define DEBUG true

#if DEBUG
LOG_MODULE_REGISTER(sensor, LOG_LEVEL_DBG);
#else
LOG_MODULE_REGISTER(sensor, LOG_LEVEL_INF);
#endif

static int sensor_scan(void);
static int sensor_init(void);
static void sensor_loop(void);
static struct k_thread sensor_thread_id;
static K_THREAD_STACK_DEFINE(sensor_thread_id_stack, 1024);

K_THREAD_DEFINE(sensor_init_thread_id, 256, sensor_request_scan, true, NULL, NULL, SENSOR_REQUEST_SCAN_THREAD_PRIORITY, 0, 0);
//crashing on nrf54l at 256

/* init thread handles starting scanner on the main thread, and then switches to the loop, before returning
   afterwards, other calls to start scanner will stop the loop on their thread and start the scanner on its own; it will also wait for the scanner to finish
   if the loop needs to handle power off, it should start another thread or otherwise offload the call so it does not try to kill itself
   in this case, it is appropriate to queue the request to power thread
*/

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, int0_gpios)
#define IMU_INT_EXISTS true
static const struct gpio_dt_spec int0 = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, int0_gpios);
#endif

const char *sensor_get_sensor_imu_name(void)
{
	if (sensor_imu_id < 0)
		return "None";
	return dev_imu_names[sensor_imu_id];
}

const char *sensor_get_sensor_mag_name(void)
{
	if (sensor_mag_id < 0)
		return "None";
	return dev_mag_names[sensor_mag_id];
}

const char *sensor_get_sensor_fusion_name(void)
{
	if (fusion_id < 0)
		return "None";
	return fusion_names[fusion_id];
}

void sensor_scan_thread(void)
{
	int err;
	sys_interface_resume(); // make sure interfaces are enabled
	err = sensor_scan(); // IMUs discovery
	if (err)
	{
		k_msleep(5);
		LOG_INF("Retrying sensor detection");

		// Reset address before retrying sensor detection
		sensor_imu_dev.addr = 0x00;

		err = sensor_scan(); // on POR, the sensor may not be ready yet
	}
	sys_interface_suspend();
//	if (err)
//		return err;
}

int sensor_scan(void)
{
	while (sensor_sensor_scanning)
		k_usleep(1); // already scanning
	if (sensor_sensor_init)
		return 0; // already initialized
	sensor_sensor_scanning = true;

	sensor_scan_read();
	int imu_id = -1;
#if SENSOR_IMU_SPI_EXISTS
	// for SPI scan, set frequency of 10MHz, it will be set later by the driver initialization if needed
	sensor_imu_spi_dev.config.frequency = MHZ(10);
	LOG_INF("Scanning SPI bus for IMU");
	imu_id = sensor_scan_imu_spi(&sensor_imu_spi_dev, &sensor_imu_dev_reg);
	if (imu_id >= 0)
		sensor_interface_register_sensor_imu_spi(&sensor_imu_spi_dev);
#endif
#if SENSOR_IMU_EXISTS
	if (imu_id < 0)
	{
		LOG_INF("Scanning I2C bus for IMU");
		imu_id = sensor_scan_imu(&sensor_imu_dev, &sensor_imu_dev_reg);
		if (imu_id >= 0)
			sensor_interface_register_sensor_imu_i2c(&sensor_imu_dev);
	}
#endif
#if !SENSOR_IMU_SPI_EXISTS && !SENSOR_IMU_EXISTS
	LOG_ERR("IMU node does not exist");
#endif
	if (imu_id >= (int)ARRAY_SIZE(dev_imu_names))
		LOG_WRN("Found unknown device");
	else if (imu_id < 0)
		LOG_ERR("No IMU detected");
	else
		LOG_INF("Found %s", dev_imu_names[imu_id]);
	if (imu_id >= 0)
	{
		if (imu_id >= (int)ARRAY_SIZE(sensor_imus) || sensor_imus[imu_id] == NULL || sensor_imus[imu_id] == &sensor_imu_none)
		{
			sensor_scan_clear(); // clear invalid sensor data
			sensor_imu = &sensor_imu_none;
			sensor_sensor_scanning = false; // done
			LOG_ERR("IMU not supported");
			set_status(SYS_STATUS_SENSOR_ERROR, true);
			return -1; // an IMU was detected but not supported
		}
		else
		{
			sensor_imu = sensor_imus[imu_id];
		}
	}
	else
	{
		sensor_scan_clear(); // clear invalid sensor data
		sensor_imu = &sensor_imu_none;
		sensor_sensor_scanning = false; // done
		set_status(SYS_STATUS_SENSOR_ERROR, true);
		return -1; // no IMU detected! something is very wrong
	}

	int mag_id = -1;
#if SENSOR_MAG_SPI_EXISTS
	// for SPI scan, set frequency of 10MHz, it will be set later by the driver initialization if needed
	sensor_mag_spi_dev.config.frequency = MHZ(10);
	LOG_INF("Scanning SPI bus for magnetometer");
	mag_id = sensor_scan_mag_spi(&sensor_mag_spi_dev, &sensor_mag_dev_reg);
	if (mag_id >= 0)
		sensor_interface_register_sensor_mag_spi(&sensor_mag_spi_dev);
#endif
#if SENSOR_MAG_EXISTS
	if (mag_id < 0)
	{
		LOG_INF("Scanning bus for magnetometer");
		mag_id = sensor_scan_mag(&sensor_mag_dev, &sensor_mag_dev_reg);
		if (mag_id >= 0)
			sensor_interface_register_sensor_mag_i2c(&sensor_mag_dev);
	}
	if (mag_id < 0 && !(sensor_imu_dev_reg & 0x80)) // I2C IMU
	{
		// IMU may support passthrough mode if the magnetometer is connected through the IMU
		int err = sensor_imu->ext_passthrough(true); // no need to disable, the imu will be reset later
		if (!err)
		{
			LOG_INF("Scanning bus for magnetometer through IMU passthrough");
			if (sensor_mag_dev.addr > 0x80) // marked as external
			{
				sensor_mag_dev.addr &= 0x7F;
			}
			else
			{
				sensor_mag_dev.addr = 0x00; // reset magnetometer data
				sensor_mag_dev_reg = 0xFF;
			}
			mag_id = sensor_scan_mag(&sensor_mag_dev, &sensor_mag_dev_reg);
			if (mag_id >= 0)
			{
				sensor_mag_dev.addr |= 0x80; // mark as external
				sensor_interface_register_sensor_mag_i2c(&sensor_mag_dev); // can register as i2c
			}
		}
	}
#endif
#if SENSOR_MAG_EXT_EXISTS
	if (mag_id < 0 && (sensor_imu_dev_reg & 0x80)) // SPI IMU
	{
		// IMU may support I2CM if the magnetometer is connected through the IMU
		int err = sensor_imu->ext_setup();
		if (!err)
		{
			LOG_INF("Scanning bus for magnetometer through IMU I2CM");
			if (sensor_mag_dev.addr > 0x80) // marked as external
			{
				sensor_mag_dev.addr &= 0x7F;
			}
			else
			{
				sensor_mag_dev.addr = 0x00; // reset magnetometer data
				sensor_mag_dev_reg = 0xFF;
			}
			mag_id = sensor_scan_mag_ext(sensor_interface_ext_get(), &sensor_mag_dev.addr, &sensor_mag_dev_reg);
			if (mag_id >= 0 && mag_id < (int)ARRAY_SIZE(sensor_mags) && sensor_mags[mag_id] != NULL && sensor_mags[mag_id] != &sensor_mag_none)
			{
				err = sensor_interface_register_sensor_mag_ext(sensor_mag_dev.addr, sensor_mags[mag_id]->ext_min_burst, sensor_mags[mag_id]->ext_burst);
				sensor_mag_dev.addr |= 0x80; // mark as external
				if (err)
				{
					mag_id = -1;
					LOG_ERR("Failed to register magnetometer external interface");
				}
			}
		}
	}
#endif
#if !SENSOR_MAG_SPI_EXISTS && !SENSOR_MAG_EXISTS && !SENSOR_MAG_EXT_EXISTS
	LOG_WRN("Magnetometer node does not exist");
#endif
	if (mag_id >= (int)ARRAY_SIZE(dev_mag_names))
		LOG_WRN("Found unknown device");
	else if (mag_id < 0)
		LOG_WRN("No magnetometer detected");
	else
		LOG_INF("Found %s", dev_mag_names[mag_id]);
	if (mag_id >= 0) // if there is no magnetometer we do not care as much
	{
		if (mag_id >= (int)ARRAY_SIZE(sensor_mags) || sensor_mags[mag_id] == NULL || sensor_mags[mag_id] == &sensor_mag_none)
		{
			sensor_mag = &sensor_mag_none;
			mag_available = false;
			LOG_ERR("Magnetometer not supported");
		}
		else
		{
			sensor_mag = sensor_mags[mag_id];
			mag_available = true;
		}
	}
	else
	{
		sensor_mag = &sensor_mag_none;
		mag_available = false; // marked as not available
	}

	sensor_scan_write();
	connection_update_sensor_ids(imu_id, mag_id);
	sensor_imu_id = imu_id;
	sensor_mag_id = mag_id;

	sensor_sensor_init = true; // successfully initialized
	sensor_sensor_scanning = false; // done
	set_status(SYS_STATUS_SENSOR_ERROR, false); // clear error
	return 0;
}

static bool main_running = false;

int sensor_request_scan(bool force)
{
	if (sensor_sensor_init && !force)
		return 0; // already initialized
	main_imu_suspend();
	k_thread_abort(&sensor_thread_id); // stop the sensor thread // TODO: may need to handle fusion state
	LOG_INF("Aborted sensor thread");
	main_suspended = false;
	sensor_sensor_init = false;
	if (force)
	{
		sensor_imu_dev.addr = 0x00;
		sensor_mag_dev.addr = 0x00;
		sensor_imu_dev_reg = 0xFF;
		sensor_mag_dev_reg = 0xFF;
		LOG_INF("Requested sensor scan");
	}
	k_thread_create(&sensor_thread_id, sensor_thread_id_stack, K_THREAD_STACK_SIZEOF(sensor_thread_id_stack), (k_thread_entry_t)sensor_scan_thread, NULL, NULL, NULL, SENSOR_SCAN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_join(&sensor_thread_id, K_FOREVER); // wait for the thread to finish
	if (sensor_sensor_init && force)
	{
		k_thread_create(&sensor_thread_id, sensor_thread_id_stack, K_THREAD_STACK_SIZEOF(sensor_thread_id_stack), (k_thread_entry_t)sensor_loop, NULL, NULL, NULL, SENSOR_LOOP_THREAD_PRIORITY, K_FP_REGS, K_NO_WAIT);
		LOG_INF("Started sensor loop");
	}
	return !sensor_sensor_init;
}

void sensor_scan_read(void) // TODO: move some of this to sys?
{
	if (retained->imu_addr != 0)
	{
		sensor_imu_dev.addr = retained->imu_addr;
		sensor_imu_dev_reg = retained->imu_reg;
	}
	if (retained->mag_addr != 0)
	{
		sensor_mag_dev.addr = retained->mag_addr;
		sensor_mag_dev_reg = retained->mag_reg;
	}
	LOG_INF("IMU address: 0x%02X, register: 0x%02X", sensor_imu_dev.addr, sensor_imu_dev_reg);
	LOG_INF("Magnetometer address: 0x%02X, register: 0x%02X", sensor_mag_dev.addr, sensor_mag_dev_reg);
}

void sensor_scan_write(void) // TODO: move some of this to sys?
{
	retained->imu_addr = sensor_imu_dev.addr;
	retained->mag_addr = sensor_mag_dev.addr;
	retained->imu_reg = sensor_imu_dev_reg;
	retained->mag_reg = sensor_mag_dev_reg;
	retained_update();
}

void sensor_scan_clear(void) // TODO: move some of this to sys?
{
	retained->imu_addr = 0x00;
	retained->mag_addr = 0x00;
	retained->imu_reg = 0xFF;
	retained->mag_reg = 0xFF;
	retained_update();
}

void sensor_retained_read(void) // TODO: move some of this to sys? or move to calibration?
{
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	LOG_INF("Accelerometer matrix:");
	for (int i = 0; i < 3; i++)
		LOG_INF("%.5f %.5f %.5f %.5f", (double)retained->accBAinv[0][i], (double)retained->accBAinv[1][i], (double)retained->accBAinv[2][i], (double)retained->accBAinv[3][i]);
#else
	LOG_INF("Accelerometer bias: %.5f %.5f %.5f", (double)retained->accelBias[0], (double)retained->accelBias[1], (double)retained->accelBias[2]);
#endif
	LOG_INF("Gyroscope bias: %.5f %.5f %.5f", (double)retained->gyroBias[0], (double)retained->gyroBias[1], (double)retained->gyroBias[2]);
	if (mag_available && mag_enabled)
	{
//		LOG_INF("Magnetometer bridge offset: %.5f %.5f %.5f", (double)retained->magBias[0], (double)retained->magBias[1], (double)retained->magBias[2]);
		LOG_INF("Magnetometer matrix:");
		for (int i = 0; i < 3; i++)
			LOG_INF("%.5f %.5f %.5f %.5f", (double)retained->magBAinv[0][i], (double)retained->magBAinv[1][i], (double)retained->magBAinv[2][i], (double)retained->magBAinv[3][i]);
	}
	if (retained->fusion_id)
		LOG_INF("Fusion data recovered");
}

void sensor_retained_write(void) // TODO: move to sys?
{
	if (!sensor_fusion_init)
		return;
//	memcpy(retained->magBias, sensor_calibration_get_magBias(), sizeof(retained->magBias));
	sensor_fusion->save(retained->fusion_data);
	retained->fusion_id = fusion_id;
	retained_update();
}

void sensor_shutdown(void) // Communicate all imus to shut down
{
	int err = sensor_request_scan(false); // try initialization if possible
	if (mag_available || !err)
	{
		sys_interface_resume();
		if (mag_available) // try to shutdown magnetometer first (in case of passthrough)
			sensor_mag->shutdown();
		if (!err)
			sensor_imu->shutdown();
		sys_interface_suspend();
	}
	else
	{
		LOG_ERR("Failed to shutdown sensors");
	}
}

uint8_t sensor_setup_WOM(void)
{
	int err = sensor_request_scan(false); // try initialization if possible
	if (!err)
	{
		sys_interface_resume();
		err = sensor_imu->setup_WOM();
		sys_interface_suspend();
		return err;
	}
	else
	{
		LOG_ERR("Failed to configure IMU wake up");
		return 0;
	}
}

void sensor_fusion_invalidate(void)
{
	main_imu_restart(); // reinitialize fusion
	if (sensor_fusion_init)
	{ // clear fusion gyro offset
		float g_off[3] = {0};
		sensor_fusion->set_gyro_bias(g_off);
		sensor_retained_write();
	}
	else
	{ // TODO: always clearing the fusion?
		retained->fusion_id = 0; // Invalidate retained fusion data
		retained_update();
	}
}

int sensor_update_time_ms = 6;

// TODO: get rid of it.. ?
static void set_update_time_ms(int time_ms)
{
	// TODO: maybe not get rid of it? it is now repurposed to also change FIFO threshold
	// TODO: return pin_config and replace call in sensor_init
#if IMU_INT_EXISTS
	float fifo_threshold = time_ms / 1000.0f / sensor_actual_time; // target loop rate
	sensor_fifo_threshold = fifo_threshold;
	LOG_INF("FIFO THS/WM/WTM: %.2f -> %d", (double)fifo_threshold, sensor_fifo_threshold);
	sensor_imu->setup_DRDY(sensor_fifo_threshold); // do not need to reset pin config
#endif
	sensor_update_time_ms = time_ms; // TODO: terrible naming
}

bool main_wfi = false;

static void sensor_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	// wake up sensor thread
	if (main_wfi)
	{
		// Use to time latency
		sensor_data_time = k_uptime_ticks();
		main_wfi = false;
		k_wakeup(&sensor_thread_id);
	}
}

static struct gpio_callback sensor_cb_data;

enum sensor_sensor_mode {
//	SENSOR_SENSOR_MODE_OFF,
	SENSOR_SENSOR_MODE_LOW_NOISE,
	SENSOR_SENSOR_MODE_LOW_POWER,
	SENSOR_SENSOR_MODE_LOW_POWER_2
};

static enum sensor_sensor_mode sensor_mode = SENSOR_SENSOR_MODE_LOW_NOISE;
static enum sensor_sensor_mode last_sensor_mode = SENSOR_SENSOR_MODE_LOW_NOISE;

enum sensor_sensor_timeout {
	SENSOR_SENSOR_TIMEOUT_IMU,
	SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED,
	SENSOR_SENSOR_TIMEOUT_ACTIVITY,
	SENSOR_SENSOR_TIMEOUT_ACTIVITY_ELAPSED,
};

static enum sensor_sensor_timeout sensor_timeout = SENSOR_SENSOR_TIMEOUT_IMU;

// Check the IMU gyroscope // TODO: gyro sanity not used
 // TODO: timeouts and power management should be outside sensor! (ie. sleeping/shutdown even if the imu completely errored out)
 // all this really means is that this should be called in sensor loop while the sensor is in an error state
static void sensor_update_sensor_state(void)
{
	bool calibrating = get_status(SYS_STATUS_CALIBRATION_RUNNING);
	bool resting = sensor_fusion->get_gyro_sanity() == 0 ? q_epsilon(q, last_q, 0.005) : q_epsilon(q, last_q, 0.05); // TODO: Probably okay to use the constantly updating last_q?
	if (!calibrating && resting)
	{
		int64_t last_data_delta = k_uptime_get() - last_data_time;
		if (sensor_mode < SENSOR_SENSOR_MODE_LOW_POWER && last_data_delta > CONFIG_SENSOR_LP_TIMEOUT) // No motion in lp timeout
		{
			LOG_INF("No motion from sensors in %dms", CONFIG_SENSOR_LP_TIMEOUT);
			sensor_mode = SENSOR_SENSOR_MODE_LOW_POWER;
		}
#if CONFIG_SENSOR_USE_LOW_POWER_2 || CONFIG_USE_IMU_TIMEOUT
		int64_t imu_timeout = CLAMP(last_data_time - last_suspend_attempt_time, CONFIG_IMU_TIMEOUT_RAMP_MIN, CONFIG_IMU_TIMEOUT_RAMP_MAX); // Ramp timeout from last_data_time
#endif
#if CONFIG_SENSOR_USE_LOW_POWER_2
		if (sensor_mode < SENSOR_SENSOR_MODE_LOW_POWER_2 && last_data_delta > imu_timeout) // No motion in ramp time
			sensor_mode = SENSOR_SENSOR_MODE_LOW_POWER_2;
#endif
#if CONFIG_USE_ACTIVE_TIMEOUT
		if (sensor_timeout < SENSOR_SENSOR_TIMEOUT_ACTIVITY && last_data_delta > CONFIG_ACTIVE_TIMEOUT_THRESHOLD) // higher priority than IMU timeout
		{
			LOG_INF("Switching to activity timeout");
			sensor_timeout = SENSOR_SENSOR_TIMEOUT_ACTIVITY;
		}
		if (sensor_timeout == SENSOR_SENSOR_TIMEOUT_ACTIVITY && last_data_delta > CONFIG_ACTIVE_TIMEOUT_DELAY)
		{
			LOG_INF("No motion from sensors in %dm", CONFIG_ACTIVE_TIMEOUT_DELAY / 60000);
#if CONFIG_SLEEP_ON_ACTIVE_TIMEOUT && CONFIG_USE_IMU_WAKE_UP
			// Queue power state request, it is possible for the request to be overridden so the thread may continue unaware
			sys_request_WOM(true, false);
#elif CONFIG_SHUTDOWN_ON_ACTIVE_TIMEOUT && CONFIG_USER_SHUTDOWN
			// Queue power state request, thread will be suspended when entering system_off
			sys_request_system_off(false);
#endif
			sensor_timeout = SENSOR_SENSOR_TIMEOUT_ACTIVITY_ELAPSED; // only try to suspend once
		}
#endif
#if CONFIG_USE_IMU_TIMEOUT && CONFIG_USE_IMU_WAKE_UP
		if (sensor_timeout == SENSOR_SENSOR_TIMEOUT_IMU && last_data_delta > imu_timeout) // No motion in ramp time
		{
			LOG_INF("No motion from sensors in %llds", imu_timeout / 1000);
			// Queue power state request
			sys_request_WOM(false, false);
			sensor_timeout = SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED; // only try to suspend once
		}
#endif
	}
	else
	{
		if (sensor_mode == SENSOR_SENSOR_MODE_LOW_POWER_2 || sensor_timeout == SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED)
			last_suspend_attempt_time = k_uptime_get();
		last_data_time = k_uptime_get();
		if (sensor_timeout == SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED) // Resetting IMU timeout
			sensor_timeout = SENSOR_SENSOR_TIMEOUT_IMU;
		sensor_mode = SENSOR_SENSOR_MODE_LOW_NOISE;
	}
}

int sensor_init(void)
{
	int err;
	// TODO: on any errors set main_ok false and skip (make functions return nonzero)
	if (mag_available) // shutdown magnetometer first (in case of passthrough)
		sensor_mag->shutdown(); // TODO: is this needed?
	sensor_imu->shutdown(); // TODO: is this needed?

	float clock_actual_rate = 0;
#if CONFIG_USE_SENSOR_CLOCK
	set_sensor_clock(true, 32768, &clock_actual_rate); // enable the clock source for IMU if present
#endif
	if (clock_actual_rate != 0)
		LOG_INF("Sensor clock rate: %.2fHz", (double)clock_actual_rate);

	// wait for sensor register reset // TODO: is this needed?
	k_usleep(250);

	// set FS/range
	float accel_range = CONFIG_SENSOR_ACCEL_FS;
	float gyro_range = CONFIG_SENSOR_GYRO_FS;
	float accel_actual_range, gyro_actual_range;
	sensor_imu->update_fs(accel_range, gyro_range, &accel_actual_range, &gyro_actual_range);
	LOG_INF("Accelerometer range: %.2fg", (double)accel_actual_range);
	LOG_INF("Gyroscope range: %.2fdps", (double)gyro_actual_range);

	// setup sensor, set ODR
	float accel_initial_time = 1.0 / CONFIG_SENSOR_ACCEL_ODR; // configure with ~1000Hz ODR
	float gyro_initial_time = 1.0 / CONFIG_SENSOR_GYRO_ODR; // configure with ~1000Hz ODR
	float mag_initial_time = sensor_update_time_ms / 1000.0; // configure with ~200Hz ODR
	err = sensor_imu->init(clock_actual_rate, accel_initial_time, gyro_initial_time, &accel_actual_time, &gyro_actual_time);
	sensor_actual_time = MIN(accel_actual_time, gyro_actual_time);
#if SENSOR_IMU_SPI_EXISTS
	LOG_INF("Requested SPI frequency: %.2fMHz", (double)sensor_imu_spi_dev.config.frequency / 1000000.0);
#endif
	LOG_INF("Accelerometer initial rate: %.2fHz", 1.0 / (double)accel_actual_time);
	LOG_INF("Gyrometer initial rate: %.2fHz", 1.0 / (double)gyro_actual_time);
	if (err < 0)
		return err;
// 55-66ms to wait, get chip ids, and setup icm (50ms spent waiting for accel and gyro to start)
	if (mag_available && mag_enabled)
	{
		// TODO: need to flag passthrough enabled
//			sensor_imu->ext_passthrough(true); // reenable passthrough
		err = sensor_mag->init(mag_initial_time, &mag_actual_time); // configure with ~200Hz ODR
#if SENSOR_MAG_SPI_EXISTS
		LOG_INF("Requested SPI frequency: %.2fMHz", (double)sensor_mag_spi_dev.config.frequency / 1000000.0);
#endif
		LOG_INF("Magnetometer initial rate: %.2fHz", 1.0 / (double)mag_actual_time);
		if (err < 0)
			return err;
// 0-1ms to setup mmc
	}
	LOG_INF("Initialized sensors");

	// Setup fusion
	sensor_retained_read(); // TODO: useless
	if (fusion_id == FUSION_VQF)
		vqf_update_sensor_ids(sensor_imu_id);
	if (retained->fusion_id == fusion_id) // Check if the retained fusion data is valid and matches the selected fusion
	{ // Load state if the data is valid (fusion was initialized before)
		sensor_fusion->load(retained->fusion_data);
		retained->fusion_id = 0; // Invalidate retained fusion data
		retained_update();
	}
	else
	{
		sensor_fusion->init(gyro_actual_time, accel_actual_time, mag_initial_time); // TODO: using initial time since mag are not polled at the actual rate
	}

	sensor_calibration_update_sensor_ids(sensor_imu_id);
	if (sensor_imu == &sensor_imu_bmi270) // bmi270 specific
	{
		LOG_INF("Applying gyroscope gain");
		bmi270_gain_apply(sensor_calibration_get_sensor_data());
	}

#if IMU_INT_EXISTS
	// Setup interrupt
	float fifo_threshold = sensor_update_time_ms / 1000.0f / sensor_actual_time; // target loop rate
	sensor_fifo_threshold = fifo_threshold;
	LOG_INF("FIFO THS/WM/WTM: %.2f -> %d", (double)fifo_threshold, sensor_fifo_threshold);
	uint8_t pin_config = sensor_imu->setup_DRDY(sensor_fifo_threshold);
	if (pin_config == 0)
		return -1;
	uint32_t int0_gpios = NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, int0_gpios);
	LOG_INF("FIFO THS/WM/WTM GPIO pin: %u, config: %u", int0_gpios, pin_config);
	uint32_t pull_flags = ((pin_config >> 4) == NRF_GPIO_PIN_PULLDOWN ? GPIO_PULL_DOWN : 0) | ((pin_config >> 4) == NRF_GPIO_PIN_PULLUP ? GPIO_PULL_UP : 0);
	gpio_pin_configure_dt(&int0, GPIO_INPUT | pull_flags);
	uint32_t int_flags = ((pin_config & 0xF) == NRF_GPIO_PIN_SENSE_LOW ? GPIO_INT_EDGE_FALLING : 0) | ((pin_config & 0xF) == NRF_GPIO_PIN_SENSE_HIGH ? GPIO_INT_EDGE_RISING : 0);
	gpio_pin_interrupt_configure_dt(&int0, int_flags);
	gpio_init_callback(&sensor_cb_data, sensor_interrupt_handler, BIT(int0.pin));
	gpio_add_callback(int0.port, &sensor_cb_data);
#else
	LOG_WRN("IMU FIFO THS/WM/WTM GPIO does not exist");
	LOG_WRN("IMU FIFO THS/WM/WTM not available");
#endif

	LOG_INF("Using %s", fusion_names[fusion_id]);
	LOG_INF("Initialized fusion");
	sensor_fusion_init = true;
	return 0;
}

static bool main_ok = false;

static int packet_errors = 0;

#define ACQUISITION_START_MS 1000
#define STATUS_INTERVAL_MS 5000

static int64_t last_status_time = 0;
static int64_t max_loop_time = 0;

#if DEBUG
static int64_t last_acquisition_time = INT64_MAX;
static uint64_t total_acquisition_time = 0;
static uint64_t total_read_packets = 0;
static uint64_t total_processed_packets = 0;
static uint64_t total_gyro_samples = 0;
static uint64_t total_accel_samples = 0;
static uint64_t total_loop_time = 0;
static uint64_t total_loop_iterations = 0;
#endif

void sensor_loop(void)
{
	if (!sensor_sensor_init)
		return;
	main_running = true;
	sys_interface_resume(); // make sure interfaces are enabled
	int err = sensor_init(); // Initialize IMUs and Fusion // TODO: run as thread before loop
	// TODO: handle imu init error, maybe restart device?
	// TODO: on failure to init, disable sensor interface
	if (err)
		set_status(SYS_STATUS_SENSOR_ERROR, true); // TODO: only handles general init error
	else
		main_ok = true;
	while (1)
	{
		int64_t time_begin = k_uptime_get();
		if (main_ok)
		{
#if DEBUG
			int64_t loop_begin = k_uptime_ticks();
#endif
			// Resume devices
			sys_interface_resume();

			// Trigger reconfig on sensor mode change
			bool reconfig = last_sensor_mode != sensor_mode;
			last_sensor_mode = sensor_mode;

			// Reading IMUs will take between 2.5ms (~7 samples, low noise) - 7ms (~33 samples, low power)
			// Magneto sample will take ~400us
			// Fusing data will take between 100us (~7 samples, low noise) - 500us (~33 samples, low power) for xiofusion
			// TODO: on any errors set main_ok false and skip (make functions return nonzero)

			// At high speed, use oneshot mode to have synced magnetometer data
			// Call before FIFO and get the data after
			if (mag_available && mag_enabled && mag_use_oneshot)
				sensor_mag->mag_oneshot();

			// Read IMU temperature
			float temp = sensor_imu->temp_read(); // TODO: use as calibration data
			connection_update_sensor_temp(temp);

			// Read gyroscope (FIFO)
#if CONFIG_SENSOR_USE_LOW_POWER_2
			uint8_t* rawData = (uint8_t*)k_malloc(1900);  // Limit FIFO read to 2048 bytes (worst case is ICM 20 byte packet at 1000Hz and 100ms update time)
			if (rawData == NULL)
			{
				LOG_ERR("Failed to allocate memory for FIFO buffer");
				set_status(SYS_STATUS_SENSOR_ERROR, true);
				main_ok = false;
			}
			uint16_t packets = sensor_imu->fifo_read(rawData, 1900); // TODO: name this better?
#else
			uint8_t* rawData = (uint8_t*)k_malloc(1024);  // Limit FIFO read to 768 bytes (worst case is ICM 20 byte packet at 1000Hz and 33ms update time)
			if (rawData == NULL)
			{
				LOG_ERR("Failed to allocate memory for FIFO buffer");
				set_status(SYS_STATUS_SENSOR_ERROR, true);
				main_ok = false;
			}
			uint16_t packets = sensor_imu->fifo_read(rawData, 1024); // TODO: name this better?
#endif

			// Debug info
#if DEBUG
			int64_t acquisition_time = k_uptime_ticks();
			bool valid_acquisition = k_uptime_get() > ACQUISITION_START_MS && last_acquisition_time < acquisition_time; // wait before beginning profiling
			if (valid_acquisition)
			{
				total_acquisition_time += acquisition_time - last_acquisition_time;
				total_read_packets += packets;
			}
			last_acquisition_time = acquisition_time;
#endif

			// Read magnetometer
			float raw_m[3];
			if (mag_available && mag_enabled)
				sensor_mag->mag_read(raw_m); // reading mag last, and it will be processed last

			if (reconfig) // TODO: get rid of reconfig?
			{
				// Changing FIFO threshold here should be fine since FIFO is empty now
				// TODO: causing warnings since packet processing and loop timing still expects previous update_time
				switch (sensor_mode)
				{
				case SENSOR_SENSOR_MODE_LOW_NOISE:
					set_update_time_ms(6);
					LOG_INF("Switching sensors to low noise");
					break;
				case SENSOR_SENSOR_MODE_LOW_POWER:
					set_update_time_ms(33);
					LOG_INF("Switching sensors to low power");
					break;
				case SENSOR_SENSOR_MODE_LOW_POWER_2:
					set_update_time_ms(100);
					LOG_INF("Switching sensors to low power 2");
					break;
				};
			}

			// Suspend devices
			sys_interface_suspend();

			// Fuse all data
			int g_count = 0;
			float a_sum[3] = {0};
			int a_count = 0;
			max_gyro_speed_square = 0;
			int processed_packets = 0;
			for (uint16_t i = 0; i < packets; i++)
			{
				float raw_a[3] = {0};
				float raw_g[3] = {0};
				if (sensor_imu->fifo_process(i, rawData, raw_a, raw_g))
					continue; // skip on error

				// TODO: split into separate functions
				if (raw_g[0] != 0 || raw_g[1] != 0 || raw_g[2] != 0)
				{
#if DEBUG
					if (valid_acquisition)
						total_gyro_samples++;
#endif
					sensor_calibration_process_gyro(raw_g);
					float gx = raw_g[0];
					float gy = raw_g[1];
					float gz = raw_g[2];
					float g[] = {gx, gy, gz};

					// Process fusion
					sensor_fusion->update_gyro(g, gyro_actual_time);

					g_count++;

					if (mag_available && mag_enabled)
					{
						// Get fusion's corrected gyro data (or get gyro bias from fusion) and use it here
						float g_off[3] = {};
						sensor_fusion->get_gyro_bias(g_off);
						for (int i = 0; i < 3; i++)
							g_off[i] = g[i] - g_off[i];
	
						// Get the highest gyro speed
						float gyro_speed_square = g_off[0] * g_off[0] + g_off[1] * g_off[1] + g_off[2] * g_off[2];
						if (gyro_speed_square > max_gyro_speed_square)
							max_gyro_speed_square = gyro_speed_square;
					}
				}

				if (raw_a[0] != 0 || raw_a[1] != 0 || raw_a[2] != 0)
				{
#if DEBUG
					if (valid_acquisition)
						total_accel_samples++;
#endif
					sensor_calibration_process_accel(raw_a);
					float ax = raw_a[0];
					float ay = raw_a[1];
					float az = raw_a[2];
					float a[] = {ax, ay, az};

					// Process fusion
					sensor_fusion->update_accel(a, accel_actual_time);

					for (int i = 0; i < 3; i++)
						a_sum[i] += a[i];
					a_count++;
				}

				processed_packets++;
			}

			// If sensors have asymmetric packets in FIFO, timesteps will not match packet count
			int processed_timesteps = MAX(g_count, a_count);

			// Free the FIFO buffer
			k_free(rawData);

#if DEBUG
			if (valid_acquisition)
				total_processed_packets += processed_packets;
#endif

			if (mag_available && mag_enabled)
			{
				bool mag_calibrated = true;
				float uncalibrated_m[3] = {0};
				memcpy(uncalibrated_m, raw_m, sizeof(uncalibrated_m)); // copy raw magnetometer data
				sensor_calibration_process_mag(raw_m);
				float zero_m[3] = {0};
				if (v_epsilon(raw_m, zero_m, 1e-6)) // if the magnetometer is not calibrated, skip and send raw data
				{
					memcpy(raw_m, uncalibrated_m, sizeof(uncalibrated_m));
					mag_calibrated = false;
				}
				float mx = raw_m[0];
				float my = raw_m[1];
				float mz = raw_m[2];
				float m[] = {SENSOR_MAGNETOMETER_AXES_ALIGNMENT};

				// Process fusion
				if (mag_calibrated)
					sensor_fusion->update_mag(m, sensor_update_time_ms / 1000.0); // TODO: use actual time?

				v_rotate(m, q3, m); // magnetic field in local device frame, no other transformation will be done
				connection_update_sensor_mag(m);
			}

			// Copy average acceleration for this frame
			static float a[3] = {0}; // keep persistent
			if (a_count > 0)
			{
				for (int i = 0; i < 3; i++)
					a[i] = a_sum[i] / a_count;
			}

			// Check packet processing
			if ((packets != 0 || k_uptime_get() > 100) && processed_packets == 0)
			{
				if (packets)
					LOG_WRN("No packets processed");
				else
					LOG_WRN("No packets in buffer");
				if (++packet_errors == 10)
				{
					LOG_ERR("Packet error threshold exceeded");
					set_status(SYS_STATUS_SENSOR_ERROR, true);
					if (packets)
					{
						sensor_retained_write(); // keep the fusion state
						sys_request_system_reboot(false);
					}
				}
			}
			else if (processed_packets == packets && packets > 0)
			{
				packet_errors = 0;
			}

			// Also check if expected number of timesteps when using FIFO threshold
			if (processed_timesteps && processed_timesteps != sensor_fifo_threshold)
				LOG_WRN("Expected %d timesteps, got %d", sensor_fifo_threshold, processed_timesteps);

			// Update fusion gyro sanity? // TODO: use to detect drift and correct or suspend tracking
//			sensor_fusion->update_gyro_sanity(g, m);

			// Get updated quaternion from fusion
			sensor_fusion->get_quat(q);
			q_normalize(q, q); // safe to use self as output

			// Get linear acceleration
			float lin_a[3] = {0};
			if (v_diff_mag(a, lin_a) != 0) // lin_a as zero vector
				a_to_lin_a(q, a, lin_a);

			sensor_update_sensor_state();

			// Update magnetometer mode
			if (mag_available && mag_enabled)
			{
				// TODO: magnetometer might be better to limit to a lower (fixed) rate
				float gyro_speed = sqrtf(max_gyro_speed_square);
				float mag_target_time = 1.0f / (4 * gyro_speed); // target mag ODR for ~0.25 deg error
				if (mag_target_time < 0.005f && mag_skip_oneshot) // only use continuous modes if oneshot is not available
					mag_target_time = 0.005;
				if (mag_target_time > 0.1f) // limit to 0.1 (minimum 10Hz)
					mag_target_time = 0.1;
				sys_interface_resume();
				if (mag_target_time < 0.005f) // cap at 0.005 (200Hz), above this the sensor will use oneshot mode instead
				{
					int err = sensor_mag->update_odr(INFINITY, &mag_actual_time);
					if (mag_actual_time == INFINITY)
					{
						if (!err)
							LOG_DBG("Switching magnetometer to oneshot");
						mag_use_oneshot = true;
					}
					else // magnetometer did not have a oneshot mode, try 200Hz
					{
						if (!err)
							mag_skip_oneshot = true;
						mag_target_time = 0.005;
					}
				}
				if (mag_target_time >= 0.005f || mag_actual_time != INFINITY) // under 200Hz or magnetometer did not have a oneshot mode
				{
					int err = sensor_mag->update_odr(mag_target_time, &mag_actual_time);
					if (!err)
						LOG_DBG("Switching magnetometer ODR to %.2fHz", 1.0 / (double)mag_actual_time);
					mag_use_oneshot = false;
				}
				sys_interface_suspend();
			}

			// Update orientation
			bool send_quat_data = !q_epsilon(q, last_q, 0.001);
			bool send_lin_accel_data = !v_epsilon(lin_a, last_lin_a, 0.05);
			if (send_quat_data || send_lin_accel_data)
			{
				memcpy(last_q, q, sizeof(q));
				memcpy(last_lin_a, lin_a, sizeof(lin_a));
				float q_offset[4];
				q_multiply(q, q3, q_offset); // quaternion in device orientation, connection will change format from wxyz to xyzw
				v_rotate(lin_a, q3, lin_a); // linear acceleration in local device frame, no other transformation will be done
				connection_update_sensor_data(q_offset, lin_a, sensor_data_time);
			}

			// Handle magnetometer calibration
			if (mag_available && mag_enabled && last_sensor_mode == SENSOR_SENSOR_MODE_LOW_POWER && sensor_mode == SENSOR_SENSOR_MODE_LOW_POWER)
				sensor_request_calibration_mag();
				
#if DEBUG
			if (valid_acquisition)
			{
				total_loop_time += k_uptime_ticks() - loop_begin;
				total_loop_iterations++;
			}
#endif
		}

		main_running = false;
		int64_t time_delta = k_uptime_get() - time_begin;

		if (time_delta > sensor_update_time_ms && time_delta > max_loop_time)
			max_loop_time = time_delta;

		if (k_uptime_get() - last_status_time > STATUS_INTERVAL_MS)
		{
			last_status_time = k_uptime_get();
			if (max_loop_time > 0)
			{
				LOG_WRN("Last update steps took up to %lld ms", time_delta);
				max_loop_time = 0;
			}
#if DEBUG
			LOG_DBG("loop iterations: %llu, packets read: %llu, processed: %llu, gyro samples: %llu, accel samples: %llu, total acquisition time: %lld us, total loop time: %lld us", total_loop_iterations, total_read_packets, total_processed_packets, total_gyro_samples, total_accel_samples, k_ticks_to_us_near64(total_acquisition_time), k_ticks_to_us_near64(total_loop_time));
			LOG_DBG("sensor loop rate: %.2fHz, processing time: %.2f/%.2f us -> %.2f%%", (double)total_loop_iterations / (double)k_ticks_to_us_near64(total_acquisition_time) * 1000000.0, (double)k_ticks_to_us_near64(total_loop_time) / (double)total_loop_iterations, (double)k_ticks_to_us_near64(total_acquisition_time) / (double)total_loop_iterations, (double)total_loop_time / (double)total_acquisition_time * 100.0);
			LOG_DBG("reported gyro rate: %.2fHz, actual: %.2fHz, reported accel rate: %.2fHz, actual: %.2fHz", 1.0 / (double)gyro_actual_time, (double)total_gyro_samples / (double)k_ticks_to_us_near64(total_acquisition_time) * 1000000.0, 1.0 / (double)accel_actual_time, (double)total_accel_samples / (double)k_ticks_to_us_near64(total_acquisition_time) * 1000000.0);
#endif
		}

#if IMU_INT_EXISTS
		sensor_data_time = 0; // reset data time
		main_wfi = true; // TODO: this is terrible
		k_msleep(sensor_update_time_ms + 10); // will be resumed by interrupt // TODO: dont use hard timeout
		if (main_wfi) // timeout
		{
			LOG_WRN("Sensor interrupt timeout");
			main_wfi = false;
		}
#else
		// TODO: old behavior
//		led_clock_offset += time_delta;
		if (time_delta > sensor_update_time_ms)
			k_yield();
		else
			k_msleep(sensor_update_time_ms - time_delta);
#endif

		if (main_suspended) // TODO:
			k_thread_suspend(&sensor_thread_id);

		main_running = true;
	}
}

void wait_for_threads(void) // TODO: add timeout
{
	while (main_running)
		k_usleep(1); // bane of my existence. don't use k_yield()!!!!!!
}

void main_imu_suspend(void) // TODO: add timeout
{
	main_suspended = true;
	if (!main_running) // don't suspend if already stopped (TODO: may be called from sensor thread)
		return;
	while (sensor_sensor_scanning)
		k_usleep(1); // try not to interrupt scanning
	while (main_running) // TODO: change to detect if i2c is busy
		k_usleep(1); // try not to interrupt anything actually
	k_thread_suspend(&sensor_thread_id);
	LOG_INF("Suspended sensor thread");
}

void main_imu_resume(void)
{
	if (!main_suspended) // not suspended
		return;
	k_thread_resume(&sensor_thread_id);
	LOG_INF("Resumed sensor thread");
}

void main_imu_wakeup(void)
{
	if (!main_suspended) // don't wake up if pending suspension
		k_wakeup(&sensor_thread_id);
}

void main_imu_restart(void)
{
	if (main_ok) // only restart fusion if initialized
		sensor_fusion->init(gyro_actual_time, accel_actual_time, 6 / 1000.0f); // TODO: using default initial time
}

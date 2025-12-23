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
#include "util.h"
#include "esb.h"
#include "build_defines.h"
#include "hid.h"
#include "system/battery_tracker.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>

static uint8_t tracker_id, batt, batt_v, sensor_temp, imu_id, mag_id, tracker_status, tracker_button;
static uint8_t tracker_svr_status = SVR_STATUS_OK;
static float sensor_q[4], sensor_a[3], sensor_m[3];

static uint8_t data_buffer[21] = {0};
static int64_t last_data_time = 0;
static uint8_t packet_sequence = 0;

LOG_MODULE_REGISTER(connection, LOG_LEVEL_INF);

static void connection_thread(void);
K_THREAD_DEFINE(connection_thread_id, 512, connection_thread, NULL, NULL, NULL, CONNECTION_THREAD_PRIORITY, K_FP_REGS, 0);

void connection_clocks_request_start(void)
{
	clocks_request_start(0);
}

void connection_clocks_request_start_delay_us(uint32_t delay_us)
{
	clocks_request_start(delay_us);
}

void connection_clocks_request_stop(void)
{
	clocks_stop();
}

void connection_clocks_request_stop_delay_us(uint32_t delay_us)
{
	clocks_request_stop(delay_us);
}

uint8_t connection_get_id(void)
{
	return tracker_id;
}

void connection_set_id(uint8_t id)
{
	tracker_id = id;
}

void connection_update_sensor_ids(int imu, int mag)
{
	imu_id = get_server_constant_imu_id(imu);
	// not using get_server_constant_mag_id, does not exist in server enums
	if (mag < 0)
		mag_id = SVR_MAG_STATUS_NOT_SUPPORTED;
	else if CONFIG_1_SETTINGS_READ(CONFIG_1_SENSOR_USE_MAG)
		mag_id = SVR_MAG_STATUS_ENABLED;
	else
		mag_id = SVR_MAG_STATUS_DISABLED;
}

static int64_t quat_update_time = 0;
static int64_t last_quat_time = 0;
static bool send_precise_quat;

void connection_update_sensor_data(float *q, float *a, int64_t data_time)
{
	// data_time is in system ticks, nonzero means valid measurement
	// TODO: use data_time to measure latency! the latency should be calculated up to before radio sent data
	send_precise_quat = q_epsilon(q, sensor_q, 0.005);
	memcpy(sensor_q, q, sizeof(sensor_q));
	memcpy(sensor_a, a, sizeof(sensor_a));
	quat_update_time = k_uptime_get();
}

static int64_t mag_update_time = 0;
static int64_t last_mag_time = 0;

void connection_update_sensor_mag(float *m)
{
	memcpy(sensor_m, m, sizeof(sensor_m));
	mag_update_time = k_uptime_get();
}

void connection_update_sensor_temp(float temp)
{
	// sensor_temp == zero means no data
	if (temp < -38.5f)
		sensor_temp = 1;
	else if (temp > 88.5f)
		sensor_temp = 255;
	else
		sensor_temp = ((temp - 25) * 2 + 128.5f); // -38.5 - +88.5 -> 1-255
}

void connection_update_battery(bool battery_available, bool plugged, bool charged, uint32_t battery_pptt, int battery_mV) // format for packet send
{
	if (!battery_available) // No battery, and voltage is <=1500mV
	{
		batt = 0;
		batt_v = 0;
		return;
	}

	battery_pptt /= 100;
	batt = battery_pptt;
	batt |= 0x80; // battery_available, server will show a battery indicator

	if (charged) // 255, server will show fully charged indicator (not yet)
		batt = 255;

	if (plugged) // Charging
		battery_mV = MAX(battery_mV, 4310); // server will show a charging indicator

	battery_mV /= 10;
	battery_mV -= 245;
	if (battery_mV < 0) // Very dead but it is what it is
		batt_v = 0;
	else if (battery_mV > 255)
		batt_v = 255;
	else
		batt_v = battery_mV; // 0-255 -> 2.45-5.00V
}

void connection_update_status(int status)
{
	tracker_status = status;
	tracker_svr_status = get_server_constant_tracker_status(status);
}

static int64_t button_update_time = 0;

void connection_update_button(int button)
{
	tracker_button = button;
	button_update_time = k_uptime_get();
}

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |id      |packet data                                                                                                                  |
//|0       |id      |batt    |batt_v  |temp    |brd_id  |mcu_id  |resv    |imu_id  |mag_id  |fw_date          |major   |minor   |patch   |rssi    |
//|1       |id      |q0               |q1               |q2               |q3               |a0               |a1               |a2               |
//|2       |id      |batt    |batt_v  |temp    |q_buf                              |a0               |a1               |a2               |rssi    |
//|3	   |id      |svr_stat|status  |resv                                                                                              |rssi    |
//|4       |id      |q0               |q1               |q2               |q3               |m0               |m1               |m2               |
//|5	   |id      |runtime                                                                |resv                                        |rssi    |
//|6       |id      |button  |resv                                                                                                       |rssi    |
//|7       |id      |button  |resv             |q_buf                              |a0               |a1               |a2               |rssi    |

void connection_write_packet_0() // device info
{
	uint8_t data[16] = {0};
	data[0] = 0; // packet 0
	data[1] = tracker_id;
	data[2] = batt;
	data[3] = batt_v;
	data[4] = sensor_temp; // temp
	data[5] = FW_BOARD; // brd_id
	data[6] = FW_MCU; // mcu_id
	data[7] = 0; // resv
	data[8] = imu_id; // imu_id
	data[9] = mag_id; // mag_id
	uint16_t *buf = (uint16_t *)&data[10];
	buf[0] = ((BUILD_YEAR - 2020) & 127) << 9 | (BUILD_MONTH & 15) << 5 | (BUILD_DAY & 31); // fw_date
	data[12] = FW_VERSION_MAJOR & 255; // fw_major
	data[13] = FW_VERSION_MINOR & 255; // fw_minor
	data[14] = FW_VERSION_PATCH & 255; // fw_patch
	data[15] = 0; // rssi (supplied by receiver)
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_1() // full precision quat and accel
{
	uint8_t data[16] = {0};
	data[0] = 1; // packet 1
	data[1] = tracker_id;
	uint16_t *buf = (uint16_t *)&data[2];
	buf[0] = TO_FIXED_15(sensor_q[1]); // ±1.0
	buf[1] = TO_FIXED_15(sensor_q[2]);
	buf[2] = TO_FIXED_15(sensor_q[3]);
	buf[3] = TO_FIXED_15(sensor_q[0]);
	buf[4] = TO_FIXED_7(sensor_a[0]); // range is ±256m/s² or ±26.1g
	buf[5] = TO_FIXED_7(sensor_a[1]);
	buf[6] = TO_FIXED_7(sensor_a[2]);
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_2() // reduced precision quat and accel with battery, temp, and rssi
{
	uint8_t data[16] = {0};
	data[0] = 2; // packet 2
	data[1] = tracker_id;
	data[2] = batt;
	data[3] = batt_v;
	data[4] = sensor_temp; // temp
	float v[3] = {0};
	q_fem(sensor_q, v); // exponential map
	for (int i = 0; i < 3; i++)
		v[i] = (v[i] + 1) / 2; // map -1-1 to 0-1
	uint16_t v_buf[3] = {SATURATE_UINT10((1 << 10) * v[0]), SATURATE_UINT11((1 << 11) * v[1]), SATURATE_UINT11((1 << 11) * v[2])}; // fill 32 bits
	uint32_t *q_buf = (uint32_t *)&data[5];
	*q_buf = v_buf[0] | (v_buf[1] << 10) | (v_buf[2] << 21);

//	v[0] = FIXED_10_TO_DOUBLE(*q_buf & 1023);
//	v[1] = FIXED_11_TO_DOUBLE((*q_buf >> 10) & 2047);
//	v[2] = FIXED_11_TO_DOUBLE((*q_buf >> 21) & 2047);
//	for (int i = 0; i < 3; i++)
//	v[i] = v[i] * 2 - 1;
//	float q[4] = {0};
//	q_iem(v, q); // inverse exponential map

	uint16_t *buf = (uint16_t *)&data[9];
	buf[0] = TO_FIXED_7(sensor_a[0]);
	buf[1] = TO_FIXED_7(sensor_a[1]);
	buf[2] = TO_FIXED_7(sensor_a[2]);
	data[15] = 0; // rssi (supplied by receiver)
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_3() // status
{
	uint8_t data[16] = {0};
	data[0] = 3; // packet 3
	data[1] = tracker_id;
	data[2] = tracker_svr_status;
	data[3] = tracker_status;
	// data[4] - packets received (filled by dongle)
	// data[5] - packets lost (filled by dongle)
	// data[6] - windows hit (filled by dongle)
	// data[7] - windows missed (filled by dongle)
	data[15] = 0; // rssi (supplied by receiver)
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_4() // full precision quat and magnetometer
{
	uint8_t data[16] = {0};
	data[0] = 4; // packet 4
	data[1] = tracker_id;
	uint16_t *buf = (uint16_t *)&data[2];
	buf[0] = TO_FIXED_15(sensor_q[1]);
	buf[1] = TO_FIXED_15(sensor_q[2]);
	buf[2] = TO_FIXED_15(sensor_q[3]);
	buf[3] = TO_FIXED_15(sensor_q[0]);
	buf[4] = TO_FIXED_10(sensor_m[0]); // range is ±32G
	buf[5] = TO_FIXED_10(sensor_m[1]);
	buf[6] = TO_FIXED_10(sensor_m[2]);
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_5() // runtime
{
	uint8_t data[16] = {0};
	data[0] = 5; // packet 5
	data[1] = tracker_id;
	int64_t *buf = (int64_t *)&data[2];
	if (sys_get_valid_battery_pptt() >= 0)
		buf[0] = k_ticks_to_us_floor64(sys_get_battery_remaining_time_estimate());
	else
		buf[0] = -1; // no valid reading yet, but previous estimate may still be valid
	k_mutex_lock(&data_buffer_mutex, K_FOREVER);
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
//	esb_write(data); // TODO: schedule in thread
	k_mutex_unlock(&data_buffer_mutex);
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_6() // reduced precision quat and accel with button
{
	uint8_t data[16] = {0};
	data[0] = 6; // packet 6
	data[1] = tracker_id;
	data[2] = tracker_button;
	data[15] = 0; // rssi (supplied by receiver)
	k_mutex_lock(&data_buffer_mutex, K_FOREVER);
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
	if (tracker_button && k_uptime_get() > button_update_time + 1000) // attempt to "hold" button presses for 1000 ms
	{
		tracker_button = 0;
		button_update_time = 0;
	}
//	esb_write(data); // TODO: schedule in thread
	k_mutex_unlock(&data_buffer_mutex);
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_7() // button
{
	uint8_t data[16] = {0};
	data[0] = 7; // packet 7
	data[1] = tracker_id;
	data[2] = tracker_button;
	float v[3] = {0};
	q_fem(sensor_q, v); // exponential map
	for (int i = 0; i < 3; i++)
		v[i] = (v[i] + 1) / 2; // map -1-1 to 0-1
	uint16_t v_buf[3] = {SATURATE_UINT10((1 << 10) * v[0]), SATURATE_UINT11((1 << 11) * v[1]), SATURATE_UINT11((1 << 11) * v[2])}; // fill 32 bits
	uint32_t *q_buf = (uint32_t *)&data[5];
	*q_buf = v_buf[0] | (v_buf[1] << 10) | (v_buf[2] << 21);
	uint16_t *buf = (uint16_t *)&data[9];
	buf[0] = TO_FIXED_7(sensor_a[0]);
	buf[1] = TO_FIXED_7(sensor_a[1]);
	buf[2] = TO_FIXED_7(sensor_a[2]);
	data[15] = 0; // rssi (supplied by receiver)
	k_mutex_lock(&data_buffer_mutex, K_FOREVER);
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
	if (tracker_button && k_uptime_get() > button_update_time + 1000) // attempt to "hold" button presses for 1000 ms
	{
		tracker_button = 0;
		button_update_time = 0;
	}
//	esb_write(data); // TODO: schedule in thread
	k_mutex_unlock(&data_buffer_mutex);
	hid_write_packet_n(data); // TODO:
}

// TODO: get radio channel from receiver
// TODO: new packet format

// TODO: use timing from IMU to get actual delay in tracking
// TODO: aware of sensor state? error status, timing/phase, maybe "send_precise_quat"

// TODO: queuing, status is lowest priority, info low priority, existing data highest priority (from sensor loop)

// TODO: queue packets directly for HID, or maintain separate loop while connected by USB

static int64_t last_info_time = 0;
static int64_t last_info2_time = 0;
static int64_t last_status_time = 0;
static int64_t last_status2_time = 0;

void connection_thread(void)
{
	bool use_button = !CONFIG_0_SETTINGS_READ(CONFIG_0_USER_EXTRA_ACTIONS); // TODO: until info2 has extra data, it can be disabled for now if extra actions overrides button
	uint8_t data_copy[17];
	// TODO: checking for connection_update events from sensor_loop, here we will time and send them out
	while (1)
	{
		if (last_data_time != 0) // have valid data
		{
			last_data_time = 0;
			memcpy(data_copy, data_buffer, sizeof(data_copy));
			data_copy[16] = packet_sequence++;
			esb_write(data_copy, packet_sequence - 1);
		}
		else if (k_uptime_get() - last_status_time > 1000)
		{
			last_status_time = k_uptime_get();
			connection_write_packet_3();
			continue;
		}
		// mag is higher priority (skip accel, quat is full precision)
		else if (mag_update_time && k_uptime_get() - last_mag_time > 200)
		{
			mag_update_time = 0; // data has been sent
			last_mag_time = k_uptime_get();
			connection_write_packet_4();
			continue;
		}
		// if time for info and precise quat not needed
		else if (quat_update_time && !send_precise_quat && k_uptime_get() - last_info_time > 100)
		{
			quat_update_time = 0;
			last_quat_time = k_uptime_get();
			last_info_time = k_uptime_get();
			connection_write_packet_2();
			continue;
		}
		// if time for info2 and precise quat not needed
		else if (use_button && quat_update_time && !send_precise_quat && k_uptime_get() - last_info2_time > 100)
		{
			quat_update_time = 0;
			last_quat_time = k_uptime_get();
			last_info2_time = k_uptime_get();
			connection_write_packet_7();
			continue;
		}
		// send quat otherwise
		else if (quat_update_time)
		{
			quat_update_time = 0;
			last_quat_time = k_uptime_get();
			connection_write_packet_1();
			continue;
		}
		else if (k_uptime_get() - last_info_time > 500)
		{
			last_info_time = k_uptime_get();
			connection_write_packet_0();
			continue;
		}
		else if (use_button && k_uptime_get() - last_info2_time > 100)
		{
			last_info2_time = k_uptime_get();
			connection_write_packet_6();
			continue;
		}
		else if (k_uptime_get() - last_status_time > 1000)
		{
			last_status_time = k_uptime_get();
			connection_write_packet_3();
			continue;
		}
		else if (k_uptime_get() - last_status2_time > 1000)
		{
			last_status2_time = k_uptime_get();
			connection_write_packet_5();
			continue;
		}
		else
		{
			connection_clocks_request_stop();
		}
		k_msleep(1);
	}
}

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

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

#include <math.h>

static uint8_t tracker_id, batt, batt_v, sensor_temp, imu_id, mag_id, tracker_status;
static uint8_t tracker_svr_status = SVR_STATUS_OK;
static float sensor_q[4], sensor_a[3], sensor_m[3];

static struct k_spinlock sensor_lock;

static uint8_t data_buffer[16] = {0};
static int64_t last_data_time = 0;

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
	mag_id = get_server_constant_mag_id(mag);
}

static int64_t quat_update_time = 0;
static int64_t last_quat_time = 0;
static bool send_precise_quat;

static bool connection_validate_sample(const float *q, const float *a)
{
        float norm = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
        if (fabsf(norm - 1.0f) > 0.1f) {
                LOG_WRN("Rejecting quaternion with norm %f", norm);
                return false;
        }

        for (int i = 0; i < 3; ++i) {
                if (fabsf(a[i]) > 32.0f) {
                        LOG_WRN("Rejecting accel[%d] = %f", i, a[i]);
                        return false;
                }
        }

        return true;
}

void connection_update_sensor_data(float *q, float *a, int64_t data_time)
{
	// data_time is in system ticks, nonzero means valid measurement
	// TODO: use data_time to measure latency! the latency should be calculated up to before radio sent data
        struct k_spinlock_key key = k_spin_lock(&sensor_lock);
        send_precise_quat = q_epsilon(q, sensor_q, 0.005);
        memcpy(sensor_q, q, sizeof(sensor_q));
        memcpy(sensor_a, a, sizeof(sensor_a));
        quat_update_time = k_uptime_get();
        k_spin_unlock(&sensor_lock, key);
}

static int64_t mag_update_time = 0;
static int64_t last_mag_time = 0;

void connection_update_sensor_mag(float *m)
{
        struct k_spinlock_key key = k_spin_lock(&sensor_lock);
        memcpy(sensor_m, m, sizeof(sensor_m));
        mag_update_time = k_uptime_get();
        k_spin_unlock(&sensor_lock, key);
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

void connection_update_battery(bool battery_available, bool plugged, uint32_t battery_pptt, int battery_mV) // format for packet send
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

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |id      |packet data                                                                                                                  |
//|0       |id      |batt    |batt_v  |temp    |brd_id  |mcu_id  |resv    |imu_id  |mag_id  |fw_date          |major   |minor   |patch   |rssi    |
//|1       |id      |q0               |q1               |q2               |q3               |a0               |a1               |a2               |
//|2       |id      |batt    |batt_v  |temp    |q_buf                              |a0               |a1               |a2               |rssi    |
//|3	   |id      |svr_stat|status  |resv                                                                                              |rssi    |

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
//	esb_write(data); // TODO: schedule in thread
	hid_write_packet_n(data); // TODO:
}

static void connection_write_packet_1(const float *q, const float *a) // full precision quat and accel
{
        uint8_t data[16] = {0};
        data[0] = 1; // packet 1
        data[1] = tracker_id;
        uint16_t *buf = (uint16_t *)&data[2];
        buf[0] = TO_FIXED_15(q[1]); // ±1.0
        buf[1] = TO_FIXED_15(q[2]);
        buf[2] = TO_FIXED_15(q[3]);
        buf[3] = TO_FIXED_15(q[0]);
        buf[4] = TO_FIXED_7(a[0]); // range is ±256m/s² or ±26.1g
        buf[5] = TO_FIXED_7(a[1]);
        buf[6] = TO_FIXED_7(a[2]);
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
//	esb_write(data); // TODO: schedule in thread
	hid_write_packet_n(data); // TODO:
}

static void connection_write_packet_2(const float *q, const float *a) // reduced precision quat and accel with battery, temp, and rssi
{
        uint8_t data[16] = {0};
        data[0] = 2; // packet 2
        data[1] = tracker_id;
        data[2] = batt;
        data[3] = batt_v;
        data[4] = sensor_temp; // temp
        float v[3] = {0};
        q_fem(q, v); // exponential map
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
        buf[0] = TO_FIXED_7(a[0]);
        buf[1] = TO_FIXED_7(a[1]);
        buf[2] = TO_FIXED_7(a[2]);
	data[15] = 0; // rssi (supplied by receiver)
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
//	esb_write(data); // TODO: schedule in thread
	hid_write_packet_n(data); // TODO:
}

void connection_write_packet_3() // status
{
	uint8_t data[16] = {0};
	data[0] = 3; // packet 3
	data[1] = tracker_id;
	data[2] = tracker_svr_status;
	data[3] = tracker_status;
	data[15] = 0; // rssi (supplied by receiver)
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
//	esb_write(data); // TODO: schedule in thread
	hid_write_packet_n(data); // TODO:
}

static void connection_write_packet_4(const float *q, const float *m) // full precision quat and magnetometer
{
        uint8_t data[16] = {0};
        data[0] = 4; // packet 4
        data[1] = tracker_id;
        uint16_t *buf = (uint16_t *)&data[2];
        buf[0] = TO_FIXED_15(q[1]);
        buf[1] = TO_FIXED_15(q[2]);
        buf[2] = TO_FIXED_15(q[3]);
        buf[3] = TO_FIXED_15(q[0]);
        buf[4] = TO_FIXED_10(m[0]); // range is ±32G
        buf[5] = TO_FIXED_10(m[1]);
        buf[6] = TO_FIXED_10(m[2]);
	memcpy(data_buffer, data, sizeof(data));
	last_data_time = k_uptime_get(); // TODO: use ticks
//	esb_write(data); // TODO: schedule in thread
	hid_write_packet_n(data); // TODO:
}

// TODO: get radio channel from receiver
// TODO: new packet format

// TODO: use timing from IMU to get actual delay in tracking
// TODO: aware of sensor state? error status, timing/phase, maybe "send_precise_quat"

// TODO: queuing, status is lowest priority, info low priority, existing data highest priority (from sensor loop)

// TODO: queue packets directly for HID, or maintain separate loop while connected by USB

static int64_t last_info_time = 0;
static int64_t last_status_time = 0;

void connection_thread(void)
{
	// TODO: checking for connection_update events from sensor_loop, here we will time and send them out
        while (1)
        {
                if (last_data_time != 0) // have valid data
                {
                        last_data_time = 0;
                        esb_write(data_buffer);
                        continue;
                }

                int64_t now = k_uptime_get();
                float quat[4];
                float accel[3];
                float mag[3];
                bool have_quat = false;
                bool have_mag = false;
                bool precise_quat = false;

                struct k_spinlock_key key = k_spin_lock(&sensor_lock);

                if (mag_update_time && now - last_mag_time > 200)
                {
                        memcpy(quat, sensor_q, sizeof(quat));
                        memcpy(mag, sensor_m, sizeof(mag));
                        mag_update_time = 0;
                        have_mag = true;
                }
                else if (quat_update_time)
                {
                        memcpy(quat, sensor_q, sizeof(quat));
                        memcpy(accel, sensor_a, sizeof(accel));
                        precise_quat = send_precise_quat;
                        quat_update_time = 0;
                        have_quat = true;
                }

                k_spin_unlock(&sensor_lock, key);

                if (have_mag)
                {
                        last_mag_time = now;
                        connection_write_packet_4(quat, mag);
                        continue;
                }

                if (have_quat)
                {
                        if (!connection_validate_sample(quat, accel))
                        {
                                continue;
                        }

                        if (precise_quat && now - last_info_time > 100)
                        {
                                last_quat_time = now;
                                last_info_time = now;
                                connection_write_packet_2(quat, accel);
                        }
                        else
                        {
                                last_quat_time = now;
                                connection_write_packet_1(quat, accel);
                        }
                        continue;
                }

                if (now - last_info_time > 100)
                {
                        last_info_time = now;
                        connection_write_packet_0();
                        continue;
                }

                if (now - last_status_time > 1000)
                {
                        last_status_time = now;
                        connection_write_packet_3();
                        continue;
                }

                connection_clocks_request_stop();
                k_msleep(1); // TODO: should be getting timing from receiver, for now just send asap
        }
}

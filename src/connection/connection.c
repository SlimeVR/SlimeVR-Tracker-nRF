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
#include "connection.h"
#include "util.h"
#include "esb.h"
#include "build_defines.h"
#include "hid.h"
#include "system/battery_tracker.h"
#include "system/clocks.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>

#define ALWAYS_SEND false

static uint8_t tracker_id, batt, batt_v, sensor_temp, imu_id, mag_id, sensor_id, sensor_position, tracker_status, tracker_button;
static uint8_t tracker_svr_status = SVR_STATUS_OK;
static float sensor_q[4], sensor_a[3], sensor_m[3];

static uint8_t raw_data_buffer[ESB_PACKET_MAX_SIZE] = {0};
static uint8_t *data_buffer = &raw_data_buffer[1];
static uint8_t data_buffer_position = 0;
static int64_t last_data_time = 0;
static uint8_t packet_sequence = 0;
static bool allow_packet_bundling = false; // Can only be used with new server and dongle
static bool motion_acked = false;
static enum esb_prtocol_version_t server_protocol = P_VERSION_LEGACY;

LOG_MODULE_REGISTER(connection, LOG_LEVEL_DBG);

struct k_msgq send_packets;
K_MSGQ_DEFINE(send_packets, sizeof(struct packet_t), 10, 1);

void connection_clocks_request_stop(void)
{
	clocks_stop();
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
	else if CONFIG_1_SETTINGS_READ (CONFIG_1_SENSOR_USE_MAG)
		mag_id = SVR_MAG_STATUS_ENABLED;
	else
		mag_id = SVR_MAG_STATUS_DISABLED;
}

static int64_t quat_update_time = 0;
static int64_t last_quat_time = 0;
static bool send_precise_quat;

void connection_update_sensor_data(float *q, float *a, int64_t data_time, uint8_t current_sensor_position)
{
	// data_time is in system ticks, nonzero means valid measurement
	// TODO: use data_time to measure latency! the latency should be calculated up to before radio sent data
	sensor_position = current_sensor_position;
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

static int64_t timeout_time = INT64_MAX;

void connection_update_sensor_timeout_time(int64_t timeout)
{
	timeout_time = timeout;
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

	if (plugged)							// Charging
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

static bool shutdown = false;

void connection_set_shutdown(void)
{
	shutdown = true;
}

bool data_buffer_write(uint8_t *data, size_t size)
{
	if (data_buffer_position + size > ESB_PACKET_MAX_DATA_SIZE)
	{
		LOG_ERR("ESB data buffer overflow. Writing %d, have space for %d", size, ESB_PACKET_MAX_DATA_SIZE - data_buffer_position);
		return false;
	}
	memcpy(data_buffer + data_buffer_position, data, size);
	data_buffer_position += size;
	last_data_time = k_uptime_get(); // TODO: use ticks
	hid_write_packet_n(data);		 // TODO:
	return true;
}

void data_buffer_reset()
{
	data_buffer_position = 0;
}

bool can_send_packet(size_t size)
{
	if (data_buffer_position + size > ESB_PACKET_MAX_DATA_SIZE)
	{
		return false;
	}
	if (data_buffer_position != 0 && !allow_packet_bundling)
	{
		return false;
	}
	return true;
}

//|type    |priority|motion  |precise |interval|description
//|TX     0|       4|        |        |     100|device info ("info")
//|TX     1|       3|*       |*       |       -|full precision quat and accel
//|TX     2|       1|*       |        |     100|reduced precision quat and accel with battery, temp, and rssi ("info")
//|TX     3|       6|        |        |    1000|status ("status")
//|TX     4|       0|*       |*       |     200|full precision quat and magnetometer
//|TX     5|       7|        |        |    1000|runtime ("status2")
//|TX     6|       5|*       |        |     100|reduced precision quat and accel with button and sleep time ("info2")
//|TX     7|       2|        |        |     100|button and sleep time ("info2")

// precise: priority override; interval: target interval in milliseconds

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |id      |packet data                                                                                                                  |
//|TX     0|id      |batt    |batt_v  |temp    |brd_id  |mcu_id  |resv----|imu_id  |mag_id  |fw_date          |major   |minor   |patch   |rssi    |
//|TX     1|id      |q0               |q1               |q2               |q3               |a0               |a1               |a2               |
//|TX     2|id      |batt    |batt_v  |temp    |q_buf                              |a0               |a1               |a2               |rssi    |
//|TX     3|id      |svr_stat|status  |resv----------------------------------------------------------------------------------------------|rssi    |
//|TX     4|id      |q0               |q1               |q2               |q3               |m0               |m1               |m2               |
//|TX     5|id      |runtime                                                                |resv----------------------------------------|rssi    |
//|TX     6|id      |button  |sleeptime        |resv-------------------------------------------------------------------------------------|rssi    |
//|TX     7|id      |button  |sleeptime        |q_buf                              |a0               |a1               |a2               |rssi    |

// runtime is in microseconds (overkill), sleeptime is in milliseconds (overkill but less)

void connection_queue_packet(uint8_t *data, uint8_t length)
{
	if (length > sizeof(((struct packet_t *)0)->data))
	{
		LOG_ERR("Trying too queue too large packet: %d, packet id %d", length, data[0]);
	}
	struct packet_t packet;
	packet.length = length;
	memcpy(&packet.data, data, length);
	k_msgq_put(&send_packets, &packet, K_NO_WAIT);
}

bool connection_write_packet_0() // device info
{
	uint8_t data[16] = {0};
	data[0] = 0; // packet 0
	data[1] = tracker_id;
	data[2] = batt;
	data[3] = batt_v;
	data[4] = sensor_temp; // temp
	data[5] = FW_BOARD;	   // brd_id
	data[6] = FW_MCU;	   // mcu_id
	data[7] = 0;		   // resv
	data[8] = imu_id;	   // imu_id
	data[9] = mag_id;	   // mag_id
	uint16_t *buf = (uint16_t *)&data[10];
	buf[0] = ((BUILD_YEAR - 2020) & 127) << 9 | (BUILD_MONTH & 15) << 5 | (BUILD_DAY & 31); // fw_date
	data[12] = FW_VERSION_MAJOR & 255;														// fw_major
	data[13] = FW_VERSION_MINOR & 255;														// fw_minor
	data[14] = FW_VERSION_PATCH & 255;														// fw_patch
	data[15] = 0;																			// rssi (supplied by receiver)
	return data_buffer_write(data, sizeof(data));
}

bool connection_write_packet_1() // full precision quat and accel
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
	motion_acked = false;
	return data_buffer_write(data, sizeof(data));
}

bool connection_write_packet_2() // reduced precision quat and accel with battery, temp, and rssi
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
		v[i] = (v[i] + 1) / 2;																									   // map -1-1 to 0-1
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
	motion_acked = false;
	return data_buffer_write(data, sizeof(data));
}

bool connection_write_packet_3() // status
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
	fill_packets_stat(data); // Fills 4 fields below
	// data[8] - packets sent (by the tracker)
	// data[9] - packets received (by the tracker)
	// data[10] - packets failed (by the tracker)
	// data[11] - average rssi (received by the tracker)
	// data[12] - repeat packets (filled by dongle)
	// data[13] - largest gap (filled by dongle)
	data[15] = 0; // rssi (supplied by receiver)
	return data_buffer_write(data, sizeof(data));
}

bool connection_write_packet_4() // full precision quat and magnetometer
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
	motion_acked = false;
	return data_buffer_write(data, sizeof(data));
}

bool connection_write_packet_5() // runtime
{
	uint8_t data[16] = {0};
	data[0] = 5; // packet 5
	data[1] = tracker_id;
	int64_t *buf = (int64_t *)&data[2];
	if (sys_get_valid_battery_pptt() >= 0)
		*buf = k_ticks_to_us_floor64(sys_get_battery_remaining_time_estimate());
	else
		*buf = -1; // no valid reading yet, but previous estimate may still be valid
	return data_buffer_write(data, sizeof(data));
}

bool connection_write_packet_6() // reduced precision quat and accel with button and sleep time
{
	uint8_t data[16] = {0};
	data[0] = 6; // packet 6
	data[1] = tracker_id;
	data[2] = tracker_button;
	uint16_t *buf = (uint16_t *)&data[3];
	if (shutdown)
		*buf = 1;
	else
		*buf = timeout_time < 1 ? 1 : timeout_time;
	if (k_ticks_to_ms_floor64(sys_get_battery_remaining_time_estimate()) < 60000 && timeout_time == UINT16_MAX)
		timeout_time = UINT16_MAX - 1;
	data[15] = 0;														// rssi (supplied by receiver)
	if (tracker_button && k_uptime_ticks() > button_update_time + 1000) // attempt to send button press for 1000 ms
	{
		tracker_button = 0;
		button_update_time = 0;
	}
	motion_acked = false;
	return data_buffer_write(data, sizeof(data));
}

bool connection_write_packet_7() // button and sleep time
{
	uint8_t data[16] = {0};
	data[0] = 7; // packet 7
	data[1] = tracker_id;
	data[2] = tracker_button;
	uint16_t *buf = (uint16_t *)&data[3];
	if (shutdown)
		*buf = 1;
	else
		*buf = timeout_time < 1 ? 1 : timeout_time;
	if (k_ticks_to_ms_floor64(sys_get_battery_remaining_time_estimate()) < 60000 && timeout_time == UINT16_MAX)
		timeout_time = UINT16_MAX - 1;
	float v[3] = {0};
	q_fem(sensor_q, v); // exponential map
	for (int i = 0; i < 3; i++)
		v[i] = (v[i] + 1) / 2;																									   // map -1-1 to 0-1
	uint16_t v_buf[3] = {SATURATE_UINT10((1 << 10) * v[0]), SATURATE_UINT11((1 << 11) * v[1]), SATURATE_UINT11((1 << 11) * v[2])}; // fill 32 bits
	uint32_t *q_buf = (uint32_t *)&data[5];
	*q_buf = v_buf[0] | (v_buf[1] << 10) | (v_buf[2] << 21);
	buf = (uint16_t *)&data[9];
	buf[0] = TO_FIXED_7(sensor_a[0]);
	buf[1] = TO_FIXED_7(sensor_a[1]);
	buf[2] = TO_FIXED_7(sensor_a[2]);
	data[15] = 0;														// rssi (supplied by receiver)
	if (tracker_button && k_uptime_ticks() > button_update_time + 1000) // attempt to send button press for 1000 ms
	{
		tracker_button = 0;
		button_update_time = 0;
	}
	motion_acked = false;
	return data_buffer_write(data, sizeof(data));
}

void connection_motion_ack(uint8_t packet_sequence)
{
	motion_acked = true;
	// TODO Check if this sequence is from motion
}

void connection_led_control(uint8_t *data, uint8_t length)
{
	if (length < sizeof(packet_led_control_t))
	{
		LOG_WRN("LED control packet is too short: %d", length);
		return;
	}
	packet_led_control_t *packet = (packet_led_control_t *)data;
	led_override(packet->pattern, packet->r, packet->g, packet->b, packet->brightness, packet->timeout);
}

static void send_device_info()
{
	packet_device_info_t device_info = {
		.packet_id = ESB_PACKET_DEVICE_INFO,
		.tracker_id = tracker_id,
		.hwid = *((uint64_t *)NRF_FICR->DEVICEADDR) & 0xFFFFFFFFFFFF,
		.protocol_version = ESB_TRACKER_PROTOCOL,
		.board_id = FW_BOARD,
		.mcu_id = FW_MCU,
		.board_revision = 0,
		.device_type = DEVICE_TYPE,
		.fw_build_date = ((BUILD_YEAR - 2020) & 127) << 9 | (BUILD_MONTH & 15) << 5 | (BUILD_DAY & 31),
		.fw_major = FW_VERSION_MAJOR & 255,
		.fw_minor = FW_VERSION_MINOR & 255,
		.fw_patch = FW_VERSION_PATCH,
		.sensors_number = SENSORS_NUMBER};

	connection_queue_packet((uint8_t *)&device_info, sizeof(device_info));
}

static void send_sensor_info()
{
	packet_sensor_info_t sensor_info = {
		.packet_id = ESB_PACKET_SENSOR_INFO,
		.tracker_id = tracker_id,
		.sensor_id = 0,
		.imu_id = imu_id,
		.mag_id = mag_id,
		.sensor_state = tracker_svr_status, // TODO Better status
		.def_body_position = sensor_position,
		.target_tps = 100,
		._reserved = 0};
	connection_queue_packet((uint8_t *)&sensor_info, sizeof(sensor_info));
}

void connection_hello(uint8_t *data, uint8_t length)
{
	if (length < sizeof(packet_hello_t))
	{
		LOG_WRN("Hello packet is too short: %d", length);
		return;
	}
	packet_hello_t *packet = (packet_hello_t *)data;
	server_protocol = packet->protocol_version;
	LOG_INF("Hello from server!");
	send_device_info();
	send_sensor_info();
	// TODO Send device state too
}

void connection_packet_received(uint8_t *data, uint8_t length)
{
	if (tracker_id != data[2])
	{
		LOG_WRN("Received packet for wrong tracker id: %d != %d", data[2], tracker_id);
		return;
	}
	// TODO Check sequence for packet loss statistics
	switch (data[3])
	{
	case ESB_PACKET_LED_CONTROL:
		connection_led_control(data, length);
		break;
	case ESB_PACKET_HELLO:
		connection_hello(data, length);
		break;
	}
}

// TODO: use timing from IMU to get actual delay in tracking
// TODO: queue packets directly for HID, or maintain separate loop while connected by USB

static int64_t last_info_time = 0;
static int64_t last_info2_time = 0;
static int64_t last_status_time = 0;
static int64_t last_status2_time = 0;

bool connection_process(void)
{
	bool packet_written = false;
	// Have packets in buffer, send them first
	if (k_msgq_num_used_get(&send_packets) > 0)
	{
		struct packet_t packet;
		int ret = k_msgq_peek(&send_packets, &packet);
		if (ret == 0)
		{
			if (can_send_packet(packet.length))
			{
				ret = k_msgq_get(&send_packets, &packet, K_NO_WAIT);
				if (ret == 0)
				{
					packet_written = data_buffer_write(packet.data, packet.length);
				}
			}
		}
	}
	if (packet_written)
	{
		// Nothing
	}
	// Didn't send status in 2 seconds, prioritize it
	else if (k_uptime_get() - last_status_time > 2000)
	{
		last_status_time = k_uptime_get();
		connection_write_packet_3();
	}
	// mag is higher priority (skip accel, quat is full precision)
	else if (mag_update_time && k_uptime_get() - last_mag_time > 200)
	{
		mag_update_time = 0; // data has been sent
		last_mag_time = k_uptime_get();
		connection_write_packet_4();
	}
	// if time for info and precise quat not needed
	else if (quat_update_time && !send_precise_quat && k_uptime_get() - last_info_time > 100)
	{
		quat_update_time = 0;
		last_quat_time = k_uptime_get();
		last_info_time = k_uptime_get();
		connection_write_packet_2();
	}
	// if time for info2 and precise quat not needed
	else if (quat_update_time && !send_precise_quat && k_uptime_get() - last_info2_time > 100)
	{
		quat_update_time = 0;
		last_quat_time = k_uptime_get();
		last_info2_time = k_uptime_get();
		connection_write_packet_7();
	}
	// send quat otherwise
	else if (quat_update_time)
	{
		quat_update_time = 0;
		last_quat_time = k_uptime_get();
		connection_write_packet_1();
	}
	else if (k_uptime_get() - last_status_time > 1000)
	{
		last_status_time = k_uptime_get();
		connection_write_packet_3();
	}
	else if (server_protocol < P_VERSION_TRANSITIONAL && k_uptime_get() - last_info_time > 500)
	{
		last_info_time = k_uptime_get();
		connection_write_packet_0();
	}
	else if (k_uptime_get() - last_info2_time > 100)
	{
		last_info2_time = k_uptime_get();
		connection_write_packet_6();
	}
	else if (k_uptime_get() - last_status2_time > 1000)
	{
		last_status2_time = k_uptime_get();
		connection_write_packet_5();
	}
	else if (!motion_acked || ALWAYS_SEND) // Didn't ack last motion packet, will send rotation again
	{
		quat_update_time = 0;
		last_quat_time = k_uptime_get();
		connection_write_packet_1();
	}

	if (data_buffer_position != 0) // have valid data
	{
		last_data_time = 0;
		raw_data_buffer[0] = packet_sequence++;
		esb_write(raw_data_buffer, packet_sequence - 1, data_buffer_position + 1);
		data_buffer_reset();
		return true;
	}
	return false;
}

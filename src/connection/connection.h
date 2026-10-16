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
#pragma once

#include "../system/led.h"
#include "esb.h"

void connection_clocks_request_stop(void);

uint8_t connection_get_id(void);
void connection_set_id(uint8_t id);

void connection_update_sensor_ids(int imu_id, int mag_id);
void connection_update_sensor_data(float *q, float *a, int64_t data_time); // ticks
void connection_update_sensor_mag(float *m);
void connection_update_sensor_temp(float temp);
void connection_update_sensor_timeout_time(int64_t timeout);
void connection_update_battery(bool battery_available, bool plugged, bool charged, uint32_t battery_pptt, int battery_mV);
void connection_update_status(int status);
void connection_update_button(int button);

void connection_set_shutdown(void);

bool connection_process(void);

void connection_packet_received(uint8_t * data, uint8_t length);

void connection_motion_ack(uint8_t packet_sequence);

#define ESB_PACKET_DEVICE_INFO 15
#define ESB_PACKET_CUSTOM_BOARD_ID 16
#define ESB_PACKET_SENSOR_INFO 17

#define ESB_PACKET_HELLO 50
#define ESB_PACKET_LED_CONTROL 52

struct packet_t {
	uint8_t length;
	uint8_t data[ESB_PACKET_MAX_SIZE];
};

typedef struct packet_led_control {
	uint8_t sequence;
	uint8_t packet_id;
	uint8_t tracker_id;
	uint8_t sensor_id;
	enum user_led_pattern pattern;
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t brightness;
	uint16_t timeout;
} __packed packet_led_control_t;

// TODO: using packed attribute is maybe not super safe, could use distinct uint8_t type in structs instead and cast from enum
enum server_type {
    SERVER_UNKNOWN = 0,
    SERVER_NORMAL = 1,
    SERVER_ASTERTRACK = 2
} __packed;

enum esb_protocol_version {
    P_VERSION_LEGACY = 0,
    P_VERSION_LEGACY_2 = 2,
    P_VERSION_TRANSITIONAL = 3,
    P_VERSION_MODERN = 4
} __packed;

#define ESB_TRACKER_PROTOCOL P_VERSION_TRANSITIONAL

typedef struct packet_hello {
    uint8_t seq;
    uint8_t packet_id;
	uint8_t tracker_id;
    enum esb_protocol_version protocol_version;
    enum server_type server_type;
    unsigned int : 7;
    unsigned int flag_send_all : 1;
    uint64_t server_time;
} __packed packet_hello_t;

typedef struct packet_device_info {
    uint8_t packet_id;
	uint8_t tracker_id;
	uint64_t hwid;
    enum esb_protocol_version protocol_version;
	uint8_t board_id;
	uint8_t mcu_id;
	uint8_t board_revision;
	uint8_t device_type;
	uint16_t fw_build_date;
	uint8_t fw_major;
	uint8_t fw_minor;
	uint8_t fw_patch;
	uint8_t sensors_number;
} __packed packet_device_info_t;

typedef struct packet_sensor_info {
    uint8_t packet_id;
	uint8_t tracker_id;
	uint8_t sensor_id;
	uint8_t imu_id;
	uint8_t mag_id;
	uint8_t sensor_state;
	uint8_t def_body_position;
	uint16_t target_tps;
	uint8_t _reserved; // Reserved
} __packed packet_sensor_info_t;

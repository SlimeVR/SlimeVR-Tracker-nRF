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

#include <esb.h>
#include <nrfx_timer.h>

#define ESB_VERSION 2
#define PROTOCOL_VERSION 2

#define ESB_PACKET_MAX_SIZE 17
#define ESB_PACKET_DATA_LEGACY_SIZE 16

#define ESB_PACKET_BROADCAST 255
#define ESB_PACKET_DONGLE_PACKETS 200
#define ESB_PACKET_CONTROL_PACKETS 230

#define ESB_PACKET_DONGLE_CONNECT 201
#define ESB_PACKET_DONGLE_CONNECT_REPLY 202
#define ESB_PACKET_DONGLE_RECONNECT 203

#define ESB_PACKET_CONTROL_PAIR_REQEST 231
#define ESB_PACKET_CONTROL_PAIR_RESPONSE 232
#define ESB_PACKET_CONTROL_DONGLE_STATUS 233
#define ESB_PACKET_CONTROL_NO_WINDOWS 234
#define ESB_PACKET_CONTROL_WINDOW_INFO 235

#define ESB_PACKET_CONTROL_TEST 250

#define ESB_DONGLE_FLAG_ACCEPTS_NEW_TRACKERS 0x1
#define ESB_DONGLE_FLAG_FORCE_PAIRING 0x2

#define ESB_STATUS_ERROR 200
#define ESB_STATUS_NO_SLOTS 201
#define ESB_STATUS_NOT_ACCEPTING 202
#define ESB_STATUS_NOT_PAIRED 203

#define ESB_EMPTY_PAYLOAD(_pipe, _length) \
	{                                     \
		.pipe = _pipe,                    \
		.length = _length,                \
		.data = { 0 }                     \
	}

void event_handler(struct esb_evt const *event);
int esb_initialize(bool tx, bool advertize);
void esb_deinitialize(void);
void fill_packets_stat(uint8_t *data);

void esb_set_addr_discovery(void);
void esb_set_addr_paired(uint8_t *paired_addr);

void esb_set_channel(uint8_t channel);
void esb_set_receiver_addr(uint64_t receiver_addr);
int esb_get_frequency(void);

void esb_write(uint8_t *data, uint8_t packet_sequnce); // TODO: give packets some names

bool esb_ready(void);

struct pairing_discovery_t {
	uint64_t dongle_hwid;
	uint8_t channel;
	int8_t rssi;
	uint8_t flags;
	bool rejected;
	bool accepted;
	uint8_t tracker_id;
	uint8_t response;
	uint32_t response_time;
};

enum esb_tracker_state_t {
	NOT_PAIRED,
	PAIRING_FIND_DONGLES,
	PAIRING_PICK_DONGLE,
	PAIRING_WAIT_RESPONSE,
	PAIRING_REJECTED,
	NOT_CONNECTED,
	FIND_DONGLE,
	DONGLE_CONNECT,
	CONNECTED,
	PAIRING_ERROR,
	CONNECTION_ERROR
};

enum esb_tracker_state_t esb_get_tracker_state(void);
void esb_set_tracker_state(enum esb_tracker_state_t state);
// Waits state change from from_state maximum timeout_ms millisconds.
// Returns true if the state was changed or false if the state remained from_state
// when timeout was reached
bool esb_wait_state_change(enum esb_tracker_state_t from_state, uint32_t timeout_ms);

/*
 * Best channels and frequencies to use:
 * 20, 21, 22, 23, 24: 2420MHz to 2424Mhz - BT channels 8, 9, 10 outside of WiFi channels
 * 48, 49, 50, 51, 52: 2448MHz to 2452Mhz - BT channels 21, 22, 23 outside of WiFi channels
 * 72, 73, 74, 75, 76, 77, 78: 2472MHz to 2478Mhz - BT channels 33, 34, 35, 36 outside WiFi channels
 * 82, 83: 2482Mhz to 2483Mhz - outside of used WiFi and BT spectrums
 * Higher channels can be restricted by country.
 *
 * WARNING: Using nearby channels can lead to overlap, i.e. packets sent to Channel 20
 * can be received by a device tuned to Channel 21, so it's recommended to
 * ONLY USE EVEN CHANNELS
 */

#define ESB_RIMARY_ADVERTISEMENT_CHANNEL 22
#define ESB_SECONDARY_ADVERTISEMENT_CHANNEL 4
#define ESB_CHANNELS_AMOUNT 36
#define ESB_CHANNELS 52, 72, 74, 76, 78, 82, 50, 24, 48, 70, 68, 46, 44, 20, 54, 56, 28, 30, 6, 8, 10, 12, 14, 16, 18, 32, 34, 36, 38, 40, 42, 58, 60, 62, 64, 66

#define ESB_SEARCH_TIMEOUT 30000
#define ESB_SEARCH_DONGLES_PAIRING 2000
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
#include "connection.h"
#include "nettests.h"

#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */
#include <zephyr/sys/crc.h>

#include "esb.h"
#include "tdma.h"
#include "util.h"
#include "system/clocks.h"
#include "pairing.h"

#define ALLOW_RETRANSMIT true
#define TX_ERROR_THRESHOLD 10
#define TX_ERROR_MAX 100
#define TX_ERROR_CLEAR_RATE 10
#define SWEEP_TEST false
#define FREQUENCY_HOPPING false

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 512, esb_thread, NULL, NULL, NULL, ESB_THREAD_PRIORITY, 0, 0);

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_EMPTY_PAYLOAD(0, ESB_PACKET_MAX_SIZE);

/*
base_addr_p0: Base address for pipe 0, in big endian.
base_addr_p1: Base address for pipe 1-7, in big endian (not used, set to dongle's address)
pipe_prefixes: Address prefix for pipe 0 to 7.
This was randomly generated
*/
static const uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8}; // Not used
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

static bool esb_initialized = false;
static bool esb_tx = false;
static bool esb_advertize = false;
static uint8_t esb_channel = ESB_RIMARY_ADVERTISEMENT_CHANNEL;
static bool dongle_found = false;
static enum esb_tracker_state_t esb_tracker_state = NOT_PAIRED;
static uint32_t esb_tracker_state_last_change = 0;
static const uint8_t ESB_ALLOWED_CHANNEL_BUNDLES[] = {ESB_CHANNELS};
static uint8_t currentChannelBundle = 0;

uint32_t tx_errors = 0;
int64_t last_tx_success = 0;
int64_t last_tx_fail = 0;
uint8_t last_packet_sequence = 0;
uint8_t packets_sent;
uint8_t packets_received;
uint8_t packets_failed;
uint32_t packets_rssi;
uint64_t last_received_packet = 0;

void found_dongle(uint64_t dongle_hwid, uint8_t channel, int32_t received_time) {
	esb_channel = channel;
	dongle_found = true;
	int32_t time = tdma_get_time();
	int32_t diff = (received_time - time);
	LOG_INF("Our time: %d, rcvd time: %d", time, received_time);
	tdma_update_timer_offset(diff);
	for(int i = 0; i < sizeof(ESB_ALLOWED_CHANNEL_BUNDLES); ++i) {
		if(ESB_ALLOWED_CHANNEL_BUNDLES[i] == channel) {
			currentChannelBundle = i;
			break;
		}
	}
	retained->last_dongle_channel = esb_channel;
	retained_update();
	esb_deinitialize();
	clocks_stop();
	LOG_INF("Dongle successfully found!");
	esb_set_tracker_state(DONGLE_CONNECT);
}

// TODO Split into multiple functions
void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id)
	{
	case ESB_EVENT_TX_SUCCESS:
		last_tx_success = k_uptime_get();
		if (tx_errors > TX_ERROR_CLEAR_RATE)
			tx_errors -= TX_ERROR_CLEAR_RATE;
		else
			tx_errors = 0;
		LOG_DBG("TX SUCCESS");
		// TODO : Better clocks blocking and control based on state
		if (pairing_is_paired())
			clocks_stop();
		break;
	case ESB_EVENT_TX_FAILED:
		last_tx_fail = k_uptime_get();
		if (tx_errors < TX_ERROR_MAX)
			tx_errors++;
		packets_failed++;
		LOG_DBG("TX FAILED, last packet %d", last_packet_sequence);
		// TODO : Better clocks blocking and control based on state
		if (pairing_is_paired())
			clocks_stop();
		break;
	case ESB_EVENT_RX_RECEIVED:
		LOG_DBG("RX");
		int err = 0;
		while (!err) // zero, rx success
		{
			err = esb_read_rx_payload(&rx_payload);
			if (err == -ENODATA) {
				return;
			} else if (err) {
				LOG_ERR("Error while reading rx packet: %d", err);
				return;
			}
			if(rx_payload.length < 2) {
				LOG_ERR("Too short packet received");
				return;
			}

			if (rx_payload.pipe == 0 && rx_payload.length == 8) // Legacy pairing, ignore
			{
				return;
			}

			if(rx_payload.length < 2) {
				LOG_WRN("Too short packet received");
				return;
			}
			
			uint8_t packet_number = rx_payload.data[0];
			if(packet_number != 0 && packet_number != last_packet_sequence) {
				LOG_WRN("Packet number missmatch %d != %d", packet_number, last_packet_sequence);
				break;
			}
			packets_received++;
			packets_rssi += (uint8_t) rx_payload.rssi;
			
			//LOG_INF("Packet %016llX", *(uint64_t *)tx_payload_pair.data);
			if(rx_payload.data[1] > ESB_PACKET_DONGLE_PACKETS) {
				// Control packet received
				switch(rx_payload.data[1]) {
					case ESB_PACKET_CONTROL_DONGLE_STATUS:
						if(rx_payload.length < 16) {
							LOG_ERR("Too short packet received");
							return;
						}
						uint64_t dongle_hwid = *((uint64_t *) &rx_payload.data[2]) & 0xFFFFFFFFFFFF;
						uint8_t channel = rx_payload.data[8];
        				LOG_INF("Found dongle %012llX on channel %d, rssi %d", dongle_hwid, channel, rx_payload.rssi);
						if(pairing_is_paired()) {
							// Looking for dongle when paired
							if(!dongle_found && pairing_get_paired_dongle() == dongle_hwid) {
								found_dongle(dongle_hwid, channel, *((uint32_t *) &rx_payload.data[10]));
							}
						} else {
							pairing_dongle_found(&rx_payload);
						}
					break;
					case ESB_PACKET_CONTROL_PAIR_RESPONSE:
						pairing_dongle_response(&rx_payload);
					break;
					case ESB_PACKET_DONGLE_CONNECT_REPLY:
						uint8_t tracker_id = rx_payload.data[2];
						//uint64_t tracker_hwid = *((uint64_t *) &rx_payload.data[3]) & 0xFFFFFFFFFFFF;
						if(tracker_id < ESB_STATUS_ERROR) {
							connection_set_id(tracker_id);
							esb_set_tracker_state(CONNECTED);
						} else {
							if(tracker_id == ESB_STATUS_NOT_PAIRED) {
								esb_set_tracker_state(NOT_PAIRED);
							} else if(tracker_id == ESB_STATUS_NO_SLOTS) {
								// Shouldn't happen really...
								esb_set_tracker_state(NOT_PAIRED);
							}
							esb_set_tracker_state(CONNECTION_ERROR);
						}
						break;
					case ESB_PACKET_DONGLE_RECONNECT:
						esb_set_tracker_state(DONGLE_CONNECT);
						break;
					case ESB_PACKET_CONTROL_WINDOW_INFO: // Window Info (5)
						int32_t time = tdma_get_time();
						int32_t packet_time = tdma_get_packet_time();
						uint8_t window = rx_payload.data[2];
						tdma_set_our_window(window);
						retained->tdma_window = window;
						int32_t received_time = *((uint32_t *) &rx_payload.data[3]);
						
						// See NTP algorithm
						int32_t diff = ((received_time - packet_time) + (received_time - time)) / 2;
						// int32_t roundtrip_time = time - packet_time;

						tdma_update_timer_offset(diff);
						// if(ABS(diff) != 0) 
						// 	LOG_WRN("Our: %d, packet: %d, dongle's: %d, diff: %d, roundtrip: %d (was slot %d), clock 0x%08x", time, packet_time, received_time, diff, roundtrip_time, tdma_get_slot(packet_time), nrf_clock_lf_src_get(NRF_CLOCK));
						break;
				default:
					LOG_WRN("Unknown control packet %d received", rx_payload.data[2]);
				}
			}
			// if(last_received_packet != 0) {
			// 	uint64_t diff = k_uptime_get() - last_received_packet;
			// 	if(diff > 35) {
			// 		LOG_WRN("Packet gap of %dms", diff);
			// 	}
			// }
			last_received_packet = k_uptime_get();
			connection_motion_ack(packet_number);
		}
		break;
	}
}

void esb_set_tracker_state(enum esb_tracker_state_t state) {
	esb_tracker_state_last_change = k_uptime_get_32();
	esb_tracker_state = state;
	LOG_INF("New state: %d", state);
}

bool esb_wait_state_change(enum esb_tracker_state_t from_state, uint32_t timeout_ms) {
	uint64_t start = k_uptime_get();
	while(esb_tracker_state == from_state && start + timeout_ms > k_uptime_get()) {
		k_msleep(1);
	}
	return esb_tracker_state != from_state;
}

enum esb_tracker_state_t esb_get_tracker_state(void) {
	return esb_tracker_state;
}

void fill_packets_stat(uint8_t *data) {
	data[8] = packets_sent;
	data[9] = packets_received;
	data[10] = packets_failed;
	data[11] = packets_received == 0 ? 0 : packets_rssi / packets_received;
	packets_sent = 0;
	packets_received = 0;
	packets_failed = 0;
	packets_rssi = 0;
}

int esb_initialize(bool tx, bool advertize)
{
	if(esb_initialized) {
		if(tx == esb_tx && advertize == esb_advertize)
			return 0;
		esb_deinitialize();
	}
	esb_tx = tx;
	esb_advertize = advertize;
	esb_initialized = true;
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	if (tx)
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		// config.mode = ESB_MODE_PTX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		config.crc = SWEEP_TEST ? ESB_CRC_OFF : ESB_CRC_16BIT;
		config.tx_output_power = CONFIG_2_SETTINGS_READ(CONFIG_2_RADIO_TX_POWER);
		config.retransmit_delay = 435;
		config.retransmit_count = SWEEP_TEST ? 0 : 1;
		config.tx_mode = ESB_TXMODE_MANUAL;
		config.payload_length = CONFIG_ESB_MAX_PAYLOAD_LENGTH;
		config.selective_auto_ack = true;
		// config.use_fast_ramp_up = false;
	}
	else
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.mode = ESB_MODE_PRX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = CONFIG_2_SETTINGS_READ(CONFIG_2_RADIO_TX_POWER);
		config.retransmit_delay = 435;
		config.retransmit_count = 0;
		// config.tx_mode = ESB_TXMODE_AUTO;
		config.payload_length = CONFIG_ESB_MAX_PAYLOAD_LENGTH;
		config.selective_auto_ack = true;
		// config.use_fast_ramp_up = false;
	}

	err = esb_init(&config);

	if (!err)
	{
		esb_set_base_address_0(base_addr_0);
		esb_set_base_address_1(base_addr_1);
		esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
		esb_set_rf_channel(advertize ? ESB_RIMARY_ADVERTISEMENT_CHANNEL : esb_channel);
	}
	else
	{
		LOG_ERR("ESB initialization failed: %d", err);
		set_status(SYS_STATUS_CONNECTION_ERROR, true);
		esb_initialized = false;
		return err;
	}

	int32_t ch;
	esb_get_rf_channel(&ch);
	LOG_INF("Initialized ESB, %sX mode ch %d", tx ? "T" : "R", ch);

	return 0;
}

void esb_deinitialize(void)
{
	if (esb_initialized)
	{
		esb_initialized = false;
		k_msleep(10); // wait for pending transmissions
		esb_disable();
	}
}

void esb_set_addr_discovery(void)
{
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(base_addr_1, discovery_base_addr_1, sizeof(base_addr_1));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

void esb_set_addr_paired(uint8_t *paired_addr)
{
	// Recreate receiver address
	uint8_t addr_buffer[16] = {0};
	for (int i = 0; i < 4; i++)
	{
		addr_buffer[i] = paired_addr[i + 2];
		addr_buffer[i + 4] = paired_addr[i + 2] + paired_addr[6];
	}
	for (int i = 0; i < 8; i++)
		addr_buffer[i + 8] = paired_addr[7] + i;
	for (int i = 0; i < 16; i++)
	{
		if (addr_buffer[i] == 0x00 || addr_buffer[i] == 0x55 || addr_buffer[i] == 0xAA) // Avoid invalid addresses (see nrf datasheet)
			addr_buffer[i] += 8;
	}
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

void esb_set_receiver_addr(uint64_t receiver_addr) {
	esb_set_addr_paired((uint8_t *) &receiver_addr);
}

int esb_get_frequency(void) {
	uint32_t channel;
	esb_get_rf_channel(&channel);
	return 2400UL + channel; // MHz
}

void esb_write(uint8_t *data, uint8_t packet_sequnce)
{
	if (!esb_initialized || esb_get_tracker_state() != CONNECTED)
		return;
	tx_payload.pipe = 1; // using base address 1
#if defined(NRF54L15_XXAA) // TODO: esb halts with ack and tx fail
	tx_payload.noack = true;
#else
	tx_payload.noack = false;
#endif
	memcpy(tx_payload.data, data, tx_payload.length);
	esb_write_payload();
	last_packet_sequence = packet_sequnce;
	packets_sent++;
}

void esb_write_payload() {
	clocks_start();
	// Wait for our window to broadcast
	while(!tdma_is_our_window())
		k_sleep(K_TICKS(1)); // Spin wait?
	esb_flush_tx(); // this will clear all transmissions even if they did not complete
	esb_write_payload(&tx_payload); // Add transmission to queue
	tdma_tx_started();
#if FREQUENCY_HOPPING
	uint32_t timer = tdma_get_time_with_static_offset();
	uint32_t current_slot = tdma_get_slot(timer);
	esb_set_rf_channel(ESB_ALLOWED_CHANNEL_BUNDLES[(current_slot) % 10]);
#endif
	esb_start_tx();
}

bool esb_ready(void)
{
	return esb_initialized && pairing_is_paired() && dongle_found;
}

void esb_set_channel(uint8_t channel) {
	esb_channel = channel;
}

bool find_dongle() {
	dongle_found = false;
	LOG_INF("Searching for our dongle %012llX...", (*(uint64_t *)&retained->paired_addr[0] >> 16) & 0xFFFFFFFFFFFF);
	esb_initialize(false, true);
	clocks_start();
	esb_start_rx();
	uint64_t start = k_uptime_get();
	while(!dongle_found && start + ESB_SEARCH_TIMEOUT > k_uptime_get()) {
		k_msleep(20);
		if(esb_get_tracker_state() != FIND_DONGLE)
			return true;
	}
	return dongle_found;
}

void populate_connect_payload() {
	uint64_t device_addr = *((uint64_t *) NRF_FICR->DEVICEADDR) & 0xFFFFFFFFFFFF;
	tx_payload.data[0] = 0;
	tx_payload.data[1] = ESB_PACKET_DONGLE_CONNECT;
	tx_payload.data[2] = connection_get_id();
	memcpy(&tx_payload.data[3], &device_addr, 6);
	tx_payload.data[7] = ESB_VERSION;
	tx_payload.data[8] = PROTOCOL_VERSION;
}

void connect_to_dongle() {
	// TODO : Fast connect if we just woke up
	// TODO : Frequency hopping
	esb_initialize(true, false);
	uint64_t start = k_uptime_get();
	while(esb_get_tracker_state() == DONGLE_CONNECT && start + 1000 > k_uptime_get()) {
		populate_connect_payload();
		esb_write_payload();
		k_msleep(15);
	}
	if(esb_get_tracker_state() == DONGLE_CONNECT)
		esb_set_tracker_state(FIND_DONGLE);
}

static void esb_thread(void)
{
	bool use_hid = CONFIG_0_SETTINGS_READ(CONFIG_0_CONNECTION_OVER_HID);
	bool use_shutdown = CONFIG_0_SETTINGS_READ(CONFIG_0_USER_SHUTDOWN);

	pairing_restore();
	esb_channel = retained->last_dongle_channel; // TODO Channel bundles?
	if(esb_channel != 0 && esb_get_tracker_state() == FIND_DONGLE) {
		dongle_found = true;
		esb_set_tracker_state(DONGLE_CONNECT);
	}
	tdma_set_our_window(retained->tdma_window);
	// TODO Restore TDMA timer & frequency hopping
	// But we need RTC for this...

#if SWEEP_TEST
	sweep_test_run();
#endif

	while (1)
	{
		switch(esb_tracker_state) {
			case PAIRING_REJECTED:
				// pairing_rejected_counter++;
				// Fall-trhough
			case NOT_PAIRED:
			case PAIRING_FIND_DONGLES:
				if(!pairing_find_dongles_to_pair()) {
					LOG_WRN("Pairing timeout");
					esb_set_tracker_state(PAIRING_ERROR);
					sys_request_system_off(false);
					return;
				}
			break;
			case PAIRING_PICK_DONGLE:
				if(!pairing_pick_dongle_and_pair()) {
					LOG_WRN("No dongles approved pairing");
					esb_set_tracker_state(PAIRING_ERROR);
					sys_request_system_off(false);
					return;
				}
				break;
			case FIND_DONGLE:
				if(!find_dongle()) {
					LOG_WRN("Couldn't find our dongle");
					esb_set_tracker_state(CONNECTION_ERROR);
					sys_request_system_off(false);
					return;
				}
				break;
			case DONGLE_CONNECT:
				connect_to_dongle();
				break;
			case PAIRING_ERROR:
			case CONNECTION_ERROR:
				// only raise error while not potentially communicating by usb
				if (!get_status(SYS_STATUS_CONNECTION_ERROR) && (!use_hid || !get_status(SYS_STATUS_USB_CONNECTED)))
					set_status(SYS_STATUS_CONNECTION_ERROR, true);
				if (use_shutdown && k_uptime_get() - last_tx_success > CONFIG_3_SETTINGS_READ(CONFIG_3_CONNECTION_TIMEOUT_DELAY)) // shutdown if receiver is not detected // TODO: is shutdown necessary if usb is connected at the time?
				{
					LOG_WRN("No response from receiver in %dm", CONFIG_3_SETTINGS_READ(CONFIG_3_CONNECTION_TIMEOUT_DELAY) / 60000);
					sys_request_system_off(false);
					break;
				}
			break;
			default: // Other states are handled in a different place
				break;
		}
		pairing_save_retained();

		if (tx_errors >= TX_ERROR_THRESHOLD)
		{
			esb_set_tracker_state(FIND_DONGLE); // Try to find dongle again
		}
		else if (tx_errors < TX_ERROR_THRESHOLD && get_status(SYS_STATUS_CONNECTION_ERROR) && k_uptime_get() - last_tx_fail > 3000) // TODO: there is possibly some race condition causing tx_error to potentially be above zero more often than not, so the check is more lenient; tx_error under threshold and last errors above threshold was not recent
		{
			set_status(SYS_STATUS_CONNECTION_ERROR, false);
		}
		k_msleep(100);
	}
}
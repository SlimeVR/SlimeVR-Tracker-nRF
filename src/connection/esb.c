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

#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */
#include <zephyr/sys/crc.h>

#include "esb.h"
#include "tdma.h"
#include "util.h"

#define ESB_CHANNEL 83

uint8_t last_reset = 0;
bool esb_state = false;
bool timer_state = false;
bool send_data = false;
uint16_t led_clock = 0;
uint32_t led_clock_offset = 0;

uint32_t tx_errors = 0;
int64_t last_tx_success = 0;
int64_t last_tx_fail = 0;
uint8_t last_packet_sequence = 0;

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0);

static uint8_t paired_addr[8] = {0};

static bool esb_initialized = false;
static bool esb_paired = false;

#define TX_ERROR_THRESHOLD 30
#define TX_ERROR_MAX 100
#define TX_ERROR_CLEAR_RATE 3

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 512, esb_thread, NULL, NULL, NULL, ESB_THREAD_PRIORITY, 0, 0);

uint8_t packets_sent;
uint8_t packets_received;
uint8_t packets_failed;
uint32_t packets_rssi;

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id)
	{
	case ESB_EVENT_TX_SUCCESS:
		if (tx_errors >= TX_ERROR_THRESHOLD && tx_errors < TX_ERROR_THRESHOLD + TX_ERROR_CLEAR_RATE && last_tx_fail == 0)
		{
			last_tx_success = 0; // reset last_tx_success on threshold reached
			last_tx_fail = k_uptime_get();
		}
		if (tx_errors > TX_ERROR_CLEAR_RATE)
			tx_errors -= TX_ERROR_CLEAR_RATE;
		else
			tx_errors = 0;
		LOG_DBG("TX SUCCESS");
		if (esb_paired)
			clocks_stop();
		break;
	case ESB_EVENT_TX_FAILED:
		if (tx_errors < TX_ERROR_MAX)
			tx_errors++;
		if (tx_errors == TX_ERROR_THRESHOLD && last_tx_success == 0) // consecutive failure to transmit
		{
			last_tx_fail = 0; // reset last_tx_fail on threshold reached
			last_tx_success = k_uptime_get();
		}
		packets_failed++;
		LOG_DBG("TX FAILED");
		if (esb_paired)
			clocks_stop();
		break;
	case ESB_EVENT_RX_RECEIVED:
		// TODO: have to read rx until -ENODATA (or -EACCES/-EINVAL)
		if (!esb_read_rx_payload(&rx_payload)) // zero, rx success
		{
			packets_received++;
			packets_rssi += (uint8_t) rx_payload.rssi;
			if (!paired_addr[0] && rx_payload.length == 8 && rx_payload.pipe == 0) // not paired
			{
				LOG_INF("tx: %16llX rx: %16llX", *(uint64_t *)tx_payload_pair.data, *(uint64_t *)rx_payload.data);
				if (rx_payload.length == 8) // Potentially pairing packet, will try to parse it in the esb thread
					memcpy(paired_addr, rx_payload.data, sizeof(paired_addr));
				return;
			}

			if(rx_payload.data[0] == ESB_CONTROL_PREAMBLE) {
				// Control packet received
				switch(rx_payload.data[1]) {
					case ESB_PACKET_CONTROL_NO_WINDOWS: // No Windows (4)
						// TODO Enter error state and show LED to the user
						break;
					case ESB_PACKET_CONTROL_WINDOW_INFO: // Window Info (5)
						uint8_t packet_number = rx_payload.data[7];
						if(packet_number != last_packet_sequence) {
							LOG_ERR("Window Info (5) packet number missmatch %d != %d", packet_number, last_packet_sequence);
							break;
						}
						int32_t time = tdma_get_time();
						int32_t packet_time = tdma_get_packet_time();
						uint8_t window = rx_payload.data[2];
						tdma_set_our_window(window);
						int32_t received_time = *((uint32_t *) &rx_payload.data[3]);
						
						// See NTP algorithm
						int32_t diff = ((received_time - packet_time) + (received_time - time)) / 2;
						// int32_t roundtrip_time = time - packet_time;

						tdma_update_timer_offset(diff);
						// if(ABS(diff) != 0) 
						// 	LOG_WRN("Our: %d, packet: %d, dongle's: %d, diff: %d, roundtrip: %d (was slot %d), clock 0x%08x", time, packet_time, received_time, diff, roundtrip_time, tdma_get_slot(packet_time), nrf_clock_lf_src_get(NRF_CLOCK));
						break;
				default:
					LOG_INF("Control packet %d received", rx_payload.data[1]);
				}
			}
		}
		break;
	}
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

bool clock_status = false;

#if defined(CONFIG_CLOCK_CONTROL_NRF)
static struct onoff_manager *clk_mgr;

static int clocks_init(void)
{
	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr)
	{
		LOG_ERR("Unable to get the Clock manager");
		return -ENOTSUP;
	}

	return 0;
}

SYS_INIT(clocks_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

int clocks_start(void)
{
	if (clock_status)
		return 0;

	int err;
	int res;
	struct onoff_client clk_cli;
	int fetch_attempts = 0;

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0)
	{
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do
	{
		k_usleep(100);
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res)
		{
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
		if (err && ++fetch_attempts > 10) {
			LOG_WRN_ONCE("Unable to fetch Clock request result: %d", err);
			return err;
		}
	} while (err);

#if defined(NRF54L15_XXAA)
	/* MLTPAN-20 */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* defined(NRF54L15_XXAA) */

	LOG_DBG("HF clock started");
	clock_status = true;
	return 0;
}

void clocks_stop(void)
{
	if (!clock_status)
		return;
	clock_status = false;

	onoff_release(clk_mgr);

	LOG_DBG("HF clock stop request");
}

#else
BUILD_ASSERT(false, "No Clock Control driver");
#endif

static struct k_thread clocks_thread_id;
static K_THREAD_STACK_DEFINE(clocks_thread_id_stack, 128);

void clocks_request_start(uint32_t delay_us)
{
	k_thread_create(&clocks_thread_id, clocks_thread_id_stack, K_THREAD_STACK_SIZEOF(clocks_thread_id_stack), (k_thread_entry_t)clocks_start, NULL, NULL, NULL, CLOCKS_START_THREAD_PRIORITY, 0, K_USEC(delay_us));
}

static struct k_thread clocks_stop_thread_id;
static K_THREAD_STACK_DEFINE(clocks_stop_thread_id_stack, 128);

void clocks_request_stop(uint32_t delay_us)
{
	k_thread_create(&clocks_stop_thread_id, clocks_stop_thread_id_stack, K_THREAD_STACK_SIZEOF(clocks_stop_thread_id), (k_thread_entry_t)clocks_stop, NULL, NULL, NULL, CLOCKS_STOP_THREAD_PRIORITY, 0, K_USEC(delay_us));
}

// this was randomly generated
// TODO: I have no idea?
// TODO: see esb information, check CONFIG_ESB_PIPE_COUNT
/*
base_addr_p0: Base address for pipe 0, in big endian.
base_addr_p1: Base address for pipe 1-7, in big endian.
pipe_prefixes: Address prefix for pipe 0 to 7.
*/
static const uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8}; // TODO: not used
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

int esb_initialize(bool tx)
{
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	if (tx)
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		// config.mode = ESB_MODE_PTX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = CONFIG_2_SETTINGS_READ(CONFIG_2_RADIO_TX_POWER);
		config.retransmit_delay = 600;
		config.retransmit_count = 0;
		config.tx_mode = ESB_TXMODE_MANUAL;
		config.payload_length = 32;
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
		config.retransmit_delay = 600;
		config.retransmit_count = 0;
		// config.tx_mode = ESB_TXMODE_AUTO;
		config.payload_length = 32;
		config.selective_auto_ack = true;
		// config.use_fast_ramp_up = false;
	}

	err = esb_init(&config);

	if (!err)
	{
		esb_set_base_address_0(base_addr_0);
		esb_set_base_address_1(base_addr_1);
		esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
		esb_set_rf_channel(ESB_CHANNEL);
	}
	else
	{
		LOG_ERR("ESB initialization failed: %d", err);
		set_status(SYS_STATUS_CONNECTION_ERROR, true);
		return err;
	}

	int32_t ch;
	esb_get_rf_channel(&ch);
	LOG_INF("Initialized ESB, %sX mode ch %d", tx ? "T" : "R", ch);

	esb_initialized = true;
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
	esb_initialized = false;
}

inline void esb_set_addr_discovery(void)
{
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(base_addr_1, discovery_base_addr_1, sizeof(base_addr_1));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

inline void esb_set_addr_paired(void)
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

void esb_set_pair(uint64_t addr)
{
	uint64_t *device_addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	uint8_t buf[6] = {0};
	memcpy(buf, device_addr, 6);
	uint8_t checksum = crc8_ccitt(0x07, buf, 6);
	if (checksum == 0)
		checksum = 8;
	if ((addr & 0xFF) != checksum)
	{
		LOG_INF("Incorrect checksum");
		return;
	}
	esb_reset_pair();
	memcpy(paired_addr, &addr, sizeof(paired_addr));
	LOG_INF("Paired");
	sys_write(PAIRED_ID, retained->paired_addr, paired_addr, sizeof(paired_addr)); // Write new address and tracker id
}

bool esb_pair(void)
{
	if (get_status(SYS_STATUS_CONNECTION_ERROR))
		set_status(SYS_STATUS_CONNECTION_ERROR, false);
	tx_errors = 0;
	if (!paired_addr[0]) // zero, no receiver paired
	{
		LOG_INF("Pairing");
		esb_set_addr_discovery();
		esb_initialize(true);
		const int64_t time_pairing_start = k_uptime_get();

		tx_payload_pair.pipe = 0;
		tx_payload_pair.noack = false;
		uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
		memcpy(&tx_payload_pair.data[2], addr, 6);
		LOG_INF("Device address: %012llX", *addr & 0xFFFFFFFFFFFF);
		uint8_t checksum = crc8_ccitt(0x07, &tx_payload_pair.data[2], 6);
		if (checksum == 0)
			checksum = 8;
		LOG_INF("Checksum: %02X", checksum);
		tx_payload_pair.data[0] = checksum; // Use checksum to make sure packet is for this device
		set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
		while (paired_addr[0] != checksum && ((*(uint64_t *)&paired_addr[0] >> 16) & 0xFFFFFFFFFFFF) != *addr)
		{
			int64_t time_begin = k_uptime_get();
			if(time_pairing_start + CONFIG_3_SETTINGS_READ(CONFIG_3_CONNECTION_TIMEOUT_DELAY) < time_begin)
				return false; // Pairing timeout

			if (!esb_initialized)
			{
				esb_set_addr_discovery();
				esb_initialize(true);
			}

			if (!clock_status)
				clocks_start();

			if (paired_addr[0])
			{
				LOG_INF("Incorrect checksum: %02X", paired_addr[0]);
				paired_addr[0] = 0; // Packet not for this device
			}

			// send pairing request
			tx_payload_pair.data[1] = 0;
			esb_write_payload(&tx_payload_pair);
			esb_start_tx();
			k_usleep(100);
			while (!esb_is_idle() && (k_uptime_get() < (time_begin + 10)))
				k_usleep(1);

			if (clock_status)
				clocks_stop();

			int64_t time_delta = k_uptime_get() - time_begin;
			if (time_delta > 1000)
				k_yield();
			else
				k_msleep(1000 - time_delta);
		}
		set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_CONNECTION);
		LOG_INF("Paired");
		sys_write(PAIRED_ID, retained->paired_addr, paired_addr, sizeof(paired_addr)); // Write new address and tracker id
		esb_deinitialize();
		k_msleep(1600); // wait for led pattern
	}
	LOG_INF("Tracker ID: %u", paired_addr[1]);
	LOG_INF("Receiver address: %012llX", (*(uint64_t *)&retained->paired_addr[0] >> 16) & 0xFFFFFFFFFFFF);

	connection_set_id(paired_addr[1]);

	esb_set_addr_paired();
	esb_paired = true;
	clocks_stop();
	return true;
}

void esb_reset_pair(void)
{
	if (paired_addr[0] || esb_paired)
	{
		esb_deinitialize(); // make sure esb is off
		esb_paired = false;
		memset(paired_addr, 0, sizeof(paired_addr));
		LOG_INF("Pairing requested");
	}
}

void esb_clear_pair(void)
{
	esb_reset_pair();
	sys_write(PAIRED_ID, &retained->paired_addr, paired_addr, sizeof(paired_addr)); // write zeroes
	LOG_INF("Pairing data reset");
}

int esb_get_frequency(void) {
	uint32_t channel;
	esb_get_rf_channel(&channel);
	return 2400UL + channel; // MHz
}

void esb_write(uint8_t *data, uint8_t packet_sequnce)
{
	if (!esb_initialized || !esb_paired)
		return;
	if (!clock_status)
		clocks_start();
	last_packet_sequence = packet_sequnce;
	tx_payload.pipe = 1; // using base address 1
#if defined(NRF54L15_XXAA) // TODO: esb halts with ack and tx fail
	tx_payload.noack = true;
#else
	tx_payload.noack = false;
#endif
	memcpy(tx_payload.data, data, tx_payload.length);
	esb_flush_tx(); // this will clear all transmissions even if they did not complete
	esb_write_payload(&tx_payload); // Add transmission to queue
	//k_usleep(1);
	// Wait for our window to broadcast
	while(!tdma_is_our_window())
		k_sleep(Z_TIMEOUT_TICKS(1)); // Spin wait?
	tdma_tx_started();
	esb_start_tx();
	packets_sent++;
	send_data = true;
}

bool esb_ready(void)
{
	return esb_initialized && esb_paired;
}

static void esb_thread(void)
{
	bool use_hid = CONFIG_0_SETTINGS_READ(CONFIG_0_CONNECTION_OVER_HID);
	bool use_shutdown = CONFIG_0_SETTINGS_READ(CONFIG_0_USER_SHUTDOWN);
	int64_t start_time = k_uptime_get();

	// Read paired address from retained
	memcpy(paired_addr, retained->paired_addr, sizeof(paired_addr));

	clocks_start();

	// Switch to external oscillator for LF clock for good TDMA precision
	#if defined(NRF_CLOCK_USE_EXTERNAL_LFCLK_SOURCES) || defined(__NRFX_DOXYGEN__)
		nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTOP);
		nrf_clock_lf_src_set(NRF_CLOCK, NRF_CLOCK_LFCLK_XTAL_FULL_SWING);
		nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTART);
	#endif

	while (1)
	{
		if (!esb_paired && (!use_hid || (!get_status(SYS_STATUS_USB_CONNECTED) && k_uptime_get() - 750 > start_time))) // only automatically enter pairing while not potentially communicating by usb
		{
			if(!esb_pair()) {
				LOG_WRN("Pairing timeout");
				sys_request_system_off(false);
			} else {
				esb_initialize(true);
			}
		}
		if (tx_errors >= TX_ERROR_THRESHOLD)
		{
			if (!get_status(SYS_STATUS_CONNECTION_ERROR) && (!use_hid || !get_status(SYS_STATUS_USB_CONNECTED))) // only raise error while not potentially communicating by usb
				set_status(SYS_STATUS_CONNECTION_ERROR, true);
			if (use_shutdown && k_uptime_get() - last_tx_success > CONFIG_3_SETTINGS_READ(CONFIG_3_CONNECTION_TIMEOUT_DELAY)) // shutdown if receiver is not detected // TODO: is shutdown necessary if usb is connected at the time?
			{
				LOG_WRN("No response from receiver in %dm", CONFIG_3_SETTINGS_READ(CONFIG_3_CONNECTION_TIMEOUT_DELAY) / 60000);
				sys_request_system_off(false);
			}
		}
		else if (tx_errors < TX_ERROR_THRESHOLD && get_status(SYS_STATUS_CONNECTION_ERROR) && k_uptime_get() - last_tx_fail > 3000) // TODO: there is possibly some race condition causing tx_error to potentially be above zero more often than not, so the check is more lenient; tx_error under threshold and last errors above threshold was not recent
		{
			set_status(SYS_STATUS_CONNECTION_ERROR, false);
		}
		k_msleep(100);
	}
}



#include "globals.h"
#include "nettests.h"

#if SWEEP_TEST
#include "../system/clocks.h"
#include <zephyr/sys/crc.h>
#include <stdint.h>
#include "esb.h"
#include "../system/led.h"

LOG_MODULE_REGISTER(net_test, LOG_LEVEL_INF);

static struct esb_payload sweep_payload = ESB_EMPTY_PAYLOAD(0, ESB_PACKET_MAX_SIZE); 

void sweep_test_run(void) {
    k_msleep(2000);

	LOG_INF("Executing transmittion sweep test");
	esb_set_addr_discovery();
	k_msleep(3000);
	esb_initialize(true, true);
	sweep_payload.pipe = 0;
	sweep_payload.data[0] = 0;
	sweep_payload.data[1] = ESB_PACKET_CONTROL_TEST;
	uint64_t *device_addr = (uint64_t *)NRF_FICR->DEVICEADDR;
	memcpy(&sweep_payload.data[7], device_addr, 6);
	while(true) {
		for(int i = 0; i <= 84; i+=2) {
			esb_set_channel(i);
			sweep_payload.data[2] = i;
			LOG_INF("Sending on channel %d", i);
			set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_PAIR);
			for(int j = 0; j < 10000; ++j) {
				while(!esb_is_idle())
					k_sleep(K_TICKS(1));
				memcpy(&sweep_payload.data[3], &j, 4);
				sweep_payload.data[13] = 0;
				uint16_t crc = crc16_ansi(&sweep_payload.data[0], 14);
				memcpy(&sweep_payload.data[14], &crc, 2);
				esb_write_payload(&sweep_payload); // Add transmission to queue
				esb_set_rf_channel(i);
				esb_start_tx();
			}
		}
	}
}
#else
void sweep_test_run(void) {}
#endif
#include "nettests.h"

#if SWEEP_TEST
static struct esb_payload sweep_payload = ESB_EMPTY_PAYLOAD(0, ESB_PACKET_MAX_SIZE);

void sweep_test_run(void) {
    k_msleep(2000); // TODO : FOR DEBUG ONLY

	clocks_start();
	clock_init_external();

	LOG_INF("Executing transmittion sweep test");
	esb_set_addr_discovery();
	k_msleep(3000); // TODO : FOR DEBUG ONLY
	esb_initialize(true, true);
	sweep_payload.pipe = 0;
	sweep_payload.data[0] = 0;
	sweep_payload.data[1] = ESB_PACKET_CONTROL_TEST;
	uint64_t *device_addr = (uint64_t *)NRF_FICR->DEVICEADDR;
	memcpy(&sweep_payload.data[7], device_addr, 6);
	while(true) {
		for(int i = 0; i <= 84; i+=2) {
			esb_channel = i;
			sweep_payload.data[2] = esb_channel;
			LOG_INF("Sending on channel %d", esb_channel);
			for(int j = 0; j < 10000; ++j) {
				memcpy(&sweep_payload.data[3], &j, 4);
				sweep_payload.data[13] = bat;
				uint16_t crc = crc16_ansi(&sweep_payload.data[0], 14);
				memcpy(&sweep_payload.data[14], &crc, 2);
				while(!esb_is_idle())
					k_sleep(K_TICKS(1));
				esb_write_payload(&sweep_payload); // Add transmission to queue
				esb_set_rf_channel(esb_channel); 
				esb_start_tx();
			}
		}
	}
}
#else
void sweep_test_run(void) {}
#endif
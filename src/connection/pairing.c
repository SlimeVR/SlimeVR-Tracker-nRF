
#include "globals.h"
#include "pairing.h"
#include "system/system.h"
#include "connection.h"
#include <zephyr/sys/crc.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(pairing, LOG_LEVEL_INF);

static struct pairing_discovery_t* discovered_dongles = NULL;
static struct pairing_discovery_t current_pairing_dongle;
static struct esb_payload tx_payload_pair = ESB_EMPTY_PAYLOAD(0, 15);
static uint8_t paired_addr[8] = {0}; // Paired address bytes: <0> Is Paired | <1> Tracker Id | <2-8> Dongle Address
static bool pairing_needs_saving = false;

void prepare_pair_payload() {
	uint64_t device_addr = *((uint64_t *) NRF_FICR->DEVICEADDR) & 0xFFFFFFFFFFFF;
	tx_payload_pair.pipe = 1;
	tx_payload_pair.noack = false;
	tx_payload_pair.data[0] = 0;
	tx_payload_pair.data[1] = ESB_PACKET_CONTROL_PAIR_REQEST;
	tx_payload_pair.data[2] = 0;
	memcpy(&tx_payload_pair.data[3], &device_addr, 6);
	memcpy(&tx_payload_pair.data[9], &device_addr, 6);
}

void pairing_restore(void) {
    // Read paired address from retained
	memcpy(paired_addr, retained->paired_addr, sizeof(paired_addr));
    if(paired_addr[0] != 0) {
        connection_set_id(paired_addr[1]);
        esb_set_addr_paired(&paired_addr[2]);
        esb_set_tracker_state(FIND_DONGLE);
    }
}

bool pairing_find_dongles_to_pair() {
    esb_set_tracker_state(PAIRING_FIND_DONGLES);
    set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_PAIR);
    // Find dongles around us
    esb_set_addr_discovery();
    esb_initialize(false, true);
    esb_start_rx();
    discovered_dongles = k_calloc(sizeof(struct pairing_discovery_t), ESB_CHANNELS_AMOUNT);
    memset(discovered_dongles, 0, sizeof(struct pairing_discovery_t) * ESB_CHANNELS_AMOUNT);
    // Gather dongles for 2 seconds
    // TODO Move ESB_SEARCH_DONGLES_PAIRING to CONFIG
    if(esb_wait_state_change(PAIRING_FIND_DONGLES, ESB_SEARCH_DONGLES_PAIRING)) {
        return true; // State changed, it's not our problem anymore
    }
    for(int i = 0; i < ESB_CHANNELS_AMOUNT; ++i) {
        struct pairing_discovery_t* dg = &discovered_dongles[i];
        if(dg->dongle_hwid != 0 && (dg->flags & ESB_DONGLE_FLAG_ACCEPTS_NEW_TRACKERS) > 0) {
            esb_set_tracker_state(PAIRING_PICK_DONGLE);
            return true;
        }
    }
	return false;
}

void pairing_dongle_found(const struct esb_payload *payload) {
    if(esb_get_tracker_state() == PAIRING_FIND_DONGLES && discovered_dongles != NULL) {
        uint64_t dongle_hwid = *((uint64_t *) &payload->data[2]) & 0xFFFFFFFFFFFF;
        uint8_t channel = payload->data[8];
        for(int i = 0; i < ESB_CHANNELS_AMOUNT; ++i) {
            struct pairing_discovery_t* dg = &discovered_dongles[i];
            if(dg->dongle_hwid == 0) {
                dg->dongle_hwid = dongle_hwid;
                dg->channel = channel;
                dg->rssi = payload->rssi;
                dg->flags = payload->data[9];
                dg->response = 255;
                dg->response_time = k_uptime_get_32();
                LOG_INF("New dongle for pairing: %012llX, ch %d, rssi %d, flags %d", dongle_hwid, channel, payload->rssi, payload->data[9]);
                break;
            } else if(dg->dongle_hwid == dongle_hwid) {
                dg->rssi = MIN(dg->rssi, payload->rssi);
                break;
            }
        }
    }
}

void pairing_set_pair(uint64_t dongle_hwid, uint8_t tracker_id) {
	paired_addr[0] = 1;
    paired_addr[1] = tracker_id;
	memcpy(&paired_addr[2], &dongle_hwid, 6);
	memcpy(retained->paired_addr, paired_addr, sizeof(paired_addr));
    pairing_needs_saving = true;
	LOG_INF("Pairing info set");
	pairing_restore();
    set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_PAIR);
}

void pairing_save_retained() {
    if(pairing_needs_saving) {
        pairing_needs_saving = false;
        sys_write(PAIRED_ID, retained->paired_addr, paired_addr, sizeof(paired_addr)); // Write new address and tracker id
    }
}

uint64_t pairing_get_paired_dongle() {
    return (*(uint64_t *) &retained->paired_addr[0] >> 16) & 0xFFFFFFFFFFFF;
}

void pairing_dongle_response(const struct esb_payload *payload) {
    if(payload->length < 15) {
        LOG_ERR("Response payload too short");
        return;
    }
    uint64_t device_addr = *((uint64_t *) NRF_FICR->DEVICEADDR) & 0xFFFFFFFFFFFF;
    uint64_t dongle_hwid = *((uint64_t *) &payload->data[3]) & 0xFFFFFFFFFFFF;
    uint64_t tracker_hwid = *((uint64_t *) &payload->data[9]) & 0xFFFFFFFFFFFF;
    uint8_t tracker_id = payload->data[2];
    LOG_INF("Dongle reponse: %012llX, %012llX, %d", dongle_hwid, tracker_hwid, tracker_id);
    if(esb_get_tracker_state() == PAIRING_PICK_DONGLE && current_pairing_dongle.dongle_hwid == dongle_hwid && tracker_hwid == device_addr) {
        if(tracker_id >= 200) { // Rejected
            LOG_INF("Pairing rejected from dongle %012llX, reason: %d", dongle_hwid, tracker_id);
            esb_set_tracker_state(PAIRING_REJECTED);
        } else {
            LOG_INF("Pairing accepted from dongle %012llX, tracker id: %d", dongle_hwid, tracker_id);
            pairing_set_pair(dongle_hwid, tracker_id);
            esb_set_tracker_state(FIND_DONGLE);
        }
        // TODO : Add WAIT_RESPONSE state on specific error codes
        // to confirm from the user that we need to pair
    }
}


int compare_dongles(const void* a, const void* b) {
    const struct pairing_discovery_t* a_d = (const struct pairing_discovery_t*) a;
    const struct pairing_discovery_t* b_d = (const struct pairing_discovery_t*) b;
    // TODO Add foce pair to sorting
    // TODO Double-check sorting...
    return (a_d->rssi < b_d->rssi) - (a_d->rssi > b_d->rssi);
}

bool pairing_pick_dongle_and_pair(void) {
    set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_PAIR);
    int members = 0;
	for(; members < ESB_CHANNELS_AMOUNT; ++members) {
        struct pairing_discovery_t* dg = &discovered_dongles[members];
        if(dg->dongle_hwid == 0) {
            break;
        }
    }
    if(members != 0) {
        qsort(discovered_dongles, members, sizeof(struct pairing_discovery_t), compare_dongles);
    }
    for(int i = 0; i < members; ++i) {
        struct pairing_discovery_t* dg = &discovered_dongles[i];
        if(dg->dongle_hwid != 0 && (dg->flags & ESB_DONGLE_FLAG_ACCEPTS_NEW_TRACKERS) > 0) {
            LOG_INF("Trying to pair to %012llX...", dg->dongle_hwid);
            current_pairing_dongle = *dg;
            esb_set_tracker_state(PAIRING_PICK_DONGLE);
            prepare_pair_payload();
            esb_set_channel(current_pairing_dongle.channel);
            esb_set_receiver_addr(current_pairing_dongle.dongle_hwid);
            esb_initialize(true, false);
            uint32_t start = k_uptime_get_32();
            while(start + 2000 > k_uptime_get_32() && esb_get_tracker_state() == PAIRING_PICK_DONGLE) {
                // TODO If we use channel hopping, we need to do something with timings here
                // We should sync our timer to the received packets
                esb_write_payload(&tx_payload_pair);
                esb_start_tx();
                k_msleep(20);
            }
            // Wait up to 5 minutes for the user to approve the request
            if(!esb_wait_state_change(PAIRING_WAIT_RESPONSE, 300000)) {
                esb_set_tracker_state(PAIRING_REJECTED);
            }
            switch(esb_get_tracker_state()) {
                case PAIRING_REJECTED:
                case PAIRING_PICK_DONGLE:
                    break; // Continue with the next dongle
                case FIND_DONGLE:
                case DONGLE_CONNECT:
                    // Yooo??? dongle?? said "yes"???
                    // By this time we already have everything saved, we go directly to finding it
                    return true;
                default:
                    return true; // State was changed outside, it's not our problem anymore
            }
        }
    }
	return false; // No dongle was found and we're still in the state of finding it
}

void pairing_request_pair(void)
{
    esb_set_tracker_state(PAIRING_FIND_DONGLES);
    LOG_INF("Pairing requested, looking for dongle");
}

void pairing_clear_pair(void)
{
    memset(paired_addr, 0, sizeof(paired_addr));
	sys_write(PAIRED_ID, &retained->paired_addr, paired_addr, sizeof(paired_addr)); // write zeroes
	LOG_INF("Pairing data reset");
	pairing_request_pair();
}
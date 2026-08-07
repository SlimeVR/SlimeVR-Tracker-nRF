#pragma once

#include "esb.h"

void pairing_restore(void);
void pairing_dongle_found(const struct esb_payload *payload);
void pairing_dongle_response(const struct esb_payload *payload);
bool pairing_pick_dongle_and_pair(void);
void pairing_request_pair(void);
void pairing_clear_pair(void);
bool pairing_find_dongles_to_pair(void);
bool pairing_is_paired(void);
void pairing_set_pair(uint64_t dongle_hwid, uint8_t tracker_id);
void pairing_save_retained();
uint64_t pairing_get_paired_dongle();
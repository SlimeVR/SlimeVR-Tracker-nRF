/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 Eiren Rain & SlimeVR Contributors

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
#include "tdma.h"
#include <zephyr/kernel.h>

// TODO Timer skew correction

uint8_t our_window = 0;
uint32_t last_slot = 0;
int32_t timer_offset = 0;
static const int32_t timer_offset_static = 0;
uint32_t packet_sent_time = 0;

LOG_MODULE_REGISTER(tdma, LOG_LEVEL_INF);

uint32_t tdma_get_time() {
    return (k_cycle_get_32() + timer_offset) & TDMA_TIMER_MASK;
}

uint32_t tdma_get_time_with_static_offset() {
    return (k_cycle_get_32() + timer_offset + timer_offset_static) & TDMA_TIMER_MASK;
}

uint32_t tdma_get_packet_time() {
    return (packet_sent_time + timer_offset) & TDMA_TIMER_MASK;
}

uint32_t tdma_get_slot(uint32_t timer) {
    return timer >> TDMA_SLOT_SHIFT;
}

uint8_t tdma_get_window(uint32_t slot) {
    return (slot - TDMA_DONGLE_SLOTS) % TDMA_MAX_TRACKERS;
}

bool tdma_is_dongle_window(uint32_t slot) {
    return slot < TDMA_DONGLE_SLOTS;
}

void tdma_set_our_window(uint8_t window) {
    our_window = window;
}

void tdma_update_timer_offset(int32_t delta) {
	if(delta != 0) {
		timer_offset = timer_offset + delta;
		//LOG_INF("New timer offset %d & %d", timer_offset, timer_offset_static);
	}
}

k_timeout_t tdma_get_time_till_our_window() {
	uint32_t timer = tdma_get_time_with_static_offset();
	uint32_t current_slot = tdma_get_slot(timer);
	uint16_t row = tdma_get_row(current_slot);
	uint16_t target_slot = tdma_get_slot_from_window(row, our_window);
	if(target_slot < current_slot)
		target_slot = tdma_get_slot_from_window(row + 1, our_window);
	uint32_t our_time = tdma_get_slot_time(target_slot);
	return K_CYC(our_time - timer);
}

bool tdma_is_our_window() {
	uint32_t timer = tdma_get_time_with_static_offset();
	uint32_t current_slot = tdma_get_slot(timer);
	if(last_slot == current_slot || current_slot < TDMA_DONGLE_SLOTS)
		return false;
	uint8_t current_window = tdma_get_window(current_slot);
	if(current_window == our_window) {
		last_slot = current_slot;
		return true;
	}
	return false;
}

void tdma_tx_started() {
	packet_sent_time = k_cycle_get_32();
}
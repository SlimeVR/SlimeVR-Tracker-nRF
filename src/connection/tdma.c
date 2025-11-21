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

uint8_t our_window = 0;
uint32_t last_slot = 32757;
int32_t timer_offset = 0;
static int32_t timer_offset_static = -30;
uint32_t packet_sent_time = 0;

LOG_MODULE_REGISTER(tdma, LOG_LEVEL_INF);

uint32_t tdma_get_timer() {
    return k_cycle_get_32() & TDMA_TIMER_MASK;
}

uint32_t tdma_get_timer_with_offsets() {
    return (k_cycle_get_32() + timer_offset + timer_offset_static) & 0x7FFF;
}

uint32_t tdma_get_timer_with_offsets_from_packet() {
    return (packet_sent_time + timer_offset) & 0x7FFF;
}

uint32_t tdma_get_slot(uint32_t timer) {
    return timer >> TDMA_SLOT_SHIFT;
}

uint8_t tdma_get_window(uint32_t slot) {
    return (slot - 24) % 10;
}

bool tdma_is_dongle_window(uint32_t slot) {
    return slot < 24;
}

void tdma_set_our_window(uint8_t window) {
    our_window = window;
}

void tdma_update_timer_offset(int32_t diff) {
    timer_offset = timer_offset + (diff + 1) / 2;
}

bool tdma_is_our_window() {
	int32_t timer = tdma_get_timer_with_offsets();
	uint32_t current_slot = tdma_get_slot(timer);
	if(last_slot == current_slot || current_slot < 24)
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
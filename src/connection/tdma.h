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
#pragma once
#include <stdint.h>
#include "globals.h"

#define TDMA_TIMER_SIZE 32768
#define TDMA_TIMER_MASK 0x7FFF
#define TDMA_SLOT_SIZE 32
#define TDMA_SLOT_SHIFT 5
#define TDMA_SLOTS_COUNT 1024
#define TDMA_MAX_TRACKERS 10
#define TDMA_WRONG_WINDOW 255

#if TDMA_TIMER_SIZE != CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC
	#assert "CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC is not equal TDMA_TIMER_SIZE"
#endif

uint32_t tdma_get_time();
uint32_t tdma_get_time_with_offsets();
uint32_t tdma_get_packet_time();
uint32_t tdma_get_slot(uint32_t timer);
uint8_t tdma_get_window(uint32_t slot);
bool tdma_is_dongle_window(uint32_t slot);
void tdma_set_our_window(uint8_t window);
void tdma_update_timer_offset(int32_t diff);
bool tdma_is_our_window();
void tdma_tx_started();
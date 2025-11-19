/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 SlimeVR Contributors

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
#include "clock_control.h"

#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <hal/nrf_clock.h>

// Safely switch LF clock source
void clock_switch(nrf_clock_lfclk_t source) {
	unsigned int key = irq_lock();

    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTOP);

	uint32_t waited_us = 0;
    while (nrf_clock_lf_is_running(NRF_CLOCK) && (waited_us < 5000)) {
        k_busy_wait(100);
        waited_us += 100;
    }

	nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);

    nrf_clock_lf_src_set(NRF_CLOCK, source);
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTART);

	waited_us = 0;
    while (nrf_clock_lf_is_running(NRF_CLOCK) && (waited_us < 5000)) {
        k_busy_wait(100);
        waited_us += 100;
    }

	nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);

	irq_unlock(key);
}

// Switch to RC clock before shut down to avoid any problems with the bootloader
void clock_pre_shutdown() {
	clock_switch(NRF_CLOCK_LFCLK_RC);
}

// Switch to external oscillator for LF clock for good TDMA precision
void clock_init_external() {
	#if defined(NRF_CLOCK_USE_EXTERNAL_LFCLK_SOURCES) || defined(__NRFX_DOXYGEN__)
		clock_switch(NRF_CLOCK_LFCLK_XTAL_FULL_SWING);
	#endif
}


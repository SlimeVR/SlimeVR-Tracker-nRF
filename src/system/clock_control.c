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
#include <hal/nrf_gpio.h>
#include <hal/nrf_clock.h>

void clock_pre_shutdown()
{
    // Prevent interrupts during clock transition (kernel timing depends on LFCLK)
    unsigned int key = irq_lock();

    // Request LFCLK stop (likely times out — RTC holds the request, this is expected)
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTOP);

    // Wait up to 5ms for stop (won't happen while RTC is active, just a formality)
    uint32_t waited_us = 0;
    while (nrf_clock_lf_is_running(NRF_CLOCK) && (waited_us < 5000))
    {
        k_busy_wait(100); // 100µs steps, uses HFCLK so safe under irq_lock
        waited_us += 100;
    }

    // LFCLKSTARTED is a latched event — clear it so we can detect the new transition
    nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);

    // Select internal RC as new LFCLK source
    nrf_clock_lf_src_set(NRF_CLOCK, NRF_CLOCK_LFCLK_RC);

    // Trigger glitchless shadow transition — hardware keeps external running until RC is stable
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTART);

    // Wait up to 5ms for RC to take over (RC startup is ~600µs, 5ms gives 8x margin)
    waited_us = 0;
    while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED) && (waited_us < 5000))
    {
        k_busy_wait(100);
        waited_us += 100;
    }

    // Clean up event flag
    nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);

    // Restore interrupts — kernel timer now running on RC safely
    irq_unlock(key);
}

void clock_init_external()
{
#if defined(NRF_CLOCK_USE_EXTERNAL_LFCLK_SOURCES) || defined(__NRFX_DOXYGEN__)
    // Prevent interrupts during clock transition
    unsigned int key = irq_lock();

    // Request LFCLK stop (likely times out — RTC holds the request, this is expected)
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTOP);

    // Wait up to 5ms for stop
    uint32_t waited_us = 0;
    while (nrf_clock_lf_is_running(NRF_CLOCK) && (waited_us < 5000))
    {
        k_busy_wait(100);
        waited_us += 100;
    }

    // LFCLKSTARTED is a latched event — clear it so we can detect the new transition
    nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);

    // Select external full swing TCXO as new LFCLK source
    nrf_clock_lf_src_set(NRF_CLOCK, NRF_CLOCK_LFCLK_XTAL_FULL_SWING);

    // Trigger glitchless shadow transition — hardware keeps RC running until TCXO locks
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTART);

    // Wait up to 5ms for TCXO to take over
    waited_us = 0;
    while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED) && (waited_us < 5000))
    {
        k_busy_wait(100);
        waited_us += 100;
    }

    // Clean up event flag
    nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);

    // Restore interrupts
    irq_unlock(key);
#endif
}
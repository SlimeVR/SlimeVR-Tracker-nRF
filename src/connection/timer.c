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
#include "esb.h"
#include "connection.h"

LOG_MODULE_REGISTER(timer, 4);

void timer_handler(nrf_timer_event_t event_type, void *p_context)
{
	if (event_type == NRF_TIMER_EVENT_COMPARE1 && esb_state == true)
	{
		if (last_reset < LAST_RESET_LIMIT)
		{
			last_reset++;
			if (send_data)
			{ // scuffed check
//				esb_write_payload(&tx_payload); // Add transmission to queue
				esb_start_tx();
				send_data = false;
			}
//			esb_flush_tx();
		}
		else
		{
			esb_disable();
			esb_initialize(false);
			esb_start_rx();
			esb_state = false;
//			nrfx_timer_pause(&m_timer);
			timer_state = false;
			LOG_INF("Timer reset elapsed");
		}
	}
	else if (event_type == NRF_TIMER_EVENT_COMPARE2 && esb_state == true)
	{
		esb_disable();
		esb_initialize(false);
		esb_start_rx();
		esb_state = false;
	}
	else if (event_type == NRF_TIMER_EVENT_COMPARE3 && esb_state == false)
	{
		esb_stop_rx();
		esb_disable();
		esb_initialize(true);
		esb_state = true;
	}
}

void timer_init(void)
{
//	uint8_t tracker_id = connection_get_id(); // TODO: make sure it is ready
//	nrfx_err_t err;
//	nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(1000000);
	//timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
	//timer_cfg.mode = NRF_TIMER_MODE_TIMER;
	//timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_16;
	//timer_cfg.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY;
	//timer_cfg.p_context = NULL;
//	nrfx_timer_init(&m_timer, &timer_cfg, timer_handler);
//	uint32_t ticks = nrfx_timer_ms_to_ticks(&m_timer, 3);
//	nrfx_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
//	LOG_INF("Timer at %d", ticks * (tracker_id*2 + 3) / 21); // TODO: temp set max 8
//	nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL1, ticks * (tracker_id*2 + 3) / 21, true); // timeslot to send data  TODO: temp set max 8
//	nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL2, ticks * 19 / 21, true); // switch to rx
//	nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL3, ticks * 2 / 21, true); // switch to tx
//	nrfx_timer_enable(&m_timer);
//	IRQ_DIRECT_CONNECT(TIMER1_IRQn, 0, nrfx_timer_1_irq_handler, 0);
//	irq_enable(TIMER1_IRQn);
	timer_state = true;
}

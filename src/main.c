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
#include "system/system.h"
//#include "timer.h"
#include "connection/esb.h"
#include "sensor/sensor.h"

#include <zephyr/sys/reboot.h>

#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e

static uint32_t *dbl_reset_mem __attribute__((unused)) = ((uint32_t *)DFU_DBL_RESET_MEM); // retained

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios)
#define BUTTON_EXISTS true
#endif

#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
#define ADAFRUIT_BOOTLOADER CONFIG_BUILD_OUTPUT_UF2

int main(void)
{
#ifdef NRF_RESET
	bool reset_pin_reset = NRF_RESET->RESETREAS & RESET_RESETREAS_RESETPIN_Msk;
	NRF_RESET->RESETREAS = NRF_RESET->RESETREAS; // Clear RESETREAS
#else
	bool reset_pin_reset = NRF_POWER->RESETREAS & POWER_RESETREAS_RESETPIN_Msk;
	NRF_POWER->RESETREAS = NRF_POWER->RESETREAS; // Clear RESETREAS
#endif

#if BUTTON_EXISTS
	if (CONFIG_0_SETTINGS_READ(CONFIG_0_IGNORE_RESET))
		reset_pin_reset = false;
#endif

	set_led(SYS_LED_PATTERN_ONESHOT_WAKE, SYS_LED_PRIORITY_BOOT); // Boot LED

	uint8_t reboot_counter = reboot_counter_read();
	bool booting_from_shutdown = !reboot_counter && (reset_pin_reset || button_read()); // 0 means from user shutdown or failed ram validation

	/* if button is not held after booting from shutdown, power off again
	 * if button press is normal, continue boot
	 * if button is held for 1 second, reset pairing and continue boot
	 * if button is held but tracker was waking (not booting from shutdown) ignore the press // TODO: should it pass on to regular handler (e.g. intent to shutdown)
	 */

	if (booting_from_shutdown)
	{
		set_led(SYS_LED_PATTERN_ONESHOT_POWERON, SYS_LED_PRIORITY_BOOT);
		if (button_read())
		{
			set_status(SYS_STATUS_BUTTON_PRESSED, true); // prevent going to sleep while still holding
			int64_t start_time = k_uptime_get();
			while (button_read())
			{
				if (k_uptime_get() - start_time > 1000)
				{
					LOG_INF("Pairing requested");
					esb_reset_pair();
					break;
				}
				k_msleep(1);
			}
			if (CONFIG_0_SETTINGS_READ(CONFIG_0_USER_SHUTDOWN) && k_uptime_get() - start_time < 50) // debounce // TODO: does sense pin stay configured?
				sys_request_system_silent_off(false);
			set_status(SYS_STATUS_BUTTON_PRESSED, false);
		}
	}

	bool docked = dock_read();

	uint8_t reset_mode = -1;

	if (reboot_counter == 0)
		reboot_counter = 100;
	else if (reboot_counter > 200)
		reboot_counter = 200; // How did you get here
	reset_mode = reboot_counter - 100;
	if (reset_pin_reset && !docked) // Count pin resets while not docked
	{
		reboot_counter++;
		reboot_counter_write(reboot_counter);
		LOG_INF("Reset count: %u", reboot_counter);
#if ADAFRUIT_BOOTLOADER
#if BUTTON_EXISTS
		if (!CONFIG_0_SETTINGS_READ(CONFIG_0_IGNORE_RESET))
			(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Using Adafruit bootloader, skip DFU if reset button could be used
#else
		(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Using Adafruit bootloader, skip DFU since reset button is used
#endif
#endif
		k_msleep(1000); // Wait before clearing counter and continuing
	}
	reboot_counter_write(100);
	if (!reset_pin_reset && reset_mode == 0) // Only need to check once, if the button is pressed again an interrupt is triggered from before
		reset_mode = -1; // Cancel reset_mode (shutdown)

	if (CONFIG_0_SETTINGS_READ(CONFIG_0_USER_SHUTDOWN))
	{
		bool charging = chg_read();
		bool charged = stby_read();
		bool plugged = vin_read();

		if (reset_mode == 0 && !booting_from_shutdown && !charging && !charged && !plugged) // Reset mode user shutdown, only if unplugged and undocked
			sys_user_shutdown();
	}

	sys_reset_mode(reset_mode);

	return 0;
}

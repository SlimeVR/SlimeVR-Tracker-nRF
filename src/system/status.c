#include "globals.h"
#include "connection/connection.h"

#include <zephyr/kernel.h>

#include "status.h"
#include "led.h"

static int status_state = 0;

LOG_MODULE_REGISTER(status, LOG_LEVEL_INF);

static void status_error_thread(void);
K_THREAD_DEFINE(status_error_thread_id, 256, status_error_thread, NULL, NULL, NULL, STATUS_ERROR_THREAD_PRIORITY, 0, 0);
static void status_warn_thread(void);
K_THREAD_DEFINE(status_warn_thread_id, 256, status_warn_thread, NULL, NULL, NULL, STATUS_WARN_THREAD_PRIORITY, 0, 0);

void set_status(enum sys_status status, bool set)
{
	if (set)
	{
		status_state |= status;
		switch (status)
		{
		case SYS_STATUS_SENSOR_ERROR:
			LOG_ERR("Sensor communication error");
			k_thread_resume(status_error_thread_id);
			break;
		case SYS_STATUS_CONNECTION_ERROR:
			LOG_WRN("Connection error");
			k_thread_resume(status_warn_thread_id);
			break;
		case SYS_STATUS_SYSTEM_ERROR:
			LOG_ERR("General error");
			k_thread_resume(status_error_thread_id);
			break;
		case SYS_STATUS_USB_CONNECTED:
			LOG_INF("USB connected");
			break;
		case SYS_STATUS_PLUGGED:
			LOG_INF("Charger plugged");
			break;
		case SYS_STATUS_CALIBRATION_RUNNING:
			LOG_INF("Calibration running");
			break;
		case SYS_STATUS_BUTTON_PRESSED:
			LOG_INF("Button pressed");
			break;
		case SYS_STATUS_SERIAL_ACTIVE:
			LOG_INF("Serial connected");
			break;
		default:
			break;
		}
	}
	else
	{
		status_state &= ~status;
		LOG_INF("Cleared status: %d", status);
	}
	connection_update_status(status_state);
	LOG_INF("Status: %d", status_state);
}

int get_status(enum sys_status status)
{
	return status_state & status;
}

static void status_error_thread(void)
{
	while (1) // cycle through errors
	{
		int status = status_state & SYS_STATUS_ERROR;
		if (status & SYS_STATUS_SENSOR_ERROR)
		{
			set_led(SYS_LED_PATTERN_ERROR_A, SYS_LED_PRIORITY_ERROR);
			k_msleep(5000);
		}
		if (status & SYS_STATUS_SYSTEM_ERROR)
		{
			set_led(SYS_LED_PATTERN_ERROR_C, SYS_LED_PRIORITY_ERROR);
			k_msleep(5000);
		}
		if (!status)
		{
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_ERROR);
			k_thread_suspend(status_error_thread_id);
		}
	}
}

static void status_warn_thread(void)
{
	// will only show if there are no errors! they are higher priority than warnings
	while (1) // cycle through warnings
	{
		int status = status_state & SYS_STATUS_WARN;
		if (status & SYS_STATUS_CONNECTION_ERROR)
		{
			set_led(SYS_LED_PATTERN_ERROR_B, SYS_LED_PRIORITY_WARN);
			k_msleep(5000);
		}
		if (!status)
		{
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_WARN);
			k_thread_suspend(status_warn_thread_id);
		}
	}
}

bool status_ready(void) // true if no important statuses are active
{
	return (status_state & ~SYS_STATUS_CONNECTION_ERROR) == 0; // connection error is temporary, not critical
}

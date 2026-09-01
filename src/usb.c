#include "globals.h"
#include "console.h"
#include "hid.h"
#include "system/status.h"

#if CONFIG_USB_DEVICE_STACK
#define USB DT_NODELABEL(usbd)
#define USB_EXISTS (DT_NODE_HAS_STATUS(USB, okay) && CONFIG_UART_CONSOLE)
#endif

#if USB_EXISTS
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/pm/device.h>

static bool configured;

LOG_MODULE_REGISTER(usb, LOG_LEVEL_INF);

static void usb_init_thread(void);
K_THREAD_DEFINE(usb_init_thread_id, 512, usb_init_thread, NULL, NULL, NULL, USB_INIT_THREAD_PRIORITY, 0, 500); // Wait before enabling USB

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	const struct log_backend *backend = log_backend_get_by_name("log_backend_uart");
	const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	bool use_hid = CONFIG_0_SETTINGS_READ(CONFIG_0_CONNECTION_OVER_HID);
	switch (status)
	{
	case USB_DC_RESET:
		configured = false;
		break;
	case USB_DC_CONNECTED:
		set_status(SYS_STATUS_USB_CONNECTED, true);
		pm_device_action_run(cons, PM_DEVICE_ACTION_RESUME);
		log_backend_enable(backend, backend->cb->ctx, CONFIG_LOG_MAX_LEVEL);
		console_thread_create();
		if (use_hid)
			hid_thread_create();
		break;
	case USB_DC_CONFIGURED:
		int configurationIndex = *param;
		if (configurationIndex == 0)
		{
			// from usb_device.c: A configuration index of 0 unconfigures the device.
			configured = false;
		}
		else
		{
			if (!configured)
			{
				hid_int_in_ready();
				configured = true;
			}
		}
		break;
	case USB_DC_DISCONNECTED:
		set_status(SYS_STATUS_USB_CONNECTED, false);
		if (use_hid)
			hid_thread_abort();
		console_thread_abort();
		log_backend_disable(backend);
		pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
		break;
	case USB_DC_SOF:
		break;
	default:
		LOG_DBG("status %u unhandled", status);
		break;
	}
}

static void usb_init_thread(void)
{
	usb_enable(status_cb);
}

#endif
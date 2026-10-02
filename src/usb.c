#include "globals.h"
#include "console.h"
#include "hid.h"
#include "system/status.h"
#include "system/system.h"

#if CONFIG_USB_DEVICE_STACK
#define USB DT_NODELABEL(usbd)
#define USB_EXISTS (DT_NODE_HAS_STATUS(USB, okay) && CONFIG_UART_CONSOLE)
#endif

#if USB_EXISTS
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#define DEV_CONSOLE DEVICE_DT_GET(DT_CHOSEN(zephyr_console))
#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
#define ADAFRUIT_BOOTLOADER CONFIG_BUILD_OUTPUT_UF2
#define NRF5_BOOTLOADER CONFIG_BOARD_HAS_NRF5_BOOTLOADER

static bool configured;

uint32_t dtr;
uint32_t last_dtr = 0;

LOG_MODULE_REGISTER(usb, LOG_LEVEL_INF);

static void usb_init_thread(void);
K_THREAD_DEFINE(usb_init_thread_id, 256, usb_init_thread, NULL, NULL, NULL, USB_INIT_THREAD_PRIORITY, 0, 0);

static void usb_ctrl_thread(void);
static struct k_thread usb_ctrl_thread_id;
static K_THREAD_STACK_DEFINE(usb_ctrl_thread_stack, 256);

#if NRF5_BOOTLOADER
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
#endif

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
		k_thread_create(&usb_ctrl_thread_id, usb_ctrl_thread_stack, K_THREAD_STACK_SIZEOF(usb_ctrl_thread_stack), (k_thread_entry_t)usb_ctrl_thread, NULL, NULL, NULL, CONSOLE_THREAD_PRIORITY, 0, K_NO_WAIT);
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
		k_thread_abort(&usb_ctrl_thread_id);
		pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
		if (dtr)
		{
			LOG_INF("USB disconnected before DTR unset");
			set_status(SYS_STATUS_SERIAL_ACTIVE, false);
			console_thread_abort();
			log_backend_disable(backend);
		}
		break;
	case USB_DC_SOF:
		break;
	default:
		LOG_DBG("status %u unhandled", status);
		break;
	}
}

int64_t start_time;

static void usb_init_thread(void)
{
	start_time = k_uptime_get();
}

static void usb_ctrl_thread(void)
{
	const struct log_backend *backend = log_backend_get_by_name("log_backend_uart");

	// first check dfu on usb plug in
#if DFU_EXISTS
	if (k_uptime_get() - start_time < 100 ? button_read_filtered() : button_read()) // button held on usb connect, enter DFU. if held from wake and usb is plugged in after, then allow initial button hold
	{
#if ADAFRUIT_BOOTLOADER
		NRF_POWER->GPREGRET = 0x57; // DFU_MAGIC_UF2_RESET
		sys_request_system_reboot(false);
#endif
#if NRF5_BOOTLOADER
		gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
#endif
	}
#endif

	// wait for serial to start
	last_dtr = 0;
	while (1)
	{
		uart_line_ctrl_get(DEV_CONSOLE, UART_LINE_CTRL_DTR, &dtr);
		if (dtr == last_dtr)
		{
			k_msleep(10);
		}
		else
		{
			last_dtr = dtr;
			if (dtr)
			{
				set_status(SYS_STATUS_SERIAL_ACTIVE, true);
				console_thread_create();
				log_backend_enable(backend, backend->cb->ctx, CONFIG_LOG_MAX_LEVEL);
			}
			else
			{
#if DFU_EXISTS
				uint32_t baudrate;
				uart_line_ctrl_get(DEV_CONSOLE, UART_LINE_CTRL_BAUD_RATE, &baudrate);
				if (baudrate == 1200)
				{
#if ADAFRUIT_BOOTLOADER
					NRF_POWER->GPREGRET = 0x4e; // DFU_MAGIC_SERIAL_ONLY_RESET
					sys_request_system_reboot(false);
#endif
					// TODO: not sure if nrf5 bootloader can make use of this
#if NRF5_BOOTLOADER
					gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
#endif
				}
#endif
				set_status(SYS_STATUS_SERIAL_ACTIVE, false);
				console_thread_abort();
				log_backend_disable(backend);
			}
		}
	}
}

#endif

void usb_initialize(void)
{
#if USB_EXISTS
	usb_enable(status_cb);
#endif
}

void usb_deinitialize(void)
{
#if USB_EXISTS
	usb_disable();
#endif
}

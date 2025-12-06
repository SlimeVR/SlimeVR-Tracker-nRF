#include "globals.h"
#include "assets.h"
#include "system/battery_tracker.h"
#include "system/status.h"

#include <zephyr/drivers/display.h>
#include <zephyr/device.h>
#include <stdio.h>

static uint8_t clear_buf[2 * 128];

static uint8_t top_line_buf[2 * 96];
static uint8_t bottom_line_buf[2 * 128];

static uint8_t top_line_offset = 0;
static uint8_t bottom_line_offset = 0;

const struct display_buffer_descriptor top_desc = {
	.buf_size = 2 * 96,
	.width = 96,
	.height = 16,
	.pitch = 96,
};

const struct display_buffer_descriptor bottom_desc = {
	.buf_size = 2 * 128,
	.width = 128,
	.height = 16,
	.pitch = 128,
};

const struct display_buffer_descriptor battery_desc = {
	.buf_size = 2 * 32,
	.width = 32,
	.height = 16,
	.pitch = 32,
};

static const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

LOG_MODULE_REGISTER(display, LOG_LEVEL_INF);

static void display_status_thread(void);
K_THREAD_DEFINE(display_status_thread_id, 512, display_status_thread, NULL, NULL, NULL, 8, 0, 0);

static void display_runtime_thread(void);
K_THREAD_DEFINE(display_runtime_thread_id, 512, display_runtime_thread, NULL, NULL, NULL, 8, 0, 0);

static void display_thread(void);
K_THREAD_DEFINE(display_thread_id, 512, display_thread, NULL, NULL, NULL, 8, 0, 0);

static int write_char_to_top_buf(char c)
{
    if (top_line_offset + 7 > 96)
        return -1;
    if (c >= 0x21 && c <= 0x7e)
    {
        uint16_t offset = (c - 0x21) * 7;
        uint8_t width = 7;
        memcpy(top_line_buf + top_line_offset, unifont + offset, width);
        memcpy(top_line_buf + 96 + top_line_offset, unifont + 658 + offset, width);
    }
    top_line_offset += 8;
    return 0;
}

static void display_top_str(char *s)
{
    static char last[13]; // top line can fit at most 12 chars
    if (strncmp(last, s, 13) == 0)
        return;
    strncpy(last, s, 13);

    top_line_offset = 0;
    memset(top_line_buf, 0, 192);
    while (*s)
    {
        if (write_char_to_top_buf(*s))
            break;
        s++;
    }

    display_write(dev, 0, 0, &top_desc, top_line_buf);
}

static int write_char_to_bottom_buf(char c)
{
    if (c == 0x20)
    {
        if (bottom_line_offset + 4 > 128)
            return -1;
        bottom_line_offset += 5;
        return 0;
    }
    int offset = 0;
    int width = 0;
    for (int i = 0; i < 14; i++)
    {
        if (c == pentatonic_point[i])
        {
            width = pentatonic_width[i];
            break;
        }
        offset += pentatonic_width[i];
    }
    if (bottom_line_offset + width > 128)
        return -1;
    memcpy(bottom_line_buf + bottom_line_offset, pentatonic + offset, width);
    memcpy(bottom_line_buf + 128 + bottom_line_offset, pentatonic + 165 + offset, width);
    bottom_line_offset += width + 1;
    if (c == 0x2d)
        bottom_line_offset++;
}

static void display_bottom_str(char *s)
{
    static char last[27]; // bottom line can fit at most 26? chars
    if (strncmp(last, s, 27) == 0)
        return;
    strncpy(last, s, 27);

    bottom_line_offset = 0;
    memset(bottom_line_buf, 0, 256);
    while (*s)
    {
        if (write_char_to_bottom_buf(*s))
            break;
        s++;
    }

    display_write(dev, 0, 16, &bottom_desc, bottom_line_buf);
}

static void display_battery_state(int state)
{
    static int last_state = -1;
    if (state == last_state)
        return;
    last_state = state;

    switch (state)
    {
    case 0:
        display_write(dev, 96, 0, &battery_desc, clear_buf);
        break;
    case 1:
        display_write(dev, 96, 0, &battery_desc, battery);
        break;
    case 2:
        display_write(dev, 96, 0, &battery_desc, battery + 64);
        break;
    case 3:
        display_write(dev, 96, 0, &battery_desc, battery + 128);
        break;
    case 4:
        display_write(dev, 96, 0, &battery_desc, battery + 192);
        break;
    }
}

static void display_top_blink(void)
{
    k_msleep(500);
    display_write(dev, 0, 0, &top_desc, clear_buf);
    k_msleep(500);
    display_write(dev, 0, 0, &top_desc, top_line_buf);
    k_msleep(500);
    display_write(dev, 0, 0, &top_desc, clear_buf);
    k_msleep(500);
    display_write(dev, 0, 0, &top_desc, top_line_buf);
    k_msleep(2500);
    display_write(dev, 0, 0, &top_desc, clear_buf);
    k_msleep(500);
}

static void display_top_clear(void)
{
    display_write(dev, 0, 0, &top_desc, clear_buf);
}

static void display_bottom_clear(void)
{
    display_write(dev, 0, 16, &bottom_desc, clear_buf);
}

static void display_battery_clear(void)
{
    display_battery_state(0);
}

static void display_status_thread(void)
{
    display_top_clear();
	while (1) // cycle through errors
	{
        int status = get_status(SYS_STATUS_ERROR);
        if (status & SYS_STATUS_SENSOR_ERROR)
		{
            display_top_str("Sensor Err");
            display_top_blink();
		}
		if (status & SYS_STATUS_CONNECTION_ERROR)
		{
            display_top_str("Conn Err");
            display_top_blink();
		}
		if (status & SYS_STATUS_SYSTEM_ERROR)
		{
            display_top_str("System Err");
            display_top_blink();
		}
		if (!status)
		{
            display_top_clear();
			k_msleep(1000);
		}
	}
}

static void display_runtime_thread(void)
{
    display_bottom_clear();
    display_battery_clear();
	while (1)
    {
        int16_t pptt = sys_get_valid_battery_pptt();
        pptt = sys_get_calibrated_battery_pptt(pptt);
        uint64_t remaining = sys_get_battery_remaining_time_estimate();
	    remaining = k_ticks_to_ms_floor64(remaining);

        static int16_t last_pptt = -2;
        static int32_t last_minutes = -1;
        static char bottom[10];
        if (!remaining && last_pptt != pptt)
        {
            last_pptt = pptt;
            last_minutes = -1;
            if (pptt >= 0)
            {
                snprintk(bottom, 10, "%u%", pptt / 100);
                display_bottom_str(bottom);
            }
            else
            {
                display_bottom_str("---");
            }
        }
        else if (remaining && last_minutes != remaining / 60000)
        {
            last_pptt = -2;
            last_minutes = remaining / 60000;
	        uint32_t hours = remaining / 3600000;
	        uint8_t minutes = (remaining % 3600000) / 60000;
            snprintk(bottom, 10, "%uh %um", hours, minutes);
            display_bottom_str(bottom);
        }

        if (pptt < 1000)
            display_battery_state(1);
        else if (pptt < 2000)
            display_battery_state(2);
        else if (pptt < 4000)
            display_battery_state(3);
        else
            display_battery_state(4);
        k_msleep(500);

        if (remaining ? remaining < 10800000 : pptt < 1000)
            display_battery_state(0);
        k_msleep(500);
    }
}

static void display_thread(void)
{
	display_blanking_off(dev);
//    display_set_contrast(dev, 0);
}

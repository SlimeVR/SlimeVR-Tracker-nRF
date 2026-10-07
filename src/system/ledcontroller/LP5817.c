#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "LP5817.h"

int write_to_i2c(uint8_t reg, uint8_t val, struct i2c_dt_spec *dev)
{
    uint8_t buf[2] = {reg, val};
    int ret = i2c_write_dt(dev, buf, sizeof(buf));
    if (ret != 0)
    {
        // LOG_ERR("I2C Fail [Reg 0x%02X = 0x%02X], Err: %d", reg, val, ret);
    }
    return ret;
}

int read_from_i2c(uint8_t reg, uint8_t *val, struct i2c_dt_spec *dev)
{
    int ret = i2c_write_read_dt(dev, &reg, sizeof(reg), val, sizeof(*val));
    if (ret != 0)
    {
        // LOG_ERR("Failed to read reg 0x%02X [Addr 0x%02X], Err: %d",
        //        reg, dev->addr, ret);
    }
    return ret;
}

void set_leds_controller(uint8_t r, uint8_t g, uint8_t b, struct i2c_dt_spec *dev)
{
    int ret;

    ret = write_to_i2c(0x00, 0x01, dev);
    if (ret != 0)
    {
        return;
    }
    k_msleep(2);

    ret = write_to_i2c(0x02, 0x3F, dev);
    if (ret != 0)
    {
        return;
    }

    // Set Max current
    write_to_i2c(0x01, 0x01, dev);

    // Enable OUT0, OUT1, OUT2
    write_to_i2c(0x02, 0x07, dev);

    // LED Brightness
    write_to_i2c(0x14, 0x7F, dev); // Red
    write_to_i2c(0x15, 0x7F, dev); // Green
    write_to_i2c(0x16, 0x7F, dev); // Blue

    // LED Colors
    write_to_i2c(0x18, r, dev); // Red
    write_to_i2c(0x19, g, dev); // Green
    write_to_i2c(0x1A, b, dev); // Blue

    // Push settings to output latch
    write_to_i2c(0x0F, 0x55, dev);
}

void suspend_led_controller(struct i2c_dt_spec *dev)
{
    write_to_i2c(0xD0, 0x33, dev);
}
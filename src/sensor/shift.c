
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "shift.h"

LOG_MODULE_REGISTER(sensor_shift, LOG_LEVEL_DBG);

int init_shift_reg(sensor_shift_register_t *sensor_shift_registers)
{

    if (!gpio_is_ready_dt(&sensor_shift_registers[0].dsb) || !gpio_is_ready_dt(&sensor_shift_registers[0].cp) ||
        !gpio_is_ready_dt(&sensor_shift_registers[1].dsb) || !gpio_is_ready_dt(&sensor_shift_registers[1].cp))
    {
        LOG_ERR("Shift register GPIO pins not ready");
        return -1;
    }

    gpio_pin_configure_dt(&sensor_shift_registers[0].dsb, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&sensor_shift_registers[0].cp, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&sensor_shift_registers[1].dsb, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&sensor_shift_registers[1].cp, GPIO_OUTPUT_INACTIVE);

    LOG_INF("Settings shift reg0 all high");
    shift_pattern(&sensor_shift_registers[0], 0xFF);
    LOG_INF("Settings shift reg1 all high");
    shift_pattern(&sensor_shift_registers[1], 0xFF);

    k_busy_wait(75);

    return 0;
}

void shift_pattern(sensor_shift_register_t *sensor_shift_register, uint8_t pattern)
{
    LOG_DBG("Shifting pattern %d", pattern);

    for (int i = 7; i >= 0; i--)
    {
        uint8_t bit = (pattern >> i) & 0x01;

        gpio_pin_set_dt(&sensor_shift_register->dsb, bit);
        k_busy_wait(10);
        gpio_pin_set_dt(&sensor_shift_register->cp, 1);
        k_busy_wait(10);
        gpio_pin_set_dt(&sensor_shift_register->cp, 0);
        k_busy_wait(10);
    }
    // gpio_pin_set_dt(&sensor_shift_register->dsb, 0);
    k_busy_wait(10);
}

int set_all_high(void)
{
    return 0;
}

shift_register_ops_t shift_register_ops = {
    .init_shift_reg = init_shift_reg,
    .shift_pattern = shift_pattern,
    .set_all_high = set_all_high,
};

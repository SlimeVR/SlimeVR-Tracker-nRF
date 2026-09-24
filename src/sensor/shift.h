
#ifndef SHIFT_h
#define SHIFT_h

#include <zephyr/drivers/gpio.h>

#define SENSOR_SHIFT_REGISTER_COUNT 2

typedef struct sensor_shift_register
{
    struct gpio_dt_spec dsb;
    struct gpio_dt_spec cp;
    uint8_t current_pattern;
} sensor_shift_register_t;

/*
struct shift_spi_ctx
{
    const struct device *spi_dev;
    struct spi_config config;
    sensor_shift_register_t *shift_reg;
    uint8_t sensor_mask;
};
*/

typedef int (*init_shift_reg_func)(sensor_shift_register_t *sensor_shift_registers);
typedef void (*shift_pattern_func)(sensor_shift_register_t *sensor_shift_register, uint8_t pattern);
typedef void (*set_all_high_func)(void);
// typedef void (*shift_spi_transceive)(const struct imu_spi_ctx *ctx, const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs);

typedef struct shift_register_ops
{
    init_shift_reg_func init_shift_reg;
    shift_pattern_func shift_pattern;
    set_all_high_func set_all_high;
} shift_register_ops_t;

extern shift_register_ops_t shift_register_ops;

#endif

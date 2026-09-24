#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT svr_lv164_gpio

LOG_MODULE_REGISTER(shifter, LOG_LEVEL_INF);

struct lv164_config
{
    struct gpio_driver_config common;
    struct gpio_dt_spec dsb;
    struct gpio_dt_spec cp;
    uint8_t num_outputs;
};

struct lv164_data
{
    struct gpio_driver_data common;
    uint32_t bitmask;
} __aligned(4);

static int lv164_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(pin);
    return (flags & GPIO_INPUT) ? -ENOTSUP : 0;
}

static void lv164_shift_out(const struct device *dev)
{
    const struct lv164_config *cfg = dev->config;
    struct lv164_data *data = dev->data;
    for (int i = cfg->num_outputs - 1; i >= 0; i--)
    {
        uint8_t bit = (data->bitmask >> i) & 0x01;
        gpio_pin_set_dt(&cfg->dsb, bit);
        // k_busy_wait(10);
        gpio_pin_set_dt(&cfg->cp, 1);
        // k_busy_wait(10);
        gpio_pin_set_dt(&cfg->cp, 0);
        // k_busy_wait(10);
    }
    k_busy_wait(10);
}

int lv164_init(const struct device *dev)
{
    const struct lv164_config *cfg = dev->config;
    struct lv164_data *data = dev->data;
    LOG_DBG("bitmask %d", data->bitmask);
    if (!gpio_is_ready_dt(&cfg->dsb) || !gpio_is_ready_dt(&cfg->cp))
    {
        LOG_ERR("Parent GPIO devices for shift register are not ready!");
        return -ENODEV;
    }
    gpio_pin_configure_dt(&cfg->dsb, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&cfg->cp, GPIO_OUTPUT_INACTIVE);
    data->bitmask = 0;
    lv164_shift_out(dev);
    return 0;
}

static int lv164_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
    struct lv164_data *data = dev->data;
    unsigned int key = irq_lock();
    data->bitmask |= pins;
    lv164_shift_out(dev);
    irq_unlock(key);

    LOG_DBG("CS assert, out=0x%02x", (uint8_t)data->bitmask);
    return 0;
}

static int lv164_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
    struct lv164_data *data = dev->data;

    unsigned int key = irq_lock();
    data->bitmask &= ~pins;
    lv164_shift_out(dev);
    irq_unlock(key);

    return 0;
}

static const struct gpio_driver_api lv164_api = {
    .pin_configure = lv164_pin_configure,
    .port_set_bits_raw = lv164_port_set_bits_raw,
    .port_clear_bits_raw = lv164_port_clear_bits_raw,
};

#define LV164_INIT(inst)                                       \
    static struct lv164_data lv164_data_##inst __aligned(4);   \
    static const struct lv164_config lv164_config_##inst = {   \
        .common = {                                            \
            .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_NGPIOS(   \
                DT_INST_PROP(inst, num_outputs)),              \
        },                                                     \
        .dsb = GPIO_DT_SPEC_INST_GET(inst, dsb_gpios),         \
        .cp = GPIO_DT_SPEC_INST_GET(inst, cp_gpios),           \
        .num_outputs = DT_INST_PROP(inst, num_outputs),        \
    };                                                         \
    DEVICE_DT_DEFINE(DT_DRV_INST(inst), lv164_init, NULL,      \
                     &lv164_data_##inst, &lv164_config_##inst, \
                     POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &lv164_api);

DT_INST_FOREACH_STATUS_OKAY(LV164_INIT)

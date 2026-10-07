#include "../led.h"

void set_leds_controller(uint8_t r, uint8_t g, uint8_t b, struct i2c_dt_spec *dev);

void suspend_led_controller(struct i2c_dt_spec *dev);
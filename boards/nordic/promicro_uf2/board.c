/*
 * Copyright (c) 2018 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <hal/nrf_gpio.h>
#include <zephyr/devicetree.h>

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, vcc_gpios)
	#define VCC_GPIO_PIN DT_GPIO_PIN(ZEPHYR_USER_NODE, vcc_gpios)
	#define VCC_GPIO_PORT_NUM DT_PROP(DT_GPIO_CTLR(ZEPHYR_USER_NODE, vcc_gpios), port)
#endif

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, gnd_gpios)
	#define GND_GPIO_PIN DT_GPIO_PIN(ZEPHYR_USER_NODE, gnd_gpios)
	#define GND_GPIO_PORT_NUM DT_PROP(DT_GPIO_CTLR(ZEPHYR_USER_NODE, gnd_gpios), port)
#endif

static int board_promicro_init(void)
{
	/* using vcc-gpios for sensor power defined in promicro_uf2.dts or variant device tree
	 */
	#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, vcc_gpios)
		nrf_gpio_cfg(NRF_GPIO_PIN_MAP(VCC_GPIO_PORT_NUM, VCC_GPIO_PIN), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_D0H1, NRF_GPIO_PIN_NOSENSE);
		nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(VCC_GPIO_PORT_NUM, VCC_GPIO_PIN));
	#endif

	/* using gnd-gpios for sensor power defined in promicro_uf2.dts or variant or variant device tree
	 */
	#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, gnd_gpios)
		nrf_gpio_cfg(NRF_GPIO_PIN_MAP(GND_GPIO_PORT_NUM, GND_GPIO_PIN), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_D0H1, NRF_GPIO_PIN_NOSENSE);
		nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(GND_GPIO_PORT_NUM, GND_GPIO_PIN));
	#endif

	/* pull down on P0.13, disables external 3V3 regulator
	 */
	nrf_gpio_cfg(NRF_GPIO_PIN_MAP(0, 13), NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
	/* if external 3V3 regulator is needed, use PULLUP instead of PULLDOWN: nrf_gpio_cfg(NRF_GPIO_PIN_MAP(0, 13), NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
	*/
	return 0;
}

SYS_INIT(board_promicro_init, PRE_KERNEL_1,
	CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
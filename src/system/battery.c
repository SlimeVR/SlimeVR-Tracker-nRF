/*
 * Copyright (c) 2018-2019 Peter Bigot Consulting, LLC
 * Copyright (c) 2019-2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "battery.h"

LOG_MODULE_REGISTER(BATTERY, CONFIG_ADC_LOG_LEVEL);

#define VBATT DT_PATH(battery_divider)
#define ZEPHYR_USER DT_PATH(zephyr_user)

struct io_channel_config {
	uint8_t channel;
};

struct divider_config {
	struct io_channel_config io_channel;
	struct gpio_dt_spec power_gpios;
	/* output_ohm is used as a flag value: if it is nonzero then
	 * the battery is measured through a voltage divider;
	 * otherwise it is assumed to be directly connected to Vdd.
	 */
	uint32_t output_ohm;
	uint32_t full_ohm;
};

static const struct divider_config divider_config = {
#if DT_NODE_HAS_STATUS(VBATT, okay)
	.io_channel = {
		DT_IO_CHANNELS_INPUT(VBATT),
	},
	.power_gpios = GPIO_DT_SPEC_GET_OR(VBATT, power_gpios, {}),
	.output_ohm = DT_PROP(VBATT, output_ohms),
	.full_ohm = DT_PROP(VBATT, full_ohms),
#else /* /vbatt exists */
#error "Battery divider node does not exist"
	.io_channel = {
		DT_IO_CHANNELS_INPUT(ZEPHYR_USER),
	},
#endif /* /vbatt exists */
};

struct divider_data {
	const struct device* adc;
	struct adc_channel_cfg adc_cfg;
	struct adc_sequence adc_seq;
	int16_t raw;
};
static struct divider_data divider_data = {
#if DT_NODE_HAS_STATUS(VBATT, okay)
	.adc = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(VBATT)),
#else
#error "Battery divider node does not exist"
	.adc = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(ZEPHYR_USER)),
#endif
};

static int divider_setup(void) {
	const struct divider_config* cfg = &divider_config;
	const struct io_channel_config* iocp = &cfg->io_channel;
	const struct gpio_dt_spec* gcp = &cfg->power_gpios;
	struct divider_data* ddp = &divider_data;
	struct adc_sequence* asp = &ddp->adc_seq;
	struct adc_channel_cfg* accp = &ddp->adc_cfg;
	int rc;

	if (!device_is_ready(ddp->adc)) {
		LOG_ERR("ADC device is not ready %s", ddp->adc->name);
		return -ENOENT;
	}

	if (gcp->port) {
		if (!device_is_ready(gcp->port)) {
			LOG_ERR("%s: device not ready", gcp->port->name);
			return -ENOENT;
		}
		rc = gpio_pin_configure_dt(gcp, GPIO_OUTPUT_INACTIVE);
		if (rc != 0) {
			LOG_ERR("Failed to control feed %s.%u: %d", gcp->port->name, gcp->pin, rc);
			return rc;
		}
	}

	*asp = (struct adc_sequence){
		.channels = BIT(0),
		.buffer = &ddp->raw,
		.buffer_size = sizeof(ddp->raw),
		.oversampling = 7, // TODO: using R3 board, ADC is very noisy, are other boards okay?
		.calibrate = true,
	};

#ifdef CONFIG_ADC_NRFX_SAADC
	enum adc_gain battery_adc_gain = ADC_GAIN_1_6;

	float max_adc_voltage = cfg->output_ohm != 0 ? 5.0f * cfg->output_ohm / cfg->full_ohm : 3.6f; // Maximum voltage on input

	if (max_adc_voltage < 0.6f)
		battery_adc_gain = ADC_GAIN_1;
	else if (max_adc_voltage < 1.2f)
		battery_adc_gain = ADC_GAIN_1_2;
	else if (max_adc_voltage < 1.8f)
		battery_adc_gain = ADC_GAIN_1_3;
	else if (max_adc_voltage < 2.4f)
		battery_adc_gain = ADC_GAIN_1_4;
	else if (max_adc_voltage < 3.0f)
		battery_adc_gain = ADC_GAIN_1_5;

	LOG_INF("ADC gain enum: %d, max voltage: %.2f mV",
		battery_adc_gain, (double)(max_adc_voltage * 1000.0f));

	*accp = (struct adc_channel_cfg){
		.gain = battery_adc_gain,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 3),
	};

	if (cfg->output_ohm != 0) {
		accp->input_positive = 1  // SAADC_CH_PSELP_PSELP_AnalogInput0
							 + iocp->channel;
	} else {
		accp->input_positive = 9;  // SAADC_CH_PSELP_PSELP_VDD
	}

	if (iocp->channel == 12) { // VDDHDIV5
		asp->oversampling = 2;
		accp->acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10);
	}

	asp->resolution = 14;
#else /* CONFIG_ADC_var */
#error Unsupported ADC
#endif /* CONFIG_ADC_var */

	rc = adc_channel_setup(ddp->adc, accp);
	LOG_INF("Setup AIN%u got %d", iocp->channel, rc);

	return rc;
}

static bool battery_ok;

static int battery_setup() {
	int rc = divider_setup();

	battery_ok = (rc == 0);
	LOG_INF("Battery setup: %d %d", rc, battery_ok);
	return rc;
}

SYS_INIT(battery_setup, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

int battery_measure_enable(bool enable) {
	int rc = -ENOENT;

	if (battery_ok) {
		const struct gpio_dt_spec* gcp = &divider_config.power_gpios;

		rc = 0;
		if (gcp->port) {
			rc = gpio_pin_set_dt(gcp, enable);
		}
	}
	return rc;
}

int battery_sample(void) {
	int rc = -ENOENT;

	if (battery_ok) {
		struct divider_data* ddp = &divider_data;
		const struct divider_config* dcp = &divider_config;
		struct adc_sequence* sp = &ddp->adc_seq;

		rc = adc_read(ddp->adc, sp);
		sp->calibrate = false;
		if (rc == 0) {
			int32_t val = ddp->raw;

			adc_raw_to_millivolts(
				adc_ref_internal(ddp->adc),
				ddp->adc_cfg.gain,
				sp->resolution,
				&val
			);

			if (dcp->output_ohm != 0) {
				rc = val * (uint64_t)dcp->full_ohm / dcp->output_ohm;
				LOG_INF("raw %u ~ %u mV => %d mV\n", ddp->raw, val, rc);
			} else {
				rc = val;
				LOG_INF("raw %u ~ %u mV\n", ddp->raw, val);
			}
		}
	}

	return rc;
}

unsigned int
battery_level_pptt(unsigned int batt_mV, const struct battery_level_point* curve) {
	const struct battery_level_point* pb = curve;

	if (batt_mV >= pb->lvl_mV) {
		/* Measured voltage above highest point, cap at maximum. */
		return pb->lvl_pptt;
	}
	/* Go down to the last point at or below the measured voltage. */
	while ((pb->lvl_pptt > 0) && (batt_mV < pb->lvl_mV)) {
		++pb;
	}
	if (batt_mV < pb->lvl_mV) {
		/* Below lowest point, cap at minimum */
		return pb->lvl_pptt;
	}

	/* Linear interpolation between below and above points. */
	const struct battery_level_point* pa = pb - 1;

	return pb->lvl_pptt
		 + ((pa->lvl_pptt - pb->lvl_pptt) * (batt_mV - pb->lvl_mV)
			/ (pa->lvl_mV - pb->lvl_mV));
}

static const struct battery_level_point levels[] = {
#if CONFIG_BATTERY_USE_REG_BUCK_MAPPING
	{10000, 4150},
	{9500, 4075},
	{3000, 3775},
	{500, 3450},
	{0, 3200},
#elif CONFIG_BATTERY_USE_REG_LDO_MAPPING
	{10000, 4150},
	{9500, 4025},
	{3000, 3650},
	{500, 3400},
	{0, 3200},
#else
#warning "Battery voltage map not defined"
	{10000, 0},
	{0, 0},
#endif
};

unsigned int read_batt() {
	int rc = battery_measure_enable(true);

	if (rc != 0) {
		LOG_ERR("Failed initialize battery measurement: %d", rc);
		return -1;
	}

	int batt_mV = battery_sample();

	if (batt_mV < 0) {
		LOG_DBG("Failed to read battery voltage: %d", batt_mV);
	}

	battery_measure_enable(false);

	return battery_level_pptt(batt_mV, levels);
}

unsigned int read_batt_mV(int* out) {
	int rc = battery_measure_enable(true);

	if (rc != 0) {
		LOG_ERR("Failed initialize battery measurement: %d", rc);
		return -1;
	}

	int batt_mV = battery_sample();

	if (batt_mV < 0) {
		LOG_DBG("Failed to read battery voltage: %d", batt_mV);
	}

	battery_measure_enable(false);

	*out = batt_mV;
	return battery_level_pptt(batt_mV, levels);
}

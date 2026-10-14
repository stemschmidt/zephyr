/*
 * Copyright (c) 2026 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT zephyr_dc_motor

#include <zephyr/drivers/motor_driver.h>

LOG_MODULE_REGISTER(dc_motor_driver);

struct dc_motor_driver_config {
	struct pwm_dt_spec pwm_towards_min;
	struct pwm_dt_spec pwm_towards_max;
	struct gpio_dt_spec en_gpio;
};

struct dc_motor_driver_data {
	struct motor_common_data common;
	struct gpio_callback overcurrent_cb;
	const struct device *dev;
};

/* Ignore overcurrent detection for now! */
static int dc_motor_driver_enable(const struct device *dev)
{
	const struct dc_motor_driver_config *config = dev->config;
	struct dc_motor_driver_data *data = dev->data;
	int rc = 0;

	if (data->common.enabled) {
		return -EALREADY;
	} else {
		rc = gpio_pin_configure_dt(&config->en_gpio, GPIO_INPUT | GPIO_PULL_UP);
		if (rc == 0) {
			data->common.enabled = true;
		}
	}

	return rc;
}

/* Ignore overcurrent detection for now! */
static int dc_motor_driver_disable(const struct device *dev)
{
	const struct dc_motor_driver_config *config = dev->config;
	struct dc_motor_driver_data *data = dev->data;
	int rc = 0;

	if (!data->common.enabled) {
		return -EALREADY;
	} else {
		rc = gpio_pin_configure_dt(&config->en_gpio, GPIO_INPUT | GPIO_PULL_DOWN);
		if (rc == 0) {
			data->common.enabled = false;
		}
	}

	return rc;
}

static int dc_motor_driver_stop(const struct device *dev)
{
	const struct dc_motor_driver_config *config = dev->config;
	struct dc_motor_driver_data *data = dev->data;
	int rc = 0;

	if (data->common.duty == (float)0.0) {
		rc = -EALREADY;
	} else {
		rc = pwm_set_dt(&config->pwm_towards_min, config->pwm_towards_min.period, 0);
		rc |= pwm_set_dt(&config->pwm_towards_max, config->pwm_towards_max.period, 0);
		if (rc == 0) {
			data->common.duty = 0.0;
		}
	}

	return rc;
}

static int dc_motor_driver_actuator_set_duty(const struct device *dev, const float *duty, uint8_t n)
{
	const struct dc_motor_driver_config *config = dev->config;
	struct dc_motor_driver_data *data = dev->data;
	float duty_cycle = *duty;
	int rc = 0;

	if (duty_cycle > (float)1.0 || duty_cycle < (float)-1.0) {
		return -EINVAL;
	}

	if (!data->common.enabled) {
		rc = -EIO;
	} else {
		if (duty_cycle < (float)0.0) {
			if (data->common.duty > (float)0.0) {
				LOG_ERR("Add a stop before changing directions");
				return -EINVAL;
			}
			if (duty_cycle < (float)-0.3) {
				LOG_ERR("Limit duty_cycle to -0.3 (was %f)", (double)duty_cycle);
				duty_cycle = (float)-0.3;
			}
			rc = pwm_set_dt(&config->pwm_towards_min, config->pwm_towards_min.period,
					(uint32_t)(-duty_cycle * config->pwm_towards_min.period));
		} else {
			if (data->common.duty < (float)0.0) {
				LOG_ERR("Add a stop before changing directions");
				return -EINVAL;
			}
			if (duty_cycle > (float)0.3) {
				LOG_ERR("Limit duty_cycle to 0.3 (was %f)", (double)duty_cycle);
				duty_cycle = (float)0.3;
			}
			rc = pwm_set_dt(&config->pwm_towards_max, config->pwm_towards_max.period,
					(uint32_t)(duty_cycle * config->pwm_towards_max.period));
		}
		if (rc == 0) {
			data->common.duty = duty_cycle;
		}
	}

	return rc;
}

static void irq_overcurrent_handler(const struct device *port, struct gpio_callback *cb,
				    uint32_t pins)
{
	struct dc_motor_driver_data *data =
		CONTAINER_OF(cb, struct dc_motor_driver_data, overcurrent_cb);
	dc_motor_driver_disable(data->dev);
}

static int dc_motor_driver_init(const struct device *dev)
{
	const struct dc_motor_driver_config *config = dev->config;
	struct dc_motor_driver_data *data = dev->data;
	int rc = 0;

	if (!pwm_is_ready_dt(&config->pwm_towards_min)) {
		LOG_ERR("pwm_towards_min device is not ready\n");
		return -ENODEV;
	}

	if (!pwm_is_ready_dt(&config->pwm_towards_max)) {
		LOG_ERR("pwm_towards_max device is not ready\n");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->en_gpio)) {
		LOG_ERR("ENABLE GPIO device not ready");
		return -ENODEV;
	}

	motor_common_init(&data->common);

	rc = gpio_pin_configure_dt(&config->en_gpio, GPIO_INPUT | GPIO_PULL_DOWN);
	if (rc == 0) {
		gpio_init_callback(&data->overcurrent_cb, irq_overcurrent_handler,
				   BIT(config->en_gpio.pin));
		gpio_add_callback(config->en_gpio.port, &data->overcurrent_cb);
	}

	return rc;
}

static const struct motor_driver_api dc_motor_driver_api = {
	.enable = dc_motor_driver_enable,
	.disable = dc_motor_driver_disable,
	.stop = dc_motor_driver_stop,
	.set_duty = dc_motor_driver_actuator_set_duty};

/* clang-format off */

#define DC_MOTOR_DRIVER_DEFINE(inst)	\
	static struct dc_motor_driver_data data_##inst;	\
	\
	static const struct dc_motor_driver_config config_##inst = {	\
		.pwm_towards_min = PWM_DT_SPEC_INST_GET_BY_NAME(inst, towards_min),	\
		.pwm_towards_max = PWM_DT_SPEC_INST_GET_BY_NAME(inst, towards_max),	\
		.en_gpio = GPIO_DT_SPEC_INST_GET(inst, enable_gpios),	\
	};	\
	\
	DEVICE_DT_INST_DEFINE(inst, dc_motor_driver_init, NULL, &data_##inst,	\
						  &config_##inst, POST_KERNEL,	\
						  CONFIG_MOTOR_INIT_PRIORITY, &dc_motor_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DC_MOTOR_DRIVER_DEFINE)

/* clang-format on */

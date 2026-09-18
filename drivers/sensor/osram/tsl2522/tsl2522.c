/*
 * Copyright (c) 2026 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tsl2522, CONFIG_SENSOR_LOG_LEVEL);

static int tsl2522_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	return 0;
}

static int tsl2522_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	val->val1 = 1;
	val->val2 = 5000;

	return 0;
}

static DEVICE_API(sensor, tsl2522_driver_api) = {
	.sample_fetch = tsl2522_sample_fetch,
	.channel_get = tsl2522_channel_get,
};

static int tsl2522_init(const struct device *dev)
{
	return 0;
}

#define DT_DRV_COMPAT osram_tsl2522

#define TLS2522_DEFINE(inst)                                                                       \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, tsl2522_init, NULL, NULL, NULL, POST_KERNEL,            \
				     CONFIG_SENSOR_INIT_PRIORITY, &tsl2522_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TLS2522_DEFINE)

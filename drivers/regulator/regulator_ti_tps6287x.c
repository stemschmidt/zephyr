/*
 * Copyright (c) 2026 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tps62873

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/linear_range.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(tps6287x, CONFIG_REGULATOR_LOG_LEVEL);

struct regulator_tps62873_data {
	struct regulator_common_data data;
	int32_t min_uv;
	int32_t max_uv;
	bool active_discharge;
};

struct regulator_tps6287x_config {
	struct regulator_common_config common;
	struct i2c_dt_spec i2c;
};

static unsigned int regulator_tps6287x_count_voltages(const struct device *dev)
{
	LOG_INF("regulator_tps6287x_count_voltages, return 10");
	return 10U;
}

static int regulator_tps6287x_list_voltage(const struct device *dev, unsigned int idx,
					   int32_t *volt_uv)
{
	LOG_INF("regulator_tps6287x_list_voltage");
	*volt_uv = idx * 1000;
	return 0;
}

static int regulator_tps6287x_set_voltage(const struct device *dev, int32_t min_uv, int32_t max_uv)
{
	struct regulator_tps62873_data *data = (struct regulator_tps62873_data *)dev->data;
	data->min_uv = min_uv;
	data->max_uv = max_uv;
	LOG_INF("regulator_tps6287x_set_voltage: min_uv %d, max_uv: %d", min_uv, max_uv);
	return 0;
}

static int regulator_tps6287x_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	struct regulator_tps62873_data *data = (struct regulator_tps62873_data *)dev->data;
	LOG_INF("regulator_tps6287x_get_voltage: %d", data->min_uv);
	*volt_uv = data->min_uv;
	return 0;
}

static int regulator_tps6287x_set_active_discharge(const struct device *dev, bool active_discharge)
{
	struct regulator_tps62873_data *data = (struct regulator_tps62873_data *)dev->data;
	data->active_discharge = active_discharge;
	LOG_INF("regulator_tps6287x_set_active_discharge %s", active_discharge ? "on" : "off");
	return 0;
}

static int regulator_tps6287x_get_active_discharge(const struct device *dev, bool *active_discharge)
{
	struct regulator_tps62873_data *data = (struct regulator_tps62873_data *)dev->data;

	LOG_INF("regulator_tps6287x_get_active_discharge %s",
		data->active_discharge ? "on" : "off");
	*active_discharge = data->active_discharge;
	return 0;
}

static int regulator_tps6287x_enable(const struct device *dev)
{
	LOG_INF("regulator_tps6287x_enable");
	return 0;
}

static int regulator_tps6287x_disable(const struct device *dev)
{
	LOG_INF("regulator_tps6287x_disable");
	return 0;
}

static int regulator_tps6287x_init(const struct device *dev)
{
	return 0;
}

static DEVICE_API(regulator, api) = {
	.enable = regulator_tps6287x_enable,
	.disable = regulator_tps6287x_disable,
	.count_voltages = regulator_tps6287x_count_voltages,
	.list_voltage = regulator_tps6287x_list_voltage,
	.set_voltage = regulator_tps6287x_set_voltage,
	.get_voltage = regulator_tps6287x_get_voltage,
	.set_active_discharge = regulator_tps6287x_set_active_discharge,
	.get_active_discharge = regulator_tps6287x_get_active_discharge,
};

/* clang-format off */
#define REGULATOR_TPS6287X_DEFINE_ALL(inst) \
	static struct regulator_tps62873_data data_##inst; \
                                                        \
	static const struct regulator_tps6287x_config config_##inst = { \
		.common = REGULATOR_DT_INST_COMMON_CONFIG_INIT(inst), \
		.i2c = I2C_DT_SPEC_INST_GET(inst), \
	};                                               \
                                                     \
	DEVICE_DT_INST_DEFINE(inst, regulator_tps6287x_init, NULL, &data_##inst, &config_##inst, \
			      POST_KERNEL, CONFIG_REGULATOR_TPS6287X_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(REGULATOR_TPS6287X_DEFINE_ALL)
/* clang-format on */

/*
 * Copyright (c) 2026 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tps62873

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/linear_range.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(tps6287x, CONFIG_REGULATOR_LOG_LEVEL);

#define TPS6287x_MIN_DIV_OUTPUT 1400000U /* Minimum difference between in- and output voltage. */

#define TPS6287X_REG_VSET 0x00U /* Output voltage setpoint, Reset = X */

#define TPS6287X_VSET_MASK GENMASK(7, 0)

#define TPS6287X_REG_CONTROL1 0x01U /* Control 1, Reset = 0x2A */

#define TPS6287X_CONTROL1_RESET      BIT(7)
#define TPS6287X_CONTROL1_SSCEN      BIT(6)
#define TPS6287X_CONTROL1_SWEN       BIT(5)
#define TPS6287X_CONTROL1_FPWMEN     BIT(4)
#define TPS6287X_CONTROL1_DISCHEN    BIT(3)
#define TPS6287X_CONTROL1_HICCUPEN   BIT(2)
#define TPS6287X_CONTROL1_VRAMP_MASK GENMASK(1, 0)

/* RESET (bit 7) */
#define TPS6287X_RESET_NO_EFFECT 0x0U
#define TPS6287X_RESET_ALL_REGS  0x1U /* Resets all registers to default. Read-back is always 0 */

/* SSCEN (bit 6) - Spread spectrum clocking enable */
#define TPS6287X_SSCEN_DISABLED 0x0U
#define TPS6287X_SSCEN_ENABLED  0x1U

/* SWEN (bit 5) - Software enable */
#define TPS6287X_SWEN_DISABLED 0x0U /* Switching disabled, register values retained */
#define TPS6287X_SWEN_ENABLED  0x1U /* Switching enabled (without the enable delay)  */

/* FPWMEN (bit 4) - Forced PWM enable (logically ORed with MODE/SYNC pin) */
#define TPS6287X_FPWMEN_POWER_SAVE 0x0U
#define TPS6287X_FPWMEN_FORCED_PWM 0x1U

/* DISCHEN (bit 3) - Output discharge enable */
#define TPS6287X_DISCHEN_DISABLED 0x0U
#define TPS6287X_DISCHEN_ENABLED  0x1U

/* HICCUPEN (bit 2) - Hiccup operation enable. Do not enable during stacked operation. */
#define TPS6287X_HICCUPEN_DISABLED 0x0U
#define TPS6287X_HICCUPEN_ENABLED  0x1U

/* VRAMP (bits 1-0) - Output voltage ramp speed */
#define TPS6287X_VRAMP_10000_UV_PER_US 0x0U
#define TPS6287X_VRAMP_5000_UV_PER_US  0x1U
#define TPS6287X_VRAMP_1250_UV_PER_US  0x2U
#define TPS6287X_VRAMP_500_UV_PER_US   0x3U

#define TPS6287X_VSET_RANGE1_BASE_UV 400000 /* 0.400 V */
#define TPS6287X_VSET_RANGE1_STEP_UV 1250   /* 1.25 mV/step  (0.400 V - 0.71875 V) */
#define TPS6287X_VSET_RANGE2_BASE_UV 400000 /* 0.400 V */
#define TPS6287X_VSET_RANGE2_STEP_UV 2500   /* 2.5 mV/step   (0.400 V - 1.0375 V)  */
#define TPS6287X_VSET_RANGE3_BASE_UV 400000 /* 0.400 V */
#define TPS6287X_VSET_RANGE3_STEP_UV 5000   /* 5 mV/step     (0.400 V - 1.675 V)   */
#define TPS6287X_VSET_RANGE4_BASE_UV 800000 /* 0.800 V */
#define TPS6287X_VSET_RANGE4_STEP_UV 10000  /* 10 mV/step    (0.800 V - 3.35 V)    */

#define TPS6287X_REG_CONTROL2 0x02U /* Control 2, Reset = 0x09 */

#define TPS6287X_CONTROL2_RESERVED_MASK GENMASK(7, 4)
#define TPS6287X_CONTROL2_VRANGE_MASK   GENMASK(3, 2)
#define TPS6287X_CONTROL2_SSTIME_MASK   GENMASK(1, 0)

/* VRANGE (bits 3-2) - Output voltage range, applies to VSET register */
#define TPS6287X_VRANGE_1_400MV_TO_719MV_STEP_1P25MV 0x0U
#define TPS6287X_VRANGE_2_400MV_TO_1038MV_STEP_2P5MV 0x1U
#define TPS6287X_VRANGE_3_400MV_TO_1675MV_STEP_5MV   0x2U
#define TPS6287X_VRANGE_4_800MV_TO_3350MV_STEP_10MV  0x3U

/* SSTIME (bits 1-0) - Soft-start ramp time */
#define TPS6287X_SSTIME_0_5_MS 0x0U
#define TPS6287X_SSTIME_1_MS   0x1U
#define TPS6287X_SSTIME_2_MS   0x2U
#define TPS6287X_SSTIME_4_MS   0x3U

#define TPS6287X_REG_CONTROL3 0x03U /* Control 3, Reset = 0x00 */

#define TPS6287X_CONTROL3_RESERVED_MASK GENMASK(7, 2)
#define TPS6287X_CONTROL3_SINGLE        BIT(1)
#define TPS6287X_CONTROL3_PGBLNKDVS     BIT(0)

/* SINGLE (bit 1) - Controls internal EN pulldown and SYNC_OUT */
#define TPS6287X_SINGLE_EN_PULLDOWN_SYNCOUT_ENABLED  0x0U
#define TPS6287X_SINGLE_EN_PULLDOWN_SYNCOUT_DISABLED 0x1U

/* PGBLNKDVS (bit 0) - Power-good blanking during DVS */
#define TPS6287X_PGBLNKDVS_PG_REFLECTS_COMPARATOR 0x0U
#define TPS6287X_PGBLNKDVS_PG_HIGH_Z_DURING_DVS   0x1U

#define TPS6287X_REG_STATUS 0x04U /* Status, Reset = 0x02 */

#define TPS6287X_STATUS_RESERVED_MASK GENMASK(7, 6)
#define TPS6287X_STATUS_HICCUP        BIT(5) /* Hiccup event occurred since last read */
#define TPS6287X_STATUS_ILIM          BIT(4) /* Current limit event occurred since last read */
#define TPS6287X_STATUS_TWARN         BIT(3) /* Thermal warning event occurred since last read */
#define TPS6287X_STATUS_TSHUT         BIT(2) /* Thermal shutdown event occurred since last read */
#define TPS6287X_STATUS_PBUV BIT(1) /* Power-bad undervolt. event occurred since last read */
#define TPS6287X_STATUS_PBOV BIT(0) /* Power-bad overvoltage event occurred since last read */

struct regulator_tps62873_data {
	struct regulator_common_data data;
	int32_t min_uv;
	int32_t max_uv;
};

struct regulator_tps6287x_config {
	struct regulator_common_config common;
	struct i2c_dt_spec i2c;
	uint32_t input_voltage_uv;
};

static const struct linear_range voltage_ranges[] = {
	LINEAR_RANGE_INIT(400000u, 1250u, 0, BIT_MASK(8)),
	LINEAR_RANGE_INIT(400000u, 2500u, 0, BIT_MASK(8)),
	LINEAR_RANGE_INIT(400000u, 5000u, 0, BIT_MASK(8)),
	LINEAR_RANGE_INIT(800000u, 10000u, 0, BIT_MASK(8)),
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
	const struct regulator_tps6287x_config *cfg =
		(const struct regulator_tps6287x_config *)dev->config;
	int rc = 0;
	uint8_t control2 = 0;

	if ((min_uv + TPS6287x_MIN_DIV_OUTPUT) > cfg->input_voltage_uv ||
	    (max_uv + TPS6287x_MIN_DIV_OUTPUT) > cfg->input_voltage_uv) {
		return -EINVAL;
	}

	rc = i2c_reg_read_byte_dt(&cfg->i2c, TPS6287X_REG_CONTROL2, &control2);
	if (rc < 0) {
		return rc;
	}

	uint8_t vrange = FIELD_GET(TPS6287X_CONTROL2_VRANGE_MASK, control2);
	uint16_t idx = 0;

	rc = linear_range_get_win_index(&voltage_ranges[vrange], min_uv, max_uv, &idx);

	/* If we cannot find a matching voltage in the current range, check the other ranges */
	if (rc < 0) {
		vrange = ARRAY_SIZE(voltage_ranges);
		/* We start with the highest voltage range and work our way down */
		do {
			vrange--;

			rc = linear_range_get_win_index(&voltage_ranges[vrange], min_uv, max_uv,
							&idx);
		} while ((rc < 0) && (vrange > 0U));

		if (rc < 0) {
			return rc;
		}

		if (vrange == 3U) {
			LOG_ERR("Switch to 0.8 Setpoint not yet implemented!");
			return -EINVAL;
		}

		control2 = FIELD_PREP(TPS6287X_CONTROL2_VRANGE_MASK, vrange);
		rc = i2c_reg_write_byte_dt(&cfg->i2c, TPS6287X_REG_CONTROL2, control2);
		if (rc < 0) {
			return rc;
		}
	}

	LOG_INF("%s: Setting voltage to range %u, index %u", dev->name, vrange, idx);

	return i2c_reg_write_byte_dt(&cfg->i2c, TPS6287X_REG_VSET, (uint8_t)idx);
}

static int regulator_tps6287x_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_tps6287x_config *cfg =
		(const struct regulator_tps6287x_config *)dev->config;
	int rc = 0;
	uint8_t control2 = 0;

	rc = i2c_reg_read_byte_dt(&cfg->i2c, TPS6287X_REG_CONTROL2, &control2);
	if (rc < 0) {
		return rc;
	}

	uint8_t vrange = FIELD_GET(TPS6287X_CONTROL2_VRANGE_MASK, control2);
	uint8_t idx = 0;

	rc = i2c_reg_read_byte_dt(&cfg->i2c, TPS6287X_REG_VSET, &idx);
	if (rc < 0) {
		return rc;
	}

	rc = linear_range_get_value(&voltage_ranges[vrange], (uint16_t)idx, volt_uv);

	LOG_INF("%s: Got voltage: %d uV (range %u, index %u)", dev->name, *volt_uv, vrange, idx);

	return rc;
}

static int regulator_tps6287x_set_active_discharge(const struct device *dev, bool active_discharge)
{
	const struct regulator_tps6287x_config *cfg = dev->config;

	return i2c_reg_update_byte_dt(&cfg->i2c, TPS6287X_REG_CONTROL1, TPS6287X_CONTROL1_DISCHEN,
				      active_discharge ? TPS6287X_CONTROL1_DISCHEN : 0);
}

static int regulator_tps6287x_get_active_discharge(const struct device *dev, bool *active_discharge)
{
	const struct regulator_tps6287x_config *cfg = dev->config;
	uint8_t control1;
	int rc;

	rc = i2c_reg_read_byte_dt(&cfg->i2c, TPS6287X_REG_CONTROL1, &control1);
	if (rc == 0) {
		*active_discharge = control1 & TPS6287X_CONTROL1_DISCHEN;
	}

	return rc;
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
	const struct regulator_tps6287x_config *cfg = dev->config;
	bool is_enabled = false;
	int rc = 0;
	uint8_t status;

	rc = i2c_reg_read_byte_dt(&cfg->i2c, TPS6287X_REG_CONTROL1, &status);
	if (rc < 0) {
		return rc;
	}

	is_enabled = (status & 0x20) != 0;

	regulator_common_data_init(dev);

	/* TO BE REMOVED!!!! */
	i2c_reg_update_byte_dt(&cfg->i2c, TPS6287X_REG_CONTROL1, 0x80, 0x80);
	k_sleep(K_MSEC(1));

	rc = regulator_common_init(dev, is_enabled);
	if (rc < 0) {
		LOG_ERR("%s: Failed to initialize regulator: %d", dev->name, rc);
	}
	return rc;
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
#define SANITY_CHECK_INIT_MICROVOLT(inst) \
	BUILD_ASSERT(DT_PROP_OR(DT_DRV_INST(inst), regulator_init_microvolt, INT32_MIN) < \
				(DT_INST_PROP(inst, input_voltage_microvolt) - TPS6287x_MIN_DIV_OUTPUT),  \
		     "input-voltage-microvolt must be at least 1.4V greater than regulator-init-microvolt")

#define REGULATOR_TPS6287X_DEFINE_ALL(inst) \
	SANITY_CHECK_INIT_MICROVOLT(inst); \
	\
	static struct regulator_tps62873_data data_##inst; \
                                                        \
	static const struct regulator_tps6287x_config config_##inst = { \
		.common = REGULATOR_DT_INST_COMMON_CONFIG_INIT(inst), \
		.i2c = I2C_DT_SPEC_INST_GET(inst), \
        .input_voltage_uv = DT_INST_PROP(inst, input_voltage_microvolt) \
	};                                               \
                                                     \
	DEVICE_DT_INST_DEFINE(inst, regulator_tps6287x_init, NULL, &data_##inst, &config_##inst, \
			      POST_KERNEL, CONFIG_REGULATOR_TPS6287X_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(REGULATOR_TPS6287X_DEFINE_ALL)
/* clang-format on */

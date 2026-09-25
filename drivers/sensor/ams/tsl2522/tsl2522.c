/*
 * Copyright (c) 2026 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ams_tsl2522

#include "tsl2522.h"

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(tsl2522, CONFIG_SENSOR_LOG_LEVEL);

/* gain TSL2522_GAIN_MOD_HALF returns 500, TSL2522_GAIN_MOD_1X returns 1000, ... */
static uint32_t get_gain_value(enum sensor_gain_tsl2522 again)
{
	if (again >= TSL2522_GAIN_MOD_1X && again <= TSL2522_GAIN_MOD_4096X) {
		return 1000U * (1U << (again - 1));
	} else if (again == TSL2522_GAIN_MOD_HALF) {
		return 500U;
	}
	return 0;
}

/* Information provided by ams OSRAM:
 * TSL2522:  If IR/PHO < 1.074 n=1, else n=2: Lux = DGFn*((CoefAn*Ch0)+(CoefBn*Ch1))/(ATime*AGain)
 * TSL2522:  n = Seg-n coefficient number
 * 	n=1 (L)  n=2 (H)
 * CoefA  0.6132   0.6099
 * CoefB -0.1557  -0.1133
 * DGF    2.4529   2.4398
 *
 * ATIME in [ms]
 * CH0: PHOTOPIC
 * CH1: IR
 */
static uint32_t tsl2522_calc_lux(uint32_t pho, uint32_t ir, uint32_t atime_us,
				 enum sensor_gain_tsl2522 again)
{
	int64_t numerator;
	uint64_t denominator;
	uint32_t gain = get_gain_value(again);

	if ((uint64_t)ir * 1000ULL < (uint64_t)pho * 1074ULL) {
		numerator = (int64_t)TSL2522_L_A * pho + (int64_t)TSL2522_L_B * ir;
	} else {
		numerator = (int64_t)TSL2522_H_A * pho + (int64_t)TSL2522_H_B * ir;
	}

	if (numerator <= 0) {
		return 0;
	}

	denominator = (uint64_t)TSL2522_SCALE * atime_us / 1000U * gain;

	return (uint32_t)(numerator / denominator);
}

static void log_state(uint8_t status2_5[4])
{
#if CONFIG_SENSOR_LOG_LEVEL >= LOG_LEVEL_DBG
	LOG_DBG("status2 (0x%02x): als data valid %d, dig sat %d, flicker det sat %d, mod sat1 %d, "
		"mod sat0 %d",
		status2_5[0], (bool)FIELD_GET(TSL2522_STATUS2_ALS_DATA_VALID, status2_5[0]),
		(bool)FIELD_GET(TSL2522_STATUS2_ALS_DIG_SAT, status2_5[0]),
		(bool)FIELD_GET(TSL2522_STATUS2_ALS_FD_DIG_SAT, status2_5[0]),
		(bool)FIELD_GET(TSL2522_STATUS2_MOD_ANA_SAT1, status2_5[0]),
		(bool)FIELD_GET(TSL2522_STATUS2_MOD_ANA_SAT0, status2_5[0]));
	LOG_DBG("status3 (0x%02x): aint hyst state valid %d, aint hyst state read %d, aint high %d,"
		" aint low thres %d, osc cal sat %d, osc cal finished %d",
		status2_5[1], (bool)FIELD_GET(TSL2522_STATUS3_AINT_HYST_STATE_VALID, status2_5[1]),
		(bool)FIELD_GET(TSL2522_STATUS3_AINT_HYST_STATE_RD, status2_5[1]),
		(bool)FIELD_GET(TSL2522_STATUS3_AINT_AIHT, status2_5[1]),
		(bool)FIELD_GET(TSL2522_STATUS3_AINT_AILT, status2_5[1]),
		(bool)FIELD_GET(TSL2522_STATUS3_OSC_CALIB_SATURATION, status2_5[1]),
		(bool)FIELD_GET(TSL2522_STATUS3_OSC_CALIB_FINISHED, status2_5[1]));
	LOG_DBG("status4 (0x%02x): mod sample trigger error %d, mod trigger error %d, sai_active "
		"%d, init busy %d",
		status2_5[2],
		(bool)FIELD_GET(TSL2522_STATUS4_MOD_SAMPLE_TRIGGER_ERROR, status2_5[2]),
		(bool)FIELD_GET(TSL2522_STATUS4_MOD_TRIGGER_ERROR, status2_5[2]),
		(bool)FIELD_GET(TSL2522_STATUS4_SAI_ACTIVE, status2_5[2]),
		(bool)FIELD_GET(TSL2522_STATUS4_INIT_BUSY, status2_5[2]));
	LOG_DBG("status5 (0x%02x): meas seq sys int %d, vsync lost %d", status2_5[3],
		(bool)FIELD_GET(TSL2522_STATUS5_SINT_MEASUREMENT_SEQUENCER, status2_5[3]),
		(bool)FIELD_GET(TSL2522_STATUS5_SINT_VSYNC, status2_5[3]));
#endif
}

static int internal_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int rc = 0;
	const struct tsl2522_dts_config *cfg = dev->config;
	struct tsl2522_data *data = dev->data;
	uint8_t status2_5[4];
	uint8_t als_status = 0U;
	uint8_t als_data[4];
	bool als_data_valid = false;
	bool measured_data_valid = false;

	/* Read the status fields 2...5 in one read. */
	rc = i2c_burst_read_dt(&cfg->i2c, TSL2522_REG_STATUS2, status2_5, sizeof(status2_5));
	if (rc < 0) {
		return rc;
	}

	als_data_valid = (bool)FIELD_GET(TSL2522_STATUS2_ALS_DATA_VALID, status2_5[0]);
	measured_data_valid =
		!(bool)FIELD_GET(TSL2522_STATUS4_MOD_SAMPLE_TRIGGER_ERROR, status2_5[2]);

	log_state(status2_5);

	if (als_data_valid && measured_data_valid) {
		/*Fetch actual data. */
		rc = i2c_reg_read_byte_dt(&cfg->i2c, TSL2522_REG_ALS_STATUS, &als_status);
		if (rc < 0) {
			return rc;
		}
		LOG_INF("als_status (0x%02x): seq step %lu, ana_sat_dat0 %d, ana_sat_dat1 %d, "
			"dat0_scaled "
			"%d, "
			"dat1_scaled %d",
			als_status, FIELD_GET(TSL2522_ALS_STATUS_MEAS_SEQR_STEP, als_status),
			(bool)FIELD_GET(TSL2522_ALS_STATUS_DATA0_ANA_SAT, als_status),
			(bool)FIELD_GET(TSL2522_ALS_STATUS_DATA1_ANA_SAT, als_status),
			(bool)FIELD_GET(TSL2522_ALS_STATUS_DATA0_SCALED, als_status),
			(bool)FIELD_GET(TSL2522_ALS_STATUS_DATA1_SCALED, als_status));

		rc = i2c_burst_read_dt(&cfg->i2c, TSL2522_REG_ALS_DATA, als_data, sizeof(als_data));
		if (rc < 0) {
			return rc;
		}

		if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_AMBIENT_LIGHT ||
		    chan == SENSOR_CHAN_LIGHT) {
			data->photopic_channel = (uint32_t)sys_get_le16(&als_data[0]);
			if (!FIELD_GET(TSL2522_ALS_STATUS_DATA0_SCALED, als_status)) {
				data->photopic_channel = data->photopic_channel << data->als_scale;
			}
		}

		if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_AMBIENT_LIGHT ||
		    chan == SENSOR_CHAN_IR) {
			data->ir_channel = (uint32_t)sys_get_le16(&als_data[2]);
			if (!FIELD_GET(TSL2522_ALS_STATUS_DATA1_SCALED, als_status)) {
				data->ir_channel = data->ir_channel << data->als_scale;
			}
		}

		data->atime_us = data->number_of_samples * data->sample_time_us;
	} else {
		if (!als_data_valid) {
			LOG_ERR("als data invalid!");
		}

		if (!measured_data_valid) {
			LOG_ERR("Measured data corrupted!");
			/* Clear all bits in status4. */
			rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_STATUS4, status2_5[2]);
			if (rc < 0) {
				return rc;
			}
		}
		rc = -EINVAL;
	}

	return rc;
}

static int tsl2522_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int rc = 0;
	struct tsl2522_data *data = dev->data;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_AMBIENT_LIGHT &&
	    chan != SENSOR_CHAN_LIGHT && chan != SENSOR_CHAN_IR) {
		return -ENOTSUP;
	}

	k_sem_take(&data->sem, K_FOREVER);

	rc = internal_sample_fetch(dev, chan);

	k_sem_give(&data->sem);

	return rc;
}

static int tsl2522_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct tsl2522_data *data = dev->data;
	int rc = 0;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_LIGHT:
		val->val1 = tsl2522_calc_lux(data->photopic_channel, data->ir_channel,
					     data->atime_us, data->gain);
		break;
	case SENSOR_CHAN_LIGHT:
		val->val1 =
			tsl2522_calc_lux(data->photopic_channel, 0U, data->atime_us, data->gain);
		break;
	case SENSOR_CHAN_IR:
		val->val1 = tsl2522_calc_lux(0U, data->ir_channel, data->atime_us, data->gain);
		break;
	default:
		val->val1 = 0;
		rc = -ENOTSUP;
		break;
	}

	val->val2 = 0;

	return rc;
}

static DEVICE_API(sensor, tsl2522_driver_api) = {
	.sample_fetch = tsl2522_sample_fetch,
	.channel_get = tsl2522_channel_get,
};

static int reset_device(const struct device *dev)
{
	int rc = 0;
	const struct tsl2522_dts_config *cfg = dev->config;

	rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_ENABLE, TSL2522_ENABLE_PON);
	if (rc < 0) {
		return rc;
	}

	k_busy_wait(500);

	return 0;
}

static int setup_device(const struct device *dev)
{
	int rc = 0;
	const struct tsl2522_dts_config *cfg = dev->config;
	struct tsl2522_data *data = dev->data;

	if (data->sample_time_us >= TLS2522_MIN_SAMPLE_TIME_MS &&
	    data->sample_time_us <= TLS2522_MAX_SAMPLE_TIME_MS) {
		uint8_t sample_time[2];

		sys_put_le16(convert_us_to_counts(data->sample_time_us), sample_time);
		rc = i2c_burst_write_dt(&cfg->i2c, TSL2522_REG_SAMPLE_TIME0, sample_time,
					sizeof(sample_time));
		if (rc < 0) {
			return rc;
		}
	} else {
		return -EINVAL;
	}

	if (data->number_of_samples > 0 && data->number_of_samples <= TLS2522_MAX_NR_SAMPLES) {
		uint8_t nr_samples[2];
		sys_put_le16(data->number_of_samples - 1U, nr_samples);
		rc = i2c_burst_write_dt(&cfg->i2c, TSL2522_REG_ALS_NR_SAMPLES0, nr_samples,
					sizeof(nr_samples));
		if (rc < 0) {
			return rc;
		}
	} else {
		return -EINVAL;
	}

	rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_TRIGGER_MODE,
				   TSL2522_TRIGGER_MODE_NORMAL);
	if (rc < 0) {
		return rc;
	}

	rc = i2c_reg_write_byte_dt(
		&cfg->i2c, TSL2522_REG_MEAS_SEQR_STEP0_MOD_GAINX_L,
		FIELD_PREP(TSL2522_MEAS_SEQR_STEP0_MOD_GAIN1, data->gain) |
			FIELD_PREP(TSL2522_MEAS_SEQR_STEP0_MOD_GAIN0, data->gain));
	if (rc < 0) {
		return rc;
	}

	/* Assign photopic diodes to modulator 0 and the IR diodes to modulator 1. */
	rc = i2c_reg_write_byte_dt(
		&cfg->i2c, TSL2522_REG_MEAS_SEQR_STEP0_MOD_PHDX_SMUX_L,
		FIELD_PREP(TSL2522_MEAS_SEQR_STEP0_MOD_PHD3, TSL2522_MOD_SEL_MOD_0) |
			FIELD_PREP(TSL2522_MEAS_SEQR_STEP0_MOD_PHD2, TSL2522_MOD_SEL_MOD_0) |
			FIELD_PREP(TSL2522_MEAS_SEQR_STEP0_MOD_PHD1, TSL2522_MOD_SEL_MOD_0) |
			FIELD_PREP(TSL2522_MEAS_SEQR_STEP0_MOD_PHD0, TSL2522_MOD_SEL_MOD_1));
	if (rc < 0) {
		return rc;
	}

	rc = i2c_reg_write_byte_dt(
		&cfg->i2c, TSL2522_REG_MEAS_SEQR_STEP0_MOD_PHDX_SMUX_H,
		FIELD_PREP(TSL2522_MEAS_SEQR_STEP0_MOD_PHD5, TSL2522_MOD_SEL_MOD_1) |
			FIELD_PREP(TSL2522_MEAS_SEQR_STEP0_MOD_PHD4, TSL2522_MOD_SEL_MOD_0));
	if (rc < 0) {
		return rc;
	}

	/* Enable ALS and device. */
	rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_ENABLE,
				   TSL2522_ENABLE_PON | TSL2522_ENABLE_AEN);
	if (rc < 0) {
		return rc;
	}

	uint8_t status4 = 0;

	rc = i2c_reg_read_byte_dt(&cfg->i2c, TSL2522_REG_STATUS4, &status4);
	if (rc < 0) {
		return rc;
	}

	if (FIELD_GET(TSL2522_STATUS4_MOD_TRIGGER_ERROR, status4)) {
		LOG_ERR("WTIME is too short for the programmed configuration (SAMPLE_TIME, "
			"ALS_NR_SAMPLES).");
		return -EINVAL;
	}

	if (status4 != 0) {
		/* Clear all bits in status4. */
		rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_STATUS4, status4);
		if (rc < 0) {
			return rc;
		}
	}

	return 0;
}

static int tsl2522_init(const struct device *dev)
{
	const struct tsl2522_dts_config *cfg = dev->config;
	struct tsl2522_data *data = dev->data;
	int rc = -EAGAIN;
	uint8_t devid = 0;

	data->als_scale = 4U;
	data->sample_time_us = 1000U;
	data->number_of_samples = 63U;
	data->gain = TSL2522_GAIN_MOD_16X;

	k_sem_init(&data->sem, 1, K_SEM_MAX_LIMIT);

	/* Try to access device. */
	for (int i = 0; i < TSL2522_MAX_INIT_RETRY && rc != 0; i++) {
		rc = i2c_reg_read_byte_dt(&cfg->i2c, TSL2522_REG_DEVICE_ID, &devid);
		if (rc < 0) {
			k_busy_wait(100);
		}
	}

	if (rc < 0) {
		return rc;
	}

	if (devid != TSL2522_DEVICE_ID) {
		LOG_ERR("Invalid chip ID (was 0x%2x, expected 0x5c)", devid);
		return -EIO;
	}

	rc = reset_device(dev);
	if (rc < 0) {
		return rc;
	}

	rc = setup_device(dev);
	return rc;
}

#define TSL2522_DEFINE(inst)                                                                       \
	static const struct tsl2522_dts_config tsl2522_config_##inst = {                           \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
	};                                                                                         \
                                                                                                   \
	static struct tsl2522_data tsl2522_data_##inst;                                            \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, tsl2522_init, NULL, &tsl2522_data_##inst,               \
				     &tsl2522_config_##inst, POST_KERNEL,                          \
				     CONFIG_SENSOR_INIT_PRIORITY, &tsl2522_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TSL2522_DEFINE)

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

/*
 * Information provided by ams OSRAM:
 * TSL2522: If IR/PHO < 1.074 n=1, else n=2: Lux = DGFn*((CoefAn*Ch0)+(CoefBn*Ch1))/(ATime*AGain)
 * TSL2522: n = Seg-n coefficient number
 * n=1 (L) n=2 (H)
 * CoefA  0.6132   0.6099
 * CoefB -0.1557  -0.1133
 * DGF    2.4529   2.4398
 *
 * ATIME in [ms]
 * CH0: PHOTOPIC
 * CH1: IR
 */
static int32_t tsl2522_calc_lux(const struct tsl2522_dts_config *cfg,
				struct tsl2522_measurement *measurement)
{
	int64_t numerator;
	uint64_t denominator;

	int64_t pho = (int64_t)measurement->photopic_channel * cfg->glass_attenuation;
	int64_t ir = (int64_t)measurement->ir_channel * cfg->glass_ir_attenuation;

	if (ir * 1000ULL < pho * 1074ULL) {
		numerator = (int64_t)TSL2522_L_A * pho + (int64_t)TSL2522_L_B * ir;
	} else {
		numerator = (int64_t)TSL2522_H_A * pho + (int64_t)TSL2522_H_B * ir;
	}

	if (numerator <= 0) {
		if (measurement->photopic_channel == 0) {
			/* Get IR channel only. */
			numerator = -numerator;
		} else {
			return 0;
		}
	}

	denominator =
		(uint64_t)(TSL2522_SCALE / US_IN_MS) * measurement->atime_us * measurement->gain;

	return (int32_t)(numerator / denominator);
}

static void log_state(const uint8_t *status2_5)
{
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
}

static int internal_sample_fetch(const struct device *dev)
{
	int rc = 0;
	const struct tsl2522_dts_config *cfg = dev->config;
	struct tsl2522_data *data = dev->data;
	uint8_t status = 0;
	uint8_t status2_5[4];
	uint8_t als_status = 0U;
	uint8_t als_data[4];
	bool als_data_valid = false;
	bool measured_data_valid = false;

	/* Read status in order to clear the modulator saturation bits. */
	rc = i2c_reg_read_byte_dt(&cfg->i2c, TSL2522_REG_STATUS, &status);
	if (rc < 0) {
		return rc;
	}

	/* Read the status fields 2...5 in one read. */
	rc = i2c_burst_read_dt(&cfg->i2c, TSL2522_REG_STATUS2, status2_5, sizeof(status2_5));
	if (rc < 0) {
		goto exit;
	}

	als_data_valid = (bool)FIELD_GET(TSL2522_STATUS2_ALS_DATA_VALID, status2_5[0]);
	measured_data_valid =
		!(bool)FIELD_GET(TSL2522_STATUS4_MOD_SAMPLE_TRIGGER_ERROR, status2_5[2]);

	log_state(status2_5);

	if (als_data_valid && measured_data_valid) {
		/* Fetch actual data. */
		rc = i2c_reg_read_byte_dt(&cfg->i2c, TSL2522_REG_ALS_STATUS, &als_status);
		if (rc < 0) {
			goto exit;
		}
		LOG_DBG("als_status (0x%02x): seq step %lu, ana_sat_dat0 %d, ana_sat_dat1 %d, "
			"dat0_scaled %d, dat1_scaled %d",
			als_status, FIELD_GET(TSL2522_ALS_STATUS_MEAS_SEQR_STEP, als_status),
			(bool)FIELD_GET(TSL2522_ALS_STATUS_DATA0_ANA_SAT, als_status),
			(bool)FIELD_GET(TSL2522_ALS_STATUS_DATA1_ANA_SAT, als_status),
			(bool)FIELD_GET(TSL2522_ALS_STATUS_DATA0_SCALED, als_status),
			(bool)FIELD_GET(TSL2522_ALS_STATUS_DATA1_SCALED, als_status));

		data->measurement.saturation =
			(bool)FIELD_GET(TSL2522_ALS_STATUS_DATA0_ANA_SAT, als_status) ||
			(bool)FIELD_GET(TSL2522_ALS_STATUS_DATA1_ANA_SAT, als_status) ||
			(bool)FIELD_GET(TSL2522_ALS_STATUS_DATA0_SCALED, als_status) ||
			(bool)FIELD_GET(TSL2522_ALS_STATUS_DATA1_SCALED, als_status);

		rc = i2c_burst_read_dt(&cfg->i2c, TSL2522_REG_ALS_DATA, als_data, sizeof(als_data));
		if (rc < 0) {
			goto exit;
		}

		data->measurement.photopic_channel = (uint32_t)sys_get_le16(&als_data[0]);
		if (!FIELD_GET(TSL2522_ALS_STATUS_DATA0_SCALED, als_status)) {
			data->measurement.photopic_channel = data->measurement.photopic_channel
							     << data->als_scale;
		}
		data->measurement.ir_channel = (uint32_t)sys_get_le16(&als_data[2]);
		if (!FIELD_GET(TSL2522_ALS_STATUS_DATA1_SCALED, als_status)) {
			data->measurement.ir_channel = data->measurement.ir_channel
						       << data->als_scale;
		}

		data->measurement.atime_us = data->number_of_samples * data->time_per_sample_us;
		data->measurement.gain = data->gain;
	} else {
		if (!als_data_valid) {
			LOG_DBG("als data not yet available!");
			rc = -EAGAIN;
		} else {
			LOG_ERR("Measured data corrupted!");
			/* Clear all bits in status4. */
			(void)i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_STATUS4, status2_5[2]);
			rc = -ENODATA;
		}
	}

exit:
	/* Clear the modulator saturation bits, no check of return. */
	(void)i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_STATUS, status);

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

	k_mutex_lock(&data->mutex, K_FOREVER);

	rc = internal_sample_fetch(dev);

	k_mutex_unlock(&data->mutex);

	return rc;
}

static int tsl2522_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct tsl2522_data *data = dev->data;
	const struct tsl2522_dts_config *cfg = dev->config;
	int rc = 0;

	k_mutex_lock(&data->mutex, K_FOREVER);

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_LIGHT:
		val->val1 = tsl2522_calc_lux(cfg, &data->measurement);
		break;
	case SENSOR_CHAN_LIGHT:
		val->val1 = tsl2522_calc_lux(cfg, &data->measurement);
		break;
	case SENSOR_CHAN_IR:
		val->val1 = tsl2522_calc_lux(cfg, &data->measurement);
		break;
	default:
		val->val1 = 0;
		rc = -ENOTSUP;
		break;
	}

	if (data->measurement.saturation) {
		rc = -EOVERFLOW;
	}

	val->val2 = 0;

	k_mutex_unlock(&data->mutex);

	return rc;
}

static int set_modulator_gain(const struct device *dev, enum sensor_gain_tsl2522 gain)
{
	const struct tsl2522_dts_config *cfg = dev->config;

	return i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_MEAS_SEQR_STEP0_MOD_GAINX_L,
				     FIELD_PREP(TSL2522_MEAS_SEQR_STEP0_MOD_GAIN1, gain) |
					     FIELD_PREP(TSL2522_MEAS_SEQR_STEP0_MOD_GAIN0, gain));
}

static int set_time_per_sample_us(const struct device *dev, uint16_t time_per_sample_us)
{
	const struct tsl2522_dts_config *cfg = dev->config;
	uint8_t sample_time[2];
	uint16_t counts = tsl2522_convert_us_to_counts(time_per_sample_us);

	sys_put_le16(counts, sample_time);
	return i2c_burst_write_dt(&cfg->i2c, TSL2522_REG_SAMPLE_TIME0, sample_time,
				  sizeof(sample_time));
}

static int set_number_of_samples(const struct device *dev, uint16_t number_of_samples)
{
	const struct tsl2522_dts_config *cfg = dev->config;
	uint8_t nr_samples[2];

	sys_put_le16(number_of_samples - 1U, nr_samples);
	return i2c_burst_write_dt(&cfg->i2c, TSL2522_REG_ALS_NR_SAMPLES0, nr_samples,
				  sizeof(nr_samples));
}

static int enable_ambient_light_sensing(const struct device *dev)
{
	const struct tsl2522_dts_config *cfg = dev->config;

	/* Enable ALS and device. */
	return i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_ENABLE,
				     TSL2522_ENABLE_PON | TSL2522_ENABLE_AEN);
}

static int disable_ambient_light_sensing(const struct device *dev)
{
	const struct tsl2522_dts_config *cfg = dev->config;

	/* Disable ALS and device. */
	return i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_ENABLE, 0U);
}

static int tsl2522_attribute_get(const struct device *dev, enum sensor_channel chan,
				 enum sensor_attribute attr, struct sensor_value *val)
{
	int rc = 0;
	struct tsl2522_data *data = dev->data;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_AMBIENT_LIGHT &&
	    chan != SENSOR_CHAN_LIGHT && chan != SENSOR_CHAN_IR) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->mutex, K_FOREVER);
	if (attr == SENSOR_ATTR_GAIN) {
		val->val1 = convert_gain_value_to_enum(data->gain);
		val->val2 = 0;
	} else {
		switch ((enum sensor_attribute_tsl2522)attr) {
		case SENSOR_ATTR_TIME_PER_SAMPLE_US:
			val->val1 =
				tsl2522_convert_sample_time_us_to_enum(data->time_per_sample_us);
			val->val2 = 0;
			break;
		case SENSOR_ATTR_NUMBER_OF_SAMPLES:
			val->val1 = data->number_of_samples;
			val->val2 = 0;
			break;
		default:
			rc = -ENOTSUP;
			break;
		}
	}
	k_mutex_unlock(&data->mutex);

	return rc;
}

static int tsl2522_attribute_set(const struct device *dev, enum sensor_channel chan,
				 enum sensor_attribute attr, const struct sensor_value *val)
{
	int rc = 0;
	struct tsl2522_data *data = dev->data;
	bool enable_sensing = false;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_AMBIENT_LIGHT &&
	    chan != SENSOR_CHAN_LIGHT && chan != SENSOR_CHAN_IR) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->mutex, K_FOREVER);

	if (attr == SENSOR_ATTR_GAIN) {
		enum sensor_gain_tsl2522 gain_enum = (enum sensor_gain_tsl2522)val->val1;

		if (IN_RANGE(gain_enum, TSL2522_GAIN_MOD_HALF, TSL2522_GAIN_MOD_4096X)) {
			uint32_t gain = tsl2522_convert_gain_enum_to_value(gain_enum);

			if (gain != data->gain) {
				rc = disable_ambient_light_sensing(dev);
				if (rc < 0) {
					k_mutex_unlock(&data->mutex);
					return rc;
				}
				enable_sensing = true;
				rc = set_modulator_gain(dev, gain_enum);
				if (rc == 0) {
					data->gain = gain;
				}
			}
		} else {
			rc = -EINVAL;
		}
	} else {
		switch ((enum sensor_attribute_tsl2522)attr) {
		case SENSOR_ATTR_TIME_PER_SAMPLE_US:
			if (IN_RANGE(val->val1, TSL2522_100US_PER_SAMPLE,
				     TSL2522_1000US_PER_SAMPLE)) {
				enum us_per_sample_tsl2522 time_per_sample_us_enum =
					(enum us_per_sample_tsl2522)val->val1;

				/* Convert it from enum to actual us. */
				uint16_t time_per_sample_us =
					tsl2522_convert_sample_time_enum_to_us(
						time_per_sample_us_enum);

				if (time_per_sample_us != data->time_per_sample_us) {
					rc = disable_ambient_light_sensing(dev);
					if (rc < 0) {
						k_mutex_unlock(&data->mutex);
						return rc;
					}
					enable_sensing = true;
					rc = set_time_per_sample_us(dev, time_per_sample_us);
					if (rc == 0) {
						data->time_per_sample_us = time_per_sample_us;
					}
				}
			} else {
				rc = -EINVAL;
			}
			break;
		case SENSOR_ATTR_NUMBER_OF_SAMPLES:
			if (IN_RANGE(val->val1, NUMBER_OF_SAMPLES_MIN, NUMBER_OF_SAMPLES_MAX)) {
				uint16_t number_of_samples = (uint16_t)val->val1;

				if (number_of_samples != data->number_of_samples) {
					rc = disable_ambient_light_sensing(dev);
					if (rc < 0) {
						k_mutex_unlock(&data->mutex);
						return rc;
					}
					enable_sensing = true;
					rc = set_number_of_samples(dev, number_of_samples);
					if (rc == 0) {
						data->number_of_samples = number_of_samples;
					}
				}
			} else {
				rc = -EINVAL;
			}
			break;
		default:
			rc = -ENOTSUP;
			break;
		}
	}

	if (enable_sensing) {
		enable_ambient_light_sensing(dev);
	}

	k_mutex_unlock(&data->mutex);

	return rc;
}

static DEVICE_API(sensor, tsl2522_driver_api) = {.sample_fetch = tsl2522_sample_fetch,
						 .channel_get = tsl2522_channel_get,
						 .attr_get = tsl2522_attribute_get,
						 .attr_set = tsl2522_attribute_set};

static int reset_device(const struct device *dev)
{
	int rc = 0;
	const struct tsl2522_dts_config *cfg = dev->config;

	rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_ENABLE, TSL2522_ENABLE_PON);
	if (rc < 0) {
		return rc;
	}

	rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_CONTROL, TSL2522_CONTROL_SOFT_RESET);
	if (rc < 0) {
		return rc;
	}

	k_busy_wait(TSL2522_SOFTRESET_WAIT_US);

	return 0;
}

static int setup_device(const struct device *dev)
{
	int rc = 0;
	const struct tsl2522_dts_config *cfg = dev->config;
	struct tsl2522_data *data = dev->data;
	uint8_t measure_mode = 0;

	rc = i2c_reg_read_byte_dt(&cfg->i2c, TSL2522_REG_MEAS_MODE, &measure_mode);
	if (rc < 0) {
		return rc;
	}

	data->als_scale = FIELD_GET(TSL2522_MEAS_MODE_ALS_SCALE, measure_mode);
	data->measurement.atime_us = data->number_of_samples * data->time_per_sample_us;

	rc = set_time_per_sample_us(dev, data->time_per_sample_us);
	if (rc < 0) {
		return rc;
	}

	rc = set_number_of_samples(dev, data->number_of_samples);
	if (rc < 0) {
		return rc;
	}

	rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_TRIGGER_MODE,
				   TSL2522_TRIGGER_MODE_NORMAL);
	if (rc < 0) {
		return rc;
	}

	rc = set_modulator_gain(dev, cfg->gain_enum);
	if (rc < 0) {
		return rc;
	}
	data->gain = tsl2522_convert_gain_enum_to_value(cfg->gain_enum);

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
	rc = enable_ambient_light_sensing(dev);
	if (rc < 0) {
		return rc;
	}

	uint8_t status4 = 0;

	rc = i2c_reg_read_byte_dt(&cfg->i2c, TSL2522_REG_STATUS4, &status4);
	if (rc < 0) {
		return rc;
	}

	/* Clear all bits in status4. */
	return i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_STATUS4, status4);
}

static int tsl2522_init(const struct device *dev)
{
	const struct tsl2522_dts_config *cfg = dev->config;
	struct tsl2522_data *data = dev->data;
	int rc = -EAGAIN;
	uint8_t devid = 0;

	k_mutex_init(&data->mutex);

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR_DEVICE_NOT_READY(cfg->i2c.bus);
		return -ENODEV;
	}

	/* Try to access device. */
	for (int i = 0; i < TSL2522_MAX_INIT_RETRY && rc != 0; i++) {
		rc = i2c_reg_read_byte_dt(&cfg->i2c, TSL2522_REG_DEVICE_ID, &devid);
		if (rc < 0) {
			k_busy_wait(TSL2522_RETRY_ACCESS_US);
		}
	}

	if (rc < 0) {
		return rc;
	}

	if (devid != TSL2522_DEVICE_ID) {
		LOG_ERR("Invalid chip ID (was 0x%2x, expected 0x%2x)", devid, TSL2522_DEVICE_ID);
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
		.glass_attenuation = DT_INST_PROP(inst, glass_attenuation),                        \
		.glass_ir_attenuation = DT_INST_PROP(inst, glass_ir_attenuation),                  \
		.gain_enum = (enum sensor_gain_tsl2522)DT_INST_ENUM_IDX(inst, modulator_gain),     \
	};                                                                                         \
                                                                                                   \
	static struct tsl2522_data tsl2522_data_##inst = {                                         \
		.time_per_sample_us =                                                              \
			(1U + DT_INST_ENUM_IDX(inst, time_per_sample_us)) * SAMPLE_TIME_STEP_US,   \
		.number_of_samples = (uint16_t)DT_INST_PROP(inst, number_of_samples),              \
	};                                                                                         \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, tsl2522_init, NULL, &tsl2522_data_##inst,               \
				     &tsl2522_config_##inst, POST_KERNEL,                          \
				     CONFIG_SENSOR_INIT_PRIORITY, &tsl2522_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TSL2522_DEFINE)

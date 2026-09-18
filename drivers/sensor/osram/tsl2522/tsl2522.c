/*
 * Copyright (c) 2026 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(tsl2522, CONFIG_SENSOR_LOG_LEVEL);

#define TSL2522_MAX_INIT_RETRY 5U
#define TSL2522_DEVICE_ID      0x5C

#define TSL2522_SCALE              1000000LL
#define TLS2522_MAX_SAMPLE_TIME_MS (2048U / 4U)
#define TLS2522_TICKS_PER_MS       (180U * 4U)
#define TLS2522_MAX_NR_SAMPLES     2048U

/* Low segment */
#define TSL2522_L_A 1504118LL
#define TSL2522_L_B (-381917LL)

/* High segment */
#define TSL2522_H_A 1488034LL
#define TSL2522_H_B (-276429LL)

#define TSL2522_REG_ENABLE  0x80
#define TSL2522_ENABLE_FDEN BIT(6)
#define TSL2522_ENABLE_AEN  BIT(1)
#define TSL2522_ENABLE_PON  BIT(0)

#define TSL2522_REG_MEAS_MODE1                        0x82
#define TSL2522_MEAS_MODE1_MOD_FIFO_FD_END_MARKER_WEN BIT(7)
#define TSL2522_MEAS_MODE1_MOD_FIFO_FD_CHECKSUM_WEN   BIT(6)
#define TSL2522_MEAS_MODE1_MOD_FIFO_FD_GAIN_WEN       BIT(5)
#define TSL2522_MEAS_MODE1_ALS_MSB_POSITION_MASK      GENMASK(4, 0)
#define TSL2522_MEAS_MODE1_ALS_MSB_POSITION_DEFAULT   0x08

#define TSL2522_REG_SAMPLE_TIME0 0x83

#define TSL2522_REG_ALS_NR_SAMPLES0 0x85

#define TSL2522_REG_WTIME     0x89
#define TSL2522_WTIME_DEFAULT 0x46

#define TSL2522_REG_DEVICE_ID 0x92

#define TSL2522_REG_STATUS  0x93
#define TSL2522_STATUS_MINT BIT(7)
#define TSL2522_STATUS_AINT BIT(3)
#define TSL2522_STATUS_FINT BIT(2)
#define TSL2522_STATUS_SINT BIT(0)

#define TSL2522_REG_ALS_STATUS            0x94
#define TSL2522_ALS_STATUS_MEAS_SEQR_STEP GENMASK(7, 6)
#define TSL2522_ALS_STATUS_DATA0_ANA_SAT  BIT(5)
#define TSL2522_ALS_STATUS_DATA1_ANA_SAT  BIT(4)
#define TSL2522_ALS_STATUS_DATA0_SCALED   BIT(2)
#define TSL2522_ALS_STATUS_DATA1_SCALED   BIT(1)

#define TSL2522_REG_ALS_DATA 0x95

#define TSL2522_REG_ALS_STATUS2        0x9B
#define TSL2522_ALS_STATUS2_DATA1_GAIN GENMASK(7, 4)
#define TSL2522_ALS_STATUS2_DATA0_GAIN GENMASK(3, 0)

#define TSL2522_REG_STATUS2            0x9D
#define TSL2522_STATUS2_ALS_DATA_VALID BIT(6)
#define TSL2522_STATUS2_ALS_DIG_SAT    BIT(4)
#define TSL2522_STATUS2_ALS_FD_DIG_SAT BIT(3)
#define TSL2522_STATUS2_MOD_ANA_SAT1   BIT(1)
#define TSL2522_STATUS2_MOD_ANA_SAT0   BIT(0)

#define TSL2522_REG_STATUS3                   0x9E
#define TSL2522_STATUS3_AINT_HYST_STATE_VALID BIT(7)
#define TSL2522_STATUS3_AINT_HYST_STATE_RD    BIT(6)
#define TSL2522_STATUS3_AINT_AIHT             BIT(5)
#define TSL2522_STATUS3_AINT_AILT             BIT(4)
#define TSL2522_STATUS3_VSYNC_LOST            BIT(3)
#define TSL2522_STATUS3_OSC_CALIB_SATURATION  BIT(1)
#define TSL2522_STATUS3_OSC_CALIB_FINISHED    BIT(0)

#define TSL2522_REG_STATUS4                      0x9F
#define TSL2522_STATUS4_MOD_SAMPLE_TRIGGER_ERROR BIT(3)
#define TSL2522_STATUS4_MOD_TRIGGER_ERROR        BIT(2)
#define TSL2522_STATUS4_SAI_ACTIVE               BIT(1)
#define TSL2522_STATUS4_INIT_BUSY                BIT(0)

#define TSL2522_REG_STATUS5                        0xA0
#define TSL2522_STATUS5_SINT_MEASUREMENT_SEQUENCER BIT(1)
#define TSL2522_STATUS5_SINT_VSYNC                 BIT(0)

#define TSL2522_REG_CFG2 0xA3

#define TSL2522_REG_TRIGGER_MODE      0xAE
#define TSL2522_TRIGGER_MODE_MASK     GENMASK(2, 0)
#define TSL2522_TRIGGER_MODE_OFF      0x00
#define TSL2522_TRIGGER_MODE_NORMAL   0x01
#define TSL2522_TRIGGER_MODE_LONG     0x02
#define TSL2522_TRIGGER_MODE_FAST     0x03
#define TSL2522_TRIGGER_MODE_FASTLONG 0x04
#define TSL2522_TRIGGER_MODE_VSYNC    0x05

#define TSL2522_REG_CONTROL              0xB1
#define TSL2522_CONTROL_SOFT_RESET       BIT(3)
#define TSL2522_CONTROL_FIFO_CLR         BIT(1)
#define TSL2522_CONTROL_CLEAR_SAI_ACTIVE BIT(0)

#define TSL2522_REG_MEAS_SEQR_STEP0_MOD_GAINX_L 0xD4
#define TSL2522_MEAS_SEQR_STEP0_MOD_GAIN1       GENMASK(7, 4)
#define TSL2522_MEAS_SEQR_STEP0_MOD_GAIN0       GENMASK(3, 0)

#define TSL2522_GAIN_MOD_HALF  0x00
#define TSL2522_GAIN_MOD_1X    0x01
#define TSL2522_GAIN_MOD_2X    0x02
#define TSL2522_GAIN_MOD_4X    0x03
#define TSL2522_GAIN_MOD_8X    0x04
#define TSL2522_GAIN_MOD_16X   0x05
#define TSL2522_GAIN_MOD_32X   0x06
#define TSL2522_GAIN_MOD_64X   0x07
#define TSL2522_GAIN_MOD_128X  0x08
#define TSL2522_GAIN_MOD_256X  0x09
#define TSL2522_GAIN_MOD_512X  0x0A
#define TSL2522_GAIN_MOD_1024X 0x0B
#define TSL2522_GAIN_MOD_2048X 0x0C
#define TSL2522_GAIN_MOD_4096X 0x0D

#define TSL2522_REG_MEAS_SEQR_STEP0_MOD_PHDX_SMUX_L 0xDC
#define TSL2522_MEAS_SEQR_STEP0_MOD_PHD3            GENMASK(7, 6)
#define TSL2522_MEAS_SEQR_STEP0_MOD_PHD2            GENMASK(5, 4)
#define TSL2522_MEAS_SEQR_STEP0_MOD_PHD1            GENMASK(3, 2)
#define TSL2522_MEAS_SEQR_STEP0_MOD_PHD0            GENMASK(1, 0)

#define TSL2522_REG_MEAS_SEQR_STEP0_MOD_PHDX_SMUX_H 0xDD
#define TSL2522_MEAS_SEQR_STEP0_MOD_PHD5            GENMASK(3, 2)
#define TSL2522_MEAS_SEQR_STEP0_MOD_PHD4            GENMASK(1, 0)

#define TSL2522_MOD_SEL_NO_CONN 0U
#define TSL2522_MOD_SEL_MOD_0   1U
#define TSL2522_MOD_SEL_MOD_1   2U

struct tsl2522_dts_config {
	struct i2c_dt_spec i2c;
};

struct tsl2522_data {
	uint8_t als_scale;
	uint8_t again_pho;
	uint8_t again_ir;
	uint32_t atime_ms;
	uint16_t sample_time_ms;
	uint16_t nr_samples;
	uint32_t photopic_channel;
	uint32_t ir_channel;
};

static uint32_t get_gain_value(uint8_t again)
{
	if (again >= TSL2522_GAIN_MOD_1X && again <= TSL2522_GAIN_MOD_4096X) {
		return 1U << (again - 1);
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
static uint32_t tsl2522_calc_lux(uint32_t pho, uint32_t ir, uint32_t atime_ms, uint32_t again)
{
	int64_t numerator;
	uint64_t denominator;

	if ((uint64_t)ir * 1000ULL < (uint64_t)pho * 1074ULL) {
		numerator = (int64_t)TSL2522_L_A * pho + (int64_t)TSL2522_L_B * ir;
	} else {
		numerator = (int64_t)TSL2522_H_A * pho + (int64_t)TSL2522_H_B * ir;
	}

	if (numerator <= 0) {
		return 0;
	}

	denominator = (uint64_t)TSL2522_SCALE * atime_ms * again;

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

static int tsl2522_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int rc = 0;
	const struct tsl2522_dts_config *cfg = dev->config;
	struct tsl2522_data *data = dev->data;
	uint8_t status = 0U;
	uint8_t status2_5[4];
	uint8_t als_status = 0U;
	uint8_t als_data[4];
	bool als_data_valid = false;
	bool measured_data_valid = false;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_AMBIENT_LIGHT &&
	    chan != SENSOR_CHAN_LIGHT && chan != SENSOR_CHAN_IR) {
		return -ENOTSUP;
	}

	rc = i2c_reg_read_byte_dt(&cfg->i2c, TSL2522_REG_STATUS, &status);
	if (rc < 0) {
		return rc;
	}
	LOG_INF("status (0x%02x): modulator int %d, ALS int %d, FIFO int %d, SYSTEM int %d", status,
		(bool)FIELD_GET(TSL2522_STATUS_MINT, status),
		(bool)FIELD_GET(TSL2522_STATUS_AINT, status),
		(bool)FIELD_GET(TSL2522_STATUS_FINT, status),
		(bool)FIELD_GET(TSL2522_STATUS_SINT, status));

	/* Clear status bits. */
	rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_STATUS, status);
	if (rc < 0) {
		return rc;
	}

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

		data->photopic_channel = (uint32_t)sys_get_le16(&als_data[0]);
		if (!FIELD_GET(TSL2522_ALS_STATUS_DATA0_SCALED, als_status)) {
			data->photopic_channel = data->photopic_channel << data->als_scale;
		}
		data->ir_channel = (uint32_t)sys_get_le16(&als_data[2]);
		if (!FIELD_GET(TSL2522_ALS_STATUS_DATA1_SCALED, als_status)) {
			data->ir_channel = data->ir_channel << data->als_scale;
		}

		uint8_t als_status2;

		rc = i2c_reg_read_byte_dt(&cfg->i2c, TSL2522_REG_ALS_STATUS2, &als_status2);
		if (rc < 0) {
			return rc;
		}

		data->again_pho = FIELD_GET(TSL2522_ALS_STATUS2_DATA0_GAIN, als_status2);
		data->again_ir = FIELD_GET(TSL2522_ALS_STATUS2_DATA1_GAIN, als_status2);

		LOG_INF("again_pho: %u, again_ir %u", get_gain_value(data->again_pho),
			get_gain_value(data->again_ir));
		/* Do the calculation here with the read-back information from the sequence step. */
		data->atime_ms = data->nr_samples * data->sample_time_ms;
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

static int tsl2522_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct tsl2522_data *data = dev->data;
	int rc = 0;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_LIGHT:
		val->val1 = tsl2522_calc_lux(data->photopic_channel, data->ir_channel,
					     data->atime_ms, get_gain_value(data->again_pho));
		break;
	case SENSOR_CHAN_LIGHT:
		val->val1 = tsl2522_calc_lux(data->photopic_channel, 0U, data->atime_ms,
					     get_gain_value(data->again_pho));
		break;
	case SENSOR_CHAN_IR:
		val->val1 = tsl2522_calc_lux(0U, data->ir_channel, data->atime_ms,
					     get_gain_value(data->again_ir));
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

	rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_WTIME, TSL2522_WTIME_DEFAULT);
	if (rc < 0) {
		return rc;
	}

	if (data->sample_time_ms > 0 && data->sample_time_ms <= TLS2522_MAX_SAMPLE_TIME_MS) {
		uint8_t sample_time[2];
		sys_put_le16((data->sample_time_ms * TLS2522_TICKS_PER_MS) - 1U, sample_time);
		rc = i2c_burst_write_dt(&cfg->i2c, TSL2522_REG_SAMPLE_TIME0, sample_time,
					sizeof(sample_time));
		if (rc < 0) {
			return rc;
		}
	} else {
		return -EINVAL;
	}

	if (data->nr_samples > 0 && data->nr_samples <= TLS2522_MAX_NR_SAMPLES) {
		uint8_t nr_samples[2];
		sys_put_le16(data->nr_samples - 1U, nr_samples);
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
#if 0
	rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_MEAS_MODE1,
				   TSL2522_MEAS_MODE1_ALS_MSB_POSITION_DEFAULT);
	if (rc < 0) {
		return rc;
	}
#endif
	rc = i2c_reg_write_byte_dt(
		&cfg->i2c, TSL2522_REG_MEAS_SEQR_STEP0_MOD_GAINX_L,
		FIELD_PREP(TSL2522_MEAS_SEQR_STEP0_MOD_GAIN1, TSL2522_GAIN_MOD_16X) |
			FIELD_PREP(TSL2522_MEAS_SEQR_STEP0_MOD_GAIN0, TSL2522_GAIN_MOD_16X));
	if (rc < 0) {
		return rc;
	}

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
	data->sample_time_ms = 1U;
	data->nr_samples = 63U;

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

#define DT_DRV_COMPAT ams_tsl2522

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

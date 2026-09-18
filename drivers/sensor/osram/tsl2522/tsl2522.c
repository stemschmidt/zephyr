/*
 * Copyright (c) 2026 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tsl2522, CONFIG_SENSOR_LOG_LEVEL);

#define TSL2522_MAX_INIT_RETRY 5U
#define TSL2522_DEVICE_ID      0x5C

#define TSL2522_REG_ENABLE  0x80
#define TSL2522_ENABLE_FDEN BIT(6)
#define TSL2522_ENABLE_AEN  BIT(1)
#define TSL2522_ENABLE_PON  BIT(0)

#define TSL2522_REG_MEAS_MODE1                        0x82
#define TSL2522_MEAS_MODE1_MOD_FIFO_FD_END_MARKER_WEN BIT(7)
#define AMBIENT25_MEAS_MODE1_MOD_FIFO_FD_CHECKSUM_WEN BIT(6)
#define AMBIENT25_MEAS_MODE1_MOD_FIFO_FD_GAIN_WEN     BIT(5)
#define AMBIENT25_MEAS_MODE1_ALS_MSB_POSITION_MASK    GENMASK(4, 0)
#define AMBIENT25_MEAS_MODE1_ALS_MSB_POSITION_DEFAULT 0x08

#define TSL2522_REG_WTIME     0x89
#define TSL2522_WTIME_DEFAULT 0x46

#define TSL2522_REG_DEVICE_ID 0x92

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

#define TSL2522_REG_FIFO_THR 0xFC

#define TSL2522_REG_FIFO_STATUS0 0xFD

#define TSL2522_REG_FIFO_STATUS1       0xFE
#define TSL2522_FIFO_STATUS1_OVERFLOW  BIT(7)
#define TSL2522_FIFO_STATUS1_UNDERFLOW BIT(6)
#define TSL2522_FIFO_STATUS1_LVL_10    GENMASK(1, 0)

#define TSL2522_REG_FIFO_DATA 0xFF

struct tsl2522_dts_config {
	struct i2c_dt_spec i2c;
};

struct tsl2522_data {
	uint16_t channel_0;
	uint16_t channel_1;
};

static int tsl2522_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int rc = 0;
	const struct tsl2522_dts_config *cfg = dev->config;
	struct tsl2522_data *data = dev->data;

	uint16_t entries = 0U;
	uint8_t fifo_status[2U];

	rc = i2c_burst_read_dt(&cfg->i2c, TSL2522_REG_FIFO_STATUS0, fifo_status,
			       sizeof(fifo_status));
	if (rc < 0) {
		return rc;
	}

	entries = (fifo_status[0] << 2) + FIELD_GET(TSL2522_FIFO_STATUS1_LVL_10, fifo_status[1]);
	bool fifo_overflow = FIELD_GET(TSL2522_FIFO_STATUS1_OVERFLOW, fifo_status[1]);
	bool fifo_underflow = FIELD_GET(TSL2522_FIFO_STATUS1_UNDERFLOW, fifo_status[1]);

	if (fifo_overflow || fifo_underflow) {
		LOG_ERR("FIFO error:%s%s", fifo_overflow ? " overflow" : "",
			fifo_underflow ? " underflow" : "");
	}
	if (entries > 5) {
		uint8_t fifo_data[6] = {0};
		rc = i2c_burst_read_dt(&cfg->i2c, TSL2522_REG_FIFO_DATA, fifo_data,
				       sizeof(fifo_data));
		if (rc < 0) {
			return rc;
		}
		data->channel_0 = ((uint16_t)fifo_data[1] << 8) | fifo_data[0];
		data->channel_1 = ((uint16_t)fifo_data[3] << 8) | fifo_data[2];

		rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_CONTROL,
					   TSL2522_CONTROL_FIFO_CLR);
	}

	return rc;
}

static int tsl2522_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct tsl2522_data *data = dev->data;

	val->val1 = 0;
	val->val2 = (int32_t)data->channel_0;

	return 0;
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

	rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_WTIME, TSL2522_WTIME_DEFAULT);
	if (rc < 0) {
		return rc;
	}

	rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_TRIGGER_MODE,
				   TSL2522_TRIGGER_MODE_NORMAL);
	if (rc < 0) {
		return rc;
	}

	rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_MEAS_MODE1,
				   AMBIENT25_MEAS_MODE1_ALS_MSB_POSITION_DEFAULT);
	if (rc < 0) {
		return rc;
	}

	rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_FIFO_THR, 5 >> 1);
	if (rc < 0) {
		return rc;
	}

	rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_CFG2, 5 & 0x01);
	if (rc < 0) {
		return rc;
	}

	rc = i2c_reg_write_byte_dt(
		&cfg->i2c, TSL2522_REG_MEAS_SEQR_STEP0_MOD_GAINX_L,
		FIELD_PREP(TSL2522_MEAS_SEQR_STEP0_MOD_GAIN1, TSL2522_GAIN_MOD_4X) |
			FIELD_PREP(TSL2522_MEAS_SEQR_STEP0_MOD_GAIN0, TSL2522_GAIN_MOD_32X));
	if (rc < 0) {
		return rc;
	}

	rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_ENABLE,
				   TSL2522_ENABLE_PON | TSL2522_ENABLE_AEN);
	if (rc < 0) {
		return rc;
	}

	rc = i2c_reg_write_byte_dt(&cfg->i2c, TSL2522_REG_CONTROL, TSL2522_CONTROL_FIFO_CLR);

	return rc;
}

static int tsl2522_init(const struct device *dev)
{
	const struct tsl2522_dts_config *cfg = dev->config;
	int rc = -EAGAIN;
	uint8_t devid = 0;

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

#define DT_DRV_COMPAT osram_tsl2522

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

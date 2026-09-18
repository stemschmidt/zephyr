/*
 * Copyright (c) 2026 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_DRIVERS_SENSOR_TSL2522_TSL2522_H_
#define ZEPHYR_DRIVERS_SENSOR_TSL2522_TSL2522_H_

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/sensor/tsl2522.h>

#define NUMBER_OF_SAMPLES_MIN 1U
#define NUMBER_OF_SAMPLES_MAX 128U
#define US_IN_MS              1000U

#define TSL2522_MAX_INIT_RETRY    5U
#define TSL2522_DEVICE_ID         0x5C
#define TSL2522_RETRY_ACCESS_US   100U
#define TSL2522_SOFTRESET_WAIT_US 500U
#define TSL2522_SCALE             100000000LL
/* Low segment */
#define TSL2522_L_A               1504118LL
#define TSL2522_L_B               (-381917LL)
/* High segment */
#define TSL2522_H_A               1488034LL
#define TSL2522_H_B               (-276429LL)

#define TSL2522_REG_ENABLE  0x80
#define TSL2522_ENABLE_FDEN BIT(6)
#define TSL2522_ENABLE_AEN  BIT(1)
#define TSL2522_ENABLE_PON  BIT(0)

#define TSL2522_REG_MEAS_MODE                        0x81
#define TSL2522_MEAS_MODE_STOP_AFTER_NTH_ITERATION   BIT(7)
#define TSL2522_MEAS_MODE_EN_AGC_ASAT_DBL_STEP_DOWN  BIT(6)
#define TSL2522_MEAS_MODE_MEAS_SEQ_SINGLE_SHOT_MODE  BIT(5)
#define TSL2522_MEAS_MODE_MOD_FIFO_ALS_STAT_WRITE_EN BIT(4)
#define TSL2522_MEAS_MODE_ALS_SCALE                  GENMASK(3, 0)

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

#define SAMPLE_TIME_STEP_US 100U

static inline uint16_t
tsl2522_convert_sample_time_enum_to_us(enum us_per_sample_tsl2522 time_per_sample_enum)
{
	return (time_per_sample_enum + 1U) * SAMPLE_TIME_STEP_US;
}

static inline enum us_per_sample_tsl2522
tsl2522_convert_sample_time_us_to_enum(uint16_t time_per_sample_us)
{
	return (time_per_sample_us / SAMPLE_TIME_STEP_US) - 1U;
}

static inline uint16_t tsl2522_convert_us_to_counts(uint16_t us)
{
	/* Time step is 1.388889μs, so 72 time step (count) per 100 µs. */
	return (uint16_t)(((uint32_t)us * 72U / 100U) - 1U);
}

static inline uint32_t tsl2522_convert_gain_enum_to_value(enum sensor_gain_tsl2522 again)
{
	if (again >= TSL2522_GAIN_MOD_1X && again <= TSL2522_GAIN_MOD_4096X) {
		return 1000U * (1U << (again - 1));
	} else if (again == TSL2522_GAIN_MOD_HALF) {
		return 500U;
	}
	return 1U;
}

static inline enum sensor_gain_tsl2522 convert_gain_value_to_enum(uint32_t gain)
{
	switch (gain) {
	case 1000U:
		return TSL2522_GAIN_MOD_1X;
	case 2000U:
		return TSL2522_GAIN_MOD_2X;
	case 4000U:
		return TSL2522_GAIN_MOD_4X;
	case 8000U:
		return TSL2522_GAIN_MOD_8X;
	case 16000U:
		return TSL2522_GAIN_MOD_16X;
	case 32000U:
		return TSL2522_GAIN_MOD_32X;
	case 64000U:
		return TSL2522_GAIN_MOD_64X;
	case 128000U:
		return TSL2522_GAIN_MOD_128X;
	case 256000U:
		return TSL2522_GAIN_MOD_256X;
	case 512000U:
		return TSL2522_GAIN_MOD_512X;
	case 1024000U:
		return TSL2522_GAIN_MOD_1024X;
	case 2048000U:
		return TSL2522_GAIN_MOD_2048X;
	case 4096000U:
		return TSL2522_GAIN_MOD_4096X;
	case 500U:
	default:
		return TSL2522_GAIN_MOD_HALF;
	}
}

struct tsl2522_dts_config {
	struct i2c_dt_spec i2c;
	uint32_t glass_attenuation;
	uint32_t glass_ir_attenuation;
	enum sensor_gain_tsl2522 gain_enum;
};

struct tsl2522_measurement {
	uint32_t photopic_channel;
	uint32_t ir_channel;
	uint32_t atime_us;
	uint32_t gain;
	bool saturation;
};

struct tsl2522_data {
	struct k_mutex mutex;
	struct tsl2522_measurement measurement;
	uint16_t time_per_sample_us;
	uint16_t number_of_samples;
	uint32_t gain; /* gain value * 1000U for calculations. */
	uint8_t als_scale;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_TSL2522_TSL2522_H_ */

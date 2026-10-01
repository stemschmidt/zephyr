/*
 * Copyright (c) 2026 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Extended public API for AMS's TSL2522 ambient light sensor
 *
 * This exposes attributes for the TSL2522 which can be used for
 * setting the on-chip gain and integration time parameters.
 *
 * For SENSOR_CHAN_IR the driver returns a normalized counts per ms at 1x gain, not lux.
 *
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_TSL2522_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_TSL2522_H_

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TSL2522_NUMBER_OF_SAMPLES_MIN      1U
#define TSL2522_NUMBER_OF_SAMPLES_MAX      2047U
#define TSL2522_MEASUREMENT_TIME_STEPS_MIN 1U
#define TSL2522_MEASUREMENT_TIME_STEPS_MAX 2047U

enum sensor_attribute_tsl2522 {
	/* ALS measurement time step. val1 contains the time in steps of 1.388889μs modulator clock.
	 */
	SENSOR_ATTR_MEASUREMENT_TIME_STEPS = SENSOR_ATTR_PRIV_START + 1,
	/* Number of samples in a conversion. val1 contains a value between
	 * TSL2522_NUMBER_OF_SAMPLES_MIN and TSL2522_NUMBER_OF_SAMPLES_MAX
	 */
	SENSOR_ATTR_NUMBER_OF_SAMPLES,
};

enum sensor_gain_tsl2522 {
	TSL2522_GAIN_MOD_HALF = 0U, /**< Gain x 0.5 */
	TSL2522_GAIN_MOD_1X,        /**< Gain x 1.0 */
	TSL2522_GAIN_MOD_2X,        /**< Gain x 2.0 */
	TSL2522_GAIN_MOD_4X,        /**< Gain x 4.0 */
	TSL2522_GAIN_MOD_8X,        /**< Gain x 8.0 */
	TSL2522_GAIN_MOD_16X,       /**< Gain x 16.0 */
	TSL2522_GAIN_MOD_32X,       /**< Gain x 32.0 */
	TSL2522_GAIN_MOD_64X,       /**< Gain x 64.0 */
	TSL2522_GAIN_MOD_128X,      /**< Gain x 128.0 */
	TSL2522_GAIN_MOD_256X,      /**< Gain x 256.0 */
	TSL2522_GAIN_MOD_512X,      /**< Gain x 512.0 */
	TSL2522_GAIN_MOD_1024X,     /**< Gain x 1024.0 */
	TSL2522_GAIN_MOD_2048X,     /**< Gain x 2048.0 */
	TSL2522_GAIN_MOD_4096X      /**< Gain x 4096.0 */
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_TSL2522_H_ */

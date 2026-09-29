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

enum sensor_attribute_tsl2522 {
	/* Time per sample (in us) */
	SENSOR_ATTR_TIME_PER_SAMPLE_US = SENSOR_ATTR_PRIV_START + 1,
	/* Number of samples in a conversion */
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

enum us_per_sample_tsl2522 {
	TSL2522_100US_PER_SAMPLE, /**< 100µs per Sample */
	TSL2522_200US_PER_SAMPLE, /**< 200µs per Sample */
	TSL2522_300US_PER_SAMPLE, /**< 300µs per Sample */
	TSL2522_400US_PER_SAMPLE, /**< 400µs per Sample */
	TSL2522_500US_PER_SAMPLE, /**< 500µs per Sample */
	TSL2522_600US_PER_SAMPLE, /**< 600µs per Sample */
	TSL2522_700US_PER_SAMPLE, /**< 700µs per Sample */
	TSL2522_800US_PER_SAMPLE, /**< 800µs per Sample */
	TSL2522_900US_PER_SAMPLE, /**< 900µs per Sample */
	TSL2522_1000US_PER_SAMPLE /**< 1000µs per Sample */
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_TSL2522_H_ */

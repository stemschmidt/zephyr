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
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_TSL2522_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_TSL2522_H_

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

enum sensor_gain_tsl2522 {
	TSL2522_GAIN_MOD_HALF = 0U,
	TSL2522_GAIN_MOD_1X,
	TSL2522_GAIN_MOD_2X,
	TSL2522_GAIN_MOD_4X,
	TSL2522_GAIN_MOD_8X,
	TSL2522_GAIN_MOD_16X,
	TSL2522_GAIN_MOD_32X,
	TSL2522_GAIN_MOD_64X,
	TSL2522_GAIN_MOD_128X,
	TSL2522_GAIN_MOD_256X,
	TSL2522_GAIN_MOD_512X,
	TSL2522_GAIN_MOD_1024X,
	TSL2522_GAIN_MOD_2048X,
	TSL2522_GAIN_MOD_4096X
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_TSL2522_H_ */

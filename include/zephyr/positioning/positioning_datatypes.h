/*
 * Copyright (c) 2026 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#ifndef ZEPHYR_INCLUDE_POSITIONING_POSITIONING_DATATYPES_H_
#define ZEPHYR_INCLUDE_POSITIONING_POSITIONING_DATATYPES_H_

#include <zephyr/positioning/positioning_datatypes.h>

enum positioning_events {
	PS_END_POSITION_REACHED,
	PS_POS_REACHED,
	PS_BLOCKED
};

struct position {
	int32_t val1;
	int32_t val2;
};

typedef void *positioning_handle_t;

typedef void (*positioning_calibrate_ready_cb_t)(int err, void *user_data);

typedef void (*positioning_position_update_cb_t)(const struct position current, void *user_data);

typedef void (*positioning_event_cb_t)(const enum positioning_events event, void *user_data);

#endif /* ZEPHYR_INCLUDE_POSITIONING_POSITIONING_DATATYPES_H_ */

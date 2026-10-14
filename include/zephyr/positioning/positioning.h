/*
 * Copyright (c) 2026 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_POSITIONING_POSITIONING_H_
#define ZEPHYR_INCLUDE_POSITIONING_POSITIONING_H_

#include <zephyr/positioning/positioning_datatypes.h>

#ifdef __cplusplus
extern "C" {
#endif

int positioning_system_calibrate(positioning_handle_t system, positioning_calibrate_ready_cb_t cb);

int positioning_system_move_to(positioning_handle_t system, const struct position *target_position,
			       const struct speed *travel_speed);

int positioning_system_stop(positioning_handle_t system);

int positioning_system_get_position(positioning_handle_t system, struct position *current);

int positioning_system_get_min_max_position(positioning_handle_t system,
					    struct position *min_position,
					    struct position *max_position);

int positioning_system_register_position_callback(positioning_handle_t system,
						  positioning_position_update_cb_t cb,
						  void *user_data);

int positioning_system_register_event_callback(positioning_handle_t system,
					       positioning_event_cb_t cb, void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_POSITIONING_POSITIONING_H_ */

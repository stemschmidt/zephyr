/*
 * Copyright (c) 2026 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_positioning

#include <stdlib.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#if 0
int positioning_system_calibrate(positioning_handle_t system, positioning_calibrate_ready_cb_t cb)
{
	return 0;
}

int positioning_system_move_to(positioning_handle_t system, const struct position *target_position,
			       const struct speed *travel_speed)
{
	return 0;
}

int positioning_system_stop(positioning_handle_t system)
{
	return 0;
}

int positioning_system_get_position(positioning_handle_t system, struct position *current)
{
	return 0;
}

int positioning_system_get_min_max_position(positioning_handle_t system,
					    struct position *min_position,
					    struct position *max_position)
{
	return 0;
}

int positioning_system_register_position_callback(positioning_handle_t system,
						  positioning_position_update_cb_t cb,
						  void *user_data)
{
	return 0;
}

int positioning_system_register_event_callback(positioning_handle_t system,
					       positioning_event_cb_t cb, void *user_data)
{
	return 0;
}

/* clang-format off */

#define POSITIONING_DEFINE(inst)	\
	static struct positioning_data data_##inst;	\
	\
	static const struct positioning_config config_##inst = {	\
	};	\
	\
	DEVICE_DT_INST_DEFINE(inst, positioning_init, NULL, &data_##inst,	\
						  &config_##inst, APPLICATION,	\
						  CONFIG_POSITIONING_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(POSITIONING_DEFINE)

/* clang-format on */
#endif

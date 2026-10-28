/*
 * SPDX-FileCopyrightText:  Copyright (c) 2026 The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Main header file for peripheral interconnect driver API.
 * @ingroup peripheral_interconnect_interface
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PERIPHERAL_INTERCONNECT_H_
#define ZEPHYR_INCLUDE_DRIVERS_PERIPHERAL_INTERCONNECT_H_

/**
 * @brief Interfaces for peripheral interconnect controllers.
 * @defgroup peripheral_interconnect_interface Peripheral Interconnect
 * @version 0.1.0
 * @ingroup io_interfaces
 * @{
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque handle identifying a peripheral interconnect lane */
typedef void *peripheral_interconnect_handle_t;

typedef void *peripheral_interconnect_event_t;

typedef void *peripheral_interconnect_task_t;
/**
 * @def_driverbackendgroup{peripheral_interconnect,peripheral_interconnect_interface}
 * @{
 */

/**
 * @brief Callback API to configure a peripheral interconnect lane.
 */
typedef int (*peripheral_interconnect_acquire_lane_t)(const struct device *dev,
						      peripheral_interconnect_event_t *publisher,
						      peripheral_interconnect_task_t **subscriber,
						      uint32_t num_subscribers,
						      peripheral_interconnect_handle_t *handle);

/**
 * @brief Callback API to obtain PWM cycles per second (frequency).
 */
typedef int (*peripheral_interconnect_release_lane_t)(const struct device *dev,
						      peripheral_interconnect_handle_t handle);

/**
 * @}
 */

/**
 * @driver_ops{peripheral_interconnect}
 */
__subsystem struct peripheral_interconnect_driver_api {
	/**
	 * @driver_ops_mandatory @copybrief peripheral_interconnect_acquire_lane
	 */
	peripheral_interconnect_acquire_lane_t aquire_lane;
	/**
	 * @driver_ops_mandatory @copybrief peripheral_interconnect_release_lane
	 */
	peripheral_interconnect_release_lane_t release_lane;
};
/**
 * @}
 */

static inline int peripheral_interconnect_get_event()
{
}
/**
 * @brief Configure a peripheral interconnect lane.
 *
 * @param dev Pointer to the device structure for the peripheral interconnect driver
 * instance.
 * @param publisher Pointer to the device structure of the publishing driver instance.
 * @param subscriber Pointer to an arry of pointers to the device structure of the
 * subscribing driver instances.
 * @param num_subscribers Number of subscribing driver instances
 * @param handle Pointer to an opaque handle. Can be used to release the lane. If NULL, one
 * event from publisher will be handled and the lane will be freed.
 *
 * @return 0 on success, otherwise error
 */
static inline int peripheral_interconnect_acquire_lane(const struct device *dev,
						       peripheral_interconnect_event_t *publisher,
						       peripheral_interconnect_task_t **subscriber,
						       uint32_t num_subscribers,
						       peripheral_interconnect_handle_t *handle)
{
	const struct peripheral_interconnect_driver_api *api =
		DEVICE_API_GET(peripheral_interconnect, dev);

	if (api->aquire_lane == NULL) {
		return -ENOSYS;
	}

	return api->aquire_lane(dev, publisher, subscriber, num_subscribers, handle);
}

/**
 * @brief Release a peripheral interconnect lane.
 *
 * @param dev Pointer to the device structure for the peripheral interconnect driver instance.
 * @param handle Opaque handle identifying the lane that shall be released.
 *
 * @return 0 on success, otherwise error
 */
static inline int peripheral_interconnect_release_lane(const struct device *dev,
						       peripheral_interconnect_handle_t handle)
{
	const struct peripheral_interconnect_driver_api *api =
		DEVICE_API_GET(peripheral_interconnect, dev);

	if (api->release_lane == NULL) {
		return -ENOSYS;
	}

	return api->release_lane(dev, handle);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_PERIPHERAL_INTERCONNECT_H_ */

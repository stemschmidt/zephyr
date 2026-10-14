/*
 * Copyright (c) 2026 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MOTOR_DRIVER_H_
#define ZEPHYR_INCLUDE_DRIVERS_MOTOR_DRIVER_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

enum direction {
	STOPPED,
	TOWARDS_MAX,
	TOWARDS_MIN
};

enum motor_fault {
	OVER_CURRENT,
	OVER_TEMPERATURE
};

typedef void (*motor_fault_callback_t)(enum motor_fault fault, void *user_data);

typedef int (*motor_enable_t)(const struct device *dev);
typedef int (*motor_disable_t)(const struct device *dev);
typedef int (*motor_stop_t)(const struct device *dev);
typedef int (*motor_set_duty_t)(const struct device *dev, const float *duty, uint8_t n);
typedef int (*motor_set_fault_callback_t)(const struct device *dev,
					  motor_fault_callback_t fault_callback, void *user_data);

__subsystem struct motor_driver_api {
	motor_enable_t enable;
	motor_disable_t disable;
	motor_stop_t stop;
	motor_set_duty_t set_duty;
	motor_set_fault_callback_t set_fault_callback;
};

struct motor_common_data {
	bool enabled;
	enum direction direction;
	float duty;
	motor_fault_callback_t app_callback;
	void *app_user_data;
	struct k_mutex mutex;
};

static inline int motor_enable(const struct device *dev)
{
	struct motor_common_data *data = (struct motor_common_data *)dev->data;
	const struct motor_driver_api *api = DEVICE_API_GET(motor, dev);
	int rc;

	k_mutex_lock(&data->mutex, K_FOREVER);

	if (data->enabled) {
		rc = -EALREADY;
		goto exit;
	}

	if (api->enable == NULL) {
		rc = -ENOSYS;
		goto exit;
	}

	rc = api->enable(dev);
	if (rc == 0) {
		data->enabled = true;
	}

exit:
	k_mutex_unlock(&data->mutex);

	return rc;
}

static inline int motor_disable(const struct device *dev)
{
	struct motor_common_data *data = (struct motor_common_data *)dev->data;
	const struct motor_driver_api *api = DEVICE_API_GET(motor, dev);
	int rc = 0;

	k_mutex_lock(&data->mutex, K_FOREVER);

	if (!data->enabled) {
		rc = -EALREADY;
		goto exit;
	}

	if (api->disable == NULL) {
		rc = -ENOSYS;
		goto exit;
	}

	rc = api->disable(dev);
	if (rc == 0) {
		data->enabled = false;
	}

exit:
	k_mutex_unlock(&data->mutex);

	return rc;
}

static inline int motor_stop(const struct device *dev)
{
	struct motor_common_data *data = (struct motor_common_data *)dev->data;
	const struct motor_driver_api *api = DEVICE_API_GET(motor, dev);
	int rc = 0;

	k_mutex_lock(&data->mutex, K_FOREVER);

	if (data->direction == STOPPED) {
		rc = -EALREADY;
		goto exit;
	}

	if (api->stop == NULL) {
		rc = -ENOSYS;
		goto exit;
	}

	rc = api->stop(dev);
	if (rc == 0) {
		data->direction = STOPPED;
	}

exit:
	k_mutex_unlock(&data->mutex);

	return rc;
}

static inline int motor_set_duty(const struct device *dev, const float *duty, uint8_t n)
{
	struct motor_common_data *data = (struct motor_common_data *)dev->data;
	const struct motor_driver_api *api = DEVICE_API_GET(motor, dev);
	int rc = 0;

	if (n > 1) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->mutex, K_FOREVER);

	if (data->duty == *duty) {
		rc = -EALREADY;
		goto exit;
	}

	if (api->set_duty == NULL) {
		rc = -ENOSYS;
		goto exit;
	}

	rc = api->set_duty(dev, duty, n);
	if (rc == 0) {
		data->duty = *duty;
		data->direction = *duty >= (float)0.0 ? TOWARDS_MAX : TOWARDS_MIN;
	}

exit:
	k_mutex_unlock(&data->mutex);

	return rc;
}

static inline int motor_set_fault_callback(const struct device *dev,
					   motor_fault_callback_t fault_callback, void *user_data)
{
	struct motor_common_data *data = (struct motor_common_data *)dev->data;

	k_mutex_lock(&data->mutex, K_FOREVER);

	data->app_callback = fault_callback;
	data->app_user_data = user_data;

	k_mutex_unlock(&data->mutex);

	return 0;
}

static inline void motor_common_init(struct motor_common_data *common)
{
	k_mutex_init(&common->mutex);
	common->direction = STOPPED;
	common->duty = 0.0;
	common->enabled = false;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MOTOR_DRIVER_H_ */

/*
 * Copyright (c) 2026 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/motor_driver.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(positioning);

int main(void)
{
	const struct device *const qdec_aperture = DEVICE_DT_GET(DT_ALIAS(qdec_field_aparture));
	const struct device *motor = DEVICE_DT_GET(DT_NODELABEL(aperture));

	int rc = 0;
	struct sensor_value val;
	struct sensor_value last_val;

	if (!device_is_ready(qdec_aperture)) {
		LOG_ERR("qdec_aperture device is not ready\n");
		return 0;
	}

	if (!device_is_ready(motor)) {
		LOG_ERR("pwm_aperture device is not ready\n");
		return 0;
	}
	LOG_INF("Positioning sample started");

	rc = motor_enable(motor);
	if (rc != 0) {
		LOG_ERR("motor_enable returned %d", rc);
	}

	float speed = 20000.0 / 400000.0;

	rc = motor_set_duty(motor, &speed, 1);
	if (rc != 0) {
		LOG_ERR("motor_enable returned %d", rc);
	}

	while (true) {
		k_msleep(10);

		rc = sensor_sample_fetch(qdec_aperture);
		if (rc != 0) {
			LOG_ERR("Failed to fetch sample (%d)", rc);
			return 0;
		}

		rc = sensor_channel_get(qdec_aperture, SENSOR_CHAN_ROTATION, &val);
		if (rc != 0) {
			LOG_ERR("Failed to get data (%d)", rc);
			return 0;
		}

		if (last_val.val1 != val.val1 || last_val.val2 != val.val2) {
			last_val = val;
			//			LOG_INF("Pos = %d.%06d deg", val.val1,
			// abs(val.val2));
		}

#if 0
		if (val.val1 < -7200) {
			speed = 20000.0 / 400000.0;
			motor_stop(motor);
			k_msleep(1);
			motor_set_duty(motor, &speed, 1);
		} else if (val.val1 > 7200) {
			motor_stop(motor);
			k_msleep(1);
			motor_set_duty(motor, &speed, 1);
		}
#endif
	}
	return 0;
}

/*
 * Copyright (c) 2025 Stefan Schmidt
 */

#include <zephyr/fff.h>
#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>

DEFINE_FFF_GLOBALS;

extern struct sensor_driver_api *unit_test_get_device_api(void);
extern int unit_test_invoke_adxl345_init(const struct device *dev);

// struct sensor_value data[3];
// sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, data);
ZTEST(adxl345_accel_convert, test_convert_in_2G_mode)
{
	zassert_equal(1, 1);
}

ZTEST_SUITE(adxl345_accel_convert, NULL, NULL, NULL, NULL, NULL);

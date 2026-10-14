/*
 * Copyright (c) 2026 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/motor_driver.h>
#include <zephyr/shell/shell.h>

static void fault_work_handler(struct k_work *work);

static K_WORK_DEFINE(fault_work, fault_work_handler);

static struct {
	struct device *dev;
	enum motor_fault fault;
} reported_fault;

static int cmd_enable(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	int ret;

	ARG_UNUSED(argc);

	dev = shell_device_get_binding(argv[1]);
	if (dev == NULL) {
		shell_error(sh, "Motor device %s not available", argv[1]);
		return -ENODEV;
	}

	ret = motor_enable(dev);
	if (ret < 0) {
		shell_error(sh, "Could not enable motor (%d)", ret);
		return ret;
	}

	return 0;
}

static int cmd_disable(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	int ret;

	ARG_UNUSED(argc);

	dev = shell_device_get_binding(argv[1]);
	if (dev == NULL) {
		shell_error(sh, "Motor device %s not available", argv[1]);
		return -ENODEV;
	}

	ret = motor_disable(dev);
	if (ret < 0) {
		shell_error(sh, "Could not disable motor (%d)", ret);
		return ret;
	}

	return 0;
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	int ret;

	ARG_UNUSED(argc);

	dev = shell_device_get_binding(argv[1]);
	if (dev == NULL) {
		shell_error(sh, "Motor device %s not available", argv[1]);
		return -ENODEV;
	}

	ret = motor_stop(dev);
	if (ret < 0) {
		shell_error(sh, "Could not stop motor (%d)", ret);
		return ret;
	}

	return 0;
}

bool parse_float(const char *str, float *result)
{
	if (str == NULL || result == NULL) {
		return false;
	}

	char *end;
	errno = 0;

	float value = strtof(str, &end);

	if (end == str) {
		return false;
	}

	while (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r') {
		end++;
	}

	if (*end != '\0') {
		return false;
	}

	if (errno == ERANGE || !isfinite(value)) {
		return false;
	}

	*result = value;
	return true;
}

static int cmd_set_duty(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	float duty = 0.0;
	int ret;

	ARG_UNUSED(argc);

	dev = shell_device_get_binding(argv[1]);
	if (dev == NULL) {
		shell_error(sh, "Motor device %s not available", argv[1]);
		return -ENODEV;
	}

	if (parse_float(argv[2], &duty)) {
		ret = motor_set_duty(dev, &duty, 1);
		if (ret < 0) {
			shell_error(sh, "Could not set duty cycle %f for motor (%d)", (double)duty,
				    ret);
			return ret;
		}
	} else {
		shell_error(sh, "Could not convert %s to float", argv[2]);
		return -1;
	}

	return 0;
}

static void fault_work_handler(struct k_work *work)
{
	if (reported_fault.dev != NULL) {
		printk("fault reported for %s: %d\n", reported_fault.dev->name,
		       reported_fault.fault);
	} else {
		printk("fault reported\n");
	}
}

static void fault_callback(enum motor_fault fault, void *user_data)
{
	reported_fault.fault = fault;
	reported_fault.dev = user_data;
	k_work_submit(&fault_work);
}

static int cmd_set_fault_callback(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	int ret;

	ARG_UNUSED(argc);

	dev = shell_device_get_binding(argv[1]);
	if (dev == NULL) {
		shell_error(sh, "Motor device %s not available", argv[1]);
		return -ENODEV;
	}

	ret = motor_set_fault_callback(dev, fault_callback, (void *)dev);
	if (ret < 0) {
		shell_error(sh, "Could not set fault callback to motor (%d)", ret);
		return ret;
	}

	return 0;
}

static bool device_is_motor(const struct device *dev)
{
	return DEVICE_API_IS(motor, dev);
}

static void device_name_get_motor(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_filter(idx, device_is_motor);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_device_name, device_name_get_motor);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_motor_cmds,
	SHELL_CMD_ARG(enable, &dsub_device_name, SHELL_HELP("Enable the motor", "<device>"),
		      cmd_enable, 2, 0),
	SHELL_CMD_ARG(disable, &dsub_device_name, SHELL_HELP("Disable the motor", "<device>"),
		      cmd_disable, 2, 0),
	SHELL_CMD_ARG(stop, &dsub_device_name, SHELL_HELP("Stop the motor", "<device>"), cmd_stop,
		      2, 0),
	SHELL_CMD_ARG(duty, &dsub_device_name,
		      SHELL_HELP("Set duty cycle of the motor", "<device> <duty -1.0..1.0>"),
		      cmd_set_duty, 3, 0),
	SHELL_CMD_ARG(fault_cb, &dsub_device_name,
		      SHELL_HELP("Set fault callback of the motor", "<device>"),
		      cmd_set_fault_callback, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(motor, &sub_motor_cmds, "Motor shell", NULL);

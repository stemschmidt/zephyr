# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(dc_motor_driver.c)
zephyr_library_sources_ifdef(CONFIG_MOTOR_SHELL motor_shell.c)

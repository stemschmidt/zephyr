# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_include_directories(include)

zephyr_library_sources(
  positioning.c
)

zephyr_sources_ifdef(CONFIG_POSITIONING_SHELL positioning_shell.c)

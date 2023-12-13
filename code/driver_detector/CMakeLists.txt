#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
  # src/drivers_nrf/delay/nrf_delay.c
  src/nrfx/drivers/src/nrfx_twi.c
  src/drivers_nrf/twi_master/nrf_drv_twi.c
  src/libraries/util/app_error.c
  src/libraries/util/app_util_platform.c
  src/libraries/util/nrf_assert.c
)
# target_link_libraries(app PRIVATE /build/zephyr/libzephyr.a)

# NORDIC SDK APP END
zephyr_library_include_directories(.)

  # src/drivers_nrf/delay/nrf_delay.c
  # src/nrfx/hal/nrf_twi.h
  # src/drivers_nrf/twi_master/nrf_drv_twi.h
  # src/libraries/util/app_error.h
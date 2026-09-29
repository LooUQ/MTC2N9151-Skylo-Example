#
# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

# Local credentials (TagoIO token/hash), kept out of git. See secrets.conf.example.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/secrets.conf)
  list(APPEND EXTRA_CONF_FILE ${CMAKE_CURRENT_SOURCE_DIR}/secrets.conf)
else()
  message(WARNING "secrets.conf not found: TagoIO token/hash will be empty. "
                  "Copy secrets.conf.example to secrets.conf and fill it in.")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nidd)

# NORDIC SDK APP START
target_sources(app PRIVATE src/main.c)
# NORDIC SDK APP END

zephyr_include_directories(src)

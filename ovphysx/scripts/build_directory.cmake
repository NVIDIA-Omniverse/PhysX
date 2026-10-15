# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if(NOT DEFINED BUILD_DIR)
    if(DEFINED ENV{OVPHYSX_BUILD_DIR} AND NOT "$ENV{OVPHYSX_BUILD_DIR}" STREQUAL "")
        set(BUILD_DIR "$ENV{OVPHYSX_BUILD_DIR}")
    else()
        set(BUILD_DIR "_build")
    endif()
endif()
get_filename_component(BUILD_PATH "${BUILD_DIR}" ABSOLUTE BASE_DIR "${PROJECT_ROOT}")

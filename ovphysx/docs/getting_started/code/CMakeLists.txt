# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
cmake_minimum_required(VERSION 3.16)
project(falling_box C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(ovphysx REQUIRED)

add_executable(falling_box falling_box.c)
target_link_libraries(falling_box PRIVATE ovphysx::ovphysx ovphysx::ovstage)

# On Windows, copy the ovphysx libraries next to the program and make
# ovstage findable.
if(WIN32)
    ovphysx_copy_runtime_dlls(falling_box)
    ovphysx_set_ovstage_runtime_env(falling_box)
endif()
# [example-end]

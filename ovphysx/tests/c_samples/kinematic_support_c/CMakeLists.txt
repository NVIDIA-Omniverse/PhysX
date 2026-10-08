# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(kinematic_support_c C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(ovphysx REQUIRED)

add_executable(kinematic_support_c main.c)
target_link_libraries(kinematic_support_c PRIVATE ovphysx::ovphysx ovphysx::ovstage)
target_include_directories(kinematic_support_c PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../common")
get_filename_component(OVPHYSX_TEST_DATA_DIR "${CMAKE_CURRENT_LIST_DIR}/../../data" ABSOLUTE)
target_compile_definitions(kinematic_support_c PRIVATE
    OVPHYSX_TEST_DATA="${OVPHYSX_TEST_DATA_DIR}"
)
if(MSVC)
    target_compile_options(kinematic_support_c PRIVATE /std:c11 /TC)
    ovphysx_copy_runtime_dlls(kinematic_support_c)
elseif(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(kinematic_support_c PRIVATE -Werror=implicit-function-declaration -pedantic)
endif()

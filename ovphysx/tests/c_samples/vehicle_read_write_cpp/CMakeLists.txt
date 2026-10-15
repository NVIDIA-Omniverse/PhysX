# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(VehicleReadWriteCpp CXX)
find_package(ovphysx REQUIRED)
add_executable(vehicle_read_write_cpp main.cpp)
target_compile_features(vehicle_read_write_cpp PRIVATE cxx_std_17)
target_link_libraries(vehicle_read_write_cpp PRIVATE ovphysx::ovphysx ovphysx::ovstage)
target_include_directories(vehicle_read_write_cpp PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../common")
get_filename_component(OVPHYSX_TEST_DATA_DIR "${CMAKE_CURRENT_LIST_DIR}/../../data" ABSOLUTE)
target_compile_definitions(vehicle_read_write_cpp PRIVATE OVPHYSX_TEST_DATA="${OVPHYSX_TEST_DATA_DIR}")
if(WIN32)
    ovphysx_copy_runtime_dlls(vehicle_read_write_cpp)
    ovphysx_set_ovstage_runtime_env(vehicle_read_write_cpp)
endif()

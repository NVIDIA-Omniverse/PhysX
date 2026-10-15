# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(OvstageOutputCpp LANGUAGES CXX)

find_package(ovphysx REQUIRED)

add_executable(ovstage_output_cpp main.cpp)
target_compile_features(ovstage_output_cpp PRIVATE cxx_std_17)
target_link_libraries(ovstage_output_cpp PRIVATE ovphysx::ovphysx ovphysx::ovstage)
target_include_directories(ovstage_output_cpp PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../common")
get_filename_component(OVPHYSX_TEST_DATA_DIR "${CMAKE_CURRENT_LIST_DIR}/../../data" ABSOLUTE)
target_compile_definitions(ovstage_output_cpp PRIVATE OVPHYSX_TEST_DATA="${OVPHYSX_TEST_DATA_DIR}")
if(WIN32)
    ovphysx_copy_runtime_dlls(ovstage_output_cpp)
    # OVStage is not copied (it loads its plugins relative to itself), so its
    # directories have to reach the process through PATH.
    ovphysx_set_ovstage_runtime_env(ovstage_output_cpp)
endif()

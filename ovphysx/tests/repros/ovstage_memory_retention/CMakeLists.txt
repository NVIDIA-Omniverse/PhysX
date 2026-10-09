# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
# @implements REQ-CAPI-IMPORT-MEMORY-REPRO-001
cmake_minimum_required(VERSION 3.24)
project(ovstage_memory_retention_repro LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "This allocation diagnostic requires Linux/glibc (mallinfo2 and /proc).")
endif()
set(REPRO_OVPHYSX_VERSION "0.6.4" CACHE STRING "Exact ovphysx CMake package version")
set(REPRO_OVSTAGE_VERSION "0.2.1" CACHE STRING "Exact ovstage CMake package version")
find_package(ovphysx ${REPRO_OVPHYSX_VERSION} EXACT CONFIG REQUIRED)
find_package(ovstage ${REPRO_OVSTAGE_VERSION} EXACT CONFIG REQUIRED)
add_executable(control_drain_repro repro.cpp)
target_compile_features(control_drain_repro PRIVATE cxx_std_17)
target_link_libraries(control_drain_repro PRIVATE ovphysx::ovphysx ovstage::ovstage)

include(CTest)
if(BUILD_TESTING)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME ovstage_physics_import_memory_retention
        COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test_memory_retention.py"
            --binary $<TARGET_FILE:control_drain_repro>
            --output "${CMAKE_CURRENT_BINARY_DIR}/memory-test")
    set_tests_properties(ovstage_physics_import_memory_retention PROPERTIES
        TIMEOUT 620 LABELS "memory;repro")
endif()

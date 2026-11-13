# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(BasicWorkflowC C)

find_package(ovphysx REQUIRED)

add_executable(basic_workflow_c main.c)
set_source_files_properties(main.c PROPERTIES LANGUAGE C)
set_target_properties(basic_workflow_c PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(basic_workflow_c PRIVATE -Werror=implicit-function-declaration -pedantic)
endif()
if(MSVC)
    target_compile_options(basic_workflow_c PRIVATE /TC)
endif()

target_link_libraries(basic_workflow_c PRIVATE ovphysx::ovphysx ovphysx::ovstage)
target_include_directories(basic_workflow_c PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../common")
get_filename_component(OVPHYSX_TEST_DATA_DIR "${CMAKE_CURRENT_LIST_DIR}/../../data" ABSOLUTE)
target_compile_definitions(basic_workflow_c PRIVATE OVPHYSX_TEST_DATA="${OVPHYSX_TEST_DATA_DIR}")

if(WIN32)
    ovphysx_copy_runtime_dlls(basic_workflow_c)
    ovphysx_set_ovstage_runtime_env(basic_workflow_c)
endif()

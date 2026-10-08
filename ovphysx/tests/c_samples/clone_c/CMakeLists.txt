# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(CloneC C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(ovphysx REQUIRED)

add_executable(clone_c main.c)

# Force the C language so the sample never compiles as C++.
set_source_files_properties(main.c PROPERTIES LANGUAGE C)
set_target_properties(clone_c PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)

# Compiler-specific flags that enforce strict C compilation.
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    # An implicit declaration error catches any C++-only symbol leaking into the C sample.
    target_compile_options(clone_c PRIVATE 
        -Werror=implicit-function-declaration
        -pedantic
    )
endif()

if(MSVC)
    # /TC makes MSVC compile every source as C.
    target_compile_options(clone_c PRIVATE /TC)
endif()

target_link_libraries(clone_c PRIVATE ovphysx::ovphysx ovphysx::ovstage)
target_include_directories(clone_c PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../common")
get_filename_component(OVPHYSX_TEST_DATA_DIR "${CMAKE_CURRENT_LIST_DIR}/../../data" ABSOLUTE)
target_compile_definitions(clone_c PRIVATE
    OVPHYSX_TEST_DATA="${OVPHYSX_TEST_DATA_DIR}"
)
if(WIN32)
    ovphysx_copy_runtime_dlls(clone_c)
endif()

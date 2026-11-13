# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(InstalledPublicHeaders LANGUAGES C CXX)

# No source-tree include paths or ovphysx build targets: missing SDK files must fail.
foreach(root OVPHYSX_SDK_ROOT OVPHYSX_OVSTAGE_ROOT)
    if(NOT IS_DIRECTORY "${${root}}/include")
        message(FATAL_ERROR "${root} must name an installed SDK with an include directory")
    endif()
endforeach()
add_library(installed_public_headers OBJECT headers.c headers.cpp)
target_include_directories(installed_public_headers PRIVATE
    "${OVPHYSX_SDK_ROOT}/include" "${OVPHYSX_OVSTAGE_ROOT}/include")
set_target_properties(installed_public_headers PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF
    CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)

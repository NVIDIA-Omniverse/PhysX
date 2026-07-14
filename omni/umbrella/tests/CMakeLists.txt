# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

if(NOT UMBRELLA_ENABLE_TESTS)
    return()
endif()

message(STATUS "Building C++ tests")
add_subdirectory(cpp)

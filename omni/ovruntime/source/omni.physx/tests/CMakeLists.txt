# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

enable_testing()

message(STATUS "Building ovruntime tests")

add_subdirectory(test.unit)
add_subdirectory(test.benchmarks)

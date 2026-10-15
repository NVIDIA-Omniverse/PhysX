# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# No-CUDA validation. Same compile, install, and wheel as validate_all.cmake,
# then only the tests that pass with no CUDA driver. GPU passes are not run.
#
#   cmake -P scripts/validate_cpu.cmake

cmake_minimum_required(VERSION 3.16)

set(OVPHYSX_SKIP_GPU_PASSES ON)
include("${CMAKE_CURRENT_LIST_DIR}/validate_all.cmake")

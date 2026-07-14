# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# Build options for Physics ovruntime

option(OVRUNTIME_ENABLE_TESTS "Enable unit tests" ON)
option(OVRUNTIME_ENABLE_BENCHMARK_CTEST "Register benchmarks as ctests" OFF)
option(OVRUNTIME_ENABLE_PYTHON_BINDINGS "Enable Python bindings" ON)
option(OVRUNTIME_DEV_PHYSX "Build PhysX SDK from source instead of using packman package" OFF)
option(OVRUNTIME_DEV_SCHEMA "Use locally-built physics schema (../schema/) instead of packman package" OFF)
option(OVRUNTIME_ENABLE_COVERAGE "Enable C++ code coverage instrumentation (gcov)" OFF)

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Regenerate the C++ benchmark catalogue (BENCHMARK_SUMMARY.md) for the ovphysx benchmark suite.
# Wrapper around scripts/get_benchmark_summary.py, mirroring ovruntime's
# cmake/GenerateBenchmarkSummary.cmake. The catalogue is a build artifact, not a checked-in file.
# It is regenerated into _build/generated/benchmark-summaries/ovphysx/BENCHMARK_SUMMARY.md whenever
# OVPHYSX_BUILD_BENCHMARKS is ON, from the source-controlled analysis side-car
# scripts/benchmark_summary_analysis.json.
#
# Usage:
#     cmake -P scripts/generate_benchmark_summary.cmake

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
get_filename_component(OVPHYSX_ROOT "${SCRIPT_DIR}/.." ABSOLUTE)

if(WIN32)
    set(_TARGET_PYTHON "${OVPHYSX_ROOT}/_build/target-deps/python/python.exe")
else()
    set(_TARGET_PYTHON "${OVPHYSX_ROOT}/_build/target-deps/python/bin/python3")
endif()

# Fall back to system python3 if target-deps is not fetched yet. The script uses only the stdlib,
# so any Python 3.8+ works.
if(NOT EXISTS "${_TARGET_PYTHON}")
    # find_program() treats an already-set variable as resolved and skips the PATH search, so the
    # missing packman path set above has to be cleared first.
    unset(_TARGET_PYTHON)
    find_program(_TARGET_PYTHON NAMES python3 python)
    if(NOT _TARGET_PYTHON)
        message(FATAL_ERROR "No Python interpreter found (looked at packman + PATH)")
    endif()
endif()

set(_GEN_SCRIPT "${SCRIPT_DIR}/get_benchmark_summary.py")
if(NOT EXISTS "${_GEN_SCRIPT}")
    message(FATAL_ERROR "Generator script not found: ${_GEN_SCRIPT}")
endif()

message(STATUS "Generating C++ benchmark catalogue (BENCHMARK_SUMMARY.md) into _build/generated/benchmark-summaries")
# --check is always on. Every registered row must have an analysis note in the side-car and every
# side-car key must match a registered row, otherwise the build fails here instead of shipping a
# drifted catalogue.
execute_process(
    COMMAND "${_TARGET_PYTHON}" "${_GEN_SCRIPT}" "--check"
    WORKING_DIRECTORY "${OVPHYSX_ROOT}"
    RESULT_VARIABLE _RC
)
if(NOT _RC EQUAL 0)
    message(FATAL_ERROR
        "BENCHMARK_SUMMARY.md generation/--check failed (exit ${_RC}).\n"
        "Either a registered row has no analysis note in scripts/benchmark_summary_analysis.json, "
        "or a side-car entry no longer matches a registered row (stale/renamed/deleted source). "
        "Add or refresh the entry, or remove the stale key, then re-run.")
endif()

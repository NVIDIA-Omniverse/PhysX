# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-TESTDEPS-001
# @covers AC-1 AC-2 AC-3
#
# ovphysx Python benchmark driver.
# Mirrors scripts/test_python_runtime.cmake but runs the pytest-benchmark suite
# under tests/python_benchmarks/ in two passes (CPU and GPU), because the
# ovphysx process device mode is locked by the first PhysX() call.
#
# Usage:
#   cmake -P scripts/test_benchmarks_python.cmake
#
# Environment variables:
#   BENCHMARK_REGENERATE=1     Save a new pytest-benchmark baseline (compare-fail off).
#   BENCHMARK_FILTER=<expr>    pytest -k expression to filter.
#   BENCHMARK_GPU=0            Skip the GPU pass.
#   BENCHMARK_CPU=0            Skip the CPU pass.
#
# Prerequisites:
#   cmake -P scripts/install.cmake

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")
include("${SCRIPT_DIR}/build_common.cmake")
include("${SCRIPT_DIR}/dev_schema_provider.cmake")

set(INSTALL_DIR "${PROJECT_ROOT}/_install")
set(BENCHMARK_DIR "${PROJECT_ROOT}/tests/python_benchmarks")
set(RESULTS_DIR "${PROJECT_ROOT}/_build/benchmark_results/python")
file(MAKE_DIRECTORY "${RESULTS_DIR}")

if(NOT EXISTS "${BENCHMARK_DIR}")
    message(FATAL_ERROR "Python benchmark directory not found: ${BENCHMARK_DIR}")
endif()
if(NOT EXISTS "${INSTALL_DIR}/plugins")
    message(FATAL_ERROR "Install directory not found at ${INSTALL_DIR}/plugins; run cmake -P scripts/install.cmake first.")
endif()

# Use packman Python (matches test_python_runtime.cmake).
if(WIN32)
    set(TARGET_PYTHON "${PROJECT_ROOT}/_build/target-deps/python/python.exe")
else()
    set(TARGET_PYTHON "${PROJECT_ROOT}/_build/target-deps/python/bin/python3")
endif()

if(NOT OVPHYSX_UV_COMMAND)
    message(FATAL_ERROR "uv not found (needed for running pytest-benchmark)")
endif()

# Create the venv with packman Python, the same recipe as test_python_runtime.cmake.
set(VENV_DIR "${BENCHMARK_DIR}/.venv")
if(EXISTS "${VENV_DIR}")
    file(REMOVE_RECURSE "${VENV_DIR}")
endif()
message(STATUS "Creating venv with target-deps Python at ${VENV_DIR}...")
execute_process(
    COMMAND "${TARGET_PYTHON}" -m venv "${VENV_DIR}"
    WORKING_DIRECTORY "${BENCHMARK_DIR}"
    RESULT_VARIABLE VENV_RESULT
)
if(NOT VENV_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to create venv with target-deps Python")
endif()

set(REGENERATE "$ENV{BENCHMARK_REGENERATE}")
set(BENCHMARK_FILTER "$ENV{BENCHMARK_FILTER}")
set(RUN_GPU TRUE)
set(RUN_CPU TRUE)
if("$ENV{BENCHMARK_GPU}" STREQUAL "0" OR "$ENV{BENCHMARK_GPU}" STREQUAL "false")
    set(RUN_GPU FALSE)
endif()
if("$ENV{BENCHMARK_CPU}" STREQUAL "0" OR "$ENV{BENCHMARK_CPU}" STREQUAL "false")
    set(RUN_CPU FALSE)
endif()

set(UV_CACHE_DIR_PATH "${PROJECT_ROOT}/_build/.uv_cache")
file(MAKE_DIRECTORY "${UV_CACHE_DIR_PATH}")
include("${CMAKE_CURRENT_LIST_DIR}/ovstage_index.cmake")
ovphysx_ovstage_index_env(OVSTAGE_INDEX_ENV PROJECT tests/python_benchmarks)
set(BASE_UV_ENV
    "UV_CACHE_DIR=${UV_CACHE_DIR_PATH}"
    "UV_NO_CONFIG=1"
    ${OVSTAGE_INDEX_ENV}
    "UV_HTTP_TIMEOUT=300"
    "UV_SKIP_WHEEL_FILENAME_CHECK=1"
    # PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 is deliberately absent. It prevents
    # pytest-benchmark from registering even when passed via -p, so the
    # --benchmark-* args get rejected. The venv only pins pytest,
    # pytest-benchmark, numpy and optionally torch via the [gpu] extra, so
    # there are no surprise plugins to guard against.
)
# Load native OVStage from the configured package.
ovphysx_resolve_ovstage_paths()
if(WIN32)
    set(OVSTAGE_RUNTIME_DIR "${OVPHYSX_OVSTAGE_RUNTIME_DIR}")
    set(OVSTAGE_RUNTIME_FILE "${OVSTAGE_RUNTIME_DIR}/ovstage.dll")
    set(_WINDOWS_PATH_SEGMENTS
        "${INSTALL_DIR}/bin"
        "${INSTALL_DIR}/plugins"
        "${OVSTAGE_RUNTIME_DIR}"
        # ovstage.dll statically imports the USD monolith, which lives in
        # OVStage's plugins/ dir and which the SDK does not ship
        # (REQ-PACKAGING-USDFREE-001 AC-4). Linux resolves it via ovstage's
        # RUNPATH ($ORIGIN/plugins). Windows needs it on PATH.
        "${OVSTAGE_RUNTIME_DIR}/plugins"
        "$ENV{PATH}"
    )
    list(JOIN _WINDOWS_PATH_SEGMENTS ";" _WINDOWS_PATH_VALUE)
    set(ENV{PATH} "${_WINDOWS_PATH_VALUE}")
    list(APPEND BASE_UV_ENV
        "OVSTAGE_LIBRARY_PATH_HINT=${OVSTAGE_RUNTIME_DIR}"
    )
else()
    set(OVSTAGE_RUNTIME_DIR "${OVPHYSX_OVSTAGE_RUNTIME_DIR}")
    set(OVSTAGE_RUNTIME_FILE "${OVSTAGE_RUNTIME_DIR}/libovstage.so")
    list(APPEND BASE_UV_ENV
        "OVSTAGE_LIBRARY_PATH_HINT=${OVSTAGE_RUNTIME_DIR}"
        "LD_LIBRARY_PATH=${INSTALL_DIR}/lib:${INSTALL_DIR}/plugins:${OVSTAGE_RUNTIME_DIR}:$ENV{LD_LIBRARY_PATH}"
    )
endif()
if(NOT EXISTS "${OVSTAGE_RUNTIME_FILE}")
    message(FATAL_ERROR "ovstage runtime not found at ${OVSTAGE_RUNTIME_FILE}")
endif()

function(run_python_bench_pass _LABEL _DEVICE)
    set(_JSON "${RESULTS_DIR}/${_LABEL}.json")
    set(_LOG "${RESULTS_DIR}/${_LABEL}.log")
    file(REMOVE "${_LOG}")

    set(_PYTEST_ARGS
        --benchmark-only
        --benchmark-json=${_JSON}
        --bench-device=${_DEVICE}
        -v -s
    )
    if(BENCHMARK_FILTER)
        list(APPEND _PYTEST_ARGS -k "${BENCHMARK_FILTER}")
    endif()
    if(REGENERATE)
        # Save a new baseline without failing on regression.
        list(APPEND _PYTEST_ARGS --benchmark-save=${PLATFORM_NAME}-${_LABEL})
    else()
        # Compare against the most recent saved baseline and fail on a >10% regression in mean.
        list(APPEND _PYTEST_ARGS
            --benchmark-compare
            --benchmark-compare-fail=mean:10%
        )
    endif()

    # The device is selected by the --bench-device pytest option above, not by an env var.
    set(_PASS_ENV ${BASE_UV_ENV})

    message(STATUS "")
    message(STATUS "--- python ${_LABEL} pass ---")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env ${_PASS_ENV}
                "${OVPHYSX_UV_COMMAND}" run pytest ${_PYTEST_ARGS}
        WORKING_DIRECTORY "${BENCHMARK_DIR}"
        TIMEOUT 1800
        RESULT_VARIABLE _RC
        OUTPUT_VARIABLE _STDOUT
        ERROR_VARIABLE  _STDERR
        ECHO_OUTPUT_VARIABLE
        ECHO_ERROR_VARIABLE
    )
    file(WRITE "${_LOG}" "${_STDOUT}${_STDERR}")
    message(STATUS "Exit code (python ${_LABEL}): ${_RC}")

    if(NOT _RC STREQUAL "0")
        # The GPU pass is a soft skip when the device is unavailable. pytest exits 1
        # on any test failure, so a fixture skip cannot be distinguished from an
        # assertion failure here and stderr is matched instead.
        if(_LABEL STREQUAL "gpu" AND _STDERR MATCHES "GPU not available|cuda not available|no CUDA")
            message(STATUS "GPU not available; skipping GPU python benchmark pass.")
            return()
        endif()
        message(FATAL_ERROR "Python benchmarks ${_LABEL} pass failed (exit ${_RC})")
    endif()
endfunction()

if(RUN_CPU)
    run_python_bench_pass("cpu" "cpu")
endif()
if(RUN_GPU)
    run_python_bench_pass("gpu" "gpu")
endif()

message(STATUS "")
message(STATUS "Python benchmarks: PASSED")

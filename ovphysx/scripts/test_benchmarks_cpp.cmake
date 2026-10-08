# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-BENCHMARK-001
# @covers AC-1 AC-4
#
# ovphysx C++ benchmark driver.
#
# Runs ovphysx_benchmarks (the omni.physx-derived harness) against the
# installed SDK in separate passes, because the ovphysx process device
# mode is locked by the first PhysX() call.
#
# Usage:
#   cmake -P scripts/test_benchmarks_cpp.cmake
#
# Environment variables:
#   BENCHMARK_REGENERATE=1     Rewrite the baseline (golden) file in-place.
#   BENCHMARK_TOLERANCE=<pct>  Override the +/- tolerance (--slop) (default 10).
#   BENCHMARK_FILTER=<pat>     Only run benchmarks matching the glob pattern.
#   BENCHMARK_GPU=0            Skip the GPU pass (e.g. on CI hosts without a GPU).
#   BENCHMARK_CPU=0            Skip the CPU pass.
#   BENCHMARK_CPU_ST=0         Skip the single-threaded CPU baseline pass.
#                              (Default ON. Scoped to Step.cubes20_cpu via --filter
#                              to keep CI cost bounded.)
#   BENCHMARK_HIDDEN=1         Include hidden rows in every enabled pass.
#   BENCHMARK_EXPECT_ROWS=N    Require exactly N positive rows. Valid only when
#                              exactly one pass is enabled.
#   BENCHMARK_RESULTS_DIR=PATH Override the report/log/status directory. Relative
#                              paths are resolved from the ovphysx project root.
#   BENCHMARK_DIRECTGPU=1      Add --directGpu to the GPU pass (requires the GPU pass to be
#                              enabled). Needed for Probe.* and OutputRead.*_gpu rows, which throw
#                              under plain --forceGpu.
#
# Prerequisites:
#   ./build.sh --benchmarks && \
#   cmake -P scripts/install.cmake

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")
include("${SCRIPT_DIR}/build_common.cmake")
include("${SCRIPT_DIR}/benchmark_driver_common.cmake")

set(INSTALL_DIR "${PROJECT_ROOT}/_install")
set(INSTALL_PLUGINS_DIR "${INSTALL_DIR}/plugins")
set(BENCHMARK_BINARY "${STANDARD_OUTPUT_DIR}/ovphysx_benchmarks${EXE_SUFFIX}")
set(RESULTS_DIR "$ENV{BENCHMARK_RESULTS_DIR}")
if(RESULTS_DIR)
    get_filename_component(RESULTS_DIR "${RESULTS_DIR}" ABSOLUTE BASE_DIR "${PROJECT_ROOT}")
else()
    set(RESULTS_DIR "${PROJECT_ROOT}/_build/benchmark_results")
endif()
file(MAKE_DIRECTORY "${RESULTS_DIR}")

if(NOT EXISTS "${INSTALL_DIR}")
    message(FATAL_ERROR
        "_install/ not found.\n"
        "Run: ./build.sh --benchmarks && cmake -P scripts/install.cmake")
endif()
if(NOT EXISTS "${BENCHMARK_BINARY}")
    message(FATAL_ERROR
        "ovphysx_benchmarks not built at: ${BENCHMARK_BINARY}\n"
        "Run: ./build.sh --benchmarks")
endif()

# On Windows the DLLs must be on PATH. On Linux RPATH handles it.
if(OS_NAME STREQUAL "windows")
    set(INSTALL_BIN_DIR "${INSTALL_DIR}/bin")
    set(INSTALL_BIN_DEPS_DIR "${INSTALL_PLUGINS_DIR}/bin/deps")
    set(TARGET_PYTHON_DIR "${PROJECT_ROOT}/_build/target-deps/python")
    # ovphysx_benchmarks links ovphysx::ovstage, and package_deps stages the
    # ovstage plugin tree rather than ovstage.dll itself, so the configured
    # ovstage runtime has to be on PATH as well. Without it the harness cannot
    # load and Windows reports 0xC0000135 before the first benchmark runs.
    # ovstage.dll in turn statically imports the USD monolith, which lives in
    # ovstage's plugins/ subdir and which the SDK does not ship
    # (REQ-PACKAGING-USDFREE-001 AC-4). Linux resolves it via ovstage's RUNPATH
    # ($ORIGIN/plugins). Windows needs that subdir on PATH too. Both go after
    # the _install entries so installed DLLs still win for anything duplicated.
    ovphysx_resolve_ovstage_paths()
    set(OVSTAGE_RUNTIME_DIR "${OVPHYSX_OVSTAGE_RUNTIME_DIR}")
    set(_BENCH_PATH_SEGMENTS
        "${INSTALL_BIN_DIR}"
        "${INSTALL_PLUGINS_DIR}"
        "${INSTALL_BIN_DEPS_DIR}"
        "${OVSTAGE_RUNTIME_DIR}"
        "${OVSTAGE_RUNTIME_DIR}/plugins"
        "${TARGET_PYTHON_DIR}"
        "$ENV{PATH}"
    )
    list(JOIN _BENCH_PATH_SEGMENTS "${PATH_SEP}" _BENCH_PATH_VALUE)
    set(ENV{PATH} "${_BENCH_PATH_VALUE}")
    set(ENV{OVPHYSX_LIB} "${INSTALL_BIN_DIR}/ovphysx.dll")
endif()

set(REGENERATE "$ENV{BENCHMARK_REGENERATE}")
set(TOLERANCE "$ENV{BENCHMARK_TOLERANCE}")
if(NOT TOLERANCE)
    set(TOLERANCE "10")
endif()
set(BENCHMARK_FILTER "$ENV{BENCHMARK_FILTER}")

set(RUN_GPU TRUE)
set(RUN_CPU TRUE)
set(RUN_CPU_ST TRUE)
if("$ENV{BENCHMARK_GPU}" STREQUAL "0" OR "$ENV{BENCHMARK_GPU}" STREQUAL "false")
    set(RUN_GPU FALSE)
endif()
if("$ENV{BENCHMARK_CPU}" STREQUAL "0" OR "$ENV{BENCHMARK_CPU}" STREQUAL "false")
    set(RUN_CPU FALSE)
endif()
if("$ENV{BENCHMARK_CPU_ST}" STREQUAL "0" OR "$ENV{BENCHMARK_CPU_ST}" STREQUAL "false")
    set(RUN_CPU_ST FALSE)
endif()

set(RUN_DIRECTGPU FALSE)
if("$ENV{BENCHMARK_DIRECTGPU}" STREQUAL "1" OR "$ENV{BENCHMARK_DIRECTGPU}" STREQUAL "true")
    set(RUN_DIRECTGPU TRUE)
    if(NOT RUN_GPU)
        message(FATAL_ERROR "BENCHMARK_DIRECTGPU=1 requires the GPU pass to be enabled (BENCHMARK_GPU != 0)")
    endif()
endif()
ovphysx_benchmark_driver_configure(
    "${RUN_GPU}"
    "${RUN_CPU}"
    "${RUN_CPU_ST}"
    "$ENV{BENCHMARK_HIDDEN}"
    "$ENV{BENCHMARK_EXPECT_ROWS}"
)

message(STATUS "")
message(STATUS "=== ovphysx C++ benchmarks (using INSTALLED SDK) ===")
message(STATUS "  Project root: ${PROJECT_ROOT}")
message(STATUS "  Binary:       ${BENCHMARK_BINARY}")
message(STATUS "  Results:      ${RESULTS_DIR}")
message(STATUS "  Tolerance:    +/- ${TOLERANCE}%")
message(STATUS "  Regenerate:   ${REGENERATE}")
message(STATUS "  Filter:       '${BENCHMARK_FILTER}'")
message(STATUS "  Run GPU pass:    ${RUN_GPU}")
message(STATUS "  DirectGPU:       ${RUN_DIRECTGPU}")
message(STATUS "  Run CPU pass:    ${RUN_CPU}")
message(STATUS "  Run CPU-ST pass: ${RUN_CPU_ST}")
message(STATUS "  Include hidden:  ${BENCHMARK_DRIVER_HIDDEN}")
message(STATUS "  Expected rows:   '${BENCHMARK_DRIVER_EXPECT_ROWS}'")
if(OS_NAME STREQUAL "windows")
    message(STATUS "  OVStage runtime: ${OVSTAGE_RUNTIME_DIR}")
endif()

# _THREADS is passed through to --threads=N. Empty means do not override.
# _FILTER_OVERRIDE replaces BENCHMARK_FILTER for this pass. Empty means honor
# the user's BENCHMARK_FILTER. The cpu_st pass uses it to scope to
# Step.cubes20_cpu so the single-threaded baseline does not run every CPU
# bench twice.
function(run_bench_pass _LABEL _FORCE_GPU _THREADS _FILTER_OVERRIDE _DIRECT_GPU)
    set(_REPORT "${RESULTS_DIR}/${_LABEL}.txt")
    set(_LOG "${RESULTS_DIR}/${_LABEL}.log")
    set(_STATUS "${RESULTS_DIR}/${_LABEL}.status.json")
    file(REMOVE "${_REPORT}" "${_LOG}" "${_STATUS}")

    set(_ARGS
        "--data=${PROJECT_ROOT}/tests/data"
        "--report=${_REPORT}"
        "--slop=${TOLERANCE}"
        "--verbose"
    )
    if(_FORCE_GPU)
        list(APPEND _ARGS "--forceGpu")
    endif()
    if(_DIRECT_GPU)
        list(APPEND _ARGS "--directGpu")
    endif()
    if(NOT "${_THREADS}" STREQUAL "")
        list(APPEND _ARGS "--threads=${_THREADS}")
    endif()
    if(NOT "${_FILTER_OVERRIDE}" STREQUAL "")
        list(APPEND _ARGS "--filter=${_FILTER_OVERRIDE}")
    elseif(BENCHMARK_FILTER)
        list(APPEND _ARGS "--filter=${BENCHMARK_FILTER}")
    endif()
    if(REGENERATE)
        list(APPEND _ARGS "--regenerate")
    endif()
    ovphysx_benchmark_driver_append_hidden(_ARGS)

    message(STATUS "")
    message(STATUS "--- ${_LABEL} pass ---")
    execute_process(
        COMMAND "${BENCHMARK_BINARY}" ${_ARGS}
        WORKING_DIRECTORY "${PROJECT_ROOT}"
        TIMEOUT 1800
        RESULT_VARIABLE _RC
        OUTPUT_VARIABLE _STDOUT
        ERROR_VARIABLE  _STDERR
        ECHO_OUTPUT_VARIABLE
        ECHO_ERROR_VARIABLE
    )
    file(WRITE "${_LOG}" "${_STDOUT}${_STDERR}")
    ovphysx_benchmark_driver_write_status("${_STATUS}" "${_RC}")
    message(STATUS "Exit code (${_LABEL}): ${_RC}")

    if(NOT _RC STREQUAL "0")
        message(FATAL_ERROR "ovphysx_benchmarks ${_LABEL} pass failed (exit ${_RC})")
    endif()
    if(NOT "${BENCHMARK_DRIVER_EXPECT_ROWS}" STREQUAL "")
        ovphysx_benchmark_driver_require_rows(
            "${_REPORT}"
            "${BENCHMARK_DRIVER_EXPECT_ROWS}"
            "${_LABEL}"
        )
    endif()
endfunction()

if(RUN_GPU)
    run_bench_pass("gpu" TRUE "" "" "${RUN_DIRECTGPU}")
endif()
if(RUN_CPU)
    run_bench_pass("cpu" FALSE "" "" FALSE)
endif()
if(RUN_CPU_ST)
    # Single-threaded baseline. --threads=1 sets the /physics/numThreads
    # Carbonite setting before PhysX bootstrap so the dispatcher runs one
    # worker. Scoped to cubes20, the cheap minimal scene where the contrast
    # with the multithreaded pass is the signal.
    run_bench_pass("cpu_st" FALSE "1" "Step.cubes20_cpu" FALSE)
endif()

message(STATUS "")
message(STATUS "ovphysx C++ benchmarks: PASSED")

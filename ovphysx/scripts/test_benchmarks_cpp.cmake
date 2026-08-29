# ovphysx C++ benchmark driver
#
# Runs ovphysx_benchmarks (the omni.physx-derived harness) against the
# INSTALLED SDK in two passes (GPU then CPU) because the ovphysx process
# device-mode is locked by the first PhysX() call.
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
#                              (Default ON; scoped to Step.cubes20_cpu via --filter
#                              to keep CI cost bounded.)
#
# Prerequisites:
#   cmake -DOVPHYSX_BUILD_BENCHMARKS=ON -P scripts/build.cmake && \
#   cmake -P scripts/install.cmake

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")
include("${SCRIPT_DIR}/build_common.cmake")

set(INSTALL_DIR "${PROJECT_ROOT}/_install")
set(INSTALL_PLUGINS_DIR "${INSTALL_DIR}/plugins")
set(BENCHMARK_BINARY "${STANDARD_OUTPUT_DIR}/ovphysx_benchmarks${EXE_SUFFIX}")
set(RESULTS_DIR "${PROJECT_ROOT}/_build/benchmark_results")
file(MAKE_DIRECTORY "${RESULTS_DIR}")

if(NOT EXISTS "${INSTALL_DIR}")
    message(FATAL_ERROR
        "_install/ not found.\n"
        "Run: cmake -DOVPHYSX_BUILD_BENCHMARKS=ON -P scripts/build.cmake && cmake -P scripts/install.cmake")
endif()
if(NOT EXISTS "${BENCHMARK_BINARY}")
    message(FATAL_ERROR
        "ovphysx_benchmarks not built at: ${BENCHMARK_BINARY}\n"
        "Configure with -DOVPHYSX_BUILD_BENCHMARKS=ON and rebuild.")
endif()

# Windows: DLLs must be on PATH. Linux: RPATH handles it.
if(OS_NAME STREQUAL "windows")
    set(INSTALL_BIN_DIR "${INSTALL_DIR}/bin")
    set(INSTALL_BIN_DEPS_DIR "${INSTALL_PLUGINS_DIR}/bin/deps")
    set(INSTALL_GPU_PLUGINS_DIR "${INSTALL_DIR}/plugins/gpu")
    set(TARGET_PYTHON_DIR "${PROJECT_ROOT}/_build/target-deps/python")
    set(ENV{PATH} "${INSTALL_BIN_DIR}${PATH_SEP}${INSTALL_PLUGINS_DIR}${PATH_SEP}${INSTALL_GPU_PLUGINS_DIR}${PATH_SEP}${INSTALL_BIN_DEPS_DIR}${PATH_SEP}${TARGET_PYTHON_DIR}${PATH_SEP}$ENV{PATH}")
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

message(STATUS "")
message(STATUS "=== ovphysx C++ benchmarks (using INSTALLED SDK) ===")
message(STATUS "  Project root: ${PROJECT_ROOT}")
message(STATUS "  Binary:       ${BENCHMARK_BINARY}")
message(STATUS "  Results:      ${RESULTS_DIR}")
message(STATUS "  Tolerance:    +/- ${TOLERANCE}%")
message(STATUS "  Regenerate:   ${REGENERATE}")
message(STATUS "  Filter:       '${BENCHMARK_FILTER}'")
message(STATUS "  Run GPU pass:    ${RUN_GPU}")
message(STATUS "  Run CPU pass:    ${RUN_CPU}")
message(STATUS "  Run CPU-ST pass: ${RUN_CPU_ST}")

# _THREADS: passed through to --threads=N. -1 = do not override (default).
# _FILTER_OVERRIDE: overrides BENCHMARK_FILTER for this pass. Empty = honor
#   the user's BENCHMARK_FILTER. Used by the cpu_st pass to scope to
#   Step.cubes20_cpu so the single-threaded baseline doesn't run every CPU
#   bench twice.
function(run_bench_pass _LABEL _FORCE_GPU _THREADS _FILTER_OVERRIDE)
    set(_REPORT "${RESULTS_DIR}/${_LABEL}.txt")
    set(_LOG "${RESULTS_DIR}/${_LABEL}.log")
    file(REMOVE "${_LOG}")

    set(_ARGS
        "--data=${PROJECT_ROOT}/tests/data"
        "--report=${_REPORT}"
        "--slop=${TOLERANCE}"
        "--verbose"
    )
    if(_FORCE_GPU)
        list(APPEND _ARGS "--forceGpu")
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
    message(STATUS "Exit code (${_LABEL}): ${_RC}")

    if(NOT _RC STREQUAL "0")
        message(FATAL_ERROR "ovphysx_benchmarks ${_LABEL} pass failed (exit ${_RC})")
    endif()
endfunction()

if(RUN_GPU)
    run_bench_pass("gpu" TRUE "" "")
endif()
if(RUN_CPU)
    run_bench_pass("cpu" FALSE "" "")
endif()
if(RUN_CPU_ST)
    # Single-threaded baseline. --threads=1 sets the
    # /physics/numThreads Carbonite setting before PhysX bootstrap so the
    # dispatcher runs one worker. Scope to cubes20 — the cheap minimal
    # scene where the contrast vs the multithreaded pass is the signal.
    run_bench_pass("cpu_st" FALSE "1" "Step.cubes20_cpu")
endif()

message(STATUS "")
message(STATUS "ovphysx C++ benchmarks: PASSED")

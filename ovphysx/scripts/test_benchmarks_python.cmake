# ovphysx Python benchmark driver
# Mirrors scripts/test_python_runtime.cmake but runs the pytest-benchmark suite
# under tests/python_benchmarks/ in two passes (CPU and GPU) because the
# ovphysx process device-mode is locked by the first PhysX() call.
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

# Create venv with packman Python -- same recipe as test_python_runtime.cmake.
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
set(OVSTAGE_WHEEL_DIR "${PROJECT_ROOT}/_build/target-deps/ovstage_wheel")
file(GLOB OVSTAGE_WHEELS "${OVSTAGE_WHEEL_DIR}/ovstage-*.whl")
if(NOT OVSTAGE_WHEELS)
    message(FATAL_ERROR "ovstage wheel not found in ${OVSTAGE_WHEEL_DIR}. Run the ovphysx build first.")
endif()
set(BASE_UV_ENV
    "UV_CACHE_DIR=${UV_CACHE_DIR_PATH}"
    "UV_NO_CONFIG=1"
    "UV_FIND_LINKS=${OVSTAGE_WHEEL_DIR}"
    "UV_HTTP_TIMEOUT=300"
    "UV_SKIP_WHEEL_FILENAME_CHECK=1"
    # PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 was set here originally but it
    # prevents pytest-benchmark from registering even when explicitly
    # passed via -p, so --benchmark-* args get rejected. Removed: the
    # venv only pins pytest + pytest-benchmark + numpy + (optionally
    # torch via the [gpu] extra), so there are no surprise plugins to
    # guard against.
)
# ovstage is not installed; consume the configured external package.
ovphysx_resolve_ovstage_paths()
set(OVSTAGE_PYTHON_DIR "${OVPHYSX_OVSTAGE_PYTHON_DIR}")
if(NOT OVSTAGE_PYTHON_DIR OR NOT EXISTS "${OVSTAGE_PYTHON_DIR}/ovstage/__init__.py")
    message(FATAL_ERROR
        "ovstage Python package not found under the configured ovstage root "
        "'${OVPHYSX_OVSTAGE_ROOT}'.\n"
        "Run scripts/fetch_deps.cmake so the ovstage release is extracted.")
endif()
if(WIN32)
    set(OVSTAGE_RUNTIME_DIR "${OVPHYSX_OVSTAGE_RUNTIME_DIR}")
    set(OVSTAGE_RUNTIME_FILE "${OVSTAGE_RUNTIME_DIR}/ovstage.dll")
    set(_WINDOWS_PATH_SEGMENTS
        "${INSTALL_DIR}/bin"
        "${INSTALL_DIR}/plugins"
        "${OVSTAGE_RUNTIME_DIR}"
        "${OVSTAGE_PYTHON_DIR}/ovstage/bin"
        "$ENV{PATH}"
    )
    list(JOIN _WINDOWS_PATH_SEGMENTS ";" _WINDOWS_PATH_VALUE)
    set(ENV{PATH} "${_WINDOWS_PATH_VALUE}")
    list(APPEND BASE_UV_ENV
        "PYTHONPATH=${OVSTAGE_PYTHON_DIR}"
        "OVSTAGE_LIBRARY_PATH_HINT=${OVSTAGE_RUNTIME_DIR}"
    )
else()
    set(OVSTAGE_RUNTIME_DIR "${OVSTAGE_PYTHON_DIR}/ovstage/bin")
    set(OVSTAGE_RUNTIME_FILE "${OVSTAGE_RUNTIME_DIR}/libovstage.so")
    list(APPEND BASE_UV_ENV
        "PYTHONPATH=${OVSTAGE_PYTHON_DIR}"
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
        # Save a new baseline; do not fail on regression.
        list(APPEND _PYTEST_ARGS --benchmark-save=${PLATFORM_NAME}-${_LABEL})
    else()
        # Compare against the most recent saved baseline; fail on >10% regression in mean.
        list(APPEND _PYTEST_ARGS
            --benchmark-compare
            --benchmark-compare-fail=mean:10%
        )
    endif()

    # Device knob now flows via pytest CLI option (--bench-device, above).
    # The env var was switched out per review.
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
        # Treat the GPU pass as a soft-skip when the device is unavailable.
        # pytest exits 1 on test failure -- we cannot distinguish a fixture skip
        # from an assertion failure here, so devs must rely on stderr for now.
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

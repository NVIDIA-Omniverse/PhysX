# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-CLOSURE-001
# @covers AC-4

# ovphysx full validation pipeline.
# Single-command entry point that builds, installs, packages the wheel, and runs every test.
#   cmake -P scripts/validate_all.cmake
#
# Options (passed via -D flags):
#   -DBUILD_TYPE=Debug|Release    Build type (default: Release)
#   -DDEV_PHYSX=ON|OFF            Build PhysX SDK from source (default: OFF)
#   -DDEV_SCHEMA=ON|OFF           Build and test against a separate local schema provider
#   -DSKIP_GLIBC_CHECK=ON         Skip the GLIBC/libstdc++ baseline check
#                                 (forwarded to all nested install/build_wheel
#                                 invocations via env var; useful when running
#                                 on a host with newer glibc than the baseline)
#   -DSKIP_LOCK_CHECK=ON          Skip packaging-lock checks in install/wheel
#   -DOVPHYSX_SKIP_GPU_PASSES=ON  After the build, install and wheel, run only
#                                 the tests that pass with no CUDA driver.
#                                 scripts/validate_cpu.cmake sets this.
#
# Pipeline stages (all automatic):
#   1. Build C++ libraries        (delegates to build.cmake, incremental)
#   2. Install SDK into _install/ (validate_all -> install_sdk target)
#   3. Build Python wheel         (validate_all -> build_wheel target)
#   4. Run all tests              (C++, Python runtime, wheel smoke, samples)
#
# The same install + wheel + test phases are available after a full build (build.sh or
# build.cmake) through the CMake target graph:
#   cmake --build _build --target validate_all      # install + wheel + all tests
# This script runs build.cmake first (Step 1) and then that target, so one command covers
# configure/build and validation. Scoped targets:
#   cmake --build _build --target validate_runtime  # runtime tests only
#   cmake --build _build --target validate_wheel    # wheel tests only
#
# Individual test suites can be run independently:
#   cmake -P scripts/test_cpp.cmake
#   cmake -P scripts/test_cpp_samples.cmake
#   cmake -P scripts/test_python_runtime.cmake
#   cmake -P scripts/test_python_wheel.cmake    # Requires wheel (python -m ovphysx smoke)
#   cmake -P scripts/test_python_samples.cmake  # Requires wheel (sample execution)

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
get_filename_component(PROJECT_ROOT "${SCRIPT_DIR}/.." ABSOLUTE)

# Checked directly because this script delegates rather than including
# build_common.cmake, so the failure would otherwise surface stages deep.
include("${PROJECT_ROOT}/cmake/RequireCMakeVersion.cmake")
ovphysx_require_cmake_version()

include("${SCRIPT_DIR}/build_directory.cmake")

message(STATUS "")
message(STATUS "========================================")
if(OVPHYSX_SKIP_GPU_PASSES)
    message(STATUS "ovphysx No-CUDA Validation Pipeline")
else()
    message(STATUS "ovphysx Full Validation Pipeline")
endif()
message(STATUS "========================================")
message(STATUS "Project root: ${PROJECT_ROOT}")
message(STATUS "")
if(OVPHYSX_SKIP_GPU_PASSES)
    message(STATUS "This will build, then run the tests that pass with no CUDA driver:")
else()
    message(STATUS "This will run the full pipeline:")
endif()
message(STATUS "  1. Build C++ libraries (incremental)")
message(STATUS "  1b. Python type stubs (pyright)")
message(STATUS "  2. Install SDK into _install/")
message(STATUS "  3. Build Python wheel into _dist/")
if(OVPHYSX_SKIP_GPU_PASSES)
    message(STATUS "  4. Tests that pass with no CUDA driver")
    message(STATUS "     GPU passes are not run")
else()
    message(STATUS "  4. C++ unit tests (GPU + CPU)")
    message(STATUS "  5. C++ sample applications")
    message(STATUS "  6. Python runtime tests (pytest)")
    message(STATUS "  7. Python wheel smoke tests (python -m ovphysx, 4 Python versions)")
    message(STATUS "  8. Python sample applications (4 Python versions)")
endif()
message(STATUS "")

# --- Preflight: uv must be discoverable before the long build starts ---
# Every Python stage (wheel, runtime tests, samples) needs uv. Resolving it here,
# in the user's shell, makes a missing uv fail in seconds with an actionable
# message instead of much later inside a nested install/test step. The hints
# mirror build_common.cmake's resolver so the nested steps, which may run under
# an MSBuild/CTest environment that does not inherit the interactive PATH, find
# the same uv this preflight does.
set(_UV_HINTS
    "$ENV{ProgramData}/chocolatey/bin"
    "$ENV{USERPROFILE}/.local/bin"
    "$ENV{LOCALAPPDATA}/uv/bin"
    "$ENV{USERPROFILE}/.cargo/bin"
)
file(GLOB _PY_SCRIPT_DIRS "$ENV{LOCALAPPDATA}/Programs/Python/*/Scripts")
file(GLOB _PY_USER_SCRIPT_DIRS "$ENV{APPDATA}/Python/Python*/Scripts")
list(APPEND _UV_HINTS ${_PY_SCRIPT_DIRS} ${_PY_USER_SCRIPT_DIRS})
find_program(_VALIDATE_UV NAMES uv uv.exe HINTS ${_UV_HINTS})
if(NOT _VALIDATE_UV)
    message(FATAL_ERROR
        "Preflight failed: uv (Astral's Python package manager) was not found.\n"
        "ovphysx's wheel and Python test stages require it.\n"
        "Install it: https://docs.astral.sh/uv/getting-started/installation/\n"
        "Then make sure `uv --version` works in this shell before re-running.")
endif()
message(STATUS "Preflight: found uv at ${_VALIDATE_UV}")
message(STATUS "")

# Forward known SKIP_* flags to nested subprocesses via the environment. The
# Step 2-8 hop runs `cmake --build _build --target validate_all`, whose inner
# install_sdk / build_wheel custom targets invoke `cmake -P` with hardcoded
# argument lists and cannot pick up extra -D flags from this script. An env var
# passes through every layer, and each script-mode tool (install.cmake,
# build_wheel.cmake, build_common.cmake) reads these as either -D or ENV{}.
#
# The value is forwarded as-is. The consumer's if() handles ON/OFF/TRUE/FALSE,
# so -DSKIP_GLIBC_CHECK=OFF forwards the literal string "OFF", which the
# receiving if() evaluates as false.
foreach(_OVPHYSX_FORWARD_FLAG SKIP_GLIBC_CHECK SKIP_LOCK_CHECK OVPHYSX_USE_RELEASE_RUNTIME_DEPS)
    if(DEFINED ${_OVPHYSX_FORWARD_FLAG} AND NOT "${${_OVPHYSX_FORWARD_FLAG}}" STREQUAL "")
        set(ENV{${_OVPHYSX_FORWARD_FLAG}} "${${_OVPHYSX_FORWARD_FLAG}}")
    endif()
endforeach()

# --- Generator default for the aggregate flow (Windows host only) ---
# build.cmake auto-detects the Visual Studio generator on Windows when VS is
# installed. That suits interactive build.sh/build.bat, but this aggregate runs
# install_sdk/build_wheel as MSBuild custom targets whose environment does not
# inherit the interactive shell PATH, so nested `cmake -P` scripts fail to find
# tools (uv, powershell). Ninja has no such custom-target env isolation, so this
# entry point defaults to Ninja on Windows unless the caller set GENERATOR
# explicitly. An existing build tree keeps its cached generator instead, which
# avoids build.cmake's generator-mismatch fatal. Linux is unaffected because
# build.cmake's generator logic is WIN32-only and ignores GENERATOR elsewhere.
if(CMAKE_HOST_WIN32 AND (NOT DEFINED ENV{GENERATOR} OR "$ENV{GENERATOR}" STREQUAL ""))
    if(EXISTS "${BUILD_PATH}/CMakeCache.txt")
        file(STRINGS "${BUILD_PATH}/CMakeCache.txt" _CACHED_GEN_LINE
            REGEX "^CMAKE_GENERATOR:INTERNAL=")
        if(_CACHED_GEN_LINE MATCHES "Visual Studio")
            set(ENV{GENERATOR} "vs")
            message(WARNING
                "Existing _build/ uses the Visual Studio generator. validate_all is "
                "known to fail under VS on Windows because MSBuild custom targets "
                "(install_sdk/build_wheel) do not inherit PATH for nested cmake -P "
                "scripts. If install/wheel steps fail, clean and re-run to use Ninja: "
                "`build.sh --clean` (or remove _build/).")
        else()
            set(ENV{GENERATOR} "ninja")
        endif()
    else()
        set(ENV{GENERATOR} "ninja")
        message(STATUS "Defaulting GENERATOR=ninja for the validate_all flow on Windows "
                       "(set GENERATOR=vs to override).")
    endif()
endif()

# --- Step 1: Build (always runs, incremental) ---
# Append in the order given. list(INSERT 0) would reverse repeated -D flags,
# and the last definition is the one cmake keeps.
set(_BUILD_CMD_ARGS)
list(APPEND _BUILD_CMD_ARGS "-DBUILD_DIR=${BUILD_PATH}")
if(DEFINED BUILD_TYPE AND NOT BUILD_TYPE STREQUAL "")
    list(APPEND _BUILD_CMD_ARGS "-DBUILD_TYPE=${BUILD_TYPE}")
endif()
if(DEFINED DEV_PHYSX)
    list(APPEND _BUILD_CMD_ARGS "-DDEV_PHYSX=${DEV_PHYSX}")
endif()
if(DEFINED DEV_SCHEMA)
    list(APPEND _BUILD_CMD_ARGS "-DDEV_SCHEMA=${DEV_SCHEMA}")
endif()
if(DEFINED OVPHYSX_USE_RELEASE_RUNTIME_DEPS)
    list(APPEND _BUILD_CMD_ARGS "-DOVPHYSX_USE_RELEASE_RUNTIME_DEPS=${OVPHYSX_USE_RELEASE_RUNTIME_DEPS}")
endif()
if(DEFINED CLEAN_BUILD AND NOT CLEAN_BUILD STREQUAL "")
    list(APPEND _BUILD_CMD_ARGS "-DCLEAN_BUILD=${CLEAN_BUILD}")
endif()
if(DEFINED CLEAN_ONLY AND NOT CLEAN_ONLY STREQUAL "")
    list(APPEND _BUILD_CMD_ARGS "-DCLEAN_ONLY=${CLEAN_ONLY}")
endif()
if(DEFINED OVPHYSX_PASSTHROUGH_ARGS AND NOT OVPHYSX_PASSTHROUGH_ARGS STREQUAL "")
    foreach(_OVPHYSX_PASS IN LISTS OVPHYSX_PASSTHROUGH_ARGS)
        # foreach unescapes \; . Escape again so this argument stays one
        # element when the command list is expanded, including a value that
        # itself contains a semicolon.
        string(REPLACE ";" "\\;" _OVPHYSX_PASS_ESCAPED "${_OVPHYSX_PASS}")
        list(APPEND _BUILD_CMD_ARGS "${_OVPHYSX_PASS_ESCAPED}")
    endforeach()
endif()
list(APPEND _BUILD_CMD_ARGS -P "${SCRIPT_DIR}/build.cmake")

message(STATUS "Step 1: Building (cmake ${_BUILD_CMD_ARGS})")
message(STATUS "")
execute_process(
    COMMAND ${CMAKE_COMMAND} ${_BUILD_CMD_ARGS}
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    RESULT_VARIABLE BUILD_RESULT
)
if(NOT BUILD_RESULT EQUAL 0)
    message(FATAL_ERROR "Build failed (exit code: ${BUILD_RESULT})")
endif()

message(STATUS "")
message(STATUS "Step 1b: Python type stubs (pyright)")
execute_process(
    COMMAND ${CMAKE_COMMAND} -P "${SCRIPT_DIR}/test_pyright.cmake"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    RESULT_VARIABLE PYRIGHT_RESULT
)
if(NOT PYRIGHT_RESULT EQUAL 0)
    message(FATAL_ERROR "pyright failed (exit code: ${PYRIGHT_RESULT})")
endif()

# --- Steps 2-8: Install, wheel, and all tests via validate_all target ---

# Resolve the build configuration for multi-config generators. On Visual Studio
# generators, omitting --config defaults to Debug and can trigger an unintended
# full rebuild when the main build/install was Release.
file(STRINGS "${BUILD_PATH}/CMakeCache.txt" _CACHE_CONFIG_TYPES_LINE
    REGEX "^CMAKE_CONFIGURATION_TYPES:.*=")
if(NOT DEFINED BUILD_TYPE OR BUILD_TYPE STREQUAL "")
    file(STRINGS "${BUILD_PATH}/CMakeCache.txt" _CACHE_BUILD_TYPE_LINE
        REGEX "^CMAKE_BUILD_TYPE:.*=")
    if(_CACHE_BUILD_TYPE_LINE)
        string(REGEX REPLACE "^[^=]*=" "" BUILD_TYPE "${_CACHE_BUILD_TYPE_LINE}")
    endif()
endif()
if(NOT DEFINED BUILD_TYPE OR BUILD_TYPE STREQUAL "")
    set(BUILD_TYPE "Release")
endif()

# No-CUDA path: install and wheel, then the test scripts that pass with no
# driver. The validate_all CTest target is not built, so its GPU passes never
# start. A missing CUDA driver does not fail this path.
if(OVPHYSX_SKIP_GPU_PASSES)
    set(ENV{OVPHYSX_SKIP_GPU_PASSES} "ON")
    set(ENV{OVPHYSX_DISABLE_GPU} "1")
    set(_CPU_BUILD_ARGS --build "${BUILD_PATH}" --target build_wheel)
    set(_CPU_BUILD_DESC "cmake --build _build --target build_wheel")
    if(_CACHE_CONFIG_TYPES_LINE)
        list(APPEND _CPU_BUILD_ARGS --config "${BUILD_TYPE}")
        string(APPEND _CPU_BUILD_DESC " --config ${BUILD_TYPE}")
    endif()

    message(STATUS "")
    message(STATUS "Steps 2-3: Installing and building the wheel (${_CPU_BUILD_DESC})")
    message(STATUS "")
    execute_process(
        COMMAND ${CMAKE_COMMAND} ${_CPU_BUILD_ARGS}
        WORKING_DIRECTORY "${PROJECT_ROOT}"
        RESULT_VARIABLE _CPU_WHEEL_RESULT
    )
    if(NOT _CPU_WHEEL_RESULT EQUAL 0)
        message(FATAL_ERROR "build_wheel failed (exit code: ${_CPU_WHEEL_RESULT})")
    endif()

    set(_CPU_TEST_SCRIPTS
        test_cpp.cmake
        test_cpp_samples.cmake
        test_python_runtime.cmake
        test_python_wheel.cmake
        test_python_samples.cmake
        test_source_link.cmake
    )
    foreach(_CPU_TEST_SCRIPT IN LISTS _CPU_TEST_SCRIPTS)
        message(STATUS "")
        message(STATUS "Running ${_CPU_TEST_SCRIPT} (no CUDA driver)")
        message(STATUS "")
        # Same -D context the CTest custom targets pass. A bare cmake -P
        # defaults BUILD_TYPE to Release and drops the source-link toolchain.
        set(_CPU_SCRIPT_ARGS "-DBUILD_TYPE=${BUILD_TYPE}" "-DBUILD_DIR=${BUILD_PATH}")
        if(_CPU_TEST_SCRIPT STREQUAL "test_source_link.cmake")
            if(DEV_PHYSX)
                list(APPEND _CPU_SCRIPT_ARGS "-DOVPHYSX_DEV_PHYSX=ON")
            else()
                list(APPEND _CPU_SCRIPT_ARGS "-DOVPHYSX_DEV_PHYSX=OFF")
            endif()
            list(APPEND _CPU_SCRIPT_ARGS
                "-DSOURCE_LINK_CONTEXT_FILE=${BUILD_PATH}/source_link_context.cmake")
        endif()
        execute_process(
            COMMAND ${CMAKE_COMMAND} ${_CPU_SCRIPT_ARGS} -P "${SCRIPT_DIR}/${_CPU_TEST_SCRIPT}"
            WORKING_DIRECTORY "${PROJECT_ROOT}"
            RESULT_VARIABLE _CPU_TEST_RESULT
        )
        if(NOT _CPU_TEST_RESULT EQUAL 0)
            message(FATAL_ERROR "${_CPU_TEST_SCRIPT} failed (exit code: ${_CPU_TEST_RESULT})")
        endif()
    endforeach()

    message(STATUS "")
    message(STATUS "No-CUDA validation passed")
    message(STATUS "========================================")
    return()
endif()

set(_VALIDATE_ARGS --build "${BUILD_PATH}" --target validate_all)
set(_VALIDATE_DESC "cmake --build _build --target validate_all")
if(_CACHE_CONFIG_TYPES_LINE)
    list(APPEND _VALIDATE_ARGS --config "${BUILD_TYPE}")
    string(APPEND _VALIDATE_DESC " --config ${BUILD_TYPE}")
endif()

message(STATUS "")
message(STATUS "Steps 2-8: Delegating to ${_VALIDATE_DESC}")
message(STATUS "")
execute_process(
    COMMAND ${CMAKE_COMMAND} ${_VALIDATE_ARGS}
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    RESULT_VARIABLE VALIDATE_RESULT
)
if(NOT VALIDATE_RESULT EQUAL 0)
    message(FATAL_ERROR "validate_all failed (exit code: ${VALIDATE_RESULT})")
endif()
message(STATUS "")
message(STATUS "All validation passed!")
message(STATUS "========================================")

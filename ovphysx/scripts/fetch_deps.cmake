# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-PYTESTUSD-001
# @covers AC-2

# Fetch packman dependencies for ovphysx (cross-platform).
# Usage: cmake -P scripts/fetch_deps.cmake [options]
#
# Options (passed via -D flags or environment variables):
#   -DPLATFORM=<platform>    Platform target ABI (default: manylinux_2_35_x86_64 or manylinux_2_35_aarch64 on Linux by arch, windows-x86_64 or windows-arm64 on Windows)
#   -DCONFIG=<config>        Build configuration (default: release)
#
# Environment variables can also be used:
#   PLATFORM - overrides default platform (sets platform_target_abi token)
#   CONFIG - overrides default config

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
get_filename_component(PROJECT_ROOT "${SCRIPT_DIR}/.." ABSOLUTE)

# Provides the arch/platform detection for the default PLATFORM and HOST_PLATFORM.
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")

# The default platform reuses the wheel tag, mapped to packman tokens.
if(NOT DEFINED PLATFORM)
    if(DEFINED ENV{PLATFORM})
        set(PLATFORM "$ENV{PLATFORM}")
    else()
        get_wheel_platform_tag()
        # Packman uses windows-x86_64 / windows-arm64 where the wheel tag is win_amd64 / win_arm64.
        if(WHEEL_PLAT_NAME STREQUAL "win_amd64")
            set(PLATFORM "windows-x86_64")
        elseif(WHEEL_PLAT_NAME STREQUAL "win_arm64")
            set(PLATFORM "windows-arm64")
        else()
            set(PLATFORM "${WHEEL_PLAT_NAME}")
        endif()
    endif()
endif()

# The VS Developer Command Prompt sets PLATFORM=x64.
if(WIN32 AND PLATFORM STREQUAL "x64")
    message(STATUS "Normalizing PLATFORM from 'x64' to 'windows-x86_64'")
    set(PLATFORM "windows-x86_64")
endif()

# CI runners or packman may set PLATFORM to the OS name (e.g. "Linux").
if(NOT WIN32 AND (PLATFORM STREQUAL "Linux" OR PLATFORM STREQUAL "linux"))
    if(ARCH_NAME STREQUAL "aarch64")
        message(STATUS "Normalizing PLATFORM from '${PLATFORM}' to 'manylinux_2_35_aarch64'")
        set(PLATFORM "manylinux_2_35_aarch64")
    else()
        message(STATUS "Normalizing PLATFORM from '${PLATFORM}' to 'manylinux_2_35_x86_64'")
        set(PLATFORM "manylinux_2_35_x86_64")
    endif()
endif()

if(NOT DEFINED CONFIG)
    if(DEFINED ENV{CONFIG})
        set(CONFIG "$ENV{CONFIG}")
    else()
        set(CONFIG "release")
    endif()
endif()

message(STATUS "Fetching ovphysx packman dependencies...")
message(STATUS "Project root: ${PROJECT_ROOT}")
message(STATUS "Platform: ${PLATFORM}")
message(STATUS "Packman config: ${CONFIG}")
message(STATUS "USD mode: namespaced")
message(STATUS "Static Carbonite deps: ON")

if(WIN32)
    set(PACKMAN_CMD "${PROJECT_ROOT}/tools/packman/packman.cmd")
else()
    set(PACKMAN_CMD "${PROJECT_ROOT}/tools/packman/packman")
endif()

# Host toolchains use the standard platform names.
if(WIN32)
    if(ARCH_NAME STREQUAL "aarch64")
        set(HOST_PLATFORM "windows-arm64")
    else()
        set(HOST_PLATFORM "windows-x86_64")
    endif()
else()
    set(HOST_PLATFORM "${PLATFORM_NAME}")
endif()

# Common packman arguments for target dependencies.
# -t sets token values. -p is shorthand for -t platform=${PLATFORM}.
set(PACKMAN_ARGS pull -t platform_target_abi=${PLATFORM} -t config=${CONFIG})

# Host deps use the standard platform flag, not platform_target_abi.
set(PACKMAN_ARGS_HOST pull -p ${HOST_PLATFORM})

# ============================================================================
# Host dependencies (MSVC and WinSDK on Windows, Ninja on all platforms)
# ============================================================================
message(STATUS "Downloading host dependencies...")
execute_process(
    COMMAND ${PACKMAN_CMD} ${PACKMAN_ARGS_HOST} "${PROJECT_ROOT}/deps/host-deps.packman.xml"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    RESULT_VARIABLE HOST_RESULT
    OUTPUT_VARIABLE HOST_OUTPUT
    ERROR_VARIABLE HOST_ERROR
)
if(HOST_OUTPUT)
    message(STATUS "${HOST_OUTPUT}")
endif()
if(NOT HOST_RESULT EQUAL 0)
    if(WIN32)
        # Windows needs MSVC, WinSDK and Ninja from here, so the failure is fatal.
        message(FATAL_ERROR "Failed to download host dependencies (exit code: ${HOST_RESULT})\n${HOST_ERROR}")
    else()
        # The MSVC/WinSDK packages do not exist for Linux. Packman may warn, but Ninja still resolves.
        message(STATUS "Note: Some host packages not available for Linux (MSVC/WinSDK) - this is expected")
        if(HOST_ERROR)
            message(STATUS "Packman stderr: ${HOST_ERROR}")
        endif()
    endif()
endif()

# Verify host dependencies. The public source drop does not carry the
# non-redistributable MSVC/WinSDK packages. Without them the build falls back
# to a local Visual Studio installation (see scripts/build.cmake).
if(WIN32)
    if(EXISTS "${PROJECT_ROOT}/_build/host-deps/msvc")
        if(NOT EXISTS "${PROJECT_ROOT}/_build/host-deps/msvc/VC/Auxiliary/Build/vcvars64.bat")
            message(FATAL_ERROR "MSVC not properly installed - vcvars64.bat not found")
        endif()
        message(STATUS "Verified: MSVC host toolchain")
    else()
        message(STATUS "Packaged MSVC not present - the build will use a local Visual Studio installation")
    endif()
endif()

# ============================================================================
# Target dependencies (omni_physics, USD, CUDA, etc.)
# ============================================================================

# Pull one packman manifest and verify the junctions it should create.
# expected_junctions can be a single value or a semicolon-separated list.
# Extra packman args (e.g. an additional -t token) may be passed after the
# required arguments and are appended to PACKMAN_ARGS for this pull only.
function(pull_dependency dep_file description expected_junctions)
    message(STATUS "Downloading ${description}...")
    execute_process(
        COMMAND ${PACKMAN_CMD} ${PACKMAN_ARGS} ${ARGN} "${PROJECT_ROOT}/deps/${dep_file}"
        WORKING_DIRECTORY "${PROJECT_ROOT}"
        RESULT_VARIABLE RESULT
        OUTPUT_VARIABLE OUTPUT
        ERROR_VARIABLE ERROR
    )
    
    if(OUTPUT)
        message(STATUS "${OUTPUT}")
    endif()
    if(ERROR)
        message(WARNING "Packman stderr: ${ERROR}")
    endif()
    
    if(NOT RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to download ${description} (exit code: ${RESULT})")
    endif()
    
    # Packman can report success without creating the junction.
    foreach(expected_junction IN LISTS expected_junctions)
        set(JUNCTION_PATH "${PROJECT_ROOT}/_build/target-deps/${expected_junction}")
        if(NOT EXISTS "${JUNCTION_PATH}")
            message(FATAL_ERROR "Packman reported success but junction was not created: ${JUNCTION_PATH}\n"
                                "This may indicate a permissions issue or packman configuration problem.\n"
                                "Try running packman manually:\n"
                                "  ${PACKMAN_CMD} ${PACKMAN_ARGS} ${ARGN} ${PROJECT_ROOT}/deps/${dep_file}")
        endif()
        message(STATUS "Verified junction exists: ${JUNCTION_PATH}")
    endforeach()
endfunction()

# omni_physics headers, sourced from ovphysx/ovruntime.
pull_dependency("target-deps.packman.xml" "target dependencies (omni_physics)" "omni_physics")

# The ovruntime_deps package. The gsl import below reads a manifest inside it, so
# it must be pulled first. It is pulled here rather than only in build.cmake so a
# standalone fetch_deps run works.
pull_dependency("../ovruntime/deps/ovruntime-deps.packman.xml" "ovruntime_deps package" "")

pull_dependency("carb-sdk-static.packman.xml" "carbonite static libs" "carb_sdk_static")
pull_dependency("carb-sdk-deps-import.packman.xml" "carbonite build dependencies (python)" "python")

# platform_target and usd_ver are placeholders for version strings of entries
# the import filters out. The ovphysx build itself pulls no USD package.
pull_dependency("ovruntime-deps-import.packman.xml" "ovruntime build dependencies (gsl)" "gsl"
                -p ${PLATFORM} -t platform_target=${PLATFORM} -t usd_ver=unused)

# ovphysx itself links no USD, and the python tests resolve python USD from stock
# pip usd-core (REQ-PACKAGING-PYTESTUSD-001), so no internal USD monolith is fetched.

# ovstage: the self-contained wheel is fetched per platform by
# scripts/fetch_ovstage_release.py. Its ovstage/ tree is extracted into ovruntime's
# target-deps/ovstage (OVSTAGE_DIR, resolved by OvstageDependency.cmake) and the
# wheel is copied into target-deps/ovstage_wheel for the python tests. The default
# --source is internal. The open-source drop resolves the wheel from public PyPI.
if(EXISTS "${SCRIPT_DIR}/fetch_ovstage_release.py")
    get_filename_component(_OVRUNTIME_TARGET_DEPS "${PROJECT_ROOT}/ovruntime/_build/target-deps" ABSOLUTE)
    # Prefer the packman-provided python, whose junction the carb-sdk-deps-import pull
    # above creates. System python3 is not guaranteed on PATH in every CI job (e.g. the
    # sonarqube job runs this script without a system interpreter).
    set(_OVSTAGE_PY "")
    foreach(_cand
            "${PROJECT_ROOT}/_build/target-deps/python/python"
            "${PROJECT_ROOT}/_build/target-deps/python/python3"
            "${PROJECT_ROOT}/_build/target-deps/python/python.exe")
        if(EXISTS "${_cand}")
            set(_OVSTAGE_PY "${_cand}")
            break()
        endif()
    endforeach()
    if(NOT _OVSTAGE_PY)
        find_program(_OVSTAGE_PY NAMES python3 python)
    endif()
    if(NOT _OVSTAGE_PY)
        message(FATAL_ERROR "ovstage release fetch needs a python interpreter "
                            "(packman python junction or system python3).")
    endif()
    message(STATUS "Fetching ovstage release (platform=${PLATFORM})...")
    execute_process(
        COMMAND "${_OVSTAGE_PY}" "${SCRIPT_DIR}/fetch_ovstage_release.py"
            --platform "${PLATFORM}"
            --dest "${_OVRUNTIME_TARGET_DEPS}/ovstage"
            --wheel-dest "${PROJECT_ROOT}/_build/target-deps/ovstage_wheel"
        RESULT_VARIABLE _OVSTAGE_FETCH_RC)
    if(NOT _OVSTAGE_FETCH_RC EQUAL 0)
        message(FATAL_ERROR "ovstage release fetch failed (scripts/fetch_ovstage_release.py). "
                            "Check network access to the ovstage package index.")
    endif()
else()
    message(FATAL_ERROR "scripts/fetch_ovstage_release.py not found. ovstage is required to "
                        "build ovruntime on this branch; ensure the open-source tree includes "
                        "ovphysx/scripts/fetch_ovstage_release.py.")
endif()

message(STATUS "Dependencies downloaded successfully!")

# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-BENCHMARK-002
# @covers AC-4
# @implements REQ-PACKAGING-CLOSURE-001
# @covers AC-2

cmake_minimum_required(VERSION 3.16)

# Checked first so an old Windows CMake fails here, not downstream.
include("${CMAKE_CURRENT_LIST_DIR}/cmake/RequireCMakeVersion.cmake")
ovphysx_require_cmake_version()

if(POLICY CMP0091)
    cmake_policy(SET CMP0091 NEW)
endif()

# Read version from single source of truth
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" OVPHYSX_VERSION)
string(STRIP "${OVPHYSX_VERSION}" OVPHYSX_VERSION)

# Parse version components (supports semver with pre-release: X.Y.Z or X.Y.Z-prerelease)
string(REGEX MATCH "^([0-9]+)\\.([0-9]+)\\.([0-9]+)(-(.+))?" _ "${OVPHYSX_VERSION}")
set(OVPHYSX_VERSION_MAJOR "${CMAKE_MATCH_1}")
set(OVPHYSX_VERSION_MINOR "${CMAKE_MATCH_2}")
set(OVPHYSX_VERSION_PATCH "${CMAKE_MATCH_3}")
set(OVPHYSX_VERSION_PRERELEASE "${CMAKE_MATCH_5}")

# CMake project() VERSION only accepts X.Y.Z, so use just the numeric part
set(OVPHYSX_VERSION_NUMERIC "${OVPHYSX_VERSION_MAJOR}.${OVPHYSX_VERSION_MINOR}.${OVPHYSX_VERSION_PATCH}")

project(ovphysx
    VERSION ${OVPHYSX_VERSION_NUMERIC}
    DESCRIPTION "ovphysx C API and C++ wrappers"
    LANGUAGES C CXX)

# Detect whether ovphysx is the top-level project or included via add_subdirectory().
# CMake 3.21+ provides PROJECT_IS_TOP_LEVEL. This form works with 3.16+.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(OVPHYSX_IS_TOP_LEVEL TRUE)
else()
    set(OVPHYSX_IS_TOP_LEVEL FALSE)
endif()

if(OVPHYSX_IS_TOP_LEVEL)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/scripts/crossplatform_helpers.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/CarbPlatform.cmake")

# -Werror is on in CI builds, which use the baseline compiler where all warnings
# are clean. A host build with a newer compiler keeps warnings visible but non-fatal.
# CI is detected via the provider-agnostic CI env var or the OMNI_REPO_BUILD_NO_DOCKER flag.
if(DEFINED ENV{CI} OR DEFINED ENV{OMNI_REPO_BUILD_NO_DOCKER})
    option(OVPHYSX_WARNINGS_AS_ERRORS "Treat warnings as errors" ON)
else()
    option(OVPHYSX_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
endif()
if(OVPHYSX_WARNINGS_AS_ERRORS)
    message(STATUS "Warnings as errors: ENABLED")
else()
    message(STATUS "Warnings as errors: DISABLED (pass -DOVPHYSX_WARNINGS_AS_ERRORS=ON to enable)")
endif()

# NOTE: CompilerDefaults is NOT included at the top level because
# add_compile_options() is directory-scoped and would leak into ovruntime.
# ovruntime includes CompilerDefaults itself. ovphysx_apply_compiler_defaults()
# below applies the same flags per-target.

# NOTE: Compile definitions matching ovruntime (required by USD/TBB headers and
# ABI compat) are applied per-target after add_library() to avoid leaking into
# parent projects when ovphysx is included via add_subdirectory().

# Per-target equivalent of CompilerDefaults.cmake + common warning suppressions.
# Call ovphysx_apply_compiler_defaults(<target> [GCC_SUPPRESSIONS ...] [MSVC_SUPPRESSIONS ...])
# on each ovphysx target instead of using directory-scoped add_compile_options().
function(ovphysx_apply_compiler_defaults _target)
    cmake_parse_arguments(ARG "" "" "GCC_SUPPRESSIONS;MSVC_SUPPRESSIONS" ${ARGN})
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${_target} PRIVATE
            # Core flags (from CompilerDefaults.cmake)
            $<$<COMPILE_LANGUAGE:C,CXX>:-Wall>
            $<$<COMPILE_LANGUAGE:C,CXX>:-Wextra>
            $<$<COMPILE_LANGUAGE:C,CXX>:-fvisibility=hidden>
            $<$<COMPILE_LANGUAGE:C,CXX>:-fstack-protector-strong>
            $<$<COMPILE_LANGUAGE:C,CXX>:-pthread>
            # Common suppressions for ovphysx code (triggered by USD/PhysX/Carbonite headers)
            $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-unused-parameter>
            $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-unused-variable>
            $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-unused-but-set-variable>
            $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-unused-function>
            $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-deprecated-declarations>
            $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-deprecated-copy>
            $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-deprecated>
            $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-missing-field-initializers>
            ${ARG_GCC_SUPPRESSIONS}
        )
        # GCC 12+ emits false-positive warnings in inlined/instantiated code from
        # PhysX/USD/TBB headers that older GCC (CI Docker toolchain) does not.
        # SYSTEM includes alone are insufficient: GCC attributes warnings from
        # inlined SYSTEM-header code to the call site in user code.
        # Mirrors the version-gated block in ovruntime/CMakeLists.txt.
        if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND
           CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "12")
            target_compile_options(${_target} PRIVATE
                $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-array-bounds>
                $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-stringop-overread>
                $<$<COMPILE_LANGUAGE:CXX>:-Wno-dangling-reference>
            )
        endif()
        if(OVPHYSX_WARNINGS_AS_ERRORS)
            target_compile_options(${_target} PRIVATE
                $<$<COMPILE_LANGUAGE:C,CXX>:-Werror>)
        endif()
    elseif(MSVC)
        # Suppressions aligned with the ovruntime/source/pch/UsdPCH.h pragma block.
        # ovruntime scopes these via #pragma warning(push/pop) around USD/PhysX
        # includes in its precompiled header. ovphysx has no PCH, so they are
        # applied per-target instead.
        target_compile_options(${_target} PRIVATE
            $<$<COMPILE_LANGUAGE:C,CXX>:/W4>
            # From UsdPCH.h (third-party USD/PhysX header warnings)
            $<$<COMPILE_LANGUAGE:C,CXX>:/wd4003>   # not enough args for function-like macro
            $<$<COMPILE_LANGUAGE:C,CXX>:/wd4005>   # NOMINMAX macro redefinition
            $<$<COMPILE_LANGUAGE:C,CXX>:/wd4244>   # conversion from double to float / int to float
            $<$<COMPILE_LANGUAGE:C,CXX>:/wd4267>   # conversion from size_t to int
            $<$<COMPILE_LANGUAGE:C,CXX>:/wd4305>   # argument truncation from double to float
            $<$<COMPILE_LANGUAGE:C,CXX>:/wd4800>   # int to bool
            $<$<COMPILE_LANGUAGE:C,CXX>:/wd4996>   # call to std::copy with unsafe params
            # Additional third-party header warnings (Carbonite, Fabric, omni_physics)
            $<$<COMPILE_LANGUAGE:C,CXX>:/wd4099>   # class/struct mismatch
            $<$<COMPILE_LANGUAGE:C,CXX>:/wd4127>   # conditional expression is constant
            $<$<COMPILE_LANGUAGE:C,CXX>:/wd4201>   # nameless struct/union
            $<$<COMPILE_LANGUAGE:C,CXX>:/wd4459>   # declaration hides global
            # Common ovruntime per-file suppressions (see vcd.cpp, Volume.h, etc.)
            $<$<COMPILE_LANGUAGE:C,CXX>:/wd4100>   # unreferenced formal parameter
            $<$<COMPILE_LANGUAGE:C,CXX>:/wd4189>   # local variable initialized but not referenced
            $<$<COMPILE_LANGUAGE:C,CXX>:/wd4505>   # unreferenced local function removed
            ${ARG_MSVC_SUPPRESSIONS}
        )
        if(OVPHYSX_WARNINGS_AS_ERRORS)
            target_compile_options(${_target} PRIVATE
                $<$<COMPILE_LANGUAGE:C,CXX>:/WX>)
        endif()
    endif()
endfunction()

# Configure Python from target-deps (packaged by packman)
# This packaged Python is used for USD's boost::python headers and linking
if(NOT DEFINED LOCAL_TARGET_DEPS)
    set(LOCAL_TARGET_DEPS "${CMAKE_CURRENT_SOURCE_DIR}/_build/target-deps")
endif()
if(EXISTS "${LOCAL_TARGET_DEPS}/python")
    # Set Python3_ROOT_DIR so any USD-triggered find_package(Python3) finds the packaged Python.
    # Only force these when top-level to avoid overriding a parent project's Python config.
    if(OVPHYSX_IS_TOP_LEVEL)
        set(Python3_ROOT_DIR "${LOCAL_TARGET_DEPS}/python" CACHE PATH "Python root directory" FORCE)
        set(Python3_FIND_STRATEGY LOCATION CACHE STRING "Python find strategy" FORCE)
        set(Python3_FIND_REGISTRY NEVER CACHE STRING "Disable Python registry search" FORCE)
    endif()
    
    # Discover Python include directories by globbing (packaged Python may not have standard layout)
    file(GLOB PYTHON_VERSION_DIRS "${LOCAL_TARGET_DEPS}/python/include/python*")
    if(PYTHON_VERSION_DIRS)
        list(GET PYTHON_VERSION_DIRS 0 PYTHON_VERSION_DIR)
        set(PYTHON_INCLUDE_DIRS "${LOCAL_TARGET_DEPS}/python/include" "${PYTHON_VERSION_DIR}")
    else()
        set(PYTHON_INCLUDE_DIRS "${LOCAL_TARGET_DEPS}/python/include")
    endif()
    message(STATUS "Using target-deps Python: ${LOCAL_TARGET_DEPS}/python")
    message(STATUS "Python include directories: ${PYTHON_INCLUDE_DIRS}")
endif()

# Standardized output directory structure: _build/<platform>/<config>
# The binary directory is normalized first so "<platform>/<config>" is not
# appended again when the user already configured the build inside it.

if(CMAKE_CONFIGURATION_TYPES)
    # Multi-config generators manage the per-config subdirectories internally.
    set(PLATFORM_BUILD_DIR "${CMAKE_BINARY_DIR}/${PLATFORM_NAME}")
    set(OUTPUT_DIR "${PLATFORM_BUILD_DIR}/$<LOWER_CASE:$<CONFIG>>")
    # BUILD_TYPE_LOWER is needed at configure time to locate packman dependencies
    # (carb_sdk_static, usd/<config>, etc.). build.cmake passes CMAKE_BUILD_TYPE
    # even for multi-config generators to identify the fetched packman config.
    if(CMAKE_BUILD_TYPE)
        string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_LOWER)
    else()
        set(BUILD_TYPE_LOWER "release")
    endif()
else()
    if(NOT CMAKE_BUILD_TYPE AND OVPHYSX_IS_TOP_LEVEL)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
    endif()
    if(CMAKE_BUILD_TYPE)
        string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_LOWER)
    else()
        set(BUILD_TYPE_LOWER "release")
    endif()

    # Start from the user-provided binary dir and strip any trailing "<config>"
    # and "<platform>" components so we do not duplicate them.
    set(BUILD_ROOT "${CMAKE_BINARY_DIR}")

    get_filename_component(_last_component "${BUILD_ROOT}" NAME)
    if(_last_component STREQUAL "${BUILD_TYPE_LOWER}")
        get_filename_component(BUILD_ROOT "${BUILD_ROOT}" DIRECTORY)
    endif()

    get_filename_component(_last_component "${BUILD_ROOT}" NAME)
    if(_last_component STREQUAL "${PLATFORM_NAME}")
        get_filename_component(BUILD_ROOT "${BUILD_ROOT}" DIRECTORY)
    endif()

    set(PLATFORM_BUILD_DIR "${BUILD_ROOT}/${PLATFORM_NAME}")
    set(OUTPUT_DIR "${PLATFORM_BUILD_DIR}/${BUILD_TYPE_LOWER}")
endif()

set(_OVPHYSX_USE_RELEASE_RUNTIME_DEPS_DEFAULT ON)
if(NOT DEFINED OVPHYSX_USE_RELEASE_RUNTIME_DEPS
   AND DEFINED ENV{OVPHYSX_USE_RELEASE_RUNTIME_DEPS}
   AND NOT "$ENV{OVPHYSX_USE_RELEASE_RUNTIME_DEPS}" STREQUAL "")
    set(_OVPHYSX_USE_RELEASE_RUNTIME_DEPS_DEFAULT "$ENV{OVPHYSX_USE_RELEASE_RUNTIME_DEPS}")
endif()
option(OVPHYSX_USE_RELEASE_RUNTIME_DEPS
    "Use release Packman runtime dependencies even when compiling a Debug build"
    ${_OVPHYSX_USE_RELEASE_RUNTIME_DEPS_DEFAULT})
string(TOUPPER "${OVPHYSX_USE_RELEASE_RUNTIME_DEPS}" _OVPHYSX_RELEASE_RUNTIME_DEPS_VALUE)
if(_OVPHYSX_RELEASE_RUNTIME_DEPS_VALUE MATCHES "^(ON|TRUE|YES|1)$")
    set(OVPHYSX_USE_RELEASE_RUNTIME_DEPS ON CACHE BOOL
        "Use release Packman runtime dependencies even when compiling a Debug build" FORCE)
    set(OVPHYSX_RUNTIME_DEPS_CONFIG "release")
elseif(_OVPHYSX_RELEASE_RUNTIME_DEPS_VALUE MATCHES "^(OFF|FALSE|NO|0)$")
    set(OVPHYSX_USE_RELEASE_RUNTIME_DEPS OFF CACHE BOOL
        "Use release Packman runtime dependencies even when compiling a Debug build" FORCE)
    if(BUILD_TYPE_LOWER STREQUAL "debug")
        set(OVPHYSX_RUNTIME_DEPS_CONFIG "debug")
    else()
        set(OVPHYSX_RUNTIME_DEPS_CONFIG "release")
    endif()
else()
    message(FATAL_ERROR
        "OVPHYSX_USE_RELEASE_RUNTIME_DEPS must be ON or OFF; got '${OVPHYSX_USE_RELEASE_RUNTIME_DEPS}'")
endif()
if(BUILD_TYPE_LOWER STREQUAL "debug" AND NOT OVPHYSX_USE_RELEASE_RUNTIME_DEPS)
    message(FATAL_ERROR
        "True-Debug runtime dependencies are unsupported because the published "
        "OVStage package supplies a Release-only resolver/client runtime. "
        "Use -DOVPHYSX_USE_RELEASE_RUNTIME_DEPS=ON for Debug compilation.")
endif()
message(STATUS "Runtime dependencies config: ${OVPHYSX_RUNTIME_DEPS_CONFIG}")
unset(_OVPHYSX_USE_RELEASE_RUNTIME_DEPS_DEFAULT)
unset(_OVPHYSX_RELEASE_RUNTIME_DEPS_VALUE)

unset(_last_component)

if(OVPHYSX_IS_TOP_LEVEL)
    # Set all output directories
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_DIR})

    # Also set per-config directories for multi-config generators (lowercase)
    foreach(CONFIG ${CMAKE_CONFIGURATION_TYPES})
        string(TOUPPER ${CONFIG} CONFIG_UPPER)
        string(TOLOWER ${CONFIG} CONFIG_LOWER)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIG_UPPER} ${PLATFORM_BUILD_DIR}/${CONFIG_LOWER})
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIG_UPPER} ${PLATFORM_BUILD_DIR}/${CONFIG_LOWER})
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIG_UPPER} ${PLATFORM_BUILD_DIR}/${CONFIG_LOWER})
    endforeach()
else()
    # When included via add_subdirectory(), derive OUTPUT_DIR for the internal sidecar
    # without overriding the parent project's output directories.
    if(CMAKE_LIBRARY_OUTPUT_DIRECTORY)
        set(OUTPUT_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
    else()
        set(OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
    endif()
endif()

message(STATUS "Output directory: ${OUTPUT_DIR}")

# User-provided path to the folder that contains kit binary and PhysX plugins
set(OVPHYSX_BIN_DIR "" CACHE PATH "Path to folder containing kit executable and PhysX plugins")

# Headers
set(OVPHYSX_PUBLIC_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Generate version.h into build directory (ensures recompilation when VERSION changes)
set(OVPHYSX_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/include")
file(MAKE_DIRECTORY "${OVPHYSX_GENERATED_INCLUDE_DIR}/ovphysx")
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/include/ovphysx/version.h.in"
    "${OVPHYSX_GENERATED_INCLUDE_DIR}/ovphysx/version.h"
    @ONLY
)

# Generate Windows version resource (embeds version info into DLL)
if(WIN32)
    set(OVPHYSX_GENERATED_RC "${CMAKE_CURRENT_BINARY_DIR}/generated/ovphysx.rc")
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/src/ovphysx/ovphysx.rc.in"
        "${OVPHYSX_GENERATED_RC}"
        @ONLY
    )
endif()

# ============================================================================
# ovruntime as CMake subproject
# ============================================================================
option(OVPHYSX_DEV_PHYSX "Build PhysX SDK from source (passed to ovruntime)" OFF)
option(OVPHYSX_DEV_SCHEMA "Use locally-built physics schema (passed to ovruntime)" OFF)

set(OVRUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ovruntime")

# Ant.usda lives in omni.physx test data and is copied at configure time rather
# than checked in twice. The generated copy (gitignored) is what the
# c_unittests load and what install/build_wheel ship into samples/data.
configure_file(
    "${OVRUNTIME_DIR}/source/omni.physx/tests/test.unit/physics/data/Ant.usda"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/Ant.usda"
    COPYONLY)

# Configure ovruntime options.
# Only set defaults if the parent project hasn't already defined them.
# Uses get_property(... SET) for CMake 3.16 compatibility (DEFINED CACHE{} needs 3.21).
get_property(_has_ovr_tests CACHE OVRUNTIME_ENABLE_TESTS PROPERTY VALUE SET)
if(NOT _has_ovr_tests)
    set(OVRUNTIME_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
endif()
# @implements REQ-PACKAGING-USDFREE-001
# @covers AC-1
# OvruntimePhysX is USD-free in its only configuration (ADR-0027): ovphysx's USD-scene
# loading goes through ovstage population (docs/developer_guide.md#remote-usd-loading)
# and libovphysx has no USD link-time dependency. USD is still needed to BUILD and RUN
# the tests and samples. The product payload resolves its USD from the application's
# ovstage runtime (see scripts/package_deps.py).
# DEV_PHYSX/DEV_SCHEMA must stay in sync with the OVPHYSX_ options on every
# reconfigure when top-level, but respect parent overrides when subdirectory.
get_property(_has_ovr_devphysx CACHE OVRUNTIME_DEV_PHYSX PROPERTY VALUE SET)
if(OVPHYSX_IS_TOP_LEVEL OR NOT _has_ovr_devphysx)
    set(OVRUNTIME_DEV_PHYSX ${OVPHYSX_DEV_PHYSX} CACHE BOOL "" FORCE)
endif()
get_property(_has_ovr_devschema CACHE OVRUNTIME_DEV_SCHEMA PROPERTY VALUE SET)
if(OVPHYSX_IS_TOP_LEVEL OR NOT _has_ovr_devschema)
    set(OVRUNTIME_DEV_SCHEMA ${OVPHYSX_DEV_SCHEMA} CACHE BOOL "" FORCE)
endif()
set(OVRUNTIME_RUNTIME_DEPS_CONFIG "${OVPHYSX_RUNTIME_DEPS_CONFIG}" CACHE STRING "" FORCE)

# ============================================================================
# Auto-fetch dependencies at configure time (for add_subdirectory users)
# ============================================================================
option(OVPHYSX_FETCH_DEPS "Automatically fetch dependencies via packman" ON)

if(OVPHYSX_FETCH_DEPS)
    set(_ovphysx_required_target_deps
        "${LOCAL_TARGET_DEPS}/carb_sdk_static"
        "${LOCAL_TARGET_DEPS}/python")
    set(_ovphysx_fetch_target_deps OFF)
    foreach(_ovphysx_required_dep IN LISTS _ovphysx_required_target_deps)
        if(NOT EXISTS "${_ovphysx_required_dep}")
            set(_ovphysx_fetch_target_deps ON)
        endif()
    endforeach()
    if(_ovphysx_fetch_target_deps)
        message(STATUS "ovphysx: Fetching dependencies via packman...")
        execute_process(
            COMMAND ${CMAKE_COMMAND} -DCONFIG=${OVPHYSX_RUNTIME_DEPS_CONFIG}
                -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/fetch_deps.cmake"
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            RESULT_VARIABLE _fetch_result)
        if(NOT _fetch_result EQUAL 0)
            message(FATAL_ERROR "ovphysx: Failed to fetch dependencies (exit: ${_fetch_result})")
        endif()
    endif()
    set(_ovphysx_required_ovruntime_deps
        "${OVRUNTIME_DIR}/_build/target-deps/physx/include")
    set(_ovphysx_fetch_ovruntime_deps OFF)
    foreach(_ovphysx_required_ovruntime_dep IN LISTS _ovphysx_required_ovruntime_deps)
        if(NOT EXISTS "${_ovphysx_required_ovruntime_dep}")
            set(_ovphysx_fetch_ovruntime_deps ON)
        endif()
    endforeach()
    if(_ovphysx_fetch_ovruntime_deps)
        message(STATUS "ovphysx: Fetching ovruntime dependencies...")
        if(WIN32)
            set(_pull_cmd "${OVRUNTIME_DIR}/pull_dependencies.bat")
        else()
            set(_pull_cmd "${OVRUNTIME_DIR}/pull_dependencies.sh")
        endif()
        execute_process(
            COMMAND "${_pull_cmd}" --config "${OVPHYSX_RUNTIME_DEPS_CONFIG}"
            WORKING_DIRECTORY "${OVRUNTIME_DIR}"
            RESULT_VARIABLE _fetch_ovr_result)
        if(NOT _fetch_ovr_result EQUAL 0)
            message(FATAL_ERROR "ovphysx: Failed to fetch ovruntime deps (exit: ${_fetch_ovr_result})")
        endif()
    endif()
endif()

# Output directories: follow ovruntime conventions
# NvidiaBuildOptions sets CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE = PX_OUTPUT_LIB_DIR/release
# Only force these when top-level to avoid overriding a parent project's PhysX output dirs.
get_property(_has_px_lib CACHE PX_OUTPUT_LIB_DIR PROPERTY VALUE SET)
if(NOT _has_px_lib)
    set(PX_OUTPUT_LIB_DIR "${CMAKE_BINARY_DIR}" CACHE PATH "" FORCE)
endif()
get_property(_has_px_bin CACHE PX_OUTPUT_BIN_DIR PROPERTY VALUE SET)
if(NOT _has_px_bin)
    set(PX_OUTPUT_BIN_DIR "${CMAKE_BINARY_DIR}" CACHE PATH "" FORCE)
endif()

# Pre-define "checked" config flags (in case devphysx remaps Release->checked)
if(OVPHYSX_IS_TOP_LEVEL AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_C_FLAGS_CHECKED "${CMAKE_C_FLAGS_RELEASE}" CACHE STRING "" FORCE)
    set(CMAKE_CXX_FLAGS_CHECKED "${CMAKE_CXX_FLAGS_RELEASE}" CACHE STRING "" FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_CHECKED "${CMAKE_EXE_LINKER_FLAGS_RELEASE}" CACHE STRING "" FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_CHECKED "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}" CACHE STRING "" FORCE)
    set(CMAKE_STATIC_LINKER_FLAGS_CHECKED "${CMAKE_STATIC_LINKER_FLAGS_RELEASE}" CACHE STRING "" FORCE)
    set(CMAKE_MODULE_LINKER_FLAGS_CHECKED "${CMAKE_MODULE_LINKER_FLAGS_RELEASE}" CACHE STRING "" FORCE)
endif()

# Configure benchmark CUDA in the common parent scope. FindCUDAToolkit imported
# targets are directory-scoped: discovery inside the ovruntime child does not
# make CUDA::cudart_static visible to the sibling tests/benchmarks directory.
option(OVPHYSX_BUILD_BENCHMARKS "Build the ovphysx benchmark suite (opt-in)" OFF)
option(OVPHYSX_REQUIRE_BENCHMARK_CUDA
    "Fail configuration when the benchmark suite cannot use native CUDA tensors" OFF)
if(OVPHYSX_REQUIRE_BENCHMARK_CUDA AND NOT OVPHYSX_BUILD_BENCHMARKS)
    message(FATAL_ERROR
        "OVPHYSX_REQUIRE_BENCHMARK_CUDA=ON requires OVPHYSX_BUILD_BENCHMARKS=ON.")
endif()
if(OVPHYSX_BUILD_BENCHMARKS)
    if(DEFINED CUDAToolkit_ROOT AND NOT "${CUDAToolkit_ROOT}" STREQUAL "")
        get_filename_component(_OVPHYSX_BENCHMARK_CUDA_ROOT
            "${CUDAToolkit_ROOT}" ABSOLUTE)
    elseif(DEFINED CUDA_DIR AND NOT "${CUDA_DIR}" STREQUAL "")
        get_filename_component(_OVPHYSX_BENCHMARK_CUDA_ROOT "${CUDA_DIR}" ABSOLUTE)
    else()
        get_filename_component(_OVPHYSX_BENCHMARK_CUDA_ROOT
            "${OVRUNTIME_DIR}/_build/target-deps/cuda" ABSOLUTE)
    endif()

    # Pin toolkit discovery before validating the tree. Otherwise an incomplete
    # staged package can silently fall through to a host CUDA installation.
    set(CUDAToolkit_ROOT "${_OVPHYSX_BENCHMARK_CUDA_ROOT}" CACHE PATH
        "CUDA toolkit used by ovphysx benchmarks" FORCE)
    set(_OVPHYSX_BENCHMARK_CUDA_HEADER
        "${_OVPHYSX_BENCHMARK_CUDA_ROOT}/include/cuda_runtime_api.h")
    if(WIN32)
        set(_OVPHYSX_BENCHMARK_CUDA_NVCC
            "${_OVPHYSX_BENCHMARK_CUDA_ROOT}/bin/nvcc.exe")
        set(_OVPHYSX_BENCHMARK_CUDA_STATIC_RT
            "${_OVPHYSX_BENCHMARK_CUDA_ROOT}/lib/x64/cudart_static.lib")
    else()
        set(_OVPHYSX_BENCHMARK_CUDA_NVCC
            "${_OVPHYSX_BENCHMARK_CUDA_ROOT}/bin/nvcc")
        set(_OVPHYSX_BENCHMARK_CUDA_STATIC_RT
            "${_OVPHYSX_BENCHMARK_CUDA_ROOT}/lib64/libcudart_static.a")
    endif()
    set(_OVPHYSX_BENCHMARK_CUDA_STAGED_OK FALSE)
    if(EXISTS "${_OVPHYSX_BENCHMARK_CUDA_NVCC}" AND
       EXISTS "${_OVPHYSX_BENCHMARK_CUDA_HEADER}" AND
       EXISTS "${_OVPHYSX_BENCHMARK_CUDA_STATIC_RT}")
        set(_OVPHYSX_BENCHMARK_CUDA_STAGED_OK TRUE)
    elseif(OVPHYSX_REQUIRE_BENCHMARK_CUDA)
        message(FATAL_ERROR
            "OVPHYSX_REQUIRE_BENCHMARK_CUDA=ON requires one coherent CUDA "
            "toolkit at ${_OVPHYSX_BENCHMARK_CUDA_ROOT} containing:\n"
            "  ${_OVPHYSX_BENCHMARK_CUDA_NVCC}\n"
            "  ${_OVPHYSX_BENCHMARK_CUDA_HEADER}\n"
            "  ${_OVPHYSX_BENCHMARK_CUDA_STATIC_RT}\n"
            "Refusing to fall back to a host CUDA installation.")
    endif()

    # Match ovruntime's dev-PhysX discovery sequence on Linux. Enabling the
    # language gives FindCUDAToolkit the packaged nvcc/toolkit metadata it needs.
    # Windows benchmarks only need the imported runtime target; avoiding
    # enable_language(CUDA) also keeps the Visual Studio path independent of a
    # machine-installed CUDA integration.
    set(_OVPHYSX_BENCHMARK_CUDA_DISCOVERY_OK ${_OVPHYSX_BENCHMARK_CUDA_STAGED_OK})
    if(NOT WIN32 AND _OVPHYSX_BENCHMARK_CUDA_STAGED_OK)
        include(CheckLanguage)
        get_property(_OVPHYSX_ENABLED_LANGUAGES GLOBAL PROPERTY ENABLED_LANGUAGES)
        if(NOT "CUDA" IN_LIST _OVPHYSX_ENABLED_LANGUAGES)
            # CUDACXX selects the exact staged compiler while leaving
            # CMAKE_CUDA_COMPILER undefined so check_language performs its real
            # compiler probe. Setting CMAKE_CUDA_COMPILER first would make the
            # probe a no-op and let enable_language fail even in optional mode.
            set(_OVPHYSX_HAD_CUDACXX FALSE)
            if(DEFINED ENV{CUDACXX})
                set(_OVPHYSX_HAD_CUDACXX TRUE)
                set(_OVPHYSX_SAVED_CUDACXX "$ENV{CUDACXX}")
            endif()
            set(ENV{CUDACXX} "${_OVPHYSX_BENCHMARK_CUDA_NVCC}")
            unset(CMAKE_CUDA_COMPILER CACHE)
            unset(CMAKE_CUDA_COMPILER)
            check_language(CUDA)
            if(_OVPHYSX_HAD_CUDACXX)
                set(ENV{CUDACXX} "${_OVPHYSX_SAVED_CUDACXX}")
            else()
                unset(ENV{CUDACXX})
            endif()
        endif()
        get_filename_component(_OVPHYSX_BENCHMARK_CUDA_NVCC_REAL
            "${_OVPHYSX_BENCHMARK_CUDA_NVCC}" REALPATH)
        if(CMAKE_CUDA_COMPILER)
            get_filename_component(_OVPHYSX_BENCHMARK_CUDA_COMPILER_REAL
                "${CMAKE_CUDA_COMPILER}" REALPATH)
        else()
            set(_OVPHYSX_BENCHMARK_CUDA_COMPILER_REAL "")
        endif()
        if("${_OVPHYSX_BENCHMARK_CUDA_COMPILER_REAL}" STREQUAL
           "${_OVPHYSX_BENCHMARK_CUDA_NVCC_REAL}")
            if(NOT "CUDA" IN_LIST _OVPHYSX_ENABLED_LANGUAGES)
                enable_language(CUDA)
            endif()
        else()
            set(_OVPHYSX_BENCHMARK_CUDA_DISCOVERY_OK FALSE)
            if(OVPHYSX_REQUIRE_BENCHMARK_CUDA)
                message(FATAL_ERROR
                    "Required benchmark CUDA compiler resolved to "
                    "'${CMAKE_CUDA_COMPILER}', not staged nvcc "
                    "'${_OVPHYSX_BENCHMARK_CUDA_NVCC}'. Refusing host fallback.")
            endif()
        endif()
    endif()

    if(_OVPHYSX_BENCHMARK_CUDA_DISCOVERY_OK)
        find_package(CUDAToolkit QUIET)
    endif()
    if(TARGET CUDA::cudart_static)
        add_library(ovphysx_benchmark_cuda INTERFACE)
        target_link_libraries(ovphysx_benchmark_cuda INTERFACE CUDA::cudart_static)
        message(STATUS
            "ovphysx benchmark CUDA: native tensors enabled "
            "(${CUDAToolkit_VERSION} at ${CUDAToolkit_ROOT})")
    elseif(OVPHYSX_REQUIRE_BENCHMARK_CUDA)
        message(FATAL_ERROR
            "OVPHYSX_REQUIRE_BENCHMARK_CUDA=ON, but CUDA::cudart_static was not "
            "created from ${_OVPHYSX_BENCHMARK_CUDA_ROOT}. Ensure the packaged "
            "CUDA toolkit contains nvcc, headers, and static runtime libraries.")
    else()
        message(STATUS
            "ovphysx benchmark CUDA: unavailable; building explicit CPU-only suite")
    endif()
endif()

# Include ovruntime. EXCLUDE_FROM_ALL: only explicitly depended-on targets build.
#
# RPATH note: ovphysx packaging copies .so files directly from the build tree via
# package_deps.py, so the build-tree RPATH is what ships. $ORIGIN is prepended to
# CMAKE_BUILD_RPATH so those files find siblings in the flattened _install/plugins/
# directory when loaded via dlopen on a different machine.
set(_SAVED_BUILD_RPATH "${CMAKE_BUILD_RPATH}")
set(CMAKE_BUILD_RPATH "$ORIGIN;${CMAKE_BUILD_RPATH}")
add_subdirectory("${OVRUNTIME_DIR}" "${CMAKE_BINARY_DIR}/ovruntime" EXCLUDE_FROM_ALL)
set(CMAKE_BUILD_RPATH "${_SAVED_BUILD_RPATH}")

# Resolve ovstage in the ovphysx directory scope too. ovruntime consumes the
# same dependency internally, but its CMake variables are not visible here and
# ovphysx needs the resolved paths to expose/install the SDK companion target.
if(NOT DEFINED OVRUNTIME_TARGET_DEPS)
    set(OVRUNTIME_TARGET_DEPS "${OVRUNTIME_DIR}/_build/target-deps")
endif()

# Select PhysX headers from the exact SDK configured by ovruntime. The ovphysx
# library needs them for its internal test bridge. Top-level tests and install
# rules reuse the same verified path below.
if(DEFINED PHYSX_SDK_DIR)
    set(_OVPHYSX_PHYSX_INCLUDE_SOURCE "${PHYSX_SDK_DIR}/include")
    set(_OVPHYSX_PHYSX_INCLUDE_SOURCE_KIND "PHYSX_SDK_DIR")
elseif(NOT OVRUNTIME_DEV_PHYSX AND DEFINED OVRUNTIME_TARGET_DEPS)
    set(_OVPHYSX_PHYSX_INCLUDE_SOURCE "${OVRUNTIME_TARGET_DEPS}/physx/include")
    set(_OVPHYSX_PHYSX_INCLUDE_SOURCE_KIND "OVRUNTIME_TARGET_DEPS")
else()
    message(FATAL_ERROR
        "Cannot select PhysX SDK headers for ovphysx: neither PHYSX_SDK_DIR "
        "nor the normal-build OVRUNTIME_TARGET_DEPS root is available.")
endif()
if(NOT EXISTS "${_OVPHYSX_PHYSX_INCLUDE_SOURCE}/PxScene.h")
    message(FATAL_ERROR
        "PhysX SDK headers selected from ${_OVPHYSX_PHYSX_INCLUDE_SOURCE_KIND} "
        "do not contain PxScene.h at ${_OVPHYSX_PHYSX_INCLUDE_SOURCE}.")
endif()
message(STATUS
    "ovphysx PhysX headers (${_OVPHYSX_PHYSX_INCLUDE_SOURCE_KIND}): "
    "${_OVPHYSX_PHYSX_INCLUDE_SOURCE}")

include("${OVRUNTIME_DIR}/cmake/OvstageDependency.cmake")

if(WIN32)
    set(_OVPHYSX_OVSTAGE_RUNTIME_LIBRARY_NAME "ovstage.dll")
    set(_OVPHYSX_OVSTAGE_RESOLVER_LIBRARY_NAME "omni_usd_resolver.dll")
else()
    set(_OVPHYSX_OVSTAGE_RUNTIME_LIBRARY_NAME "libovstage.so")
    set(_OVPHYSX_OVSTAGE_RESOLVER_LIBRARY_NAME "libomni_usd_resolver.so")
endif()

# Select the runtime tree only from the exact OVStage root resolved above. In
# particular, do not fall back to another target-deps directory: doing so could
# compile/link against one OVStage package while validating another package's
# resolver payload.
set(_OVPHYSX_OVSTAGE_PYTHON_RUNTIME_DIR "")
foreach(_OVPHYSX_OVSTAGE_RUNTIME_CANDIDATE
    "${OVSTAGE_LIBRARY_DIR}"
    "${OVSTAGE_DIR}/bin"
    "${OVSTAGE_DIR}/ovstage/bin")
    if(EXISTS "${_OVPHYSX_OVSTAGE_RUNTIME_CANDIDATE}/${_OVPHYSX_OVSTAGE_RUNTIME_LIBRARY_NAME}" AND
       EXISTS "${_OVPHYSX_OVSTAGE_RUNTIME_CANDIDATE}/plugins/omni.usd_resolver/${_OVPHYSX_OVSTAGE_RESOLVER_LIBRARY_NAME}" AND
       EXISTS "${_OVPHYSX_OVSTAGE_RUNTIME_CANDIDATE}/plugins/omni.usd_resolver/usd/omni_usd_resolver/resources/plugInfo.json")
        set(_OVPHYSX_OVSTAGE_PYTHON_RUNTIME_DIR "${_OVPHYSX_OVSTAGE_RUNTIME_CANDIDATE}")
        break()
    endif()
endforeach()
if(NOT _OVPHYSX_OVSTAGE_PYTHON_RUNTIME_DIR)
    message(FATAL_ERROR
        "The selected OVStage root '${OVSTAGE_DIR}' does not contain a complete "
        "Release runtime with ovstage, omni.usd_resolver, and the resolver "
        "plugInfo registry. The runtime "
        "must come from the same exact OVStage provider used for headers/linking.")
endif()

set(_OVPHYSX_OVSTAGE_RUNTIME "")
if(WIN32)
    set(_OVPHYSX_OVSTAGE_RUNTIME
        "${_OVPHYSX_OVSTAGE_PYTHON_RUNTIME_DIR}/${_OVPHYSX_OVSTAGE_RUNTIME_LIBRARY_NAME}")
endif()

if(NOT TARGET ovphysx_ovstage)
    add_library(ovphysx_ovstage SHARED IMPORTED GLOBAL)
    set_target_properties(ovphysx_ovstage PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${OVSTAGE_INCLUDE_DIRS}"
    )
    if(WIN32)
        set_target_properties(ovphysx_ovstage PROPERTIES
            IMPORTED_IMPLIB "${OVSTAGE_LIBRARY}"
        )
        if(_OVPHYSX_OVSTAGE_RUNTIME)
            set_target_properties(ovphysx_ovstage PROPERTIES
                IMPORTED_LOCATION "${_OVPHYSX_OVSTAGE_RUNTIME}"
            )
        endif()
    else()
        set_target_properties(ovphysx_ovstage PROPERTIES
            IMPORTED_LOCATION "${OVSTAGE_LIBRARY}"
        )
    endif()
endif()
if(NOT TARGET ovphysx::ovstage)
    add_library(ovphysx::ovstage ALIAS ovphysx_ovstage)
endif()

# Build exactly the ovruntime targets needed at runtime.
# Only add to ALL when top-level. As a subdirectory the dependency chain suffices.
set(_OVR_RUNTIME_ALL "")
if(OVPHYSX_IS_TOP_LEVEL)
    set(_OVR_RUNTIME_ALL ALL)
endif()
add_custom_target(ovruntime_runtime_libs ${_OVR_RUNTIME_ALL})
add_dependencies(ovruntime_runtime_libs
    OvruntimePhysX
    OvruntimePhysXGpu
)

add_library(ovphysx SHARED
    src/AsyncEventManager/AsyncEventManager.cpp
    src/AsyncEventManager/AsyncEventManagerC.cpp
    src/CarboniteLoader/CarboniteLoader.cpp
    src/ovphysx/ovphysx_ABI_check.cpp
    src/ovphysx/CpuFeatureCheck.cpp
    src/ovphysx/LogManager.cpp
    src/ovphysx/Nvtx.cpp
    src/ovphysx/ovphysx.cpp
    src/ovphysx/ovphysxTensorBinding.cpp
    src/ovphysx/ovphysxContactBinding.cpp
    src/ovphysx/ovphysxSdfView.cpp
    src/ovphysx/PhysXWrapper.cpp
    src/ovphysx/TensorBindingWrapper.cpp
    src/ovphysx/ovphysxPhysXInterop.cpp
    src/ovphysx/ovphysxObjectChangeCallbacks.cpp
    src/ovphysx/ovphysxContactReport.cpp
    src/ovphysx/ovphysxSceneQuery.cpp
    src/ovphysx/ovphysxSidecarLoader.cpp
    src/ovphysx/ovphysxTestHelpers.cpp
    src/ovphysx/version.cpp
    src/UsdSchemaPaths/UsdSchemaPaths.cpp
    # Windows version resource (embeds version info visible in VS Modules window)
    $<$<PLATFORM_ID:Windows>:${OVPHYSX_GENERATED_RC}>
)

# Alias target so consumers can use ovphysx::ovphysx uniformly whether via
# find_package() or add_subdirectory().
add_library(ovphysx::ovphysx ALIAS ovphysx)

# Ensure ovruntime runtime pieces are built whenever ovphysx is built.
# In top-level builds the ALL target handles this. add_subdirectory() consumers
# need the explicit dependency so that building just `ovphysx` also produces
# the static runtime and remaining Carbonite plugins it uses.
add_dependencies(ovphysx ovruntime_runtime_libs)

# Shared library versioning (SONAME on Linux)
set_target_properties(ovphysx PROPERTIES
    VERSION ${OVPHYSX_VERSION_NUMERIC}
    SOVERSION ${OVPHYSX_VERSION_MAJOR}
)

# Define export macro for Windows DLL
if(WIN32)
    target_compile_definitions(ovphysx PRIVATE OVPHYSX_EXPORTS)
endif()

ovphysx_apply_compiler_defaults(ovphysx
    GCC_SUPPRESSIONS $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-delete-non-virtual-dtor>
)

# Symbol visibility: hide internal symbols by default
# Only symbols marked with OVPHYSX_API will be exported
if(NOT WIN32)
    # Also hide inline functions and vtables for C++ code
    set_target_properties(ovphysx PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN ON
    )
endif()

# Use local target-deps downloaded via packman
set(OMNI_PHYSICS_DIR "${LOCAL_TARGET_DEPS}/omni_physics")

# Carbonite headers and static libs both come from carb_sdk_static; ovphysx ships no
# separate libcarb runtime.
set(CARB_SDK_INCLUDE_DIR "${LOCAL_TARGET_DEPS}/carb_sdk_static/include")

target_include_directories(ovphysx
    PUBLIC
        $<BUILD_INTERFACE:${OVPHYSX_GENERATED_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${OVPHYSX_PUBLIC_INCLUDE_DIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OVRUNTIME_DIR}/include
)
# The public ovphysx headers #include ovstage/ovx types directly, so the ovstage
# include dirs go on the PUBLIC build interface for ovphysx's own TUs and downstream
# consumers (c_unittests, SDK samples). The matching install interface is the shared
# include/ root, where these headers are installed below.
foreach(_OVPHYSX_OVSTAGE_INC IN LISTS OVSTAGE_INCLUDE_DIRS)
    target_include_directories(ovphysx SYSTEM PUBLIC $<BUILD_INTERFACE:${_OVPHYSX_OVSTAGE_INC}>)
endforeach()
# Third-party headers as SYSTEM to suppress their warnings.
target_include_directories(ovphysx SYSTEM PRIVATE
    ${_OVPHYSX_PHYSX_INCLUDE_SOURCE}
    ${OMNI_PHYSICS_DIR}/include
    ${CARB_SDK_INCLUDE_DIR}
    ${LOCAL_TARGET_DEPS}/gsl/include
)

# NVTX range instrumentation for Nsight Systems. NVTX 3 is header-only and ships
# inside the CUDA package ovruntime already pulls, so nothing is linked or
# redistributed. Ranges are emitted only when enabled at runtime, so this stays
# compiled in for release builds and shipped wheels.
#
# Missing headers are a hard error, unlike in ovruntime: ovphysx links the CUDA
# stubs unconditionally below, so a missing header set means a broken environment
# rather than a reduced-capability build. -DOVPHYSX_ENABLE_NVTX=OFF drops only the
# ovphysx domain. The PhysX domain follows ovruntime's own detection.
option(OVPHYSX_ENABLE_NVTX "Compile in NVTX range instrumentation for Nsight Systems" ON)
if(OVPHYSX_ENABLE_NVTX)
    # Honour an explicit -DCUDA_DIR= the way ovruntime does, falling back to the
    # staged dependency. Probing only the staged path would fail a build that
    # points CUDA elsewhere, even though the headers are present there.
    if(DEFINED CUDA_DIR)
        set(OVPHYSX_NVTX_INCLUDE_DIR "${CUDA_DIR}/include")
    else()
        set(OVPHYSX_NVTX_INCLUDE_DIR "${OVRUNTIME_TARGET_DEPS}/cuda/include")
    endif()
    if(NOT EXISTS "${OVPHYSX_NVTX_INCLUDE_DIR}/nvtx3/nvToolsExt.h")
        message(FATAL_ERROR
            "NVTX 3 headers not found at ${OVPHYSX_NVTX_INCLUDE_DIR}/nvtx3. They come from the "
            "CUDA target dependency: run ovruntime/pull_dependencies.bat (or .sh) first, or point "
            "-DCUDA_DIR= at a toolkit that provides them. "
            "To build without profiling support, pass -DOVPHYSX_ENABLE_NVTX=OFF.")
    endif()
    target_include_directories(ovphysx SYSTEM PRIVATE ${OVPHYSX_NVTX_INCLUDE_DIR})
    target_compile_definitions(ovphysx PRIVATE OVPHYSX_NVTX_ENABLED=1)
    message(STATUS "NVTX instrumentation: enabled (headers: ${OVPHYSX_NVTX_INCLUDE_DIR}/nvtx3)")
else()
    message(STATUS "NVTX instrumentation: disabled (OVPHYSX_ENABLE_NVTX=OFF)")
endif()
# Single install interface. Generated headers (version.h) are installed
# to the same include/ directory, so one entry suffices.
set_property(TARGET ovphysx APPEND PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

# Carbonite linking: static-only for the SDK path.

# ovrtx with static-carb links the static Carbonite libraries from the matching
# build config. ovphysx does the same so Windows Debug links Debug Carbonite
# objects instead of forcing the release CRT/iterator ABI.
set(CARB_STATIC_LIB_DIR "${LOCAL_TARGET_DEPS}/carb_sdk_static/_build/${PLATFORM_NAME}/${BUILD_TYPE_LOWER}")
set(CARB_STATIC_PLUGIN_NAMES
    carb.assets.static
    carb.datasource-file.static
    carb.dictionary.static
    carb.dictionary.serializer-json.static
    carb.dictionary.serializer-toml.static
    carb.eventdispatcher.static
    carb.events.static
    carb.settings.static
    carb.tasking.static
    carb.tokens.static
    carb.variant.static
)

unset(CARB_STATIC_LIBRARY)
unset(CARB_STATIC_LIBRARY CACHE)
find_library(CARB_STATIC_LIBRARY carb.static PATHS ${CARB_STATIC_LIB_DIR} NO_DEFAULT_PATH)
if(NOT CARB_STATIC_LIBRARY)
    message(FATAL_ERROR "carb.static library not found in ${CARB_STATIC_LIB_DIR}")
endif()
message(STATUS "Found carb.static: ${CARB_STATIC_LIBRARY}")

set(CARB_STATIC_PLUGIN_LIBS "")
foreach(PLUGIN_NAME ${CARB_STATIC_PLUGIN_NAMES})
    unset(_CARB_STATIC_PLUGIN_LIB)
    unset(_CARB_STATIC_PLUGIN_LIB CACHE)
    find_library(_CARB_STATIC_PLUGIN_LIB ${PLUGIN_NAME} PATHS ${CARB_STATIC_LIB_DIR} NO_DEFAULT_PATH)
    if(NOT _CARB_STATIC_PLUGIN_LIB)
        message(FATAL_ERROR "${PLUGIN_NAME} not found in ${CARB_STATIC_LIB_DIR}")
    endif()
    list(APPEND CARB_STATIC_PLUGIN_LIBS ${_CARB_STATIC_PLUGIN_LIB})
endforeach()

set(CARB_STATIC_INCLUDE_DIR "${LOCAL_TARGET_DEPS}/carb_sdk_static/include")
if(NOT EXISTS "${CARB_STATIC_INCLUDE_DIR}")
    message(FATAL_ERROR "Static Carbonite include directory not found: ${CARB_STATIC_INCLUDE_DIR}")
endif()
target_include_directories(ovphysx BEFORE PRIVATE ${CARB_STATIC_INCLUDE_DIR})
target_compile_definitions(ovphysx PRIVATE
    CARB_STATIC_PLUGIN_BUILD=1
    CARB_PLUGIN_CLIENT_NAME=omni_physx_sdk
    carb_ErrorApi=carb_ErrorApi_latest
    carb_Framework=carb_Framework_latest
    carb_filesystem_IFileSystem=carb_filesystem_IFileSystem_latest
    carb_dictionary_IDictionary=carb_dictionary_IDictionary_latest
)
target_link_libraries(ovphysx PRIVATE ${CARB_STATIC_LIBRARY} ${CARB_STATIC_PLUGIN_LIBS})
target_link_libraries(ovphysx PRIVATE OvruntimePhysXCudaShim OvruntimePhysX)

# OvruntimePhysX is a static archive. Its private external dependencies are
# linked into the final owner binary, so ovphysx must provide the matching
# search paths here.
set(_OVPHYSX_OVRUNTIME_TARGET_DEPS "${OVRUNTIME_DIR}/_build/target-deps")
if(OVPHYSX_RUNTIME_DEPS_CONFIG STREQUAL "per-config")
    set(_OVPHYSX_OVRUNTIME_LINK_CONFIG "$<IF:$<CONFIG:Debug>,debug,release>")
else()
    set(_OVPHYSX_OVRUNTIME_LINK_CONFIG "${OVPHYSX_RUNTIME_DEPS_CONFIG}")
endif()

target_link_directories(ovphysx PRIVATE
    "${_OVPHYSX_OVRUNTIME_TARGET_DEPS}/carb_sdk_plugins/_build/${CARB_PLATFORM_DIR}/${_OVPHYSX_OVRUNTIME_LINK_CONFIG}"
    "${_OVPHYSX_OVRUNTIME_TARGET_DEPS}/physx/bin/${PHYSX_PLATFORM_BIN}/$<IF:$<CONFIG:Debug>,debug,checked>"
)

if(WIN32)
    target_link_directories(ovphysx PRIVATE
        "${_OVPHYSX_OVRUNTIME_TARGET_DEPS}/cuda/lib/x64"
    )
else()
    target_link_directories(ovphysx PRIVATE
        "${_OVPHYSX_OVRUNTIME_TARGET_DEPS}/cuda/lib64"
    )
    target_link_options(ovphysx PRIVATE
        "-L${_OVPHYSX_OVRUNTIME_TARGET_DEPS}/cuda/lib64/stubs"
    )
endif()

if(WIN32)
    target_link_libraries(ovphysx PRIVATE shlwapi pathcch winmm)
    target_compile_options(ovphysx PRIVATE /wd4211)
else()
    # dlopen/dlsym for runtime driver loading
    target_link_libraries(ovphysx PRIVATE ${CMAKE_DL_LIBS})
endif()

# Do not export private build-tree targets such as OvruntimePhysX to downstream
# CMake users.
set_property(TARGET ovphysx PROPERTY INTERFACE_LINK_LIBRARIES "")

# ============================================================================
# Internal Sidecar (omni.physx-dependent functionality)
# ============================================================================
# The sidecar is a peer DSO loaded on demand by the main library. Neither
# library links OpenUSD; ovphysx consumes scenes through ovstage only.
add_subdirectory(src/ovphysxInternal)

# Compile definitions matching ovruntime (ABI compatibility).
# Applied per-target to avoid leaking into parent projects via add_subdirectory().
if(NOT WIN32)
    target_compile_definitions(ovphysx PRIVATE _FILE_OFFSET_BITS=64 _GLIBCXX_USE_CXX11_ABI=1)
    target_compile_definitions(ovphysx_internal PRIVATE _FILE_OFFSET_BITS=64 _GLIBCXX_USE_CXX11_ABI=1)
endif()

# Optionally help the runtime loader find Carbonite/Kit and PhysX plugins during development
if(OVPHYSX_BIN_DIR)
    message(STATUS "OVPHYSX_BIN_DIR set to: ${OVPHYSX_BIN_DIR}")
endif()

# Platform-specific runtime path handling
if(NOT WIN32)
    # Linux: Use RPATH for relocatable installations
    # $ORIGIN = directory containing the library at runtime
    set(ORIGIN_VAR "$ORIGIN")
    set(_OVPHYSX_BUILD_RPATH_DIRS "${ORIGIN_VAR}")
    if(OVPHYSX_BIN_DIR)
        list(APPEND _OVPHYSX_BUILD_RPATH_DIRS "${OVPHYSX_BIN_DIR}")
    endif()
    list(JOIN _OVPHYSX_BUILD_RPATH_DIRS ":" _OVPHYSX_BUILD_RPATH)

    # The INSTALL_RPATH stays relocatable. BUILD_RPATH keeps the build tree
    # runnable without adding the packman Python runtime to the shipped package.
    set_target_properties(ovphysx PROPERTIES
        INSTALL_RPATH "${ORIGIN_VAR}:${ORIGIN_VAR}/../plugins"
        # For BUILD tree: $ORIGIN for internal sidecar (same dir), plus dep dirs
        BUILD_RPATH "${_OVPHYSX_BUILD_RPATH}"
        # Link-time library paths are kept out of RPATH. The list above is explicit.
        INSTALL_RPATH_USE_LINK_PATH FALSE
        # Force RPATH instead of RUNPATH
        LINK_FLAGS "-Wl,--disable-new-dtags"
    )
    message(STATUS "Set INSTALL_RPATH for ovphysx: ${ORIGIN_VAR}-relative (relocatable)")
else()
    # Windows: DLLs are found via PATH or by being in the same directory
    # Set the output directory to ensure DLLs are together
    message(STATUS "Windows platform: Using PATH-based DLL resolution")
    
    # On Windows, copy required DLLs to the output directory after build
    # This is handled by install rules and runtime environment setup
endif()

# Group headers for IDEs (optional)
source_group(TREE ${OVPHYSX_PUBLIC_INCLUDE_DIR} FILES
    ${OVPHYSX_PUBLIC_INCLUDE_DIR}/ovphysx/ovphysx.h
    ${OVPHYSX_PUBLIC_INCLUDE_DIR}/ovphysx/experimental/Helpers.hpp
    ${OVPHYSX_PUBLIC_INCLUDE_DIR}/ovphysx/experimental/ovphysx.hpp
    ${OVPHYSX_PUBLIC_INCLUDE_DIR}/ovphysx/experimental/TensorBinding.hpp
    ${OVPHYSX_PUBLIC_INCLUDE_DIR}/ovphysx/version.h
)

# Copy include folder to build directory.
# Only add to ALL when top-level. As a subdirectory the ovphysx dependency suffices.
set(_COPY_HEADERS_ALL "")
if(OVPHYSX_IS_TOP_LEVEL)
    set(_COPY_HEADERS_ALL ALL)
endif()
add_custom_target(copy_headers ${_COPY_HEADERS_ALL}
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_BINARY_DIR}/include
    COMMENT "Copying include folder to build directory"
)

# Ensure headers are copied before building the library
add_dependencies(ovphysx copy_headers)

# Tests, install rules, and workflow targets are only relevant when building
# ovphysx as the top-level project. When included via add_subdirectory() the
# parent project owns testing, installation, and packaging.
if(OVPHYSX_IS_TOP_LEVEL)

# Add test targets
enable_testing()
add_subdirectory(tests/c_unittests)

# Opt-in benchmark suite. OFF by default: benchmarks are excluded from the
# standard build/validate flow so noise never reaches `validate_all`.
# Enable with `cmake -DOVPHYSX_BUILD_BENCHMARKS=ON ...` before configure.
if(OVPHYSX_BUILD_BENCHMARKS)
    message(STATUS "Benchmarks: ENABLED (will build ovphysx_benchmarks)")
    add_subdirectory(tests/benchmarks)

    # Regenerate the BENCHMARK_SUMMARY.md catalogue once per build (into _build/generated). A
    # custom target with no OUTPUT/BYPRODUCTS is always out of date, so it runs exactly once per
    # build. It derives from the .cpp sources, not compiled artifacts, so it never gates the build.
    add_custom_target(generate_benchmark_summary ALL
        COMMAND ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_benchmark_summary.cmake"
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        COMMENT "Generating C++ benchmark catalogue (BENCHMARK_SUMMARY.md)"
        VERBATIM
    )
else()
    message(STATUS "Benchmarks: disabled (pass -DOVPHYSX_BUILD_BENCHMARKS=ON to enable)")
endif()

# ==============================================================================
# CMake Package Configuration for find_package() support
# ==============================================================================

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Set installation directories
set(CMAKE_INSTALL_INCLUDEDIR "include")
set(CMAKE_INSTALL_LIBDIR "lib")
set(CMAKE_INSTALL_BINDIR "bin")
set(OVPHYSX_PLUGINSDIR "plugins")

# Install the library
install(TARGETS ovphysx
    EXPORT ovphysxTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install the internal sidecar library (required at runtime)
install(TARGETS ovphysx_internal
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install PDB debug symbol files alongside DLLs for Debug builds (Windows only).
# Enables stepping through ovphysx source code with a debugger.
if(WIN32)
    install(FILES $<TARGET_PDB_FILE:ovphysx> DESTINATION ${CMAKE_INSTALL_BINDIR}
            CONFIGURATIONS Debug OPTIONAL)
    install(FILES $<TARGET_PDB_FILE:ovphysx_internal> DESTINATION ${CMAKE_INSTALL_BINDIR}
            CONFIGURATIONS Debug OPTIONAL)
endif()

# Install headers
install(DIRECTORY ${OVPHYSX_PUBLIC_INCLUDE_DIR}/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# ovstage is not installed: no headers, library, runtime, or Python package.
# It remains a required external dependency. Native SDK consumers provide the
# matching OVStage native package, which ovphysxConfig.cmake resolves through
# find_dependency(ovstage).
#
# The notice stays because OVStage is the required application-supplied scene
# population dependency represented by the installed companion target.
set(_OVPHYSX_OVSTAGE_NOTICES "${OVSTAGE_DIR}/THIRD-PARTY-NOTICES.txt")
install(FILES "${_OVPHYSX_OVSTAGE_NOTICES}"
    DESTINATION .
    RENAME ovstage-THIRD-PARTY-NOTICES.txt
)

# Install the same PhysX SDK headers selected above for pointer interop
# (Apache-2.0). The selection is shared with c_unittests so neither validation
# nor packaging can silently use headers from a different PhysX dependency.
install(DIRECTORY "${_OVPHYSX_PHYSX_INCLUDE_SOURCE}/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/physx
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.inl"
)

# Install generated version.h (overwrites the .in template if present)
install(FILES "${OVPHYSX_GENERATED_INCLUDE_DIR}/ovphysx/version.h"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ovphysx
)

# Export targets to build directory (for testing without install)
export(EXPORT ovphysxTargets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/ovphysxTargets.cmake"
    NAMESPACE ovphysx::
)

# Create and install the targets file
install(EXPORT ovphysxTargets
    FILE ovphysxTargets.cmake
    NAMESPACE ovphysx::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ovphysx
)

# Create the config file
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ovphysxConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/ovphysxConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ovphysx
    PATH_VARS CMAKE_INSTALL_BINDIR CMAKE_INSTALL_LIBDIR OVPHYSX_PLUGINSDIR
)

# Create the version file
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/ovphysxConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Install the config files
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/ovphysxConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/ovphysxConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ovphysx
)

# ==============================================================================
# Named Workflow Targets and CTest Registration
# ==============================================================================
# These targets encode the dependency graph between build, install, packaging,
# and test steps. The simplest full validation (no prior build required) is:
#   cmake -P scripts/validate_all.cmake
# For more control (after a full build via build.sh|bat or build.cmake):
#   cmake --build _build --target validate_all      # same install + wheel + tests as validate_all.cmake (without its build.cmake prefix)
#   cmake --build _build --target validate_runtime  # runtime tests only
#   cmake --build _build --target validate_wheel    # wheel tests only
#
# Presets (for re-configuring after build.sh has already fetched deps):
#   cmake --preset host-release
#   cmake --build --preset validate-all
#
# Target dependency graph:
#   install_sdk ---------> _install/
#     +---> test_cpp
#     +---> test_cpp_samples
#     +---> test_python_runtime
#     +---> package_sdk ----------> _dist/*.tar.gz
#     +---> build_wheel ----------> _dist/*.whl
#             +---> test_python_wheel
#             +---> test_python_samples
#
#   ovphysx + ovphysx_internal + ovruntime_runtime_libs
#     +---> test_source_link (add_subdirectory workflow)
#
# Meta-targets:
#   validate_runtime = install_sdk + ctest -L runtime
#   validate_wheel   = build_wheel + ctest -L wheel
#   validate_all     = build_wheel + full ctest
# ==============================================================================

# Multi-config generators need --config passed to cmake -P scripts.
# Single-config generators use CMAKE_BUILD_TYPE.
if(CMAKE_CONFIGURATION_TYPES)
    set(_SCRIPT_BUILD_TYPE_ARG "-DBUILD_TYPE=$<CONFIG>")
    set(_CTEST_CONFIG_ARGS -C $<CONFIG>)
else()
    set(_SCRIPT_BUILD_TYPE_ARG "-DBUILD_TYPE=${CMAKE_BUILD_TYPE}")
    set(_CTEST_CONFIG_ARGS)
endif()
set(_SCRIPT_RUNTIME_DEPS_ARG "-DOVPHYSX_USE_RELEASE_RUNTIME_DEPS=${OVPHYSX_USE_RELEASE_RUNTIME_DEPS}")

# The source-link sample runs a nested configure from CTest/custom-target
# processes that may not inherit the Packman compiler environment. Serialize
# the effective outer generator/toolchain once and pass only the generated file
# path through the harness command line.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/SourceLinkContext.cmake")
set(_SOURCE_LINK_CONTEXT_FILE "${CMAKE_CURRENT_BINARY_DIR}/source_link_context.cmake")
ovphysx_write_source_link_context("${_SOURCE_LINK_CONTEXT_FILE}")

# --- install_sdk: assemble _install/ ---
add_custom_target(install_sdk
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG} ${_SCRIPT_RUNTIME_DEPS_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/install.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Assembling SDK in _install/"
    USES_TERMINAL
)
add_dependencies(install_sdk ovphysx ovphysx_internal ovruntime_runtime_libs)

# --- package_sdk: create SDK archive in _dist/ ---
add_custom_target(package_sdk
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG} ${_SCRIPT_RUNTIME_DEPS_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/package_sdk.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Creating SDK archive in _dist/"
    USES_TERMINAL
)
add_dependencies(package_sdk install_sdk)

# --- build_wheel: stage and build wheel into _dist/ ---
add_custom_target(build_wheel
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG} ${_SCRIPT_RUNTIME_DEPS_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_wheel.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Building Python wheel into _dist/"
    USES_TERMINAL
)
add_dependencies(build_wheel install_sdk)

# --- Individual test targets ---

add_custom_target(test_cpp
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_cpp.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Running C++ unit tests"
    USES_TERMINAL
)
add_dependencies(test_cpp install_sdk)

add_custom_target(test_cpp_samples
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_cpp_samples.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Running C++ sample applications"
    USES_TERMINAL
)
add_dependencies(test_cpp_samples install_sdk)

add_custom_target(test_source_link
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        "-DOVPHYSX_DEV_PHYSX=${OVRUNTIME_DEV_PHYSX}"
        "-DSOURCE_LINK_CONTEXT_FILE=${_SOURCE_LINK_CONTEXT_FILE}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_source_link.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Building and running source-link sample (add_subdirectory workflow)"
    USES_TERMINAL
    VERBATIM
)
# install_sdk provides the flattened plugins/ directory needed at runtime.
add_dependencies(test_source_link install_sdk)

add_custom_target(test_python_runtime
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_python_runtime.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Running Python runtime tests"
    USES_TERMINAL
)
add_dependencies(test_python_runtime install_sdk)

add_custom_target(test_python_wheel
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_python_wheel.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Running Python wheel smoke tests"
    USES_TERMINAL
)
add_dependencies(test_python_wheel build_wheel)

add_custom_target(test_python_samples
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_python_samples.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Running Python sample applications"
    USES_TERMINAL
)
add_dependencies(test_python_samples build_wheel)

# --- Benchmark targets (opt-in, label: benchmarks) ---
# These targets are registered unconditionally so a developer can `cmake -P`
# the driver scripts even from a stale tree, but the underlying binary is only
# built when OVPHYSX_BUILD_BENCHMARKS=ON. The driver scripts themselves error
# out with a clear message if the binary is missing.

add_custom_target(test_benchmarks_cpp
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_benchmarks_cpp.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Running C++ benchmark suite"
    USES_TERMINAL
)
add_dependencies(test_benchmarks_cpp install_sdk)
if(OVPHYSX_BUILD_BENCHMARKS)
    add_dependencies(test_benchmarks_cpp ovphysx_benchmarks)
endif()

add_custom_target(test_benchmarks_python
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_benchmarks_python.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Running Python benchmark suite"
    USES_TERMINAL
)
add_dependencies(test_benchmarks_python install_sdk)

# Producer contract for the hidden L1B (Authoring/WriteScaling) benchmark
# inventory: checks that the executable's registered row set matches
# tests/benchmarks/tests/producer_inventory.json exactly. See
# scripts/test_benchmark_contract.cmake.
add_custom_target(test_benchmark_contract
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_benchmark_contract.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Running L1B benchmark inventory contract"
    USES_TERMINAL
)
add_dependencies(test_benchmark_contract install_sdk)
if(OVPHYSX_BUILD_BENCHMARKS)
    add_dependencies(test_benchmark_contract ovphysx_benchmarks)
endif()

# --- CTest registration ---
# Each test invokes the corresponding cmake -P script so that CTest can
# discover and run them via labels (ctest -L runtime, ctest -L wheel).

add_test(NAME cpp-unit
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_cpp.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
set_tests_properties(cpp-unit PROPERTIES
    LABELS "cpp-unit;runtime"
)

add_test(NAME cpp-samples
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_cpp_samples.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
set_tests_properties(cpp-samples PROPERTIES
    LABELS "cpp-samples;runtime"
)

add_test(NAME source-link
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        "-DOVPHYSX_DEV_PHYSX=${OVRUNTIME_DEV_PHYSX}"
        "-DSOURCE_LINK_CONTEXT_FILE=${_SOURCE_LINK_CONTEXT_FILE}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_source_link.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
set_tests_properties(source-link PROPERTIES
    LABELS "source-link;runtime"
)

add_test(NAME python-runtime
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_python_runtime.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
set_tests_properties(python-runtime PROPERTIES
    LABELS "python-runtime;runtime"
    RUN_SERIAL TRUE
)

add_test(NAME python-wheel
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_python_wheel.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
set_tests_properties(python-wheel PROPERTIES
    LABELS "python-wheel;wheel"
)

add_test(NAME python-samples
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_python_samples.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
set_tests_properties(python-samples PROPERTIES
    LABELS "python-samples;wheel"
)

# Benchmarks: registered with the `benchmarks` label only, so `ctest -L benchmarks`
# runs them and `validate_all` excludes them via `-LE benchmarks`.
add_test(NAME benchmarks-cpp
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_benchmarks_cpp.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
set_tests_properties(benchmarks-cpp PROPERTIES
    LABELS "benchmarks-cpp;benchmarks"
)

add_test(NAME benchmarks-python
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_benchmarks_python.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
set_tests_properties(benchmarks-python PROPERTIES
    LABELS "benchmarks-python;benchmarks"
)

add_test(NAME benchmark-contract
    COMMAND ${CMAKE_COMMAND} ${_SCRIPT_BUILD_TYPE_ARG}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_benchmark_contract.cmake"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
set_tests_properties(benchmark-contract PROPERTIES
    LABELS "benchmark-contract;benchmarks"
)

# --- Meta-targets ---

add_custom_target(validate_runtime
    COMMAND ${CMAKE_CTEST_COMMAND} ${_CTEST_CONFIG_ARGS} -L runtime --output-on-failure
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Running runtime validation (ctest -L runtime)"
    USES_TERMINAL
)
add_dependencies(validate_runtime install_sdk)

add_custom_target(validate_wheel
    COMMAND ${CMAKE_CTEST_COMMAND} ${_CTEST_CONFIG_ARGS} -L wheel --output-on-failure
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Running wheel validation (ctest -L wheel)"
    USES_TERMINAL
)
add_dependencies(validate_wheel build_wheel)

add_custom_target(validate_benchmarks
    COMMAND ${CMAKE_CTEST_COMMAND} ${_CTEST_CONFIG_ARGS} -L benchmarks --output-on-failure
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Running benchmark validation (ctest -L benchmarks)"
    USES_TERMINAL
)
add_dependencies(validate_benchmarks install_sdk)
if(OVPHYSX_BUILD_BENCHMARKS)
    add_dependencies(validate_benchmarks ovphysx_benchmarks)
endif()

add_custom_target(validate_all
    # `-LE benchmarks` excludes the benchmark suites: they are opt-in (slow,
    # noisy without dedicated hardware) and run via `validate_benchmarks`.
    COMMAND ${CMAKE_CTEST_COMMAND} ${_CTEST_CONFIG_ARGS} -LE benchmarks --output-on-failure
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Running full validation (all CTest suites)"
    USES_TERMINAL
)
add_dependencies(validate_all build_wheel)

endif() # OVPHYSX_IS_TOP_LEVEL

# ==============================================================================
# Source-link helper (available to parent projects using add_subdirectory)
# ==============================================================================
# Call ovphysx_setup_source_link_runtime(<target>) in a parent project to
# configure runtime library discovery for an executable that links ovphysx
# via add_subdirectory() (i.e. without an installed SDK).

# Export paths via CACHE INTERNAL for the function below (child scope is not
# visible to the parent project that calls the function).
set(_OVPHYSX_OUTPUT_DIR "${OUTPUT_DIR}" CACHE INTERNAL "")
get_filename_component(_OVPHYSX_CONFIGURED_OVSTAGE_DIR
    "${OVSTAGE_DIR}" REALPATH BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
get_filename_component(_OVPHYSX_CONFIGURED_OVSTAGE_RUNTIME_DIR
    "${_OVPHYSX_OVSTAGE_PYTHON_RUNTIME_DIR}" REALPATH BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
get_filename_component(_OVPHYSX_CONFIGURED_OVSTAGE_LIBRARY_DIR
    "${OVSTAGE_LIBRARY_DIR}" REALPATH BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(_OVPHYSX_OVSTAGE_LIBRARY_DIR "${_OVPHYSX_CONFIGURED_OVSTAGE_LIBRARY_DIR}" CACHE INTERNAL "")
set(_OVPHYSX_OVSTAGE_RUNTIME "${_OVPHYSX_OVSTAGE_RUNTIME}" CACHE INTERNAL "")
set(_OVPHYSX_OVSTAGE_PYTHON_RUNTIME_DIR "${_OVPHYSX_OVSTAGE_PYTHON_RUNTIME_DIR}" CACHE INTERNAL "")
set(_OVPHYSX_OVSTAGE_DIR "${_OVPHYSX_CONFIGURED_OVSTAGE_DIR}" CACHE INTERNAL "")
set(_OVPHYSX_OVSTAGE_RUNTIME_DIR "${_OVPHYSX_CONFIGURED_OVSTAGE_RUNTIME_DIR}" CACHE INTERNAL "")
unset(_OVPHYSX_CONFIGURED_OVSTAGE_DIR)
unset(_OVPHYSX_CONFIGURED_OVSTAGE_LIBRARY_DIR)
unset(_OVPHYSX_CONFIGURED_OVSTAGE_RUNTIME_DIR)

function(ovphysx_setup_source_link_runtime target)
    # Set up runtime library discovery for executables using ovphysx via
    # add_subdirectory(). On Linux this adds RPATH entries. On Windows it
    # copies core DLLs next to the executable.
    #
    # NOTE: This handles library *loading* (dlopen of libovphysx, the internal
    # sidecar, and ovstage). Runtime schema/plugin *discovery* is derived from
    # the ovphysx library directory, or from OVPHYSX_LIB when set. For
    # source-link runs that need the installed SDK runtime layout, set
    # OVPHYSX_LIB to the installed ovphysx library. See docs/local_development.md.

    if(UNIX AND NOT APPLE)
        set_property(TARGET ${target} APPEND PROPERTY BUILD_RPATH
            "${_OVPHYSX_OUTPUT_DIR}"
            "${_OVPHYSX_OVSTAGE_LIBRARY_DIR}")
    elseif(WIN32)
        foreach(_rt_target ovphysx ovphysx_internal)
            add_custom_command(TARGET ${target} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "$<TARGET_FILE:${_rt_target}>" "$<TARGET_FILE_DIR:${target}>")
        endforeach()
        # Leave ovstage in its own runtime root. Copying ovstage.dll beside an
        # ovphysx consumer makes both libraries resolve the same local plugins/
        # directory, which can register Carbonite plugins through two frameworks.
    endif()
endfunction()

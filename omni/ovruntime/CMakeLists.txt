# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

cmake_minimum_required(VERSION 3.22)

# Include INSTALL in the default VS build so that building the solution
# automatically deploys binaries to _install/ (where ovexts symlinks point).
set(CMAKE_VS_INCLUDE_INSTALL_TO_DEFAULT_BUILD 1)

project(physics-ovruntime)

# Per-config install prefix.  build.bat passes --prefix at install time which
# overrides this, but when VS runs the INSTALL target directly (no --prefix)
# we need a sensible default.  install(CODE) runs at install time where
# CMAKE_INSTALL_CONFIG_NAME holds the active configuration (Release/Debug).
# This sets the prefix to _install/ovruntime/release or _install/ovruntime/debug
# automatically — no manual deploy step needed.
# Per-config install prefix for the VS INSTALL target.  At install time,
# CMAKE_INSTALL_CONFIG_NAME holds the active config (Release / Debug).
# If --prefix was passed (build.bat), CMAKE_INSTALL_PREFIX differs from the
# configure-time default and we leave it alone.  Otherwise (VS INSTALL target)
# we override it to _install/ovruntime/{config}.
set(_OVRUNTIME_CONFIGURE_PREFIX "${CMAKE_INSTALL_PREFIX}")
install(CODE "
    if(CMAKE_INSTALL_PREFIX STREQUAL \"${_OVRUNTIME_CONFIGURE_PREFIX}\")
        string(TOLOWER \"\${CMAKE_INSTALL_CONFIG_NAME}\" _config_lower)
        set(CMAKE_INSTALL_PREFIX \"${CMAKE_CURRENT_SOURCE_DIR}/_install/ovruntime/\${_config_lower}\")
        message(STATUS \"Install prefix: \${CMAKE_INSTALL_PREFIX}\")
    endif()
")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
      ${CMAKE_CURRENT_SOURCE_DIR}/cmake              # ovruntime-specific (Options.cmake)
      ${CMAKE_CURRENT_SOURCE_DIR}/../cmake             # shared omni/cmake modules (NvidiaBuildOptions, CompilerDefaults, etc.)
)

set(OVRUNTIME_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Platform detection (can be overridden via -DTARGET_BUILD_PLATFORM=...)
if(WIN32)
    set(_DEFAULT_PLATFORM "windows")
elseif(UNIX AND NOT APPLE)
    set(_DEFAULT_PLATFORM "linux")
elseif(APPLE)
    set(_DEFAULT_PLATFORM "mac")
endif()
set(TARGET_BUILD_PLATFORM "${_DEFAULT_PLATFORM}" CACHE STRING "Target build platform")

# Required by NvidiaBuildOptions for output directory structure
set(PX_OUTPUT_LIB_DIR "${CMAKE_BINARY_DIR}" CACHE PATH "Base lib output directory")
set(PX_OUTPUT_BIN_DIR "${CMAKE_BINARY_DIR}" CACHE PATH "Base bin output directory")

# Find Carbonite SDK from kit-sdk dependencies (pulled via packman).
# When included as a subdirectory (e.g. from ovexts), the parent project
# may set OVRUNTIME_TARGET_DEPS, OVRUNTIME_KIT_SDK_DIR, and CARB_SDK_DIR
# before add_subdirectory() — respect those values.
if(NOT DEFINED OVRUNTIME_TARGET_DEPS)
    set(OVRUNTIME_TARGET_DEPS "${CMAKE_CURRENT_SOURCE_DIR}/_build/target-deps")
endif()

# Kit SDK directory: when built as an ovphysx subproject, ovphysx fetches
# kit_sdk_<config> into its own target-deps; OVRUNTIME_KIT_SDK_DIR lets
# ovruntime cmake modules resolve headers from the right kit SDK.
# Ovruntime's own pull_dependencies always fetches release; ovphysx may
# fetch debug.  Try config-specific first, fall back to release.
if(NOT DEFINED OVRUNTIME_KIT_SDK_DIR)
    if(CMAKE_BUILD_TYPE)
        string(TOLOWER "${CMAKE_BUILD_TYPE}" _ovr_bt)
    else()
        set(_ovr_bt "release")
    endif()
    set(OVRUNTIME_KIT_SDK_DIR "${OVRUNTIME_TARGET_DEPS}/kit_sdk_${_ovr_bt}")
    if(NOT EXISTS "${OVRUNTIME_KIT_SDK_DIR}")
        set(OVRUNTIME_KIT_SDK_DIR "${OVRUNTIME_TARGET_DEPS}/kit_sdk_release")
    endif()
endif()
if(NOT DEFINED CARB_SDK_DIR)
    if(EXISTS "${OVRUNTIME_TARGET_DEPS}/carb_sdk_plugins/include")
        set(CARB_SDK_DIR "${OVRUNTIME_TARGET_DEPS}/carb_sdk_plugins")
        message(STATUS "Found Carbonite SDK: ${CARB_SDK_DIR}")
    else()
        message(WARNING "Carbonite SDK not found. Run pull_dependencies.bat (or .sh) first.")
    endif()
endif()

# Find PhysX SDK (pulled via packman) — skipped when building from source (OVRUNTIME_DEV_PHYSX)
if(NOT OVRUNTIME_DEV_PHYSX AND NOT DEFINED PHYSX_SDK_DIR)
    if(EXISTS "${OVRUNTIME_TARGET_DEPS}/physx")
        set(PHYSX_SDK_DIR "${OVRUNTIME_TARGET_DEPS}/physx")
        message(STATUS "Found PhysX SDK: ${PHYSX_SDK_DIR}")
    else()
        message(WARNING "PhysX SDK not found. Run pull_dependencies.bat (or .sh) first.")
    endif()
endif()

# Find USD (pulled via packman, config-dependent)
if(NOT DEFINED USD_DIR)
    if(EXISTS "${OVRUNTIME_TARGET_DEPS}/usd")
        set(USD_DIR "${OVRUNTIME_TARGET_DEPS}/usd")
        message(STATUS "Found USD: ${USD_DIR}")
    else()
        message(WARNING "USD not found. Run pull_dependencies.bat (or .sh) first.")
    endif()
endif()

# Find CUDA (pulled via packman)
if(NOT DEFINED CUDA_DIR)
    if(EXISTS "${OVRUNTIME_TARGET_DEPS}/cuda")
        set(CUDA_DIR "${OVRUNTIME_TARGET_DEPS}/cuda")
        message(STATUS "Found CUDA: ${CUDA_DIR}")
    else()
        message(WARNING "CUDA not found. Run pull_dependencies.bat (or .sh) first.")
    endif()
endif()

# Find Python (pulled via packman). Allow override via -DPYTHON_DIR=...
# for multi-version binding builds (e.g. py311 pass from ovphysx).
if(NOT DEFINED PYTHON_DIR OR NOT EXISTS "${PYTHON_DIR}")
    set(PYTHON_DIR "${OVRUNTIME_TARGET_DEPS}/python")
endif()
if(EXISTS "${PYTHON_DIR}")
    message(STATUS "Found Python: ${PYTHON_DIR}")
    # Many targets transitively need the Python import library (via pybind11
    # auto-link pragma or USD's usd_python import lib). Add the directory
    # globally so the linker can resolve it without every target having an
    # explicit link_directories().
    if(WIN32)
        link_directories(${PYTHON_DIR}/libs)
    else()
        link_directories(${PYTHON_DIR}/lib)
    endif()
else()
    message(WARNING "Python not found. Run pull_dependencies.bat (or .sh) first.")
endif()

# Locate pip dependencies (Newton USD schemas, etc.) pre-fetched by
# pull_dependencies into OVRUNTIME_TARGET_DEPS.  Requires PYTHON_DIR above.
include(FetchPipDependencies)

# Find doctest (pulled via packman)
if(NOT DEFINED DOCTEST_DIR)
    if(EXISTS "${OVRUNTIME_TARGET_DEPS}/doctest")
        set(DOCTEST_DIR "${OVRUNTIME_TARGET_DEPS}/doctest")
        message(STATUS "Found doctest: ${DOCTEST_DIR}")
    else()
        message(WARNING "doctest not found. Run pull_dependencies.bat (or .sh) first.")
    endif()
endif()

# Find cxxopts (pulled via packman, used by unit tests)
if(NOT DEFINED CXXOPTS_DIR)
    if(EXISTS "${OVRUNTIME_TARGET_DEPS}/cxxopts")
        set(CXXOPTS_DIR "${OVRUNTIME_TARGET_DEPS}/cxxopts")
        message(STATUS "Found cxxopts: ${CXXOPTS_DIR}")
    else()
        message(WARNING "cxxopts not found. Run pull_dependencies.bat (or .sh) first.")
    endif()
endif()

# Find ovruntime_deps (RTX/fabric plugin binaries and headers — replaces kit-sdk + rtx_plugins)
if(NOT DEFINED OVRUNTIME_DEPS_DIR)
    if(EXISTS "${OVRUNTIME_TARGET_DEPS}/ovruntime_deps_release")
        set(OVRUNTIME_DEPS_DIR "${OVRUNTIME_TARGET_DEPS}/ovruntime_deps_release")
        message(STATUS "Found ovruntime_deps: ${OVRUNTIME_DEPS_DIR}")
    else()
        message(WARNING "ovruntime_deps not found. Run pull_dependencies.bat (or .sh) first.")
    endif()
endif()

# Find physics schema (physxSchema, physicsSchemaTools)
# When OVRUNTIME_DEV_SCHEMA is ON, use the locally-built schema from ../schema/
# instead of the packman-provided usd_ext_physics package.
if(NOT DEFINED USD_EXT_PHYSICS_DIR)
    if(OVRUNTIME_DEV_SCHEMA)
        if(WIN32)
            set(_SCHEMA_PLATFORM "windows-x86_64")
        elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
            set(_SCHEMA_PLATFORM "linux-aarch64")
        else()
            set(_SCHEMA_PLATFORM "linux-x86_64")
        endif()
        set(USD_EXT_PHYSICS_DIR "${OVRUNTIME_ROOT_DIR}/../schema/_build/${_SCHEMA_PLATFORM}/release/schema")
        message(STATUS "DevSchema mode: using local physics schema: ${USD_EXT_PHYSICS_DIR}")
        if(NOT EXISTS "${USD_EXT_PHYSICS_DIR}/include")
            message(WARNING "Local schema not found at ${USD_EXT_PHYSICS_DIR}. Run schema/repo.sh usd first.")
        endif()
    else()
        if(EXISTS "${OVRUNTIME_TARGET_DEPS}/usd_ext_physics/release/include")
            set(USD_EXT_PHYSICS_DIR "${OVRUNTIME_TARGET_DEPS}/usd_ext_physics/release")
            message(STATUS "Found physics schema: ${USD_EXT_PHYSICS_DIR}")
        else()
            message(WARNING "Physics schema not found. Run pull_dependencies.bat (or .sh) first.")
        endif()
    endif()
endif()

# Find pybind11 (pulled via packman, used by Carbonite Python bindings)
if(NOT DEFINED PYBIND11_DIR)
    if(EXISTS "${OVRUNTIME_TARGET_DEPS}/pybind11/include")
        set(PYBIND11_DIR "${OVRUNTIME_TARGET_DEPS}/pybind11")
        message(STATUS "Found pybind11: ${PYBIND11_DIR}")
    else()
        message(WARNING "pybind11 not found. Run pull_dependencies.bat (or .sh) first.")
    endif()
endif()

include(Options)                    # ovruntime-specific options (OVRUNTIME_ENABLE_TESTS, etc.)



include(NvidiaBuildOptions)         # shared: CRT, output dirs, platform naming
include(CarbPlatform)              # shared: CARB_PLATFORM_DIR for Carbonite SDK lib path
# PhysX must be included BEFORE CompilerDefaults which adds -Werror globally.
# PhysX has its own warning settings and must not inherit ovruntime's strict flags.
# Note: PhysXDependency may remap CMAKE_BUILD_TYPE (e.g. Release → checked) when
# OVRUNTIME_DEV_PHYSX is ON so that PhysX builds with its "checked" configuration.
include(PhysXDependency)            # PhysX SDK: source build (--devphysx) or packman package
include(CompilerDefaults)           # shared: compiler warning flags, optimizations

# GCC 12+ triggers false-positive warnings in PhysX/USD/TBB headers that don't
# fire on GCC 11 (the CI Docker toolchain). SYSTEM includes (below) don't help
# here because GCC attributes warnings from inlined/instantiated SYSTEM-header
# code to the call site in user code. Suppress them so -Werror doesn't break.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "12")
    add_compile_options(
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-uninitialized>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-dangling-pointer>
        $<$<COMPILE_LANGUAGE:CXX>:-Wno-changes-meaning>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-array-bounds>        # GCC 13: false positives in SmallVector/ujitso/vcd inlined code
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-stringop-overread>   # GCC 13: false positives in inlined STL/SmallVector memcpy
        $<$<COMPILE_LANGUAGE:CXX>:-Wno-dangling-reference>    # GCC 13: false positives on USD attribute accessors
    )
endif()

include(ExternalSystemIncludes)     # marks external dep headers as SYSTEM (no warnings)
include(UsdLinkDependencies)        # ovruntime_link_usd_deps() helper for USD/TBB/Python libs
include(CudaUtils)                  # ovruntime_target_add_cuda_sources() + global CUDA setup

# Carbonite config directory mapping.  Carbonite SDK ships "debug" and "release"
# subdirectories.  When PhysXDependency remaps CMAKE_BUILD_TYPE to "checked",
# $<LOWER_CASE:$<CONFIG>> would resolve to "checked" which does not exist in the
# Carbonite SDK.  OVRUNTIME_CARB_CONFIG maps checked → release so all modules can
# use ${OVRUNTIME_CARB_CONFIG} for Carb library paths.
set(OVRUNTIME_CARB_CONFIG "$<IF:$<CONFIG:checked>,release,$<LOWER_CASE:$<CONFIG>>>")

# Suppress high-volume warnings from ovruntime code that are impractical to fix
# individually (ported from premake where these were never enabled).  Potentially
# dangerous categories (-Wuninitialized, -Wmaybe-uninitialized, -Wnarrowing, etc.)
# are intentionally kept enabled.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Warnings as errors (C/CXX only — nvcc does not understand GCC warning flags)
    add_compile_options($<$<COMPILE_LANGUAGE:C,CXX>:-Werror>)
    # Suppress specific warning categories that are too noisy to fix across the
    # codebase right now.  Guarded with COMPILE_LANGUAGE so nvcc is not affected.
    add_compile_options(
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-switch>                     # 1050 — intentional partial enum handling
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-unused-parameter>           #  399 — interface/virtual overrides
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-unused-variable>            #  146 — side-effect-only declarations
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-strict-aliasing>            #  117 — common in low-level physics code
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-sign-compare>               #   54 — mixed signed/unsigned comparisons
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-deprecated-declarations>    #   51 — deprecated USD/PhysX API usage
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-unused-but-set-variable>    #   38 — debug remnants
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-reorder>                    #   30 — member init order vs declaration
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-unused-function>            #   29 — unreferenced static/inline helpers
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-ignored-qualifiers>         #   18 — const on non-ref return types
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-class-memaccess>            #   14 — memcpy on POD-like types
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-enum-compare>               #   13 — cross-enum comparisons
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-missing-field-initializers> #   12 — partial aggregate initialization
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-range-loop-construct>       #    5 — copy vs reference in range-for
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-nonnull-compare>            #    3 — defensive null checks
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-maybe-uninitialized>        #   ~24 — GCC false positives in optimized code
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-delete-incomplete>          #    1 — void* deletion (needs design fix)
    )
elseif(MSVC)
    # MSVC equivalents of the GCC/Clang suppressions above.
    # Guarded with COMPILE_LANGUAGE:C,CXX so nvcc does not receive these flags.
    add_compile_options(
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4100>  # unreferenced formal parameter           (= -Wno-unused-parameter)
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4189>  # local variable initialized but not referenced (= -Wno-unused-variable)
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4456>  # declaration hides previous local        (= -Wno-shadow subset)
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4457>  # declaration hides function parameter
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4458>  # declaration hides class member
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4505>  # unreferenced local function removed     (= -Wno-unused-function)
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4701>  # potentially uninitialized local variable (= -Wno-maybe-uninitialized)
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4703>  # potentially uninitialized local pointer variable
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4702>  # unreachable code
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4244>  # conversion, possible loss of data       (numeric conversions in physics math)
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4267>  # conversion from size_t                  (common in physics math)
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4018>  # signed/unsigned mismatch relational     (= -Wno-sign-compare)
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4245>  # signed/unsigned mismatch conversion     (int→unsigned in assignments/args)
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4389>  # signed/unsigned mismatch '=='/'!='
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4996>  # deprecated declaration                  (= -Wno-deprecated-declarations)
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4127>  # conditional expression is constant      (common in templates/macros)
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4305>  # truncation from 'double' to 'float'     (literal constants in physics math)
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4373>  # virtual function override, params differ by const/volatile qualifiers
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4101>  # unreferenced local variable             (= -Wno-unused-variable subset)
    )
endif()

# Code coverage instrumentation (gcov)
if(OVRUNTIME_ENABLE_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        message(STATUS "Code coverage enabled (gcov)")
        add_compile_options($<$<COMPILE_LANGUAGE:C,CXX>:--coverage>)
        add_link_options(--coverage)
    else()
        message(WARNING "Code coverage is only supported with GCC/Clang, ignoring OVRUNTIME_ENABLE_COVERAGE")
    endif()
endif()

# C++ standard and position-independent code
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Configuration types — keep Debug/Release only (Checked/Profile are PhysX-specific
# and require additional linker flag variables that CMake does not define by default)
if(CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)
endif()

# Ensure _DEBUG is defined for Debug builds (required by PhysX/carb headers;
# CMake only defines NDEBUG for Release automatically on non-MSVC).
# "checked" is a Release-like config so it gets NDEBUG but not _DEBUG.
add_compile_definitions($<$<CONFIG:Debug>:_DEBUG>)
add_compile_definitions($<$<CONFIG:checked>:NDEBUG>)

# Compile definitions matching premake (required by USD/TBB headers and ABI compat)
if(NOT WIN32)
    add_compile_definitions(
        _FILE_OFFSET_BITS=64
        _GLIBCXX_USE_CXX11_ABI=1
        TBB_SUPPRESS_DEPRECATED_MESSAGES
    )
    add_compile_options(
        $<$<COMPILE_LANGUAGE:C,CXX>:-fstack-protector-strong>
        $<$<COMPILE_LANGUAGE:C,CXX>:-pthread>
    )
    add_link_options(
        -Wl,-z,relro
        -Wl,--gc-sections
    )
endif()

# RPATH configuration:
# - Build tree: CMake auto-computes RPATH from TARGET_LINK_DIRECTORIES (target-deps paths)
# - Install tree: $ORIGIN so co-located plugins/libs find each other
set(CMAKE_INSTALL_RPATH "$ORIGIN")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

message("Physics ovruntime root dir: ${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory(source/common)
add_subdirectory(source/omni.convexdecomposition)
add_subdirectory(source/omni.physx.gpu)
add_subdirectory(source/omni.usdphysics)
add_subdirectory(source/omni.physx.cooking)
add_subdirectory(source/omni.physx)
add_subdirectory(source/omni.physx.fabric)
add_subdirectory(source/omni.physx.tensors)
add_subdirectory(source/omni.physics.tensors)

# Coverage: add gcov flush function to each shared library (see GcovFlush.cpp).
# Must come AFTER all add_subdirectory() calls so the targets exist.
if(OVRUNTIME_ENABLE_COVERAGE)
    set(_GCOV_FLUSH_SRC "${CMAKE_CURRENT_SOURCE_DIR}/source/common/source/utilities/GcovFlush.cpp")
    set(_GCOV_FLUSH_TARGETS
        OvruntimePhysX
        OvruntimePhysXCooking
        OvruntimePhysXFabric
        OvruntimePhysXTensors
        OvruntimePhysicsTensors
        OvruntimeUsdPhysics
    )
    foreach(_target IN LISTS _GCOV_FLUSH_TARGETS)
        if(TARGET ${_target})
            target_sources(${_target} PRIVATE "${_GCOV_FLUSH_SRC}")
            set_source_files_properties("${_GCOV_FLUSH_SRC}"
                TARGET_DIRECTORY ${_target}
                PROPERTIES SKIP_PRECOMPILE_HEADERS ON
            )
        endif()
    endforeach()
endif()

# Tests
if(OVRUNTIME_ENABLE_TESTS)
    enable_testing()
    add_subdirectory(source/omni.physx/tests)
    add_subdirectory(source/omni.physx.cooking/tests)
endif()

# Per-project optimization overrides (see cmake/UserOverrides.cmake.default)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/UserOverrides.cmake")
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/UserOverrides.cmake")
    if(OVRUNTIME_DEBUG_TARGETS)
        foreach(_target IN LISTS OVRUNTIME_DEBUG_TARGETS)
            if(TARGET ${_target})
                message(STATUS "UserOverride: disabling optimization for ${_target}")
                if(MSVC)
                    target_compile_options(${_target} PRIVATE /Od)
                else()
                    target_compile_options(${_target} PRIVATE -O0 -g)
                endif()
            else()
                message(WARNING "UserOverride: target '${_target}' not found")
            endif()
        endforeach()
    endif()
endif()

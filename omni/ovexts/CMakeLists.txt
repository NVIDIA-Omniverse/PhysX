# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

cmake_minimum_required(VERSION 3.22)

project(physics-ovexts)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake                  # ovexts-specific modules
    ${CMAKE_CURRENT_SOURCE_DIR}/../ovruntime/cmake      # ovruntime modules (UsdLinkDependencies, PhysXDependency, etc.)
    ${CMAKE_CURRENT_SOURCE_DIR}/../cmake                # shared omni/cmake modules (NvidiaBuildOptions, CompilerDefaults, etc.)
)

set(OVEXTS_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

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

# ============================================================================
# Dependency discovery (all from packman via _build/target-deps/)
# ============================================================================

# ovexts uses the same target-deps layout as ovruntime. Set OVRUNTIME_TARGET_DEPS
# so that ovruntime cmake modules (ExternalSystemIncludes, UsdLinkDependencies, etc.)
# can find dependencies in the ovexts tree.
set(OVEXTS_TARGET_DEPS "${CMAKE_CURRENT_SOURCE_DIR}/_build/target-deps")
set(OVRUNTIME_TARGET_DEPS "${OVEXTS_TARGET_DEPS}")

# Kit SDK directory — needed by ovruntime's ExternalSystemIncludes.cmake for
# Kit SDK headers (dev/include, dev/fabric/include). Must be set here (not
# only in ovruntime's CMakeLists.txt) because ovexts UX extensions also call
# ovruntime_add_external_system_includes() from the ovexts scope.
if(EXISTS "${OVEXTS_TARGET_DEPS}/kit_sdk_release")
    set(OVRUNTIME_KIT_SDK_DIR "${OVEXTS_TARGET_DEPS}/kit_sdk_release")
elseif(EXISTS "${OVEXTS_TARGET_DEPS}/kit_sdk_debug")
    set(OVRUNTIME_KIT_SDK_DIR "${OVEXTS_TARGET_DEPS}/kit_sdk_debug")
endif()

# ovruntime_deps (RTX/fabric plugins + ujitso/fabric/blobkey headers). Setting
# OVRUNTIME_DEPS_DIR here lets ovruntime's cooking build detect ujitso support
# and its test CMakeLists resolve plugin search paths — same as a standalone
# ovruntime build.
if(EXISTS "${OVEXTS_TARGET_DEPS}/ovruntime_deps_release")
    set(OVRUNTIME_DEPS_DIR "${OVEXTS_TARGET_DEPS}/ovruntime_deps_release")
    message(STATUS "Found ovruntime_deps: ${OVRUNTIME_DEPS_DIR}")
endif()

# Carbonite SDK
if(EXISTS "${OVEXTS_TARGET_DEPS}/carb_sdk_plugins/include")
    set(CARB_SDK_DIR "${OVEXTS_TARGET_DEPS}/carb_sdk_plugins")
    message(STATUS "Found Carbonite SDK: ${CARB_SDK_DIR}")
else()
    message(WARNING "Carbonite SDK not found. Run packman first.")
endif()

# PhysX SDK — use packman package unless building from source (--devphysx)
if(NOT OVEXTS_DEV_PHYSX)
    if(EXISTS "${OVEXTS_TARGET_DEPS}/physx")
        set(PHYSX_SDK_DIR "${OVEXTS_TARGET_DEPS}/physx")
        message(STATUS "Found PhysX SDK: ${PHYSX_SDK_DIR}")
    else()
        message(WARNING "PhysX SDK not found. Run packman first.")
    endif()
endif()

# USD
if(EXISTS "${OVEXTS_TARGET_DEPS}/usd")
    set(USD_DIR "${OVEXTS_TARGET_DEPS}/usd")
    message(STATUS "Found USD: ${USD_DIR}")
else()
    message(WARNING "USD not found. Run packman first.")
endif()

# CUDA
if(EXISTS "${OVEXTS_TARGET_DEPS}/cuda")
    set(CUDA_DIR "${OVEXTS_TARGET_DEPS}/cuda")
    message(STATUS "Found CUDA: ${CUDA_DIR}")
else()
    message(WARNING "CUDA not found. Run packman first.")
endif()

# Python
set(PYTHON_DIR "${OVEXTS_TARGET_DEPS}/python")
if(EXISTS "${PYTHON_DIR}")
    message(STATUS "Found Python: ${PYTHON_DIR}")
    if(WIN32)
        set(OVEXTS_PYTHON_EXE "${PYTHON_DIR}/python.exe")
        link_directories(${PYTHON_DIR}/libs)
    else()
        set(OVEXTS_PYTHON_EXE "${PYTHON_DIR}/bin/python3")
    endif()
    # Query the platform-specific Python extension suffix (e.g. .cpython-312-x86_64-linux-gnu.so)
    # Used by ovexts_add_python_bindings() so that repo stubgen can discover the bindings.
    execute_process(
        COMMAND "${OVEXTS_PYTHON_EXE}" -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'))"
        OUTPUT_VARIABLE OVEXTS_PYTHON_EXT_SUFFIX
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(OVEXTS_PYTHON_EXT_SUFFIX)
        message(STATUS "Python extension suffix: ${OVEXTS_PYTHON_EXT_SUFFIX}")
    elseif(WIN32)
        set(OVEXTS_PYTHON_EXT_SUFFIX ".pyd")
    else()
        set(OVEXTS_PYTHON_EXT_SUFFIX ".so")
    endif()
else()
    message(WARNING "Python not found. Run packman first.")
endif()

# pybind11
if(EXISTS "${OVEXTS_TARGET_DEPS}/pybind11/include")
    set(PYBIND11_DIR "${OVEXTS_TARGET_DEPS}/pybind11")
    message(STATUS "Found pybind11: ${PYBIND11_DIR}")
else()
    message(WARNING "pybind11 not found. Run packman first.")
endif()

# Physics schema
include(Options)

if(OVEXTS_DEV_SCHEMA)
    if(WIN32)
        set(_SCHEMA_PLATFORM "windows-x86_64")
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
        set(_SCHEMA_PLATFORM "linux-aarch64")
    else()
        set(_SCHEMA_PLATFORM "linux-x86_64")
    endif()
    set(USD_EXT_PHYSICS_DIR "${OVEXTS_ROOT_DIR}/../schema/_build/${_SCHEMA_PLATFORM}/release/schema")
    message(STATUS "DevSchema mode: using local physics schema: ${USD_EXT_PHYSICS_DIR}")
else()
    # Physics schema: headers are identical across debug/release, but native
    # libraries (_physxSchema.pyd etc.) must match the USD runtime config.
    # USD_EXT_PHYSICS_DIR points to the release package for headers/includes
    # (used by ExternalSystemIncludes and EXISTS checks).
    # USD_EXT_PHYSICS_BASE is the parent for config-dependent lib/runtime paths.
    set(USD_EXT_PHYSICS_BASE "${OVEXTS_TARGET_DEPS}/usd_ext_physics")
    if(EXISTS "${USD_EXT_PHYSICS_BASE}/release/include")
        set(USD_EXT_PHYSICS_DIR "${USD_EXT_PHYSICS_BASE}/release")
        message(STATUS "Found physics schema: ${USD_EXT_PHYSICS_DIR}")
    else()
        message(WARNING "Physics schema not found. Run packman first.")
    endif()
endif()

# ============================================================================
# Compiler and build settings (shared modules)
# ============================================================================

include(NvidiaBuildOptions)         # CRT, output dirs, platform naming
include(CarbPlatform)              # CARB_PLATFORM_DIR for Carbonite SDK lib path
# OVRUNTIME_DEV_PHYSX must be set before PhysXDependency so it can build PhysX
# from source and set PHYSX_SDK_DIR to the source tree instead of packman.
set(OVRUNTIME_DEV_PHYSX ${OVEXTS_DEV_PHYSX} CACHE BOOL "" FORCE)
include(PhysXDependency)           # PhysX SDK: source build (--devphysx) or packman
# CompilerDefaults is included AFTER add_subdirectory(ovruntime) so that -Werror
# does not propagate into the PhysX SDK source tree when building with --devphysx.

# GCC false-positive warning suppressions (same as ovruntime).
# These must be before add_subdirectory(ovruntime) so ovruntime targets inherit them.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-uninitialized>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-maybe-uninitialized>
    )
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "12")
        add_compile_options(
            $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-dangling-pointer>
            $<$<COMPILE_LANGUAGE:CXX>:-Wno-changes-meaning>
            $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-array-bounds>
            $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-stringop-overread>
            $<$<COMPILE_LANGUAGE:CXX>:-Wno-dangling-reference>
        )
    endif()
endif()

include(ExternalSystemIncludes)    # ovruntime_add_external_system_includes()
include(UsdLinkDependencies)       # ovruntime_link_usd_deps(), ovruntime_link_bindings_deps()
include(CudaUtils)                 # CUDA compilation helpers

# Carbonite config mapping (checked → release)
set(OVEXTS_CARB_CONFIG "$<IF:$<CONFIG:checked>,release,$<LOWER_CASE:$<CONFIG>>>")

# Warning flags are added AFTER the ovruntime subproject (see below) so that
# -Werror does not propagate into PhysX SDK when building with --devphysx.
# The ovruntime subproject manages its own warning flags independently.

# C++ standard and position-independent code
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Configuration types
if(CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)
endif()

# Ensure _DEBUG / NDEBUG are defined correctly
add_compile_definitions($<$<CONFIG:Debug>:_DEBUG>)
add_compile_definitions($<$<CONFIG:checked>:NDEBUG>)

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

# RPATH
set(CMAKE_INSTALL_RPATH "$ORIGIN")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# ============================================================================
# Kit SDK discovery
# ============================================================================

# Compute the platform-specific output bin directory (matches premake targetDir)
if(WIN32)
    set(_PLATFORM_ARCH "windows-x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(_PLATFORM_ARCH "linux-aarch64")
else()
    set(_PLATFORM_ARCH "linux-x86_64")
endif()

if(CMAKE_CONFIGURATION_TYPES)
    # Multi-config generators (Visual Studio): use a genex-based path for target
    # output directories and file(GENERATE), and a concrete per-config list for
    # configure-time operations (file copies, symlinks).
    set(OVEXTS_BIN_DIR "${OVEXTS_ROOT_DIR}/_build/${_PLATFORM_ARCH}/$<LOWER_CASE:$<CONFIG>>")
    set(OVEXTS_BIN_DIR_CONFIGS "")
    foreach(_cfg IN LISTS CMAKE_CONFIGURATION_TYPES)
        string(TOLOWER "${_cfg}" _cfg_lower)
        list(APPEND OVEXTS_BIN_DIR_CONFIGS "${OVEXTS_ROOT_DIR}/_build/${_PLATFORM_ARCH}/${_cfg_lower}")
    endforeach()
else()
    string(TOLOWER "${CMAKE_BUILD_TYPE}" _config_lower)
    set(OVEXTS_BIN_DIR "${OVEXTS_ROOT_DIR}/_build/${_PLATFORM_ARCH}/${_config_lower}")
    set(OVEXTS_BIN_DIR_CONFIGS "${OVEXTS_BIN_DIR}")
endif()

# Extension output directories (match premake extsPhysics_dir / extsPhysics_repo_dir)
set(OVEXTS_EXTS_PHYSICS_DIR "${OVEXTS_BIN_DIR}/extsPhysics")
set(OVEXTS_EXTS_PHYSICS_REPO_DIR "${OVEXTS_BIN_DIR}/extsPhysicsRepo")

include(KitSdkDependency)

# ============================================================================
# ovruntime as CMake subproject
# ============================================================================

set(OVRUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ovruntime")

# Configure ovruntime options
set(OVRUNTIME_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
set(OVRUNTIME_ENABLE_PYTHON_BINDINGS ON CACHE BOOL "" FORCE)
set(OVRUNTIME_DEV_SCHEMA ${OVEXTS_DEV_SCHEMA} CACHE BOOL "" FORCE)
set(OVRUNTIME_DEV_PHYSX ${OVEXTS_DEV_PHYSX} CACHE BOOL "" FORCE)

# RPATH: prepend $ORIGIN so build-tree .so files find siblings
set(_SAVED_BUILD_RPATH "${CMAKE_BUILD_RPATH}")
set(CMAKE_BUILD_RPATH "$ORIGIN;${CMAKE_BUILD_RPATH}")
add_subdirectory("${OVRUNTIME_DIR}" "${CMAKE_BINARY_DIR}/ovruntime" EXCLUDE_FROM_ALL)
set(CMAKE_BUILD_RPATH "${_SAVED_BUILD_RPATH}")

# ovruntime include directories (used by UX extensions)
set(OVEXTS_OVRUNTIME_INCLUDE_DIR "${OVRUNTIME_DIR}/include")
set(OVEXTS_OVRUNTIME_COMMON_INCLUDE_DIR "${OVRUNTIME_DIR}/source/common/include")
set(OVEXTS_OVRUNTIME_PCH_DIR "${OVRUNTIME_DIR}/source/pch")

# ============================================================================
# Umbrella as CMake subproject
# ============================================================================

set(UMBRELLA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../umbrella")
set(OVEXTS_UMBRELLA_INCLUDE_DIR "${UMBRELLA_DIR}/include")

# Configure umbrella options — disable tests (ovexts has its own test runner)
set(UMBRELLA_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
set(UMBRELLA_ENABLE_PYTHON_BINDINGS ON CACHE BOOL "" FORCE)

# Share ovexts target-deps so umbrella doesn't need its own packman pull
set(UMBRELLA_TARGET_DEPS "${OVEXTS_TARGET_DEPS}")

add_subdirectory("${UMBRELLA_DIR}" "${CMAKE_BINARY_DIR}/umbrella" EXCLUDE_FROM_ALL)

include(CompilerDefaults)          # compiler warning flags, optimizations (-Werror)

# Additional warning suppressions matching premake5.lua and premake5-public.lua.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-switch>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-unused-parameter>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-unused-variable>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-strict-aliasing>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-sign-compare>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-deprecated-declarations>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-deprecated-copy>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-unused-but-set-variable>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-reorder>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-unused-function>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-ignored-qualifiers>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-class-memaccess>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-enum-compare>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-missing-field-initializers>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-range-loop-construct>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-nonnull-compare>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-maybe-uninitialized>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-delete-incomplete>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-narrowing>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-attributes>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-parentheses>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-format>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-format-security>
    )
elseif(MSVC)
    add_compile_options(
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4100>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4189>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4244>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4267>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4305>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4456>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4457>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4458>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4505>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4701>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4703>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4702>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4996>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4018>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4245>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4389>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4127>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4373>
        $<$<COMPILE_LANGUAGE:C,CXX>:/wd4101>
    )
endif()

# ============================================================================
# ovexts helpers
# ============================================================================

include(OvextsHelpers)
include(OgnGenerate)
include(TestRunnerScripts)
include(AppRunnerScripts)

# ============================================================================
# Extensions
# ============================================================================

message(STATUS "Physics ovexts root dir: ${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory(extensions/runtime)

add_subdirectory(extensions/ux)

# ============================================================================
# App launcher scripts and shared content
# ============================================================================

# PACKAGE-LICENSES — repo package (bloky mode) expects this at _build/PACKAGE-LICENSES.
# OGN output — repo package expects _build/ogn but CMake generates into the compiler dir.
# These are not config-dependent.  Remove stale junctions before creating new ones
# (a junction from a previous build may point to a deleted directory).
foreach(_link_name "PACKAGE-LICENSES" "ogn")
    set(_link_path "${CMAKE_CURRENT_SOURCE_DIR}/_build/${_link_name}")
    if(IS_SYMLINK "${_link_path}" OR IS_DIRECTORY "${_link_path}")
        if(WIN32)
            file(TO_NATIVE_PATH "${_link_path}" _native_rm)
            execute_process(COMMAND cmd /c rmdir "${_native_rm}" ERROR_QUIET)
        else()
            file(REMOVE "${_link_path}")
        endif()
    endif()
endforeach()
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ogn")
ovexts_prebuild_link("${CMAKE_BINARY_DIR}/ogn"
                     "${CMAKE_CURRENT_SOURCE_DIR}/_build/ogn")

# Symlink apps/ directory into the build output
ovexts_prebuild_link("${CMAKE_CURRENT_SOURCE_DIR}/apps" "${OVEXTS_BIN_DIR}/apps")


# Bloky app launchers (and VS app projects)
ovexts_define_app("omni.bloky.physx.kit"
    EXTRA_ARGS "--portable --/renderer/debug/validation/enabled=false")

# Set omni.bloky.physx.kit as the default startup project in Visual Studio
if(CMAKE_GENERATOR MATCHES "Visual Studio")
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT omni_bloky_physx_kit_app)
endif()

# ============================================================================
# Install rules (omni_physics package)
# ============================================================================
# Populates CMAKE_INSTALL_PREFIX (_install/ by default) with the public headers,
# dependency descriptors, licenses, and premake helpers that form the
# omni_physics package.  package.toml then sources everything from _install/.
#
# Run:  cmake --install <build-dir>

if(NOT CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    # User explicitly set a prefix — respect it.
else()
    set(CMAKE_INSTALL_PREFIX "${OVEXTS_ROOT_DIR}/_install" CACHE PATH "" FORCE)
endif()

# Public headers — ovruntime
install(DIRECTORY "${OVRUNTIME_DIR}/include/omni/physx/"
        DESTINATION "include/omni/physx")
install(DIRECTORY "${OVRUNTIME_DIR}/include/omni/convexdecomposition/"
        DESTINATION "include/omni/convexdecomposition")
install(DIRECTORY "${OVRUNTIME_DIR}/include/omni/physics/"
        DESTINATION "include/omni/physics")

# Public headers — umbrella
install(DIRECTORY "${UMBRELLA_DIR}/include/omni/physics/"
        DESTINATION "include/omni/physics")

# Packman dependency descriptors
install(FILES "${OVEXTS_ROOT_DIR}/deps/target-deps.packman.xml"
              "${OVEXTS_ROOT_DIR}/deps/schema-deps.packman.xml"
        DESTINATION "deps")

# Licenses

# Premake5 public helpers (consumed by schema build)
install(FILES "${OVEXTS_ROOT_DIR}/premake5-public.lua"
        DESTINATION ".")

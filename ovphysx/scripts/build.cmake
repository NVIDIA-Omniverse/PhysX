# ovphysx Build Script (Cross-Platform)
# Usage: cmake [options] -P scripts/build.cmake
#
# Options (passed via -D flags):
#   -DCLEAN_BUILD=ON|OFF          Clean build directory (default: OFF)
#   -DCLEAN_ONLY=ON|OFF           Exit after cleaning, skip build (default: ON when CLEAN_BUILD)
#   -DBUILD_TYPE=Debug|Release    Build type (default: Release)
#   -DVERBOSE=ON|OFF              Verbose build output (default: OFF)
#   -DDEV_PHYSX=ON|OFF            Build PhysX SDK from source (default: OFF)
#   -DDEV_SCHEMA=ON|OFF           Use locally-built namespaced physics schema (default: OFF).
#   -DBENCHMARKS=ON|OFF           Build the opt-in benchmark suite (default: OFF)
#   -DOVPHYSX_USE_RELEASE_RUNTIME_DEPS=ON|OFF
#                                   Use release Packman runtime deps for Debug builds
#                                   (required for Debug; OVStage runtime is Release-only)
#   -DGENERATE_ONLY=ON|OFF        Configure only, skip build (default: OFF)
#   -DBUILD_TARGET=<name>         Build a specific CMake target (default: all)
#
# Examples:
#   cmake -DCLEAN_BUILD=ON -DBUILD_TYPE=Debug -P scripts/build.cmake
#   cmake -DDEV_PHYSX=ON -P scripts/build.cmake
#
# For SDK installation (required for C++ sample tests and packaging):
#   cmake -P scripts/install.cmake
# For release packaging (after install):
#   cmake -P scripts/package_sdk.cmake
# The combined PhysX + ovphysx open-source tree is produced by the
# release staging flow, not by this local build script.

cmake_minimum_required(VERSION 3.16)

# Get script directory and project root
get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
get_filename_component(PROJECT_ROOT "${SCRIPT_DIR}/.." ABSOLUTE)
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")
include("${SCRIPT_DIR}/build_common.cmake")
include("${SCRIPT_DIR}/host_path_utils.cmake")

# Default values
if(NOT DEFINED BUILD_DIR)
    set(BUILD_DIR "_build")
endif()

if(NOT DEFINED CLEAN_BUILD)
    set(CLEAN_BUILD OFF)
endif()

if(NOT DEFINED VERBOSE)
    set(VERBOSE OFF)
endif()

if(NOT DEFINED GENERATE_ONLY)
    set(GENERATE_ONLY OFF)
endif()

if(NOT DEFINED BUILD_TARGET)
    set(BUILD_TARGET "")
endif()

if(NOT DEFINED DEV_SCHEMA)
    set(DEV_SCHEMA OFF)
endif()

# Parallel job count: -DJOBS=N overrides, otherwise core+RAM-aware auto-detect
# (shared helper in build_common.cmake; see there for the bounding rationale).
ovphysx_compute_build_jobs(JOBS)

message(STATUS "ovphysx Build")
message(STATUS "===================")
message(STATUS "Project root: ${PROJECT_ROOT}")
message(STATUS "Build type: ${BUILD_TYPE}")
message(STATUS "Build directory: ${BUILD_DIR}")
message(STATUS "Platform: ${CMAKE_HOST_SYSTEM_NAME}")
message(STATUS "Parallel jobs: ${JOBS}")
message(STATUS "USD mode: namespaced")

# Set build path
set(BUILD_PATH "${PROJECT_ROOT}/${BUILD_DIR}")
set(OVPHYSX_DEV_SCHEMA_STAMP "${BUILD_PATH}/ovphysx_dev_schema.stamp")

# Sibling ovruntime repository (included as CMake subproject, not built separately)
set(OVRUNTIME_DIR "${PROJECT_ROOT}/ovruntime")
if(NOT EXISTS "${OVRUNTIME_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "Sibling ovruntime repository not found at: ${OVRUNTIME_DIR}")
endif()


# Clean build if requested
if(CLEAN_BUILD)
    message(STATUS "Cleaning build directories and artifacts...")

    # Clean ovruntime build artifacts directly (don't call build.sh -x which also rebuilds)
    file(GLOB OVRUNTIME_UNDERSCORE_DIRS "${OVRUNTIME_DIR}/_*")
    foreach(_DIR ${OVRUNTIME_UNDERSCORE_DIRS})
        if(IS_DIRECTORY "${_DIR}")
            message(STATUS " Removing ovruntime: ${_DIR}")
            file(REMOVE_RECURSE "${_DIR}")
        endif()
    endforeach()

    # Find and remove common build/cache directories. Cmake globbing doesn't work well with folders, so we do it manually.
    file(GLOB_RECURSE ALL_ITEMS LIST_DIRECTORIES true "${PROJECT_ROOT}/*")
    set(DIRS_TO_REMOVE "")
    foreach(ITEM ${ALL_ITEMS})
        if(IS_DIRECTORY "${ITEM}")
            get_filename_component(DIR_NAME "${ITEM}" NAME)
            if(DIR_NAME STREQUAL "__pycache__" OR 
               DIR_NAME STREQUAL ".venv" OR 
               DIR_NAME STREQUAL "_build" OR
               DIR_NAME STREQUAL "_install" OR
               DIR_NAME STREQUAL "_repo" OR
               DIR_NAME STREQUAL "_dist")
                list(APPEND DIRS_TO_REMOVE "${ITEM}")
            endif()
        endif()
    endforeach()
    # Remove duplicates and sort.
    if(DIRS_TO_REMOVE)
        list(REMOVE_DUPLICATES DIRS_TO_REMOVE)
        list(SORT DIRS_TO_REMOVE)
        
        # Filter out subdirectories whose parents are already being removed
        set(FILTERED_DIRS "")
        foreach(DIR ${DIRS_TO_REMOVE})
            set(IS_SUBDIR FALSE)
            foreach(PARENT ${FILTERED_DIRS})
                # Check if DIR is a subdirectory of PARENT
                string(FIND "${DIR}" "${PARENT}/" SUBDIR_POS)
                if(SUBDIR_POS EQUAL 0)
                    set(IS_SUBDIR TRUE)
                    break()
                endif()
            endforeach()
            if(NOT IS_SUBDIR)
                list(APPEND FILTERED_DIRS "${DIR}")
            endif()
        endforeach()
        
        foreach(DIR ${FILTERED_DIRS})
            message(STATUS " Removing: ${DIR}")
            file(REMOVE_RECURSE "${DIR}")
        endforeach()
    endif()

    # Clean transient packaging-lock snapshots generated on lock mismatches.
    # Keep canonical lock files (without _new suffix) untouched.
    set(PACKAGING_LOCK_DIR "${PROJECT_ROOT}/packaging_lock")
    if(IS_DIRECTORY "${PACKAGING_LOCK_DIR}")
        file(GLOB PACKAGING_LOCK_NEW_FILES "${PACKAGING_LOCK_DIR}/*_new.json")
        foreach(LOCK_NEW_FILE ${PACKAGING_LOCK_NEW_FILES})
            message(STATUS " Removing packaging lock snapshot: ${LOCK_NEW_FILE}")
            file(REMOVE "${LOCK_NEW_FILE}")
        endforeach()
    endif()

    message(STATUS "Build directories and artifacts cleaned.")
    if(NOT DEFINED CLEAN_ONLY)
        set(CLEAN_ONLY ON)
    endif()
    if(CLEAN_ONLY)
        message(STATUS "Clean-only mode, exiting.")
        return()
    endif()
endif()

# Start overall build timer
string(TIMESTAMP _BUILD_START_TS "%s")

# Initialize repo and generate build system (if not already done)
message(STATUS "  Initializing build system (generate + fetch dependencies)...")

# Create build directory
message(STATUS "Creating build directory...")
file(MAKE_DIRECTORY "${BUILD_PATH}")
file(REMOVE "${OVPHYSX_DEV_SCHEMA_STAMP}")

# Use centralized cross-platform fetch script
execute_process(
    COMMAND ${CMAKE_COMMAND}
            "-DCONFIG=${OVPHYSX_RUNTIME_DEPS_CONFIG}"
            -P "${SCRIPT_DIR}/fetch_deps.cmake"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    RESULT_VARIABLE FETCH_DEPS_RESULT
)
if(NOT FETCH_DEPS_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to download packman dependencies (exit code: ${FETCH_DEPS_RESULT})")
endif()

# Use target-deps Python (3.12) for scripts that need tomllib
# Note: packman's bootstrap Python (tools/packman/python.bat) is older and lacks tomllib
if(WIN32)
    set(TARGET_PYTHON "${PROJECT_ROOT}/_build/target-deps/python/python.exe")
else()
    set(TARGET_PYTHON "${PROJECT_ROOT}/_build/target-deps/python/bin/python3")
endif()

if(NOT EXISTS "${TARGET_PYTHON}")
    message(FATAL_ERROR "Target Python not found at: ${TARGET_PYTHON}\n"
                        "Run scripts/fetch_deps.bat (Windows) or scripts/fetch_deps.sh (Linux) first.")
endif()

# ============================================================================
# Fetch ovruntime dependencies (replaces build.sh invocation)
# ============================================================================

message(STATUS "")
message(STATUS "Pulling ovruntime dependencies...")

set(OVRUNTIME_PULL_DEPS "${OVRUNTIME_DIR}/pull_dependencies${SCRIPT_SUFFIX}")
set(_OVRUNTIME_PULL_CONFIG "${OVPHYSX_RUNTIME_DEPS_CONFIG}")
set(_OVRUNTIME_PULL_ARGS "--config" "${_OVRUNTIME_PULL_CONFIG}")
if(DEV_PHYSX)
    list(APPEND _OVRUNTIME_PULL_ARGS "--devphysx")
endif()
execute_process(
    COMMAND "${OVRUNTIME_PULL_DEPS}" ${_OVRUNTIME_PULL_ARGS}
    WORKING_DIRECTORY "${OVRUNTIME_DIR}"
    RESULT_VARIABLE PULL_DEPS_RESULT
)
if(NOT PULL_DEPS_RESULT STREQUAL "0")
    message(FATAL_ERROR "Failed to pull ovruntime dependencies (exit code: ${PULL_DEPS_RESULT})")
endif()
# On Windows with --devphysx, PhysX's CMake needs PM_SECURELOADLIBRARY_PATH to
# find nvSecureLoadLibrary.c.  pull_dependencies.bat runs packman pull on
# physx/dependencies.xml which downloads the package, but packman only sets
# PM_*_PATH env vars for deps with a linkPath — SecureLoadLibrary has none.
# Locate it directly in the packman cache.
if(DEV_PHYSX AND WIN32)
    file(GLOB _SLL_CANDIDATES "$ENV{PM_PACKAGES_ROOT}/chk/SecureLoadLibrary/*/src/nvSecureLoadLibrary.c")
    if(_SLL_CANDIDATES)
        list(GET _SLL_CANDIDATES 0 _SLL_FILE)
        get_filename_component(_SLL_PATH "${_SLL_FILE}" DIRECTORY)  # .../src
        get_filename_component(_SLL_PATH "${_SLL_PATH}" DIRECTORY)  # .../1.0.xxx
        set(ENV{PM_SECURELOADLIBRARY_PATH} "${_SLL_PATH}")
        message(STATUS "  Set PM_SECURELOADLIBRARY_PATH=${_SLL_PATH}")
    endif()
endif()
message(STATUS "  [OK] ovruntime dependencies ready")

# Make sure the pulled schema package is codeless (no native lib, plugInfo is a
# resource plugin). A codefull package regressing back in would reintroduce the
# USD ABI hazard the codeless migration removed.
if(NOT DEV_SCHEMA)
    # usd_ext_physics is a single config-neutral package staged at a flat path;
    # both release and debug builds consume the same tree.
    set(_SCHEMA_VERIFY_DIR "${OVRUNTIME_DIR}/_build/target-deps/usd_ext_physics")
    if(EXISTS "${_SCHEMA_VERIFY_DIR}")
        message(STATUS "Verifying schema package (codeless) at ${_SCHEMA_VERIFY_DIR}...")
        execute_process(
            COMMAND "${TARGET_PYTHON}" "${SCRIPT_DIR}/verify_schema_package.py"
                    "--dir=${_SCHEMA_VERIFY_DIR}"
            WORKING_DIRECTORY "${PROJECT_ROOT}"
            RESULT_VARIABLE _SCHEMA_VERIFY_RESULT
        )
        if(NOT _SCHEMA_VERIFY_RESULT EQUAL 0)
            message(FATAL_ERROR
                "Schema package at ${_SCHEMA_VERIFY_DIR} is not a valid codeless schema. "
                "Delete _build/target-deps and re-pull, or update the schema pin in "
                "ovphysx/ovruntime/deps/schema-deps.packman.xml.")
        endif()
        message(STATUS "  [OK] schema package verified as codeless")
    else()
        message(FATAL_ERROR
            "Schema package directory not found at ${_SCHEMA_VERIFY_DIR}. "
            "Delete _build/target-deps and re-pull, or check ovphysx/ovruntime/deps/schema-deps.packman.xml.")
    endif()
endif()

# ============================================================================
# Build local physics schema when requested (--devschema / -DDEV_SCHEMA=ON)
# ============================================================================
# Namespaced ovphysx uses the prebuilt namespaced-monolithic usd_ext_physics
# packman package by default (pulled above via ovruntime's schema-deps
# manifest). --devschema is an explicit override for schema
# developers who want to iterate on schemas/physx locally; it rebuilds the
# schema against the USD we just pulled and replaces the packman package
# as ovruntime's USD_EXT_PHYSICS_DIR source.
if(DEV_SCHEMA)
    set(SCHEMA_DIR "${PROJECT_ROOT}/../schemas/physx")
    set(SCHEMA_BUILD_DIR "${SCHEMA_DIR}/_build/schema")
    set(SCHEMA_HEADER "${SCHEMA_BUILD_DIR}/include/physxSchema/tokens.h")
    set(_SCHEMA_NEEDS_BUILD TRUE)

    if(EXISTS "${SCHEMA_HEADER}")
        execute_process(
            COMMAND "${TARGET_PYTHON}" "${SCRIPT_DIR}/verify_schema_package.py"
                    "--dir=${SCHEMA_BUILD_DIR}"
            WORKING_DIRECTORY "${PROJECT_ROOT}"
            RESULT_VARIABLE _LOCAL_SCHEMA_VERIFY_RESULT
        )
        if(_LOCAL_SCHEMA_VERIFY_RESULT EQUAL 0)
            set(_SCHEMA_NEEDS_BUILD FALSE)
        else()
            message(STATUS "Local physics schema is missing or not codeless; rebuilding.")
        endif()
    endif()

    if(_SCHEMA_NEEDS_BUILD)
        message(STATUS "")
        message(STATUS "Building local physics schema...")
        # build.sh/build.bat pull deps, regenerate codeless artifacts, and stage
        # them to _build/schema (no repo_build/repo_usd/premake).
        execute_process(
            COMMAND "${SCHEMA_DIR}/build${SCRIPT_SUFFIX}"
            WORKING_DIRECTORY "${SCHEMA_DIR}"
            RESULT_VARIABLE _SCHEMA_BUILD_RESULT
        )
        if(NOT _SCHEMA_BUILD_RESULT STREQUAL "0")
            message(FATAL_ERROR "Failed to build local schema (exit code: ${_SCHEMA_BUILD_RESULT})")
        endif()
        execute_process(
            COMMAND "${TARGET_PYTHON}" "${SCRIPT_DIR}/verify_schema_package.py"
                    "--dir=${SCHEMA_BUILD_DIR}"
            WORKING_DIRECTORY "${PROJECT_ROOT}"
            RESULT_VARIABLE _LOCAL_SCHEMA_VERIFY_RESULT
        )
        if(NOT _LOCAL_SCHEMA_VERIFY_RESULT EQUAL 0)
            message(FATAL_ERROR "Local physics schema at ${SCHEMA_BUILD_DIR} is not a valid codeless schema.")
        endif()
        message(STATUS "  [OK] local physics schema built")
    else()
        message(STATUS "  [OK] local physics schema already built (codeless)")
    endif()
    file(WRITE "${OVPHYSX_DEV_SCHEMA_STAMP}" "ON\n")
endif()

# ============================================================================
# Setup packaged toolchain for ovphysx build (Windows only)
# ============================================================================
# ovphysx uses CMake (Ninja or VS generator) which discovers the compiler via
# PATH and environment variables. This section configures those from the
# packaged MSVC/WinSDK (imported via host-deps.packman.xml from ovruntime).
#
# ovruntime does not need this — its premake/MSBuild toolchain is configured
# by premake5-public.lua and repo_build independently.
#
# Generator selection via GENERATOR environment variable:
#   GENERATOR=ninja  - Ninja + packaged cl.exe (CLI, no .sln, no VS needed)
#   GENERATOR=vs     - VS generator using packaged MSVC as portable instance
#                      (CMake 3.23+). Produces .sln. No local VS needed.
#                      CLI build uses packaged MSBuild. SDK paths redirected
#                      to packaged WinSDK. Devs can also open .sln in local VS IDE.
#   (not set)        - Auto-detect: if local VS found via vswhere -> vs, else -> ninja
#
# Backward compat: USE_NINJA_GENERATOR=1 is mapped to GENERATOR=ninja.
#

if(WIN32)
    message(STATUS "")
    message(STATUS "Setting up toolchain...")

    # ---- Packaged tool paths (fetched by packman in deps/host-deps.packman.xml) ----
    set(PACKAGED_MSVC "${PROJECT_ROOT}/_build/host-deps/msvc")
    set(PACKAGED_WINSDK "${PROJECT_ROOT}/_build/host-deps/winsdk")
    set(PACKAGED_NINJA_DIR "${PROJECT_ROOT}/_build/host-deps/ninja")
    set(PACKAGED_VSWHERE "${PROJECT_ROOT}/_build/host-deps/vswhere/VsWhere.exe")

    # vswhere: prefer the packaged copy, else the VS installer's canonical one.
    set(VSWHERE "${PACKAGED_VSWHERE}")
    if(NOT EXISTS "${VSWHERE}")
        set(VSWHERE "$ENV{ProgramFiles\(x86\)}/Microsoft Visual Studio/Installer/vswhere.exe")
    endif()

    # ---- Toolchain mode ----
    # Internal checkouts pull the packaged MSVC/WinSDK via packman. The public
    # source drop does not ship them (non-redistributable, not on the public
    # remotes), so fall back to a local Visual Studio installation there.
    if(EXISTS "${PACKAGED_MSVC}/VC/Tools/MSVC")
        set(TOOLCHAIN_MODE "packaged")
        set(MSVC_ROOT "${PACKAGED_MSVC}")
        set(WINSDK_ROOT "${PACKAGED_WINSDK}")
    else()
        set(TOOLCHAIN_MODE "local")
        # OVPHYSX_VS_ROOT pins a specific installation; otherwise pick the
        # newest supported VS (2022, then 2019), falling back to the absolute
        # newest installation.
        if(DEFINED ENV{OVPHYSX_VS_ROOT} AND EXISTS "$ENV{OVPHYSX_VS_ROOT}/VC/Tools/MSVC")
            set(MSVC_ROOT "$ENV{OVPHYSX_VS_ROOT}")
            message(STATUS "  Using OVPHYSX_VS_ROOT override: ${MSVC_ROOT}")
        else()
            if(NOT EXISTS "${VSWHERE}")
                message(FATAL_ERROR "Packaged MSVC not present and vswhere.exe not found.\n"
                                    "Install Visual Studio 2019/2022 with the 'Desktop development with C++' workload.")
            endif()
            # NOTE: list elements must not carry the surrounding [..) brackets —
            # an unbalanced '[' makes CMake's list parser swallow the ';'
            # separators, collapsing the list into one garbage element. The
            # brackets are added when the vswhere argument is composed.
            set(_vs_ranges "17.0,18.0" "16.0,17.0")
            # Last resort: newest of anything (GENERATOR=ninja works with any VS).
            list(APPEND _vs_ranges "16.0,")
            set(MSVC_ROOT "")
            foreach(_vs_range IN LISTS _vs_ranges)
                execute_process(
                    COMMAND "${VSWHERE}" -latest -products * -version "[${_vs_range})"
                            -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64
                            -property installationPath
                    OUTPUT_VARIABLE MSVC_ROOT
                    OUTPUT_STRIP_TRAILING_WHITESPACE
                )
                if(MSVC_ROOT AND EXISTS "${MSVC_ROOT}/VC/Tools/MSVC")
                    break()
                endif()
            endforeach()
            if(NOT MSVC_ROOT OR NOT EXISTS "${MSVC_ROOT}/VC/Tools/MSVC")
                message(FATAL_ERROR "Packaged MSVC not present and no local Visual Studio installation found.\n"
                                    "Install Visual Studio 2019/2022 with the 'Desktop development with C++' workload.")
            endif()
            message(STATUS "  Using local Visual Studio: ${MSVC_ROOT}")
        endif()
        # Local WinSDK (installed with the VS C++ workload).
        if(DEFINED ENV{WindowsSdkDir} AND EXISTS "$ENV{WindowsSdkDir}/Include")
            set(WINSDK_ROOT "$ENV{WindowsSdkDir}")
        else()
            set(WINSDK_ROOT "$ENV{ProgramFiles\(x86\)}/Windows Kits/10")
        endif()
        if(NOT EXISTS "${WINSDK_ROOT}/Include")
            message(FATAL_ERROR "Windows SDK not found at ${WINSDK_ROOT}.\n"
                                "Install the Windows 10/11 SDK (ships with the VS C++ workload).")
        endif()
    endif()
    set(MSBUILD_EXE "${MSVC_ROOT}/MSBuild/Current/Bin/MSBuild.exe")

    # ---- Discover MSVC tools version (e.g., 14.29.30133); use the newest ----
    file(GLOB MSVC_TOOLS_DIRS "${MSVC_ROOT}/VC/Tools/MSVC/*")
    if(NOT MSVC_TOOLS_DIRS)
        message(FATAL_ERROR "MSVC tools not found in ${MSVC_ROOT}/VC/Tools/MSVC/")
    endif()
    list(SORT MSVC_TOOLS_DIRS)
    list(GET MSVC_TOOLS_DIRS -1 MSVC_TOOLS_DIR)
    message(STATUS "  Found MSVC tools: ${MSVC_TOOLS_DIR}")

    # ---- Discover WinSDK layout (flat vs versioned) ----
    if(EXISTS "${WINSDK_ROOT}/Include/ucrt")
        # Flat structure (packaged WinSDK)
        set(WINSDK_INCLUDE_BASE "${WINSDK_ROOT}/Include")
        set(WINSDK_LIB_BASE "${WINSDK_ROOT}/Lib")
        set(WINSDK_BIN_BASE "${WINSDK_ROOT}/bin")
        message(STATUS "  Found WinSDK (flat structure)")
    else()
        # Versioned structure (full WinSDK installation); use the newest
        file(GLOB WINSDK_VERSION_DIRS "${WINSDK_ROOT}/Include/10.*")
        if(NOT WINSDK_VERSION_DIRS)
            message(FATAL_ERROR "WinSDK include not found in ${WINSDK_ROOT}/Include/")
        endif()
        list(SORT WINSDK_VERSION_DIRS)
        list(GET WINSDK_VERSION_DIRS -1 WINSDK_VERSION_DIR)
        get_filename_component(WINSDK_VERSION "${WINSDK_VERSION_DIR}" NAME)
        set(WINSDK_INCLUDE_BASE "${WINSDK_ROOT}/Include/${WINSDK_VERSION}")
        set(WINSDK_LIB_BASE "${WINSDK_ROOT}/Lib/${WINSDK_VERSION}")
        set(WINSDK_BIN_BASE "${WINSDK_ROOT}/bin/${WINSDK_VERSION}")
        message(STATUS "  Found WinSDK version: ${WINSDK_VERSION}")
    endif()

    # ---- Compute all include/lib/bin paths (shared by both generators) ----
    set(MSVC_BIN_DIR "${MSVC_TOOLS_DIR}/bin/Hostx64/x64")
    set(WINSDK_BIN_DIR "${WINSDK_BIN_BASE}/x64")
    set(MSVC_INCLUDE "${MSVC_TOOLS_DIR}/include")
    set(MSVC_LIB "${MSVC_TOOLS_DIR}/lib/x64")
    set(WINSDK_UCRT_INCLUDE "${WINSDK_INCLUDE_BASE}/ucrt")
    set(WINSDK_UM_INCLUDE "${WINSDK_INCLUDE_BASE}/um")
    set(WINSDK_SHARED_INCLUDE "${WINSDK_INCLUDE_BASE}/shared")
    set(WINSDK_UCRT_LIB "${WINSDK_LIB_BASE}/ucrt/x64")
    set(WINSDK_UM_LIB "${WINSDK_LIB_BASE}/um/x64")

    if(NOT EXISTS "${MSVC_BIN_DIR}/cl.exe")
        message(FATAL_ERROR "cl.exe not found at ${MSVC_BIN_DIR}\n"
                            "Run scripts/fetch_deps.bat first to download host dependencies, "
                            "or install the Visual Studio C++ workload for a local toolchain.")
    endif()

    message(STATUS "  Using ${TOOLCHAIN_MODE} MSVC: ${MSVC_ROOT}")
    message(STATUS "  Using ${TOOLCHAIN_MODE} WinSDK: ${WINSDK_ROOT}")
    message(STATUS "  MSVC bin: ${MSVC_BIN_DIR}")

    # ---- Determine generator mode ----
    if(DEFINED ENV{GENERATOR})
        string(TOLOWER "$ENV{GENERATOR}" GENERATOR_MODE)
        # `set GENERATOR=ninja && ...` in cmd keeps the space before the &&.
        string(STRIP "${GENERATOR_MODE}" GENERATOR_MODE)
        if(NOT GENERATOR_MODE STREQUAL "ninja" AND NOT GENERATOR_MODE STREQUAL "vs")
            message(FATAL_ERROR "Invalid GENERATOR='$ENV{GENERATOR}'.\n"
                                "Use GENERATOR=ninja or GENERATOR=vs, or unset GENERATOR for auto-detect.")
        endif()
    elseif(DEFINED ENV{USE_NINJA_GENERATOR} AND "$ENV{USE_NINJA_GENERATOR}" STREQUAL "1")
        # Backward compat: USE_NINJA_GENERATOR=1 maps to GENERATOR=ninja
        set(GENERATOR_MODE "ninja")
    else()
        # Auto-detect: check for local VS installation via vswhere
        set(GENERATOR_MODE "ninja")  # safe default
        if(EXISTS "${VSWHERE}")
            execute_process(
                COMMAND "${VSWHERE}" -latest -version "[16.0,)"
                        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64
                        -property installationPath
                OUTPUT_VARIABLE VS_INSTALL_PATH
                OUTPUT_STRIP_TRAILING_WHITESPACE
                RESULT_VARIABLE VSWHERE_RESULT
            )
            if(VSWHERE_RESULT STREQUAL "0" AND VS_INSTALL_PATH AND EXISTS "${VS_INSTALL_PATH}")
                set(GENERATOR_MODE "vs")
                message(STATUS "  Auto-detected local VS: ${VS_INSTALL_PATH}")
            else()
                message(STATUS "  No local VS found, defaulting to Ninja")
            endif()
        else()
            message(STATUS "  vswhere not available, defaulting to Ninja")
        endif()
    endif()

    message(STATUS "  Generator mode: ${GENERATOR_MODE}")
    message(STATUS "    Set GENERATOR=ninja or GENERATOR=vs to override")
endif()

# End setup timer, start configure+build timer
string(TIMESTAMP _SETUP_END_TS "%s")

# Configure with CMake
message(STATUS "Configuring with CMake...")

# Generator change safeguard: CMake cannot switch generator without a clean build.
if(WIN32 AND EXISTS "${BUILD_PATH}/CMakeCache.txt")
    file(STRINGS "${BUILD_PATH}/CMakeCache.txt" _cached_generator REGEX "^CMAKE_GENERATOR:INTERNAL=")
    if(_cached_generator)
        string(REGEX REPLACE "^CMAKE_GENERATOR:INTERNAL=" "" _cached_generator "${_cached_generator}")
        set(_generator_match FALSE)
        if(GENERATOR_MODE STREQUAL "ninja" AND _cached_generator STREQUAL "Ninja")
            set(_generator_match TRUE)
        elseif(GENERATOR_MODE STREQUAL "vs")
            string(FIND "${_cached_generator}" "Visual Studio" _vs_pos)
            if(_vs_pos GREATER_EQUAL 0)
                set(_generator_match TRUE)
            endif()
        endif()
        if(NOT _generator_match)
            message(FATAL_ERROR "Generator changed (cached: ${_cached_generator}, requested: ${GENERATOR_MODE}). Please clean first using build.bat --clean")
        endif()
    endif()
endif()

set(CMAKE_ARGS
    "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
    "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    "-DOVPHYSX_USE_RELEASE_RUNTIME_DEPS=${OVPHYSX_USE_RELEASE_RUNTIME_DEPS}"
)

# Pass dev flags through explicitly so stale cache values in _build/CMakeCache.txt
# cannot keep local PhysX/schema overrides enabled after the caller removed the
# CLI flag on a later build.
if(DEV_PHYSX)
    list(APPEND CMAKE_ARGS "-DOVPHYSX_DEV_PHYSX=ON")
else()
    list(APPEND CMAKE_ARGS "-DOVPHYSX_DEV_PHYSX=OFF")
endif()

if(DEV_SCHEMA)
    list(APPEND CMAKE_ARGS "-DOVPHYSX_DEV_SCHEMA=ON")
else()
    list(APPEND CMAKE_ARGS "-DOVPHYSX_DEV_SCHEMA=OFF")
endif()

# --benchmarks: build the opt-in benchmark suite under tests/benchmarks/.
if(BENCHMARKS)
    list(APPEND CMAKE_ARGS "-DOVPHYSX_BUILD_BENCHMARKS=ON")
else()
    list(APPEND CMAKE_ARGS "-DOVPHYSX_BUILD_BENCHMARKS=OFF")
endif()

# Platform-specific generator selection
if(WIN32)
    if(GENERATOR_MODE STREQUAL "ninja")
        # ==================================================================
        # Ninja generator: packaged cl.exe + Ninja (CLI, no .sln)
        # ==================================================================
        # Best for CI and local devs without VS. Fully self-contained.

        # Add packaged Ninja to PATH
        if(NOT EXISTS "${PACKAGED_NINJA_DIR}/ninja.exe")
            message(FATAL_ERROR "Packaged Ninja not found at ${PACKAGED_NINJA_DIR}/ninja.exe\n"
                                "Run scripts/fetch_deps.bat first to download host dependencies.")
        endif()

        # nvcc string-compares the cl.exe dir it finds in PATH against -ccbin,
        # and both come from MSVC_BIN_DIR below. For a local VS under
        # "C:/Program Files/...", CMake 8.3-shortens -ccbin but not PATH, so they
        # stop matching and nvcc aborts. Space-free makes both sides identical;
        # forward slashes also survive the UNIX_COMMAND split in the compiler-id.
        ovphysx_space_free_host_path("${MSVC_BIN_DIR}" MSVC_BIN_DIR)

        set(ENV{PATH} "${PACKAGED_NINJA_DIR};${MSVC_BIN_DIR};${WINSDK_BIN_DIR};$ENV{PATH}")

        # Set INCLUDE / LIB for the compiler
        set(ENV{INCLUDE} "${MSVC_INCLUDE};${WINSDK_UCRT_INCLUDE};${WINSDK_UM_INCLUDE};${WINSDK_SHARED_INCLUDE}")
        set(ENV{LIB} "${MSVC_LIB};${WINSDK_UCRT_LIB};${WINSDK_UM_LIB}")

        # Set additional environment variables that CMake/Ninja may need
        set(ENV{VCINSTALLDIR} "${MSVC_ROOT}/VC/")
        set(ENV{VCToolsInstallDir} "${MSVC_TOOLS_DIR}/")
        set(ENV{WindowsSdkDir} "${WINSDK_ROOT}/")

        if(DEV_PHYSX)
            set(_cuda_toolset_dir "${OVRUNTIME_DIR}/_build/target-deps/cuda")
            if(EXISTS "${_cuda_toolset_dir}/bin/nvcc.exe")
                # Ninja + --devphysx: skip nvcc's vcvars64.bat during CMake CUDA detection.
                # Environment is seeded above; PhysXDependency.cmake also sets --use-local-env.
                list(APPEND CMAKE_ARGS
                    "-DCMAKE_CUDA_COMPILER=${_cuda_toolset_dir}/bin/nvcc.exe"
                    "-DCMAKE_CUDA_HOST_COMPILER=${MSVC_BIN_DIR}/cl.exe"
                    "-DCMAKE_CUDA_FLAGS=--use-local-env"
                )
            endif()
        endif()

        list(APPEND CMAKE_ARGS "-G" "Ninja")
        message(STATUS "Using Ninja generator (${TOOLCHAIN_MODE} cl.exe + packaged Ninja)")
        message(STATUS "  [OK] ${TOOLCHAIN_MODE} toolchain ready for Ninja build")

    elseif(GENERATOR_MODE STREQUAL "vs")
        # ==================================================================
        # VS generator: produces .sln using packaged MSVC as portable instance
        # ==================================================================
        # - Uses CMAKE_GENERATOR_INSTANCE with version= to register the packaged
        #   MSVC as a portable VS instance (CMake 3.23+ feature)
        # - No local VS installation required for generation or building
        # - SDK include/lib paths redirected to packaged WinSDK via CMAKE_VS_SDK_*
        #   (PhysX pattern), baked into .vcxproj files
        # - cmake --build invokes the packaged MSBuild
        # - Devs with local VS can also open the generated .sln in their IDE

        # CMAKE_GENERATOR_INSTANCE with version= requires CMake 3.23+.
        # The project minimum is 3.16 (fine for Ninja), so check at runtime.
        if(CMAKE_VERSION VERSION_LESS "3.23")
            message(FATAL_ERROR "GENERATOR=vs requires CMake 3.23+ (found ${CMAKE_VERSION}).\n"
                                "Upgrade CMake or use GENERATOR=ninja.")
        endif()

        # Resolve VS build version from MSBuild (packaged or local VS).
        # Prefer MSBuild.exe file version (robust across config layout changes).
        # Fall back to MSBuild.exe.config parsing for older package shapes.
        set(MSBUILD_CONFIG "${MSVC_ROOT}/MSBuild/Current/Bin/MSBuild.exe.config")
        set(VS_BUILD_VERSION "")
        if(EXISTS "${MSBUILD_EXE}")
            execute_process(
                COMMAND powershell -NoProfile -ExecutionPolicy Bypass -Command "(Get-Item '${MSBUILD_EXE}').VersionInfo.FileVersion"
                OUTPUT_VARIABLE _msbuild_file_version
                OUTPUT_STRIP_TRAILING_WHITESPACE
                RESULT_VARIABLE _msbuild_file_version_result
            )
            if(_msbuild_file_version_result EQUAL 0 AND _msbuild_file_version MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+\\.[0-9]+$")
                set(VS_BUILD_VERSION "${_msbuild_file_version}")
            endif()
        endif()
        if(NOT VS_BUILD_VERSION AND EXISTS "${MSBUILD_CONFIG}")
            file(READ "${MSBUILD_CONFIG}" _msbuild_cfg)
            string(REGEX MATCH
                "codeBase version=\"([0-9]+\\.[0-9]+\\.[0-9]+\\.[0-9]+)\""
                _match "${_msbuild_cfg}")
            if(CMAKE_MATCH_1)
                set(VS_BUILD_VERSION "${CMAKE_MATCH_1}")
            endif()
        endif()
        if(NOT VS_BUILD_VERSION)
            message(FATAL_ERROR "Could not determine VS build version from MSBuild.\n"
                                "Checked file version of ${MSBUILD_EXE} and codeBase entries in ${MSBUILD_CONFIG}.")
        endif()

        # Derive the VS generator string from the major version
        # (16 -> "Visual Studio 16 2019", 17 -> "Visual Studio 17 2022", etc.)
        string(REGEX MATCH "^([0-9]+)" _vs_major "${VS_BUILD_VERSION}")
        if(_vs_major STREQUAL "16")
            set(VS_GENERATOR "Visual Studio 16 2019")
        elseif(_vs_major STREQUAL "17")
            set(VS_GENERATOR "Visual Studio 17 2022")
        else()
            message(FATAL_ERROR "Unsupported VS major version '${_vs_major}' (from ${TOOLCHAIN_MODE} MSVC).\n"
                                "Update the VS generator mapping in build.cmake, "
                                "or build with GENERATOR=ninja instead.")
        endif()

        if(TOOLCHAIN_MODE STREQUAL "packaged")
            # Use portable instance: path + version= (CMake 3.23+)
            # This allows CMake's VS generator to use the packaged MSVC without
            # the instance being registered with the Visual Studio Installer.
            # See: https://cmake.org/cmake/help/latest/variable/CMAKE_GENERATOR_INSTANCE.html
            set(GENERATOR_INSTANCE "${MSVC_ROOT},version=${VS_BUILD_VERSION}")
            if(NOT EXISTS "${GENERATOR_INSTANCE}")
                # CMake/MSBuild may resolve CMAKE_GENERATOR_INSTANCE literally (including ",version=")
                # in generated vcxproj imports. Create a junction alias so those imports resolve.
                execute_process(
                    COMMAND powershell -NoProfile -ExecutionPolicy Bypass -Command "New-Item -ItemType Junction -Path '${GENERATOR_INSTANCE}' -Target '${MSVC_ROOT}' -Force | Out-Null"
                    RESULT_VARIABLE _junction_result
                    ERROR_VARIABLE _junction_error
                )
                if(NOT _junction_result EQUAL 0 OR NOT EXISTS "${GENERATOR_INSTANCE}")
                    message(FATAL_ERROR "Failed to create VS generator instance junction:\n"
                                        "  ${GENERATOR_INSTANCE} -> ${MSVC_ROOT}\n"
                                        "Error: ${_junction_error}")
                endif()
                message(STATUS "  Created generator instance junction: ${GENERATOR_INSTANCE}")
            else()
                message(STATUS "  Using existing generator instance junction: ${GENERATOR_INSTANCE}")
            endif()
        else()
            # Local VS is a registered instance: plain path, no portable-instance
            # junction and no patching inside the installation.
            set(GENERATOR_INSTANCE "${MSVC_ROOT}")
        endif()

        # Set environment variables so MSBuild (invoked by CMake during configure)
        # can find the compiler, headers, and libraries in the packaged MSVC/WinSDK.
        # Note: the actual fix for the compiler ID test is Directory.Build.props below,
        # which provides LibraryPath/IncludePath to MSBuild (it overrides the LIB env var).
        # These env vars are kept as a safety net for non-MSBuild tools.
        get_filename_component(_vc_tools_version "${MSVC_TOOLS_DIR}" NAME)

        # The packaged NVIDIA.ImportBefore.props ships with stale values that
        # break the VS generator: wrong VCToolsVersion, and a relative
        # VCInstallDir_170 that resolves incorrectly from the amd64 MSBuild.
        # Rather than patching the packman-distributed file (which may be
        # read-only or cached), we drop an override props file that MSBuild
        # imports AFTER the NVIDIA one (alphabetical wildcard import).
        if(TOOLCHAIN_MODE STREQUAL "packaged")
            set(_import_before_dir "${MSVC_ROOT}/MSBuild/Current/Imports/Microsoft.Common.Props/ImportBefore")
            set(_override_props "${_import_before_dir}/ZZ_PackmanFix.ImportBefore.props")
            if(IS_DIRECTORY "${_import_before_dir}")
                file(WRITE "${_override_props}"
"<Project xmlns=\"http://schemas.microsoft.com/developer/msbuild/2003\">\n\
  <PropertyGroup>\n\
    <VCToolsVersion>${_vc_tools_version}</VCToolsVersion>\n\
    <VCInstallDir_170>${MSVC_ROOT}/VC/</VCInstallDir_170>\n\
    <VCToolsInstallDir_170>$(VCInstallDir_170)Tools\\MSVC\\$(VCToolsVersion)\\</VCToolsInstallDir_170>\n\
  </PropertyGroup>\n\
</Project>\n")
                message(STATUS "  Created ${_override_props}")
                message(STATUS "    VCToolsVersion=${_vc_tools_version}  VCInstallDir_170=${MSVC_ROOT}/VC/")
            endif()
        endif()
        set(ENV{VCINSTALLDIR} "${MSVC_ROOT}/VC/")
        set(ENV{VCToolsInstallDir} "${MSVC_TOOLS_DIR}/")
        set(ENV{VCToolsVersion} "${_vc_tools_version}")
        set(ENV{WindowsSdkDir} "${WINSDK_ROOT}/")
        get_filename_component(_msbuild_dir "${MSBUILD_EXE}" DIRECTORY)
        set(ENV{PATH} "${_msbuild_dir};${MSVC_BIN_DIR};${WINSDK_BIN_DIR};$ENV{PATH}")
        set(ENV{INCLUDE} "${MSVC_INCLUDE};${WINSDK_UCRT_INCLUDE};${WINSDK_UM_INCLUDE};${WINSDK_SHARED_INCLUDE}")
        set(ENV{LIB} "${MSVC_LIB};${WINSDK_UCRT_LIB};${WINSDK_UM_LIB}")

        if(TOOLCHAIN_MODE STREQUAL "packaged")
            # Ensure the exact toolset props file exists for imports generated
            # during compiler-id checks. A local VS installation ships the real
            # props file and must never be written into.
            set(_vctools_props_dir "${MSVC_ROOT}/VC/Auxiliary/Build/${_vc_tools_version}")
            set(_vctools_props_file "${_vctools_props_dir}/Microsoft.VCToolsVersion.${_vc_tools_version}.props")
            if(NOT EXISTS "${_vctools_props_file}")
                file(MAKE_DIRECTORY "${_vctools_props_dir}")
                file(WRITE "${_vctools_props_file}"
"<?xml version=\"1.0\" encoding=\"utf-8\"?>\n\
<Project ToolsVersion=\"4.0\" xmlns=\"http://schemas.microsoft.com/developer/msbuild/2003\">\n\
  <PropertyGroup>\n\
    <VCToolsVersion>${_vc_tools_version}</VCToolsVersion>\n\
  </PropertyGroup>\n\
</Project>\n")
                message(STATUS "  Created missing toolset props: ${_vctools_props_file}")
            endif()
        endif()

        list(APPEND CMAKE_ARGS
            "-G" "${VS_GENERATOR}"
            "-A" "x64"
            "-DCMAKE_GENERATOR_INSTANCE=${GENERATOR_INSTANCE}"
        )
        if(TOOLCHAIN_MODE STREQUAL "packaged")
            # Pin the exact toolset the portable instance ships; the matching
            # full-version props file is synthesized above. A local VS carries
            # props only for the minor toolset line, so this pin would break
            # MSBuild's compiler detection there — local VS uses its default
            # (newest) toolset instead.
            list(APPEND CMAKE_ARGS "-DCMAKE_VS_PLATFORM_TOOLSET_VERSION=${_vc_tools_version}")
        endif()
        set(_cuda_toolset_dir "${OVRUNTIME_DIR}/_build/target-deps/cuda")
        if(EXISTS "${_cuda_toolset_dir}/bin/nvcc.exe")
            # PhysX enables CMake's CUDA language in --devphysx builds.  The
            # VS generator requires an explicit CUDA toolset when CUDA comes
            # from packman rather than a machine-wide Visual Studio install.
            list(APPEND CMAKE_ARGS "-T" "cuda=${_cuda_toolset_dir}")
        elseif(DEV_PHYSX)
            message(WARNING "Packaged CUDA toolkit not found at ${_cuda_toolset_dir}; "
                            "Visual Studio --devphysx builds may fail during CUDA detection.")
        endif()

        # Redirect SDK include/lib paths to packaged WinSDK (PhysX pattern)
        # These are baked into the generated .vcxproj files, so builds use the
        # packaged SDK regardless of whether invoked from CLI or the VS IDE.
        # Semicolons must be escaped so they survive CMake list expansion in execute_process.
        set(ALL_INCLUDE_DIRS "${MSVC_INCLUDE};${WINSDK_UCRT_INCLUDE};${WINSDK_UM_INCLUDE};${WINSDK_SHARED_INCLUDE}")
        set(ALL_LIB_DIRS "${MSVC_LIB};${WINSDK_UCRT_LIB};${WINSDK_UM_LIB}")
        string(REPLACE ";" "\\;" ALL_INCLUDE_DIRS_ESC "${ALL_INCLUDE_DIRS}")
        string(REPLACE ";" "\\;" ALL_LIB_DIRS_ESC "${ALL_LIB_DIRS}")
        list(APPEND CMAKE_ARGS
            "-DCMAKE_VS_SDK_INCLUDE_DIRECTORIES=${ALL_INCLUDE_DIRS_ESC}"
            "-DCMAKE_VS_SDK_LIBRARY_DIRECTORIES=${ALL_LIB_DIRS_ESC}"
        )

        # Generate Directory.Build.props so MSBuild can find SDK libs/headers.
        # The packaged MSVC blanks out Microsoft.Cpp.WindowsSDK.props, so MSBuild
        # can't auto-discover SDK paths. This file is auto-imported for ALL .vcxproj
        # in the build tree -- critically, for CMake's compiler ID test project
        # (which does NOT get CMAKE_VS_SDK_* overrides).
        # For the main project, CMAKE_VS_SDK_* in each .vcxproj takes precedence.
        file(WRITE "${BUILD_PATH}/Directory.Build.props"
"<Project>\n\
  <PropertyGroup>\n\
    <IncludePath>${MSVC_INCLUDE};${WINSDK_UCRT_INCLUDE};${WINSDK_UM_INCLUDE};${WINSDK_SHARED_INCLUDE};$(IncludePath)</IncludePath>\n\
    <LibraryPath>${MSVC_LIB};${WINSDK_UCRT_LIB};${WINSDK_UM_LIB};$(LibraryPath)</LibraryPath>\n\
    <ExecutablePath>${MSVC_BIN_DIR};${WINSDK_BIN_DIR};$(ExecutablePath)</ExecutablePath>\n\
  </PropertyGroup>\n\
</Project>\n")

        message(STATUS "Using Visual Studio generator (${TOOLCHAIN_MODE} MSVC)")
        message(STATUS "  CMAKE_GENERATOR_INSTANCE: ${GENERATOR_INSTANCE}")
        message(STATUS "  VS build version: ${VS_BUILD_VERSION}")
        if(EXISTS "${_cuda_toolset_dir}/bin/nvcc.exe")
            message(STATUS "  CUDA toolset: ${_cuda_toolset_dir}")
        endif()
        if(EXISTS "${MSBUILD_EXE}")
            message(STATUS "  MSBuild: ${MSBUILD_EXE}")
        else()
            message(WARNING "MSBuild not found at ${MSBUILD_EXE}.")
        endif()
        message(STATUS "  SDK paths redirected to ${TOOLCHAIN_MODE} WinSDK (baked into .vcxproj)")
        message(STATUS "  [OK] VS generator configured")
    else()
        message(FATAL_ERROR "Unknown GENERATOR mode: '${GENERATOR_MODE}'.\n"
                            "Use GENERATOR=ninja or GENERATOR=vs, or unset GENERATOR for auto-detect.")
    endif()
endif()

if(VERBOSE)
    list(APPEND CMAKE_ARGS "-DCMAKE_VERBOSE_MAKEFILE=ON")
endif()

# Add any additional CMake arguments from environment
if(DEFINED ENV{CMAKE_EXTRA_ARGS})
    separate_arguments(EXTRA_ARGS NATIVE_COMMAND "$ENV{CMAKE_EXTRA_ARGS}")
    list(APPEND CMAKE_ARGS ${EXTRA_ARGS})
endif()

execute_process(
    COMMAND ${CMAKE_COMMAND} ${CMAKE_ARGS} "${PROJECT_ROOT}"
    WORKING_DIRECTORY "${BUILD_PATH}"
    RESULT_VARIABLE CONFIG_RESULT
)

if(NOT CONFIG_RESULT EQUAL 0)
    message("")
    message("========================================")
    message("CONFIGURATION FAILED")
    message("========================================")
    message(FATAL_ERROR "CMake configuration failed (exit code: ${CONFIG_RESULT})")
endif()

if(GENERATE_ONLY)
    message(STATUS "Generate-only mode: skipping build.")
    return()
endif()

string(TIMESTAMP _COMPILE_START_TS "%s")
message(STATUS "Building ovphysx...")
set(BUILD_ARGS --build . --config ${BUILD_TYPE} --parallel ${JOBS})
if(NOT "${BUILD_TARGET}" STREQUAL "")
    list(APPEND BUILD_ARGS --target "${BUILD_TARGET}")
endif()
if(VERBOSE)
    list(APPEND BUILD_ARGS --verbose)
endif()
execute_process(
    COMMAND ${CMAKE_COMMAND} ${BUILD_ARGS}
    WORKING_DIRECTORY "${BUILD_PATH}"
    RESULT_VARIABLE BUILD_RESULT
)
string(TIMESTAMP _COMPILE_END_TS "%s")
if(NOT BUILD_RESULT EQUAL 0)
    message("")
    message("========================================")
    message("BUILD FAILED")
    message("========================================")
    message(FATAL_ERROR "Build failed (exit code: ${BUILD_RESULT})")
endif()

math(EXPR _COMPILE_SECS "${_COMPILE_END_TS} - ${_COMPILE_START_TS}")
message(STATUS "  Main compilation finished in ${_COMPILE_SECS}s")

# Verify ovruntime .so files landed in the expected output directory.
# ovruntime output follows NvidiaBuildOptions convention: PX_OUTPUT_LIB_DIR/<config_lower>
set(OVRUNTIME_OUTPUT_DIR "${BUILD_PATH}/${BUILD_TYPE_LOWER}")
if(NOT EXISTS "${OVRUNTIME_OUTPUT_DIR}" AND BUILD_TYPE_LOWER STREQUAL "release")
    # devphysx remaps Release->checked
    set(OVRUNTIME_OUTPUT_DIR "${BUILD_PATH}/checked")
endif()
if(EXISTS "${OVRUNTIME_OUTPUT_DIR}")
    message(STATUS "  [OK] ovruntime subproject output: ${OVRUNTIME_OUTPUT_DIR}")
else()
    message(WARNING "ovruntime subproject output directory not found at ${BUILD_PATH}/${BUILD_TYPE_LOWER}")
endif()


string(TIMESTAMP _BUILD_END_TS "%s")
math(EXPR _SETUP_SECS "${_SETUP_END_TS} - ${_BUILD_START_TS}")
math(EXPR _CONFIGURE_SECS "${_COMPILE_START_TS} - ${_SETUP_END_TS}")
math(EXPR _TOTAL_SECS "${_BUILD_END_TS} - ${_BUILD_START_TS}")
math(EXPR _TOTAL_MIN "${_TOTAL_SECS} / 60")
math(EXPR _TOTAL_REM "${_TOTAL_SECS} % 60")

message("")
message("========================================")
message("BUILD SUCCESSFUL  (${_TOTAL_MIN}m ${_TOTAL_REM}s)")
message("========================================")
message(STATUS "  Setup / fetch deps:       ${_SETUP_SECS}s")
message(STATUS "  CMake configure:          ${_CONFIGURE_SECS}s")
message(STATUS "  Main compilation:         ${_COMPILE_SECS}s")
message(STATUS "  Total:                    ${_TOTAL_MIN}m ${_TOTAL_REM}s")
message(STATUS "All targets compiled successfully")
message(STATUS "")
message(STATUS "Build complete. C++ unit tests and Python tests will run from _build/")
message(STATUS "Next step: cmake -P scripts/install.cmake")
message(STATUS "Release packaging:       cmake -P scripts/package_sdk.cmake")

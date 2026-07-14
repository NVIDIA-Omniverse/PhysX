# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

# Kit SDK discovery for ovexts.
#
# Kit SDK ships debug and release variants in target-deps.
# This module sets up paths for includes, plugins, and extension libraries.
#
# Outputs:
#   OVEXTS_KIT_SDK_DIR       - Kit SDK root (config-dependent via generator expression)
#   OVEXTS_KIT_SDK_INCLUDES  - Kit SDK include directory
#   OVEXTS_KIT_SDK_EXTS_DEPS - Kit SDK bundled extensions (from kit_sdk package)
#   OVEXTS_KIT_SDK_EXTS_BUILD - Kit extensions populated at build time (from kit prebuild)

set(_KIT_SDK_BASE "${OVEXTS_TARGET_DEPS}/kit_sdk")

# Config-dependent Kit SDK path.
# Single-config generators (Makefiles): use CMAKE_BUILD_TYPE.
# Multi-config generators (VS): use generator expressions for build-time paths,
# and a concrete "release" path for configure-time discovery (EXISTS checks, etc.).
if(CMAKE_CONFIGURATION_TYPES)
    set(OVEXTS_KIT_SDK_DIR "${_KIT_SDK_BASE}_$<LOWER_CASE:$<CONFIG>>")
    # Concrete path for configure-time discovery (prefer release)
    if(EXISTS "${_KIT_SDK_BASE}_release")
        set(_KIT_SDK_DIR_CONFIGURE "${_KIT_SDK_BASE}_release")
    else()
        set(_KIT_SDK_DIR_CONFIGURE "${_KIT_SDK_BASE}_debug")
    endif()
else()
    string(TOLOWER "${CMAKE_BUILD_TYPE}" _kit_config)
    set(OVEXTS_KIT_SDK_DIR "${_KIT_SDK_BASE}_${_kit_config}")
    set(_KIT_SDK_DIR_CONFIGURE "${OVEXTS_KIT_SDK_DIR}")
endif()

set(OVEXTS_KIT_SDK_INCLUDES "${OVEXTS_KIT_SDK_DIR}/dev/include")

# Extension library directories.
# kit_sdk_exts_deps: extensions bundled with the Kit SDK package itself.
set(OVEXTS_KIT_SDK_EXTS_DEPS "${OVEXTS_KIT_SDK_DIR}/exts")

# kit_sdk_exts_build: extensions resolved from the Kit extension registry.
# These are populated by `repo.sh precache_exts` into the build tree.
# Two possible locations:
#   - _build/<platform>/<config>/exts/ (Kit runtime cache, used by premake builds)
#   - _build/<platform>/<config>/extsbuild/ (repo_tools links.path)
set(OVEXTS_KIT_SDK_EXTS_BUILD "${OVEXTS_BIN_DIR}/exts")
set(OVEXTS_KIT_SDK_EXTS_LINKED "${OVEXTS_BIN_DIR}/extsbuild")

# Concrete configure-time paths for file discovery (EXISTS checks, find_*, etc.).
# On multi-config generators OVEXTS_BIN_DIR contains a generator expression that
# cannot be evaluated at configure time.  Prefer release (the common build config)
# since precached extensions typically live there.
set(_BIN_DIR_CONFIGURE "")
foreach(_candidate IN LISTS OVEXTS_BIN_DIR_CONFIGS)
    if(EXISTS "${_candidate}/exts")
        set(_BIN_DIR_CONFIGURE "${_candidate}")
        break()
    endif()
endforeach()
if(NOT _BIN_DIR_CONFIGURE)
    # Fallback: use the last entry (release comes after debug in CMAKE_CONFIGURATION_TYPES)
    list(LENGTH OVEXTS_BIN_DIR_CONFIGS _len)
    math(EXPR _last "${_len} - 1")
    list(GET OVEXTS_BIN_DIR_CONFIGS ${_last} _BIN_DIR_CONFIGURE)
endif()
set(OVEXTS_KIT_SDK_EXTS_BUILD_CONFIGURE "${_BIN_DIR_CONFIGURE}/exts")
set(OVEXTS_KIT_SDK_EXTS_LINKED_CONFIGURE "${_BIN_DIR_CONFIGURE}/extsbuild")
set(OVEXTS_KIT_SDK_EXTS_DEPS_CONFIGURE "${_KIT_SDK_DIR_CONFIGURE}/exts")

# Verify at least one Kit SDK variant exists at configure time
if(EXISTS "${_KIT_SDK_BASE}_release")
    message(STATUS "Found Kit SDK (release): ${_KIT_SDK_BASE}_release")
elseif(EXISTS "${_KIT_SDK_BASE}_debug")
    message(STATUS "Found Kit SDK (debug): ${_KIT_SDK_BASE}_debug")
else()
    message(WARNING "Kit SDK not found at ${_KIT_SDK_BASE}_{debug,release}. Run build.sh --fetch-only or packman first.")
endif()

# Helper: add Kit SDK extension library directories to a target.
# This adds both the deps and build extension paths for the given Kit extension.
function(ovexts_link_kit_ext_libs target)
    foreach(ext_name ${ARGN})
        target_link_directories(${target} PRIVATE
            "${OVEXTS_KIT_SDK_EXTS_BUILD}/${ext_name}/bin"
            "${OVEXTS_KIT_SDK_EXTS_LINKED}/${ext_name}/bin"
            "${OVEXTS_KIT_SDK_EXTS_DEPS}/${ext_name}/bin"
        )
    endforeach()
endfunction()

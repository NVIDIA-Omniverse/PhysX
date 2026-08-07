# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Resolve the ovstage dependency for the opt-in omni.physics.ovstage backend
# (ADR-0002 Milestone 1). pull_dependencies links a local ovstage source
# checkout into _build/target-deps/ovstage (see deps/ovstage-deps.packman.xml);
# the artifactory wheel layout at _build/target-deps/ovstage_pip is a fallback.
#
# Resolution order for the ovstage root (override with -DOVSTAGE_DIR=...):
#   1. explicit -DOVSTAGE_DIR
#   2. the Packman source link at _build/target-deps/ovstage
#   3. the wheel install directory at _build/target-deps/ovstage_pip
#
# Supported layouts:
#   legacy:   ovstage/public/include, ovrtx/public/include, _build/*/release
#   wheel:    ovstage/include/{ovstage,ovx,dlpack}, ovstage/lib, ovstage/bin
#   package:  include/{ovstage,ovx,dlpack}, bin/libovstage.so (+ bin/plugins), lib/cmake
#             (the artifactory ovstage package; libovstage.so ships in bin/, not lib/)
#
# Sets, in the parent scope:
#   OVSTAGE_INCLUDE_DIRS  - list of include directories
#   OVSTAGE_LIBRARY       - full path to the ovstage link library
#   OVSTAGE_LIBRARY_DIR   - directory containing the ovstage library (for RPATH/PATH)

if(NOT DEFINED OVSTAGE_DIR)
    if(EXISTS "${OVRUNTIME_TARGET_DEPS}/ovstage")
        set(OVSTAGE_DIR "${OVRUNTIME_TARGET_DEPS}/ovstage")
    elseif(EXISTS "${OVRUNTIME_TARGET_DEPS}/ovstage_pip")
        set(OVSTAGE_DIR "${OVRUNTIME_TARGET_DEPS}/ovstage_pip")
    endif()
endif()

if(NOT OVSTAGE_DIR OR NOT EXISTS "${OVSTAGE_DIR}")
    message(FATAL_ERROR
        "The ovstage backend (omni.physics.ovstage) is built on this branch but "
        "the ovstage dependency was not found. Run pull_dependencies so the "
        "local ovstage source is linked into _build/target-deps/ovstage (see "
        "deps/ovstage-deps.packman.xml), or set "
        "-DOVSTAGE_DIR=<ovstage rendering root> explicitly.")
endif()

set(_ovstage_include_candidates)
if(EXISTS "${OVSTAGE_DIR}/include/ovstage/ovstage.h")
    list(APPEND _ovstage_include_candidates "${OVSTAGE_DIR}/include")
endif()
if(EXISTS "${OVSTAGE_DIR}/ovstage/include/ovstage/ovstage.h")
    list(APPEND _ovstage_include_candidates "${OVSTAGE_DIR}/ovstage/include")
endif()
if(EXISTS "${OVSTAGE_DIR}/ovstage/public/include/ovstage/ovstage.h")
    list(APPEND _ovstage_include_candidates "${OVSTAGE_DIR}/ovstage/public/include")
endif()

set(_ovrtx_include_candidates)
if(EXISTS "${OVSTAGE_DIR}/include/ovx/string_types.h")
    list(APPEND _ovrtx_include_candidates "${OVSTAGE_DIR}/include")
endif()
if(EXISTS "${OVSTAGE_DIR}/ovstage/include/ovx/string_types.h")
    list(APPEND _ovrtx_include_candidates "${OVSTAGE_DIR}/ovstage/include")
endif()
if(EXISTS "${OVSTAGE_DIR}/ovrtx/public/include/ovx/string_types.h")
    list(APPEND _ovrtx_include_candidates "${OVSTAGE_DIR}/ovrtx/public/include")
endif()

list(LENGTH _ovstage_include_candidates _ovstage_include_candidate_count)
if(_ovstage_include_candidate_count GREATER 0)
    list(GET _ovstage_include_candidates 0 _ovstage_inc)
endif()

list(LENGTH _ovrtx_include_candidates _ovrtx_include_candidate_count)
if(_ovrtx_include_candidate_count GREATER 0)
    list(GET _ovrtx_include_candidates 0 _ovrtx_inc)
endif()

# Library: a config subdir under the linked _build (release by default).
if(WIN32)
    file(GLOB _ovstage_lib_candidates
        "${OVSTAGE_DIR}/ovstage/lib/ovstage.lib"
        "${OVSTAGE_DIR}/_build/windows-x86_64/release/ovstage.lib"
        "${OVSTAGE_DIR}/_build/*/release/ovstage.lib"
        "${OVSTAGE_DIR}/lib/ovstage.lib")
else()
    file(GLOB _ovstage_lib_candidates
        "${OVSTAGE_DIR}/bin/libovstage.so"
        "${OVSTAGE_DIR}/ovstage/bin/libovstage.so"
        "${OVSTAGE_DIR}/ovstage/lib/libovstage.so"
        "${OVSTAGE_DIR}/_build/linux-x86_64/release/libovstage.so"
        "${OVSTAGE_DIR}/_build/*/release/libovstage.so"
        "${OVSTAGE_DIR}/lib/libovstage.so")
endif()
list(LENGTH _ovstage_lib_candidates _ovstage_lib_candidate_count)
if(_ovstage_lib_candidate_count GREATER 0)
    list(GET _ovstage_lib_candidates 0 OVSTAGE_LIBRARY)
endif()

if(NOT OVSTAGE_LIBRARY OR NOT _ovstage_inc OR NOT _ovrtx_inc)
    message(FATAL_ERROR
        "ovstage layout under '${OVSTAGE_DIR}' is incomplete: expected "
        "wheel layout ovstage/include + ovstage/lib, or legacy layout "
        "ovstage/public/include + ovrtx/public/include + _build/*/release. "
        "Point -DOVSTAGE_DIR at a compatible ovstage root.")
endif()

get_filename_component(OVSTAGE_LIBRARY_DIR "${OVSTAGE_LIBRARY}" DIRECTORY)
if(WIN32 AND EXISTS "${OVSTAGE_DIR}/bin/ovstage.dll")
    set(OVSTAGE_LIBRARY_DIR "${OVSTAGE_DIR}/bin")
elseif(WIN32 AND EXISTS "${OVSTAGE_DIR}/ovstage/bin/ovstage.dll")
    set(OVSTAGE_LIBRARY_DIR "${OVSTAGE_DIR}/ovstage/bin")
endif()

set(OVSTAGE_INCLUDE_DIRS "${_ovstage_inc}" "${_ovrtx_inc}")
list(REMOVE_DUPLICATES OVSTAGE_INCLUDE_DIRS)

message(STATUS "ovstage: include=${OVSTAGE_INCLUDE_DIRS}")
message(STATUS "ovstage: library=${OVSTAGE_LIBRARY}")
message(STATUS "ovstage: runtime=${OVSTAGE_LIBRARY_DIR}")

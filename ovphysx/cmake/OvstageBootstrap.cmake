# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-OVSTAGE-001
# @covers AC-4
#
# Fetch the SDK's pinned OVStage when no compatible package was found (ADR-0033).
# OVPHYSX_FETCH_OVSTAGE disables downloads; OVPHYSX_OVSTAGE_CACHE_DIR overrides the cache.
# Return the root or an error for ovphysxConfig.cmake's package-not-found message.

if(DEFINED _OVPHYSX_OVSTAGE_BOOTSTRAP_DONE)
    return()
endif()

set(ovphysx_OVSTAGE_BOOTSTRAP_ERROR "")

set(_ovphysx_fetch_script "${OVPHYSX_PACKAGE_PREFIX}/scripts/fetch_ovstage_release.py")
if(NOT EXISTS "${_ovphysx_fetch_script}")
    set(ovphysx_OVSTAGE_BOOTSTRAP_ERROR
        "the bundled fetch script is missing at ${_ovphysx_fetch_script}")
    return()
endif()

file(STRINGS "${_ovphysx_fetch_script}" _ovphysx_ovstage_version_line
     REGEX "^OVSTAGE_VERSION = \"")
string(REGEX REPLACE "^OVSTAGE_VERSION = \"([^\"]+)\".*$" "\\1"
       _ovphysx_ovstage_version "${_ovphysx_ovstage_version_line}")
if(NOT _ovphysx_ovstage_version)
    set(ovphysx_OVSTAGE_BOOTSTRAP_ERROR
        "could not read OVSTAGE_VERSION from ${_ovphysx_fetch_script}")
    return()
endif()

if(WIN32 AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
    set(_ovphysx_ovstage_platform "windows-x86_64")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
    set(_ovphysx_ovstage_platform "manylinux_2_35_aarch64")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
    set(_ovphysx_ovstage_platform "manylinux_2_35_x86_64")
else()
    set(ovphysx_OVSTAGE_BOOTSTRAP_ERROR
        "unsupported OVStage platform: ${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}")
    return()
endif()

if(NOT DEFINED OVPHYSX_OVSTAGE_CACHE_DIR)
    if(WIN32 AND DEFINED ENV{LOCALAPPDATA})
        set(_ovphysx_cache_root "$ENV{LOCALAPPDATA}/ovphysx")
    elseif(DEFINED ENV{XDG_CACHE_HOME})
        set(_ovphysx_cache_root "$ENV{XDG_CACHE_HOME}/ovphysx")
    elseif(DEFINED ENV{HOME})
        set(_ovphysx_cache_root "$ENV{HOME}/.cache/ovphysx")
    else()
        # No user cache available (some CI containers); fall back to the build tree.
        set(_ovphysx_cache_root "${CMAKE_BINARY_DIR}/_ovphysx_deps")
    endif()
    # Separate caches for each pinned build and target platform.
    set(OVPHYSX_OVSTAGE_CACHE_DIR
        "${_ovphysx_cache_root}/ovstage-${_ovphysx_ovstage_version}-${_ovphysx_ovstage_platform}")
endif()

# Reuse a completed cache without network access.
set(_ovphysx_ovstage_stamp "${OVPHYSX_OVSTAGE_CACHE_DIR}/.ovphysx-ovstage-stamp")

# A custom cache directory may contain a different build or platform.
set(_ovphysx_ovstage_want "${_ovphysx_ovstage_version} ${_ovphysx_ovstage_platform}")
set(_ovphysx_ovstage_cached FALSE)
if(EXISTS "${_ovphysx_ovstage_stamp}")
    file(READ "${_ovphysx_ovstage_stamp}" _ovphysx_ovstage_have)
    string(STRIP "${_ovphysx_ovstage_have}" _ovphysx_ovstage_have)
    if(_ovphysx_ovstage_have STREQUAL _ovphysx_ovstage_want)
        set(_ovphysx_ovstage_cached TRUE)
    else()
        message(STATUS
            "ovphysx: cached OVStage is '${_ovphysx_ovstage_have}', want "
            "'${_ovphysx_ovstage_want}' -- refetching")
    endif()
endif()

if(NOT _ovphysx_ovstage_cached)
    if(NOT OVPHYSX_FETCH_OVSTAGE)
        set(ovphysx_OVSTAGE_BOOTSTRAP_ERROR
            "OVPHYSX_FETCH_OVSTAGE is OFF, so the pinned build (${_ovphysx_ovstage_version}) was not downloaded")
        return()
    endif()

    find_package(Python3 3.8 COMPONENTS Interpreter QUIET)
    if(NOT Python3_Interpreter_FOUND)
        set(ovphysx_OVSTAGE_BOOTSTRAP_ERROR
            "downloading OVStage needs Python 3.8+ on PATH, and none was found")
        return()
    endif()

    message(STATUS "ovphysx: fetching OVStage ${_ovphysx_ovstage_version} (${_ovphysx_ovstage_platform})")
    message(STATUS "ovphysx:   -> ${OVPHYSX_OVSTAGE_CACHE_DIR}")

    # Serialize cache replacement across concurrent configures.
    file(LOCK "${OVPHYSX_OVSTAGE_CACHE_DIR}.lock" GUARD FILE
         RESULT_VARIABLE _ovphysx_lock_rc TIMEOUT 600)
    if(NOT _ovphysx_lock_rc STREQUAL "0")
        set(ovphysx_OVSTAGE_BOOTSTRAP_ERROR "could not lock the OVStage cache: ${_ovphysx_lock_rc}")
        return()
    endif()

    # Download and validate in a separate tree before publishing it.
    string(MD5 _ovphysx_ovstage_staging_hash "${CMAKE_BINARY_DIR}")
    set(_ovphysx_ovstage_staging "${OVPHYSX_OVSTAGE_CACHE_DIR}.tmp.${_ovphysx_ovstage_staging_hash}")
    file(REMOVE_RECURSE "${_ovphysx_ovstage_staging}")

    execute_process(
        COMMAND "${Python3_EXECUTABLE}" "${_ovphysx_fetch_script}"
            --platform "${_ovphysx_ovstage_platform}"
            --dest "${_ovphysx_ovstage_staging}"
        RESULT_VARIABLE _ovphysx_fetch_rc)
    if(NOT _ovphysx_fetch_rc EQUAL 0)
        file(REMOVE_RECURSE "${_ovphysx_ovstage_staging}")
        set(ovphysx_OVSTAGE_BOOTSTRAP_ERROR
            "the OVStage download failed (see the output above)")
        return()
    endif()

    file(WRITE "${_ovphysx_ovstage_staging}/.ovphysx-ovstage-stamp"
         "${_ovphysx_ovstage_want}\n")

    # A configure that held the lock first may already have published this build.
    set(_ovphysx_ovstage_published FALSE)
    if(EXISTS "${_ovphysx_ovstage_stamp}")
        file(READ "${_ovphysx_ovstage_stamp}" _ovphysx_ovstage_published_id)
        string(STRIP "${_ovphysx_ovstage_published_id}" _ovphysx_ovstage_published_id)
        if(_ovphysx_ovstage_published_id STREQUAL _ovphysx_ovstage_want)
            set(_ovphysx_ovstage_published TRUE)
        endif()
    endif()

    if(_ovphysx_ovstage_published)
        file(REMOVE_RECURSE "${_ovphysx_ovstage_staging}")
    else()
        set(_ovphysx_cache_occupied FALSE)
        if(EXISTS "${OVPHYSX_OVSTAGE_CACHE_DIR}")
            file(GLOB _ovphysx_cache_entries "${OVPHYSX_OVSTAGE_CACHE_DIR}/*")
            if(_ovphysx_cache_entries)
                set(_ovphysx_cache_occupied TRUE)
            endif()
        endif()
        if(_ovphysx_cache_occupied AND NOT EXISTS "${_ovphysx_ovstage_stamp}")
            file(REMOVE_RECURSE "${_ovphysx_ovstage_staging}")
            set(ovphysx_OVSTAGE_BOOTSTRAP_ERROR
                "${OVPHYSX_OVSTAGE_CACHE_DIR} is not empty and was not created by ovphysx, so it was left untouched. Point OVPHYSX_OVSTAGE_CACHE_DIR at a dedicated directory, or add the OVStage package there to CMAKE_PREFIX_PATH.")
            return()
        endif()
        file(REMOVE_RECURSE "${OVPHYSX_OVSTAGE_CACHE_DIR}")
        file(RENAME "${_ovphysx_ovstage_staging}" "${OVPHYSX_OVSTAGE_CACHE_DIR}")
    endif()
endif()

if(NOT EXISTS "${OVPHYSX_OVSTAGE_CACHE_DIR}/include")
    set(ovphysx_OVSTAGE_BOOTSTRAP_ERROR
        "the cached OVStage at ${OVPHYSX_OVSTAGE_CACHE_DIR} has no include/ directory")
    return()
endif()

set(ovphysx_OVSTAGE_ROOT "${OVPHYSX_OVSTAGE_CACHE_DIR}")
list(APPEND CMAKE_PREFIX_PATH "${ovphysx_OVSTAGE_ROOT}")
set(_OVPHYSX_OVSTAGE_BOOTSTRAP_DONE TRUE)

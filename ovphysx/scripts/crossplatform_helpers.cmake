# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# Cross-Platform Helpers

include_guard(GLOBAL)

# Detect target operating system
# In script mode (cmake -P), CMAKE_SYSTEM_NAME is not set, so fall back to CMAKE_HOST_SYSTEM_NAME
if(NOT CMAKE_SYSTEM_NAME)
    set(CMAKE_SYSTEM_NAME "${CMAKE_HOST_SYSTEM_NAME}")
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(EXE_SUFFIX ".exe")
    set(SCRIPT_SUFFIX ".bat")
    set(OS_NAME "windows")
    set(PATH_SEP ";")
    set(PATH_VAR "PATH")
    set(PLUGIN_EXTENSION ".dll")
    set(SHARED_LIB_PREFIX "")
    set(SHARED_LIB_SUFFIX ".dll")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(EXE_SUFFIX "")
    set(SCRIPT_SUFFIX ".sh")
    set(OS_NAME "linux")
    set(PATH_SEP ":")
    set(PATH_VAR "LD_LIBRARY_PATH")
    set(PLUGIN_EXTENSION ".plugin.so")
    set(SHARED_LIB_PREFIX "lib")
    set(SHARED_LIB_SUFFIX ".so")
else()
    message(FATAL_ERROR "Unknown target OS: ${CMAKE_SYSTEM_NAME}")
endif()

# Detect target architecture
# In script mode (cmake -P), CMAKE_SYSTEM_PROCESSOR may not be set, so fall back to CMAKE_HOST_SYSTEM_PROCESSOR
if(NOT CMAKE_SYSTEM_PROCESSOR)
    set(CMAKE_SYSTEM_PROCESSOR "${CMAKE_HOST_SYSTEM_PROCESSOR}")
endif()

# If still empty, try to detect using system commands (see bug https://gitlab.kitware.com/cmake/cmake/-/issues/25151)
if(NOT CMAKE_SYSTEM_PROCESSOR OR CMAKE_SYSTEM_PROCESSOR STREQUAL "")
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        # On Windows, check the PROCESSOR_ARCHITECTURE environment variable.
        # A 32-bit CMake running under WOW64 on a 64-bit host (e.g. the CMake
        # bundled with VS Build Tools) reports PROCESSOR_ARCHITECTURE=x86 while
        # the true host arch is in PROCESSOR_ARCHITEW6432 (AMD64 or ARM64).
        # Prefer PROCESSOR_ARCHITEW6432 when set so the host is detected
        # correctly regardless of CMake bitness.
        if(NOT "$ENV{PROCESSOR_ARCHITEW6432}" STREQUAL "")
            set(CMAKE_SYSTEM_PROCESSOR "$ENV{PROCESSOR_ARCHITEW6432}")
        else()
            set(CMAKE_SYSTEM_PROCESSOR "$ENV{PROCESSOR_ARCHITECTURE}")
        endif()
    else()
        # On Unix-like systems, use uname -m
        execute_process(
            COMMAND uname -m
            OUTPUT_VARIABLE CMAKE_SYSTEM_PROCESSOR
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
    endif()
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)")
    set(ARCH_NAME "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|armv7|ARM)")
    set(ARCH_NAME "arm")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)")
    set(ARCH_NAME "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(i386|i686|x86)")
    set(ARCH_NAME "x86")
else()
    message(FATAL_ERROR "Unknown target architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Combined platform identifiers
set(PLATFORM_NAME "${OS_NAME}-${ARCH_NAME}")

# Install directory layout
# Windows: DLLs go in bin/, Linux: .so files go in lib/
# Deps always go in lib/deps/ on both platforms
if(OS_NAME STREQUAL "windows")
    set(INSTALL_RUNTIME_SUBDIR "bin")
else()
    set(INSTALL_RUNTIME_SUBDIR "lib")
endif()
set(INSTALL_DEPS_SUBDIR "lib/deps")

# SDK package archive extension
if(OS_NAME STREQUAL "windows")
    set(SDK_ARCHIVE_EXT "zip")
else()
    set(SDK_ARCHIVE_EXT "tar.gz")
endif()

# Copy a single file only when content differs (avoids timestamp churn).
function(copy_file_if_different SRC_FILE DST_FILE)
    if(NOT EXISTS "${SRC_FILE}" OR IS_DIRECTORY "${SRC_FILE}")
        message(FATAL_ERROR "Expected file not found: ${SRC_FILE}")
    endif()
    get_filename_component(DST_DIR "${DST_FILE}" DIRECTORY)
    file(MAKE_DIRECTORY "${DST_DIR}")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SRC_FILE}" "${DST_FILE}"
        RESULT_VARIABLE COPY_RESULT
    )
    if(NOT COPY_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to copy file: ${SRC_FILE} -> ${DST_FILE} (exit code: ${COPY_RESULT})")
    endif()
endfunction()

# Recursively copy a directory tree, only updating files whose content changed.
# EXCLUDE_DIRS matches any path component below SRC_DIR, file names included:
# a file named like an excluded directory is skipped too.
function(copy_tree_if_different SRC_DIR DST_DIR)
    cmake_parse_arguments(_CT "" "" "EXCLUDE_DIRS" ${ARGN})
    if(NOT IS_DIRECTORY "${SRC_DIR}")
        message(FATAL_ERROR "Expected directory not found: ${SRC_DIR}")
    endif()
    file(MAKE_DIRECTORY "${DST_DIR}")
    file(GLOB_RECURSE SRC_ITEMS RELATIVE "${SRC_DIR}" "${SRC_DIR}/*")
    set(_SKIPPED 0)
    foreach(REL_PATH IN LISTS SRC_ITEMS)
        set(SRC_PATH "${SRC_DIR}/${REL_PATH}")
        if(IS_DIRECTORY "${SRC_PATH}")
            continue()
        endif()
        set(_EXCLUDED FALSE)
        string(REPLACE "/" ";" _REL_PARTS "${REL_PATH}")
        foreach(_PART IN LISTS _REL_PARTS)
            if("${_PART}" IN_LIST _CT_EXCLUDE_DIRS)
                set(_EXCLUDED TRUE)
                break()
            endif()
        endforeach()
        if(_EXCLUDED)
            math(EXPR _SKIPPED "${_SKIPPED} + 1")
            continue()
        endif()
        copy_file_if_different("${SRC_PATH}" "${DST_DIR}/${REL_PATH}")
    endforeach()
    if(_SKIPPED GREATER 0)
        message(STATUS "  Skipped ${_SKIPPED} files in excluded dirs: ${_CT_EXCLUDE_DIRS}")
    endif()
endfunction()

# Stage a Python samples tree into a package tree, leaving developer-local
# state behind.  .venv/, __pycache__/ and .pytest_cache/ are
# build-environment-specific and must not ship.  They have to be excluded from
# the copy rather than deleted after it: .venv/bin/python is a symlink to an
# interpreter outside the repository, which copy_file_if_different() cannot
# resolve, so merely walking the tree aborts the wheel build on any machine
# that has run the python-samples tests.
function(stage_python_samples_tree SRC_DIR DST_DIR)
    copy_tree_if_different("${SRC_DIR}" "${DST_DIR}"
        EXCLUDE_DIRS .venv __pycache__ .pytest_cache)
    # The lock file pins one local resolution; wheel consumers re-resolve.
    if(EXISTS "${DST_DIR}/uv.lock")
        file(REMOVE "${DST_DIR}/uv.lock")
    endif()
endfunction()

# Function to convert semver to PEP 440 format for Python
# Semver: X.Y.Z-suffix (hyphen for pre-release)
# PEP 440: X.Y.Z.suffix (dot for local version segment)
# Sets PEP440_VERSION in parent scope
function(semver_to_pep440 SEMVER_VERSION)
    string(REPLACE "-" "." PEP440_RESULT "${SEMVER_VERSION}")
    set(PEP440_VERSION "${PEP440_RESULT}" PARENT_SCOPE)
endfunction()

# Git helpers (prefer CI vars, fall back to git)
function(get_git_sha OUTPUT_VAR WORKING_DIR SHORT)
    if(NOT DEFINED WORKING_DIR OR WORKING_DIR STREQUAL "")
        set(WORKING_DIR ".")
    endif()
    if(SHORT)
        set(GIT_SHA "$ENV{CI_COMMIT_SHORT_SHA}")
        set(GIT_REV_PARSE_ARGS "--short" "HEAD")
    else()
        set(GIT_SHA "$ENV{CI_COMMIT_SHA}")
        set(GIT_REV_PARSE_ARGS "HEAD")
    endif()
    if(GIT_SHA STREQUAL "")
        execute_process(
            COMMAND git rev-parse ${GIT_REV_PARSE_ARGS}
            WORKING_DIRECTORY "${WORKING_DIR}"
            OUTPUT_VARIABLE GIT_SHA
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE GIT_RESULT
        )
        if(NOT GIT_RESULT STREQUAL "0")
            message(WARNING "Failed to get git sha (git rev-parse ${GIT_REV_PARSE_ARGS}): ${GIT_RESULT} ${GIT_SHA}")
            set(GIT_SHA "")
        endif()
    endif()
    set(${OUTPUT_VAR} "${GIT_SHA}" PARENT_SCOPE)
endfunction()

function(get_git_branch OUTPUT_VAR WORKING_DIR)
    if(NOT DEFINED WORKING_DIR OR WORKING_DIR STREQUAL "")
        set(WORKING_DIR ".")
    endif()
    set(BRANCH "$ENV{CI_COMMIT_REF_NAME}")
    if(BRANCH STREQUAL "")
        execute_process(
            COMMAND git rev-parse --abbrev-ref HEAD
            WORKING_DIRECTORY "${WORKING_DIR}"
            OUTPUT_VARIABLE BRANCH
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE GIT_RESULT
        )
        if(NOT GIT_RESULT STREQUAL "0")
            message(WARNING "Failed to get git branch (git rev-parse --abbrev-ref HEAD): ${GIT_RESULT} ${BRANCH}")
            set(BRANCH "")
        endif()
    endif()
    set(${OUTPUT_VAR} "${BRANCH}" PARENT_SCOPE)
endfunction()

# Append branch info to PEP 440 version as local segment (e.g. 1.2.3+fix.test.ab12cd)
# Output variable is set in parent scope.
function(append_branch_local_version BASE_VERSION WORKING_DIR OUTPUT_VAR)
    if(NOT DEFINED WORKING_DIR OR WORKING_DIR STREQUAL "")
        set(WORKING_DIR ".")
    endif()
    set(VERSION_WITH_LOCAL "${BASE_VERSION}")
    get_git_branch(GIT_BRANCH "${WORKING_DIR}")
    if(NOT GIT_BRANCH STREQUAL "")
        string(TOLOWER "${GIT_BRANCH}" GIT_BRANCH)
        if(GIT_BRANCH MATCHES "^release/")
            set(${OUTPUT_VAR} "${VERSION_WITH_LOCAL}" PARENT_SCOPE)
            return()
        endif()
        string(REGEX REPLACE "[^a-z0-9]+" "." GIT_BRANCH "${GIT_BRANCH}")
        string(REGEX REPLACE "^\\.+" "" GIT_BRANCH "${GIT_BRANCH}")
        string(REGEX REPLACE "\\.+$" "" GIT_BRANCH "${GIT_BRANCH}")
        if(NOT GIT_BRANCH STREQUAL "")
            set(LOCAL_SUFFIX "${GIT_BRANCH}")
            get_git_sha(GIT_SHA "${WORKING_DIR}" TRUE)
            if(NOT GIT_SHA STREQUAL "")
                string(TOLOWER "${GIT_SHA}" GIT_SHA)
                string(REGEX REPLACE "[^a-z0-9]+" "" GIT_SHA "${GIT_SHA}")
                if(NOT GIT_SHA STREQUAL "")
                    set(LOCAL_SUFFIX "${LOCAL_SUFFIX}.${GIT_SHA}")
                endif()
            endif()
            set(VERSION_WITH_LOCAL "${BASE_VERSION}+${LOCAL_SUFFIX}")
        endif()
    endif()
    set(${OUTPUT_VAR} "${VERSION_WITH_LOCAL}" PARENT_SCOPE)
endfunction()

# Function to get Python wheel platform tag
# Sets WHEEL_PLAT_NAME in parent scope
# Uses the already-detected OS_NAME and ARCH_NAME variables
#
# Linux wheels use PEP 600 manylinux tags to communicate the minimum glibc
# version required. This is standard practice for GPU-capable wheels (PyTorch,
# CuPy, etc.) even though a strict auditwheel check may flag conditionally-
# loaded GPU plugins.
function(get_wheel_platform_tag)
    if(OS_NAME STREQUAL "linux")
        if(ARCH_NAME STREQUAL "x86_64")
            set(WHEEL_PLAT_NAME "manylinux_2_35_x86_64" PARENT_SCOPE)
        elseif(ARCH_NAME STREQUAL "aarch64")
            set(WHEEL_PLAT_NAME "manylinux_2_35_aarch64" PARENT_SCOPE)
        else()
            set(WHEEL_PLAT_NAME "linux_${ARCH_NAME}" PARENT_SCOPE)
        endif()
    elseif(OS_NAME STREQUAL "windows")
        if(ARCH_NAME STREQUAL "x86_64")
            set(WHEEL_PLAT_NAME "win_amd64" PARENT_SCOPE)
        elseif(ARCH_NAME STREQUAL "aarch64")
            set(WHEEL_PLAT_NAME "win_arm64" PARENT_SCOPE)
        else()
            set(WHEEL_PLAT_NAME "win_${ARCH_NAME}" PARENT_SCOPE)
        endif()
    else()
        message(FATAL_ERROR "Unknown OS: ${OS_NAME}")
    endif()
endfunction()

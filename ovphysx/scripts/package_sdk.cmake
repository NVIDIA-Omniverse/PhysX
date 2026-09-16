# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# ovphysx SDK packaging script.
# Creates a distributable SDK archive from _install/ into _dist/.
# Usage: cmake [options] -P scripts/package_sdk.cmake
#
# Prerequisites: Run scripts/install.cmake first
#
# Options (passed via -D flags):
#   -DBUILD_TYPE=Debug|Release    Build type (default: Release)

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
get_filename_component(PROJECT_ROOT "${SCRIPT_DIR}/.." ABSOLUTE)

include("${SCRIPT_DIR}/build_common.cmake")
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")

if(NOT EXISTS "${PROJECT_ROOT}/_install")
    message(FATAL_ERROR "Install directory not found. Run 'cmake -P scripts/install.cmake' first.")
endif()

# The VERSION file is the single source of truth for the package version.
file(READ "${PROJECT_ROOT}/VERSION" PACKAGE_VERSION)
string(STRIP "${PACKAGE_VERSION}" PACKAGE_VERSION)
if(NOT PACKAGE_VERSION)
    message(FATAL_ERROR "Could not read version from VERSION file")
endif()

set(PACKAGE_OS "${OS_NAME}")
set(PACKAGE_ARCH "${ARCH_NAME}")

set(COMPONENT_NAME "ovphysx")
set(ARCHIVE_NAME "${COMPONENT_NAME}-${PACKAGE_OS}-${PACKAGE_ARCH}-${PACKAGE_VERSION}.${SDK_ARCHIVE_EXT}")
set(DIST_DIR "${PROJECT_ROOT}/_dist")
set(ARCHIVE_PATH "${DIST_DIR}/${ARCHIVE_NAME}")

file(MAKE_DIRECTORY "${DIST_DIR}")

message(STATUS "Creating SDK package...")
message(STATUS "  Component: ${COMPONENT_NAME}")
message(STATUS "  Version: ${PACKAGE_VERSION}")
message(STATUS "  Platform: ${PACKAGE_OS}-${PACKAGE_ARCH}")
message(STATUS "  Archive: ${ARCHIVE_NAME}")

# The archive is only rebuilt when an input is newer than it.
set(NEED_REBUILD FALSE)
if(NOT EXISTS "${ARCHIVE_PATH}")
    set(NEED_REBUILD TRUE)
    message(STATUS "  Archive doesn't exist, will create")
else()
    file(TIMESTAMP "${ARCHIVE_PATH}" ARCHIVE_TIME "%s")

    # Check whether any file in _install is newer than the archive.
    file(GLOB_RECURSE INSTALL_FILES "${PROJECT_ROOT}/_install/**/*")
    set(CHANGED_FILES "")
    foreach(INSTALL_FILE ${INSTALL_FILES})
        if(NOT IS_DIRECTORY "${INSTALL_FILE}")
            file(TIMESTAMP "${INSTALL_FILE}" FILE_TIME "%s")
            if(FILE_TIME GREATER ARCHIVE_TIME)
                list(APPEND CHANGED_FILES "${INSTALL_FILE}")
                list(LENGTH CHANGED_FILES CHANGED_COUNT)
                if(CHANGED_COUNT GREATER_EQUAL 10)
                    break()
                endif()
            endif()
        endif()
    endforeach()

    list(LENGTH CHANGED_FILES CHANGED_COUNT)
    if(CHANGED_COUNT GREATER 0)
        set(NEED_REBUILD TRUE)
        message(STATUS "  Install files changed (${CHANGED_COUNT}+ newer than archive), will rebuild:")
        foreach(CHANGED_FILE ${CHANGED_FILES})
            message(STATUS "    - ${CHANGED_FILE}")
        endforeach()
    endif()

    if(NOT NEED_REBUILD)
        file(TIMESTAMP "${PROJECT_ROOT}/VERSION" VERSION_TIME "%s")
        if(VERSION_TIME GREATER ARCHIVE_TIME)
            set(NEED_REBUILD TRUE)
            message(STATUS "  Version changed, will rebuild archive")
        endif()
    endif()

    if(NOT NEED_REBUILD)
        file(TIMESTAMP "${CMAKE_CURRENT_LIST_FILE}" PACKAGE_SCRIPT_TIME "%s")
        if(PACKAGE_SCRIPT_TIME GREATER ARCHIVE_TIME)
            set(NEED_REBUILD TRUE)
            message(STATUS "  Packaging script changed, will rebuild archive")
        endif()
    endif()

    if(NOT NEED_REBUILD)
        message(STATUS "  Archive is up-to-date, skipping rebuild")
    endif()
endif()

if(NEED_REBUILD)
    # Stage the install tree under a component-named root so the archive unpacks into one directory.
    set(TEMP_PACKAGE_DIR "${PROJECT_ROOT}/_package_temp")
    file(REMOVE_RECURSE "${TEMP_PACKAGE_DIR}")
    file(MAKE_DIRECTORY "${TEMP_PACKAGE_DIR}")

    message(STATUS "  Copying install tree to temporary package directory...")
    file(COPY "${PROJECT_ROOT}/_install/"
         DESTINATION "${TEMP_PACKAGE_DIR}/${COMPONENT_NAME}"
         USE_SOURCE_PERMISSIONS)

    message(STATUS "  Creating archive...")
    if(WIN32)
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E tar cf "${ARCHIVE_PATH}" --format=zip -- "${COMPONENT_NAME}"
            WORKING_DIRECTORY "${TEMP_PACKAGE_DIR}"
            RESULT_VARIABLE TAR_RESULT
        )
    else()
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E tar czf "${ARCHIVE_PATH}" "${COMPONENT_NAME}"
            WORKING_DIRECTORY "${TEMP_PACKAGE_DIR}"
            RESULT_VARIABLE TAR_RESULT
        )
    endif()

    file(REMOVE_RECURSE "${TEMP_PACKAGE_DIR}")

    if(NOT TAR_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to create archive (exit code: ${TAR_RESULT})")
    endif()

    file(SIZE "${ARCHIVE_PATH}" ARCHIVE_SIZE)
    math(EXPR ARCHIVE_SIZE_MB "${ARCHIVE_SIZE} / 1048576")

    message(STATUS "SDK package created successfully:")
    message(STATUS "  Path: ${ARCHIVE_PATH}")
    message(STATUS "  Size: ${ARCHIVE_SIZE_MB} MB")
endif()

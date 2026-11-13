# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if(NOT DEFINED BUILD_DIR)
    if(DEFINED ENV{OVPHYSX_BUILD_DIR} AND NOT "$ENV{OVPHYSX_BUILD_DIR}" STREQUAL "")
        set(BUILD_DIR "$ENV{OVPHYSX_BUILD_DIR}")
    else()
        set(BUILD_DIR "_build")
    endif()
endif()
set(OVPHYSX_BUILD_ROOT "${PROJECT_ROOT}/_build")
get_filename_component(BUILD_PATH "${BUILD_DIR}" ABSOLUTE BASE_DIR "${PROJECT_ROOT}")

# Build variants share one local cache root. Never accept an arbitrary directory
# as disposable build output, even when the caller only intends to configure.
get_filename_component(OVPHYSX_BUILD_ROOT "${OVPHYSX_BUILD_ROOT}" ABSOLUTE)
function(ovphysx_require_build_path path)
    get_filename_component(_build_path "${path}" ABSOLUTE BASE_DIR "${PROJECT_ROOT}")
    set(_build_root_compare "${OVPHYSX_BUILD_ROOT}")
    set(_build_path_compare "${_build_path}")
    if(CMAKE_HOST_WIN32)
        string(TOLOWER "${_build_root_compare}" _build_root_compare)
        string(TOLOWER "${_build_path_compare}" _build_path_compare)
    endif()
    string(FIND "${_build_path_compare}/" "${_build_root_compare}/" _build_prefix)
    if(NOT _build_prefix EQUAL 0)
        message(FATAL_ERROR "Build directories must be inside '${OVPHYSX_BUILD_ROOT}'. "
                            "Use _build or a variant such as _build/debug.")
    endif()

    # A directory link could redirect a valid-looking path outside the cache root.
    # IS_SYMLINK also recognizes Windows junctions; REALPATH is not used for this.
    while(1)
        if(IS_SYMLINK "${_build_path}")
            message(FATAL_ERROR "Build directories must not use symbolic links or junctions: '${_build_path}'.")
        endif()
        if(_build_path_compare STREQUAL _build_root_compare)
            break()
        endif()
        get_filename_component(_build_path "${_build_path}" DIRECTORY)
        set(_build_path_compare "${_build_path}")
        if(CMAKE_HOST_WIN32)
            string(TOLOWER "${_build_path_compare}" _build_path_compare)
        endif()
    endwhile()
endfunction()

ovphysx_require_build_path("${BUILD_PATH}")

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Content identity includes uncommitted source edits and the pinned dependencies.
# Configure only lists the inputs (CONFIGURE_DEPENDS notices additions and removals);
# the build hashes them, so an edit regenerates one header instead of reconfiguring.
set(_cache_identity_files "")
# The shared ovphysx/cmake modules set compile options for every target.
get_filename_component(_cache_shared_cmake "${OVRUNTIME_ROOT_DIR}/../cmake" ABSOLUTE)
foreach(_cache_root IN ITEMS "${OVRUNTIME_ROOT_DIR}" "${PHYSX_SDK_DIR}" "${_cache_shared_cmake}")
    if("${_cache_root}" STREQUAL "${OVRUNTIME_ROOT_DIR}")
        set(_cache_dirs include source cmake deps)
    elseif("${_cache_root}" STREQUAL "${_cache_shared_cmake}")
        set(_cache_dirs .)
    else()
        set(_cache_dirs include source)
    endif()
    foreach(_cache_dir IN LISTS _cache_dirs)
        file(GLOB_RECURSE _cache_found CONFIGURE_DEPENDS "${_cache_root}/${_cache_dir}/*")
        foreach(_cache_file IN LISTS _cache_found)
            # Match tests below the root only; the checkout itself may sit under a tests directory.
            file(RELATIVE_PATH _cache_relative "${_cache_root}" "${_cache_file}")
            if(NOT _cache_relative MATCHES "(^|/)tests/|/__pycache__/|\\.pyc$")
                list(APPEND _cache_identity_files "${_cache_file}")
            endif()
        endforeach()
    endforeach()
endforeach()
if(NOT OVRUNTIME_DEV_PHYSX)
    file(GLOB_RECURSE _cache_sdk_binaries "${PHYSX_SDK_DIR}/bin/*.a" "${PHYSX_SDK_DIR}/bin/*.lib")
    list(APPEND _cache_identity_files ${_cache_sdk_binaries})
endif()
if(EXISTS "${OVSTAGE_LIBRARY}")
    list(APPEND _cache_identity_files "${OVSTAGE_LIBRARY}")
endif()
list(SORT _cache_identity_files)
list(APPEND _cache_identity_files "${OVRUNTIME_ROOT_DIR}/CMakeLists.txt")
set(_cache_identity "${CMAKE_SYSTEM_NAME};${CMAKE_SYSTEM_PROCESSOR};${CMAKE_CXX_COMPILER_ID};${CMAKE_CXX_COMPILER_VERSION};${CMAKE_CXX_FLAGS};${CMAKE_CXX_FLAGS_RELEASE};${CMAKE_CXX_FLAGS_DEBUG};${CMAKE_BUILD_TYPE};${OVRUNTIME_DEV_PHYSX}")
get_cmake_property(_cache_build_options CACHE_VARIABLES)
list(SORT _cache_build_options)
foreach(_cache_option IN LISTS _cache_build_options)
    if(_cache_option MATCHES "^(PX_|PHYSX_|OVRUNTIME_|CMAKE_CXX_)")
        string(APPEND _cache_identity "${_cache_option}=${${_cache_option}};")
    endif()
endforeach()
# Written only when changed, so an unchanged configure does not rehash.
set(_cache_identity_text "${CMAKE_CURRENT_BINARY_DIR}/CompiledCacheBuildIdentity.txt")
set(_cache_files_text "${CMAKE_CURRENT_BINARY_DIR}/CompiledCacheBuildFiles.txt")
string(REPLACE ";" "\n" _cache_files_lines "${_cache_identity_files}")
file(CONFIGURE OUTPUT "${_cache_identity_text}" CONTENT "${_cache_identity}" @ONLY)
file(CONFIGURE OUTPUT "${_cache_files_text}" CONTENT "${_cache_files_lines}\n" @ONLY)
# Multi-config generators (Visual Studio) leave CMAKE_BUILD_TYPE empty and build
# several configurations in one tree: each hashes its own name into its own header.
set(_cache_build_dir "${CMAKE_CURRENT_BINARY_DIR}/compiled-cache-identity/$<CONFIG>")
add_custom_command(
    OUTPUT "${_cache_build_dir}/CompiledCacheBuild.h"
    COMMAND "${CMAKE_COMMAND}" "-DIDENTITY=${_cache_identity_text}" "-DFILES=${_cache_files_text}"
            "-DCONFIG=$<CONFIG>" "-DOUTPUT=${_cache_build_dir}/CompiledCacheBuild.h"
            -P "${OVRUNTIME_ROOT_DIR}/cmake/CompiledCacheIdentityHash.cmake"
    DEPENDS ${_cache_identity_files} "${_cache_identity_text}" "${_cache_files_text}"
            "${OVRUNTIME_ROOT_DIR}/cmake/CompiledCacheIdentityHash.cmake"
    COMMENT "Hashing the compiled-cache build identity"
    VERBATIM)
target_sources(OvruntimePhysX PRIVATE "${_cache_build_dir}/CompiledCacheBuild.h")
target_include_directories(OvruntimePhysX PRIVATE "${_cache_build_dir}")

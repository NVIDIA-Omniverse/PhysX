# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Bind external modules to the actual headers and packaged native archives, not
# just PX_PHYSICS_VERSION (which does not identify an SDK commit/build).
get_filename_component(_comp_sdk_root "${_OVPHYSX_PHYSX_INCLUDE_SOURCE}" DIRECTORY)
file(GLOB_RECURSE _comp_sdk_files RELATIVE "${_comp_sdk_root}"
    "${_comp_sdk_root}/include/*.h" "${_comp_sdk_root}/include/*.hpp"
    "${_comp_sdk_root}/include/*.inl" "${_comp_sdk_root}/bin/*.a" "${_comp_sdk_root}/bin/*.lib")
list(SORT _comp_sdk_files)
set(_comp_sdk_manifest "${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}\n")
foreach(_comp_sdk_file IN LISTS _comp_sdk_files)
    file(SHA256 "${_comp_sdk_root}/${_comp_sdk_file}" _comp_sdk_hash)
    string(APPEND _comp_sdk_manifest "${_comp_sdk_file}:${_comp_sdk_hash}\n")
endforeach()
string(SHA256 COMPANION_PHYSX_BUILD "${_comp_sdk_manifest}")
file(MAKE_DIRECTORY "${OVPHYSX_GENERATED_INCLUDE_DIR}/ovphysx_companion")
set(OVPHYSX_COMPANION_DEBUG_BUILD 0)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(OVPHYSX_COMPANION_DEBUG_BUILD 1)
endif()
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/companion/cmake/native_build.h.in"
    "${OVPHYSX_GENERATED_INCLUDE_DIR}/ovphysx_companion/native_build.h" @ONLY)

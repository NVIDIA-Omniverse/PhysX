# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# Collect header files (public API)
set(COMMON_HEADERS
    # foundation
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/Algorithms.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/Allocator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/Preprocessor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/TypeCast.h
    # ui
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/ui/ImguiDrawingUtils.h
    # utilities
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/CudaHelpers.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/Defer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/LockingList.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/MemoryMacros.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/MemoryUtilities.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/OmniPhysXUtilities.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/PhysXErrorCallback.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/PrimHierarchyStorage.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/PrimUtilities.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/UsdMaterialParsing.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/Utilities.h
)
source_group(include FILES ${COMMON_HEADERS})

# Collect source files
set(COMMON_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/source/foundation/Allocator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/source/ui/ImguiDrawingUtils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/source/utilities/OptionalCudaShim.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/source/utilities/PhysXErrorCallback.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/source/utilities/PrimHierarchyStorage.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/source/utilities/PrimUtilities.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/source/utilities/UsdMaterialParsing.cpp
)
source_group(source FILES ${COMMON_SOURCE})

# PCH files
set(COMMON_PCH
    ${OVRUNTIME_ROOT_DIR}/source/pch/UsdPCH.h
    ${OVRUNTIME_ROOT_DIR}/source/pch/UsdPCH.cpp
)
source_group(pch FILES ${COMMON_PCH})

# Create the static library (matching old premake "foundation" target)
add_library(OvruntimeCommon STATIC
    ${COMMON_HEADERS}
    ${COMMON_SOURCE}
    ${COMMON_PCH}
)

set_property(TARGET OvruntimeCommon PROPERTY FOLDER "Runtime/Common")

# RTTI enabled (matching old premake config)
set_target_properties(OvruntimeCommon PROPERTIES
    OUTPUT_NAME "OvruntimeCommon"
)

# Include directories
target_include_directories(OvruntimeCommon
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include             # <common/foundation/...>, <common/utilities/...>, <common/ui/...>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation
        ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities
        ${CMAKE_CURRENT_SOURCE_DIR}/include/common/ui
        ${OVRUNTIME_ROOT_DIR}/source/pch                # UsdPCH.h
        ${OVRUNTIME_ROOT_DIR}/include                   # <omni/physx/...>, <omni/physics/...>
        ${OVRUNTIME_ROOT_DIR}/include/private           # <private/omni/...>
)

# External dependencies — SYSTEM includes to suppress third-party warnings
ovruntime_add_external_system_includes(OvruntimeCommon)

# Platform-specific settings
if(WIN32)
    # Disable vectorized STL algorithms for maximum MSVC toolset compatibility.
    target_compile_definitions(OvruntimeCommon PRIVATE _USE_STD_VECTOR_ALGORITHMS=0)
else()
    # Match old premake: exceptions on, fPIC, suppress some warnings
    target_compile_options(OvruntimeCommon PRIVATE -fexceptions -fPIC)
endif()

# Precompiled header: use UsdPCH.h for source files that include it
# The PCH is used by most .cpp files except Allocator.cpp
target_precompile_headers(OvruntimeCommon PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${OVRUNTIME_ROOT_DIR}/source/pch/UsdPCH.h>"
)

# Install rules - public headers (preserving directory structure)
install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/Algorithms.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/Allocator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/Preprocessor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/foundation/TypeCast.h
    DESTINATION include/common/foundation
)

install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/ui/ImguiDrawingUtils.h
    DESTINATION include/common/ui
)

install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/CudaHelpers.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/Defer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/LockingList.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/MemoryMacros.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/MemoryUtilities.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/OmniPhysXUtilities.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/PhysXErrorCallback.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/PrimHierarchyStorage.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/PrimUtilities.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/UsdMaterialParsing.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/utilities/Utilities.h
    DESTINATION include/common/utilities
)

install(
    TARGETS OvruntimeCommon
    EXPORT OvruntimeCommon
    ARCHIVE DESTINATION lib
)

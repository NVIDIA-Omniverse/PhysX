# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# --- Plugin headers (private) ---
set(CONVEXDECOMP_PLUGIN_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/FM.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/JobSystem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/MergeConvexHulls.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Qh.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/QhBounds3.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/QhMath.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/QhPreprocessor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/QhVec3.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Quantizer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/QuickHull.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/RaycastMesh.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ScopedTime.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ShrinkWrap.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/SimpleMesh.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/SphereApprox.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/TriangulateUsdMeshPrim.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Vector.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Voxel.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/VoxelFillMode.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/VoxelTriMesh.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/aabb.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/vcd.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Volume.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/voxelize.h
)
source_group(plugins FILES ${CONVEXDECOMP_PLUGIN_HEADERS})

# --- Plugin sources ---
set(CONVEXDECOMP_PLUGIN_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ConvexDecomposition.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/FM.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/JobSystem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/MergeConvexHulls.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Quantizer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/QuickHull.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/RaycastMesh.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ShrinkWrap.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/TriangulateUsdMeshPrim.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Volume.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/VoxelTriMesh.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/aabb.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/vcd.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/voxelize.cpp
)
source_group(source FILES ${CONVEXDECOMP_PLUGIN_SOURCE})

# --- Public header ---
set(CONVEXDECOMP_PUBLIC_HEADERS
    ${OVRUNTIME_ROOT_DIR}/include/omni/convexdecomposition/ConvexDecomposition.h
)
source_group(include FILES ${CONVEXDECOMP_PUBLIC_HEADERS})

# --- Create static library ---
add_library(OvruntimeConvexDecomposition STATIC
    ${CONVEXDECOMP_PUBLIC_HEADERS}
    ${CONVEXDECOMP_PLUGIN_HEADERS}
    ${CONVEXDECOMP_PLUGIN_SOURCE}
)

set_property(TARGET OvruntimeConvexDecomposition PROPERTY FOLDER "Runtime/ConvexDecomposition")
set_property(TARGET OvruntimeConvexDecomposition PROPERTY POSITION_INDEPENDENT_CODE ON)

# RTTI and exceptions enabled (matching old premake)
if(NOT WIN32)
    target_compile_options(OvruntimeConvexDecomposition PRIVATE -fexceptions -frtti -fPIC)
endif()

# Include directories
target_include_directories(OvruntimeConvexDecomposition
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins          # internal headers
        ${OVRUNTIME_ROOT_DIR}/include                 # <omni/convexdecomposition/...>
        ${OVRUNTIME_ROOT_DIR}/source/pch              # UsdPCH.h
        ${OVRUNTIME_ROOT_DIR}/source/common/include   # <common/utilities/...>
)

# External dependencies — SYSTEM includes to suppress third-party warnings
ovruntime_add_external_system_includes(OvruntimeConvexDecomposition)

# Compile definitions
target_compile_definitions(OvruntimeConvexDecomposition PRIVATE
    "carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)"
)

# Link dependencies
target_link_libraries(OvruntimeConvexDecomposition
    PRIVATE OvruntimeCommon
)

if(DEFINED CARB_SDK_DIR)
    set(_CARB_LIB_DIR "${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}")
    target_link_directories(OvruntimeConvexDecomposition PRIVATE
        ${_CARB_LIB_DIR}/${OVRUNTIME_CARB_CONFIG}
    )
    target_link_libraries(OvruntimeConvexDecomposition PRIVATE carb)
endif()

# USD, TBB, Python libraries (matching premake extension_usd_deps)
ovruntime_link_usd_deps(OvruntimeConvexDecomposition)

# Platform-specific: suppress -Wstrict-aliasing for FM.cpp on Linux
if(NOT WIN32)
    set_source_files_properties(
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins/FM.cpp
        PROPERTIES COMPILE_OPTIONS "-Wno-strict-aliasing"
    )
endif()

# Precompiled header
target_precompile_headers(OvruntimeConvexDecomposition PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${OVRUNTIME_ROOT_DIR}/source/pch/UsdPCH.h>"
)

# Install rules
install(FILES
    ${OVRUNTIME_ROOT_DIR}/include/omni/convexdecomposition/ConvexDecomposition.h
    DESTINATION include/omni/convexdecomposition
)

install(
    TARGETS OvruntimeConvexDecomposition
    EXPORT OvruntimeConvexDecomposition
    ARCHIVE DESTINATION lib
)

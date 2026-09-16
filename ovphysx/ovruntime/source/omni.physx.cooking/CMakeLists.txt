# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# --- Plugin headers (private) ---
set(COOKING_PLUGIN_HEADERS
    # service
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/service/CookingComputeService.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/service/CookingHashing.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/service/CookingTask.h
    # utility
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/utility/MeshSimplifyInternal.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/utility/MeshThickness.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/utility/TriangulateUsdMeshPrim.h
)
source_group(plugins\\headers FILES ${COOKING_PLUGIN_HEADERS})

# --- Plugin sources ---
set(COOKING_PLUGIN_SOURCE
    # entry point
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXCookingService.cpp
    # service
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/service/CookingComputeService.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/service/CookingTask.cpp
    # tasks
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tasks/ConvexDecompositionCookingTask.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tasks/ConvexMeshCookingTask.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tasks/DeformableVolumeMeshCookingTask.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tasks/ParticlePoissonSamplingCookingTask.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tasks/SphereFillCookingTask.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tasks/SurfaceDeformableBodyCookingTask.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tasks/TriangleMeshCookingTask.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tasks/VolumeDeformableBodyCookingTask.cpp
    # utility
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/utility/MeshSimplifyInternal.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/utility/MeshThickness.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/utility/TriangulateUsdMeshPrim.cpp
)
source_group(plugins\\source FILES ${COOKING_PLUGIN_SOURCE})

# --- Ujitso sources (optional — requires ujitso SDK headers) ---
set(COOKING_UJITSO_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/LoggingUtils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoContainerCommon.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoCookingComputeService.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoDeformableVolumeMeshInputContainer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoHashUtils.inl
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoMeshCookingContext.inl
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoMeshProcessors.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoMeshUtils.inl
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoProcessContext.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoProcessManager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoResourceManager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoServiceUtils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoSurfaceDeformableBodyInputContainer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoTriangulationContainer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoVolumeDeformableBodyInputContainer.h
)
set(COOKING_UJITSO_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoCookingComputeService.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoDeformableVolumeMeshInputContainer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoMeshCookingProcessor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoMeshTriangulationProcessor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoProcessContext.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoProcessManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoResourceManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoSurfaceDeformableBodyInputContainer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoTriangulationContainer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ujitso/UjitsoVolumeDeformableBodyInputContainer.cpp
)
source_group(plugins\\ujitso FILES ${COOKING_UJITSO_HEADERS} ${COOKING_UJITSO_SOURCE})

# Check if ujitso SDK headers are available
set(_HAS_UJITSO FALSE)

# Ujitso headers come from carb_sdk_plugins or ovruntime_deps.
# Priority order: CARB_SDK_DIR (explicit override) > OVRUNTIME_DEPS_DIR (dep tree).
if(DEFINED CARB_SDK_DIR AND EXISTS "${CARB_SDK_DIR}/include/carb/ujitso")
    set(_HAS_UJITSO TRUE)
elseif(DEFINED OVRUNTIME_DEPS_DIR AND EXISTS "${OVRUNTIME_DEPS_DIR}/include/carb/ujitso")
    set(_HAS_UJITSO TRUE)
endif()

if(_HAS_UJITSO)
    message(STATUS "omni.physx.cooking: Ujitso SDK found — building with ujitso support")
else()
    message(STATUS "omni.physx.cooking: Ujitso SDK not found — building without ujitso support")
endif()

# --- Static library linked by the owning runtime modules ---
add_library(OvruntimePhysXCooking STATIC
    ${COOKING_PLUGIN_HEADERS}
    ${COOKING_PLUGIN_SOURCE}
    $<$<BOOL:${_HAS_UJITSO}>:${COOKING_UJITSO_HEADERS}>
    $<$<BOOL:${_HAS_UJITSO}>:${COOKING_UJITSO_SOURCE}>
)

set_property(TARGET OvruntimePhysXCooking PROPERTY FOLDER "Runtime/PhysXCooking")
set_target_properties(OvruntimePhysXCooking PROPERTIES POSITION_INDEPENDENT_CODE ON)

# RTTI and exceptions enabled (matching old premake)
if(NOT WIN32)
    target_compile_options(OvruntimePhysXCooking PRIVATE -fexceptions -frtti -fPIC)
endif()

if(NOT _HAS_UJITSO)
    target_compile_definitions(OvruntimePhysXCooking PRIVATE OVRUNTIME_NO_UJITSO=1)
endif()

# Include directories
target_include_directories(OvruntimePhysXCooking
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins          # internal headers
        ${OVRUNTIME_ROOT_DIR}/include                 # <omni/physx/IPhysxCookingService.h> etc.
        ${OVRUNTIME_ROOT_DIR}/include/private         # <private/omni/physx/...>
        ${OVRUNTIME_ROOT_DIR}/source/common/include   # <common/utilities/...>
        ${OVRUNTIME_ROOT_DIR}/source/omni.physx.foundation/plugins # foundation static service
)

# External dependencies — SYSTEM includes to suppress third-party warnings
ovruntime_add_external_system_includes(OvruntimePhysXCooking NO_USD)

# Compile definitions
target_compile_definitions(OvruntimePhysXCooking PRIVATE
    "PX_PHYSX_STATIC_LIB"
    "carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)"
)

# Link dependencies
target_link_libraries(OvruntimePhysXCooking
    PRIVATE OvruntimeCommon
    PRIVATE OvruntimeConvexDecomposition
    PRIVATE OvruntimePhysXFoundation
)

if(DEFINED CARB_SDK_DIR)
    set(_CARB_LIB_DIR "${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}")
    target_link_directories(OvruntimePhysXCooking PRIVATE
        ${_CARB_LIB_DIR}/${OVRUNTIME_CARB_CONFIG}
    )
    target_link_libraries(OvruntimePhysXCooking PRIVATE carb)
endif()

# PhysX static libraries
ovruntime_link_physx_static_libs(OvruntimePhysXCooking FULL)

# CI grep gate: verify no USD includes leak into the cooking library's own plugins/ tree
# (production sources only -- tests/ links USD deliberately and is not scanned).
add_custom_target(cooking_usd_free_check ALL
    COMMAND ${CMAKE_COMMAND} -E echo "Checking omni.physx.cooking plugins/ for USD includes..."
    COMMAND ${CMAKE_COMMAND}
        -DCOOKING_PLUGINS_DIR=${CMAKE_CURRENT_SOURCE_DIR}/plugins
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cooking_usd_free_check.cmake
    VERBATIM
)

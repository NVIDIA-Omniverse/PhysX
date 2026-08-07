# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# --- Plugin headers (private) ---
set(PHYSX_PLUGIN_HEADERS
    # root
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ChangeRegister.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/CollisionRepresentation.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ConeCylinderConvexMesh.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ContactReport.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/CookingDataAsync.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/MeshCache.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ObjectDataQuery.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/OmniPhysX.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXCarbCPUDispatcher.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXCooking.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXCustomGeometry.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXCustomJoint.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXDebugVisualization.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXDefines.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXMirror.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXPrivate.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXPropertyQuery.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXReplicator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXScene.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXSceneQuery.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXSettings.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXSimulationCallbacks.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXStageUpdate.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXTools.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXUSDProperties.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXUpdate.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXUnitTestsIface.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Raycast.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/SceneMultiGPUMode.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ScopedNoticeLock.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Setup.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Trigger.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/VehicleGenerator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/VoxelMap.h
    # attachment
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/attachment/PhysXAttachment.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/attachment/PhysXPointFinder.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/attachment/PhysXPoissonSampling.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/attachment/PhysXTetFinder.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/attachment/PhysXTriFinder.h
    # deformables
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/deformables/PhysXDeformablePost.h
    # internal
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/Internal.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalActor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalDebugDraw.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalDeformable.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalDeformableAttachment.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalFilteredPairs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalMimicJoint.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalParticle.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalPhysXDatabase.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalScene.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalTools.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalVehicle.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalVoxelMap.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalXformOpResetStorage.h
    # particles
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/particles/PhysXParticlePost.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/particles/PhysXParticleSampling.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/particles/TypeConversionUtility.cuh
    # propertiesUpdate
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/propertiesUpdate/PhysXPropertiesUpdate.h
    # tensors
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/ArticulationMetatype.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/CommonTypes.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/GlobalsAreBad.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/PhysicsTypes.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/SimulationBackend.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/BaseArticulationView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/BaseDeformableMaterialView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/BaseRigidBodyView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/BaseRigidContactView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/BaseSdfShapeView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/BaseSimulationData.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/BaseSimulationView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/PathPatternMatcher.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/BaseSurfaceDeformableBodyView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/BaseVolumeDeformableBodyView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuArticulationView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuDeformableMaterialView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuRigidBodyView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuRigidContactView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuSdfShapeView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuSimulationData.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuSimulationView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuSurfaceDeformableBodyView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuVolumeDeformableBodyView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/CudaCommon.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/CudaKernels.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuArticulationView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuDeformableMaterialView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuRigidBodyView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuRigidContactView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuSdfShapeView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuSimulationData.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuSimulationView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuSurfaceDeformableBodyView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuVolumeDeformableBodyView.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/ThrustUtils.h
    # usdInterface
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdInterface/UsdInterface.h
    # usdLoad
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Articulation.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/AttachedStage.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/AttributeHelpers.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/ChangeParams.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Collision.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/CollisionGroup.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/DeformableAttachment.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/DeformableBodyConverter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/FilteredPairs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/FixedTendon.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Joint.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/JointInstancer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/LoadStage.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/LoadTools.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/LoadUsd.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Mass.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/MassProperties.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Material.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/MimicJoint.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Particles.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/PhysicsBody.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/PointInstancer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/PrimUpdate.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/ScannedShapeCookingDispatch.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/ReplicatorRange.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Scene.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/SpatialTendon.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Vehicle.h
    # utils
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/utils/OmniRenderBuffer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/utils/Pair.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/utils/Profile.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/utils/SplinesCurve.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/utils/Time.h
)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/plugins PREFIX "plugins" FILES ${PHYSX_PLUGIN_HEADERS})

set(PHYSX_PUBLIC_HEADERS
    ${OVRUNTIME_ROOT_DIR}/include/omni/physx/PhysXRuntime.h
)
source_group(TREE ${OVRUNTIME_ROOT_DIR}/include PREFIX "include" FILES ${PHYSX_PUBLIC_HEADERS})

# --- Plugin sources ---
set(PHYSX_PLUGIN_SOURCE
    # root
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ChangeRegister.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/CollisionRepresentation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ConeCylinderConvexMesh.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/ContactReport.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/CookingDataAsync.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/MeshCache.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/OmniPhysX.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysX.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXCooking.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXCustomGeometry.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXCustomJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXDebugVisualization.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysxJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXMirror.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXPrivate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXPropertyQuery.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/OvxPhysicsRead.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXReplicator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXScene.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXSceneQuery.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXSettings.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXSimulationCallbacks.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXStageUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXStatistics.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXUnitTestsIface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXUSDProperties.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Raycast.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Setup.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/Trigger.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/VehicleGenerator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/VoxelMap.cpp
    # attachment
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/attachment/PhysXAttachment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/attachment/PhysXPointFinder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/attachment/PhysXPoissonSampling.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/attachment/PhysXTetFinder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/attachment/PhysXTriFinder.cpp
    # deformables
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/deformables/PhysXDeformablePost.cpp
    # internal
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/Internal.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalActor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalDeformable.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalDeformableAttachment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalFilteredPairs.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalMimicJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalParticle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalPhysXDatabase.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalScene.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalVehicle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalVoxelMap.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalXformOpResetStorage.cpp
    # particles
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/particles/PhysXParticlePost.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/particles/PhysXParticleSampling.cpp
    # propertiesUpdate
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/propertiesUpdate/PhysXCctPropertiesUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/propertiesUpdate/PhysXCollisionPropertiesUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/propertiesUpdate/PhysXDeformablePropertiesUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/propertiesUpdate/PhysXFilteredPairsUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/propertiesUpdate/PhysXForcePropertiesUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/propertiesUpdate/PhysXJointsPropertiesUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/propertiesUpdate/PhysXMaterialPropertiesUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/propertiesUpdate/PhysXMimicJointPropertiesUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/propertiesUpdate/PhysXParticlePropertiesUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/propertiesUpdate/PhysXPointInstancerPropertiesUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/propertiesUpdate/PhysXRigidBodyPropertiesUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/propertiesUpdate/PhysXScenePropertiesUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/propertiesUpdate/PhysXTendonPropertiesUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/propertiesUpdate/PhysXVehiclePropertiesUpdate.cpp
    # tensors
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/ArticulationMetatype.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/PhysxTensors.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/SimulationBackend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/BaseArticulationView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/BaseDeformableMaterialView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/BaseRigidBodyView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/BaseRigidContactView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/BaseSdfShapeView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/BaseSimulationView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/PathPatternMatcher.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/BaseSurfaceDeformableBodyView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/base/BaseVolumeDeformableBodyView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuArticulationView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuDeformableMaterialView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuRigidBodyView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuRigidContactView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuSdfShapeView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuSimulationData.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuSimulationView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuSurfaceDeformableBodyView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/cpu/CpuVolumeDeformableBodyView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/CudaCommon.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuArticulationView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuDeformableMaterialView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuRigidBodyView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuRigidContactView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuSdfShapeView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuSimulationData.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuSimulationView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuSurfaceDeformableBodyView.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/GpuVolumeDeformableBodyView.cpp
    # usdInterface
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdInterface/UsdInterface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdInterface/UsdInterfaceDeformable.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdInterface/UsdInterfaceDeformableAttachment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdInterface/UsdInterfaceParticle.cpp
    # usdLoad
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Articulation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/AttachedStage.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Collision.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/CollisionGroup.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/DeformableAttachment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/DeformableBodyConverter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/FilteredPairs.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/FixedTendon.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Joint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/JointInstancer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/LoadStage.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/LoadTools.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/IceDescriptorAllocator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/LoadUsd.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Mass.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Material.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/MimicJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Particles.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/PhysicsBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/PointInstancer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/PrimUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/ScannedShapeCookingDispatch.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Scene.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/TimeSampledCallbacks.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/SpatialTendon.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Vehicle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/NewtonTokens.cpp
    # utils
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/utils/SplinesCurve.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/utils/Time.cpp
)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/plugins PREFIX "plugins" FILES ${PHYSX_PLUGIN_SOURCE})

# --- Tensor API CUDA sources ---
set(TENSORS_CUDA_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/tensors/gpu/CudaKernels.cu
)
source_group(plugins\\tensors\\cuda FILES ${TENSORS_CUDA_SOURCE})

# --- Generate Version.h (once, then cached until clean build) ---
set(_VERSION_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/include/omni/physx")
file(MAKE_DIRECTORY "${_VERSION_DIR}")

if(NOT EXISTS "${_VERSION_DIR}/Version.h")
    execute_process(COMMAND git rev-parse --short HEAD
        WORKING_DIRECTORY "${OVRUNTIME_ROOT_DIR}"
        OUTPUT_VARIABLE _GIT_SHA OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    execute_process(COMMAND git show -s --format=%ad --date=format:%b-%d-%Y
        WORKING_DIRECTORY "${OVRUNTIME_ROOT_DIR}"
        OUTPUT_VARIABLE _GIT_DATE OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    execute_process(COMMAND git rev-parse --abbrev-ref HEAD
        WORKING_DIRECTORY "${OVRUNTIME_ROOT_DIR}"
        OUTPUT_VARIABLE _GIT_BRANCH OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    if(NOT _GIT_SHA)
        set(_GIT_SHA "unknown")
    endif()
    if(NOT _GIT_DATE)
        set(_GIT_DATE "unknown")
    endif()
    if(NOT _GIT_BRANCH)
        set(_GIT_BRANCH "unknown")
    endif()
    # Read VERSION from ovruntime
    if(EXISTS "${OVRUNTIME_ROOT_DIR}/VERSION")
        file(READ "${OVRUNTIME_ROOT_DIR}/VERSION" _BUILD_VERSION)
        string(STRIP "${_BUILD_VERSION}" _BUILD_VERSION)
    else()
        set(_BUILD_VERSION "0.0.0")
    endif()
    file(WRITE "${_VERSION_DIR}/Version.h"
        "#pragma once\n"
        "#define PHYSICS_BUILD_SHA \"${_GIT_SHA}\"\n"
        "#define PHYSICS_BUILD_DATE \"${_GIT_DATE}\"\n"
        "#define PHYSICS_BUILD_BRANCH \"${_GIT_BRANCH}\"\n"
        "#define PHYSICS_BUILD_VERSION \"${_BUILD_VERSION}\"\n"
    )
    message(STATUS "Generated Version.h (${_GIT_SHA}, ${_GIT_BRANCH}, ${_BUILD_VERSION})")
else()
    message(STATUS "Version.h already exists, skipping generation (clean build to regenerate)")
endif()

# --- Core PhysX runtime static library ---
add_library(OvruntimePhysX STATIC
    ${PHYSX_PUBLIC_HEADERS}
    ${PHYSX_PLUGIN_HEADERS}
    ${PHYSX_PLUGIN_SOURCE}
)
ovruntime_register_gcov_target(OvruntimePhysX)

set_property(TARGET OvruntimePhysX PROPERTY FOLDER "Runtime/PhysX")

set_target_properties(OvruntimePhysX PROPERTIES POSITION_INDEPENDENT_CODE ON)

# RTTI and exceptions enabled, AVX instruction set (matching old premake)
if(NOT WIN32)
    target_compile_options(OvruntimePhysX PRIVATE
        -fexceptions -frtti -fPIC
        $<$<STREQUAL:${CMAKE_SYSTEM_PROCESSOR},x86_64>:-mavx>
        -Wno-invalid-offsetof
    )
else()
    target_compile_options(OvruntimePhysX PRIVATE /arch:AVX)
endif()

# Include directories
target_include_directories(OvruntimePhysX
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins          # internal headers
        ${OVRUNTIME_ROOT_DIR}/include                 # public interfaces
        ${OVRUNTIME_ROOT_DIR}/include/private         # private interfaces
        ${OVRUNTIME_ROOT_DIR}/source/pch              # UsdPCH.h
        ${OVRUNTIME_ROOT_DIR}/source/common/include   # <common/utilities/...>
        ${OVRUNTIME_ROOT_DIR}/source/omni.physx.foundation/plugins # foundation static service
        ${OVRUNTIME_ROOT_DIR}/source/omni.physx.cooking/plugins # cooking static service
        ${CMAKE_CURRENT_BINARY_DIR}/generated/include # Version.h
        ${OVRUNTIME_ROOT_DIR}/_build/generated       # NewtonSchemaTokens.h
)

# External dependencies — SYSTEM includes to suppress third-party warnings
ovruntime_add_external_system_includes(OvruntimePhysX)

# Compile definitions
# carb_eventdispatcher_IEventDispatcher must be CXX-only: it expands to
# CARB_HEXVERSION(1, 4), which is not a valid identifier on an nvcc -D flag
# ("macro names must be identifiers"). Now that this target compiles a CUDA
# source (tensors/gpu/CudaKernels.cu), guard it for C++ compilation only.
target_compile_definitions(OvruntimePhysX PRIVATE
    "PX_PHYSX_STATIC_LIB"
    "$<$<COMPILE_LANGUAGE:CXX>:carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)>"
)
# OvruntimePhysX is a static archive. Its consumers must reference the standalone
# ovstage output-read entry points directly rather than through MSVC import thunks.
target_compile_definitions(OvruntimePhysX PUBLIC "OMNI_PHYSX_OVX_STATIC")

# Link dependencies
target_link_libraries(OvruntimePhysX
    PRIVATE OvruntimeCommon
    PRIVATE OvruntimePhysicsUsd
    PRIVATE OvruntimePhysicsOvstage
    PRIVATE OvruntimePhysXFoundation
    PRIVATE OvruntimePhysXCooking
)
# omni.physx selects USD vs ovstage parsing at runtime through the abstract
# registries (parse backend + omni.physics.usd scan backend, ADR-0002 M2c). The
# plugin links the ovstage backend so its `attachOvstage` entry point can install
# those backends itself (ADR-0002 Option 1): the ovstage parsing always exists on
# this branch, and the per-module registry can only be populated from inside the
# plugin (the test/consumer lives in a separate module). The dependency is one-way
# (omni.physx -> ovstage -> usd -> parse); nothing here names a concrete USD type
# in the ovstage path — selection stays through the registries, no #ifdefs.

if(DEFINED CARB_SDK_DIR)
    set(_CARB_LIB_DIR "${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}")
    target_link_directories(OvruntimePhysX PRIVATE
        ${_CARB_LIB_DIR}/${OVRUNTIME_CARB_CONFIG}
    )
    target_link_libraries(OvruntimePhysX PRIVATE carb)
endif()

# USD, TBB, Python libraries (matching premake extension_usd_deps)
ovruntime_link_usd_deps(OvruntimePhysX)

# PhysX static libraries
ovruntime_link_physx_static_libs(OvruntimePhysX FULL)

# CUDA libraries
if(NOT WIN32)
    # CUDA: don't link libcuda directly on Linux — loaded at runtime via dlopen.
    # Only add lib64 to RUNPATH (for cudart etc.).  The stubs/ directory is
    # passed as a bare -L flag so the linker can resolve symbols at build
    # time, but CMake does NOT add it to RUNPATH.  This prevents the stub
    # libcuda.so (SONAME libcuda.so.1) from shadowing the real CUDA driver
    # at runtime (CUDA_ERROR_STUB_LIBRARY / error 34).
    if(DEFINED CUDA_DIR)
        target_link_directories(OvruntimePhysX PRIVATE
            ${CUDA_DIR}/lib64
        )
        target_link_options(OvruntimePhysX PRIVATE
            -Wl,--allow-shlib-undefined
            -L${CUDA_DIR}/lib64/stubs
        )
    endif()
else()
    if(DEFINED CUDA_DIR)
        target_link_directories(OvruntimePhysX PRIVATE ${CUDA_DIR}/lib/x64)
        target_link_libraries(OvruntimePhysX PRIVATE cuda delayimp)
        target_link_options(OvruntimePhysX PRIVATE /DELAYLOAD:nvcuda.dll)
    endif()
endif()

# --- CUDA for the tensor plugin ---
if(DEFINED CUDA_DIR)
    if(WIN32)
        target_link_libraries(OvruntimePhysX PRIVATE cudart_static)
    else()
        target_link_libraries(OvruntimePhysX PRIVATE cuda cudart_static)
    endif()
endif()

# CUDA sources — platform-adaptive (native CUDA language on Linux / nvcc on Windows)
ovruntime_target_add_cuda_sources(OvruntimePhysX
    SOURCES     ${TENSORS_CUDA_SOURCE}
    INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/plugins"
)
if(NOT WIN32)
    set_target_properties(OvruntimePhysX PROPERTIES CUDA_RESOLVE_DEVICE_SYMBOLS ON)
endif()

# Precompiled header
target_precompile_headers(OvruntimePhysX PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${OVRUNTIME_ROOT_DIR}/source/pch/UsdPCH.h>"
)

# PhysXGpu, PVDRuntime, and (Windows-only) PhysXDevice64 are loaded by the
# foundation-owned PhysXGpuLoader at runtime. Keep build-tree copies in the
# runtime output directory used by the owner binary.
if(WIN32)
    set(_PHYSXGPU_LIB_NAME "PhysXGpu_64.dll")
    set(_PVDRUNTIME_LIB_NAME "PVDRuntime_64.dll")
else()
    set(_PHYSXGPU_LIB_NAME "libPhysXGpu_64.so")
    set(_PVDRUNTIME_LIB_NAME "libPVDRuntime_64.so")
endif()

set(_GPU_BIN_DIR "bin")
set(_PHYSX_BIN_DIR "bin")

if(OVRUNTIME_PHYSX_FROM_SOURCE)
    # EXCLUDE_FROM_ALL on the PhysX subdirectory means these targets won't build
    # automatically; depend on them when PhysX needs to copy their runtime binaries.
    if(TARGET PhysXGpu)
        add_dependencies(OvruntimePhysX PhysXGpu)
        add_custom_command(TARGET OvruntimePhysX POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<TARGET_FILE:PhysXGpu>"
                "$<TARGET_FILE_DIR:OvruntimePhysX>/${_PHYSXGPU_LIB_NAME}"
            COMMENT "Copying ${_PHYSXGPU_LIB_NAME} next to the PhysX runtime owner"
        )
        install(FILES "$<TARGET_FILE:PhysXGpu>" DESTINATION ${_GPU_BIN_DIR} RENAME "${_PHYSXGPU_LIB_NAME}")
        install(FILES "$<TARGET_FILE:PhysXGpu>" DESTINATION ${_PHYSX_BIN_DIR} RENAME "${_PHYSXGPU_LIB_NAME}")
    endif()
    if(TARGET PVDRuntime)
        add_dependencies(OvruntimePhysX PVDRuntime)
        add_custom_command(TARGET OvruntimePhysX POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<TARGET_FILE:PVDRuntime>"
                "$<TARGET_FILE_DIR:OvruntimePhysX>/${_PVDRUNTIME_LIB_NAME}"
            COMMENT "Copying ${_PVDRUNTIME_LIB_NAME} next to the PhysX runtime owner"
        )
        install(FILES "$<TARGET_FILE:PVDRuntime>" DESTINATION ${_PHYSX_BIN_DIR} RENAME "${_PVDRUNTIME_LIB_NAME}")
    endif()
elseif(DEFINED PHYSX_SDK_DIR)
    set(_PHYSXGPU_LIB_DIR "${PHYSX_SDK_DIR}/bin/${PHYSX_PLATFORM_BIN}")
    set(_PHYSXGPU_LIB "${_PHYSXGPU_LIB_DIR}/$<IF:$<CONFIG:Debug>,debug,checked>/${_PHYSXGPU_LIB_NAME}")
    set(_PVDRUNTIME_LIB "${_PHYSXGPU_LIB_DIR}/$<IF:$<CONFIG:Debug>,debug,checked>/${_PVDRUNTIME_LIB_NAME}")

    add_custom_command(TARGET OvruntimePhysX POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_PHYSXGPU_LIB}"
            "$<TARGET_FILE_DIR:OvruntimePhysX>/${_PHYSXGPU_LIB_NAME}"
        COMMENT "Copying ${_PHYSXGPU_LIB_NAME} next to the PhysX runtime owner"
    )
    install(FILES "${_PHYSXGPU_LIB}" DESTINATION ${_GPU_BIN_DIR})
    install(FILES "${_PHYSXGPU_LIB}" DESTINATION ${_PHYSX_BIN_DIR})

    add_custom_command(TARGET OvruntimePhysX POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_PVDRUNTIME_LIB}"
            "$<TARGET_FILE_DIR:OvruntimePhysX>/${_PVDRUNTIME_LIB_NAME}"
        COMMENT "Copying ${_PVDRUNTIME_LIB_NAME} next to the PhysX runtime owner"
    )
    install(FILES "${_PVDRUNTIME_LIB}" DESTINATION ${_PHYSX_BIN_DIR})
endif()

if(WIN32)
    set(_PHYSX_DEVICE_DLL "${OVRUNTIME_TARGET_DEPS}/physxdevice/bin/x86/PhysXDevice64.dll")
    if(EXISTS "${_PHYSX_DEVICE_DLL}")
        add_custom_command(TARGET OvruntimePhysX POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_PHYSX_DEVICE_DLL}"
                "$<TARGET_FILE_DIR:OvruntimePhysX>/PhysXDevice64.dll"
            COMMENT "Copying PhysXDevice64.dll next to the PhysX runtime owner"
        )
        install(FILES "${_PHYSX_DEVICE_DLL}" DESTINATION ${_GPU_BIN_DIR})
    endif()
endif()

# Install rules
install(
    TARGETS OvruntimePhysX
    EXPORT OvruntimePhysX
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION bin
    ARCHIVE DESTINATION lib
)

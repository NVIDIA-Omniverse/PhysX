# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

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
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXStats.h
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
    # usdInterface
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdInterface/UsdInterface.h
    # usdLoad
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Articulation.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/AttachedStage.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/AttributeHelpers.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Cct.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/ChangeParams.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Collision.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/CollisionGroup.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/DeformableAttachment.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/FabricSync.h
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
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXReplicator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXScene.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXSceneQuery.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXSettings.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXSimulationCallbacks.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXStageUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/PhysXStats.cpp
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
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/internal/InternalTools.cpp
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
    # usdInterface
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdInterface/UsdInterface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdInterface/UsdInterfaceDeformable.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdInterface/UsdInterfaceDeformableAttachment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdInterface/UsdInterfaceParticle.cpp
    # usdLoad
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Articulation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/AttachedStage.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Cct.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Collision.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/CollisionGroup.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/DeformableAttachment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/FabricSync.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/FilteredPairs.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/FixedTendon.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Joint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/JointInstancer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/LoadStage.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/LoadTools.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/LoadUsd.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Mass.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Material.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/MimicJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Particles.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/PhysicsBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/PointInstancer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/PrimUpdate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Scene.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/SpatialTendon.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/Vehicle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/usdLoad/NewtonTokens.cpp
    # utils
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/utils/SplinesCurve.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/utils/Time.cpp
)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/plugins PREFIX "plugins" FILES ${PHYSX_PLUGIN_SOURCE})

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
    # Read VERSION from ovexts (same source as old premake)
    if(EXISTS "${OVRUNTIME_ROOT_DIR}/../ovexts/VERSION")
        file(READ "${OVRUNTIME_ROOT_DIR}/../ovexts/VERSION" _BUILD_VERSION)
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

# --- Create shared library (Carbonite plugin) ---
add_library(OvruntimePhysX SHARED
    ${PHYSX_PLUGIN_HEADERS}
    ${PHYSX_PLUGIN_SOURCE}
)

set_property(TARGET OvruntimePhysX PROPERTY FOLDER "Runtime/PhysX")

set_target_properties(OvruntimePhysX PROPERTIES
    OUTPUT_NAME "omni.physx.plugin"
)

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
        ${CMAKE_CURRENT_BINARY_DIR}/generated/include # Version.h
        ${OVRUNTIME_ROOT_DIR}/_build/generated       # NewtonSchemaTokens.h
)

# External dependencies — SYSTEM includes to suppress third-party warnings
ovruntime_add_external_system_includes(OvruntimePhysX)

# Compile definitions
target_compile_definitions(OvruntimePhysX PRIVATE
    "PX_PHYSX_STATIC_LIB"
    "carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)"
)

# Link dependencies
target_link_libraries(OvruntimePhysX
    PRIVATE OvruntimeCommon
)

if(DEFINED CARB_SDK_DIR)
    set(_CARB_LIB_DIR "${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}")
    target_link_directories(OvruntimePhysX PRIVATE
        ${_CARB_LIB_DIR}/${OVRUNTIME_CARB_CONFIG}
    )
    target_link_libraries(OvruntimePhysX PRIVATE carb)
endif()

# physicsSchemaTools + physxSchema libraries
if(EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
    target_link_directories(OvruntimePhysX PRIVATE
        ${USD_EXT_PHYSICS_DIR}/lib
    )
    target_link_libraries(OvruntimePhysX PRIVATE physicsSchemaTools physxSchema)
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

# Precompiled header
target_precompile_headers(OvruntimePhysX PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${OVRUNTIME_ROOT_DIR}/source/pch/UsdPCH.h>"
)

# --- Python bindings (optional) ---
if(OVRUNTIME_ENABLE_PYTHON_BINDINGS)
    set(PHYSX_BINDINGS_SOURCE
        ${CMAKE_CURRENT_SOURCE_DIR}/bindings/BindingsImpl.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/bindings/BindingsPhysXCooking.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/bindings/BindingsPhysXObjectChangedCallback.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/bindings/BindingsPhysXPython.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/bindings/BindingsPhysXReplicator.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/bindings/BindingsPhysXStageUpdate.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/bindings/BindingsPhysXStatistics.cpp
    )
    set(PHYSX_BINDINGS_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/bindings/BindingsImpl.h
        ${CMAKE_CURRENT_SOURCE_DIR}/bindings/ResultBuffer.h
    )
    source_group(bindings FILES ${PHYSX_BINDINGS_SOURCE} ${PHYSX_BINDINGS_HEADERS})

    add_library(OvruntimePhysXBindings SHARED
        ${PHYSX_BINDINGS_SOURCE}
        ${PHYSX_BINDINGS_HEADERS}
    )

    set_property(TARGET OvruntimePhysXBindings PROPERTY FOLDER "Runtime/PhysX")

    set_target_properties(OvruntimePhysXBindings PROPERTIES
        OUTPUT_NAME "_physx"
        PREFIX ""
    )
    if(WIN32)
        set_target_properties(OvruntimePhysXBindings PROPERTIES SUFFIX ".pyd")
    else()
        set_target_properties(OvruntimePhysXBindings PROPERTIES SUFFIX ".so")
    endif()

    target_include_directories(OvruntimePhysXBindings PRIVATE
        ${OVRUNTIME_ROOT_DIR}/include
        ${OVRUNTIME_ROOT_DIR}/include/private
        ${OVRUNTIME_ROOT_DIR}/source/pch
        ${OVRUNTIME_ROOT_DIR}/source/common/include
    )

    # External dependencies — SYSTEM includes to suppress third-party warnings
    ovruntime_add_external_system_includes(OvruntimePhysXBindings)

    # physicsSchemaTools + physxSchema (bindings link these too)
    if(EXISTS "${USD_EXT_PHYSICS_DIR}/lib")
        target_link_directories(OvruntimePhysXBindings PRIVATE
            ${USD_EXT_PHYSICS_DIR}/lib
        )
        target_link_libraries(OvruntimePhysXBindings PRIVATE physicsSchemaTools physxSchema)
    endif()

    target_compile_definitions(OvruntimePhysXBindings PRIVATE
        "carb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1, 4)"
    )

    if(NOT WIN32)
        target_compile_options(OvruntimePhysXBindings PRIVATE -fexceptions -frtti -fPIC)
    endif()

    if(DEFINED CARB_SDK_DIR)
        target_link_directories(OvruntimePhysXBindings PRIVATE
            ${_CARB_LIB_DIR}/${OVRUNTIME_CARB_CONFIG}
        )
        target_link_libraries(OvruntimePhysXBindings PRIVATE carb)
    endif()

    # Python + USD boost::python libraries
    ovruntime_link_bindings_deps(OvruntimePhysXBindings)

    # Precompiled header
    target_precompile_headers(OvruntimePhysXBindings PRIVATE
        "$<$<COMPILE_LANGUAGE:CXX>:${OVRUNTIME_ROOT_DIR}/source/pch/UsdPCH.h>"
    )

    # Install bindings
    install(
        TARGETS OvruntimePhysXBindings
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION bin
    )
endif()

# Install rules
install(
    TARGETS OvruntimePhysX
    EXPORT OvruntimePhysX
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION bin
    ARCHIVE DESTINATION lib
)

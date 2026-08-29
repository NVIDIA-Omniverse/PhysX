# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# omni.physics.parse — USD-free core parsing library (static)
#
# CI invariant: no USD/pxr includes allowed in public headers.
# grep -r 'pxr/\|PXR_NS::' ${OVRUNTIME_ROOT_DIR}/include/omni/physics/parse/ must be empty.

set(PARSE_INCLUDE_DIR "${OVRUNTIME_ROOT_DIR}/include")

set(PARSE_HEADERS
    ${PARSE_INCLUDE_DIR}/omni/physics/parse/Handles.h
    ${PARSE_INCLUDE_DIR}/omni/physics/parse/Math.h
    ${PARSE_INCLUDE_DIR}/omni/physics/parse/Descriptors.h
    ${PARSE_INCLUDE_DIR}/omni/physics/parse/IPhysicsSource.h
    ${PARSE_INCLUDE_DIR}/omni/physics/parse/IParseBackend.h
    ${PARSE_INCLUDE_DIR}/omni/physics/parse/ParseContext.h
    ${PARSE_INCLUDE_DIR}/omni/physics/parse/ParseApi.h
    ${PARSE_INCLUDE_DIR}/omni/physics/parse/KnownTokens.h
    ${PARSE_INCLUDE_DIR}/omni/physics/parse/Allocator.h
    ${PARSE_INCLUDE_DIR}/omni/physics/parse/ScannedStage.h
    ${PARSE_INCLUDE_DIR}/omni/physics/parse/ScanBackend.h
    ${PARSE_INCLUDE_DIR}/omni/physics/parse/ArticulationGraph.h
)
source_group(include FILES ${PARSE_HEADERS})

set(PARSE_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/KnownTokens.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseContext.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseMaterial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseMass.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseArticulation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseCollision.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseScene.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseShape.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseFilteredPairs.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseCollisionGroup.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseAttachment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseDeformableBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseParticleSystem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseCct.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseMimicJoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseSpatialTendon.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseFixedTendon.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseTireFrictionTable.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseVehicleContext.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseVehicleWheel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseVehicleTire.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseVehicleSuspension.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseVehicleEngine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseVehicleGears.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseVehicleClutch.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseVehicleDriveBasic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseVehicleDifferential.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseVehicleAutoGearBox.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseVehicleBrakes.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseVehicleSteering.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseVehicleNonlinearCmdResponse.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseVehicleDriveStandard.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseVehicleSuspensionCompliance.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseVehicleWheelAttachment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseVehicle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ProceduralSource.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParseBackend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ScannedStage.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ScanBackend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ArticulationGraph.cpp
)
source_group(source FILES ${PARSE_SOURCE})

add_library(OvruntimePhysicsParse STATIC
    ${PARSE_HEADERS}
    ${PARSE_SOURCE}
)

set_property(TARGET OvruntimePhysicsParse PROPERTY FOLDER "Runtime/Parse")

set_target_properties(OvruntimePhysicsParse PROPERTIES
    OUTPUT_NAME "OvruntimePhysicsParse"
)

target_include_directories(OvruntimePhysicsParse
    PUBLIC
        ${PARSE_INCLUDE_DIR}    # <omni/physics/parse/...>
)

# External dependencies — SYSTEM includes to suppress third-party warnings.
# Core parse headers use carb types (Float3, etc.) and PhysX-runtime types
# (ObjectId, MeshKey, CookingParams) but never include pxr/ USD headers.
ovruntime_add_external_system_includes(OvruntimePhysicsParse)

if(NOT WIN32)
    target_compile_options(OvruntimePhysicsParse PRIVATE -fexceptions -fPIC)
endif()

# Install public headers
install(FILES ${PARSE_HEADERS}
    DESTINATION include/omni/physics/parse
)

install(
    TARGETS OvruntimePhysicsParse
    EXPORT OvruntimePhysicsParse
    ARCHIVE DESTINATION lib
)

# CI grep gate: verify no USD includes leak into core parse headers.
# The `omni/physics/usd/` subdir is the USD backend's public surface
# (StandaloneParser.h etc.) — it MAY reference USD types and is excluded.
add_custom_target(parse_usd_free_check ALL
    COMMAND ${CMAKE_COMMAND} -E echo "Checking omni.physics.parse headers for USD includes..."
    COMMAND ${CMAKE_COMMAND}
        -DPARSE_INCLUDE_DIR=${PARSE_INCLUDE_DIR}/omni/physics/parse
        -P ${CMAKE_CURRENT_SOURCE_DIR}/parse_usd_free_check.cmake
    VERBATIM
)

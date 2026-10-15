// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-BATCH-001
 * @covers AC-4
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-4
 */
#pragma once
#include "OvstageWalker.h"
#include "OvstageDeformableBatch.h"
#include <omni/physics/parse/PreparedScene.h>
#include <omni/physics/parse/PreparedMaterial.h>
#include <omni/physics/parse/PreparedParticle.h>
#include <omni/physics/parse/PreparedJoint.h>
#include <omni/physics/parse/PreparedVehicle.h>
#include <omni/physics/parse/PreparedTendon.h>
#include <omni/physics/parse/PreparedShape.h>
#include <cstdint>
namespace omni::physics::ovstage
{
// Coordinator-only capacity accounting, excluding allocator headers and node
// containers. Source caches and borrowed mesh buffers are measured separately
// by process RSS. New owning fields must be added alongside their fill kernel.
namespace descriptor_memory
{
template<class T> size_t nested(const T&) { return 0; }
inline size_t nested(const CollisionGroupDesc& value);
inline size_t nested(const PhysxShapeDesc& value);
inline size_t nested(const MergeMeshDesc& value);
inline size_t nested(const SpherePointsPhysxShapeDesc& value);
inline size_t nested(const PhysxRigidBodyDesc& value);
inline size_t nested(const D6PhysxJointDesc& value);
inline size_t nested(const SphericalPhysxJointDesc& value);
inline size_t nested(const PhysxArticulationDesc& value);
inline size_t nested(const PhysxTendonAxisDesc& value);
inline size_t nested(const PhysxDeformableBodyDesc& value);
inline size_t nested(const TireFrictionTableDesc& value);
inline size_t nested(const SuspensionComplianceDesc& value);
inline size_t nested(const GearsDesc& value);
inline size_t nested(const AutoGearBoxDesc& value);
inline size_t nested(const NonlinearCmdResponseDesc& value);
inline size_t nested(const MultiWheelDifferentialDesc& value);
inline size_t nested(const TankDifferentialDesc& value);
inline size_t nested(const BrakesDesc& value);
inline size_t nested(const SteeringBasicDesc& value);
inline size_t nested(const VehicleDesc& value);
inline size_t nested(const ParticleSystemDesc& value);
inline size_t nested(const ParticleDesc& value);
inline size_t nested(const ParticleSetDesc& value);
inline size_t nested(const ParticleIsosurfaceDesc& value);
inline size_t nested(const SceneInfo& value);
inline size_t nested(const JointInfo& value);
inline size_t nested(const ShapeInfo& value);
inline size_t nested(const CollisionGroupInfo& value);
inline size_t nested(const DeformableBodyParse& value);
inline size_t nested(const CctInfo& value);
inline size_t nested(const TireFrictionTableInfo& value);
inline size_t nested(const GearsInfo& value);
inline size_t nested(const AutoGearBoxInfo& value);
inline size_t nested(const DifferentialInfo& value);
inline size_t nested(const TankDifferentialInfo& value);
inline size_t nested(const BrakesInfo& value);
inline size_t nested(const SteeringBasicInfo& value);
inline size_t nested(const NonlinearCmdResponseInfo& value);
inline size_t nested(const SuspensionComplianceInfo& value);
inline size_t nested(const PreparedSuspension& value);
inline size_t nested(const PreparedSteeringAckermann& value);
inline size_t nested(const PreparedSteeringBasic& value);
inline size_t nested(const PreparedWheelAttachment& value);
inline size_t nested(const PreparedAutoGearBox& value);
inline size_t nested(const PreparedTire& value);
inline size_t nested(const PreparedBrakes& value);
inline size_t nested(const PreparedVehicleContext& value);
inline size_t nested(const PreparedNonlinearCmdResponse& value);
inline size_t nested(const PreparedWheel& value);
inline size_t nested(const PreparedClutch& value);
inline size_t nested(const PreparedGears& value);
inline size_t nested(const PreparedEngine& value);
inline size_t nested(const PreparedDriveBasic& value);
inline size_t nested(const PreparedSuspensionCompliance& value);
inline size_t nested(const PreparedTankDifferential& value);
inline size_t nested(const PreparedMultiWheelDifferential& value);
inline size_t nested(const PreparedDriveStandard& value);
inline size_t nested(const PreparedVehicle& value);
inline size_t nested(const PreparedTireFrictionTable& value);
inline size_t nested(const VehicleAttributeInput& value);
inline size_t nested(const VehiclePathInput& value);
inline size_t nested(const PreparedVehicleReads& value);
inline size_t nested(const PreparedFixedTendon& value);
inline size_t nested(const PreparedParticleSystem& value);
inline size_t nested(const PreparedParticleIsosurface& value);
inline size_t nested(const PreparedParticleSet& value);
inline size_t nested(const PreparedScene& value);
inline size_t nested(const PreparedShapeCommon& value);
inline size_t nested(const PreparedJoint& value);
inline size_t nested(const PreparedOvstageDeformable& value);
inline size_t nested(const std::string& value)
{
    const uintptr_t data = reinterpret_cast<uintptr_t>(value.data());
    const uintptr_t object = reinterpret_cast<uintptr_t>(&value);
    return data >= object && data < object + sizeof(value) ? 0 : value.capacity() + 1;
}
template<class T> size_t nested(const std::vector<T>& values)
{
    size_t bytes = values.capacity() * sizeof(T);
    for (const T& value : values) bytes += nested(value);
    return bytes;
}
inline size_t nested(const CollisionGroupDesc& value)
{
    return nested(value.sourceFilteredGroups) + nested(value.sourceMembers) + nested(value.filteredGroups);
}
inline size_t nested(const PhysxShapeDesc& value)
{
    return nested(value.materials) + nested(value.sceneIds) + nested(value.sourceMaterials) + nested(value.sourceSimulationOwners) + nested(value.sourceFilteredCollisions);
}
inline size_t nested(const MergeMeshDesc& value)
{
    return nested(value.points) + nested(value.indices) + nested(value.faces) + nested(value.holes);
}
inline size_t nested(const SpherePointsPhysxShapeDesc& value)
{
    return nested(static_cast<const PhysxShapeDesc&>(value)) + nested(value.spheres);
}
inline size_t nested(const PhysxRigidBodyDesc& value)
{
    return nested(value.shapes) + nested(value.collisionBlocks) + nested(value.sceneIds) + nested(value.sourceSimulationOwners) + nested(value.sourceFilteredCollisions);
}
inline size_t nested(const D6PhysxJointDesc& value)
{
    return nested(value.jointLimits) + nested(value.jointDrives) + nested(value.jointStates) + nested(value.jointProperties);
}
inline size_t nested(const SphericalPhysxJointDesc& value)
{
    return nested(value.jointProperties);
}
inline size_t nested(const PhysxArticulationDesc& value)
{
    return nested(value.sourceFilteredCollisions);
}
inline size_t nested(const PhysxTendonAxisDesc& value)
{
    return nested(value.gearings) + nested(value.forceCoefficients) + nested(value.axes);
}
inline size_t nested(const PhysxDeformableBodyDesc& value)
{
    return nested(value.skinGeomPaths) + nested(value.skinGeomBindPoseTokens) + nested(value.sourceSimulationOwners) + nested(value.sourceFilteredCollisions);
}
inline size_t nested(const TireFrictionTableDesc& value)
{
    return nested(value.materialPaths) + nested(value.materialIds) + nested(value.frictionValues);
}
inline size_t nested(const SuspensionComplianceDesc& value)
{
    return nested(value.wheelToeAngleList) + nested(value.wheelCamberAngleList) + nested(value.suspensionForceAppPointList) + nested(value.tireForceAppPointList);
}
inline size_t nested(const GearsDesc& value)
{
    return nested(value.ratios);
}
inline size_t nested(const AutoGearBoxDesc& value)
{
    return nested(value.upRatios) + nested(value.downRatios);
}
inline size_t nested(const NonlinearCmdResponseDesc& value)
{
    return nested(value.commandValues) + nested(value.speedResponsesPerCommandValue) + nested(value.speedResponses);
}
inline size_t nested(const MultiWheelDifferentialDesc& value)
{
    return nested(value.wheels) + nested(value.torqueRatios) + nested(value.averageWheelSpeedRatios);
}
inline size_t nested(const TankDifferentialDesc& value)
{
    return nested(static_cast<const MultiWheelDifferentialDesc&>(value)) + nested(value.numberOfWheelsPerTrack) + nested(value.thrustIndexPerTrack) + nested(value.trackToWheelIndices) + nested(value.wheelIndicesInTrackOrder);
}
inline size_t nested(const BrakesDesc& value)
{
    return nested(value.wheels) + nested(value.torqueMultipliers);
}
inline size_t nested(const SteeringBasicDesc& value)
{
    return nested(value.wheels) + nested(value.angleMultipliers);
}
inline size_t nested(const VehicleDesc& value)
{
    return nested(value.wheelAttachments) + nested(value.wheelControllers) + nested(value.brakes);
}
inline size_t nested(const ParticleSystemDesc& value)
{
    return nested(value.filteredCollisions);
}
inline size_t nested(const ParticleDesc& value)
{
    return nested(value.points) + nested(value.velocities);
}
inline size_t nested(const ParticleSetDesc& value)
{
    return nested(static_cast<const ParticleDesc&>(value)) + nested(value.simulationPoints);
}
inline size_t nested(const ParticleIsosurfaceDesc& value)
{
    return nested(value.gridFilteringPasses);
}
inline size_t nested(const SceneInfo& value)
{
    return nested(value.quasistaticActors);
}
inline size_t nested(const JointInfo& value)
{
    return nested(value.jointLimits) + nested(value.jointDrives) + nested(value.limit) + nested(value.drive);
}
inline size_t nested(const ShapeInfo& value)
{
    return nested(value.simulationOwners);
}
inline size_t nested(const CollisionGroupInfo& value)
{
    return nested(value.filteredGroups) + nested(value.members);
}
inline size_t nested(const DeformableBodyParse& value)
{
    return nested(value.simulationOwners) + nested(value.filteredCollisions);
}
inline size_t nested(const CctInfo& value)
{
    return nested(value.simulationOwners);
}
inline size_t nested(const TireFrictionTableInfo& value)
{
    return nested(value.materialPaths) + nested(value.frictionValues);
}
inline size_t nested(const GearsInfo& value)
{
    return nested(value.ratios);
}
inline size_t nested(const AutoGearBoxInfo& value)
{
    return nested(value.upRatios) + nested(value.downRatios);
}
inline size_t nested(const DifferentialInfo& value)
{
    return nested(value.wheels) + nested(value.torqueRatios) + nested(value.averageWheelSpeedRatios);
}
inline size_t nested(const TankDifferentialInfo& value)
{
    return nested(value.numberOfWheelsPerTrack) + nested(value.thrustIndexPerTrack) + nested(value.wheelIndicesInTrackOrder) + nested(value.trackToWheelIndices);
}
inline size_t nested(const BrakesInfo& value)
{
    return nested(value.wheels) + nested(value.torqueMultipliers);
}
inline size_t nested(const SteeringBasicInfo& value)
{
    return nested(value.wheels) + nested(value.angleMultipliers);
}
inline size_t nested(const NonlinearCmdResponseInfo& value)
{
    return nested(value.commandValues) + nested(value.speedResponsesPerCommandValue) + nested(value.speedResponses);
}
inline size_t nested(const SuspensionComplianceInfo& value)
{
    return nested(value.wheelToeAngles) + nested(value.wheelCamberAngles) + nested(value.suspensionForceAppPoints) + nested(value.tireForceAppPoints);
}
inline size_t nested(const PreparedSuspension& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedSteeringAckermann& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedSteeringBasic& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedWheelAttachment& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedAutoGearBox& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedTire& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedBrakes& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedVehicleContext& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedNonlinearCmdResponse& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedWheel& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedClutch& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedGears& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedEngine& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedDriveBasic& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedSuspensionCompliance& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedTankDifferential& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedMultiWheelDifferential& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedDriveStandard& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedVehicle& value)
{
    return nested(value.reads);
}
inline size_t nested(const PreparedTireFrictionTable& value)
{
    return nested(value.reads);
}
inline size_t nested(const VehicleAttributeInput& value)
{
    return nested(value.tokenText);
}
inline size_t nested(const VehiclePathInput& value)
{
    return nested(value.path);
}
inline size_t nested(const PreparedVehicleReads& value)
{
    return nested(value.ownerPath) + nested(value.attributes) + nested(value.references);
}
inline size_t nested(const PreparedFixedTendon& value)
{
    return nested(value.info);
}
inline size_t nested(const PreparedParticleSystem& value)
{
    return nested(value.filteredCollisions);
}
inline size_t nested(const PreparedParticleIsosurface& value)
{
    return nested(value.passes);
}
inline size_t nested(const PreparedParticleSet& value)
{
    return nested(value.points) + nested(value.velocities) + nested(value.simulationPoints);
}
inline size_t nested(const PreparedScene& value)
{
    return nested(value.info);
}
inline size_t nested(const PreparedShapeCommon& value)
{
    return nested(value.sceneIds) + nested(value.info);
}
inline size_t nested(const PreparedJoint& value)
{
    return nested(value.info);
}
inline size_t nested(const PreparedOvstageDeformable& value)
{
    return nested(value.skin) + nested(value.skinPose) + nested(value.body);
}
template<class T> size_t outputs(const std::vector<DescPtr<T>>& values)
{
    size_t bytes = 0;
    for (const auto& value : values) if (value) bytes += nested(*value);
    return bytes;
}
} // namespace descriptor_memory
inline size_t descriptorMemberCapacityBytes(const OvstageScanResult& output)
{
    size_t bytes = 0;
    bytes += descriptor_memory::outputs(output.scenes);
    bytes += descriptor_memory::outputs(output.materials);
    bytes += descriptor_memory::outputs(output.pbdMaterials);
    bytes += descriptor_memory::outputs(output.deformableMaterials);
    bytes += descriptor_memory::outputs(output.bodies);
    bytes += descriptor_memory::outputs(output.ownedMeshes);
    bytes += descriptor_memory::outputs(output.shapes);
    bytes += descriptor_memory::outputs(output.joints);
    bytes += descriptor_memory::outputs(output.mimicJoints);
    bytes += descriptor_memory::outputs(output.collisionGroups);
    bytes += descriptor_memory::outputs(output.articulations);
    bytes += descriptor_memory::outputs(output.deformables);
    bytes += descriptor_memory::outputs(output.attachments);
    bytes += descriptor_memory::outputs(output.deformableCollisionFilters);
    bytes += descriptor_memory::outputs(output.spatialTendonAttachments);
    bytes += descriptor_memory::outputs(output.fixedTendonAxes);
    bytes += descriptor_memory::outputs(output.fixedTendons);
    bytes += descriptor_memory::outputs(output.tireFrictionTables);
    bytes += descriptor_memory::outputs(output.vehicleContexts);
    bytes += descriptor_memory::outputs(output.vehicleWheels);
    bytes += descriptor_memory::outputs(output.vehicleTires);
    bytes += descriptor_memory::outputs(output.vehicleSuspensions);
    bytes += descriptor_memory::outputs(output.vehicleEngines);
    bytes += descriptor_memory::outputs(output.vehicleGears);
    bytes += descriptor_memory::outputs(output.vehicleClutches);
    bytes += descriptor_memory::outputs(output.vehicleDrivesBasic);
    bytes += descriptor_memory::outputs(output.vehicleMultiWheelDifferentials);
    bytes += descriptor_memory::outputs(output.vehicleTankDifferentials);
    bytes += descriptor_memory::outputs(output.vehicleAutoGearBoxes);
    bytes += descriptor_memory::outputs(output.vehicleBrakes);
    bytes += descriptor_memory::outputs(output.vehicleSteeringBasic);
    bytes += descriptor_memory::outputs(output.vehicleSteeringAckermann);
    bytes += descriptor_memory::outputs(output.vehicleNonlinearCmdResponses);
    bytes += descriptor_memory::outputs(output.vehicleDrivesStandard);
    bytes += descriptor_memory::outputs(output.vehicleWheelAttachments);
    bytes += descriptor_memory::outputs(output.vehicles);
    bytes += descriptor_memory::outputs(output.vehicleSuspensionCompliances);
    bytes += descriptor_memory::outputs(output.particleSystems);
    bytes += descriptor_memory::outputs(output.particleAnisotropies);
    bytes += descriptor_memory::outputs(output.particleSmoothings);
    bytes += descriptor_memory::outputs(output.particleIsosurfaces);
    bytes += descriptor_memory::outputs(output.particleSets);
    bytes += descriptor_memory::outputs(output.particleSamplers);
    bytes += descriptor_memory::outputs(output.ccts);
    for (const auto& joint : output.joints)
    {
        if (!joint) continue;
        if (joint->type == eJointD6) bytes += descriptor_memory::nested(static_cast<const D6PhysxJointDesc&>(*joint));
        else if (joint->type == eJointSpherical) bytes += descriptor_memory::nested(static_cast<const SphericalPhysxJointDesc&>(*joint));
    }
    for (const auto& shape : output.shapes)
        if (shape && shape->type == eSpherePointsShape) bytes += descriptor_memory::nested(static_cast<const SpherePointsPhysxShapeDesc&>(*shape).spheres);
    return bytes;
}
} // namespace omni::physics::ovstage

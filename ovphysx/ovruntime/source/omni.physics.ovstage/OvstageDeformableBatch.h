// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-BATCH-001
 * @covers AC-1 AC-2 AC-3
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2 AC-3
 */
#pragma once
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedAttribute.h>
#include <algorithm>
namespace omni::physics::ovstage
{
using namespace omni::physics::parse;
struct PreparedOvstageDeformable
{
    bool prepared = false, surface = false, simLeftHanded = false, collisionLeftHanded = false, hexahedral = false;
    ObjectKey key, simMesh, collisionMesh, cookingMesh;
    TokenId simPose, collisionPose, cookingPose;
    Matrix4d transform;
    DeformableBodyParse body;
    std::vector<ObjectKey> skin;
    std::vector<TokenId> skinPose;
    PreparedAttribute<bool> hasAutoAPI;
    PreparedAttribute<uint32_t> collisionPairUpdateFrequency, collisionIterationMultiplier, hexahedralResolution;
    PreparedAttribute<float> linearDamping;
    PreparedAttribute<float> maxLinearVelocity;
    PreparedAttribute<float> sleepThreshold;
    PreparedAttribute<float> settlingThreshold;
    PreparedAttribute<float> settlingDamping;
    PreparedAttribute<float> maxDepenetrationVelocity;
    PreparedAttribute<float> selfCollisionFilterDistance;
    PreparedAttribute<uint32_t> solverPositionIterationCount;
    PreparedAttribute<bool> enableSpeculativeCCD;
    PreparedAttribute<bool> selfCollision;
    PreparedAttribute<bool> disableGravity;
    PreparedAttribute<bool> isAutoMeshSimplificationEnabled;
    PreparedAttribute<bool> isAutoRemeshingEnabled;
    PreparedAttribute<uint32_t> autoRemeshingResolution;
    PreparedAttribute<uint32_t> autoTriangleTargetCount;
    PreparedAttribute<bool> hasAutoForceConforming;
};
inline void prepareOvstageDeformableStorage(const PreparedOvstageDeformable& input, PhysxDeformableBodyDesc& output)
{
    output.sourceSimulationOwners.resize(input.body.simulationOwners.size());
    output.sourceFilteredCollisions.resize(input.body.filteredCollisions.size());
    output.skinGeomPaths.resize(input.skin.size());
    output.skinGeomBindPoseTokens.resize(input.skinPose.size());
}
inline bool fillOvstageDeformable(const PreparedOvstageDeformable& input, const SourceUnits& units, PhysxDeformableBodyDesc& output) noexcept
{
    if (!input.prepared || output.type != (input.surface ? eSurfaceDeformableBody : eVolumeDeformableBody) ||
        output.sourceSimulationOwners.size() != input.body.simulationOwners.size() ||
        output.sourceFilteredCollisions.size() != input.body.filteredCollisions.size() ||
        output.skinGeomPaths.size() != input.skin.size() || output.skinGeomBindPoseTokens.size() != input.skinPose.size()) return false;
    setToDefault(output, units);
    output.primKey = input.key;
    output.bodyEnabled = input.body.bodyEnabled;
    output.kinematicBody = input.body.kinematicBody;
    output.startsAsleep = input.body.startsAsleep;
    output.mass = input.body.mass <= 0.0f ? -1.0f : input.body.mass;
    std::copy(input.body.simulationOwners.begin(), input.body.simulationOwners.end(), output.sourceSimulationOwners.begin());
    std::copy(input.body.filteredCollisions.begin(), input.body.filteredCollisions.end(), output.sourceFilteredCollisions.begin());
    std::copy(input.skin.begin(), input.skin.end(), output.skinGeomPaths.begin());
    std::copy(input.skinPose.begin(), input.skinPose.end(), output.skinGeomBindPoseTokens.begin());
    std::copy(input.transform.data, input.transform.data + 16, output.transform.data);
    output.simMeshKey = input.simMesh;
    output.simMeshBindPoseToken = input.simPose;
    output.simMeshLeftHandedOrientation = input.simLeftHanded;
    output.collisionMeshKey = input.collisionMesh;
    output.collisionMeshBindPoseToken = input.collisionPose;
    output.collisionMeshLeftHandedOrientation = input.collisionLeftHanded;
    output.cookingSrcMeshKey = input.cookingMesh;
    output.cookingSrcMeshBindPoseToken = input.cookingPose;
    input.hasAutoAPI.read(output.hasAutoAPI);
    input.linearDamping.read(output.linearDamping);
    input.maxLinearVelocity.read(output.maxLinearVelocity);
    input.sleepThreshold.read(output.sleepThreshold);
    input.settlingThreshold.read(output.settlingThreshold);
    input.settlingDamping.read(output.settlingDamping);
    input.maxDepenetrationVelocity.read(output.maxDepenetrationVelocity);
    input.selfCollisionFilterDistance.read(output.selfCollisionFilterDistance);
    input.solverPositionIterationCount.read(output.solverPositionIterationCount);
    input.enableSpeculativeCCD.read(output.enableSpeculativeCCD);
    input.selfCollision.read(output.selfCollision);
    input.disableGravity.read(output.disableGravity);
    input.isAutoMeshSimplificationEnabled.read(output.isAutoMeshSimplificationEnabled);
    input.isAutoRemeshingEnabled.read(output.isAutoRemeshingEnabled);
    input.autoRemeshingResolution.read(output.autoRemeshingResolution);
    input.autoTriangleTargetCount.read(output.autoTriangleTargetCount);
    input.hasAutoForceConforming.read(output.hasAutoForceConforming);

    if (input.surface)
    {
        PhysxSurfaceDeformableBodyDesc& surface = static_cast<PhysxSurfaceDeformableBodyDesc&>(output);
        input.collisionPairUpdateFrequency.read(surface.collisionPairUpdateFrequency);
        input.collisionIterationMultiplier.read(surface.collisionIterationMultiplier);
    }
    else if (input.hexahedral)
    {
        PhysxVolumeDeformableBodyDesc& volume = static_cast<PhysxVolumeDeformableBodyDesc&>(output);
        volume.isAutoHexahedralMeshEnabled = true;
        input.hexahedralResolution.read(volume.autoHexahedralResolution);
    }
    return true;
}
}

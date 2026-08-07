// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <omni/physics/parse/Allocator.h>
#include <omni/physics/parse/Descriptors.h>
#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>

#include <ovstage/ovstage.h>
#include <ovstage/ovx_path_dictionary.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace omni::physics::ovstage
{
using namespace omni::physics::parse;

// ---------------------------------------------------------------------------
// OvstageWalker (ADR-0002 Milestone 2, slice 1).
//
// Produces parse-library descriptors from an ovstage instance by issuing one
// query per concept (the ovstage-idiomatic enumeration, not a tree walk),
// resolving each matched prim's data through OvstageSource, and delegating to
// the existing USD-free parsers (parseScene / parseDynamicBody / parseStaticBody
// / shape parsers). The descriptor lists are the same `DescPtr` vectors a
// `ScannedStage` holds, so slice 2 can move them into a real ScannedStage and
// route `scanStage` through the backend (the invasive ScannedStage::Impl
// generalisation) with no rework of the walker itself.
//
// Scope: Scene + RigidBody + analytic and mesh collision shapes + rigid-body
// materials (incl. material-to-shape binding) + joints (Fixed / Revolute /
// Prismatic / Spherical / Distance / D6,
// with body0/body1 resolution + local frames + axis/limit; child relationship
// targets are remapped when the endpoint body resolves from the scan or source),
// collision groups (filteredGroups + collider membership), per-shape/body
// filtered pairs, and
// articulations (via the SHARED parse::buildArticulations root-election / graph
// algorithm - the same code the native USD walker runs). Unhandled collision
// geometry is counted in `skippedShapes`.
// ---------------------------------------------------------------------------


struct OvstageScanFilter
{
    std::vector<std::string> scanRoots;
    std::vector<std::string> excludePaths;
    bool prunePointInstancerDescendants = true;
    DescendantScope descendantScope = DescendantScope::eActiveInstanced;
};

struct OvstageScanResult
{
    // The exact source that mints descriptor handles. Declared first so it is
    // destroyed after every descriptor and moved into ScannedStage on attach.
    std::unique_ptr<IPhysicsSource> source;
    std::vector<DescPtr<PhysxSceneDesc>> scenes;
    std::vector<DescPtr<PhysxMaterialDesc>> materials;
    std::vector<DescPtr<PBDMaterialDesc>> pbdMaterials;
    std::vector<DescPtr<PhysxDeformableMaterialDesc>> deformableMaterials;
    std::vector<DescPtr<PhysxRigidBodyDesc>> bodies;
    std::vector<DescPtr<PhysxShapeDesc>> shapes;
    // Owned-storage for mesh-shape MergeMeshDesc data (bounding-sphere/cube
    // approximations): the emitted Bounding{Sphere,Box}PhysxShapeDesc point at
    // these via mergedMesh, so they must outlive the shape descriptors. Moved
    // into the ScannedStage (retainOwnedMesh) in OvstageScan.cpp.
    std::vector<DescPtr<MergeMeshDesc>> ownedMeshes;
    std::vector<DescPtr<PhysxJointDesc>> joints;
    std::vector<DescPtr<MimicJointDesc>> mimicJoints;
    std::vector<DescPtr<CollisionGroupDesc>> collisionGroups;
    std::vector<DescPtr<PhysxArticulationDesc>> articulations;
    std::vector<DescPtr<PhysxDeformableBodyDesc>> deformables;
    std::vector<DescPtr<PhysxDeformableAttachmentDesc>> attachments;
    std::vector<DescPtr<PhysxDeformableCollisionFilterDesc>> deformableCollisionFilters;
    std::vector<DescPtr<PhysxTendonAttachmentDesc>> spatialTendonAttachments;
    std::vector<DescPtr<PhysxTendonAxisDesc>> fixedTendonAxes;
    std::vector<DescPtr<PhysxTendonFixedDesc>> fixedTendons;
    std::vector<DescPtr<TireFrictionTableDesc>> tireFrictionTables;
    std::vector<DescPtr<VehicleContextDesc>> vehicleContexts;
    std::vector<DescPtr<WheelDesc>> vehicleWheels;
    std::vector<DescPtr<TireDesc>> vehicleTires;
    std::vector<DescPtr<SuspensionDesc>> vehicleSuspensions;
    std::vector<DescPtr<EngineDesc>> vehicleEngines;
    std::vector<DescPtr<GearsDesc>> vehicleGears;
    std::vector<DescPtr<ClutchDesc>> vehicleClutches;
    std::vector<ObjectKey> vehicleGearsPaths;
    std::vector<ObjectKey> vehicleClutchPaths;
    std::vector<DescPtr<DriveBasicDesc>> vehicleDrivesBasic;
    std::vector<DescPtr<MultiWheelDifferentialDesc>> vehicleMultiWheelDifferentials;
    std::vector<DescPtr<TankDifferentialDesc>> vehicleTankDifferentials;
    std::vector<DescPtr<AutoGearBoxDesc>> vehicleAutoGearBoxes;
    std::vector<ObjectKey> vehicleMultiWheelDifferentialPaths;
    std::vector<ObjectKey> vehicleTankDifferentialPaths;
    std::vector<ObjectKey> vehicleAutoGearBoxPaths;
    std::vector<DescPtr<BrakesDesc>> vehicleBrakes;
    std::vector<ObjectKey> vehicleBrakesPaths;
    std::vector<TokenId> vehicleBrakesInstanceTokens;
    std::vector<DescPtr<SteeringBasicDesc>> vehicleSteeringBasic;
    std::vector<DescPtr<SteeringAckermannDesc>> vehicleSteeringAckermann;
    std::vector<ObjectKey> vehicleSteeringBasicPaths;
    std::vector<ObjectKey> vehicleSteeringAckermannPaths;
    std::vector<DescPtr<NonlinearCmdResponseDesc>> vehicleNonlinearCmdResponses;
    std::vector<ObjectKey> vehicleNonlinearCmdResponsePaths;
    std::vector<TokenId> vehicleNonlinearCmdResponseInstanceTokens;
    std::vector<DescPtr<DriveStandardDesc>> vehicleDrivesStandard;
    std::vector<ObjectKey> vehicleDrivesStandardPaths;
    std::vector<DriveStandardInfo> vehicleDrivesStandardCrossRefs;
    std::vector<DescPtr<WheelAttachmentDesc>> vehicleWheelAttachments;
    std::vector<WheelAttachmentInfo> vehicleWheelAttachmentInfos;
    std::vector<std::pair<ObjectKey, ObjectKey>> vehicleWheelAttachmentOwners;
    std::vector<DescPtr<VehicleDesc>> vehicles;
    std::vector<ObjectKey> vehiclePaths;
    std::vector<DescPtr<SuspensionComplianceDesc>> vehicleSuspensionCompliances;
    std::vector<ObjectKey> vehicleSuspensionCompliancePaths;
    std::vector<DescPtr<ParticleSystemDesc>> particleSystems;
    std::vector<DescPtr<ParticleAnisotropyDesc>> particleAnisotropies;
    std::vector<DescPtr<ParticleSmoothingDesc>> particleSmoothings;
    std::vector<DescPtr<ParticleIsosurfaceDesc>> particleIsosurfaces;
    std::vector<DescPtr<ParticleSetDesc>> particleSets;
    std::vector<DescPtr<ParticleSamplingDesc>> particleSamplers;
    std::vector<ObjectKey> particleSamplerKeys;
    bool hasPointInstancerPrims = false;

    // Collision prims that matched but whose geometry type is not handled.
    // Surfaced so callers/tests can see coverage gaps rather than silently
    // dropping them.
    size_t skippedShapes = 0;
};

OvstageScanResult scanOvstage(ovstage_instance_t* instance,
                              ovx_path_dictionary_t* dict,
                              IDescriptorAllocator& allocator,
                              ovstage_ordinal_t readOrdinal = 1,
                              const OvstageScanFilter* filter = nullptr,
                              uint64_t usdStageId = 0);

} // namespace omni::physics::ovstage

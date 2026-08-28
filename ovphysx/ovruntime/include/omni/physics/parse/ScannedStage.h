// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * `ScannedStage` — the source-agnostic snapshot of a scanned physics
 * scene. Owns the parse-library descriptors plus the `IPhysicsSource`
 * that minted their handles.
 *
 * This is the USD-free core of the scan result (ADR-0002). Backends
 * produce it: the native USD walk (`omni.physics.usd`) and the ovstage
 * walk (`omni.physics.ovstage`) both build a `ScannedStage` over their
 * own `IPhysicsSource`. The USD backend additionally derives
 * `omni::physics::usd::ScannedStage` from this type to add USD-typed
 * (`SdfPath`/`TfToken`) handle resolvers for USD consumers.
 *
 * CI invariant (parse_usd_free_check): this header lives under
 * `include/omni/physics/parse/` and therefore must stay free of any USD
 * include or USD-namespace type (the check is a substring scan, so even this
 * note avoids spelling those tokens).
 *
 * @implements REQ-PARSE-SCAN-001
 * @covers AC-1 AC-2 AC-3
 */

#pragma once

#include <omni/physics/parse/Allocator.h>
#include <omni/physics/parse/Descriptors.h>
#include <omni/physics/parse/Handles.h>
#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h> // DriveStandardInfo / WheelAttachmentInfo cross-ref structs

#include <memory>
#include <utility>
#include <vector>

namespace omni::physics::parse
{

class ScannedStage;

// Assemble an empty ScannedStage around an abstract backend source (ADR-0002
// Milestone 2). `source` becomes the scan's key/token resolver; the caller
// then moves its already-built descriptor lists
// into the returned scan's public per-concept members. Used by backends that
// produce descriptors out-of-band (the native USD walker and the ovstage
// walker). USD-free in signature — that is the whole point: it lets a backend
// build a scan without depending on the USD layer.
ScannedStage makeScannedStageFromSource(std::unique_ptr<IPhysicsSource> source);

// Snapshot of a scanned physics scene's parsed state. Owns the parse-lib
// descriptors and the backend source used to mint their handles. Immutable
// after the scan returns; consumers iterate the typed lists and can move
// descriptors out individually if they need to take ownership.
//
// `ObjectKey` / `TokenId` values on these descriptors are minted by this
// scan's source and must be resolved through that same source (or, on the USD
// side, through `omni::physics::usd::ScannedStage`'s `pathFor` / `tfTokenFor`).
// Handles are not portable across `ScannedStage` instances.
class ScannedStage
{
public:
    // Per-concept descriptor lists. Order within a list mirrors the
    // schema parser's prim-iteration order (USD: depth-first stage
    // walk), so consumers see deterministic results across runs.
    // Descriptors are owned via `parse::DescPtr<T>` — a `unique_ptr`
    // bound to the allocator passed to the scan. Consumers that want to
    // take ownership of individual descriptors call `.release()` on the
    // per-entry `DescPtr` (the allocator's matching `deallocate` runs
    // once the consumer's own release path frees the descriptor).
    std::vector<parse::DescPtr<parse::PhysxSceneDesc>>                       scenes;
    std::vector<parse::DescPtr<parse::PhysxMaterialDesc>>                    materials;
    std::vector<parse::DescPtr<parse::PBDMaterialDesc>>                      pbdMaterials;
    std::vector<parse::DescPtr<parse::PhysxDeformableMaterialDesc>>          deformableMaterials;
    std::vector<parse::DescPtr<parse::PhysxRigidBodyDesc>>                   bodies;
    std::vector<parse::DescPtr<parse::PhysxArticulationDesc>>                articulations;
    std::vector<parse::DescPtr<parse::PhysxShapeDesc>>                       shapes;
    std::vector<parse::DescPtr<parse::PhysxJointDesc>>                       joints;
    std::vector<parse::DescPtr<parse::CollisionGroupDesc>>                   collisionGroups;
    std::vector<parse::DescPtr<parse::PhysxDeformableBodyDesc>>              deformables;
    std::vector<parse::DescPtr<parse::PhysxDeformableAttachmentDesc>>        attachments;
    std::vector<parse::DescPtr<parse::PhysxDeformableCollisionFilterDesc>>   deformableCollisionFilters;
    std::vector<parse::DescPtr<parse::CapsuleCctDesc>>                       ccts;
    std::vector<parse::DescPtr<parse::MimicJointDesc>>                       mimicJoints;
    std::vector<parse::DescPtr<parse::PhysxTendonAttachmentDesc>>            spatialTendonAttachments;
    std::vector<parse::DescPtr<parse::PhysxTendonAxisDesc>>                  fixedTendonAxes;
    std::vector<parse::DescPtr<parse::PhysxTendonFixedDesc>>                 fixedTendons;
    std::vector<parse::DescPtr<parse::TireFrictionTableDesc>>                tireFrictionTables;
    std::vector<parse::DescPtr<parse::VehicleContextDesc>>                   vehicleContexts;
    std::vector<parse::DescPtr<parse::WheelDesc>>                            vehicleWheels;
    std::vector<parse::DescPtr<parse::TireDesc>>                             vehicleTires;
    std::vector<parse::DescPtr<parse::SuspensionDesc>>                       vehicleSuspensions;
    std::vector<parse::DescPtr<parse::EngineDesc>>                           vehicleEngines;
    std::vector<parse::DescPtr<parse::GearsDesc>>                            vehicleGears;
    std::vector<parse::DescPtr<parse::ClutchDesc>>                           vehicleClutches;
    // Engine path lives on its descriptor; gears/clutch don't have a
    // path field, so parallel ObjectKey side-tables. Consumer adapter
    // dedups across vehicles by SdfPath.
    std::vector<parse::ObjectKey>                                            vehicleGearsPaths;
    std::vector<parse::ObjectKey>                                            vehicleClutchPaths;
    // DriveBasicDesc carries a path field; MultiWheelDifferentialDesc /
    // TankDifferentialDesc / AutoGearBoxDesc don't, so parallel
    // ObjectKey side-tables on those.
    std::vector<parse::DescPtr<parse::DriveBasicDesc>>                       vehicleDrivesBasic;
    std::vector<parse::DescPtr<parse::MultiWheelDifferentialDesc>>           vehicleMultiWheelDifferentials;
    std::vector<parse::DescPtr<parse::TankDifferentialDesc>>                 vehicleTankDifferentials;
    std::vector<parse::DescPtr<parse::AutoGearBoxDesc>>                      vehicleAutoGearBoxes;
    std::vector<parse::ObjectKey>                                            vehicleMultiWheelDifferentialPaths;
    std::vector<parse::ObjectKey>                                            vehicleTankDifferentialPaths;
    std::vector<parse::ObjectKey>                                            vehicleAutoGearBoxPaths;
    // Each brake instance produces one BrakesDesc tagged with the
    // multi-apply instance token + the brakesIndex (0 for "brakes0",
    // 1 for "brakes1").
    std::vector<parse::DescPtr<parse::BrakesDesc>>                           vehicleBrakes;
    std::vector<parse::ObjectKey>                                            vehicleBrakesPaths;        // owner prim per entry
    std::vector<parse::TokenId>                                              vehicleBrakesInstanceTokens; // instance token per entry
    std::vector<parse::DescPtr<parse::SteeringBasicDesc>>                    vehicleSteeringBasic;
    std::vector<parse::DescPtr<parse::SteeringAckermannDesc>>                vehicleSteeringAckermann;
    std::vector<parse::ObjectKey>                                            vehicleSteeringBasicPaths;
    std::vector<parse::ObjectKey>                                            vehicleSteeringAckermannPaths;
    // NonlinearCmdResponse is multi-apply per command instance. The
    // instance token determines the attach target on the consumer
    // adapter side (drive → DriveDesc, steer → SteeringDesc,
    // brakes<N> → BrakesDesc with matching brakesIndex).
    std::vector<parse::DescPtr<parse::NonlinearCmdResponseDesc>>             vehicleNonlinearCmdResponses;
    std::vector<parse::ObjectKey>                                            vehicleNonlinearCmdResponsePaths;       // owner prim per entry
    std::vector<parse::TokenId>                                              vehicleNonlinearCmdResponseInstanceTokens;
    // DriveStandard: walker emits one descriptor per applied API plus
    // a parallel cross-ref struct with the four resolved ObjectKey
    // paths (engine / gears / autoGearBox / clutch). Consumer adapter
    // resolves the paths to engine-side pointers via the previously-
    // populated tracker maps.
    std::vector<parse::DescPtr<parse::DriveStandardDesc>>                    vehicleDrivesStandard;
    std::vector<parse::ObjectKey>                                            vehicleDrivesStandardPaths;
    std::vector<parse::DriveStandardInfo>                                    vehicleDrivesStandardCrossRefs;
    // WheelAttachment (per-vehicle, non-shareable) + SuspensionCompliance
    // (applied alongside on the same prim). WheelAttachmentDesc has
    // its own path field; SuspensionCompliance does not (parallel
    // ObjectKey side-table). A parallel WheelAttachmentInfo carries
    // the three walker-resolved rel-or-API ObjectKeys (wheel / tire /
    // suspension) so the consumer adapter can wire them to engine-side
    // pointers via the tracker maps.
    std::vector<parse::DescPtr<parse::WheelAttachmentDesc>>                  vehicleWheelAttachments;
    std::vector<parse::WheelAttachmentInfo>                                  vehicleWheelAttachmentInfos;
    // (owning-vehicle key, wheel-attachment key) for EVERY prim with
    // PhysxVehicleWheelAttachmentAPI — recorded before validation, so it
    // includes malformed attachments dropped from `vehicleWheelAttachments`
    // (bad index / nested collider). Lets the consumer enumerate a vehicle's
    // attachments (and still mark the vehicle invalid on a malformed one)
    // without a USD descendant walk.
    std::vector<std::pair<parse::ObjectKey, parse::ObjectKey>>               vehicleWheelAttachmentOwners;
    // Vehicle chassis root. Walker emits one VehicleDesc per prim
    // with PhysxVehicleAPI applied; carries scalar / bool / sticky-
    // tire / substep config only. Wheel attachments + cross-refs
    // (drive / differential / steering / brakes) + controller initial
    // state remain consumer-side responsibilities. Parallel ObjectKey
    // side-table maps a vehicles entry back to its prim path.
    // Particle systems. Walker emits one ParticleSystemDesc per
    // PhysxParticleSystem prim (offsets/solver/wind/lockedAxis + sub-API enable
    // bools + scene/filtered cross-refs); the desc carries its own systemKey.
    // Runtime material/collisionGroup ObjectId resolution stays consumer-side.
    std::vector<parse::DescPtr<parse::ParticleSystemDesc>>                   particleSystems;
    // Sub-API descriptors applied on the same PhysxParticleSystem prim; each
    // carries its owning systemKey. Emitted alongside the system.
    std::vector<parse::DescPtr<parse::ParticleAnisotropyDesc>>               particleAnisotropies;
    std::vector<parse::DescPtr<parse::ParticleSmoothingDesc>>                particleSmoothings;
    std::vector<parse::DescPtr<parse::ParticleIsosurfaceDesc>>               particleIsosurfaces;
    // Particle sets (PhysxParticleSetAPI on UsdGeomPointBased / PointInstancer).
    // Each desc carries its own primKey + particleSystemKey cross-ref.
    std::vector<parse::DescPtr<parse::ParticleSetDesc>>                      particleSets;
    // Particle samplers (PhysxParticleSamplingAPI on a UsdGeomMesh). The desc
    // has no own key, so a parallel ObjectKey side-table maps each sampler to
    // its mesh prim (mirrors vehicles/vehiclePaths).
    std::vector<parse::DescPtr<parse::ParticleSamplingDesc>>                 particleSamplers;
    std::vector<parse::ObjectKey>                                           particleSamplerKeys;

    // True when the walk saw a point-instancer-shaped prim. The load consumer
    // uses this to enable instancer parsing without probing every source prim.
    bool hasPointInstancerPrims = false;

    std::vector<parse::DescPtr<parse::VehicleDesc>>                          vehicles;
    std::vector<parse::ObjectKey>                                            vehiclePaths;
    std::vector<parse::DescPtr<parse::SuspensionComplianceDesc>>             vehicleSuspensionCompliances;
    std::vector<parse::ObjectKey>                                            vehicleSuspensionCompliancePaths;

    // The backend source that minted every `ObjectKey` / `TokenId` on these
    // descriptors. Stays alive for the scan's lifetime. Resolve handles back to
    // their source representation through it (`sourceKeyToString`,
    // `tokenToString`, …); USD consumers use the derived
    // `omni::physics::usd::ScannedStage` for `SdfPath` / `TfToken` resolution.
    // `sourcePtr()` is null only for a default-constructed (invalid) scan.
    const parse::IPhysicsSource& source() const;
    const parse::IPhysicsSource* sourcePtr() const;

    // Retain owned auxiliary descriptor storage whose raw pointers are
    // referenced by emitted descriptors (e.g. `MergeMeshDesc`, pointed to by
    // `MergeMeshPhysxShapeDesc::mergedMesh`). Kept alive until this scan is
    // destroyed — the parse-only lifetime contract of the public surface.
    void retainOwnedMesh(parse::DescPtr<parse::MergeMeshDesc> mergedMesh);

    // Default-constructs an empty ScannedStage with no source. Used as a
    // "no-op" return value when the scan target is invalid.
    ScannedStage();
    ~ScannedStage();
    ScannedStage(ScannedStage&&) noexcept;
    ScannedStage& operator=(ScannedStage&&) noexcept;

    ScannedStage(const ScannedStage&) = delete;
    ScannedStage& operator=(const ScannedStage&) = delete;

    // Implementation detail; not stable public API. `Impl` owns the backend
    // `IPhysicsSource` and the retained owned-mesh storage.
    struct Impl;

protected:
    explicit ScannedStage(std::unique_ptr<Impl> impl);

    std::unique_ptr<Impl> mImpl;

    friend ScannedStage makeScannedStageFromSource(std::unique_ptr<IPhysicsSource>);
};

} // namespace omni::physics::parse

// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @implements REQ-TENSOR-VIEW-001
 * @covers AC-1
 *
 * @implements REQ-READ-VEHICLE-001
 * @covers AC-4, AC-5
 */

#include "tensors/base/BaseSimulationView.h"
#include "tensors/base/BaseVehicleView.h"
#include "tensors/cpu/CpuArticulationView.h"
#include "tensors/cpu/CpuRigidBodyView.h"
#include "tensors/cpu/CpuSdfShapeView.h"
#include "tensors/cpu/CpuVolumeDeformableBodyView.h"
#include "tensors/cpu/CpuSurfaceDeformableBodyView.h"
#include "tensors/cpu/CpuDeformableMaterialView.h"
#include "tensors/cpu/CpuRigidContactView.h"
#include "tensors/cpu/CpuSimulationData.h"

namespace omni
{
namespace physx
{
namespace tensors
{
class CpuPointInstancerView;

class CpuSimulationView : public BaseSimulationView
{
public:
    explicit CpuSimulationView(usdparser::AttachedStage* attachedStage,
                               ::physx::PxScene* scene,
                               CpuSimulationDataPtr cpuSimData,
                               bool notifyWhenSimStopped = false);

    ~CpuSimulationView() override;

    //
    // public API
    //

    int getDeviceOrdinal() const override
    {
        return -1;
    }

    void* getCudaContext() const override
    {
        return nullptr;
    }

    CpuArticulationView* createArticulationView(const char* pattern) override;
    CpuArticulationView* createArticulationView(const std::vector<std::string>& patterns) override;

    CpuRigidBodyView* createRigidBodyView(const char* pattern) override;
    CpuRigidBodyView* createRigidBodyView(const std::vector<std::string>& patterns) override;

    // ADR-0008: build a rigid-body view over an explicit actor set (entries the caller
    // built from its own enumeration), bypassing USD-path matching.
    CpuRigidBodyView* createRigidBodyViewFromEntries(const std::vector<RigidBodyEntry>& entries);

    // As on the GPU view: entries come from the reader, which owns the enumeration.
    CpuPointInstancerView* createPointInstancerViewFromEntries(const std::vector<PointInstancerEntry>& entries);

    // ADR-0008: same, for articulations -- entries built stagelessly via buildArticulationEntry.
    // Backs the CPU ovstage joint-state read (getDofPositionsOvStage / getDofVelocitiesOvStage).
    CpuArticulationView* createArticulationViewFromEntries(const std::vector<ArticulationEntry>& entries);

    // Rigid-body view over EVERY rigid dynamic and articulation link in the scene, built once and
    // reused, with `outRowMap` giving each body its row. The host counterpart of
    // GpuSimulationView::supersetRigidView, on the same bargain: a read selects its subset with a
    // per-read row list rather than building and tearing down a view over the requested set per read.
    //
    // It also gives per-body precomputed state somewhere to live. Anything a column needs that cannot
    // be derived from a body pointer -- the articulation caches this view creates, per-body shape
    // lists for shape-scoped attributes -- belongs on the view and is built once here.
    //
    // Null when the scene holds no rigid bodies. The caller cannot tell that apart from a scene
    // whose actors have changed underneath the cache, so it should invalidate the backend's cache
    // for this scene and retry once.
    CpuRigidBodyView* supersetRigidView(
        const std::unordered_map<const ::physx::PxRigidBody*, ::physx::PxU32>** outRowMap);

    // Articulation view over EVERY articulation in the scene, built once and reused, with
    // `outRowMap` giving each articulation its row and `outEntries` the entries the rows index.
    // The host counterpart of GpuSimulationView::supersetArticulationView, on the same bargain as
    // supersetRigidView above.
    //
    // Entries are returned rather than kept private because the joint read needs them directly:
    // dofImpls names each DOF's axis and owning joint, which is what maps a joint prim to its
    // output slots.
    //
    // Null when the scene holds no articulations. The caller cannot tell that apart from a scene
    // whose articulations have changed underneath the cache, so it should invalidate the backend's
    // cache for this scene and retry once.
    CpuArticulationView* supersetArticulationView(
        const std::unordered_map<const ::physx::PxArticulationReducedCoordinate*, ::physx::PxU32>** outRowMap,
        const std::vector<ArticulationEntry>** outEntries);

    // The scene's vehicle view, built once and reused, rebuilt when the scene's vehicle set changes.
    //
    // Here and not on BaseSimulationView, even though BaseVehicleView is device-independent: a
    // GpuSimulationView exists only for a PxSceneFlag::eENABLE_DIRECT_GPU_API scene, and a vehicle
    // cannot be attached to one at all, so a vehicle accessor on the base would be unreachable there.
    //
    // Held by value, not handed out for release: it owns no PhysX resources, only a list of the
    // scene's vehicles, and dies with this view. Takes the InternalScene because vehicles hang off it
    // rather than off the PxScene this view holds.
    //
    // Null when the scene holds no vehicles, which the caller can act on directly: unlike the superset
    // views above, nothing here can fail for another reason, so there is no stale-cache case to retry.
    BaseVehicleView* vehicleView(omni::physx::internal::InternalScene& scene);

    CpuVolumeDeformableBodyView* createVolumeDeformableBodyView(const char* pattern) override;
    CpuVolumeDeformableBodyView* createVolumeDeformableBodyView(const std::vector<std::string>& patterns) override;
    CpuSurfaceDeformableBodyView* createSurfaceDeformableBodyView(const char* pattern) override;
    CpuSurfaceDeformableBodyView* createSurfaceDeformableBodyView(const std::vector<std::string>& patterns) override;
    CpuDeformableMaterialView* createDeformableMaterialView(const char* pattern) override;
    CpuDeformableMaterialView* createDeformableMaterialView(const std::vector<std::string>& patterns) override;

    // DEPRECATED
    CpuRigidContactView* createRigidContactView(const char* pattern,
                                                const char** filterPatterns,
                                                uint32_t numFilterPatterns,
                                                uint32_t maxContactDataCount) override;

    CpuRigidContactView* createRigidContactView(const std::string pattern,
                                                const std::vector<std::string>& filterPatterns,
                                                uint32_t maxContactDataCount) override;

    CpuRigidContactView* createRigidContactView(const std::vector<std::string>& patterns,
                                                const std::vector<std::vector<std::string>>& filterPatterns,
                                                uint32_t maxContactDataCount) override;

    CpuSdfShapeView* createSdfShapeView(const char* pattern, uint32_t numSamplePoints) override;


    void clearForces() override;

    bool flush() override;

    void enableGpuUsageWarnings(bool enable) override;

    void updateArticulationsKinematic() override{};

    //
    // utilities
    //

    CpuSimulationDataPtr getCpuSimulationData()
    {
        return mCpuSimData;
    }

private:
    CpuSimulationDataPtr mCpuSimData;

    // Backend-owned lifetime: this view is the backend's cached per-scene view, and the superset is
    // its child, released with it. Never handed out for the caller to release.
    CpuRigidBodyView* mSupersetRigidView = nullptr;
    std::unordered_map<const ::physx::PxRigidBody*, ::physx::PxU32> mSupersetRigidRowMap;
    CpuArticulationView* mSupersetArtiView = nullptr;
    std::unordered_map<const ::physx::PxArticulationReducedCoordinate*, ::physx::PxU32> mSupersetArtiRowMap;
    std::vector<ArticulationEntry> mSupersetArtiEntries;
    BaseVehicleView mVehicleView;
};

} // namespace tensors
} // namespace physx
} // namespace omni

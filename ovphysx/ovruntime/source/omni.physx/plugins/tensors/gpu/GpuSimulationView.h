// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @implements REQ-TENSOR-VIEW-001
 * @covers AC-1
 */

#include "tensors/base/BaseSimulationView.h"
#include "tensors/gpu/GpuArticulationView.h"
#include "tensors/gpu/GpuRigidBodyView.h"
#include "tensors/gpu/GpuSdfShapeView.h"
#include "tensors/gpu/GpuSimulationData.h"
#include "tensors/gpu/GpuRigidContactView.h"
#include "tensors/gpu/GpuVolumeDeformableBodyView.h"
#include "tensors/gpu/GpuSurfaceDeformableBodyView.h"
#include "tensors/gpu/GpuDeformableMaterialView.h"

namespace omni
{
namespace physx
{
namespace tensors
{
class GpuPointInstancerView;

class GpuSimulationView : public BaseSimulationView
{
public:
    GpuSimulationView(usdparser::AttachedStage* attachedStage,
                      ::physx::PxScene* scene,
                      GpuSimulationDataPtr gpuSimData,
                      bool notifyWhenSimStopped = false);

    ~GpuSimulationView() override;

    //
    // public API
    //

    int getDeviceOrdinal() const override
    {
        return mDevice;
    }

    void* getCudaContext() const override
    {
        return mGpuSimData ? mGpuSimData->mCtx : nullptr;
    }


    GpuArticulationView* createArticulationView(const char* pattern) override;
    GpuArticulationView* createArticulationView(const std::vector<std::string>& patterns) override;

    GpuRigidBodyView* createRigidBodyView(const char* pattern) override;
    GpuRigidBodyView* createRigidBodyView(const std::vector<std::string>& patterns) override;

    // ADR-0008: build a rigid-body view over an explicit actor set, bypassing USD-path matching.
    GpuRigidBodyView* createRigidBodyViewFromEntries(const std::vector<RigidBodyEntry>& entries);

    // ADR-0008: same, for point instancers. Entries must be assembled by the reader: the per-instance
    // prototype inverse lives in InternalActor and the instancer transform is a stage query, and this
    // layer holds no stage.
    GpuPointInstancerView* createPointInstancerViewFromEntries(const std::vector<PointInstancerEntry>& entries);

    // ADR-0008: same, for articulations -- entries built stagelessly via buildArticulationEntry.
    GpuArticulationView* createArticulationViewFromEntries(const std::vector<ArticulationEntry>& entries);

    // Superset views over every rigid body / articulation in this view's scene, built on first use and
    // owned by this view -- releasing it tears them down. A read selects its subset with a per-read
    // record list rather than constructing a view over the requested set. The row map is this view's
    // own ordering, which is why the views live here and not on the backend.
    //
    // Null when the scene holds nothing of that kind, or when the simulation data does not describe
    // the scene's actors. The caller cannot tell those apart and should invalidate the backend's
    // cache for this scene, then retry once.
    GpuRigidBodyView* supersetRigidView(
        const std::unordered_map<const ::physx::PxRigidBody*, ::physx::PxU32>** outRowMap);

    GpuArticulationView* supersetArticulationView(
        const std::unordered_map<const ::physx::PxArticulationReducedCoordinate*, ::physx::PxU32>** outRowMap,
        const std::vector<ArticulationEntry>** outEntries);

    GpuVolumeDeformableBodyView* createVolumeDeformableBodyView(const char* pattern) override;
    GpuVolumeDeformableBodyView* createVolumeDeformableBodyView(const std::vector<std::string>& patterns) override;
    GpuSurfaceDeformableBodyView* createSurfaceDeformableBodyView(const char* pattern) override;
    GpuSurfaceDeformableBodyView* createSurfaceDeformableBodyView(const std::vector<std::string>& patterns) override;
    GpuDeformableMaterialView* createDeformableMaterialView(const char* pattern) override;
    GpuDeformableMaterialView* createDeformableMaterialView(const std::vector<std::string>& patterns) override;

    // DEPRECATED
    GpuRigidContactView* createRigidContactView(const char* pattern,
                                                const char** filterPatterns,
                                                uint32_t numFilterPatterns,
                                                uint32_t maxContactDataCount) override;

    GpuRigidContactView* createRigidContactView(const std::string pattern,
                                                const std::vector<std::string>& filterPatterns,
                                                uint32_t maxContactDataCount) override;

    GpuRigidContactView* createRigidContactView(const std::vector<std::string>& patterns,
                                                const std::vector<std::vector<std::string>>& filterPatterns,
                                                uint32_t maxContactDataCount) override;

    GpuSdfShapeView* createSdfShapeView(const char* pattern, uint32_t numSamplePoints) override;

    void clearForces() override;

    bool flush() override;

    void enableGpuUsageWarnings(bool enable) override;
    void updateArticulationsKinematic() override;

    //
    // utilities
    //

    int getDevice() const
    {
        return mDevice;
    }

    GpuSimulationDataPtr getGpuSimulationData()
    {
        return mGpuSimData;
    }

private:
    int mDevice = -1;

    GpuSimulationDataPtr mGpuSimData;

    // Built on first use, released with this view.
    GpuRigidBodyView* mSupersetRigidView = nullptr;
    std::unordered_map<const ::physx::PxRigidBody*, ::physx::PxU32> mSupersetRigidRowMap;
    GpuArticulationView* mSupersetArtiView = nullptr;
    std::unordered_map<const ::physx::PxArticulationReducedCoordinate*, ::physx::PxU32> mSupersetArtiRowMap;
    std::vector<ArticulationEntry> mSupersetArtiEntries;
};

} // namespace tensors
} // namespace physx
} // namespace omni

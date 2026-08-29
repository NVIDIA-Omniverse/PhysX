// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

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

class GpuSimulationView : public BaseSimulationView
{
public:
    GpuSimulationView(PXR_NS::UsdStageRefPtr stage, GpuSimulationDataPtr gpuSimData);

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
};

} // namespace tensors
} // namespace physx
} // namespace omni

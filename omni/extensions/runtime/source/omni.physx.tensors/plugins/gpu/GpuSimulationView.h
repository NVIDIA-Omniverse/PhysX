// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../base/BaseSimulationView.h"
#include "GpuArticulationView.h"
#include "GpuRigidBodyView.h"
#include "GpuSdfShapeView.h"
#include "GpuParticleClothView.h"
#include "GpuParticleSystemView.h"
#include "GpuParticleMaterialView.h"
#include "GpuSimulationData.h"
#include "GpuRigidContactView.h"
#include "GpuSoftBodyView.h"
#include "GpuSoftBodyMaterialView.h"
#include "GpuVolumeDeformableBodyView.h"
#include "GpuSurfaceDeformableBodyView.h"
#include "GpuDeformableMaterialView.h"

namespace omni
{
namespace physx
{
namespace tensors
{

class GpuSimulationView : public BaseSimulationView
{
public:
    GpuSimulationView(pxr::UsdStageRefPtr stage, GpuSimulationDataPtr gpuSimData);

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

    GpuSoftBodyView* createSoftBodyView(const char* pattern) override;

    GpuSoftBodyMaterialView* createSoftBodyMaterialView(const char* pattern) override;

    GpuVolumeDeformableBodyView* createVolumeDeformableBodyView(const char* pattern) override;
    GpuSurfaceDeformableBodyView* createSurfaceDeformableBodyView(const char* pattern) override;
    GpuDeformableMaterialView* createDeformableMaterialView(const char* pattern) override;

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

    GpuParticleSystemView* createParticleSystemView(const char* pattern) override;

    GpuParticleClothView* createParticleClothView(const char* pattern) override;

    GpuParticleMaterialView* createParticleMaterialView(const char* pattern) override;

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

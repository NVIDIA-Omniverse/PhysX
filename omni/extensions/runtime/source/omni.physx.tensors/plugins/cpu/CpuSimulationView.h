// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../base/BaseSimulationView.h"
#include "CpuArticulationView.h"
#include "CpuRigidBodyView.h"
#include "CpuSdfShapeView.h"
#include "CpuSoftBodyView.h"
#include "CpuSoftBodyMaterialView.h"
#include "CpuVolumeDeformableBodyView.h"
#include "CpuSurfaceDeformableBodyView.h"
#include "CpuDeformableMaterialView.h"
#include "CpuRigidContactView.h"
#include "CpuParticleSystemView.h"
#include "CpuParticleMaterialView.h"
#include "CpuSimulationData.h"

namespace omni
{
namespace physx
{
namespace tensors
{

class CpuSimulationView : public BaseSimulationView
{
public:
    explicit CpuSimulationView(pxr::UsdStageRefPtr stage, CpuSimulationDataPtr cpuSimData);

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

    CpuSoftBodyView* createSoftBodyView(const char* pattern) override;

    CpuSoftBodyMaterialView* createSoftBodyMaterialView(const char* pattern) override;

    CpuVolumeDeformableBodyView* createVolumeDeformableBodyView(const char* pattern) override;
    CpuSurfaceDeformableBodyView* createSurfaceDeformableBodyView(const char* pattern) override;
    CpuDeformableMaterialView* createDeformableMaterialView(const char* pattern) override;

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

    CpuParticleSystemView* createParticleSystemView(const char* pattern) override;

    omni::physics::tensors::IParticleClothView* createParticleClothView(const char* pattern) override;

    CpuParticleMaterialView* createParticleMaterialView(const char* pattern) override;

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
};

} // namespace tensors
} // namespace physx
} // namespace omni

// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "tensors/base/BaseSimulationView.h"
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

class CpuSimulationView : public BaseSimulationView
{
public:
    explicit CpuSimulationView(PXR_NS::UsdStageRefPtr stage, CpuSimulationDataPtr cpuSimData);

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
};

} // namespace tensors
} // namespace physx
} // namespace omni

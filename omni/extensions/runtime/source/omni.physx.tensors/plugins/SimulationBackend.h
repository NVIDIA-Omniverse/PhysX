// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "PhysicsTypes.h"
#include "cpu/CpuSimulationData.h"
#include "gpu/GpuSimulationData.h"

#include <omni/physics/tensors/ISimulationBackend.h>

#include <cstdint>
#include <memory>
#include <unordered_set>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::ISimulationView;

class SimulationBackend : public omni::physics::tensors::ISimulationBackend
{
public:
    //
    // public API
    //

    ISimulationView* createSimulationView(long stageId) override;

    void reset() override;

    //
    // utilities
    //

    ::physx::PxScene* findPhysicsScene() const;

    // total number of physics simulation steps since the application started, always increasing
    int64_t getTimestamp() const;

    // number of simulation steps in the active simulation, or 0 if there is no active simulation
    int64_t getStepCount() const;

    // stage update events
    void prePhysicsUpdate();

    // manual step counting
    void incrementStepCount()
    {
        ++mManualStepCount;
    }

    void removeSimulationView(ISimulationView* view);

private:
    pxr::UsdStageRefPtr mUsdStage;
    long mStageId = -1;

    CpuSimulationDataPtr mCpuSimData;
    GpuSimulationDataPtr mGpuSimData;
    std::unordered_set<ISimulationView*> simViews;

    uint64_t mManualStepCount = 0;
};

} // namespace tensors
} // namespace physx
} // namespace omni

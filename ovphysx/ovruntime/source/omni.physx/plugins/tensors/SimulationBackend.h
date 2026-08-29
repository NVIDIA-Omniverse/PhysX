// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "tensors/PhysicsTypes.h"
#include "tensors/cpu/CpuSimulationData.h"
#include "tensors/gpu/GpuSimulationData.h"

#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace omni
{
namespace physics
{
namespace tensors
{
class ISimulationView;
}
}
}

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::ISimulationView;

// The PhysX simulation backend. Provided to consumers through the static
// PhysX runtime TensorApi table (createSimulationView/reset/resetStage). There
// is no backend registry; PhysX is the only backend.
class SimulationBackend
{
public:
    //
    // public API
    //

    ISimulationView* createSimulationView(long stageId);

    void reset();

    void resetStage(long stageId);

    //
    // utilities
    //

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
    ::physx::PxScene* findPhysicsScene(const PXR_NS::UsdStageRefPtr& usdStage) const;

    // Lock ordering: mStageDataMutex must be held to access any of the maps/sets
    // below. Views must NOT call back into removeSimulationView() during invalidate()
    // (would deadlock -- mStageDataMutex is non-recursive). Required call order when
    // both registry and backend locks are needed: registryMutex -> mStageDataMutex.
    std::mutex mStageDataMutex;

    std::unordered_map<long, CpuSimulationDataPtr>      mCpuSimDataByStage;
    std::unordered_map<long, GpuSimulationDataPtr>      mGpuSimDataByStage;
    // Views indexed per stage so resetStage() can invalidate only that stage's
    // views; the flattened set of all live views is the union of these vectors.
    std::unordered_map<long, std::vector<ISimulationView*>> mViewsByStage;

    uint64_t mManualStepCount = 0;
};

} // namespace tensors
} // namespace physx
} // namespace omni

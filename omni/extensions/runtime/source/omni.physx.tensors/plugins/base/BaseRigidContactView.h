// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../CommonTypes.h"
#include "../base/BaseSimulationData.h"

#include <omni/physics/tensors/IRigidContactView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class BaseSimulationView;

class BaseRigidContactView : public omni::physics::tensors::IRigidContactView
{
public:
    BaseRigidContactView(BaseSimulationView* sim,
                         const std::vector<RigidContactSensorEntry>& entries,
                         uint32_t numFilters,
                         uint32_t maxContactDataCount);

    virtual ~BaseRigidContactView() override;

    uint32_t getSensorCount() const override;

    uint32_t getFilterCount() const override;

    uint32_t getMaxContactDataCount() const override;

    bool check() const override;

    void release() override;

    //
    // helpers
    //
    const char* getUsdPrimName(uint32_t sensorIdx) const override;
    const char* getUsdPrimPath(uint32_t sensorIdx) const override;
    const char* getFilterUsdPrimPath(uint32_t sensorIdx, uint32_t filterIdx) const override;
    const char* getFilterUsdPrimName(uint32_t sensorIdx, uint32_t filterIdx) const override;

    void _onParentRelease();

protected:
    BaseSimulationDataPtr mSimData;
    BaseSimulationView* mSim = nullptr;

    std::vector<RigidContactSensorEntry> mEntries;

    uint32_t mNumFilters = 0;
    uint32_t mMaxContactDataCount = 0;
};

} // namespace tensors
} // namespace physx
} // namespace omni

// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../CommonTypes.h"
#include "../base/BaseSimulationData.h"

#include <omni/physics/tensors/ISdfShapeView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class BaseSimulationView;

class BaseSdfShapeView : public omni::physics::tensors::ISdfShapeView
{
public:
    BaseSdfShapeView(BaseSimulationView* sim, const std::vector<SdfShapeEntry>& entries);

    virtual ~BaseSdfShapeView() override;

    uint32_t getCount() const override;
    uint32_t getMaxNumPoints() const override;

    bool check() const override;

    void release() override;

    //
    // helpers
    //
    const char* getUsdPrimPath(uint32_t Idx) const override;
    void _onParentRelease();

private:
    BaseSimulationView* mSim = nullptr;

protected:
    BaseSimulationDataPtr mSimData;

    std::vector<SdfShapeEntry> mEntries;
    std::vector<uint32_t> mAllIndices;
    uint32_t mMaxNumPoints = 0;
};

} // namespace tensors
} // namespace physx
} // namespace omni

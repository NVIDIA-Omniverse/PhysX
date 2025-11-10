// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../base/BaseSimulationData.h"

#include <omni/physics/tensors/ISoftBodyView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class BaseSimulationView;

class BaseSoftBodyView : public omni::physics::tensors::ISoftBodyView
{
public:
    BaseSoftBodyView(BaseSimulationView* sim, const std::vector<SoftBodyEntry>& entries);

    virtual ~BaseSoftBodyView() override;

    uint32_t getCount() const override;
    uint32_t getMaxElementsPerBody() const override;
    uint32_t getMaxSimElementsPerBody() const override;
    uint32_t getMaxVerticesPerBody() const override;
    uint32_t getMaxSimVerticesPerBody() const override;
    bool getTransforms(const TensorDesc* dstTensor) const override;
    bool check() const override;

    void release() override;

    //
    // helpers
    //

    void _onParentRelease();

private:
    BaseSimulationView* mSim = nullptr;

protected:
    BaseSimulationDataPtr mSimData;
    uint32_t mMaxElementsPerBody, mMaxSimElementsPerBody;
    uint32_t mMaxVerticesPerBody, mMaxSimVerticesPerBody;
    std::vector<SoftBodyEntry> mEntries;
};

} // namespace tensors
} // namespace physx
} // namespace omni

// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../CommonTypes.h"
#include "../base/BaseSimulationData.h"

#include <omni/physics/tensors/IParticleSystemView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class BaseSimulationView;

class BaseParticleSystemView : public omni::physics::tensors::IParticleSystemView
{
public:
    BaseParticleSystemView(BaseSimulationView* sim, const std::vector<ParticleSystemEntry>& entries);

    virtual ~BaseParticleSystemView() override;

    uint32_t getCount() const override;

    bool check() const override;

    void release() override;

    bool getSolidRestOffset(const TensorDesc* dstTensor) const;
    bool setSolidRestOffset(const TensorDesc* srcTensor, const TensorDesc* indexTensor);

    bool getFluidRestOffset(const TensorDesc* dstTensor) const;
    bool setFluidRestOffset(const TensorDesc* srcTensor, const TensorDesc* indexTensor);

    bool getParticleContactOffset(const TensorDesc* dstTensor) const;
    bool setParticleContactOffset(const TensorDesc* srcTensor, const TensorDesc* indexTensor);

    bool getWind(const TensorDesc* dstTensor) const;
    bool setWind(const TensorDesc* srcTensor, const TensorDesc* indexTensor);

    //
    // helpers
    //

    void _onParentRelease();

private:
    BaseSimulationView* mSim = nullptr;

protected:
    BaseSimulationDataPtr mSimData;

    std::vector<ParticleSystemEntry> mEntries;
    std::vector<uint32_t> mAllIndices;
};

} // namespace tensors
} // namespace physx
} // namespace omni

// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../CommonTypes.h"
#include "../base/BaseSimulationData.h"

#include <omni/physics/tensors/IParticleClothView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class BaseSimulationView;

class BaseParticleClothView : public omni::physics::tensors::IParticleClothView
{
public:
    BaseParticleClothView(BaseSimulationView* sim, const std::vector<ParticleClothEntry>& entries);

    virtual ~BaseParticleClothView() override;

    uint32_t getCount() const override;

    uint32_t getMaxParticlesPerCloth() const override;

    uint32_t getMaxSpringsPerCloth() const override;

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

    std::vector<ParticleClothEntry> mEntries;
    uint32_t mMaxParticlesPerCloth;
    uint32_t mMaxSpringsPerCloth;
};

} // namespace tensors
} // namespace physx
} // namespace omni

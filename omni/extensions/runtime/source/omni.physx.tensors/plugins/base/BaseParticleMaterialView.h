// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../CommonTypes.h"
#include "../base/BaseSimulationData.h"

#include <omni/physics/tensors/IParticleMaterialView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class BaseSimulationView;

class BaseParticleMaterialView : public omni::physics::tensors::IParticleMaterialView
{
public:
    BaseParticleMaterialView(BaseSimulationView* sim, const std::vector<ParticleMaterialEntry>& entries);

    virtual ~BaseParticleMaterialView() override;

    uint32_t getCount() const override;

    bool check() const override;

    void release() override;

    bool getFriction(const TensorDesc* dstTensor) const;
    bool setFriction(const TensorDesc* srcTensor, const TensorDesc* indexTensor);

    bool getDamping(const TensorDesc* dstTensor) const;
    bool setDamping(const TensorDesc* srcTensor, const TensorDesc* indexTensor);

    bool getGravityScale(const TensorDesc* dstTensor) const;
    bool setGravityScale(const TensorDesc* srcTensor, const TensorDesc* indexTensor);

    bool getLift(const TensorDesc* dstTensor) const;
    bool setLift(const TensorDesc* srcTensor, const TensorDesc* indexTensor);

    bool getDrag(const TensorDesc* dstTensor) const;
    bool setDrag(const TensorDesc* srcTensor, const TensorDesc* indexTensor);

    //
    // helpers
    //

    void _onParentRelease();

private:
    BaseSimulationView* mSim = nullptr;

protected:
    BaseSimulationDataPtr mSimData;

    std::vector<ParticleMaterialEntry> mEntries;
    std::vector<uint32_t> mAllIndices;
};

} // namespace tensors
} // namespace physx
} // namespace omni

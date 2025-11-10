// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../base/BaseSimulationData.h"
#include "../GlobalsAreBad.h"
#include <omni/physics/tensors/ISoftBodyMaterialView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class BaseSimulationView;

class BaseSoftBodyMaterialView : public omni::physics::tensors::ISoftBodyMaterialView
{
public:
    BaseSoftBodyMaterialView(BaseSimulationView* sim, const std::vector<SoftBodyMaterialEntry>& entries);
    virtual ~BaseSoftBodyMaterialView() override;

    uint32_t getCount() const override;

    bool getYoungsModulus(const TensorDesc* dstTensor) const;
    bool setYoungsModulus(const TensorDesc* srcTensor, const TensorDesc* indexTensor);

    bool getPoissonsRatio(const TensorDesc* dstTensor) const;
    bool setPoissonsRatio(const TensorDesc* srcTensor, const TensorDesc* indexTensor);

    bool getDynamicFriction(const TensorDesc* dstTensor) const;
    bool setDynamicFriction(const TensorDesc* srcTensor, const TensorDesc* indexTensor);

    bool getDamping(const TensorDesc* dstTensor) const;
    bool setDamping(const TensorDesc* srcTensor, const TensorDesc* indexTensor);

    bool getDampingScale(const TensorDesc* dstTensor) const;
    bool setDampingScale(const TensorDesc* srcTensor, const TensorDesc* indexTensor);

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
    std::vector<SoftBodyMaterialEntry> mEntries;
    std::vector<uint32_t> mAllIndices;
};

} // namespace tensors
} // namespace physx
} // namespace omni

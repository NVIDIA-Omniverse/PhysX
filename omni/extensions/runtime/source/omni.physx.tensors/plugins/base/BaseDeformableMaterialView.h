// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "../base/BaseSimulationData.h"
#include "../GlobalsAreBad.h"
#include <omni/physics/tensors/IDeformableMaterialView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class BaseSimulationView;

class BaseDeformableMaterialView : public omni::physics::tensors::IDeformableMaterialView
{
public:
    BaseDeformableMaterialView(BaseSimulationView* sim, const std::vector<DeformableMaterialEntry>& entries);
    virtual ~BaseDeformableMaterialView() override;

    uint32_t getCount() const override;

    bool getDynamicFriction(const TensorDesc* dstTensor) const;
    bool setDynamicFriction(const TensorDesc* srcTensor, const TensorDesc* indexTensor);

    bool getYoungsModulus(const TensorDesc* dstTensor) const;
    bool setYoungsModulus(const TensorDesc* srcTensor, const TensorDesc* indexTensor);

    bool getPoissonsRatio(const TensorDesc* dstTensor) const;
    bool setPoissonsRatio(const TensorDesc* srcTensor, const TensorDesc* indexTensor);

    bool check() const override;
    void release() override;

    //
    // helpers
    //

    void _onParentRelease();

private:
    BaseSimulationView* mSim = nullptr;

protected:

    using MaterialSetter = void (::physx::PxDeformableMaterial::*)(::physx::PxReal);
    using MaterialGetter = ::physx::PxReal (::physx::PxDeformableMaterial::*)() const;

    bool getProperty(const TensorDesc* dstTensor,
                     const MaterialGetter materialGetter,
                     const char* parameterName,
                     const char* callerFunctionName) const;

    bool setProperty(const TensorDesc* srcTensor,
                     const TensorDesc* indexTensor,
                     const MaterialSetter materialSetter,
                     const char* parameterName,
                     const char* callerFunctionName);

    BaseSimulationDataPtr mSimData;
    std::vector<DeformableMaterialEntry> mEntries;
    std::vector<uint32_t> mAllIndices;
};

} // namespace tensors
} // namespace physx
} // namespace omni

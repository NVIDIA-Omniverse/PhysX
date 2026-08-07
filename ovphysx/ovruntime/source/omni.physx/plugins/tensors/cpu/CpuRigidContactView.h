// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "tensors/CommonTypes.h"
#include "tensors/base/BaseRigidContactView.h"
#include "tensors/cpu/CpuSimulationData.h"

#include <string>
#include <vector>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::ContactDataReadStatus;
using omni::physics::tensors::TensorDesc;

class CpuSimulationView;

class CpuRigidContactView : public BaseRigidContactView
{
public:
    CpuRigidContactView(CpuSimulationView* sim,
                        const std::vector<RigidContactSensorEntry>& entries,
                        uint32_t numFilters,
                        uint32_t maxContactDataCount);

    ~CpuRigidContactView() override;

    bool getNetContactForces(const TensorDesc* dstTensor, float dt) const override;

    bool getContactForceMatrix(const TensorDesc* dstTensor, float dt) const override;

    ContactDataReadStatus getContactData(const TensorDesc* contactForceTensor,
                                         const TensorDesc* contactPointTensor,
                                         const TensorDesc* contactNormalTensor,
                                         const TensorDesc* contactSeparationTensor,
                                         const TensorDesc* contactCountTensor,
                                         const TensorDesc* contactStartIndicesTensor,
                                         float dt) const override;

    ContactDataReadStatus getFrictionData(const TensorDesc* FrictionForceTensor,
                                          const TensorDesc* contactPointTensor,
                                          const TensorDesc* contactCountTensor,
                                          const TensorDesc* contactStartIndicesTensor,
                                          float dt) const override;

    ContactDataReadStatus getRawContactData(const TensorDesc* contactForceTensor,
                                            const TensorDesc* contactPointTensor,
                                            const TensorDesc* contactNormalTensor,
                                            const TensorDesc* contactSeparationTensor,
                                            const TensorDesc* contactCountTensor,
                                            const TensorDesc* contactStartIndicesTensor,
                                            const TensorDesc* otherActorIdsTensor,
                                            float dt) const override;

private:
    CpuSimulationDataPtr mCpuSimData;

    std::vector<RigidContactBucket> mBuckets;
};

} // namespace tensors
} // namespace physx
} // namespace omni

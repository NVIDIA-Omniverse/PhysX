// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-40
 */

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
using omni::physics::tensors::TensorDesc;

class CpuSimulationView;

class CpuRigidContactView : public BaseRigidContactView
{
public:
    CpuRigidContactView(CpuSimulationView* sim,
                        std::vector<RigidContactSensorEntry>&& entries,
                        uint32_t numFilters,
                        uint32_t maxContactDataCount);

    ~CpuRigidContactView() override;

    bool getNetContactForces(const TensorDesc* dstTensor, float dt) const override;

    bool getContactForceMatrix(const TensorDesc* dstTensor, float dt) const override;

    bool getContactData(const TensorDesc* contactForceTensor,
                        const TensorDesc* contactPointTensor,
                        const TensorDesc* contactNormalTensor,
                        const TensorDesc* contactSeparationTensor,
                        const TensorDesc* contactCountTensor,
                        const TensorDesc* contactStartIndicesTensor,
                        float dt) const override;

    bool getFrictionData(const TensorDesc* FrictionForceTensor,
                         const TensorDesc* contactPointTensor,
                         const TensorDesc* contactCountTensor,
                         const TensorDesc* contactStartIndicesTensor,
                         float dt) const override;

    bool getRawContactData(const TensorDesc* contactForceTensor,
                           const TensorDesc* contactPointTensor,
                           const TensorDesc* contactNormalTensor,
                           const TensorDesc* contactSeparationTensor,
                           const TensorDesc* sensorLayoutTensor,
                           const TensorDesc* actorIdsTensor,
                           float dt) const override;

private:
    CpuSimulationDataPtr mCpuSimData;

    std::vector<RigidContactBucket> mBuckets;
};

} // namespace tensors
} // namespace physx
} // namespace omni

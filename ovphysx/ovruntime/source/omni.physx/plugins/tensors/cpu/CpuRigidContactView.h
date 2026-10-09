// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-TENSOR-CONTACT-003
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-40
 */

#pragma once

#include "tensors/CommonTypes.h"
#include "tensors/ForceComponent.h"
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

    bool getNetNormalContactForces(const TensorDesc* dstTensor, float dt) const override;

    bool getNetFrictionContactForces(const TensorDesc* dstTensor, float dt) const override;

    bool getNormalContactForceMatrix(const TensorDesc* dstTensor, float dt) const override;

    bool getFrictionContactForceMatrix(const TensorDesc* dstTensor, float dt) const override;

    bool getNormalContactData(const TensorDesc* contactForceTensor,
                              const TensorDesc* contactPointTensor,
                              const TensorDesc* contactNormalTensor,
                              const TensorDesc* contactSeparationTensor,
                              const TensorDesc* contactCountTensor,
                              const TensorDesc* contactStartIndicesTensor,
                              uint32_t* outRequiredContactCount,
                              float dt) const override;

    bool getFrictionContactData(const TensorDesc* FrictionForceTensor,
                                const TensorDesc* contactPointTensor,
                                const TensorDesc* contactCountTensor,
                                const TensorDesc* contactStartIndicesTensor,
                                uint32_t* outRequiredFrictionCount,
                                float dt) const override;

    bool getRawContactData(const TensorDesc* contactForceTensor,
                           const TensorDesc* contactPointTensor,
                           const TensorDesc* contactNormalTensor,
                           const TensorDesc* contactSeparationTensor,
                           const TensorDesc* sensorLayoutTensor,
                           const TensorDesc* actorIdsTensor,
                           uint32_t* outRequiredContactCount,
                           float dt) const override;

private:
    void accumulatePairImpulse(::physx::PxVec3& impulse,
                               const RigidContactHeaderRef& headerRef,
                               ForceComponent component) const;

    bool getNetForces(const TensorDesc* dstTensor,
                      float dt,
                      ForceComponent component,
                      const char* description,
                      const char* functionName) const;

    bool getForceMatrix(const TensorDesc* dstTensor,
                        float dt,
                        ForceComponent component,
                        const char* description,
                        const char* functionName) const;

    CpuSimulationDataPtr mCpuSimData;

    std::vector<RigidContactBucket> mBuckets;
};

} // namespace tensors
} // namespace physx
} // namespace omni

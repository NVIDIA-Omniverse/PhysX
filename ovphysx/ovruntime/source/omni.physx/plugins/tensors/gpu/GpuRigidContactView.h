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
#include "tensors/gpu/GpuSimulationData.h"

#include <omni/physics/tensors/IRigidContactView.h>

#include <string>
#include <vector>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class GpuSimulationView;


class GpuRigidContactView : public BaseRigidContactView
{
public:
    GpuRigidContactView(GpuSimulationView* sim,
                        std::vector<RigidContactSensorEntry>&& entries,
                        uint32_t numFilters,
                        uint32_t maxContactDataCount,
                        int device);

    ~GpuRigidContactView() override;

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

    int mDevice = -1;

    GpuSimulationDataPtr mGpuSimData;

    // O(1) map from physx GPU rigid dynamic index to referent index
    ::physx::PxU32* mRdContactIndicesDev = nullptr;

    // O(1) map from physx GPU articulation link index to referent index
    ::physx::PxU32* mLinkContactIndicesDev = nullptr;

    GpuRigidContactFilterIdPair* mFilterLookupDev = nullptr;

    // Scratch for getRawContactData's per-sensor bookkeeping: [0, numSensors) counts,
    // [numSensors, 2*numSensors) start indices. Kept contiguous and separate from the
    // caller's interleaved (numSensors, 2) tensor so exclusiveScan and the count/fill
    // kernels keep taking plain arrays; a final pack kernel interleaves the result.
    ::physx::PxU32* mRawLayoutScratchDev = nullptr;

    ::physx::PxU32* mContactCountMatrix = nullptr;
};

} // namespace tensors
} // namespace physx
} // namespace omni

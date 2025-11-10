// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../CommonTypes.h"
#include "../base/BaseRigidContactView.h"
#include "GpuSimulationData.h"

#include <omni/physics/tensors/IRigidContactView.h>

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
                        const std::vector<RigidContactSensorEntry>& entries,
                        uint32_t numFilters,
                        uint32_t maxContactDataCount,
                        int device);

    ~GpuRigidContactView() override;

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

private:
    int mDevice = -1;

    GpuSimulationDataPtr mGpuSimData;

    // O(1) map from physx GPU rigid dynamic index to referent index
    ::physx::PxU32* mRdContactIndicesDev = nullptr;

    // O(1) map from physx GPU articulation link index to referent index
    ::physx::PxU32* mLinkContactIndicesDev = nullptr;

    GpuRigidContactFilterIdPair* mFilterLookupDev = nullptr;

    ::physx::PxU32* mContactCountMatrix = nullptr;
};

} // namespace tensors
} // namespace physx
} // namespace omni

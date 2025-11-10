// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../CommonTypes.h"
#include "../base/BaseRigidBodyView.h"
#include "CpuSimulationData.h"

#include <omni/physics/tensors/IRigidBodyView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class CpuSimulationView;

class CpuRigidBodyView : public BaseRigidBodyView
{
public:
    CpuRigidBodyView(CpuSimulationView* sim, const std::vector<RigidBodyEntry>& entries);

    ~CpuRigidBodyView() override;

    bool getTransforms(const TensorDesc* dstTensor) const override;
    bool getVelocities(const TensorDesc* dstTensor) const override;
    bool getAccelerations(const TensorDesc* dstTensor) const override;

    bool setKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    bool applyForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                         const TensorDesc* srcTorqueTensor,
                                         const TensorDesc* srcPositionTensor,
                                         const TensorDesc* indexTensor,
                                         const bool isGlobal) override;

private:
    void prepareDirtyForceTracker();

    CpuSimulationDataPtr mCpuSimData;

    CpuRigidBodyDirtyForceTrackerPtr mDirtyForceTracker;

    // for bodies that are root articulation links, we use the articulation cache to set transforms and velocities
    std::vector<::physx::PxArticulationReducedCoordinate*> mArticulations;
    std::vector<::physx::PxArticulationCache*> mArticulationCaches;

    std::vector<uint32_t> mAllIndices;
};

} // namespace tensors
} // namespace physx
} // namespace omni

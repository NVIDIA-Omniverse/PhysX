// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../ArticulationMetatype.h"
#include "../CommonTypes.h"
#include "../base/BaseArticulationView.h"
#include "CpuSimulationData.h"

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class CpuSimulationView;

class CpuArticulationView : public BaseArticulationView
{
public:
    CpuArticulationView(CpuSimulationView* sim, const std::vector<ArticulationEntry>& entries);

    ~CpuArticulationView() override;

    //
    // public API
    //

    bool getLinkTransforms(const TensorDesc* dstTensor) const override;
    bool getLinkVelocities(const TensorDesc* dstTensor) const override;
    bool getLinkAccelerations(const TensorDesc* dstTensor) const override;

    bool getRootTransforms(const TensorDesc* dstTensor) const override;
    bool getRootVelocities(const TensorDesc* dstTensor) const override;

    bool setRootTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setRootVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    bool getDofPositions(const TensorDesc* dstTensor) const override;
    bool getDofVelocities(const TensorDesc* dstTensor) const override;

    bool setDofPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDofVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    bool setDofActuationForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    bool setDofPositionTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDofVelocityTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    bool getDofPositionTargets(const TensorDesc* dstTensor) const override;
    bool getDofVelocityTargets(const TensorDesc* dstTensor) const override;

    bool getDofActuationForces(const TensorDesc* dstTensor) const override;
    bool getDofProjectedJointForces(const TensorDesc* dstTensor) const override;

    bool getJacobians(const TensorDesc* dstTensor) const override;
    bool getMassMatrices(const TensorDesc* dstTensor) const override; // deprecated
    bool getGeneralizedMassMatrices(const TensorDesc* dstTensor) const override;

    bool getCoriolisAndCentrifugalForces(const TensorDesc* dstTensor) const override; // deprecated
    bool getCoriolisAndCentrifugalCompensationForces(const TensorDesc* dstTensor) const override;
    bool getGeneralizedGravityForces(const TensorDesc* dstTensor) const override; // deprecated
    bool getGravityCompensationForces(const TensorDesc* dstTensor) const override;

    bool getArticulationMassCenter(const TensorDesc* dstTensor, bool localFrame) const override;
    bool getArticulationCentroidalMomentum(const TensorDesc* dstTensor) const override;

    bool getLinkIncomingJointForce(const TensorDesc* dstTensor) const override;
    bool applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                         const TensorDesc* srcTorqueTensor,
                                         const TensorDesc* srcPositionTensor,
                                         const TensorDesc* indexTensor,
                                         const bool isGlobal) override;

    // tendons
    bool getFixedTendonStiffnesses(const TensorDesc* dstTensor) const override;
    bool getFixedTendonDampings(const TensorDesc* dstTensor) const override;
    bool getFixedTendonLimitStiffnesses(const TensorDesc* dstTensor) const override;
    bool getFixedTendonLimits(const TensorDesc* dstTensor) const override;
    bool getFixedTendonfixedSpringRestLengths(const TensorDesc* dstTensor) const override;
    bool getFixedTendonOffsets(const TensorDesc* dstTensor) const override;
    bool getSpatialTendonStiffnesses(const TensorDesc* dstTensor) const override;
    bool getSpatialTendonDampings(const TensorDesc* dstTensor) const override;
    bool getSpatialTendonLimitStiffnesses(const TensorDesc* dstTensor) const override;
    bool getSpatialTendonOffsets(const TensorDesc* dstTensor) const override;

    bool setFixedTendonProperties(const TensorDesc* stiffnesses,
                                  const TensorDesc* dampings,
                                  const TensorDesc* limitStiffnesses,
                                  const TensorDesc* limits,
                                  const TensorDesc* restLengths,
                                  const TensorDesc* offsets,
                                  const TensorDesc* indexTensor) const override;
    bool setSpatialTendonProperties(const TensorDesc* stiffnesses,
                                    const TensorDesc* dampings,
                                    const TensorDesc* limitStiffnesses,
                                    const TensorDesc* offsets,
                                    const TensorDesc* indexTensor) const override;
    /*bool setFixedTendonStiffnesses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    bool setFixedTendonDampings(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    bool setFixedTendonLimitStiffness(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    bool setFixedTendonLimits(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    bool setFixedTendonfixedSpringRestLengths(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const
    override;*/

private:
    void prepareDirtyForceTracker();
    CpuRigidBodyDirtyForceTrackerPtr mDirtyForceTracker;
    std::vector<::physx::PxArticulationCache*> mArticulationCaches;
    CpuSimulationDataPtr mCpuSimData;
    std::vector<uint32_t> mAllIndices;
};

} // namespace tensors
} // namespace physx
} // namespace omni

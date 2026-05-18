// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
    bool getGeneralizedMassMatrices(const TensorDesc* dstTensor) const override;

    bool getCoriolisAndCentrifugalCompensationForces(const TensorDesc* dstTensor) const override;
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

    // Masked overrides
    bool setRootTransformsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setRootVelocitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDofPositionsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDofVelocitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDofActuationForcesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDofPositionTargetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDofVelocityTargetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDofLimitsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDofStiffnessesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDofDampingsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDofMaxForcesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDofDriveModelPropertiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDofFrictionCoefficientsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDofFrictionPropertiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDofMaxVelocitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDofArmaturesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool applyForcesAndTorquesAtPositionMasked(const TensorDesc* srcForceTensor,
                                               const TensorDesc* srcTorqueTensor,
                                               const TensorDesc* srcPositionTensor,
                                               const TensorDesc* maskTensor,
                                               const bool isGlobal) override;
    bool setMassesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setCOMsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setInertiasMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDisableGravitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setMaterialPropertiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;
    bool setCompliantMaterialPropertiesMasked(const TensorDesc* srcTensor,
                                              const TensorDesc* srcCombineTensor,
                                              const TensorDesc* maskTensor) const override;
    bool setRestOffsetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;
    bool setContactOffsetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;
    bool setFixedTendonPropertiesMasked(const TensorDesc* stiffnesses,
                                        const TensorDesc* dampings,
                                        const TensorDesc* limitStiffnesses,
                                        const TensorDesc* limits,
                                        const TensorDesc* restLengths,
                                        const TensorDesc* offsets,
                                        const TensorDesc* maskTensor) const override;
    bool setSpatialTendonPropertiesMasked(const TensorDesc* stiffnesses,
                                          const TensorDesc* dampings,
                                          const TensorDesc* limitStiffnesses,
                                          const TensorDesc* offsets,
                                          const TensorDesc* maskTensor) const override;

private:
    void prepareDirtyForceTracker();
    CpuRigidBodyDirtyForceTrackerPtr mDirtyForceTracker;
    std::vector<::physx::PxArticulationCache*> mArticulationCaches;
    CpuSimulationDataPtr mCpuSimData;
    // Note: mAllIndices is inherited from BaseArticulationView (protected), initialized there.
};

} // namespace tensors
} // namespace physx
} // namespace omni

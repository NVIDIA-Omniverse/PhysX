// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "tensors/CommonTypes.h"
#include "tensors/base/BaseRigidBodyView.h"
#include "tensors/cpu/CpuSimulationData.h"

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

    // Fused ovstage output columns, one self-contained call per attribute, matching the GPU view so
    // the reader calls the same thing on either backend (ADR-0008). Writes the packed destination
    // column straight from the actors -- no [N,7]/[N,6] intermediate, and only the getter the column
    // needs is queried.
    //
    // `outRecordIdx` maps destination slot -> view record (null = identity), so a caller can serve a
    // subset of a larger view; `rowsToken` is unused here, there being no device copy to keep current,
    // and exists so both backends share one signature.
    bool getPositionsOvStage(const TensorDesc* dstTensor,
                             const ::physx::PxU32* outRecordIdx,
                             ::physx::PxU32 numOutputs,
                             uint64_t rowsToken) const;
    bool getOrientationsOvStage(const TensorDesc* dstTensor,
                                const ::physx::PxU32* outRecordIdx,
                                ::physx::PxU32 numOutputs,
                                uint64_t rowsToken) const;
    bool getLinearVelocitiesOvStage(const TensorDesc* dstTensor,
                                    const ::physx::PxU32* outRecordIdx,
                                    ::physx::PxU32 numOutputs,
                                    uint64_t rowsToken) const;
    bool getAngularVelocitiesOvStage(const TensorDesc* dstTensor,
                                     const ::physx::PxU32* outRecordIdx,
                                     ::physx::PxU32 numOutputs,
                                     uint64_t rowsToken) const;
    // Requires PxSceneFlag::eENABLE_BODY_ACCELERATIONS, which PhysXScene sets on every scene it
    // creates. Without it PhysX reports zero for a rigid dynamic rather than refusing, so a scene
    // built elsewhere would read as "not accelerating" instead of failing.
    bool getLinearAccelerationsOvStage(const TensorDesc* dstTensor,
                                       const ::physx::PxU32* outRecordIdx,
                                       ::physx::PxU32 numOutputs,
                                       uint64_t rowsToken) const;
    bool getAngularAccelerationsOvStage(const TensorDesc* dstTensor,
                                        const ::physx::PxU32* outRecordIdx,
                                        ::physx::PxU32 numOutputs,
                                        uint64_t rowsToken) const;

    // ovstage column WRITE (ADR-0012), the return direction of the four getters above and the host
    // counterpart of GpuRigidBodyView's. Same signature on both backends so the writer's table calls
    // one thing; `rowsToken` is unused here, there being no device copy to keep current.
    //
    // Position and orientation are two slices of one PxTransform, so each reads the current pose to
    // preserve the half it is not writing -- the host mirror of the GPU path's read-modify-write,
    // and cheap here because it is one accessor per body rather than a bulk DirectGPU read.
    bool setPositionsOvStage(const TensorDesc* srcTensor,
                             const ::physx::PxU32* outRecordIdx,
                             ::physx::PxU32 numOutputs,
                             uint64_t rowsToken);
    bool setOrientationsOvStage(const TensorDesc* srcTensor,
                                const ::physx::PxU32* outRecordIdx,
                                ::physx::PxU32 numOutputs,
                                uint64_t rowsToken);
    // CPU counterpart of GpuRigidBodyView::setWrenchesOvStage. Same [N,9] layout and the same
    // torque-about-COM conversion, expressed through PxRigidBodyExt rather than by hand so the host
    // path cannot drift from PhysX's own definition of "force at a point".
    bool setWrenchesOvStage(const TensorDesc* srcTensor,
                            const ::physx::PxU32* outRecordIdx,
                            ::physx::PxU32 numOutputs,
                            uint64_t rowsToken);

    // CPU counterpart of GpuRigidBodyView::setForcesOvStage. addForce, not setForce: PhysX
    // ACCUMULATES within a step and clears at the end of it, which is the write-only contract.
    //
    // Unlike the velocity setters below, this one serves articulation LINKS too -- addForce is on
    // PxRigidBody, not PxRigidDynamic.
    bool setForcesOvStage(const TensorDesc* srcTensor,
                          const ::physx::PxU32* outRecordIdx,
                          ::physx::PxU32 numOutputs,
                          uint64_t rowsToken);

    bool setLinearVelocitiesOvStage(const TensorDesc* srcTensor,
                                    const ::physx::PxU32* outRecordIdx,
                                    ::physx::PxU32 numOutputs,
                                    uint64_t rowsToken);
    bool setAngularVelocitiesOvStage(const TensorDesc* srcTensor,
                                     const ::physx::PxU32* outRecordIdx,
                                     ::physx::PxU32 numOutputs,
                                     uint64_t rowsToken);

    bool setKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    bool applyForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                         const TensorDesc* srcTorqueTensor,
                                         const TensorDesc* srcPositionTensor,
                                         const TensorDesc* indexTensor,
                                         const bool isGlobal) override;

    // Masked variants
    bool setKinematicTargetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setTransformsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setVelocitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool applyForcesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool applyForcesAndTorquesAtPositionMasked(const TensorDesc* srcForceTensor,
                                               const TensorDesc* srcTorqueTensor,
                                               const TensorDesc* srcPositionTensor,
                                               const TensorDesc* maskTensor,
                                               const bool isGlobal) override;
    bool setMassesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setCOMsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setInertiasMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    // setDisable*/material/rest/contact/compliant Masked: BaseRigidBodyView (CPU-only host mask)

private:
    bool scatterPoseColumnOvStage(bool wantOrientation,
                                  const ::physx::PxU32* outRecordIdx,
                                  ::physx::PxU32 numOutputs,
                                  const TensorDesc* srcTensor);
    bool scatterVelocityColumnOvStage(bool wantAngular,
                                      const ::physx::PxU32* outRecordIdx,
                                      ::physx::PxU32 numOutputs,
                                      const TensorDesc* srcTensor);

    bool gatherPoseColumnOvStage(bool wantOrientation,
                                 const ::physx::PxU32* outRecordIdx,
                                 ::physx::PxU32 numOutputs,
                                 const TensorDesc* dstTensor) const;
    // Serves all four of linear/angular x velocity/acceleration: one loop over the requested rows,
    // querying only the getter the column needs.
    bool gatherVelAccColumnOvStage(bool wantAngular,
                                   bool wantAcceleration,
                                   const ::physx::PxU32* outRecordIdx,
                                   ::physx::PxU32 numOutputs,
                                   const TensorDesc* dstTensor) const;
    void prepareDirtyForceTracker();

    CpuSimulationDataPtr mCpuSimData;

    CpuRigidBodyDirtyForceTrackerPtr mDirtyForceTracker;

    // for bodies that are root articulation links, we use the articulation cache to set transforms and velocities
    std::vector<::physx::PxArticulationReducedCoordinate*> mArticulations;
    std::vector<::physx::PxArticulationCache*> mArticulationCaches;

    // Note: mAllIndices is inherited from BaseRigidBodyView (protected), initialized there.
};

} // namespace tensors
} // namespace physx
} // namespace omni

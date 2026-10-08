// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "tensors/ArticulationMetatype.h"
#include "tensors/CommonTypes.h"
#include "tensors/ArticulationDofOvStageRecord.h"
#include "tensors/ArticulationLinkOvStageRecord.h"
#include "tensors/ArticulationTendonOvStageRecord.h"
#include "tensors/base/BaseArticulationView.h"
#include "tensors/cpu/CpuSimulationData.h"

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

    // Internal output-read gathers. OvxPhysicsRead validates the row mapping once before dispatching
    // any columns; these keep their tensor checks but trust that shared row contract.
    //
    // Every requested root-state column in one pass: the four columns come from two host fetches, so
    // the single-column entry points below are one-column cases of this.
    bool getRootStateColumnsOvStage(const RootStateColumn* columns,
                                    ::physx::PxU32 numColumns,
                                    const ::physx::PxU32* rows,
                                    ::physx::PxU32 count) const;

    bool getPositionsOvStage(const TensorDesc* dstTensor,
                             const ::physx::PxU32* rows,
                             ::physx::PxU32 count,
                             uint64_t rowsToken) const;
    bool getOrientationsOvStage(const TensorDesc* dstTensor,
                                const ::physx::PxU32* rows,
                                ::physx::PxU32 count,
                                uint64_t rowsToken) const;
    bool getLinearVelocitiesOvStage(const TensorDesc* dstTensor,
                                    const ::physx::PxU32* rows,
                                    ::physx::PxU32 count,
                                    uint64_t rowsToken) const;
    bool getAngularVelocitiesOvStage(const TensorDesc* dstTensor,
                                     const ::physx::PxU32* rows,
                                     ::physx::PxU32 count,
                                     uint64_t rowsToken) const;
    bool getMassCentersWorldOvStage(const TensorDesc* dstTensor,
                                    const ::physx::PxU32* rows,
                                    ::physx::PxU32 count,
                                    uint64_t rowsToken) const;
    bool getMassCentersLocalOvStage(const TensorDesc* dstTensor,
                                    const ::physx::PxU32* rows,
                                    ::physx::PxU32 count,
                                    uint64_t rowsToken) const;

    // Inverse dynamics columns (REQ-READ-INVDYN-001). Unlike the getters above, the caller supplies the
    // column width in dstTensor->dims[1]: it is a function of the selected articulations' topology,
    // not of the attribute. A width that does not match what the selected rows need is rejected.
    bool getJacobiansOvStage(const TensorDesc* dstTensor,
                             const ::physx::PxU32* rows,
                             ::physx::PxU32 count,
                             uint64_t rowsToken) const;
    bool getMassMatricesOvStage(const TensorDesc* dstTensor,
                                const ::physx::PxU32* rows,
                                ::physx::PxU32 count,
                                uint64_t rowsToken) const;
    bool getCoriolisForcesOvStage(const TensorDesc* dstTensor,
                                  const ::physx::PxU32* rows,
                                  ::physx::PxU32 count,
                                  uint64_t rowsToken) const;
    bool getGravityForcesOvStage(const TensorDesc* dstTensor,
                                 const ::physx::PxU32* rows,
                                 ::physx::PxU32 count,
                                 uint64_t rowsToken) const;
    bool getCentroidalMomentaOvStage(const TensorDesc* dstTensor,
                                     const ::physx::PxU32* rows,
                                     ::physx::PxU32 count,
                                     uint64_t rowsToken) const;

    bool setRootTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setRootVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    bool getDofPositions(const TensorDesc* dstTensor) const override;
    bool getDofVelocities(const TensorDesc* dstTensor) const override;

    // Every requested DOF STATE column, refreshed in ONE pass over the articulations the records name:
    // PxArticulationCacheFlags is a flag SET, so a single copyInternalStateToCache carrying the union
    // fills all of them, and narrowing the pass to the named articulations keeps a two-row read from
    // refreshing a scene's worth. CPU only -- each DirectGPU state column is its own
    // PxArticulationGPUAPIReadType, so there is no shared fetch for a set form to collapse.
    bool getDofStateColumnsOvStage(const DofStateColumn* columns,
                                   ::physx::PxU32 numColumns,
                                   const ArticulationDofOvStageRecord* records,
                                   ::physx::PxU32 numOutputs) const;

    // ovstage joint-state read (ADR-0008): gather one scalar per (joint-prim, enabled-axis) into a
    // flat host column of `numOutputs` float32 elements (records built by the ovphysx reader).
    // CPU counterpart of GpuArticulationView::getDofPositionsOvStage.
    bool getDofPositionsOvStage(const ArticulationDofOvStageRecord* records,
                                ::physx::PxU32 numOutputs, const TensorDesc* dstTensor) const;
    bool getDofVelocitiesOvStage(const ArticulationDofOvStageRecord* records,
                                 ::physx::PxU32 numOutputs, const TensorDesc* dstTensor) const;
    bool getDofPositionTargetsOvStage(const ArticulationDofOvStageRecord* records,
                                      ::physx::PxU32 numOutputs, const TensorDesc* dstTensor) const;
    bool getDofVelocityTargetsOvStage(const ArticulationDofOvStageRecord* records,
                                      ::physx::PxU32 numOutputs, const TensorDesc* dstTensor) const;
    bool getDofActuationForcesOvStage(const ArticulationDofOvStageRecord* records,
                                      ::physx::PxU32 numOutputs, const TensorDesc* dstTensor) const;
    // Not a DOF-buffer column like the five above: the solver reaction is the link incoming joint force
    // projected onto the axis, which the dense getDofProjectedJointForces already does (joint frame and
    // body order included). This gathers that dense result rather than re-deriving the projection.
    bool getDofProjectedForcesOvStage(const ArticulationDofOvStageRecord* records,
                                      ::physx::PxU32 numOutputs, const TensorDesc* dstTensor) const;
    // Per-LINK ovstage read: the inbound joint's spatial force in the joint frame USD authored, 6
    // floats (force, torque) per record. Gathered from the same dense getLinkIncomingJointForce.
    bool getLinkIncomingJointForcesOvStage(const ArticulationLinkOvStageRecord* records,
                                           ::physx::PxU32 numOutputs, const TensorDesc* dstTensor) const;

    // CPU counterpart of GpuArticulationView::setDofPositionsOvStage. Read-modify-write for the same
    // reason the device path is: applyCache writes a whole articulation's DOF block, so a DOF this
    // call does not address must already hold its current value.
    bool setDofPositionsOvStage(const TensorDesc* srcTensor,
                                const ArticulationDofOvStageRecord* records,
                                ::physx::PxU32 numOutputs);
    bool setDofVelocitiesOvStage(const TensorDesc* srcTensor,
                                 const ArticulationDofOvStageRecord* records,
                                 ::physx::PxU32 numOutputs);
    bool setDofPositionTargetsOvStage(const TensorDesc* srcTensor,
                                      const ArticulationDofOvStageRecord* records,
                                      ::physx::PxU32 numOutputs);
    bool setDofVelocityTargetsOvStage(const TensorDesc* srcTensor,
                                      const ArticulationDofOvStageRecord* records,
                                      ::physx::PxU32 numOutputs);
    bool setDofActuationForcesOvStage(const TensorDesc* srcTensor,
                                      const ArticulationDofOvStageRecord* records,
                                      ::physx::PxU32 numOutputs);

    // CPU counterpart of GpuArticulationView::setRootAttributeOvStage. `rows` names the view
    // articulations the query matched; output i addresses view articulation rows[i].
    //
    // No read-modify-write scratch is needed here and none is skipped: PxArticulationCache is filled
    // by copyInternalStateToCache before the overlay, so the component this write does not address is
    // already the articulation's current one. The GPU path needs the explicit DirectGPU pre-read to
    // reach the same state.
    //
    // `rowsToken` is accepted and unused -- the CPU path has no device copy to gate -- so the two
    // backends stay callable through one row in the write table.
    bool setRootAttributeOvStage(const char* attribName,
                                 const TensorDesc* srcTensor,
                                 const ::physx::PxU32* rows,
                                 ::physx::PxU32 numOutputs,
                                 uint64_t rowsToken,
                                 bool angular, // see GpuArticulationView::setRootAttributeOvStage
                                 bool pose);

    bool setRootPositionsOvStage(const TensorDesc* srcTensor,
                                 const ::physx::PxU32* rows,
                                 ::physx::PxU32 numOutputs,
                                 uint64_t rowsToken);
    bool setRootOrientationsOvStage(const TensorDesc* srcTensor,
                                    const ::physx::PxU32* rows,
                                    ::physx::PxU32 numOutputs,
                                    uint64_t rowsToken);
    bool setRootLinearVelocitiesOvStage(const TensorDesc* srcTensor,
                                        const ::physx::PxU32* rows,
                                        ::physx::PxU32 numOutputs,
                                        uint64_t rowsToken);
    bool setRootAngularVelocitiesOvStage(const TensorDesc* srcTensor,
                                         const ::physx::PxU32* rows,
                                         ::physx::PxU32 numOutputs,
                                         uint64_t rowsToken);


    // ovstage tendon read: gather one property per tendon prim into a flat host column of
    // `numOutputs * comp` float32 elements. CPU counterpart of
    // GpuArticulationView::getFixedTendonPropertiesOvStage.
    bool getFixedTendonPropertiesOvStage(const ArticulationTendonOvStageRecord* records,
                                         ::physx::PxU32 numOutputs, TendonProperty prop,
                                         const TensorDesc* dstTensor) const;
    bool getSpatialTendonPropertiesOvStage(const ArticulationTendonOvStageRecord* records,
                                           ::physx::PxU32 numOutputs, TendonProperty prop,
                                           const TensorDesc* dstTensor) const;

    // CPU counterpart of GpuArticulationView::set{Fixed,Spatial}TendonPropertiesOvStage.
    //
    // No read-modify-write scratch is needed here and none is skipped: these are individual PhysX
    // setters on the tendon object, so a property this call does not address is untouched by
    // construction. The GPU path needs its explicit pre-read only because PhysX moves whole structs.
    bool setFixedTendonPropertiesOvStage(const TensorDesc* srcTensor,
                                         const ArticulationTendonOvStageRecord* records,
                                         ::physx::PxU32 numOutputs, TendonProperty prop);
    bool setSpatialTendonPropertiesOvStage(const TensorDesc* srcTensor,
                                           const ArticulationTendonOvStageRecord* records,
                                           ::physx::PxU32 numOutputs, TendonProperty prop);

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
    // setDisableGravities/material/rest/contact/compliant Masked: BaseArticulationView
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
    // Coriolis and gravity differ only in which compute call fills the cache and which array it
    // fills; everything else, including the per-dof sign flip, is shared.
    bool getGeneralizedForceColumnOvStage(const TensorDesc* dstTensor,
                                          const ::physx::PxU32* rows,
                                          ::physx::PxU32 count,
                                          bool gravity) const;

    bool getMassCentersOvStage(const TensorDesc* dstTensor,
                               const ::physx::PxU32* rows,
                               ::physx::PxU32 count,
                               bool localFrame) const;

    // One column's values, off state getDofStateColumnsOvStage has already refreshed. Split out so the
    // refresh cannot be run per column by accident: the only caller is that set form's loop.
    bool gatherDofStateColumn(const DofStateColumn& column,
                              const ArticulationDofOvStageRecord* records,
                              ::physx::PxU32 numOutputs) const;

    // Shared body of the ovstage DOF read: refresh whatever `quantity` needs, then gather
    // dst[i] = dofScaleFor(records[i], policy) * raw(quantity, viewArtiIdx, physxDofIdx).
    bool getDofAttributeOvStage(const TensorDesc* dstTensor, DofStateQuantity quantity,
                                const ArticulationDofOvStageRecord* records, ::physx::PxU32 numOutputs) const;
    // The ovstage DOF WRITE: the inverse of the gather above, taking the same `quantity` so the two
    // cannot disagree about which cache backs a column or which folds it carries -- both read the
    // facts table on BaseArticulationView rather than being told.
    //
    // `quantity` also decides the MECHANISM, not just the field: the two drive targets are per-axis
    // joint setters, while state and actuation force go through the cache and need a
    // read-modify-write, because applyCache pushes a whole articulation's block.
    bool setDofAttributeOvStage(const TensorDesc* srcTensor, DofStateQuantity quantity,
                                const ArticulationDofOvStageRecord* records, ::physx::PxU32 numOutputs);

    // Shared body of the ovstage tendon read. `fixed` picks which per-entry tendon list the records
    // index and which getters the property resolves to.
    bool getTendonPropertiesOvStage(const char* attribName, const TensorDesc* dstTensor, bool fixed,
                                    TendonProperty prop, const ArticulationTendonOvStageRecord* records,
                                    ::physx::PxU32 numOutputs) const;
    bool setTendonPropertiesOvStage(const char* attribName, const TensorDesc* srcTensor, bool fixed,
                                    TendonProperty prop, const ArticulationTendonOvStageRecord* records,
                                    ::physx::PxU32 numOutputs);

    void prepareDirtyForceTracker();
    CpuRigidBodyDirtyForceTrackerPtr mDirtyForceTracker;
    std::vector<::physx::PxArticulationCache*> mArticulationCaches;
    // Per-read scratch, sized once and reused. `mutable` because the reads are const: these are
    // scratch, not state, and nothing observable depends on what is left in them between reads. Not
    // thread safe, like the view itself -- one ovphysx instance serialises its own reads.
    mutable std::vector<uint8_t> mDofRefreshMarks; // all zero between reads; see getDofStateColumnsOvStage
    mutable std::vector<::physx::PxU32> mDofRefreshList; // the distinct entries, in first-seen order
    mutable std::vector<float> mDofStagingRow; // whole-view dense DOF row: getDofProjectedForcesOvStage
    mutable std::vector<float> mLinkStagingRow; // whole-view dense link row: getLinkIncomingJointForcesOvStage
    CpuSimulationDataPtr mCpuSimData;
    // Note: mAllIndices is inherited from BaseArticulationView (protected), initialized there.
};

} // namespace tensors
} // namespace physx
} // namespace omni

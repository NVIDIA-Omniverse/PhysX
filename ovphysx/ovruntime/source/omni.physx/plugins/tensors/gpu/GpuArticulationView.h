// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "tensors/ArticulationMetatype.h"
#include "tensors/CommonTypes.h"
#include "tensors/ArticulationDofOvStageRecord.h"
#include "tensors/ArticulationLinkOvStageRecord.h"
#include "tensors/ArticulationTendonOvStageRecord.h"
#include "tensors/base/BaseArticulationView.h"
#include "tensors/gpu/GpuSimulationData.h"
#include "PxDirectGPUAPI.h"

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class GpuSimulationView;

class GpuArticulationView : public BaseArticulationView
{
public:
    GpuArticulationView(GpuSimulationView* sim, const std::vector<ArticulationEntry>& entries, int device);

    ~GpuArticulationView() override;

    //
    // public API
    //

    bool getLinkTransforms(const TensorDesc* dstTensor) const override;
    bool getLinkVelocities(const TensorDesc* dstTensor) const override;
    bool getLinkAccelerations(const TensorDesc* dstTensor) const override;

    bool getRootTransforms(const TensorDesc* dstTensor) const override;
    bool getRootVelocities(const TensorDesc* dstTensor) const override;

    // Every requested root-state column, with each DirectGPU fetch issued once -- three fetches for
    // four columns, since only the pose pair shares one. The single-column entry points below are
    // one-column cases of this.
    bool getRootStateColumnsOvStage(const RootStateColumn* columns,
                                    ::physx::PxU32 numColumns,
                                    const ::physx::PxU32* rows,
                                    ::physx::PxU32 count,
                                    uint64_t rowsToken) const;

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

    // Inverse dynamics columns (REQ-READ-INVDYN-001). The caller supplies the cohort's flattened width in
    // dstTensor->dims[1]; BaseArticulationView::checkInverseDynamicsColumn confirms every selected row agrees.
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

    // Cached view-owned upload of the reader's per-read ovstage record list,
    // replacing the per-read memAlloc + memcpyHtoD the reader used to do itself. Returns the device
    // pointer to `count` records, re-uploading only when `token` or `count` changes; null on
    // allocation or copy failure. `token` must identify the list content -- the reader passes its
    // record CONTENT version, which its own cached record vector is gated on, so the two turn over
    // together. DOF, fixed-tendon and spatial-tendon lists each have their OWN buffer, so an
    // alternating read of any two cannot clobber the other's still-in-flight gather, and equal-count
    // fixed/spatial lists cannot alias. The content version changes whenever the host record vector
    // rebuilds -- including a same-cardinality rebuild that keeps the view generation -- so keying on
    // generation would serve a stale buffer, and this token never does (see the callers).
    const void* ovStageDofRecordsDevice(const void* records, ::physx::PxU32 count, uint64_t token) const;
    const void* ovStageTendonRecordsDevice(const void* records, ::physx::PxU32 count, uint64_t token,
                                           bool fixed) const;
    const void* ovStageLinkForceRecordsDevice(const void* records, ::physx::PxU32 count, uint64_t token) const;

    bool setRootTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setRootVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    bool getDofPositions(const TensorDesc* dstTensor) const override;
    bool getDofVelocities(const TensorDesc* dstTensor) const override;

    // ovstage joint-state read (ADR-0008): gather one scalar per (joint-prim, enabled-axis) into a flat,
    // device-resident column of `numOutputs` float32 elements. The records (source DOF plus the per-axis
    // unit and sign facts a fold needs) are built by the ovphysx reader. Required on GPU sims, where the
    // host getArticulationJointPosition/Velocity is stale under suppressReadback.
    //
    // One method per attribute so the unit/sign fold is fixed here rather than chosen by the caller.
    bool getDofPositionsOvStage(const ArticulationDofOvStageRecord* recordsDev,
                                ::physx::PxU32 numOutputs, const TensorDesc* dstTensor) const;
    bool getDofVelocitiesOvStage(const ArticulationDofOvStageRecord* recordsDev,
                                 ::physx::PxU32 numOutputs, const TensorDesc* dstTensor) const;
    bool getDofPositionTargetsOvStage(const ArticulationDofOvStageRecord* recordsDev,
                                      ::physx::PxU32 numOutputs, const TensorDesc* dstTensor) const;
    bool getDofVelocityTargetsOvStage(const ArticulationDofOvStageRecord* recordsDev,
                                      ::physx::PxU32 numOutputs, const TensorDesc* dstTensor) const;
    bool getDofActuationForcesOvStage(const ArticulationDofOvStageRecord* recordsDev,
                                      ::physx::PxU32 numOutputs, const TensorDesc* dstTensor) const;
    // Not a DOF-buffer column: the reaction comes from the link incoming joint force, projected onto
    // the axis by the same kernel the dense getDofProjectedJointForces runs, then gathered.
    bool getDofProjectedForcesOvStage(const ArticulationDofOvStageRecord* recordsDev,
                                      ::physx::PxU32 numOutputs, const TensorDesc* dstTensor) const;
    // Per-LINK ovstage read: the inbound joint's spatial force in the joint frame USD authored, 6
    // floats (force, torque) per record. Staged and gathered like the projected force above.
    bool getLinkIncomingJointForcesOvStage(const ArticulationLinkOvStageRecord* recordsDev,
                                           ::physx::PxU32 numOutputs, const TensorDesc* dstTensor) const;

    // ovstage tendon read: gather one property per tendon prim into a flat, device-resident column of
    // `numOutputs * comp` float32 elements. One entry point per tendon KIND rather than per attribute:
    // every tendon attribute comes out of the same DirectGPU struct at a different float offset.
    bool getFixedTendonPropertiesOvStage(const ArticulationTendonOvStageRecord* recordsDev,
                                         ::physx::PxU32 numOutputs, TendonProperty prop,
                                         const TensorDesc* dstTensor) const;
    bool getSpatialTendonPropertiesOvStage(const ArticulationTendonOvStageRecord* recordsDev,
                                           ::physx::PxU32 numOutputs, TendonProperty prop,
                                           const TensorDesc* dstTensor) const;

    // ovstage tendon-property WRITE (ADR-0012): the return direction of the two getters
    // above, sharing their record list, their scratch and their struct-offset addressing.
    //
    // Reads the view's current tendon block first. Not optional: PhysX writes whole tendon structs,
    // so a property this call does not address must already hold its current value or the write
    // would zero its neighbours in the same struct.
    bool setFixedTendonPropertiesOvStage(const TensorDesc* srcTensor,
                                         const ArticulationTendonOvStageRecord* recordsDev,
                                         ::physx::PxU32 numOutputs, TendonProperty prop);
    bool setSpatialTendonPropertiesOvStage(const TensorDesc* srcTensor,
                                           const ArticulationTendonOvStageRecord* recordsDev,
                                           ::physx::PxU32 numOutputs, TendonProperty prop);
    // ovstage joint-DOF WRITE (ADR-0012): the return direction of getDofAttributeOvStage, sharing its
    // record list and its scratch, and carrying no host block.
    //
    bool setDofPositionsOvStage(const TensorDesc* srcTensor,
                                const ArticulationDofOvStageRecord* recordsDev,
                                ::physx::PxU32 numOutputs);
    bool setDofVelocitiesOvStage(const TensorDesc* srcTensor,
                                 const ArticulationDofOvStageRecord* recordsDev,
                                 ::physx::PxU32 numOutputs);
    bool setDofPositionTargetsOvStage(const TensorDesc* srcTensor,
                                      const ArticulationDofOvStageRecord* recordsDev,
                                      ::physx::PxU32 numOutputs);
    bool setDofVelocityTargetsOvStage(const TensorDesc* srcTensor,
                                      const ArticulationDofOvStageRecord* recordsDev,
                                      ::physx::PxU32 numOutputs);
    bool setDofActuationForcesOvStage(const TensorDesc* srcTensor,
                                      const ArticulationDofOvStageRecord* recordsDev,
                                      ::physx::PxU32 numOutputs);

    // ovstage articulation-ROOT WRITE (ADR-0012). `rows` names the view articulations the
    // query matched, output i addressing view articulation rows[i]; the column carries `comp` floats
    // per output (3 for a position or a velocity, 4 for an orientation).
    //
    // Same read-modify-write, same whole-view push and the same event ordering as
    // setDofAttributeOvStage -- see there for why each is not optional. Root pose carries one extra
    // reason for the RMW: PhysX has a single eROOT_GLOBAL_POSE covering position AND orientation, and
    // a session carries one attribute.
    //
    // `rowsToken` identifies the row list's CONTENTS so the device copy is re-uploaded exactly when
    // they change -- the same contract as GpuRigidBodyView::ovStageRowsDevice.
    bool setRootAttributeOvStage(const char* attribName,
                                 const TensorDesc* srcTensor,
                                 const ::physx::PxU32* rows,
                                 ::physx::PxU32 numOutputs,
                                 uint64_t rowsToken,
                                 // `angular` selects the ROTATIONAL half of the pair the attribute
                                 // belongs to -- orientation within a pose, angular velocity within a
                                 // velocity. It does NOT mean "four components": an angular velocity
                                 // is a vec3, and conflating the two is a size mismatch at the tensor
                                 // check rather than a wrong value.
                                 bool angular,
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

    // OMPE-103213: material/rest/contact/disable-gravity are CPU-only PhysX
    // APIs. No GPU overrides -- BaseArticulationView requires host tensors
    // (including Masked wrappers).

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

private:
    bool resolveMask(const TensorDesc* maskTensor, ::physx::PxU32& outK) const;

    // CALLER CONTRACT: `rowsToken` must change whenever `rows` does. It is the whole identity of the
    // selection -- a cache hit reuses the uploaded GPU indices and revalidates nothing, so a token
    // replayed over a different row list gathers the previous selection's rows and reports success.
    bool ensureOvStageSelection(const ::physx::PxU32* rows,
                                ::physx::PxU32 count,
                                uint64_t rowsToken,
                                const ::physx::PxArticulationGPUIndex*& gpuIndicesDev) const;
    bool recordOvStageReady(CUevent waitEvent) const;

    // Reads the view's current DOF scalars first. That is not an optimization to remove: PhysX writes
    // whole articulation DOF blocks, so a DOF this call does not address must already hold its
    // current value or the write would zero it.
    //
    // Every articulation in the view is pushed, not just those owning a written DOF, matching what
    // the legacy setDofAttribute does when given no index tensor. The untouched ones are written
    // their own current values, which is a no-op on their state.
    bool setDofAttributeOvStage(const char* attribName,
                                const TensorDesc* srcTensor,
                                ::physx::PxArticulationGPUAPIReadType::Enum readFlag,
                                ::physx::PxArticulationGPUAPIWriteType::Enum writeFlag,
                                const ArticulationDofOvStageRecord* recordsDev,
                                ::physx::PxU32 numOutputs,
                                // Indices into GpuSimulationData's CopyEvent and ApplyEvent arrays.
                                // Passed separately rather than as one value: the two enums happen to
                                // list these attributes in the same order today, and relying on that
                                // would break silently the first time either gains an entry.
                                int copyEvent,
                                int applyEvent,
                                // Which folds the scatter undoes. Not derivable from the write flag:
                                // the four state/target columns are eAngularSigned and the actuation
                                // force is eSigned, and every one of them is a JOINT_* write type.
                                DofScalePolicy policy);

    // Device copy of the ovstage row list, re-uploaded only when `token` differs from the one the
    // held copy was uploaded under. Owned by this view -- the caller must not free it -- for the same
    // reason GpuRigidBodyView owns its copy: this class already frees its device memory under the
    // right CUDA context in its destructor, where a caller-side buffer would have to free through a
    // context manager that may be gone by then.
    const ::physx::PxU32* ovStageRowsDevice(const ::physx::PxU32* rows, uint32_t count, uint64_t token) const;
    bool waitForDirectGpuFinish(CUevent finishEvent, const char* label) const;
    // False when the scene's shared inverse dynamics scratch was never allocated, which happens when no
    // articulation in it has a degree of freedom. Submitting a null buffer aborts the process.
    bool checkInverseDynamicsScratch(const void* scratch, const char* label, const char* funcName) const;

    // Coriolis and gravity differ only in compute type and copy event.
    bool getGeneralizedForceColumnOvStage(const TensorDesc* dstTensor,
                                          const ::physx::PxU32* rows,
                                          ::physx::PxU32 count,
                                          uint64_t rowsToken,
                                          bool gravity) const;

    bool getMassCentersOvStage(const TensorDesc* dstTensor,
                               const ::physx::PxU32* rows,
                               ::physx::PxU32 count,
                               uint64_t rowsToken,
                               bool localFrame) const;

    bool getDofAttribute(const char* attribName,
                         const TensorDesc* dstTensor,
                         const ::physx::PxArticulationGPUAPIReadType::Enum attribFlag,
                         CUevent syncEvent) const;

    // ovstage variant: one DirectGPU DOF fill into the scratch, then a flat scaled gather into
    // `numOutputs` output slots via the caller-built records (see fetchArtiDofAttributeOvStage).
    // `policy` is the unit/sign fold the attribute applies to the raw DOF scalar.
    bool getDofAttributeOvStage(const char* attribName,
                                const TensorDesc* dstTensor,
                                const ::physx::PxArticulationGPUAPIReadType::Enum attribFlag,
                                const DofScalePolicy policy,
                                const ArticulationDofOvStageRecord* recordsDev,
                                ::physx::PxU32 numOutputs,
                                CUevent syncEvent) const;

    // ovstage tendon variant: one DirectGPU tendon fill into the shared scratch, then a flat gather
    // into `numOutputs * comp` output slots (see fetchArtiTendonPropertyOvStage). `structFloats`
    // and `buffer` are what separate the fixed and spatial kinds.
    bool getTendonPropertyOvStage(const char* attribName,
                                  const TensorDesc* dstTensor,
                                  const ::physx::PxArticulationGPUAPIReadType::Enum attribFlag,
                                  ::physx::PxU32 buffer,
                                  void* scratchDev,
                                  ::physx::PxU32 structFloats,
                                  ::physx::PxU32 maxTendons,
                                  TendonProperty prop,
                                  const ArticulationTendonOvStageRecord* recordsDev,
                                  ::physx::PxU32 numOutputs,
                                  CUevent syncEvent) const;

    // Generic ovstage tendon write the two setters above pass their own engine values to, mirroring
    // getTendonPropertyOvStage exactly: same buffer, same structFloats, same field offset.
    bool setTendonPropertyOvStage(const char* attribName,
                                  const TensorDesc* srcTensor,
                                  ::physx::PxArticulationGPUAPIReadType::Enum readFlag,
                                  ::physx::PxArticulationGPUAPIWriteType::Enum writeFlag,
                                  ::physx::PxU32 buffer,
                                  void* scratchDev,
                                  ::physx::PxU32 structFloats,
                                  ::physx::PxU32 maxTendons,
                                  TendonProperty prop,
                                  const ArticulationTendonOvStageRecord* recordsDev,
                                  ::physx::PxU32 numOutputs);

    // generic helper function to submit a DOF attribute tensor
    bool setDofAttribute(const char* attribName,
                         const TensorDesc* srcTensor,
                         const TensorDesc* indexTensor,
                         const ::physx::PxArticulationGPUAPIWriteType::Enum attribFlag,
                         CUevent mApplyWaitEvents,
                         CUevent mApplySignalEvents);
    bool updateCMassData();
    bool prepareJacobianBuffers();
    bool prepareGeneralizedGravityBuffers();

    int mDevice = -1;
    ::physx::PxU32 mLinkBufSize = 0;
    ::physx::PxU32 mDofBufSize = 0;
    ::physx::PxU32 mFixedTendonBufSize = 0;
    ::physx::PxU32 mSpatialTendonBufSize = 0;
    GpuSimulationDataPtr mGpuSimData;

    std::vector<::physx::PxArticulationGPUIndex> mArtiIndices;

    // One cached, grow-only device allocation serves every whole-articulation ovstage gather. The
    // upload is synchronous, so the staging vector is free the moment the copy returns and needs no
    // lifetime tracking of its own. The producer mints a new token whenever the selection changes.
    mutable void* mOvStageSelectionDev = nullptr;
    mutable ::physx::PxArticulationGPUIndex* mOvStageGpuIndicesDev = nullptr;
    mutable std::vector<::physx::PxArticulationGPUIndex> mOvStageSelectionHost;
    mutable ::physx::PxU32 mOvStageSelectionCapacity = 0;
    mutable ::physx::PxU32 mOvStageSelectionCount = 0;
    mutable uint64_t mOvStageSelectionToken = 0;
    mutable bool mOvStageSelectionValid = false;
    // Each DirectGPU submission enqueues its wait on this start event before returning. Only after
    // that call returns may the next column re-record the shared event for its own submission.
    CUevent mOvStageSelectionReadyEvent = nullptr;

    // The per-read ovstage record lists -- DOF, tendon and link-force -- cached view-owned instead of
    // allocated and uploaded by the reader every read. Each list is keyed on the reader's record
    // CONTENT version token (recsVersion, minted by mintRecordContentVersion), NOT on view generation:
    // a same-cardinality record rebuild remints the content version WITHOUT a new generation, so
    // keying on generation would serve a stale list. The upload turns over only when that token or the
    // count changes (uploadOvStageRecordsCached), in lockstep with the reader's own record cache; a
    // token of 0 -- a fresh view -- forces the first upload. Byte-sized because the record types
    // differ only in element size; the reader casts the returned pointer.
    mutable void* mOvStageDofRecordsDev = nullptr;
    mutable size_t mOvStageDofRecordsCapBytes = 0;
    mutable uint32_t mOvStageDofRecordsCount = 0;
    mutable uint64_t mOvStageDofRecordsToken = 0;
    // Fixed and spatial tendon record lists get SEPARATE device buffers. They share the record
    // struct and can have equal cardinality, so one buffer keyed on (token, count) served the second
    // kind through the first kind's records on an equal-count read -- wrong values or an out-of-range
    // gather (the aliasing this split fixes).
    mutable void* mOvStageFixedTendonRecordsDev = nullptr;
    mutable size_t mOvStageFixedTendonRecordsCapBytes = 0;
    mutable uint32_t mOvStageFixedTendonRecordsCount = 0;
    mutable uint64_t mOvStageFixedTendonRecordsToken = 0;
    mutable void* mOvStageSpatialTendonRecordsDev = nullptr;
    mutable size_t mOvStageSpatialTendonRecordsCapBytes = 0;
    mutable uint32_t mOvStageSpatialTendonRecordsCount = 0;
    mutable uint64_t mOvStageSpatialTendonRecordsToken = 0;
    // Link incoming-joint-force record list: its OWN view-owned buffer, same rationale as DOF and
    // tendon. Was a per-read memAlloc handed to the session, which returned an undersized record block
    // to the output-column pool it never acquired from.
    mutable void* mOvStageLinkForceRecordsDev = nullptr;
    mutable size_t mOvStageLinkForceRecordsCapBytes = 0;
    mutable uint32_t mOvStageLinkForceRecordsCount = 0;
    mutable uint64_t mOvStageLinkForceRecordsToken = 0;

    ::physx::PxU32* mViewIndicesDev = nullptr; // view indices, the client can send a subset of these
    ::physx::PxArticulationGPUIndex* mArtiGpuIndicesDev = nullptr; // Gpu indices of all view articulations
    ::physx::PxArticulationGPUIndex* mDirtyArtiGpuIndicesDev = nullptr; // buffer for dirty GPU indices

    GpuArticulationRootRecord* mRootRecordsDev = nullptr;

    // The ovstage write's row list, held across writes and gated on the caller's token. Mutable and
    // const-accessed for the same reason the rigid view's copy is: uploading a device cache is not a
    // change to the view's observable state.
    mutable ::physx::PxU32* mOvStageRowsDev = nullptr;
    mutable uint32_t mOvStageRowsCapacity = 0;
    mutable uint32_t mOvStageRowsCount = 0;
    mutable uint64_t mOvStageRowsToken = 0;
    GpuArticulationLinkRecord* mLinkRecordsDev = nullptr;
    GpuArticulationDofRecord* mDofRecordsDev = nullptr;
    GpuArticulationFixedTendonRecord* mFixedTendonRecordsDev = nullptr;
    GpuArticulationSpatialTendonRecord* mSpatialTendonRecordsDev = nullptr;

    std::vector<::physx::PxVec3> cMassLocalPosePos;
    ::physx::PxVec3* cMassLocalPosePosDev = nullptr; // coms for links

    // Mask -> indices scratch (cached). Declared mutable because resolveMask()
    // is const - some interface setters (e.g. material/shape properties) are
    // const by pre-existing TensorAPI convention ("const" = view object unchanged,
    // simulation state may be mutated via pointer indirection). These buffers are
    // internal caching state, not logical view state.
    mutable ::physx::PxU32* mMaskIndicesDev = nullptr;
    mutable ::physx::PxU32 mMaskIndicesCapacity = 0;
    mutable SingleAllocPolicy mMaskAllocPolicy;

    // Dense staging row for the ovstage link incoming-joint-force read, which is DERIVED: a
    // link-parallel kernel resolves the joint frame into a [count x maxLinks x 6] row that the ovstage
    // gather reorders. It cannot reuse a shared GpuSimulationData buffer because the dense pass
    // already reads two of those as its source. (The projected DOF force stages through the shared
    // mDofScalarsDev instead, which its dense pass does not read.)
    //
    // Allocated on first use -- the size is fixed for the view's life but most views never read this.
    // Mutable for the same reason mMaskIndicesDev is: internal scratch behind a const getter.
    mutable float* mLinkIncomingJointForceScratchDev = nullptr;
};

} // namespace tensors
} // namespace physx
} // namespace omni

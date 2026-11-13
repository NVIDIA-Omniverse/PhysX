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
#include "tensors/gpu/OvStageDeviceArgs.h"
#include "tensors/gpu/OvStageReadPlans.h"
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

    // The per-articulation device columns an ovstage read gathers from this view (ADR-0055): root state and centre
    // of mass over any rows, inverse dynamics over the rows of one cohort (REQ-READ-INVDYN-001). The root pose
    // serves both pose columns; every other column has its own PhysX copy or compute.
    enum class OvStageArtiRead : uint8_t
    {
        eNone,
        eRootPosition,
        eRootOrientation,
        eRootLinearVelocity,
        eRootAngularVelocity,
        eMassCenterWorld,
        eMassCenterLocal,
        eJacobian,
        eMassMatrix,
        eCoriolisForce,
        eGravityForce,
        eCentroidalMomentum
    };
    struct OvStageArtiReadColumn
    {
        OvStageArtiRead kind = OvStageArtiRead::eNone;
        // [count, width] float32 on this view's device: 3 wide, 4 for the root orientation, or the cohort's
        // flattened width for an inverse dynamics column.
        float* dst = nullptr;
        ::physx::PxU32 width = 0;
    };
    struct OvStageArtiReadPlan
    {
        ::physx::PxU32 count = 0;
        // The view entry of the first row, whose topology every row of an inverse dynamics read shares.
        ::physx::PxU32 firstEntryIndex = 0;
        uint32_t sources = 0;
    };

    // Plans a read of `columns` (their kinds and widths; destinations may still be unset) over `rows`: the plan,
    // the bytes of device scratch it runs in, and the list that heads that scratch -- the rows' PhysX GPU
    // indices -- which the caller uploads before the read whenever its content changed. A read with an inverse
    // dynamics column must name one cohort's rows. A nonzero `rowsToken` names the content of the row list `rows`
    // derive from (nextOvStageRowsVersion), and `rowSet` which of its derived lists they are: a plan this view
    // already made for them and the same columns is reused without walking the rows.
    bool planOvStageRead(const ::physx::PxU32* rows,
                         ::physx::PxU32 count,
                         uint64_t rowsToken,
                         uint64_t rowSet,
                         const OvStageArtiReadColumn* columns,
                         ::physx::PxU32 columnCount,
                         OvStageArtiReadPlan& plan,
                         std::shared_ptr<const std::vector<uint8_t>>& head,
                         size_t& scratchBytes) const;

    // Queues a planned read on `args.stream`: one PhysX copy or compute per source into `args.scratch`, then one
    // gather per column. It blocks nothing, allocates nothing and uploads nothing, so a capture records it.
    // Outside a capture it then registers `args.stream` as a reader of this view's tables.
    bool readOvStageColumns(const OvStageDeviceReadArgs& args,
                            const OvStageArtiReadPlan& plan,
                            const OvStageArtiReadColumn* columns,
                            ::physx::PxU32 columnCount) const;

    // The per-record device columns an ovstage joint, link or tendon read gathers from this view (ADR-0055). Their
    // records may name any articulation of the view, so each source is one PhysX copy, of the whole view or of the
    // articulations a DOF state read names, made once for every column that reads it.
    enum class OvStageRecordRead : uint8_t
    {
        eNone,
        // One value per ArticulationDofOvStageRecord.
        eDofPosition,
        eDofVelocity,
        eDofPositionTarget,
        eDofVelocityTarget,
        eDofActuationForce,
        // The reaction on the DOF's axis, projected from the link incoming joint force.
        eDofProjectedForce,
        // Six values per ArticulationLinkOvStageRecord: the inbound joint's spatial force in the joint frame USD
        // authored.
        eLinkIncomingJointForce,
        // tendonPropertyComponents(tendonProperty) values per ArticulationTendonOvStageRecord.
        eFixedTendon,
        eSpatialTendon
    };
    struct OvStageRecordReadColumn
    {
        OvStageRecordRead kind = OvStageRecordRead::eNone;
        TendonProperty tendonProperty = TendonProperty::eStiffness; // a tendon column's
        // float32 on this view's device, one record's values after another.
        float* dst = nullptr;
    };
    struct OvStageRecordReadPlan
    {
        ::physx::PxU32 numRecords = 0;
        uint32_t sources = 0;
        // How many articulations each copy fetches when the read fetches only its rows'; 0 for the whole view.
        ::physx::PxU32 numFetched = 0;
    };

    // Plans a read of `columns` (their kinds; destinations may still be unset) over `numRecords` records of the
    // one type the kinds take, `recordBytes` each: the plan and the bytes of device scratch it runs in. The records
    // head that scratch; the caller uploads them before the read whenever they changed.
    //
    // With `fetchRows`, each copy fetches only those `fetchCount` view rows' articulations, and a record's
    // viewArtiIdx names its articulation's position in `fetchRows` rather than its view row. `fetchHead` then
    // receives their PhysX GPU indices, which follow the records in the scratch head, at the next OvStageScratch
    // boundary. Only the DOF state kinds read this way: the others gather through a pass over the whole view.
    bool planOvStageRecordRead(::physx::PxU32 numRecords,
                               size_t recordBytes,
                               const OvStageRecordReadColumn* columns,
                               ::physx::PxU32 columnCount,
                               const ::physx::PxU32* fetchRows,
                               ::physx::PxU32 fetchCount,
                               OvStageRecordReadPlan& plan,
                               std::vector<uint8_t>& fetchHead,
                               size_t& scratchBytes) const;

    // Queues a planned record read on `args.stream`, as readOvStageColumns does.
    bool readOvStageRecordColumns(const OvStageDeviceReadArgs& args,
                                  const OvStageRecordReadPlan& plan,
                                  const OvStageRecordReadColumn* columns,
                                  ::physx::PxU32 columnCount) const;

    bool setRootTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setRootVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    bool getDofPositions(const TensorDesc* dstTensor) const override;
    bool getDofVelocities(const TensorDesc* dstTensor) const override;

    // ovstage tendon-property WRITE (ADR-0012): the return direction of a tendon column of
    // readOvStageRecordColumns above, sharing its record struct and struct-offset addressing.
    //
    // Reads the view's current tendon block first. Not optional: PhysX writes whole tendon structs,
    // so a property this call does not address must already hold its current value or the write
    // would zero its neighbours in the same struct.
    //
    // `mode` other than eSet is the ovstage drain's one-sided limit write: prop is eLimit and the source
    // carries one value per row (TendonFieldWrite).
    bool setFixedTendonPropertiesOvStage(const TensorDesc* srcTensor,
                                         const ArticulationTendonOvStageRecord* recordsDev,
                                         ::physx::PxU32 numOutputs, TendonProperty prop,
                                         const OvStageDeviceWriteArgs& deviceArgs,
                                         TendonFieldWrite mode = TendonFieldWrite::eSet);
    bool setSpatialTendonPropertiesOvStage(const TensorDesc* srcTensor,
                                           const ArticulationTendonOvStageRecord* recordsDev,
                                           ::physx::PxU32 numOutputs, TendonProperty prop,
                                           const OvStageDeviceWriteArgs& deviceArgs);
    // ovstage joint-DOF WRITE (ADR-0012): the return direction of the DOF record read, sharing its
    // record list, and carrying no host block.
    //
    bool setDofPositionsOvStage(const TensorDesc* srcTensor,
                                const ArticulationDofOvStageRecord* recordsDev,
                                ::physx::PxU32 numOutputs,
                                const OvStageDeviceWriteArgs& deviceArgs);
    bool setDofVelocitiesOvStage(const TensorDesc* srcTensor,
                                 const ArticulationDofOvStageRecord* recordsDev,
                                 ::physx::PxU32 numOutputs,
                                 const OvStageDeviceWriteArgs& deviceArgs);
    bool setDofPositionTargetsOvStage(const TensorDesc* srcTensor,
                                      const ArticulationDofOvStageRecord* recordsDev,
                                      ::physx::PxU32 numOutputs,
                                      const OvStageDeviceWriteArgs& deviceArgs);
    bool setDofVelocityTargetsOvStage(const TensorDesc* srcTensor,
                                      const ArticulationDofOvStageRecord* recordsDev,
                                      ::physx::PxU32 numOutputs,
                                      const OvStageDeviceWriteArgs& deviceArgs);
    bool setDofActuationForcesOvStage(const TensorDesc* srcTensor,
                                      const ArticulationDofOvStageRecord* recordsDev,
                                      ::physx::PxU32 numOutputs,
                                      const OvStageDeviceWriteArgs& deviceArgs);

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
                                 bool pose,
                                 const OvStageDeviceWriteArgs& deviceArgs);

    // Session writes (ADR-0052) run in device memory the session owns, sized by this for `kind` and the
    // group's output count. A root write's leading u32 array is its row list, uploaded at open. A joint or tendon
    // write has no leading list.
    enum class OvStageArtiWrite
    {
        eRootPose,
        eRootVelocity,
        eDof,
        eFixedTendon,
        eSpatialTendon
    };
    size_t ovStageWriteScratchBytes(OvStageArtiWrite kind, ::physx::PxU32 numOutputs) const;

    bool setRootPositionsOvStage(const TensorDesc* srcTensor,
                                 const ::physx::PxU32* rows,
                                 ::physx::PxU32 numOutputs,
                                 uint64_t rowsToken,
                                 const OvStageDeviceWriteArgs& deviceArgs);
    bool setRootOrientationsOvStage(const TensorDesc* srcTensor,
                                    const ::physx::PxU32* rows,
                                    ::physx::PxU32 numOutputs,
                                    uint64_t rowsToken,
                                    const OvStageDeviceWriteArgs& deviceArgs);
    bool setRootLinearVelocitiesOvStage(const TensorDesc* srcTensor,
                                        const ::physx::PxU32* rows,
                                        ::physx::PxU32 numOutputs,
                                        uint64_t rowsToken,
                                        const OvStageDeviceWriteArgs& deviceArgs);
    bool setRootAngularVelocitiesOvStage(const TensorDesc* srcTensor,
                                         const ::physx::PxU32* rows,
                                         ::physx::PxU32 numOutputs,
                                         uint64_t rowsToken,
                                         const OvStageDeviceWriteArgs& deviceArgs);

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
                                // Index into GpuSimulationData's ApplyEvent array: the events the
                                // drain orders the apply with.
                                int applyEvent,
                                // Which folds the scatter undoes. Not derivable from the write flag:
                                // the four state/target columns are eAngularSigned and the actuation
                                // force is eSigned, and every one of them is a JOINT_* write type.
                                DofScalePolicy policy,
                                const OvStageDeviceWriteArgs& deviceArgs);
    // The ovstage drain's execution: the legacy default stream and this view's apply events for `applyEvent`.
    OvStageDeviceWriteArgs drainDeviceWriteArgs(int applyEvent) const;

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

    bool getDofAttribute(const char* attribName,
                         const TensorDesc* dstTensor,
                         const ::physx::PxArticulationGPUAPIReadType::Enum attribFlag,
                         CUevent syncEvent) const;

    // Generic ovstage tendon write the two setters above pass their own engine values to, mirroring the
    // tendon record read exactly: same struct, same structFloats, same field offset.
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
                                  ::physx::PxU32 numOutputs,
                                  const OvStageDeviceWriteArgs& deviceArgs,
                                  TendonFieldWrite mode);

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

    // The start event getArticulationMassCenter orders its zeroing ahead of the DirectGPU compute with.
    CUevent mMassCenterStartEvent = nullptr;

    // The read plans planOvStageRead made for tokened row lists.
    mutable OvStageReadPlans<OvStageArtiReadPlan> mOvStageReadPlans;

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

};

} // namespace tensors
} // namespace physx
} // namespace omni

// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-INPUT-DEVICE-001
 * @covers AC-9
 */

#pragma once

#include "tensors/CommonTypes.h"
#include "tensors/base/BaseRigidBodyView.h"
#include "tensors/gpu/GpuSimulationData.h"
#include "tensors/gpu/OvStageDeviceArgs.h"
#include "tensors/gpu/OvStageReadPlans.h"
#include "PxDirectGPUAPI.h"

#include <omni/physics/tensors/IRigidBodyView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class GpuSimulationView;

class GpuRigidBodyView : public BaseRigidBodyView
{
public:
    GpuRigidBodyView(GpuSimulationView* sim,
                     const std::vector<RigidBodyEntry>& entries,
                     int device,
                     bool invalidateOnDisabledRd = true);

    ~GpuRigidBodyView() override;

    // Test-only: number of ovstage row-list host-to-device uploads this view has performed. A write that
    // reuses a cached slot does not increment it, so a test can tell a reused list from a new upload.
    uint64_t ovStageRowsUploadCount() const { return mOvStageRowsUploadCount; }

    bool getTransforms(const TensorDesc* dstTensor) const override;
    bool getVelocities(const TensorDesc* dstTensor) const override;
    bool getAccelerations(const TensorDesc* dstTensor) const override;

    // The device columns an ovstage read gathers from this view (ADR-0055). Position and orientation are two slices
    // of one eGLOBAL_POSE copy, and an acceleration has its own copy beside its velocity's.
    //
    // The accelerations require PxSceneFlag::eENABLE_BODY_ACCELERATIONS, which PhysXScene sets on every scene it
    // creates. Without it PhysX reports zero rather than refusing.
    //
    // eWorldScale has no PhysX copy: PhysX simulates no scale, so it is gathered from mWorldScalesDev, the
    // creation-time scale this view uploaded once (RigidBodyEntry::worldScale).
    enum class OvStageRigidRead : uint8_t
    {
        eNone,
        ePosition,
        eOrientation,
        eLinearVelocity,
        eAngularVelocity,
        eLinearAcceleration,
        eAngularAcceleration,
        eWorldScale
    };
    struct OvStageRigidReadColumn
    {
        OvStageRigidRead kind = OvStageRigidRead::eNone;
        // [numOutputs, 3] float32 on this view's device, [numOutputs, 4] for an orientation.
        float* dst = nullptr;
    };
    // What one read fetches, decided on the host when its plan is made. Its rows are rigid dynamics or links, never
    // both, as its query type is. A read covering every body of its kind in the view fetches the view through the
    // view's full index list; any other fetches only its rows.
    struct OvStageRigidReadPlan
    {
        ::physx::PxU32 numOutputs = 0;
        uint32_t sources = 0; // one bit per PhysX copy the columns need, by OvStageRigidRead's fetch
        bool links = false;
        bool wholeView = false;
        ::physx::PxU32 numFetched = 0; // rigid dynamics, or articulations for links, each copy holds
    };

    // Plans a read of `columns` (their kinds; destinations may still be unset) over `rows` (destination slot -> view
    // record): the plan, the bytes of device scratch it runs in, and the list that heads that scratch -- the rows,
    // then for a packed link read the articulations fetched and each row's fetched slot -- which the caller uploads
    // before the read whenever its content changed. A nonzero `rowsToken` names the content of `rows`
    // (nextOvStageRowsVersion): a plan this view already made for it and the same column kinds is reused without
    // walking the rows. Call after the GPU index map is current (refreshDisabledRowsOvStage).
    bool planOvStageRead(const ::physx::PxU32* rows,
                         ::physx::PxU32 numOutputs,
                         uint64_t rowsToken,
                         const OvStageRigidReadColumn* columns,
                         ::physx::PxU32 columnCount,
                         OvStageRigidReadPlan& plan,
                         std::shared_ptr<const std::vector<uint8_t>>& head,
                         size_t& scratchBytes) const;

    // Queues a planned read on `args.stream`: a packed read's index build, one PhysX copy per source into
    // `args.scratch`, and one gather per column. It blocks nothing, allocates nothing and uploads nothing, so a
    // capture records it. Outside a capture it then registers `args.stream` as a reader of this view's tables.
    bool readOvStageColumns(const OvStageDeviceReadArgs& args,
                            const OvStageRigidReadPlan& plan,
                            const OvStageRigidReadColumn* columns,
                            ::physx::PxU32 columnCount) const;

    // The point-instancer columns of an ovstage read (ADR-0055): the instanced bodies' state, from one copy of the
    // view's rigid dynamics per source, scattered into each instancer's run of `dst` at its float offset in
    // `offsetsDev`; a pose is reframed into its instancer's frame. A slot with no live instance is left as the
    // caller zeroed it.
    struct OvStageInstancerReadColumn
    {
        OvStageRigidRead kind = OvStageRigidRead::eNone;
        float* dst = nullptr;
        const ::physx::PxU32* offsetsDev = nullptr;
    };
    // The bytes of device scratch the copies for `columns` take. Call after the GPU index map is current.
    size_t ovStageInstancerReadScratchBytes(const OvStageInstancerReadColumn* columns,
                                            ::physx::PxU32 columnCount) const;
    // Queues the copies into `args.scratch` and the gathers on `args.stream`, as readOvStageColumns does.
    bool readOvStageInstancerColumns(const OvStageDeviceReadArgs& args,
                                     const OvStageInstancerReadColumn* columns,
                                     ::physx::PxU32 columnCount,
                                     const GpuPointInstancerRecord* recordsDev,
                                     const InstancerAffine* inversesDev,
                                     ::physx::PxU32 numInstances) const;

    // The WRITE counterpart of readOvStageInstancerColumns above (ADR-0012). Lives here, not on the
    // instancer view, for the reason that read does: this class owns the DirectGPU call, the packed
    // index machinery and the rigid records the instances resolve through.
    //
    // `records` points at ONE instancer's range and `numOutputs` is its length, so the packed pair
    // handed to PhysX is exactly that instancer's live instances -- count known on the host, no
    // compaction, no host block.
    //
    // `rbRows` is the host-side record-row list for those same instances, in the same order, which
    // is what the packed GPU-index build indexes through.
    bool setInstancerPoseColumnOvStage(bool wantOrientation,
                                       const float* srcDev,
                                       const GpuPointInstancerRecord* recordsDev,
                                       const InstancerAffine* instancerInversesDev,
                                       const InstancerAffine* instancerForwardsDev,
                                       const ::physx::PxU32* rbRows,
                                       ::physx::PxU32 numOutputs,
                                       ::physx::PxU32 instancerIdx,
                                       uint64_t rowsToken,
                                       const OvStageDeviceWriteArgs& deviceArgs);
    bool setInstancerVelocityColumnOvStage(bool wantAngular,
                                           const float* srcDev,
                                           const GpuPointInstancerRecord* recordsDev,
                                           const ::physx::PxU32* rbRows,
                                           ::physx::PxU32 numOutputs,
                                           uint64_t rowsToken,
                                           const OvStageDeviceWriteArgs& deviceArgs);


    // Fused ovstage column WRITE (ADR-0012): the return direction of readOvStageColumns above -- one
    // self-contained call per attribute, taking the caller's dense column plus the destination-slot ->
    // view-record list, and carrying NO host block.
    //
    // Each builds a PACKED (data, gpuIndex) pair straight from the row list and issues one
    // PxDirectGPUAPI write over it. Not a flags-and-compaction scatter, and that is a correctness
    // constraint rather than a speed one: the compaction derives its element count on the host, so
    // any path using it must block, and the read's contract is that neither direction does.
    //
    // A new attribute is a thin method here plus a row in the writer's table.
    //
    // Position and orientation are two slices of ONE eGLOBAL_POSE write, so each first reads the
    // covered bodies' current poses to preserve the half it is not writing. The velocity pair needs
    // no such read -- eLINEAR_VELOCITY and eANGULAR_VELOCITY are separate write types -- and hands
    // the caller's column to PhysX untouched, a dense [N,3] float column being exactly PxVec3[N].
    bool setPositionsOvStage(const TensorDesc* srcTensor,
                             const ::physx::PxU32* outRecordIdx,
                             ::physx::PxU32 numOutputs,
                             uint64_t rowsToken, bool wholeSet,
                             const OvStageDeviceWriteArgs& deviceArgs);
    bool setOrientationsOvStage(const TensorDesc* srcTensor,
                                const ::physx::PxU32* outRecordIdx,
                                ::physx::PxU32 numOutputs,
                                uint64_t rowsToken, bool wholeSet,
                                const OvStageDeviceWriteArgs& deviceArgs);
    // True when this row list names articulation LINKS rather than rigid dynamics. Decided from the
    // FIRST valid row, not by scanning: an ovstage query carries one object type, so a mixed list is
    // not a state this API can produce.
    bool ovStageRowsAreLinks(const ::physx::PxU32* rows, ::physx::PxU32 numOutputs) const;

    // The articulation-link route for the two write-only attributes. PhysX addresses a link's force
    // through eLINK_FORCE / eLINK_TORQUE at (articulation, link), not through a rigid-dynamic row,
    // so a link query cannot use the packed rigid path at all -- it has no rd index to pack.
    //
    // `comps` selects the attribute: 3 for `force`, 9 for `wrench`.
    bool setLinkWrenchOvStage(const char* attribName,
                              const TensorDesc* srcTensor,
                              const ::physx::PxU32* outRecordIdx,
                              ::physx::PxU32 numOutputs,
                              uint64_t rowsToken,
                              ::physx::PxU32 comps,
                              bool wholeSet,
                              const OvStageDeviceWriteArgs& deviceArgs);

    // Session writes (ADR-0052) run in device memory the session owns, sized by this for `kind` and the
    // group's row count. Its leading u32 array is the group's superset row list, uploaded at open.
    enum class OvStageRigidWrite
    {
        ePose, // position or orientation: packed poses, read-modify-written
        eVec3, // linear or angular velocity, or a force: a dense column merged under a mask
        eWrench,
        eLinkForce,
        eLinkWrench,
        eInstancerPose,
        eInstancerVelocity
    };
    size_t ovStageWriteScratchBytes(OvStageRigidWrite kind, ::physx::PxU32 numOutputs) const;
    // The host work a session write does once, at open, so its commit only enqueues: a wrench refreshes the
    // centre-of-mass table, and a link force or wrench resolves which articulations it applies, uploading the
    // row-to-articulation map and the applied GPU index list into `deviceArgs.scratch` (whose row list must already
    // be uploaded) and recording the count in `deviceArgs`.
    bool prepareOvStageWrite(OvStageRigidWrite kind,
                             const ::physx::PxU32* rows,
                             ::physx::PxU32 numOutputs,
                             bool wholeSet,
                             OvStageDeviceWriteArgs& deviceArgs);
    // True when the next device operation would first upload a refreshed GPU index map: a body was
    // enabled or disabled since the last one. A captured write cannot upload, so it refuses instead.
    bool rdIndicesRefreshPending() const;
    // The scene's enable/disable epoch, which every supported toggle of eDISABLE_SIMULATION advances.
    uint64_t rdDisableEpoch() const
    {
        return mGpuSimData ? mGpuSimData->mRdDisableEpoch : 0;
    }

    // ovstage WRENCH write (ADR-0012). One [N,9] column -- force, torque, world
    // application point -- becoming PhysX's eFORCE and eTORQUE in a single pass.
    //
    // Write-only like `force`, so no read-modify-write. Unlike `force` it needs the body POSE and
    // centre of mass, because a load applied away from the COM also torques about it -- which is the
    // only thing that distinguishes the two attributes.
    bool setWrenchesOvStage(const TensorDesc* srcTensor,
                            const ::physx::PxU32* outRecordIdx,
                            ::physx::PxU32 numOutputs,
                            uint64_t rowsToken, bool wholeSet,
                            const OvStageDeviceWriteArgs& deviceArgs);

    // ovstage FORCE write (ADR-0012). Write-only: the solver consumes it and clears it
    // each step, so there is nothing to read back and nothing to preserve -- which is why this needs
    // no read-modify-write where pose does.
    //
    // Applied at the CENTRE OF MASS, which is what PxRigidDynamicGPUAPIWriteType::eFORCE means. Use
    // `wrench` for a load at an arbitrary point.
    bool setForcesOvStage(const TensorDesc* srcTensor,
                          const ::physx::PxU32* outRecordIdx,
                          ::physx::PxU32 numOutputs,
                          uint64_t rowsToken, bool wholeSet,
                          const OvStageDeviceWriteArgs& deviceArgs);

    bool setLinearVelocitiesOvStage(const TensorDesc* srcTensor,
                                    const ::physx::PxU32* outRecordIdx,
                                    ::physx::PxU32 numOutputs,
                                    uint64_t rowsToken, bool wholeSet,
                                    const OvStageDeviceWriteArgs& deviceArgs);
    bool setAngularVelocitiesOvStage(const TensorDesc* srcTensor,
                                     const ::physx::PxU32* outRecordIdx,
                                     ::physx::PxU32 numOutputs,
                                     uint64_t rowsToken, bool wholeSet,
                                     const OvStageDeviceWriteArgs& deviceArgs);

    // Block until the last ovstage write issued through this view has completed on the device.
    //
    // The ONE place the write blocks, and it is at RELEASE rather than at commit -- mirroring the
    // read, which blocks in ovxReleaseRead for exactly the same reason. The column being freed is
    // the buffer PhysX reads the payload from (the velocity paths hand it over directly), so
    // freeing it while the write is still in flight is a use-after-free on device memory.
    bool waitOvStageWriteDone() const;

    // True when an entry's actor was absent from the simulation data's actor map at construction.
    // The data does not describe this actor set -- it predates the actor, or belongs to a scene
    // that has since been destroyed and its address reused. A caller that built this view over
    // cached data should discard the cache and rebuild rather than read sentinel rows.
    bool hasUnresolvedEntries() const { return mHasUnresolvedEntries; }

    bool setTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    bool applyForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                         const TensorDesc* srcTorqueTensor,
                                         const TensorDesc* srcPositionTensor,
                                         const TensorDesc* indexTensor,
                                         const bool isGlobal) override;

    // Masked overrides
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

    // OMPE-103213: CPU-only PhysX property APIs -- no silent GPU->host staging.
    // Callers must pass host tensors. setDisableGravities inherits the shared base
    // solver refresh; setDisableSimulations keeps rd-index dirtying.
    bool setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDisableSimulations(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    // An ovstage host-property write can toggle eDISABLE_SIMULATION through BaseRigidBodyView.
    // Tell this view and every sibling sharing its GpuSimulationData to refresh their compacted
    // DirectGPU indices before the next device operation.
    void markRdDisableDirty();

    // Rebuild the compacted DirectGPU indices after a supported disable or re-enable -- including one
    // outside the current query, since the bulk copy still indexes every compacted RD row. Triggered
    // by this view's own toggle (the per-view dirty flag) and by a sibling view's toggle (the
    // scene-wide disable epoch), NOT merely because sentinel rows remain -- a parked-disabled body
    // stays on the fast path -- and never by an actor-flag walk; see the definition for the contract
    // that makes the epoch sufficient.
    bool refreshDisabledRowsOvStage() const;

    // O(1) conservative answer to "could this view hold a rigid dynamic with no DirectGPU row?".
    // Lets a caller skip its own per-body actor-flag scan on the disable-free steady state without
    // giving up correctness: it cannot return false while a notified disable is outstanding.
    bool mayHaveDisabledRdRows() const;

    // Declare this view the scene superset -- it holds an entry for every rigid dynamic in the
    // scene. Only such a view may retire GpuSimulationData::mMayHaveDisabledRd, so this is set by
    // GpuSimulationView::supersetRigidView() and by nothing else.
    void markAsSceneSuperset()
    {
        mIsSceneSuperset = true;
    }

private:
    // Device copy of an ovstage write's row list, uploaded only when `token` differs from the
    // one the held copy was uploaded under. Returns null on allocation or upload failure; the
    // buffer stays owned by this view and must not be freed by the caller.
    const ::physx::PxU32* ovStageRowsDevice(const ::physx::PxU32* rows, uint32_t count, uint64_t token) const;
    // Generic ovstage column writes the four thin setters above pass their own engine values to.
    // `attribName` is the label the tensor checks report under.
    bool setPoseColumnOvStage(const char* attribName,
                              const TensorDesc* srcTensor,
                              bool wantOrientation,
                              const ::physx::PxU32* outRecordIdx,
                              ::physx::PxU32 numOutputs,
                              uint64_t rowsToken,
                              const OvStageDeviceWriteArgs& deviceArgs);
    bool setVelocityColumnOvStage(const char* attribName,
                                  const TensorDesc* srcTensor,
                                  ::physx::PxRigidDynamicGPUAPIWriteType::Enum rdType,
                                  const ::physx::PxU32* outRecordIdx,
                                  ::physx::PxU32 numOutputs,
                                  uint64_t rowsToken,
                                  const OvStageDeviceWriteArgs& deviceArgs);
    // The ovstage drain's execution: `stream` (the stream the read handed back, so the drain's kernels and the
    // PhysX call it forks share one stream -- ordered without an extra event; null -> the legacy default stream,
    // the drain's behaviour before deferred sync) and this view's apply events for `applyEvent`.
    OvStageDeviceWriteArgs drainDeviceWriteArgs(int applyEvent, CUstream stream = nullptr) const;
    // Refreshes the GPU index map before a write, which a captured write cannot do.
    bool refreshRdIndicesForWrite(const char* attribName, const OvStageDeviceWriteArgs& deviceArgs) const;

    // The device state an ovstage write needs: the uploaded row list, a packed transform block sized
    // for the write, and the packed GPU-index list PhysX pairs with it by position.
    //
    // All three are held across writes and rebuilt only when `token` says the row list changed --
    // the same gate ovStageRowsDevice uses, and the reason the per-row sentinel validation below is
    // affordable: it runs when the list is built, not on every write.
    //
    // `outPacked` is left untouched when the caller does not need a data block (the velocity paths
    // hand PhysX the caller's column directly, a dense [N,3] float column being exactly PxVec3[N]).
    //
    // A session write's (`deviceArgs.sessionOwned()`) row list and index list are the first two arrays of its own
    // scratch, and `outPacked` is the rest of it, which the caller lays out: `payloadBytes` of it, checked against the
    // scratch's size. The index build runs on `deviceArgs.stream`.
    bool ovStageWriteBuffers(const char* attribName,
                             const ::physx::PxU32* rows,
                             ::physx::PxU32 numOutputs,
                             uint64_t token,
                             const OvStageDeviceWriteArgs& deviceArgs,
                             size_t payloadBytes,
                             const ::physx::PxU32*& outRowsDev,
                             void*& outPacked,
                             ::physx::PxRigidDynamicGPUIndex*& outIdx);

    bool updateCMassData();
    bool clearDataFlagsAndIndices();
    bool resolveMask(const TensorDesc* maskTensor, ::physx::PxU32& outK) const;
    // Fixed-membership tensor views invalidate when a member loses its DirectGPU row. The ovstage
    // superset instead retains a sentinel record so its stable actor-to-row map can continue to
    // serve enabled rows and a later re-enable write.
    void invalidateMappingForDisabledRd(const char* reason) const;

    int mDevice = -1;

    GpuSimulationDataPtr mGpuSimData;
    bool mInvalidateOnDisabledRd = true;
    mutable bool mHasDisabledRdRows = false;
    // See markAsSceneSuperset(). Gates the one direction of the scene-wide disable hint that has to
    // be earned rather than assumed.
    bool mIsSceneSuperset = false;

    // Mutable: refreshRdGpuIndices() updates the enabled-rd count per read.
    mutable uint32_t mNumRds = 0;
    // Set by setDisableSimulations; cleared by refreshRdGpuIndices once the
    // indices are re-synced. Lets the refresh skip the O(n) getGPUIndex() loop
    // when no disable/enable transition has occurred since the last sync --
    // making the standard (no disable) case a true no-op.
    mutable bool mRdIndexDirty = true;
    // Set when an index upload fails. mRdIndexDirty only recomputes the diff, which is empty on the
    // next pass because rbRecords was updated before the upload; this drives the re-send, and forces
    // the whole array since the changed range belonged to the pass that failed.
    mutable bool mDeviceUploadPending = false;
    // Last scene-wide disable epoch (GpuSimulationData::mRdDisableEpoch) this
    // view has synced to. When a *sibling* view over the same scene disables or
    // re-enables a body, the shared epoch advances past this value; the refresh
    // fast path detects the gap and forces a rebuild so this view doesn't keep
    // reading the freed/reallocated GPU indices (OMPE-94459, multi-view + body
    // disable).
    mutable uint64_t mLastSeenRdDisableEpoch = 0;
    uint32_t mNumArtis = 0;
    uint32_t mNumArtiRoots = 0;
    // Link records in this view: every link of every articulation it holds. A whole-set link write may skip its
    // dedup only when it names this many rows: it leaves out an articulation not yet in a scene when the write
    // was opened, and the next step adds that articulation to this view.
    uint32_t mNumLinkRecords = 0;

    // will keep this until direct GPU API for articulation link mass properties is available
    // made class memebrs to avoid frequent memory allocation
    // TODO: clean up these variables
    // Mutable: refreshRdGpuIndices() rebuilds the rd fields from the const
    // read paths (OMPE-94459).
    mutable std::vector<GpuRigidBodyRecord> rbRecords;
    std::vector<::physx::PxVec3> cMassLocalPosePos;
    ::physx::PxVec3* cMassLocalPosePosDev = nullptr; // coms for links + rd

    // indexing data for all rigid bodies
    GpuRigidBodyRecord* mRbRecordsDev = nullptr;
    // RigidBodyEntry::worldScale per record, uploaded once: an actor's stored scale never changes in
    // place, because a scale edit either recreates the actor, and with it the view, or is not applied.
    ::physx::PxVec3* mWorldScalesDev = nullptr;

    // all body indices in this view
    // Set at construction when an entry's actor was not in the simulation data's actor map.
    bool mHasUnresolvedEntries = false;

    ::physx::PxU32* mRbIndicesDev = nullptr;
    ::physx::PxRigidDynamicGPUIndex* mRdGpuIndicesDev = nullptr;
    ::physx::PxRigidDynamicGPUIndex* mDirtyRdGpuIndices = nullptr;

    // articulation indices for fetching all link data and applying forces
    ::physx::PxArticulationGPUIndex* mArtiIndicesDev = nullptr;
    ::physx::PxArticulationGPUIndex* mDirtyArtiGpuIndices = nullptr;
    // dirty flag buffers with the size of articulation link and rigid link in the view
    ArticulationGpuFlags* mArtiDirtyFlagsDev = nullptr;
    ArticulationGpuFlags* mArtiLinksDirtyFlagsDev = nullptr;
    ActorGpuFlags* mRdDirtyFlagsDev = nullptr;

    SingleAllocPolicy mArtiIndexSingleAllocPolicy;
    SingleAllocPolicy mRdIndexSingleAllocPolicy;

    // Mask -> indices scratch (cached). Declared mutable because resolveMask()
    // is const - some interface setters (e.g. material/shape properties) are
    // const by pre-existing TensorAPI convention ("const" = view object unchanged,
    // simulation state may be mutated via pointer indirection). These buffers are
    // internal caching state, not logical view state.
    mutable ::physx::PxU32* mMaskIndicesDev = nullptr;
    mutable ::physx::PxU32 mMaskIndicesCapacity = 0;
    mutable SingleAllocPolicy mMaskAllocPolicy;

    // Host scratch for the per-read rd GPU-index rebuild (OMPE-94459). Mutable
    // for the same const-read-path reason as the buffers above.
    mutable std::vector<::physx::PxRigidDynamicGPUIndex> mRdGpuIndicesHost;

    // Device copies of the ovstage writes' destination-slot -> superset-row lists, held across writes.
    // Owned here rather than by the reader: this view already frees its device memory under the right
    // context in its destructor. Each slot's `token` identifies its contents, so an unchanged list is
    // not re-uploaded.
    //
    // A SMALL LRU, not one slot: this superset view is the single row cache shared by every ovstage
    // write outside a write session that scatters a subset through it -- the drain's rigid-body and
    // articulation-link lists (a PxArticulationLink is a PxRigidBody), and one list per point instancer.
    // Reads keep their row lists on their query (ADR-0055), and write sessions in their own scratch
    // (ADR-0052). Several are live at once in a normal write loop, so one slot would evict the previous list
    // on every access and re-upload it, and the token gate would never hit. The LRU holds kOvStageRowsSlots
    // distinct lists. Thrash-free while the concurrently-live lists number <= kOvStageRowsSlots; beyond that
    // the least-recently-used list is evicted, never worse than the single slot it replaced (see
    // kOvStageRowsSlots for what that bound does and does not buy). ovStageRowsDevice picks the smallest-tick
    // slot as the victim (an empty slot, tick 0, wins first).
    struct OvStageRowsSlot
    {
        ::physx::PxU32* dev = nullptr;
        uint32_t capacity = 0;
        uint32_t count = 0;
        uint64_t token = 0; // 0 marks an empty slot; never matched by the hit check
        uint64_t tick = 0; // recency stamp; the smallest-tick slot is the eviction victim
    };
    // Six is a pragmatic bound, NOT a guarantee against thrash. It holds the COMMON interleave -- a
    // write's standalone list and a handful of per-instancer groups. It does not bound the worst case: a
    // scene has arbitrarily many point instancers and a write emits one row list per range, so once the
    // concurrently-live lists exceed kOvStageRowsSlots the LRU thrashes -- e.g. seven ranges committed in
    // one order hit zero times on the next pass (each miss evicts the entry needed next). Throughput then
    // falls back to the pre-LRU single-slot cost (never worse); only the warmth is lost. No test covers
    // this pattern.
    //
    // The memory cost is real and scales with the slot count: a slot's capacity only ever grows (see
    // ovStageRowsDevice), so all six converge on the largest list any has held -- up to
    // 6 * maxRowCount * sizeof(PxU32) retained per scene view (~240 MB at 10M rows), 6x the single-slot
    // footprint and a tracked device-buffer-growth concern (ADR-0027, the ovstage-read device-buffer
    // pool). Six trades that memory for warmth on the common interleave.
    static constexpr uint32_t kOvStageRowsSlots = 6;
    mutable OvStageRowsSlot mOvStageRows[kOvStageRowsSlots];
    mutable uint64_t mOvStageRowsClock = 0; // monotonic recency source; ++ on every hit and upload
    mutable uint64_t mOvStageRowsUploadCount = 0; // test-only: row-list H2D uploads; see ovStageRowsUploadCount()
    // The read plans planOvStageRead made for tokened row lists.
    mutable OvStageReadPlans<OvStageRigidReadPlan> mOvStageReadPlans;

    // The ovstage WRITE's packed pair, held across writes alongside the row list above and owned
    // here for the same reason: this view already frees its device memory under the right context in
    // its destructor, where a caller-side buffer would have to free through a context manager that
    // may be gone. Sized in PxTransform units, which is the larger element.
    ::physx::PxTransform* mOvStagePackedDev = nullptr;
    // The wrench path's two dense outputs, PxVec3[2 * N] -- forces first, torques second. Its own
    // buffer rather than carved out of mOvStagePackedDev because that block is simultaneously
    // holding this write's PACKED POSES (7 floats per output), and 7N + 6N does not fit in 7N.
    ::physx::PxVec3* mOvStageWrenchDev = nullptr;
    uint32_t mOvStageWrenchCapacity = 0;
    ::physx::PxRigidDynamicGPUIndex* mOvStageIdxDev = nullptr;
    uint32_t mOvStageWriteCapacity = 0;
    // The link-wrench SPARSE apply's two host-resolved index buffers (grown-and-cached like the wrench
    // output above). mOvStageLinkWrenchArtisDev holds the written articulations' PhysX GPU indices in
    // compact order; mOvStageLinkWrenchRowArtiDev holds, per written row, that row's compact slot. Both
    // are built on the host from the query's records, so the apply reaches only the written
    // articulations with no device->host survivor-count read.
    ::physx::PxArticulationGPUIndex* mOvStageLinkWrenchArtisDev = nullptr;
    uint32_t mOvStageLinkWrenchArtisCapacity = 0;
    ::physx::PxU32* mOvStageLinkWrenchRowArtiDev = nullptr;
    uint32_t mOvStageLinkWrenchRowArtiCapacity = 0;
    // Host scratch for the link-wrench dedup, cached across calls so steady-state reuse does not reallocate.
    std::vector<::physx::PxU32> mLinkWrenchArtiToCompact;                   // tensorArtiIdx -> compact slot
    std::vector<::physx::PxArticulationGPUIndex> mLinkWrenchWrittenArtis;   // compact slot -> GPU index
    std::vector<::physx::PxU32> mLinkWrenchRowCompact;                      // written row -> compact slot
    // What the per-row validation was last run against. TWO values, because the thing it checks can
    // be invalidated by either: the row list moving, or a disable/re-enable recycling GPU indices
    // under an unchanged row list. The index kernel itself is not gated on these at all -- see
    // ovStageWriteBuffers for why it runs every write.
    uint64_t mOvStageIdxToken = 0;
    mutable uint64_t mOvStageIdxDisableEpoch = 0;

    // Rebuild the compacted rd GPU-index list (mRdGpuIndicesDev), per-entry
    // tensorRdIdx/physxRdIdx, and mNumRds from each body's LIVE getGPUIndex(),
    // then re-upload if the mapping changed. Must run before any DirectGPU
    // rigid-dynamic read/write, because the GPU index is recycled across
    // disable/re-enable and cannot be cached across reads.
    // Returns false on a failed upload. A rigid dynamic without a GPU row
    // additionally invalidates the parent SimulationView and fails the op on a
    // fixed-membership view (mInvalidateOnDisabledRd); the ovstage superset keeps
    // the sentinel row instead and stays usable for its enabled rows.
    bool refreshRdGpuIndices() const;
};
} // namespace tensors
} // namespace physx
} // namespace omni

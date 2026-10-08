// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "tensors/CommonTypes.h"
#include "tensors/base/BaseRigidBodyView.h"
#include "tensors/gpu/GpuSimulationData.h"
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

    // Test-only: number of ovstage row-list host-to-device uploads this view has performed. A read that
    // reuses a cached slot does not increment it, so a test can assert that the two stable kOvxAll reads
    // (loose rigid bodies, articulation links) stay warm in their own LRU slots even while an
    // uncacheable kOvxActive read churns a third list between them -- the row-cache reuse this class
    // exists to provide.
    uint64_t ovStageRowsUploadCount() const { return mOvStageRowsUploadCount; }

    bool getTransforms(const TensorDesc* dstTensor) const override;
    bool getVelocities(const TensorDesc* dstTensor) const override;
    bool getAccelerations(const TensorDesc* dstTensor) const override;

    // Fused ovstage column read (ADR-0008): one self-contained call per attribute. Each issues the
    // bulk DirectGPU read its column is sourced from, waits on it, and gathers the packed
    // destination straight from the PhysX-layout scratch. Stream-ordered; the caller syncs once.
    //
    // `outRecordIdx` maps destination slot -> view record (null = identity), so a subset or
    // reordering of a larger view is served without rebuilding it; `rowsToken` identifies that list
    // for the device copy this view holds across reads.
    //
    // Position and orientation are two slices of one eGLOBAL_POSE read, so asking for both issues it
    // twice.
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
    // Acceleration reads the SAME scratch buffers velocity does (mRd*VelAccDev) under a different
    // DirectGPU read type, so a velocity and an acceleration in one read serialise on it -- covered
    // by ADR-0008 Decision 7's per-buffer event; the pair needs no buffer or event of its own.
    //
    // Requires PxSceneFlag::eENABLE_BODY_ACCELERATIONS, which PhysXScene sets on every scene it
    // creates. Without it PhysX reports zero rather than refusing.
    bool getLinearAccelerationsOvStage(const TensorDesc* dstTensor,
                                       const ::physx::PxU32* outRecordIdx,
                                       ::physx::PxU32 numOutputs,
                                       uint64_t rowsToken) const;
    bool getAngularAccelerationsOvStage(const TensorDesc* dstTensor,
                                        const ::physx::PxU32* outRecordIdx,
                                        ::physx::PxU32 numOutputs,
                                        uint64_t rowsToken) const;

    // The WRITE counterpart of the two instancer gathers below (ADR-0012). Lives here, not on the
    // instancer view, for the reason the gathers do: this class owns the DirectGPU call, the packed
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
                                       uint64_t rowsToken);
    bool setInstancerVelocityColumnOvStage(bool wantAngular,
                                           const float* srcDev,
                                           const GpuPointInstancerRecord* recordsDev,
                                           const ::physx::PxU32* rbRows,
                                           ::physx::PxU32 numOutputs,
                                           uint64_t rowsToken);

    // Point-instancer columns, for the ovstage read's instancer groups. `dstDev` is one device
    // buffer the caller has sub-allocated per instancer; `offsetsDev` gives each instancer's start
    // in FLOATS -- not "elements", which in this codebase means the dtype.lanes tuple. One offsets
    // array per column, since a column's sub-array is sized by its own width. Slots with no live
    // instance are left untouched, so the caller's zero fill stands.
    //
    // Sourced from the same bulk DirectGPU read the standalone-body columns use, over this view's
    // rows -- the superset view covers instanced bodies too.
    bool getInstancerPoseColumnOvStage(bool wantOrientation,
                                       float* dstDev,
                                       const GpuPointInstancerRecord* recordsDev,
                                       const InstancerAffine* instancerInversesDev,
                                       const ::physx::PxU32* offsetsDev,
                                       ::physx::PxU32 numInstances) const;

    // `wantAcceleration` selects the acceleration read type over the velocity one; everything else --
    // scratch buffer, buffer id, gather kernel -- is shared, since both are a plain PxVec3 per body.
    bool getInstancerVelocityColumnOvStage(bool wantAngular,
                                           bool wantAcceleration,
                                           float* dstDev,
                                           const GpuPointInstancerRecord* recordsDev,
                                           const ::physx::PxU32* offsetsDev,
                                           ::physx::PxU32 numInstances) const;

    // Fused ovstage column WRITE (ADR-0012): the return direction of the four getters above, and
    // deliberately the same shape -- one self-contained call per attribute, taking the caller's dense
    // column plus the destination-slot -> view-record list, and carrying NO host block.
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
                             uint64_t rowsToken);
    bool setOrientationsOvStage(const TensorDesc* srcTensor,
                                const ::physx::PxU32* outRecordIdx,
                                ::physx::PxU32 numOutputs,
                                uint64_t rowsToken);
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
                              ::physx::PxU32 comps);

    // ovstage WRENCH write (ADR-0012). One [N,9] column -- force, torque, world
    // application point -- becoming PhysX's eFORCE and eTORQUE in a single pass.
    //
    // Write-only like `force`, so no read-modify-write. Unlike `force` it needs the body POSE and
    // centre of mass, because a load applied away from the COM also torques about it -- which is the
    // only thing that distinguishes the two attributes.
    bool setWrenchesOvStage(const TensorDesc* srcTensor,
                            const ::physx::PxU32* outRecordIdx,
                            ::physx::PxU32 numOutputs,
                            uint64_t rowsToken);

    // ovstage FORCE write (ADR-0012). Write-only: the solver consumes it and clears it
    // each step, so there is nothing to read back and nothing to preserve -- which is why this needs
    // no read-modify-write where pose does.
    //
    // Applied at the CENTRE OF MASS, which is what PxRigidDynamicGPUAPIWriteType::eFORCE means. Use
    // `wrench` for a load at an arbitrary point.
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

    // Block until the last ovstage write issued through this view has completed on the device.
    //
    // The ONE place the write blocks, and it is at RELEASE rather than at commit -- mirroring the
    // read, which blocks in ovxReleaseRead for exactly the same reason. The column being freed is
    // the buffer PhysX reads the payload from (the velocity paths hand it over directly), so
    // freeing it while the write is still in flight is a use-after-free on device memory.
    void waitOvStageWriteDone() const;

    // True when an entry's actor was absent from the simulation data's actor map at construction.
    // The data does not describe this actor set -- it predates the actor, or belongs to a scene
    // that has since been destroyed and its address reused. A caller that built this view over
    // cached data should discard the cache and rebuild rather than read sentinel rows.
    bool hasUnresolvedEntries() const { return mHasUnresolvedEntries; }

    bool setKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    bool applyForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                         const TensorDesc* srcTorqueTensor,
                                         const TensorDesc* srcPositionTensor,
                                         const TensorDesc* indexTensor,
                                         const bool isGlobal) override;

    // Masked overrides
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

    // OMPE-103213: CPU-only PhysX property APIs -- no silent GPU->host staging.
    // Callers must pass host tensors. setDisableGravities keeps the DirectGPU
    // wake-for-refresh side effect; setDisableSimulations keeps rd-index dirtying.
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
    // Generic ovstage column read behind the thin methods above. `attribName` is the label the
    // tensor checks report under; `artiCopyEvent` indexes GpuSimulationData::mCopyEvents (an int
    // because CopyEvent's enum is unnamed).
    bool getPoseColumnOvStage(const char* attribName,
                              const TensorDesc* dstTensor,
                              bool wantOrientation,
                              const ::physx::PxU32* outRecordIdx,
                              ::physx::PxU32 numOutputs,
                              uint64_t rowsToken) const;
    bool getVelocityColumnOvStage(const char* attribName,
                                  const TensorDesc* dstTensor,
                                  ::physx::PxRigidDynamicGPUAPIReadType::Enum rdType,
                                  ::physx::PxArticulationGPUAPIReadType::Enum artiType,
                                  const ::physx::PxVec3* rdScratch,
                                  const ::physx::PxVec3* artiScratch,
                                  // Which SharedDeviceBuffer each scratch pointer IS, so the
                                  // ordering event can be keyed to it (ADR-0008 Decision 7).
                                  ::physx::PxU32 rdBuf,
                                  ::physx::PxU32 artiBuf,
                                  int artiCopyEvent,
                                  const ::physx::PxU32* outRecordIdx,
                                  ::physx::PxU32 numOutputs,
                                  uint64_t rowsToken) const;

    // Device copy of the ovstage reader's row list, uploaded only when `token` differs from the
    // one the held copy was uploaded under. Returns null on allocation or upload failure; the
    // buffer stays owned by this view and must not be freed by the caller.
    const ::physx::PxU32* ovStageRowsDevice(const ::physx::PxU32* rows, uint32_t count, uint64_t token) const;
    // Generic ovstage column writes the four thin setters above pass their own engine values to,
    // mirroring getPoseColumnOvStage / getVelocityColumnOvStage. `attribName` is the label the tensor
    // checks report under.
    bool setPoseColumnOvStage(const char* attribName,
                              const TensorDesc* srcTensor,
                              bool wantOrientation,
                              const ::physx::PxU32* outRecordIdx,
                              ::physx::PxU32 numOutputs,
                              uint64_t rowsToken);
    bool setVelocityColumnOvStage(const char* attribName,
                                  const TensorDesc* srcTensor,
                                  ::physx::PxRigidDynamicGPUAPIWriteType::Enum rdType,
                                  const ::physx::PxU32* outRecordIdx,
                                  ::physx::PxU32 numOutputs,
                                  uint64_t rowsToken);

    // The device state an ovstage write needs: the uploaded row list, a packed transform block sized
    // for the write, and the packed GPU-index list PhysX pairs with it by position.
    //
    // All three are held across writes and rebuilt only when `token` says the row list changed --
    // the same gate ovStageRowsDevice uses, and the reason the per-row sentinel validation below is
    // affordable: it runs when the list is built, not on every write.
    //
    // `outPacked` is left untouched when the caller does not need a data block (the velocity paths
    // hand PhysX the caller's column directly, a dense [N,3] float column being exactly PxVec3[N]).
    bool ovStageWriteBuffers(const char* attribName,
                             const ::physx::PxU32* rows,
                             ::physx::PxU32 numOutputs,
                             uint64_t token,
                             const ::physx::PxU32*& outRowsDev,
                             void*& outPacked,
                             ::physx::PxRigidDynamicGPUIndex*& outIdx);

    bool gatherPoseColumnOvStage(bool wantOrientation,
                                 const ::physx::PxU32* outRecordIdxDev,
                                 ::physx::PxU32 numOutputs,
                                 const TensorDesc* dstTensor) const;
    bool gatherVelocityColumnOvStage(const ::physx::PxVec3* rdSrc,
                                     const ::physx::PxVec3* linkSrc,
                                     const ::physx::PxU32* outRecordIdxDev,
                                     ::physx::PxU32 numOutputs,
                                     const TensorDesc* dstTensor) const;

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

    uint32_t mMaxLinks = 0;

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

    // Device copies of the ovstage reader's destination-slot -> superset-row lists, held across reads.
    // Owned here rather than by the reader: this view already frees its device memory under the right
    // context in its destructor. Each slot's `token` identifies its contents, so an unchanged list is
    // not re-uploaded.
    //
    // A SMALL LRU, not one slot: this superset view is the single row cache shared by every ovstage
    // list that gathers a subset through it -- kOvxAll rigid-body and articulation-link reads (a
    // PxArticulationLink is a PxRigidBody), kept warm across frames; uncacheable kOvxActive reads, a
    // fresh list every read; and the rigid write path -- a standalone list plus one per point instancer,
    // each rebuilt per commit. (Instancer READS do not pass through here.) Several are live at once in a
    // normal read/write loop, so one slot would evict the previous list on every access and re-upload
    // it, and the token gate would never hit. The LRU holds kOvStageRowsSlots distinct lists. A stable
    // read hits its slot every frame, keeping its recency newest, so a churning list -- an active read
    // or a write, whose token changes every time and is never seen again -- cycles through the surplus
    // slots and does not evict a warm read WHILE A SLOT IS FREE. Thrash-free while the concurrently-live
    // lists number <= kOvStageRowsSlots; beyond that the least-recently-used list is evicted -- warm
    // reads no longer exempt -- never worse than the single slot it replaced (see kOvStageRowsSlots for
    // what that bound does and does not buy). ovStageRowsDevice picks the smallest-tick slot as the
    // victim (an empty slot, tick 0, wins first).
    struct OvStageRowsSlot
    {
        ::physx::PxU32* dev = nullptr;
        uint32_t capacity = 0;
        uint32_t count = 0;
        uint64_t token = 0; // 0 marks an empty slot; never matched by the hit check
        uint64_t tick = 0; // recency stamp; the smallest-tick slot is the eviction victim
    };
    // Six is a pragmatic bound, NOT a guarantee against thrash. It holds the COMMON interleave -- the
    // two stable reads plus a few churning lists (an active read, a write's standalone and a handful of
    // per-instancer groups). It does not bound the worst case: a scene has arbitrarily many point
    // instancers and a write emits one row list per range, so once the concurrently-live lists exceed
    // kOvStageRowsSlots the LRU thrashes -- e.g. seven ranges committed in one order hit zero times on
    // the next pass (each miss evicts the entry needed next), and a burst that large evicts the warm
    // rigid/link reads too. Throughput then falls back to the pre-LRU single-slot cost (never worse);
    // only the warmth is lost. The current test exercises three lists (<= six), so it does NOT cover
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
    // What the per-row validation was last run against. TWO values, because the thing it checks can
    // be invalidated by either: the row list moving, or a disable/re-enable recycling GPU indices
    // under an unchanged row list. The index kernel itself is not gated on these at all -- see
    // ovStageWriteBuffers for why it runs every write.
    uint64_t mOvStageIdxToken = 0;
    mutable uint64_t mOvStageIdxDisableEpoch = 0;
    // Recorded on the null stream right after the packed index build; the startEvent of every
    // DirectGPU call that reads mOvStageIdxDev. PhysX copies on non-blocking streams, so without it
    // PhysX can read the list before the kernel has written it (see ovStageWriteBuffers).
    CUevent mOvStageIdxReadyEvent = nullptr;

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

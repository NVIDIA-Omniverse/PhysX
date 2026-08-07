// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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
    GpuRigidBodyView(GpuSimulationView* sim, const std::vector<RigidBodyEntry>& entries, int device);

    ~GpuRigidBodyView() override;

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
    bool setDisableGravitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDisableSimulationsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setMaterialPropertiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;
    bool setCompliantMaterialPropertiesMasked(const TensorDesc* srcTensor,
                                              const TensorDesc* srcCombineTensor,
                                              const TensorDesc* maskTensor) const override;
    bool setRestOffsetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;
    bool setContactOffsetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;

    // OMPE-94459 (§B9): GPU-aware overrides for the per-shape property setters.
    // BaseRigidBodyView's impls hardcode CPU device; these stage GPU tensors to
    // host buffers and delegate so GPU-mode shape-property writes don't fail.
    bool setMaterialProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    bool setRestOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    bool setContactOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    // setDisableGravities stages any GPU-resident src/index to host, then wakes
    // awake non-kinematic bodies (sleeping skipped) so DirectGPU reads updated
    // PxgBodySim.disableGravity via gpu_updateBodySim.
    bool setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDisableSimulations(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    // Reads of the same flags: write into a host buffer then memcpy to the
    // caller's GPU buffer when dst is on device.
    bool getDisableGravities(const TensorDesc* dstTensor) const override;
    bool getDisableSimulations(const TensorDesc* dstTensor) const override;
    // wakeUp on GPU pipeline: stage the GPU indices tensor to host and
    // delegate to the base impl (which calls PxRigidDynamic::wakeUp per
    // body -- a CPU-side PhysX call).
    bool wakeUp(const TensorDesc* indexTensor) override;
    bool putToSleep(const TensorDesc* indexTensor) override;

private:
    void copyActorAndLinksTransorms();
    bool updateCMassData();
    bool clearDataFlagsAndIndices();
    bool resolveMask(const TensorDesc* maskTensor, ::physx::PxU32& outK) const;

    int mDevice = -1;

    GpuSimulationDataPtr mGpuSimData;

    // Mutable: refreshRdGpuIndices() updates the enabled-rd count per read.
    mutable uint32_t mNumRds = 0;
    // Mutable: set by refreshRdGpuIndices() so the read path can skip the
    // disabled-body pose patch scan when nothing is disabled (OMPE-94459).
    mutable bool mHasDisabledRd = false;
    // Set by setDisableSimulations; cleared by refreshRdGpuIndices once the
    // indices are re-synced. Lets the refresh skip the O(n) getGPUIndex() loop
    // when no disable/enable transition has occurred since the last sync --
    // making the standard (no disable) case a true no-op.
    mutable bool mRdIndexDirty = true;
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
    ::physx::PxU32* mRbIndicesDev = nullptr;
    ::physx::PxRigidDynamicGPUIndex* mRdGpuIndicesDev = nullptr;
    ::physx::PxRigidDynamicGPUIndex* mDirtyRdGpuIndices = nullptr;

    // Disabled-body pose-patch scatter scratch (OMPE-94459): getTransforms() builds
    // the compacted patches + their body indices on the host, uploads both once,
    // then a single scatter kernel writes them into the read tensor -- instead of a
    // tiny synchronous cudaMemcpy per disabled body. mDisabledPatchDev is a
    // TensorTransform[] (typed void* to keep TensorTransform out of this header).
    void* mDisabledPatchDev = nullptr;
    ::physx::PxU32* mDisabledScatterIdxDev = nullptr;
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

    // Rebuild the compacted rd GPU-index list (mRdGpuIndicesDev), per-entry
    // tensorRdIdx/physxRdIdx, and mNumRds from each body's LIVE getGPUIndex(),
    // then re-upload if the mapping changed. Must run before any DirectGPU
    // rigid-dynamic read/write, because the GPU index is recycled across
    // disable/re-enable and cannot be cached across reads. See the .cpp.
    void refreshRdGpuIndices() const;

    // Mark this view dirty and advance the scene-wide disable epoch so sibling
    // views over the same scene also rebuild their GPU-index mapping on their
    // next read. Called from setDisableSimulations after a successful toggle
    // (OMPE-94459, multi-view + body disable).
    void markRdDisableDirty();
};
} // namespace tensors
} // namespace physx
} // namespace omni

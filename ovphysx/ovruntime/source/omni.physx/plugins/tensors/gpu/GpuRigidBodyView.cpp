// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "tensors/gpu/CudaKernels.h"
#include "tensors/gpu/GpuRigidBodyView.h"
#include "tensors/gpu/GpuSimulationView.h"

#include "tensors/GlobalsAreBad.h"
#include "tensors/CommonTypes.h"
#include "tensors/SimulationBackend.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

#include <omni/physics/tensors/TensorUtils.h>

#include <set>

using omni::physics::tensors::checkTensorDevice;
using omni::physics::tensors::checkTensorFloat32;
using omni::physics::tensors::checkTensorInt32;
using omni::physics::tensors::checkTensorSizeExact;
using omni::physics::tensors::checkTensorSizeMinimum;
using omni::physics::tensors::getTensorTotalSize;

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

GpuRigidBodyView::GpuRigidBodyView(GpuSimulationView* sim, const std::vector<RigidBodyEntry>& entries, int device)
    : BaseRigidBodyView(sim, entries), mDevice(device)
{
    if (sim)
    {
        mGpuSimData = sim->getGpuSimulationData();
    }

    PxU32 numBodies = getCount();
    // TODO: clean up the following class members once direct GPU is available for reading articulation mass props
    rbRecords.resize(numBodies);

    // figure out the body types and indices
    std::vector<PxU32> rbIndices(numBodies);
    std::vector<PxRigidDynamicGPUIndex> rdGpuIndices;
    std::map<PxU32, PxU32> linkMap; // maps articulation links to arti gpu indices
    std::set<PxU32> artiSet; // indices of articulations whose links are in this view
    PxU32 numArtiRoots = 0;
    PxU32 rdEntryCount = 0; // total rigid-dynamic entries (enabled or not)
    for (PxU32 i = 0; i < numBodies; i++)
    {
        GpuRigidBodyRecord& rb = rbRecords[i];
        rbIndices[i] = i;

        if (mEntries[i].subspace)
        {
            const carb::Float3& origin = mEntries[i].subspace->origin;
            rb.origin = { origin.x, origin.y, origin.z };
        }
        else
        {
            rb.origin = { 0.0f, 0.0f, 0.0f };
        }

        if (mEntries[i].type == RigidBodyType::eRigidDynamic)
        {
            ++rdEntryCount;
            PxRigidDynamic* rd = static_cast<PxRigidDynamic*>(mEntries[i].body);
            const bool isDisabled = rd->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION);
            // Resolve this body's row in the GPU simulation buffers via the
            // actor-pointer map. The actor pointer is stable across
            // disable/enable cycles; the island node index is not (PhysX
            // removes disabled bodies from the island system).
            std::unordered_map<::physx::PxRigidDynamic*, uint32_t>::iterator actorIt = mGpuSimData->mActor2RdIndexMap.find(rd);
            PxU32 rdIdx = 0xffffffff;
            if (actorIt != mGpuSimData->mActor2RdIndexMap.end())
                rdIdx = actorIt->second;

            if (rdIdx == 0xffffffff)
            {
                // Actor not present in either map -- body was added after
                // GpuSimulationData init. Genuine error.
                CARB_LOG_ERROR("Internal error: Unresolved rigid dynamic index!");
            }
            else if (isDisabled)
            {
                // PxDirectGPUAPI does not populate buffer rows for eDISABLE_SIMULATION actors, so
                // mark physxRdIdx/tensorRdIdx as sentinel: downstream paths (fetchRbVelAcc,
                // updateCMassData, applyForces) then skip this row instead of reading GPU memory.
                // On re-enable, refreshRdGpuIndices() resolves the live index and invalidates the
                // coms cache so updateCMassData() recomputes the (previously zero) COM.
                rb.physxRdIdx = 0xffffffff;
                rb.tensorRdIdx = 0xffffffff;
            }
            else
            {
                rb.physxRdIdx = rd->getGPUIndex(); // match refreshRdGpuIndices convention
                rb.tensorRdIdx = PxU32(rdGpuIndices.size());
                rdGpuIndices.push_back(rd->getGPUIndex());
            }
        }
        else if (mEntries[i].type == RigidBodyType::eArticulationLink)
        {
            PxArticulationLink* link = static_cast<PxArticulationLink*>(mEntries[i].body);
            PxArticulationReducedCoordinate* arti = &link->getArticulation();
            PxArticulationGPUIndex artiIdx = arti->getGPUIndex();
            PxU32 linkIdx = link->getLinkIndex();
            PxU32 numLinks = arti->getNbLinks();
            if (numLinks > mMaxLinks)
            {
                mMaxLinks = numLinks;
            }

            rb.linkIdx = linkIdx;
            rb.physxArtiIdx = artiIdx;
            rb.physxLinkIdx = artiIdx * mGpuSimData->mMaxLinks + linkIdx;
            if (linkIdx == 0)
            {
                // root link
                rb.isRootLink = true;
                ++numArtiRoots;
            }
            artiSet.insert(artiIdx);
            linkMap[i] = artiIdx;
        }
    }

    mNumRds = PxU32(rdGpuIndices.size());

    std::vector<PxU32> artiIndices(artiSet.begin(), artiSet.end());
    std::map<PxU32, PxU32> artiMap; //Maps articulation physx gpu index to unique indices of articulations in the view
    for (PxU32 i = 0; i < artiIndices.size(); i++)
        artiMap[artiIndices[i]] = i;

    for (auto it = linkMap.begin(); it != linkMap.end(); ++it)
    {
        PxU32 viewIdx = it->first;
        PxU32 artiIdx = it->second;
        GpuRigidBodyRecord& rb = rbRecords[viewIdx];
        rb.tensorArtiIdx = artiMap[artiIdx];
    }

    mNumArtis = PxU32(artiIndices.size());
    mNumArtiRoots = numArtiRoots;
    if (mNumRds > 0 || mNumArtis > 0)
    {
        // cMass is indexed by entry/record index (see updateCMassData and submitRbForcesKernel's
        // coms[rbIdx]), so it must be sized by the full entry count. Sizing it compacted
        // (mNumRds + arti links) under-allocates when the view includes disabled bodies -- mNumRds
        // drops below the entry count and an enabled entry after a disabled one writes/reads OOB.
        prepareDeviceData((void**)&cMassLocalPosePosDev, nullptr, numBodies * sizeof(PxVec3),
                          "cMassLocalPosePosDev");
    }
    cMassLocalPosePos.resize(numBodies, { 0.0f, 0.0f, 0.0f });
    // Only upload if there are live GPU rows -- cMassLocalPosePosDev is not
    // allocated when mNumRds == 0 && mNumArtis == 0 (all-disabled view).
    if (mNumRds > 0 || mNumArtis > 0)
        updateCMassData();

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    prepareDeviceData((void**)&mRbIndicesDev, rbIndices.data(), numBodies * sizeof(PxU32), "mRbIndicesDev");
    // rb indexing data
    prepareDeviceData((void**)&mRbRecordsDev, rbRecords.data(), numBodies * sizeof(GpuRigidBodyRecord), "mRbRecordsDev");

    // Disabled-body pose-patch scatter scratch (sized by the max possible patched
    // count = all bodies). See getTransforms(). (OMPE-94459)
    prepareDeviceData((void**)&mDisabledPatchDev, nullptr, numBodies * sizeof(TensorTransform), "mDisabledPatchDev");
    prepareDeviceData((void**)&mDisabledScatterIdxDev, nullptr, numBodies * sizeof(PxU32), "mDisabledScatterIdxDev");

    // Size the rd buffers by the total rigid-dynamic entry count, not just the
    // initially-enabled count: refreshRdGpuIndices() rebuilds the compacted
    // list each read and mNumRds can grow up to rdEntryCount when bodies
    // re-enable (OMPE-94459), so the allocation must cover the maximum.
    // Zero-init the device buffer first (rdGpuIndices only holds the
    // currently-enabled entries; copying rdEntryCount bytes from it would
    // over-read the vector when some rigid dynamics start disabled), then
    // copy only the populated prefix.
    if (rdEntryCount > 0)
    {
        prepareDeviceData((void**)&mRdGpuIndicesDev, nullptr, rdEntryCount * sizeof(PxRigidDynamicGPUIndex),
                          "mRdGpuIndicesDev");
        if (!rdGpuIndices.empty())
        {
            CHECK_CUDA(cudaMemcpy(mRdGpuIndicesDev, rdGpuIndices.data(),
                                  rdGpuIndices.size() * sizeof(PxRigidDynamicGPUIndex),
                                  cudaMemcpyHostToDevice));
        }
        prepareDeviceData(
            (void**)&mDirtyRdGpuIndices, nullptr, rdEntryCount * sizeof(PxRigidDynamicGPUIndex), "mDirtyRdGpuIndices");
        prepareDeviceData((void**)&mRdDirtyFlagsDev, nullptr, rdEntryCount * sizeof(ActorGpuFlags), "mRdDirtyFlagsDev");
        mRdGpuIndicesHost.reserve(rdEntryCount);
    }

    if (mNumArtis > 0)
    {
        prepareDeviceData(
            (void**)&mArtiDirtyFlagsDev, nullptr, mNumArtis * sizeof(PxArticulationCacheFlags), "mArtiDirtyFlagsDev");
        prepareDeviceData((void**)&mArtiLinksDirtyFlagsDev, nullptr,
                          mNumArtis * mMaxLinks * sizeof(PxArticulationCacheFlags),
                          "mArtiLinksDirtyFlagsDev");
        prepareDeviceData((void**)&mDirtyArtiGpuIndices, nullptr, mNumArtis * sizeof(PxArticulationGPUIndex), "mDirtyArtiGpuIndices");
        prepareDeviceData((void**)&mArtiIndicesDev, artiIndices.data(), mNumArtis * sizeof(PxArticulationGPUIndex), "mArtiIndicesDev");
    }
    clearDataFlagsAndIndices();
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
}

GpuRigidBodyView::~GpuRigidBodyView()
{
    if (mGpuSimData)
    {
        CudaContextGuard ctxGuard(mGpuSimData->mCtx);

        CHECK_CUDA(cudaFree(mRbIndicesDev));
        CHECK_CUDA(cudaFree(mRdGpuIndicesDev));
        CHECK_CUDA(cudaFree(mArtiIndicesDev));
        CHECK_CUDA(cudaFree(mRbRecordsDev));
        CHECK_CUDA(cudaFree(mDisabledPatchDev));
        CHECK_CUDA(cudaFree(mDisabledScatterIdxDev));
        CHECK_CUDA(cudaFree(mDirtyArtiGpuIndices));
        CHECK_CUDA(cudaFree(mDirtyRdGpuIndices));
        CHECK_CUDA(cudaFree(mRdDirtyFlagsDev));
        CHECK_CUDA(cudaFree(mArtiDirtyFlagsDev));
        CHECK_CUDA(cudaFree(mArtiLinksDirtyFlagsDev));
        CHECK_CUDA(cudaFree(cMassLocalPosePosDev));
        if (mMaskIndicesDev)
            CHECK_CUDA(cudaFree(mMaskIndicesDev));
        if (mMaskAllocPolicy.mBuffer)
            CHECK_CUDA(cudaFree(mMaskAllocPolicy.mBuffer));
    }
}

bool GpuRigidBodyView::clearDataFlagsAndIndices()
{
    if (mNumArtis > 0)
    {
        if (!CHECK_CUDA(cudaMemset(mDirtyArtiGpuIndices, 0, mNumArtis * sizeof(PxArticulationGPUIndex))))
        {
            return false;
        }
        if (!CHECK_CUDA(cudaMemset(mArtiDirtyFlagsDev, 0, mNumArtis * sizeof(PxArticulationCacheFlags))))
        {
            return false;
        }
        if (!CHECK_CUDA(cudaMemset(
                mArtiLinksDirtyFlagsDev, 0, mNumArtis * mMaxLinks * sizeof(PxArticulationCacheFlags))))
        {
            return false;
        }
    }
    if (mNumRds > 0)
    {
        if (!CHECK_CUDA(cudaMemset(mRdDirtyFlagsDev, 0, mNumRds * sizeof(ActorGpuFlags))))
        {
            return false;
        }
        if (!CHECK_CUDA(cudaMemset(mDirtyRdGpuIndices, 0, mNumRds * sizeof(PxRigidDynamicGPUIndex))))
        {
            return false;
        }
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
}


bool GpuRigidBodyView::getTransforms(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "transform", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "transform", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 7u, "transform", __FUNCTION__))
    {
        return false;
    }

    refreshRdGpuIndices();

    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    SYNCHRONIZE_CUDA();

    CUevent rdCopyEvent = nullptr;
    CUevent artiCopyEvent = nullptr;

    if (mNumRds > 0)
    {
        scene->getDirectGPUAPI().getRigidDynamicData((void*) mGpuSimData->mRdPoseDev, mRdGpuIndicesDev,
                                                     PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE, mNumRds, nullptr,
                                                     rdCopyEvent);
    }

    if (mNumArtis > 0)
    {
        scene->getDirectGPUAPI().getArticulationData((void*) mGpuSimData->mLinkOrRootTransformsDev, mArtiIndicesDev,
                                                     PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, mNumArtis,
                                                     nullptr, artiCopyEvent);
    }

    if (rdCopyEvent)
    {
        CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(rdCopyEvent), 0, nullptr));
    }

    if (artiCopyEvent)
    {
        CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(artiCopyEvent), 0, nullptr));
    }

    SYNCHRONIZE_CUDA();

    if (!fetchRbTransforms(static_cast<TensorTransform*>(dstTensor->data),  mGpuSimData->mRdPoseDev,  mGpuSimData->mLinkOrRootTransformsDev,
                           getCount(),  mGpuSimData->mMaxLinks, mRbRecordsDev))
    {
        CARB_LOG_ERROR("Failed to fetch rigid body tranforms");
        return false;
    }

    // OMPE-94459: a currently-disabled rigid dynamic was compacted out of the
    // DirectGPU read (no GPU slot), so the kernel left it at identity. Its pose
    // is frozen at the core value -- patch each disabled rd with getGlobalPose()
    // so the readback matches the CPU view instead of reading zero/stale rows.
    // Skip the whole scan when nothing is disabled (the common case); the flag
    // is refreshed by refreshRdGpuIndices() above.
    if (mHasDisabledRd)
    {
        // Build the compacted patches + their dst body indices on the host, upload
        // both once, then a single scatter kernel writes them into dst -- instead
        // of a tiny synchronous cudaMemcpy per disabled body in this read hot path.
        std::vector<TensorTransform> hostPatches;
        std::vector<PxU32> hostIdx;
        hostPatches.reserve(mEntries.size());
        hostIdx.reserve(mEntries.size());
        for (PxU32 i = 0; i < mEntries.size(); i++)
        {
            if (mEntries[i].type != RigidBodyType::eRigidDynamic || rbRecords[i].tensorRdIdx != 0xffffffff)
                continue;
            const PxTransform pose = mEntries[i].body->getGlobalPose();
            TensorTransform t;
            t.p = pose.p - rbRecords[i].origin;
            t.q = pose.q;
            hostPatches.push_back(t);
            hostIdx.push_back(i);
        }
        const PxU32 numPatched = PxU32(hostPatches.size());
        if (numPatched > 0)
        {
            TensorTransform* patchDev = static_cast<TensorTransform*>(mDisabledPatchDev);
            CHECK_CUDA(cudaMemcpy(patchDev, hostPatches.data(), numPatched * sizeof(TensorTransform),
                                  cudaMemcpyHostToDevice));
            CHECK_CUDA(cudaMemcpy(mDisabledScatterIdxDev, hostIdx.data(), numPatched * sizeof(PxU32),
                                  cudaMemcpyHostToDevice));
            scatterRbTransforms(static_cast<TensorTransform*>(dstTensor->data), patchDev,
                                mDisabledScatterIdxDev, numPatched);
        }
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

static bool getVelAcc(const TensorDesc* dstTensor,
                      PxVec3* rdDataLinearDev,
                      PxVec3* rdDataAngularDev,
                      PxVec3* linkDataLinearDev,
                      PxVec3* linkDataAngularDev,
                      const GpuRigidBodyRecord* rbRecordsDev,
                      const PxRigidDynamicGPUIndex* rdGpuIndicesDev,
                      const PxArticulationGPUIndex* artiIndicesDev,
                      const PxRigidDynamicGPUAPIReadType::Enum rdLinearType,
                      const PxRigidDynamicGPUAPIReadType::Enum rdAngularType,
                      const PxArticulationGPUAPIReadType::Enum linkLinearType,
                      const PxArticulationGPUAPIReadType::Enum linkAngularType,
                      const PxU32 numRds,
                      const PxU32 numArtis,
                      const PxU32 numRb,
                      const PxU32 simMaxLinks,
                      const int device,
                      GpuSimulationDataPtr gpuSimData)
{
    PxScene* scene = gpuSimData->mScene;
    PhysxCudaContextGuard ctxGuarg(gpuSimData->mCudaContextManager);

    SYNCHRONIZE_CUDA();
    CUevent rdCopyEventLin = nullptr;
    CUevent rdCopyEventAng = nullptr;
    CUevent artiCopyEventLin = nullptr;
    CUevent artiCopyEventAng = nullptr;
    if (numRds > 0)
    {
        scene->getDirectGPUAPI().getRigidDynamicData(
            (void*)rdDataLinearDev, rdGpuIndicesDev, rdLinearType, numRds, nullptr, rdCopyEventLin);
        scene->getDirectGPUAPI().getRigidDynamicData(
            (void*)rdDataAngularDev, rdGpuIndicesDev, rdAngularType, numRds, nullptr, rdCopyEventAng);
    }
    SYNCHRONIZE_CUDA();
    if (numArtis > 0)
    {
        scene->getDirectGPUAPI().getArticulationData(
            (void*)linkDataLinearDev, artiIndicesDev, linkLinearType, numArtis, nullptr, artiCopyEventLin);
        scene->getDirectGPUAPI().getArticulationData(
            (void*)linkDataAngularDev, artiIndicesDev, linkAngularType, numArtis, nullptr, artiCopyEventAng);
    }

    if (rdCopyEventLin)
    {
        CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(rdCopyEventLin), 0, nullptr));
    }
    if (rdCopyEventAng)
    {
        CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(rdCopyEventAng), 0, nullptr));
    }

    if (artiCopyEventLin)
    {
        CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(artiCopyEventLin), 0, nullptr));
    }
    if (artiCopyEventAng)
    {
        CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(artiCopyEventAng), 0, nullptr));
    }

    SYNCHRONIZE_CUDA();

    if (!fetchRbVelAcc(static_cast<TensorVelAcc*>(dstTensor->data), rdDataLinearDev, rdDataAngularDev,
                       linkDataLinearDev, linkDataAngularDev, numRb,  simMaxLinks, rbRecordsDev))
    {
        CARB_LOG_ERROR("Failed to fetch rigid body velocities or acceleration");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}


void GpuRigidBodyView::refreshRdGpuIndices() const
{
    // Fast path: if no setDisableSimulations call has occurred since the last
    // sync, skip the O(n) getGPUIndex() loop entirely. getGPUIndex() is a
    // 3-level pointer dereference (NpRigidDynamic -> BodySim -> mNodeIndex)
    // with one cache miss per body; running it on every tensor read in a
    // disable-free scene adds measurable CPU overhead for large body counts.
    //
    // Multi-view caveat (OMPE-94459): mRdIndexDirty is per-view, but a disable
    // issued through a *sibling* view over the same scene mutates the shared GPU
    // island indices without touching this view's flag. Every successful
    // setDisableSimulations bumps a scene-wide epoch in the shared
    // GpuSimulationData; if it has advanced past what this view last synced to,
    // a sibling toggled a body -- force a rebuild so we don't read stale/invalid
    // indices. This view's own setDisableSimulations also bumps the epoch, but
    // it already sets mRdIndexDirty directly, so the epoch check is the path that
    // catches sibling-driven changes.
    if (mGpuSimData && mGpuSimData->mRdDisableEpoch != mLastSeenRdDisableEpoch)
    {
        mLastSeenRdDisableEpoch = mGpuSimData->mRdDisableEpoch;
        mRdIndexDirty = true;
    }
    if (!mRdIndexDirty)
        return;

    // OMPE-94459: a rigid dynamic's GPU index is its island node handle,
    // which PhysX frees on disable (eDISABLE_SIMULATION / scene removal) and
    // reallocates -- generally to a different value, via a LIFO free list --
    // on re-enable. getGPUIndex() returns PX_INVALID_NODE (0xffffffff) while
    // disabled. The view cannot cache this mapping across reads (there is no
    // engine callback for enable/disable, and a body can toggle between two
    // reads), so rebuild the compacted rd-index list from the live
    // getGPUIndex() every read, keyed off the stable PxRigidActor pointer in
    // mEntries. Disabled bodies (invalid index) compact out and get
    // tensorRdIdx/physxRdIdx = sentinel, so the fetch/apply kernels zero or
    // skip them. physxRdIdx is only ever tested against the sentinel (never
    // used as an index), so the live value serves as the enabled marker.
    constexpr PxU32 kInvalid = 0xffffffffu;
    bool changed = false;
    bool comsNeedInvalidate = false;
    bool anyDisabledRd = false;
    // Track the changed-record sub-range so we upload only what moved (below)
    // instead of the whole numBodies array.
    PxU32 firstChanged = kInvalid;
    PxU32 lastChanged = 0;
    mRdGpuIndicesHost.clear();
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        if (mEntries[i].type != RigidBodyType::eRigidDynamic)
        {
            continue;
        }
        PxRigidDynamic* rd = static_cast<PxRigidDynamic*>(mEntries[i].body);
        const PxU32 live = rd->getGPUIndex();
        PxU32 newPhysxRd, newTensorRd;
        if (live != kInvalid)
        {
            newPhysxRd = live;
            newTensorRd = PxU32(mRdGpuIndicesHost.size());
            mRdGpuIndicesHost.push_back(live);
        }
        else
        {
            newPhysxRd = kInvalid;
            newTensorRd = kInvalid;
            anyDisabledRd = true;
        }
        if (rbRecords[i].physxRdIdx != newPhysxRd || rbRecords[i].tensorRdIdx != newTensorRd)
        {
            // A record flipping from the sentinel back to a live index means
            // the body was just re-enabled. While it was disabled,
            // updateCMassData() skipped it and left cMassLocalPosePos = {0,0,0},
            // so the coms cache must be invalidated to force a recompute on the
            // next applyForcesAndTorquesAtPosition; otherwise that path computes
            // the lever arm about the body origin instead of the COM. (OMPE-94459)
            if (rbRecords[i].physxRdIdx == kInvalid && newPhysxRd != kInvalid)
            {
                comsNeedInvalidate = true;
            }
            rbRecords[i].physxRdIdx = newPhysxRd;
            rbRecords[i].tensorRdIdx = newTensorRd;
            changed = true;
            if (firstChanged == kInvalid)
            {
                firstChanged = i;
            }
            lastChanged = i;
        }
    }
    mNumRds = PxU32(mRdGpuIndicesHost.size());
    // Lets the read hot path (getTransforms) skip the disabled-body patch scan
    // entirely when nothing is disabled -- the common case.
    mHasDisabledRd = anyDisabledRd;
    if (comsNeedInvalidate)
    {
        // Direct member access: setComsCacheStateValid() is non-const and
        // refreshRdGpuIndices() is const (called from const read-path overrides
        // of IRigidBodyView). validComsCache is mutable for this purpose.
        validComsCache = false;
    }
    if (!changed)
    {
        return;
    }
    // cudaMemcpy needs the view's CUDA context current; callers acquire
    // PhysxCudaContextGuard later for their own DirectGPU calls, but this
    // helper runs before that point in getTransforms/getVelocities/etc.
    // Match the convention used by the staged setters below.
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    if (mNumRds > 0)
    {
        CHECK_CUDA(cudaMemcpy(mRdGpuIndicesDev, mRdGpuIndicesHost.data(),
                              mNumRds * sizeof(PxRigidDynamicGPUIndex), cudaMemcpyHostToDevice));
    }
    // Upload only the changed record sub-range rather than all numBodies
    // records: a workload toggling disable each step changes only the toggled
    // rows, and this helper runs at the top of every read/write entry point, so
    // a full-array HtoD here would re-push the whole array up to 3x/step. The
    // ctor seeds the entire device array once (prepareDeviceData above), so the
    // unchanged rows stay valid and only [firstChanged, lastChanged] needs
    // re-upload. `changed` implies firstChanged was set. (OMPE-94459)
    const PxU32 rangeCount = lastChanged - firstChanged + 1;
    CHECK_CUDA(cudaMemcpy(mRbRecordsDev + firstChanged, rbRecords.data() + firstChanged,
                          rangeCount * sizeof(GpuRigidBodyRecord), cudaMemcpyHostToDevice));
    // Clear dirty only when all bodies have valid GPU indices (mHasDisabledRd==false).
    // While any body is disabled -- or was just re-enabled and is waiting for PhysX
    // to assign its new GPU index after the next simulate -- keep dirty=true so the
    // loop re-runs on the next read and picks up the assignment when it lands.
    // Once all bodies are stably enabled the loop stops running until the next
    // setDisableSimulations call.
    if (!mHasDisabledRd)
        mRdIndexDirty = false;
}

bool GpuRigidBodyView::getVelocities(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "velocity", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 6u, "velocity", __FUNCTION__))
    {
        return false;
    }

    refreshRdGpuIndices();
    return getVelAcc(
        dstTensor,  mGpuSimData->mRdLinearVelAccDev,  mGpuSimData->mRdAngularVelAccDev,  mGpuSimData->mLinkOrRootLinearVelAccDev,  mGpuSimData->mLinkOrRootAngularVelAccDev,
        mRbRecordsDev, mRdGpuIndicesDev, mArtiIndicesDev, PxRigidDynamicGPUAPIReadType::eLINEAR_VELOCITY,
        PxRigidDynamicGPUAPIReadType::eANGULAR_VELOCITY, PxArticulationGPUAPIReadType::eLINK_LINEAR_VELOCITY,
        PxArticulationGPUAPIReadType::eLINK_ANGULAR_VELOCITY, mNumRds, mNumArtis, getCount(),  mGpuSimData->mMaxLinks, mDevice,
        mGpuSimData);
}


bool GpuRigidBodyView::getAccelerations(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "acceleration", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "acceleration", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 6u, "acceleration", __FUNCTION__))
    {
        return false;
    }

    refreshRdGpuIndices();
    return getVelAcc(
        dstTensor,  mGpuSimData->mRdLinearVelAccDev,  mGpuSimData->mRdAngularVelAccDev,  mGpuSimData->mLinkOrRootLinearVelAccDev,  mGpuSimData->mLinkOrRootAngularVelAccDev,
        mRbRecordsDev, mRdGpuIndicesDev, mArtiIndicesDev, PxRigidDynamicGPUAPIReadType::eLINEAR_ACCELERATION,
        PxRigidDynamicGPUAPIReadType::eANGULAR_ACCELERATION, PxArticulationGPUAPIReadType::eLINK_LINEAR_ACCELERATION,
        PxArticulationGPUAPIReadType::eLINK_ANGULAR_ACCELERATION, mNumRds, mNumArtis, getCount(), mGpuSimData->mMaxLinks, mDevice,
        mGpuSimData);
}

bool GpuRigidBodyView::setKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CARB_LOG_ERROR("GPU Rigid Body View kinematic target setting not implemented.");
    return false;
}

bool GpuRigidBodyView::setTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    refreshRdGpuIndices();

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, mDevice, "transform", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "transform", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * 7u, "transform", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, mDevice, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mRbIndicesDev;
        numIndices = getCount();
    }

    if (getCount() - mNumRds - mNumArtiRoots > 0)
    {
        CARB_LOG_WARN("The RigidBodyView contains non-root articulation links whose transforms cannot be set directly");
    }
    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    clearDataFlagsAndIndices();
    SYNCHRONIZE_CUDA();
    if (!submitRbTransforms( mGpuSimData->mRdPoseDev,  mGpuSimData->mLinkOrRootTransformsDev, mRdDirtyFlagsDev, mArtiDirtyFlagsDev,
                            static_cast<const TensorTransform*>(srcTensor->data), indices, numIndices, getCount(),
                            mRbRecordsDev))
    {
        CARB_LOG_ERROR("Failed to submit rigid body transforms");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    SYNCHRONIZE_CUDA();

    PxU32 numDirtyArtiIndices =
        fillArtiTransforms(mArtiIndexSingleAllocPolicy, mGpuSimData->mLinkOrRootTransformsDev, mDirtyArtiGpuIndices,
                           mArtiIndicesDev, mArtiDirtyFlagsDev, ArticulationGpuFlag::eROOT_TRANSFORM, mNumArtis);

    PxU32 numDirtyRdIndices = fillRdTransforms(mRdIndexSingleAllocPolicy, mGpuSimData->mRdPoseDev, mDirtyRdGpuIndices,
                                               mRdGpuIndicesDev, mRdDirtyFlagsDev, ActorGpuFlag::eACTOR_DATA, mNumRds);

    if (numDirtyArtiIndices > 0)
        scene->getDirectGPUAPI().setArticulationData((void*)mGpuSimData->mLinkOrRootTransformsDev, mDirtyArtiGpuIndices,
                                                     PxArticulationGPUAPIWriteType::eROOT_GLOBAL_POSE,
                                                     numDirtyArtiIndices,
                                                     mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiRootTransforms],
                                                     mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootTransforms]);

    if (numDirtyRdIndices > 0)
        scene->getDirectGPUAPI().setRigidDynamicData((void*) mGpuSimData->mRdPoseDev, mDirtyRdGpuIndices,
                                                     PxRigidDynamicGPUAPIWriteType::eGLOBAL_POSE, numDirtyRdIndices,
                                                     mGpuSimData->mApplyWaitEvents[ApplyEvent::eRdData],
                                                     mGpuSimData->mApplySignalEvents[ApplyEvent::eRdData]);

    CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eRdData]), nullptr));
    CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootTransforms]), nullptr));
    SYNCHRONIZE_CUDA();

    return true;
}

bool GpuRigidBodyView::setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    refreshRdGpuIndices();

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, mDevice, "velocity", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * 6u, "velocity", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, mDevice, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mRbIndicesDev;
        numIndices = getCount();
    }

    if (getCount() - mNumRds - mNumArtiRoots > 0)
    {
        CARB_LOG_WARN("The RigidBodyView contains non-root articulation links whose velocities cannot be set directly");
    }
    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    clearDataFlagsAndIndices();
    SYNCHRONIZE_CUDA();
    if (!submitRbVelocities(mGpuSimData->mRdLinearVelAccDev, mGpuSimData->mRdAngularVelAccDev,
                            mGpuSimData->mLinkOrRootLinearVelAccDev, mGpuSimData->mLinkOrRootAngularVelAccDev,
                            mRdDirtyFlagsDev, mArtiDirtyFlagsDev, static_cast<const TensorVelAcc*>(srcTensor->data),
                            indices, numIndices, getCount(), mRbRecordsDev))
    {
        CARB_LOG_ERROR("Failed to submit rigid body velocities");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    SYNCHRONIZE_CUDA();
    PxU32 numDirtyArtiIndices = fillArtiVelocities(
        mArtiIndexSingleAllocPolicy, mGpuSimData->mLinkOrRootLinearVelAccDev, mGpuSimData->mLinkOrRootAngularVelAccDev,
        mDirtyArtiGpuIndices, mArtiIndicesDev, mArtiDirtyFlagsDev, ArticulationGpuFlag::eROOT_VELOCITY, mNumArtis);

    PxU32 numDirtyRdIndices =
        fillRdVelocities(mRdIndexSingleAllocPolicy, mGpuSimData->mRdLinearVelAccDev, mGpuSimData->mRdAngularVelAccDev,
                         mDirtyRdGpuIndices, mRdGpuIndicesDev, mRdDirtyFlagsDev, ActorGpuFlag::eACTOR_DATA, mNumRds);

    if (numDirtyArtiIndices > 0)
    {
        scene->getDirectGPUAPI().setArticulationData(
            (void*) mGpuSimData->mLinkOrRootLinearVelAccDev, mDirtyArtiGpuIndices,
            PxArticulationGPUAPIWriteType::eROOT_LINEAR_VELOCITY, numDirtyArtiIndices,
            mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiRootLinVelocities],
            mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootLinVelocities]);
        scene->getDirectGPUAPI().setArticulationData(
            (void*) mGpuSimData->mLinkOrRootAngularVelAccDev, mDirtyArtiGpuIndices,
            PxArticulationGPUAPIWriteType::eROOT_ANGULAR_VELOCITY, numDirtyArtiIndices,
            mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiRootAngVelocities],
            mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootAngVelocities]);
    }
    if (numDirtyRdIndices > 0)
    {
        scene->getDirectGPUAPI().setRigidDynamicData(
            (void*) mGpuSimData->mRdLinearVelAccDev, mDirtyRdGpuIndices,
            PxRigidDynamicGPUAPIWriteType::eLINEAR_VELOCITY, numDirtyRdIndices,
            mGpuSimData->mApplyWaitEvents[ApplyEvent::eRdLinVelocities], mGpuSimData->mApplySignalEvents[ApplyEvent::eRdLinVelocities]);
        scene->getDirectGPUAPI().setRigidDynamicData(
            (void*) mGpuSimData->mRdAngularVelAccDev, mDirtyRdGpuIndices,
            PxRigidDynamicGPUAPIWriteType::eANGULAR_VELOCITY, numDirtyRdIndices,
            mGpuSimData->mApplyWaitEvents[ApplyEvent::eRdAngVelocities], mGpuSimData->mApplySignalEvents[ApplyEvent::eRdAngVelocities]);
    }
    CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootLinVelocities]), nullptr));
    CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootAngVelocities]), nullptr));
    CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eRdLinVelocities]), nullptr));
    CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eRdAngVelocities]), nullptr));
    SYNCHRONIZE_CUDA();

    return true;
}

bool GpuRigidBodyView::applyForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CARB_LOG_WARN("Deprecated function IArticulationView::applyForces, please use IArticulationView::applyForcesAndTorquesAtPosition instead.");
    return applyForcesAndTorquesAtPosition(srcTensor, nullptr, nullptr, indexTensor, true);
}

// TODO: copyActorAndLinksTransorms appears to be dead code (no callers); delete in a follow-up MR.
void GpuRigidBodyView::copyActorAndLinksTransorms()
{
    SYNCHRONIZE_CUDA();
    PxScene* scene = mGpuSimData->mScene;

    CUevent rdCopyEvent = nullptr;
    CUevent artiCopyEvent = nullptr;

    if (mNumRds > 0)
    {
        scene->getDirectGPUAPI().getRigidDynamicData((void*) mGpuSimData->mRdPoseDev, mRdGpuIndicesDev,
                                                     PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE, mNumRds, nullptr,
                                                     rdCopyEvent);
    }

    if (mNumArtis > 0)
    {
        scene->getDirectGPUAPI().getArticulationData((void*) mGpuSimData->mLinkOrRootTransformsDev, mArtiIndicesDev,
                                                     PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, mNumArtis,
                                                     nullptr, artiCopyEvent);
    }

    if (rdCopyEvent)
    {
        CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(rdCopyEvent), 0, nullptr));
    }

    if (artiCopyEvent)
    {
        CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(artiCopyEvent), 0, nullptr));
    }

    SYNCHRONIZE_CUDA();
}
bool GpuRigidBodyView::updateCMassData()
{
    for (PxU32 idx = 0; idx < mEntries.size(); idx++)
    {
        if (rbRecords[idx].physxLinkIdx != 0xffffffff)
        {
            cMassLocalPosePos[idx] = mEntries[idx].body->getCMassLocalPose().p;
        }
        else if (rbRecords[idx].physxRdIdx != 0xffffffff)
        {
            cMassLocalPosePos[idx] = mEntries[idx].body->getCMassLocalPose().p;
        }
    }
    if (!CHECK_CUDA(cudaMemcpy(cMassLocalPosePosDev, cMassLocalPosePos.data(),
                               mEntries.size() * sizeof(PxVec3), cudaMemcpyHostToDevice)))
    {
        return false;
    }
    setComsCacheStateValid(true);
    return true;
}


bool GpuRigidBodyView::applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                                       const TensorDesc* srcTorqueTensor,
                                                       const TensorDesc* srcPositionTensor,
                                                       const TensorDesc* indexTensor,
                                                       const bool isGlobal)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    refreshRdGpuIndices();
    bool validForceTensor = false;
    bool validTorqueTensor = false;
    bool validPositionTensor = false;
    bool hasForce = srcForceTensor ? srcForceTensor->data != nullptr : false;
    bool hasTorque = srcTorqueTensor ? srcTorqueTensor->data  != nullptr : false;
    bool hasPosition = srcPositionTensor ? srcPositionTensor->data  != nullptr : false;
    const PxVec3* forceData = nullptr;
    const PxVec3* torqueData = nullptr;
    const PxVec3* positionData = nullptr;

    // skip if both tensors are undefined
    if (!hasForce && !hasTorque)
    {
        return false;
    }
    // skip if tensors are defined but are ill-defined
    if (hasForce)
    {
        validForceTensor = checkTensorDevice(*srcForceTensor, mDevice, "force", __FUNCTION__) &&
                           checkTensorFloat32(*srcForceTensor, "force", __FUNCTION__) &&
                           checkTensorSizeExact(*srcForceTensor, getCount() * 3u, "force", __FUNCTION__);
        if (!validForceTensor)
            return false;
        forceData = static_cast<const PxVec3*>(srcForceTensor->data);
    }

    if (hasTorque)
    {
        validTorqueTensor = checkTensorDevice(*srcTorqueTensor, mDevice, "torque", __FUNCTION__) &&
                            checkTensorFloat32(*srcTorqueTensor, "torque", __FUNCTION__) &&
                            checkTensorSizeExact(*srcTorqueTensor, getCount() * 3u, "torque", __FUNCTION__);
        if (!validTorqueTensor)
            return false;
        torqueData = static_cast<const PxVec3*>(srcTorqueTensor->data);
    }

    if (hasPosition)
    {
        if (!validForceTensor)
        {
            CARB_LOG_ERROR("Received a position tensor wihtout a compatible force tensor.");
            return false;
        }
        validPositionTensor = checkTensorDevice(*srcPositionTensor, mDevice, "torque", __FUNCTION__) &&
                              checkTensorFloat32(*srcPositionTensor, "torque", __FUNCTION__) &&
                              checkTensorSizeExact(*srcPositionTensor, getCount() * 3u, "torque", __FUNCTION__);
        if (!validPositionTensor)
            return false;
        positionData = static_cast<const PxVec3*>(srcPositionTensor->data);
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, mDevice, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mRbIndicesDev;
        numIndices = getCount();
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    // will keep this until direct GPU API for articulation link mass properties is available
    // This does not have to be done if setCMassLocalPose is not called
    if (validPositionTensor && validForceTensor && (mNumArtis > 0 || mNumRds > 0) && !getComsCacheStateValid())
        updateCMassData();
    PxScene* scene = mGpuSimData->mScene;

    // Need to update the transform data because they might have been rewritten
    if (!isGlobal || (validPositionTensor && validForceTensor))
    {
        CUevent rdCopyEvent = nullptr;
        CUevent artiCopyEvent = nullptr;
        if (mNumRds > 0)
        {
            scene->getDirectGPUAPI().getRigidDynamicData((void*) mGpuSimData->mRdPoseDev, mRdGpuIndicesDev,
                                                         PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE, mNumRds, nullptr,
                                                         rdCopyEvent);
        }

        if (mNumArtis > 0)
        {
            scene->getDirectGPUAPI().getArticulationData((void*) mGpuSimData->mLinkOrRootTransformsDev, mArtiIndicesDev,
                                                         PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, mNumArtis,
                                                         nullptr, artiCopyEvent);
        }

        if (rdCopyEvent)
        {
            CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(rdCopyEvent), 0, nullptr));
        }

        if (artiCopyEvent)
        {
            CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(artiCopyEvent), 0, nullptr));
        }
    }

    mGpuSimData->clearForces();
    clearDataFlagsAndIndices();
    SYNCHRONIZE_CUDA();
    if (!submitRbForces(mGpuSimData->mRdForcesDev, mGpuSimData->mRdTorquesDev, mGpuSimData->mLinkForcesDev,
                        mGpuSimData->mLinkTorquesDev, mRdDirtyFlagsDev, mArtiDirtyFlagsDev, mArtiLinksDirtyFlagsDev,
                        mGpuSimData->mLinkOrRootTransformsDev, mGpuSimData->mRdPoseDev, cMassLocalPosePosDev, forceData,
                        torqueData, positionData, indices, numIndices, mGpuSimData->mMaxLinks, mRbRecordsDev, isGlobal,
                        validForceTensor, validTorqueTensor, validPositionTensor))
    {
        CARB_LOG_ERROR("Failed to submit rigid body forces");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    SYNCHRONIZE_CUDA();
    if (validForceTensor)
    {
        PxU32 numDirtyRdIndices = 0;
        PxU32 numDirtyArtiIndices = 0;

        if (mNumRds > 0)
            numDirtyRdIndices = fillRdFT(mRdIndexSingleAllocPolicy, mGpuSimData->mRdForcesDev, mDirtyRdGpuIndices,
                                         mRdGpuIndicesDev, mRdDirtyFlagsDev, ActorGpuFlag::eFORCE, mNumRds);


        if (mNumArtis > 0)
            numDirtyArtiIndices =
                fillArtiFT(mArtiIndexSingleAllocPolicy, mGpuSimData->mLinkForcesDev, mDirtyArtiGpuIndices,
                           mArtiIndicesDev, mArtiDirtyFlagsDev, mArtiLinksDirtyFlagsDev,
                           ArticulationGpuFlag::eLINK_FORCE, mNumArtis, mGpuSimData->mMaxLinks);


        if (numDirtyRdIndices > 0)
        {
            scene->getDirectGPUAPI().setRigidDynamicData((void*)mGpuSimData->mRdForcesDev, mDirtyRdGpuIndices,
                                                         PxRigidDynamicGPUAPIWriteType::eFORCE, numDirtyRdIndices,
                                                         mGpuSimData->mApplyWaitEvents[ApplyEvent::eRdForces],
                                                         mGpuSimData->mApplySignalEvents[ApplyEvent::eRdForces]);
            mGpuSimData->mRdForcesApplied = true;
        }
        if (numDirtyArtiIndices > 0)
        {
            scene->getDirectGPUAPI().setArticulationData((void*)mGpuSimData->mLinkForcesDev, mDirtyArtiGpuIndices,
                                                         PxArticulationGPUAPIWriteType::eLINK_FORCE, numDirtyArtiIndices,
                                                         mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiLinkForces],
                                                         mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkForces]);
            mGpuSimData->mLinkForcesApplied = true;
        }
        // Need to synchronize before memset, becasue above the dirty indices may be in use on a different stream
        CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eRdForces]), 0, nullptr));
        CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkForces]), 0, nullptr));

        CHECK_CUDA(cudaStreamSynchronize(nullptr));
    }
    if (validPositionTensor || validTorqueTensor)
    {
        if (!CHECK_CUDA(cudaMemset(mDirtyRdGpuIndices, 0, mNumRds * sizeof(PxRigidDynamicGPUIndex))) ||
            !CHECK_CUDA(cudaMemset(mDirtyArtiGpuIndices, 0, mNumArtis * sizeof(PxArticulationGPUIndex))))
        {
            return false;
        }
        CHECK_CUDA(cudaStreamSynchronize(nullptr));
        PxU32 numDirtyRdIndices = 0;
        PxU32 numDirtyArtiIndices = 0;

        if (mNumRds > 0)
            numDirtyRdIndices = fillRdFT(mRdIndexSingleAllocPolicy, mGpuSimData->mRdTorquesDev, mDirtyRdGpuIndices,
                                         mRdGpuIndicesDev, mRdDirtyFlagsDev, ActorGpuFlag::eTORQUE, mNumRds);
        if (mNumArtis > 0)
            numDirtyArtiIndices =
                fillArtiFT(mArtiIndexSingleAllocPolicy, mGpuSimData->mLinkTorquesDev, mDirtyArtiGpuIndices,
                           mArtiIndicesDev, mArtiDirtyFlagsDev, mArtiLinksDirtyFlagsDev,
                           ArticulationGpuFlag::eLINK_TORQUE, mNumArtis, mGpuSimData->mMaxLinks);

        if (numDirtyRdIndices > 0)
        {
            scene->getDirectGPUAPI().setRigidDynamicData((void*)mGpuSimData->mRdTorquesDev, mDirtyRdGpuIndices,
                                                         PxRigidDynamicGPUAPIWriteType::eTORQUE, numDirtyRdIndices,
                                                         mGpuSimData->mApplyWaitEvents[ApplyEvent::eRdTorques],
                                                         mGpuSimData->mApplySignalEvents[ApplyEvent::eRdTorques]);
            mGpuSimData->mRdTorquesApplied = true;
        }
        if (numDirtyArtiIndices > 0)
        {
            scene->getDirectGPUAPI().setArticulationData(
                (void*)mGpuSimData->mLinkTorquesDev, mDirtyArtiGpuIndices, PxArticulationGPUAPIWriteType::eLINK_TORQUE,
                numDirtyArtiIndices, mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiLinkTorques],
                mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkTorques]);
            mGpuSimData->mLinkTorquesApplied = true;
        }
        CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eRdTorques]), 0, nullptr));
        CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkTorques]), 0, nullptr));
        CHECK_CUDA(cudaStreamSynchronize(nullptr));
    }
    SYNCHRONIZE_CUDA();

    return true;
}

bool GpuRigidBodyView::resolveMask(const TensorDesc* maskTensor, PxU32& outK) const
{
    if (!maskTensor || !maskTensor->data)
    {
        CARB_LOG_ERROR("mask tensor is null or has no data in %s", __FUNCTION__);
        return false;
    }

    if (!checkTensorDevice(*maskTensor, mDevice, "mask", __FUNCTION__))
        return false;

    if (maskTensor->dtype != omni::physics::tensors::TensorDataType::eUint8)
    {
        CARB_LOG_ERROR("mask tensor must be uint8 in %s", __FUNCTION__);
        return false;
    }

    if (getTensorTotalSize(*maskTensor) != getCount())
    {
        CARB_LOG_ERROR("mask tensor size (%llu) must equal view count (%u) in %s",
                       (unsigned long long)getTensorTotalSize(*maskTensor), getCount(), __FUNCTION__);
        return false;
    }

    const PxU32 N = getCount();

    // Acquire PhysX CUDA context before any device calls (cudaMalloc, thrust)
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    // Lazy-allocate (or grow) cached indices buffer
    if (!mMaskIndicesDev || N > mMaskIndicesCapacity)
    {
        if (mMaskIndicesDev)
            CHECK_CUDA(cudaFree(mMaskIndicesDev));
        mMaskIndicesDev = nullptr;
        mMaskIndicesCapacity = 0;

        if (cudaMalloc(&mMaskIndicesDev, N * sizeof(PxU32)) != cudaSuccess)
        {
            CARB_LOG_ERROR("Failed to allocate mask indices buffer in %s", __FUNCTION__);
            return false;
        }
        mMaskIndicesCapacity = N;
    }

    if (!compactMaskToIndices(mMaskAllocPolicy, mMaskIndicesDev,
                              static_cast<const uint8_t*>(maskTensor->data), N, outK))
        return false;
    return true;
}

// Macro for simple 2-param masked setters: resolve mask -> build index TensorDesc -> forward.
// IsConst should be empty or 'const'. Hand-write methods with non-standard signatures below.
#define GPU_RB_MASKED_SETTER(MethodName, IsConst)                                       \
bool GpuRigidBodyView::MethodName##Masked(const TensorDesc* src, const TensorDesc* mask) IsConst \
{                                                                                       \
    PxU32 K;                                                                            \
    if (!resolveMask(mask, K)) return false;                                            \
    if (K == 0) return true;                                                            \
    if (K == getCount()) return MethodName(src, nullptr);                               \
    TensorDesc idx{};                                                                   \
    idx.device = mDevice;                                                               \
    idx.dtype  = omni::physics::tensors::TensorDataType::eUint32;                       \
    idx.numDims = 1;                                                                    \
    idx.dims[0] = (int)K;                                                               \
    idx.data   = mMaskIndicesDev;                                                       \
    return MethodName(src, &idx);                                                       \
}

// Non-const masked setters
GPU_RB_MASKED_SETTER(setKinematicTargets, )
GPU_RB_MASKED_SETTER(setTransforms, )
GPU_RB_MASKED_SETTER(setVelocities, )
GPU_RB_MASKED_SETTER(applyForces, )
GPU_RB_MASKED_SETTER(setMasses, )
GPU_RB_MASKED_SETTER(setCOMs, )
GPU_RB_MASKED_SETTER(setInertias, )
GPU_RB_MASKED_SETTER(setDisableGravities, )
GPU_RB_MASKED_SETTER(setDisableSimulations, )

// Const masked setters
GPU_RB_MASKED_SETTER(setMaterialProperties, const)
GPU_RB_MASKED_SETTER(setRestOffsets, const)
GPU_RB_MASKED_SETTER(setContactOffsets, const)

#undef GPU_RB_MASKED_SETTER

// Hand-written: applyForcesAndTorquesAtPositionMasked (5-param signature)
bool GpuRigidBodyView::applyForcesAndTorquesAtPositionMasked(const TensorDesc* srcForceTensor,
                                                              const TensorDesc* srcTorqueTensor,
                                                              const TensorDesc* srcPositionTensor,
                                                              const TensorDesc* mask,
                                                              const bool isGlobal)
{
    PxU32 K;
    if (!resolveMask(mask, K)) return false;
    if (K == 0) return true;
    if (K == getCount()) return applyForcesAndTorquesAtPosition(srcForceTensor, srcTorqueTensor, srcPositionTensor, nullptr, isGlobal);
    TensorDesc idx{};
    idx.device = mDevice;
    idx.dtype = omni::physics::tensors::TensorDataType::eUint32;
    idx.numDims = 1;
    idx.dims[0] = (int)K;
    idx.data = mMaskIndicesDev;
    return applyForcesAndTorquesAtPosition(srcForceTensor, srcTorqueTensor, srcPositionTensor, &idx, isGlobal);
}

// Hand-written: setCompliantMaterialPropertiesMasked (extra srcCombine param)
bool GpuRigidBodyView::setCompliantMaterialPropertiesMasked(const TensorDesc* src,
                                                             const TensorDesc* srcCombine,
                                                             const TensorDesc* mask) const
{
    // setCompliantMaterialProperties delegates to BaseRigidBodyView which calls CPU-side
    // PxShape methods -- GPU src/srcCombine tensors are not supported.
    if (src && src->data && src->device >= 0)
    {
        CARB_LOG_ERROR("setCompliantMaterialPropertiesMasked: src tensor must be on host (CPU); GPU tensors not supported");
        return false;
    }
    if (srcCombine && srcCombine->data && srcCombine->device >= 0)
    {
        CARB_LOG_ERROR("setCompliantMaterialPropertiesMasked: srcCombine tensor must be on host (CPU); GPU tensors not supported");
        return false;
    }
    PxU32 K;
    if (!resolveMask(mask, K)) return false;
    if (K == 0) return true;
    if (K == getCount()) return setCompliantMaterialProperties(src, srcCombine, nullptr);
    TensorDesc idx{};
    idx.device = mDevice;
    idx.dtype = omni::physics::tensors::TensorDataType::eUint32;
    idx.numDims = 1;
    idx.dims[0] = (int)K;
    idx.data = mMaskIndicesDev;
    return setCompliantMaterialProperties(src, srcCombine, &idx);
}

// ============================================================================
// OMPE-94459 (§B9): GPU-aware overrides for per-shape property setters.
// See GpuArticulationView.cpp for the parallel rationale -- per-shape PxShape
// calls are CPU-only, so we stage GPU tensors to host buffers and delegate to
// the base impl.
// ============================================================================

namespace
{
template <typename T>
bool stageTensorToHostRb(const TensorDesc* inDesc, std::vector<T>& hostBuf, TensorDesc& outDesc,
                         const char* tensorName, const char* funcName)
{
    if (!inDesc || !inDesc->data)
    {
        outDesc = inDesc ? *inDesc : TensorDesc{};
        return true;
    }
    const size_t expectedBytes = sizeof(T);
    const size_t actualBytes = getTensorDataTypeSize(inDesc->dtype);
    if (actualBytes != expectedBytes)
    {
        CARB_LOG_ERROR("%s: %s tensor has wrong element size (%zu bytes, expected %zu)",
                       funcName, tensorName, actualBytes, expectedBytes);
        return false;
    }
    outDesc = *inDesc;
    if (inDesc->device < 0)
        return true;
    const size_t numElems = static_cast<size_t>(getTensorTotalSize(*inDesc));
    if (numElems == 0)
    {
        outDesc.data = nullptr;
        return true;
    }
    hostBuf.resize(numElems);
    if (!CHECK_CUDA(cudaMemcpy(hostBuf.data(), inDesc->data, numElems * sizeof(T), cudaMemcpyDeviceToHost)))
    {
        CARB_LOG_ERROR("%s: failed to stage %s tensor from GPU to host", funcName, tensorName);
        return false;
    }
    outDesc.data = hostBuf.data();
    outDesc.device = -1;
    return true;
}
} // namespace

// See the matching block in GpuArticulationView.cpp: cudaMemcpy requires
// the view's CUDA context to be current. The other CUDA paths in this file
// (see line 151, etc.) acquire PhysxCudaContextGuard for the same reason;
// the staged setters/getters added in OMPE-94459 must follow suit.
#define GPU_RB_STAGED_SETTER(MethodName, SrcType, ConstQual)                                            \
    bool GpuRigidBodyView::MethodName(const TensorDesc* srcTensor, const TensorDesc* indexTensor) ConstQual \
    {                                                                                                   \
        CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);                                          \
        PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);                               \
        std::vector<SrcType> srcHost;                                                                   \
        std::vector<PxU32> idxHost;                                                                     \
        TensorDesc srcStaged{};                                                                         \
        TensorDesc idxStaged{};                                                                         \
        if (!stageTensorToHostRb<SrcType>(srcTensor, srcHost, srcStaged, "src", #MethodName))           \
            return false;                                                                               \
        const TensorDesc* idxArg = nullptr;                                                             \
        if (indexTensor && indexTensor->data)                                                           \
        {                                                                                               \
            if (!stageTensorToHostRb<PxU32>(indexTensor, idxHost, idxStaged, "index", #MethodName))     \
                return false;                                                                           \
            idxArg = &idxStaged;                                                                        \
        }                                                                                               \
        return BaseRigidBodyView::MethodName(&srcStaged, idxArg);                                       \
    }

GPU_RB_STAGED_SETTER(setMaterialProperties, float, const)
GPU_RB_STAGED_SETTER(setRestOffsets, float, const)
GPU_RB_STAGED_SETTER(setContactOffsets, float, const)

// Expanded manually (not via GPU_RB_STAGED_SETTER) so we can refresh GPU body sim
// state after toggling eDISABLE_GRAVITY. DirectGPU preIntegration reads
// PxgBodySim.disableGravity, which is only copied in copyToGpuBodySim when the
// body is enqueued via gpu_updateBodySim; postActorFlagChange updates BodyCore
// but does not enqueue standalone rigid bodies (articulation links use
// eDIRTY_LINKS instead). wakeUp() is legal under DirectGPU and triggers
// gpu_updateBodySim without touching velocities.
bool GpuRigidBodyView::setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    // Host-index wake used as a DirectGPU body-sim refresh, not a public wake_up.
    // Skip eKINEMATIC: kinematics ignore gravity, and PxRigidDynamic::wakeUp emits
    // a checked-build error ("Body must be non-kinematic!"). Skip sleeping bodies:
    // they are not integrated, so a stale GPU disableGravity is irrelevant until
    // they wake (wakeUp / contact re-enqueues and copyToGpuBodySim picks up the
    // already-updated BodyCore flag). Do NOT restore the wake counter after
    // wakeUp -- setWakeCounter after wakeUp was observed to defeat the GPU
    // body-sim enqueue (gravity suppress then fails); accepting the counter
    // reset on already-awake bodies is the cost of the DirectGPU refresh.
    // Also skip eDISABLE_SIMULATION (same as BaseRigidBodyView::wakeUp).
    auto wakeForGravityRefresh = [this](const TensorDesc* idxTensor) {
        const PxU32* indices = nullptr;
        PxU32 numIndices = 0;
        if (idxTensor && idxTensor->data)
        {
            indices = static_cast<const PxU32*>(idxTensor->data);
            numIndices = PxU32(getTensorTotalSize(*idxTensor));
        }
        else
        {
            indices = mAllIndices.data();
            numIndices = PxU32(mAllIndices.size());
        }
        for (PxU32 i = 0; i < numIndices; ++i)
        {
            const PxU32 idx = indices[i];
            if (idx >= mEntries.size() || mEntries[idx].type != RigidBodyType::eRigidDynamic)
                continue;
            PxRigidDynamic* dynamicBody = static_cast<PxRigidDynamic*>(mEntries[idx].body);
            if (dynamicBody->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION))
                continue;
            if (dynamicBody->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
                continue;
            if (dynamicBody->isSleeping())
                continue;
            dynamicBody->wakeUp();
        }
    };

    // Stage any GPU-resident src/index to host before Base (CPU-only PhysX flag
    // API). Partial masked writes produce a GPU index via mMaskIndicesDev even
    // when src is CPU-resident; Base rejects non-CPU indices, so the index must
    // be staged on both branches.
    const bool srcOnGpu = srcTensor && srcTensor->data && srcTensor->device >= 0;
    const bool idxOnGpu = indexTensor && indexTensor->data && indexTensor->device >= 0;
    std::vector<uint8_t> srcHost;
    std::vector<PxU32> idxHost;
    TensorDesc srcStaged{};
    TensorDesc idxStaged{};
    const TensorDesc* srcArg = srcTensor;
    const TensorDesc* idxArg = (indexTensor && indexTensor->data) ? indexTensor : nullptr;

    if (srcOnGpu || idxOnGpu)
    {
        CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
        PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
        if (srcOnGpu)
        {
            if (!stageTensorToHostRb<uint8_t>(srcTensor, srcHost, srcStaged, "src", "setDisableGravities"))
                return false;
            srcArg = &srcStaged;
        }
        if (idxOnGpu)
        {
            if (!stageTensorToHostRb<PxU32>(indexTensor, idxHost, idxStaged, "index", "setDisableGravities"))
                return false;
            idxArg = &idxStaged;
        }
    }

    const bool ok = BaseRigidBodyView::setDisableGravities(srcArg, idxArg);
    if (!ok)
        return false;

    wakeForGravityRefresh(idxArg);
    return true;
}

// setDisableSimulations is expanded manually (not via GPU_RB_STAGED_SETTER)
// so we can set mRdIndexDirty after a successful call. The dirty flag lets
// refreshRdGpuIndices() skip its O(n) getGPUIndex() loop on reads that
// follow with no disable/enable transition.
bool GpuRigidBodyView::setDisableSimulations(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!srcTensor || !srcTensor->data || srcTensor->device < 0)
    {
        const bool ok = BaseRigidBodyView::setDisableSimulations(srcTensor, indexTensor);
        if (ok) markRdDisableDirty();
        return ok;
    }
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    std::vector<uint8_t> srcHost;
    std::vector<PxU32> idxHost;
    TensorDesc srcStaged{};
    TensorDesc idxStaged{};
    if (!stageTensorToHostRb<uint8_t>(srcTensor, srcHost, srcStaged, "src", "setDisableSimulations"))
        return false;
    const TensorDesc* idxArg = nullptr;
    if (indexTensor && indexTensor->data)
    {
        if (!stageTensorToHostRb<PxU32>(indexTensor, idxHost, idxStaged, "index", "setDisableSimulations"))
            return false;
        idxArg = &idxStaged;
    }
    const bool ok = BaseRigidBodyView::setDisableSimulations(&srcStaged, idxArg);
    if (ok) markRdDisableDirty();
    return ok;
}

void GpuRigidBodyView::markRdDisableDirty()
{
    // Dirty this view directly so its own next read rebuilds, and advance the
    // scene-wide epoch so every *sibling* view sharing mGpuSimData rebuilds too
    // (OMPE-94459). The epoch bump also flips this view's epoch comparison on
    // the next refresh, which is harmless -- mRdIndexDirty is already true.
    mRdIndexDirty = true;
    if (mGpuSimData)
    {
        ++mGpuSimData->mRdDisableEpoch;
    }
}

#undef GPU_RB_STAGED_SETTER

// Read companions for the uint8 flags: when dst lives on GPU, fill a host
// buffer via the base impl then memcpy to the caller's device pointer.
#define GPU_RB_STAGED_GETTER(MethodName)                                                                \
    bool GpuRigidBodyView::MethodName(const TensorDesc* dstTensor) const                                \
    {                                                                                                   \
        CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);                                          \
        if (!dstTensor || !dstTensor->data || dstTensor->device < 0)                                    \
            return BaseRigidBodyView::MethodName(dstTensor);                                            \
        const size_t numElems = static_cast<size_t>(getTensorTotalSize(*dstTensor));                    \
        if (numElems == 0) return true;                                                                 \
        std::vector<uint8_t> host(numElems, 0);                                                         \
        TensorDesc hostDesc = *dstTensor;                                                               \
        hostDesc.data = host.data();                                                                    \
        hostDesc.device = -1;                                                                           \
        if (!BaseRigidBodyView::MethodName(&hostDesc)) return false;                                    \
        PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);                               \
        return CHECK_CUDA(cudaMemcpy(dstTensor->data, host.data(),                                      \
                                     numElems * sizeof(uint8_t), cudaMemcpyHostToDevice));              \
    }

GPU_RB_STAGED_GETTER(getDisableGravities)
GPU_RB_STAGED_GETTER(getDisableSimulations)

#undef GPU_RB_STAGED_GETTER

// wakeUp has a single-tensor signature (indices only, no src). BaseRigidBodyView::
// wakeUp calls PxRigidDynamic::wakeUp per body, which is a CPU-side PhysX SDK
// call that works on both pipelines, but the index buffer must live on host.
// Stage the indices to host via the same helper the other GPU setters use.
bool GpuRigidBodyView::wakeUp(const TensorDesc* indexTensor)
{
    if (!indexTensor || !indexTensor->data || indexTensor->device < 0)
        return BaseRigidBodyView::wakeUp(indexTensor);

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    std::vector<PxU32> idxHost;
    TensorDesc idxStaged{};
    if (!stageTensorToHostRb<PxU32>(indexTensor, idxHost, idxStaged, "index", "wakeUp"))
        return false;
    return BaseRigidBodyView::wakeUp(&idxStaged);
}

bool GpuRigidBodyView::putToSleep(const TensorDesc* indexTensor)
{
    if (!indexTensor || !indexTensor->data || indexTensor->device < 0)
        return BaseRigidBodyView::putToSleep(indexTensor);

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    std::vector<PxU32> idxHost;
    TensorDesc idxStaged{};
    if (!stageTensorToHostRb<PxU32>(indexTensor, idxHost, idxStaged, "index", "putToSleep"))
        return false;
    return BaseRigidBodyView::putToSleep(&idxStaged);
}

} // namespace tensors
} // namespace physx
} // namespace omni

// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-CORE-001
 * @covers AC-7 AC-8
 *
 * @implements REQ-READ-DEVICE-001
 * @covers AC-2
 *
 * @implements REQ-READ-INSTANCER-001
 * @covers AC-1
 *
 * @implements REQ-READ-ATTRS-001
 * @covers AC-1, AC-5
 *
 * @implements REQ-TENSOR-INDEX-001
 * @covers AC-1 AC-2 AC-3
 * @implements REQ-TENSOR-CPU-ONLY-001
 * @covers AC-1 AC-2 AC-4
 *
 * @implements REQ-INPUT-DEVICE-001
 * @covers AC-1 AC-1b AC-2 AC-3 AC-8
 *
 * @implements REQ-INPUT-CORE-001
 * @covers AC-10
 */

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

using omni::physics::tensors::checkRecordIndices;
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

GpuRigidBodyView::GpuRigidBodyView(GpuSimulationView* sim,
                                   const std::vector<RigidBodyEntry>& entries,
                                   int device,
                                   bool invalidateOnDisabledRd)
    : BaseRigidBodyView(sim, entries),
      mDevice(device),
      mInvalidateOnDisabledRd(invalidateOnDisabledRd)
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
                // Actor in neither map: added after GpuSimulationData init, or the data describes a
                // different scene entirely (a PxScene* recycled to a new scene with the same topology
                // passes every other validation). Either way the data is stale for this actor set --
                // flag it so a caller reusing cached data rebuilds rather than reads sentinel rows.
                mHasUnresolvedEntries = true;
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
                mHasDisabledRdRows = true;
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

    if (!SHIM_CU_EVENT_CREATE(&mOvStageIdxReadyEvent, CU_EVENT_DISABLE_TIMING))
        CARB_LOG_ERROR("Failed to create the ovstage rigid packed-index ready event");

    prepareDeviceData((void**)&mRbIndicesDev, rbIndices.data(), numBodies * sizeof(PxU32), "mRbIndicesDev");
    // rb indexing data
    prepareDeviceData((void**)&mRbRecordsDev, rbRecords.data(), numBodies * sizeof(GpuRigidBodyRecord), "mRbRecordsDev");

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

    // Fixed-membership tensor views cannot carry holes. The ovstage superset deliberately can: it
    // keeps the actor row stable while the read/write layer omits disabled rows from user columns.
    for (PxU32 i = 0; mInvalidateOnDisabledRd && i < mEntries.size(); ++i)
    {
        if (mEntries[i].type != RigidBodyType::eRigidDynamic)
            continue;
        if (rbRecords[i].physxRdIdx == 0xffffffffu)
        {
            invalidateMappingForDisabledRd(
                "GpuRigidBodyView constructed with disabled rigid dynamic(s)");
            break;
        }
    }
}

GpuRigidBodyView::~GpuRigidBodyView()
{
    if (mGpuSimData)
    {
        CudaContextGuard ctxGuard(mGpuSimData->mCtx);

        // Drain before freeing: an ovstage gather is launched on the null stream and returns without
        // synchronizing, so a view destroyed in the same frame as its last read can free records the
        // kernel is still reading. cudaFree has historically synchronized implicitly; that is not a
        // guarantee to rely on. Same reason GpuArticulationView drains before its selection buffer.
        CHECK_CUDA(cudaStreamSynchronize(nullptr));

        // PhysX consumes the ovstage write buffers on its own streams, which the drain above does not
        // cover: wait for the last pose/velocity and wrench applies before freeing what they read.
        if (CUevent applyDone = mGpuSimData->mApplySignalEvents[ApplyEvent::eRdData])
            CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(applyDone), nullptr));
        if (CUevent wrenchDone = mGpuSimData->mApplySignalEvents[ApplyEvent::eRdForces])
            CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(wrenchDone), nullptr));
        if (mOvStageIdxReadyEvent)
            CHECK_CU(getCudaShim()->eventDestroy(reinterpret_cast<uintptr_t>(mOvStageIdxReadyEvent), nullptr));

        CHECK_CUDA(cudaFree(mRbIndicesDev));
        CHECK_CUDA(cudaFree(mRdGpuIndicesDev));
        CHECK_CUDA(cudaFree(mArtiIndicesDev));
        CHECK_CUDA(cudaFree(mRbRecordsDev));
        for (OvStageRowsSlot& rowsSlot : mOvStageRows)
            CHECK_CUDA(cudaFree(rowsSlot.dev));
        CHECK_CUDA(cudaFree(mOvStageWrenchDev));
        CHECK_CUDA(cudaFree(mOvStagePackedDev));
        CHECK_CUDA(cudaFree(mOvStageIdxDev));
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

    if (!refreshRdGpuIndices())
        return false;

    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    SYNCHRONIZE_CUDA();

    // Real finish events, not NULL. PxDirectGPUAPI documents finishEvent = NULL as "the function
    // will wait for the copy to finish before returning" -- a host block per call. Recording an
    // event instead lets the reads below overlap; the streamWaitEvent pair further down orders our
    // fetch kernel after them. Both events are allocated by GpuSimulationData.
    CUevent rdCopyEvent = mGpuSimData->mCopyEvents[CopyEvent::eRdData];
    CUevent artiCopyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiLinkTransforms];

    // Both getters' status is checked, and the wait is folded into the same call: PhysX refuses
    // these reads in states the caller cannot see -- pre-step being the documented one -- and writes
    // NOTHING when it does, so an unchecked refusal is gathered as whatever the scratch last held
    // and served as poses (ADR-0008 Decision 10).
    if (mNumRds > 0)
    {
        if (!rdCopyEvent)
        {
            CARB_LOG_ERROR("%s: missing rigid-dynamic completion event", __FUNCTION__);
            return false;
        }
        if (!mGpuSimData->awaitDirectGpuFetch(
                scene->getDirectGPUAPI().getRigidDynamicData(
                    (void*)mGpuSimData->mRdPoseDev, mRdGpuIndicesDev, PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE,
                    mNumRds, mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eRdPose), rdCopyEvent),
                rdCopyEvent, __FUNCTION__))
        {
            return false;
        }
    }

    if (mNumArtis > 0)
    {
        if (!artiCopyEvent)
        {
            CARB_LOG_ERROR("%s: missing articulation completion event", __FUNCTION__);
            return false;
        }
        if (!mGpuSimData->awaitDirectGpuFetch(
                scene->getDirectGPUAPI().getArticulationData(
                    (void*)mGpuSimData->mLinkOrRootTransformsDev, mArtiIndicesDev,
                    PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, mNumArtis,
                    mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eLinkOrRootTransforms), artiCopyEvent),
                artiCopyEvent, __FUNCTION__))
        {
            return false;
        }
    }

    SYNCHRONIZE_CUDA();

    // Our kernels are done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded before the branch -- the gather can fail after launching.
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eRdPose, SharedDeviceBuffer::eLinkOrRootTransforms, [&] {
            return fetchRbTransforms(static_cast<TensorTransform*>(dstTensor->data), mGpuSimData->mRdPoseDev,
                                     mGpuSimData->mLinkOrRootTransformsDev, getCount(), mGpuSimData->mMaxLinks,
                                     mRbRecordsDev);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch rigid body tranforms");
        return false;
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
    // See getTransforms: a NULL finishEvent makes each DirectGPU read block the host. The two
    // rigid-dynamic reads share eRdData because they are dispatched on the same PhysX stream in
    // order, so waiting on the last recording also covers the first; the articulation reads use
    // their own events since they are issued by a different core.
    CUevent rdCopyEventLin = gpuSimData->mCopyEvents[CopyEvent::eRdData];
    CUevent rdCopyEventAng = gpuSimData->mCopyEvents[CopyEvent::eRdData];
    CUevent artiCopyEventLin = gpuSimData->mCopyEvents[CopyEvent::eArtiLinkLinearVelocities];
    CUevent artiCopyEventAng = gpuSimData->mCopyEvents[CopyEvent::eArtiLinkAngularVelocities];
    if (numRds > 0)
    {
        // Each fetch checked against its OWN completion event, and the wait folded into the same
        // call. An unchecked refusal writes nothing and is gathered as stale scratch.
        if (!rdCopyEventLin || !rdCopyEventAng)
        {
            CARB_LOG_ERROR("%s: missing rigid-dynamic completion event", __FUNCTION__);
            return false;
        }
        if (!gpuSimData->awaitDirectGpuFetch(
                scene->getDirectGPUAPI().getRigidDynamicData(
                    (void*)rdDataLinearDev, rdGpuIndicesDev, rdLinearType, numRds,
                    gpuSimData->kernelDoneEvent(SharedDeviceBuffer::eRdLinearVel), rdCopyEventLin),
                rdCopyEventLin, __FUNCTION__) ||
            !gpuSimData->awaitDirectGpuFetch(
                scene->getDirectGPUAPI().getRigidDynamicData(
                    (void*)rdDataAngularDev, rdGpuIndicesDev, rdAngularType, numRds,
                    gpuSimData->kernelDoneEvent(SharedDeviceBuffer::eRdAngularVel), rdCopyEventAng),
                rdCopyEventAng, __FUNCTION__))
        {
            return false;
        }
    }
    SYNCHRONIZE_CUDA();
    if (numArtis > 0)
    {
        if (!artiCopyEventLin || !artiCopyEventAng)
        {
            CARB_LOG_ERROR("%s: missing articulation completion event", __FUNCTION__);
            return false;
        }
        if (!gpuSimData->awaitDirectGpuFetch(
                scene->getDirectGPUAPI().getArticulationData(
                    (void*)linkDataLinearDev, artiIndicesDev, linkLinearType, numArtis,
                    gpuSimData->kernelDoneEvent(SharedDeviceBuffer::eLinkOrRootLinearVel), artiCopyEventLin),
                artiCopyEventLin, __FUNCTION__) ||
            !gpuSimData->awaitDirectGpuFetch(
                scene->getDirectGPUAPI().getArticulationData(
                    (void*)linkDataAngularDev, artiIndicesDev, linkAngularType, numArtis,
                    gpuSimData->kernelDoneEvent(SharedDeviceBuffer::eLinkOrRootAngularVel), artiCopyEventAng),
                artiCopyEventAng, __FUNCTION__))
        {
            return false;
        }
    }

    SYNCHRONIZE_CUDA();

    const bool velAccGathered =
        fetchRbVelAcc(static_cast<TensorVelAcc*>(dstTensor->data), rdDataLinearDev, rdDataAngularDev, linkDataLinearDev,
                      linkDataAngularDev, numRb, simMaxLinks, rbRecordsDev);
    // Our kernels are done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded before the branch, not after: the gather can fail AFTER
    // launching, and returning first leaves all four buffers open to the next writer mid-kernel.
    gpuSimData->recordKernelDone(SharedDeviceBuffer::eRdLinearVel);
    gpuSimData->recordKernelDone(SharedDeviceBuffer::eRdAngularVel);
    gpuSimData->recordKernelDone(SharedDeviceBuffer::eLinkOrRootLinearVel);
    gpuSimData->recordKernelDone(SharedDeviceBuffer::eLinkOrRootAngularVel);
    if (!velAccGathered)
    {
        CARB_LOG_ERROR("Failed to fetch rigid body velocities or acceleration");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}


void GpuRigidBodyView::invalidateMappingForDisabledRd(const char* reason) const
{
    CARB_LOG_ERROR(
        "Rigid body tensor view mapping invalidated: %s. DirectGPU has no row for "
        "eDISABLE_SIMULATION rigid dynamics. getValid() is now false — recreate the "
        "simulation view / tensor binding for the enabled set (OMPE-103213).",
        reason ? reason : "disabled rigid dynamic in view");
    if (mSim)
        mSim->invalidate();
}

bool GpuRigidBodyView::refreshRdGpuIndices() const
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
        return true;

    // OMPE-94459: a rigid dynamic's GPU index is its island node handle,
    // which PhysX frees on disable (eDISABLE_SIMULATION / scene removal) and
    // reallocates -- generally to a different value, via a LIFO free list --
    // on re-enable. getGPUIndex() returns PX_INVALID_NODE (0xffffffff) while
    // disabled. Fixed-membership tensor views invalidate on a missing row. The
    // ovstage superset retains a sentinel record and selects only enabled rows.
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
    mHasDisabledRdRows = anyDisabledRd;
    if (mGpuSimData)
    {
        // Only the scene superset saw every rigid dynamic, so only it may retire the hint. A subset
        // view finding none among its own entries proves nothing about the bodies it does not hold.
        if (anyDisabledRd)
        {
            mGpuSimData->mMayHaveDisabledRd = true;
        }
        else if (mIsSceneSuperset)
        {
            mGpuSimData->mMayHaveDisabledRd = false;
        }
    }
    if (comsNeedInvalidate)
    {
        // Direct member access: setComsCacheStateValid() is non-const and
        // refreshRdGpuIndices() is const (called from const read-path overrides
        // of IRigidBodyView). validComsCache is mutable for this purpose.
        validComsCache = false;
    }

    // Fixed-membership tensor views still require recreation. The ovstage superset is a stable
    // actor-to-record map, not a promise that every record has a live DirectGPU row.
    if (anyDisabledRd && mInvalidateOnDisabledRd)
    {
        invalidateMappingForDisabledRd(
            "refreshRdGpuIndices found rigid dynamic(s) without a live GPU index");
        return false;
    }

    if (!changed && !mDeviceUploadPending)
    {
        // Nothing to upload — device map already matches what the loop re-read.
        mRdIndexDirty = false;
        return true;
    }
    // cudaMemcpy needs the view's CUDA context current; callers acquire
    // PhysxCudaContextGuard later for their own DirectGPU calls, but this
    // helper runs before that point in getTransforms/getVelocities/etc.
    // Match the convention used by other CUDA entry points in this file.
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    bool uploaded = true;
    if (mNumRds > 0)
    {
        uploaded = CHECK_CUDA(cudaMemcpy(mRdGpuIndicesDev, mRdGpuIndicesHost.data(),
                                         mNumRds * sizeof(PxRigidDynamicGPUIndex),
                                         cudaMemcpyHostToDevice));
    }
    // Upload only the changed record sub-range rather than all numBodies
    // records: a workload toggling disable each step changes only the toggled
    // rows, and this helper runs at the top of every read/write entry point, so
    // a full-array HtoD here would re-push the whole array up to 3x/step. The
    // ctor seeds the entire device array once (prepareDeviceData above), so the
    // unchanged rows stay valid and only [firstChanged, lastChanged] needs
    // re-upload. `changed` implies firstChanged was set. (OMPE-94459)
    //
    // Recovering from a failed upload re-sends the whole array: firstChanged/lastChanged describe
    // this pass, and the pass that failed is over.
    if (!rbRecords.empty())
    {
        const PxU32 first = mDeviceUploadPending ? 0u : firstChanged;
        const PxU32 last = mDeviceUploadPending ? PxU32(rbRecords.size()) - 1u : lastChanged;
        const PxU32 rangeCount = last - first + 1;
        // Evaluated first so the second upload is attempted even if the first failed.
        uploaded = CHECK_CUDA(cudaMemcpy(mRbRecordsDev + first, rbRecords.data() + first,
                                         rangeCount * sizeof(GpuRigidBodyRecord),
                                         cudaMemcpyHostToDevice)) &&
                   uploaded;
    }

    mDeviceUploadPending = !uploaded;
    // Stay dirty while an upload is pending so this runs again to retry it.
    if (uploaded)
    {
        mRdIndexDirty = false;
    }
    // The upload result, not an unconditional success. Every caller treats a false return as "do not
    // read": proceeding after a failed index or record upload gathers through a stale GPU index map,
    // which addresses whatever body now owns that island node rather than reporting an error.
    return uploaded;
}

bool GpuRigidBodyView::refreshDisabledRowsOvStage() const
{
    // Every SUPPORTED disable or re-enable already rebuilds this view: its own toggle sets
    // mRdIndexDirty directly through markRdDisableDirty(), and a sibling view's toggle bumps the
    // scene-wide disable epoch that refreshRdGpuIndices() compares against mLastSeenRdDisableEpoch.
    // So this is a pass-through -- it does NOT force a rebuild merely because a body is currently
    // disabled. Doing so on every read and write commit while a body sits parked disabled (the
    // common steady state) would pay the O(n) getGPUIndex() walk the fast path exists to skip, on
    // every I/O, with nothing changed since the last rebuild.
    //
    // There is deliberately no actor-flag walk here either. Toggling eDISABLE_SIMULATION on a raw
    // PxRigidDynamic* from getPhysXPtr is unsupported: nothing bumps the epoch, so a fresh disable is
    // never seen and the stale row resolves to whichever body inherited the freed island handle
    // (REQ-INPUT-CORE-001 AC-10, REQ-CAPI-PHYSXPTR-001 AC-6). A raw-pointer re-enable is unseen for
    // the same reason -- symmetric with the disable, and consistent with that path being undefined.
    return refreshRdGpuIndices();
}

bool GpuRigidBodyView::mayHaveDisabledRdRows() const
{
    // Conservative and O(1): sentinel rows are a known disable, and an epoch ahead of this view's
    // last sync is a notified toggle whose direction we have not resolved yet. Never false while a
    // notified disable is outstanding, which is what lets callers skip their own per-body scan.
    return mHasDisabledRdRows || (mGpuSimData && mGpuSimData->mRdDisableEpoch != mLastSeenRdDisableEpoch);
}

// The ovstage write's device state. See the header for the lifetime rule; the work here is gated on
// `token` so the allocation, the upload, the per-row validation and the index build all happen when
// the row list changes rather than on every write.
bool GpuRigidBodyView::ovStageWriteBuffers(const char* const attribName,
                                           const PxU32* rows,
                                           const PxU32 numOutputs,
                                           const uint64_t token,
                                           const PxU32*& outRowsDev,
                                           void*& outPacked,
                                           PxRigidDynamicGPUIndex*& outIdx)
{
    if (numOutputs == 0)
        return false;

    outRowsDev = rows ? ovStageRowsDevice(rows, numOutputs, token) : nullptr;
    if (rows && !outRowsDev)
    {
        CARB_LOG_ERROR("%s: ovstage record-list upload failed", attribName);
        return false;
    }

    if (numOutputs > mOvStageWriteCapacity)
    {
        int status = 0;
        // The scratch being freed here is the one the LAST setRigidDynamicData was handed, and that
        // call is asynchronous -- it signals mApplySignalEvents[eRdData] when PhysX is done reading.
        // Freeing device memory out from under an in-flight read is cudaErrorIllegalAddress, which
        // poisons the context for every later CUDA call in the process rather than failing here.
        //
        // A session reaches this with a live write in flight whenever a later group needs MORE rows
        // than an earlier one, which is routine: one attribute write emits the
        // standalone group AND one array group per instancer, and those sizes differ.
        waitOvStageWriteDone();
        if (mOvStagePackedDev)
            CHECK_CU(getCudaShim()->memFree(reinterpret_cast<uintptr_t>(mOvStagePackedDev), &status));
        if (mOvStageIdxDev)
            CHECK_CU(getCudaShim()->memFree(reinterpret_cast<uintptr_t>(mOvStageIdxDev), &status));
        mOvStagePackedDev = nullptr;
        mOvStageIdxDev = nullptr;
        mOvStageWriteCapacity = 0;
        mOvStageIdxToken = 0; // the index list did not survive the reallocation

        uintptr_t packed = 0;
        uintptr_t idx = 0;
        if (!getCudaShim()->memAlloc(&packed, size_t(numOutputs) * sizeof(PxTransform), &status) || !packed ||
            !getCudaShim()->memAlloc(&idx, size_t(numOutputs) * sizeof(PxRigidDynamicGPUIndex), &status) || !idx)
        {
            if (packed)
                CHECK_CU(getCudaShim()->memFree(packed, &status));
            CARB_LOG_ERROR("%s: ovstage write buffer allocation failed (%u rows)", attribName, numOutputs);
            return false;
        }
        mOvStagePackedDev = reinterpret_cast<PxTransform*>(packed);
        mOvStageIdxDev = reinterpret_cast<PxRigidDynamicGPUIndex*>(idx);
        mOvStageWriteCapacity = numOutputs;
    }

    // The packed index list depends on TWO things, and only one of them is the row list.
    //
    // A rigid dynamic's GPU index is its island node handle, which PhysX frees on disable and
    // reallocates -- generally to a different value -- on re-enable (OMPE-94459). refreshRdGpuIndices
    // has already rebuilt mRdGpuIndicesDev and the records' tensorRdIdx by the time we get here, but
    // the ROWS are unchanged across that: same query, same bodies. So gating this on the row token
    // alone would reuse indices built against the previous mapping and write to the wrong bodies --
    // silently, on the workflow those comments exist to describe.
    //
    // The kernel is therefore run on EVERY write. It is numOutputs threads with no synchronize, far
    // below the DirectGPU call it feeds, and making its correctness depend on enumerating every way
    // the mapping can move is the trade that produces this class of bug. The expensive things -- the
    // database walk, the canonicalisation, the row resolution -- stay cached; this does not.
    //
    // The per-row validation IS gated, on the row list (token) and the disable epoch together,
    // because what it checks -- a record with no GPU row -- changes only when one of those moves.
    // Every SUPPORTED disable OR re-enable advances the epoch: markRdDisableDirty() bumps it
    // value-independently for both the tensor override and the ovstage write, so the epoch term alone
    // catches every transition. A standalone group has its disabled bodies filtered out at plan time,
    // so it never carries a sentinel; an instancer group keeps them, but a group a sentinel refuses
    // returns before re-stamping mOvStageIdxDisableEpoch below, so the epoch stays mismatched and the
    // check keeps firing until that body is re-enabled. The packed-index kernel clamps a sentinel to
    // slot 0 rather than branching, so a skipped check on a real sentinel would be a silent write to
    // another body -- which this epoch gating prevents. A FRESH disable on a raw PxRigidDynamic*
    // outside the supported routes advances nothing and leaves stale records, so it is not caught here
    // at all (unsupported, REQ-CAPI-PHYSXPTR-001 AC-6).
    if (token == 0 || token != mOvStageIdxToken || mLastSeenRdDisableEpoch != mOvStageIdxDisableEpoch)
    {
        // A record with no GPU row would poison the whole DirectGPU call, and a kernel cannot report
        // it without a synchronize -- so it is checked HERE, on the host, where rbRecords already
        // lives. Guaranteed not to fire in practice (the ovstage rigid enumeration yields only rigid
        // dynamics), which is exactly why it is worth failing loudly rather than trusting.
        for (PxU32 i = 0; i < numOutputs; ++i)
        {
            const PxU32 rec = rows ? rows[i] : i;
            if (rec >= rbRecords.size() || rbRecords[rec].tensorRdIdx == 0xffffffff)
            {
                CARB_LOG_ERROR("%s: entry %u has no DirectGPU index -- a disabled rigid dynamic, an "
                               "articulation link, or a point-instancer instance whose body is "
                               "disabled. Disabling an individual instancer instance is unsupported, so "
                               "a disabled one is out of contract; the whole instancer group is refused "
                               "rather than resolving the sentinel to the wrong instance "
                               "(REQ-INPUT-CORE-001 AC-10). Nothing was written.",
                               attribName, i);
                return false;
            }
        }
        mOvStageIdxToken = token;
        mOvStageIdxDisableEpoch = mLastSeenRdDisableEpoch;
    }
    if (!submitRbPackedIndicesOvStage(mOvStageIdxDev, mRdGpuIndicesDev, mRbRecordsDev, outRowsDev, numOutputs))
    {
        CARB_LOG_ERROR("%s: packed index build failed", attribName);
        return false;
    }

    // PhysX's DirectGPU copies run on CU_STREAM_NON_BLOCKING streams, which the null stream this kernel
    // ran on does not order against: every consumer of the list passes this event as startEvent,
    // including the pre-reads issued before the caller's own post-scatter record. Without it PhysX
    // gathers through an unbuilt list -- garbage indices, an illegal address, a poisoned context.
    // A failed record falls back to a drain (recordKernelDone's convention); a failed drain refuses.
    if (!mOvStageIdxReadyEvent ||
        !getCudaShim()->eventRecord(reinterpret_cast<uintptr_t>(mOvStageIdxReadyEvent), uintptr_t(0), nullptr))
    {
        if (!CHECK_CUDA(cudaStreamSynchronize(nullptr)))
        {
            CARB_LOG_ERROR("%s: could not order the packed index build ahead of PhysX", attribName);
            return false;
        }
    }

    outPacked = mOvStagePackedDev;
    outIdx = mOvStageIdxDev;
    return true;
}

const PxU32* GpuRigidBodyView::ovStageRowsDevice(const PxU32* rows, uint32_t count, uint64_t token) const
{
    if (!rows || count == 0)
        return nullptr;

    CudaContextGuard ctxGuard(mGpuSimData ? mGpuSimData->mCtx : nullptr);

    // Hit: a slot already holds this exact list. The token must identify the row list ITSELF -- the
    // reader mints a version whenever it builds a new list. A cheaper token will not do: one built from
    // view generation, object type and scope leaves all three unchanged across two different queries of
    // the same type and scope, and kOvxActive rebuilds its rows every read. Count is compared too -- it
    // bounds the memcpy and the slot's count, whatever the caller derives its token from. Touching the
    // slot refreshes its recency so a list read every frame is never the eviction victim.
    for (OvStageRowsSlot& hit : mOvStageRows)
    {
        if (hit.token != 0 && hit.token == token && hit.count == count)
        {
            hit.tick = ++mOvStageRowsClock;
            return hit.dev;
        }
    }

    // Miss: evict the least-recently-used slot (an empty slot, tick 0, wins first) so every
    // concurrently-live list -- the stable rigid-body and link reads, an active read, the write scratch
    // -- keeps its own device copy. A warm read refreshed its slot this frame, so while the live lists
    // fit the slots the victim is an empty slot or the oldest churning list; once they exceed
    // kOvStageRowsSlots (a many-instancer write burst) even a warm read can be the victim.
    uint32_t victim = 0;
    for (uint32_t i = 1; i < kOvStageRowsSlots; ++i)
    {
        if (mOvStageRows[i].tick < mOvStageRows[victim].tick)
            victim = i;
    }
    OvStageRowsSlot& s = mOvStageRows[victim];

    if (count > s.capacity)
    {
        CHECK_CUDA(cudaFree(s.dev));
        s.dev = nullptr;
        s.capacity = 0;
        s.token = 0;
        s.tick = 0;
        if (!prepareDeviceData((void**)&s.dev, nullptr, count * sizeof(PxU32), "ovstage row-list slot"))
            return nullptr;
        s.capacity = count;
    }

    // Validated here rather than per gather: the kernels index records[outRecordIdx[i]] with no bound
    // of their own, so an out-of-range entry is a silent out-of-bounds device read. Failing here means
    // the list is never uploaded, and the null return already stops every caller.
    if (!checkRecordIndices(rows, count, mEntries.size(), "ovstage record list", __FUNCTION__))
    {
        s.token = 0;
        s.tick = 0;
        return nullptr;
    }
    if (!CHECK_CUDA(cudaMemcpy(s.dev, rows, count * sizeof(PxU32), cudaMemcpyHostToDevice)))
    {
        s.token = 0; // a failed upload must not be mistaken for a held copy
        s.tick = 0;
        return nullptr;
    }
    ++mOvStageRowsUploadCount; // test-only counter; see ovStageRowsUploadCount()
    s.token = token;
    s.count = count;
    s.tick = ++mOvStageRowsClock;
    return s.dev;
}

// One bulk read of the pose source, then the gather that slices it. Shared by position and
// orientation, which differ only in which slice they take.
bool GpuRigidBodyView::getPoseColumnOvStage(const char* const attribName,
                                            const TensorDesc* const dstTensor,
                                            const bool wantOrientation,
                                            const PxU32* outRecordIdx,
                                            const PxU32 numOutputs,
                                            const uint64_t rowsToken) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);

    // Everything the call can be rejected for, before any of it happens: the work below rebuilds the
    // GPU index map and issues two full-scene DirectGPU reads.
    if (!dstTensor || !dstTensor->data)
        return false;
    if (!checkTensorDevice(*dstTensor, mDevice, attribName, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, attribName, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, numOutputs * (wantOrientation ? 4u : 3u), attribName, __FUNCTION__))
    {
        return false;
    }

    // Must precede the read: it indexes through mRdGpuIndicesDev, and a disabled rigid dynamic's
    // GPU row is freed and recycled. On the ovstage superset the refresh keeps that record as a
    // sentinel; the ovstage read layer is what omits disabled rows from `outRecordIdx`, so this
    // gather never slices one. A fixed-membership view instead invalidates and fails here.
    if (!refreshRdGpuIndices())
        return false;

    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    const PxU32* outRecordIdxDev = nullptr;
    if (outRecordIdx)
    {
        outRecordIdxDev = ovStageRowsDevice(outRecordIdx, numOutputs, rowsToken);
        if (!outRecordIdxDev)
        {
            CARB_LOG_ERROR("%s: ovstage record-list upload failed", attribName);
            return false;
        }
    }

    // Rigid dynamics and articulation links are separate engine buffers; each is read only if this
    // view holds any.
    CUevent rdEvent = mGpuSimData->mCopyEvents[CopyEvent::eRdData];
    CUevent artiEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiLinkTransforms];
    // Both getters' status is checked: PhysX refuses these calls outright in states the caller cannot
    // see -- pre-step being the documented one -- and writes NOTHING when it does. The scratch is not
    // zeroed, so an unchecked refusal is gathered as whatever the buffer last held and served as
    // poses (ADR-0008 Decision 10).
    if (mNumRds > 0)
    {
        // startEvent, not nullptr: a gather from a PREVIOUS read of this buffer may still be running
        // on our stream, and position and orientation both fill it within a single read. Without
        // this PhysX overwrites it underneath that gather (ADR-0008 Decision 7).
        if (!mGpuSimData->awaitDirectGpuFetch(
                scene->getDirectGPUAPI().getRigidDynamicData(
                    (void*)mGpuSimData->mRdPoseDev, mRdGpuIndicesDev, PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE,
                    mNumRds, mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eRdPose), rdEvent),
                rdEvent, __FUNCTION__))
        {
            CARB_LOG_ERROR("%s: PxDirectGPUAPI::getRigidDynamicData(eGLOBAL_POSE) refused the read", attribName);
            return false;
        }
    }
    if (mNumArtis > 0)
    {
        if (!mGpuSimData->awaitDirectGpuFetch(
                scene->getDirectGPUAPI().getArticulationData(
                    (void*)mGpuSimData->mLinkOrRootTransformsDev, mArtiIndicesDev,
                    PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, mNumArtis,
                    mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eLinkOrRootTransforms), artiEvent),
                artiEvent, __FUNCTION__))
        {
            CARB_LOG_ERROR("%s: PxDirectGPUAPI::getArticulationData(eLINK_GLOBAL_POSE) refused the read", attribName);
            return false;
        }
    }

    // Our kernels are done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Unconditional, and never behind an early return: the gather reports
    // failure via CHECK_CUDA(cudaGetLastError()), which can surface an error raised EARLIER with this
    // launch still in flight -- exactly the case where the next DirectGPU call must wait.
    const bool gathered = gatherPoseColumnOvStage(wantOrientation, outRecordIdxDev, numOutputs, dstTensor);
    mGpuSimData->recordKernelDone(SharedDeviceBuffer::eRdPose);
    mGpuSimData->recordKernelDone(SharedDeviceBuffer::eLinkOrRootTransforms);
    return gathered;
}

// As above for a velocity source. The caller names which engine reads and scratch buffers to use,
// so linear and angular share this without either paying for the other.
bool GpuRigidBodyView::getVelocityColumnOvStage(const char* const attribName,
                                                const TensorDesc* const dstTensor,
                                                const PxRigidDynamicGPUAPIReadType::Enum rdType,
                                                const PxArticulationGPUAPIReadType::Enum artiType,
                                                const PxVec3* rdScratch,
                                                const PxVec3* artiScratch,
                                                const PxU32 rdBuf,
                                                const PxU32 artiBuf,
                                                const int artiCopyEvent,
                                                const PxU32* outRecordIdx,
                                                const PxU32 numOutputs,
                                                const uint64_t rowsToken) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);

    // As above: rejected before any engine work, not after it.
    if (!dstTensor || !dstTensor->data)
        return false;
    if (!checkTensorDevice(*dstTensor, mDevice, attribName, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, attribName, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, numOutputs * 3u, attribName, __FUNCTION__))
    {
        return false;
    }

    // As above: refresh before indexing through mRdGpuIndicesDev; disabled rows are never handed
    // to this gather rather than being patched over.
    if (!refreshRdGpuIndices())
        return false;

    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    const PxU32* outRecordIdxDev = nullptr;
    if (outRecordIdx)
    {
        outRecordIdxDev = ovStageRowsDevice(outRecordIdx, numOutputs, rowsToken);
        if (!outRecordIdxDev)
        {
            CARB_LOG_ERROR("%s: ovstage record-list upload failed", attribName);
            return false;
        }
    }

    CUevent rdEvent = mGpuSimData->mCopyEvents[CopyEvent::eRdData];
    CUevent artiEvent = mGpuSimData->mCopyEvents[artiCopyEvent];
    // Checked, for the reason spelled out in getPoseColumnOvStage: a refused read writes nothing and
    // leaves the scratch holding whatever was there before.
    if (mNumRds > 0)
    {
        if (!mGpuSimData->awaitDirectGpuFetch(
                scene->getDirectGPUAPI().getRigidDynamicData(
                    (void*)rdScratch, mRdGpuIndicesDev, rdType, mNumRds, mGpuSimData->kernelDoneEvent(rdBuf), rdEvent),
                rdEvent, __FUNCTION__))
        {
            CARB_LOG_ERROR("%s: PxDirectGPUAPI::getRigidDynamicData refused the read", attribName);
            return false;
        }
    }
    if (mNumArtis > 0)
    {
        if (!mGpuSimData->awaitDirectGpuFetch(
                scene->getDirectGPUAPI().getArticulationData((void*)artiScratch, mArtiIndicesDev, artiType, mNumArtis,
                                                             mGpuSimData->kernelDoneEvent(artiBuf), artiEvent),
                artiEvent, __FUNCTION__))
        {
            CARB_LOG_ERROR("%s: PxDirectGPUAPI::getArticulationData refused the read", attribName);
            return false;
        }
    }

    // Unconditional, for the reason spelled out in getPoseColumnOvStage: a failed gather can mean an
    // earlier error surfacing with this launch still running, which is when the record matters most.
    const bool gathered = gatherVelocityColumnOvStage(rdScratch, artiScratch, outRecordIdxDev, numOutputs, dstTensor);
    mGpuSimData->recordKernelDone(rdBuf);
    mGpuSimData->recordKernelDone(artiBuf);
    return gathered;
}

// The point-instancer columns. They live on the rigid view because everything they need is here: the
// per-read GPU-index rebuild, the shared pose/velocity scratch, and the event protocol around it
// (ADR-0008 Decision 7). The instancer view owns the instancer-shaped device arrays and hands them in.
//
// The instances are read through the SUPERSET view, which already covers them (collectSupersetRigidEntries
// does not filter instanced bodies out), so this is the same bulk fetch the standalone bodies pay for.
bool GpuRigidBodyView::getInstancerPoseColumnOvStage(const bool wantOrientation,
                                                     float* const dstDev,
                                                     const GpuPointInstancerRecord* const recordsDev,
                                                     const InstancerAffine* const instancerInversesDev,
                                                     const PxU32* const offsetsDev,
                                                     const PxU32 numInstances) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstDev || !recordsDev || !instancerInversesDev || !offsetsDev)
        return false;
    if (numInstances == 0)
        return true;

    // As in getPoseColumnOvStage. An instancer array keeps authored slot identity, so a disabled
    // instance's slot is left unwritten rather than gathered through a sentinel record.
    if (!refreshRdGpuIndices())
        return false;

    const char* const attribName = wantOrientation ? "instancer orientation (ovstage)" : "instancer position (ovstage)";
    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    CUevent rdEvent = mGpuSimData->mCopyEvents[CopyEvent::eRdData];
    if (mNumRds > 0)
    {
        if (!mGpuSimData->awaitDirectGpuFetch(
                scene->getDirectGPUAPI().getRigidDynamicData(
                    (void*)mGpuSimData->mRdPoseDev, mRdGpuIndicesDev, PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE,
                    mNumRds, mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eRdPose), rdEvent),
                rdEvent, __FUNCTION__))
        {
            CARB_LOG_ERROR("%s: PxDirectGPUAPI::getRigidDynamicData(eGLOBAL_POSE) refused the read", attribName);
            return false;
        }
    }

    // Recorded whether or not the gather reported success; see getPoseColumnOvStage.
    const bool ok = fetchInstancerPoseColumnOvStage(dstDev, mGpuSimData->mRdPoseDev, mRbRecordsDev, recordsDev,
                                                    instancerInversesDev, offsetsDev, numInstances, wantOrientation);
    mGpuSimData->recordKernelDone(SharedDeviceBuffer::eRdPose);
    return ok;
}

bool GpuRigidBodyView::setInstancerPoseColumnOvStage(const bool wantOrientation,
                                                     const float* const srcDev,
                                                     const GpuPointInstancerRecord* const recordsDev,
                                                     const InstancerAffine* const instancerInversesDev,
                                                     const InstancerAffine* const instancerForwardsDev,
                                                     const PxU32* const rbRows,
                                                     const PxU32 numOutputs,
                                                     const PxU32 instancerIdx,
                                                     const uint64_t rowsToken)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (numOutputs == 0)
        return true;
    if (!srcDev || !recordsDev || !instancerInversesDev || !instancerForwardsDev || !rbRows)
        return false;
    if (!refreshRdGpuIndices())
        return false;

    const char* const attribName =
        wantOrientation ? "instancer orientation (ovstage write)" : "instancer position (ovstage write)";
    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    const PxU32* rowsDev = nullptr;
    void* packed = nullptr;
    PxRigidDynamicGPUIndex* packedIdx = nullptr;
    if (!ovStageWriteBuffers(attribName, rbRows, numOutputs, rowsToken, rowsDev, packed, packedIdx))
        return false;
    if (!packed)
        return false;

    // The bodies' CURRENT poses, for the read-modify-write. Read into the shared rd-pose buffer the
    // gather uses, so this takes that buffer's ordering event (ADR-0008 Decision 7).
    CUevent rdEvent = mGpuSimData->mCopyEvents[CopyEvent::eRdData];
    if (mNumRds > 0)
    {
        // awaitDirectGpuFetch, not a bare call: the finish event here is the non-null shared rd-pose
        // event, so PhysX may have queued work and recorded it even on refusal. The ordering edge --
        // and the producer drain plus buffer quarantine when that edge cannot be established -- has to
        // run either way, exactly as the getInstancerPoseColumnOvStage read does over this same buffer.
        if (!mGpuSimData->awaitDirectGpuFetch(
                scene->getDirectGPUAPI().getRigidDynamicData(
                    (void*)mGpuSimData->mRdPoseDev, mRdGpuIndicesDev, PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE,
                    mNumRds, mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eRdPose), rdEvent),
                rdEvent, __FUNCTION__))
        {
            CARB_LOG_ERROR("%s: PxDirectGPUAPI::getRigidDynamicData(eGLOBAL_POSE) refused the read that "
                           "preserves the pose half this column does not carry",
                           attribName);
            return false;
        }
    }

    const bool scattered = submitInstancerPoseColumnOvStage(
        static_cast<PxTransform*>(packed), srcDev, mGpuSimData->mRdPoseDev, mRbRecordsDev, recordsDev,
        instancerInversesDev, instancerForwardsDev, numOutputs, instancerIdx, wantOrientation);
    mGpuSimData->recordKernelDone(SharedDeviceBuffer::eRdPose);
    if (!scattered)
    {
        CARB_LOG_ERROR("%s: ovstage instancer scatter failed", attribName);
        return false;
    }

    CHECK_CU(getCudaShim()->eventRecord(
        reinterpret_cast<uintptr_t>(mGpuSimData->mApplyWaitEvents[ApplyEvent::eRdData]), uintptr_t(0), nullptr));
    scene->getDirectGPUAPI().setRigidDynamicData(packed, packedIdx, PxRigidDynamicGPUAPIWriteType::eGLOBAL_POSE,
                                                 numOutputs, mGpuSimData->mApplyWaitEvents[ApplyEvent::eRdData],
                                                 mGpuSimData->mApplySignalEvents[ApplyEvent::eRdData]);
    return true;
}

bool GpuRigidBodyView::setInstancerVelocityColumnOvStage(const bool wantAngular,
                                                         const float* const srcDev,
                                                         const GpuPointInstancerRecord* const recordsDev,
                                                         const PxU32* const rbRows,
                                                         const PxU32 numOutputs,
                                                         const uint64_t rowsToken)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (numOutputs == 0)
        return true;
    if (!srcDev || !recordsDev || !rbRows)
        return false;
    if (!refreshRdGpuIndices())
        return false;

    const char* const attribName = wantAngular ? "instancer angularVelocity (ovstage write)" :
                                                 "instancer linearVelocity (ovstage write)";
    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    const PxU32* rowsDev = nullptr;
    void* packed = nullptr;
    PxRigidDynamicGPUIndex* packedIdx = nullptr;
    if (!ovStageWriteBuffers(attribName, rbRows, numOutputs, rowsToken, rowsDev, packed, packedIdx))
        return false;
    if (!packed)
        return false;

    // No pre-read: linear and angular velocity are separate PhysX write types, so neither has to
    // preserve the other -- the same reason the standalone velocity path needs no RMW.
    //
    // The packed block is PxTransform-sized, which is larger than the PxVec3[N] this needs; reusing
    // it rather than allocating a second buffer is what the standalone paths do too.
    if (!submitInstancerVelocityColumnOvStage(static_cast<PxVec3*>(packed), srcDev, recordsDev, numOutputs))
    {
        CARB_LOG_ERROR("%s: ovstage instancer velocity scatter failed", attribName);
        return false;
    }

    CHECK_CU(getCudaShim()->eventRecord(
        reinterpret_cast<uintptr_t>(mGpuSimData->mApplyWaitEvents[ApplyEvent::eRdData]), uintptr_t(0), nullptr));
    scene->getDirectGPUAPI().setRigidDynamicData(
        packed, packedIdx,
        wantAngular ? PxRigidDynamicGPUAPIWriteType::eANGULAR_VELOCITY :
                      PxRigidDynamicGPUAPIWriteType::eLINEAR_VELOCITY,
        numOutputs, mGpuSimData->mApplyWaitEvents[ApplyEvent::eRdData],
        mGpuSimData->mApplySignalEvents[ApplyEvent::eRdData]);
    return true;
}

bool GpuRigidBodyView::getInstancerVelocityColumnOvStage(const bool wantAngular,
                                                         const bool wantAcceleration,
                                                         float* const dstDev,
                                                         const GpuPointInstancerRecord* const recordsDev,
                                                         const PxU32* const offsetsDev,
                                                         const PxU32 numInstances) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstDev || !recordsDev || !offsetsDev)
        return false;
    if (numInstances == 0)
        return true;

    if (!refreshRdGpuIndices())
        return false;

    const char* const attribName =
        wantAcceleration ? (wantAngular ? "instancer angular acceleration (ovstage)" :
                                          "instancer linear acceleration (ovstage)") :
                           (wantAngular ? "instancer angular velocity (ovstage)" :
                                          "instancer linear velocity (ovstage)");
    // The scratch and the buffer id below are deliberately NOT selected on wantAcceleration: velocity
    // and acceleration are read into the same per-component buffer (hence mRd*VelAccDev), one at a
    // time, and are never in flight together. Only the read type changes.
    const PxRigidDynamicGPUAPIReadType::Enum rdType =
        wantAcceleration ? (wantAngular ? PxRigidDynamicGPUAPIReadType::eANGULAR_ACCELERATION :
                                          PxRigidDynamicGPUAPIReadType::eLINEAR_ACCELERATION) :
                           (wantAngular ? PxRigidDynamicGPUAPIReadType::eANGULAR_VELOCITY :
                                          PxRigidDynamicGPUAPIReadType::eLINEAR_VELOCITY);
    const PxVec3* const rdScratch =
        wantAngular ? mGpuSimData->mRdAngularVelAccDev : mGpuSimData->mRdLinearVelAccDev;
    const PxU32 rdBuf = wantAngular ? SharedDeviceBuffer::eRdAngularVel : SharedDeviceBuffer::eRdLinearVel;

    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    CUevent rdEvent = mGpuSimData->mCopyEvents[CopyEvent::eRdData];
    if (mNumRds > 0)
    {
        if (!mGpuSimData->awaitDirectGpuFetch(
                scene->getDirectGPUAPI().getRigidDynamicData(
                    (void*)rdScratch, mRdGpuIndicesDev, rdType, mNumRds, mGpuSimData->kernelDoneEvent(rdBuf), rdEvent),
                rdEvent, __FUNCTION__))
        {
            CARB_LOG_ERROR("%s: PxDirectGPUAPI::getRigidDynamicData refused the read", attribName);
            return false;
        }
    }

    const bool ok =
        fetchInstancerVelocityColumnOvStage(dstDev, rdScratch, mRbRecordsDev, recordsDev, offsetsDev, numInstances);
    mGpuSimData->recordKernelDone(rdBuf);
    return ok;
}

bool GpuRigidBodyView::getPositionsOvStage(const TensorDesc* dstTensor,
                                           const PxU32* outRecordIdx,
                                           const PxU32 numOutputs,
                                           const uint64_t rowsToken) const
{
    return getPoseColumnOvStage("position (ovstage)", dstTensor, /*wantOrientation=*/false, outRecordIdx, numOutputs,
                                rowsToken);
}

bool GpuRigidBodyView::getOrientationsOvStage(const TensorDesc* dstTensor,
                                              const PxU32* outRecordIdx,
                                              const PxU32 numOutputs,
                                              const uint64_t rowsToken) const
{
    return getPoseColumnOvStage("orientation (ovstage)", dstTensor, /*wantOrientation=*/true, outRecordIdx, numOutputs,
                                rowsToken);
}

bool GpuRigidBodyView::getLinearVelocitiesOvStage(const TensorDesc* dstTensor,
                                                  const PxU32* outRecordIdx,
                                                  const PxU32 numOutputs,
                                                  const uint64_t rowsToken) const
{
    return getVelocityColumnOvStage("linear velocity (ovstage)", dstTensor,
                                    PxRigidDynamicGPUAPIReadType::eLINEAR_VELOCITY,
                                    PxArticulationGPUAPIReadType::eLINK_LINEAR_VELOCITY,
                                    mGpuSimData->mRdLinearVelAccDev, mGpuSimData->mLinkOrRootLinearVelAccDev,
                                    SharedDeviceBuffer::eRdLinearVel, SharedDeviceBuffer::eLinkOrRootLinearVel,
                                    CopyEvent::eArtiLinkLinearVelocities, outRecordIdx, numOutputs, rowsToken);
}

bool GpuRigidBodyView::getAngularVelocitiesOvStage(const TensorDesc* dstTensor,
                                                   const PxU32* outRecordIdx,
                                                   const PxU32 numOutputs,
                                                   const uint64_t rowsToken) const
{
    return getVelocityColumnOvStage("angular velocity (ovstage)", dstTensor,
                                    PxRigidDynamicGPUAPIReadType::eANGULAR_VELOCITY,
                                    PxArticulationGPUAPIReadType::eLINK_ANGULAR_VELOCITY,
                                    mGpuSimData->mRdAngularVelAccDev, mGpuSimData->mLinkOrRootAngularVelAccDev,
                                    SharedDeviceBuffer::eRdAngularVel, SharedDeviceBuffer::eLinkOrRootAngularVel,
                                    CopyEvent::eArtiLinkAngularVelocities, outRecordIdx, numOutputs, rowsToken);
}

bool GpuRigidBodyView::getLinearAccelerationsOvStage(const TensorDesc* dstTensor,
                                                    const PxU32* outRecordIdx,
                                                    const PxU32 numOutputs,
                                                    const uint64_t rowsToken) const
{
    return getVelocityColumnOvStage("linear acceleration (ovstage)", dstTensor,
                                    PxRigidDynamicGPUAPIReadType::eLINEAR_ACCELERATION,
                                    PxArticulationGPUAPIReadType::eLINK_LINEAR_ACCELERATION,
                                    mGpuSimData->mRdLinearVelAccDev, mGpuSimData->mLinkOrRootLinearVelAccDev,
                                    SharedDeviceBuffer::eRdLinearVel, SharedDeviceBuffer::eLinkOrRootLinearVel,
                                    CopyEvent::eArtiLinkLinearVelocities, outRecordIdx, numOutputs, rowsToken);
}

bool GpuRigidBodyView::getAngularAccelerationsOvStage(const TensorDesc* dstTensor,
                                                     const PxU32* outRecordIdx,
                                                     const PxU32 numOutputs,
                                                     const uint64_t rowsToken) const
{
    return getVelocityColumnOvStage("angular acceleration (ovstage)", dstTensor,
                                    PxRigidDynamicGPUAPIReadType::eANGULAR_ACCELERATION,
                                    PxArticulationGPUAPIReadType::eLINK_ANGULAR_ACCELERATION,
                                    mGpuSimData->mRdAngularVelAccDev, mGpuSimData->mLinkOrRootAngularVelAccDev,
                                    SharedDeviceBuffer::eRdAngularVel, SharedDeviceBuffer::eLinkOrRootAngularVel,
                                    CopyEvent::eArtiLinkAngularVelocities, outRecordIdx, numOutputs, rowsToken);
}

bool GpuRigidBodyView::gatherPoseColumnOvStage(const bool wantOrientation,
                                               const PxU32* outRecordIdxDev,
                                               const PxU32 numOutputs,
                                               const TensorDesc* dstTensor) const
{
    // dstTensor was validated by the caller, before it did any engine work.
    return fetchRbPoseColumnOvStage(static_cast<float*>(dstTensor->data), mGpuSimData->mRdPoseDev,
                                    mGpuSimData->mLinkOrRootTransformsDev, mRbRecordsDev, outRecordIdxDev, numOutputs,
                                    mGpuSimData->mMaxLinks, wantOrientation);
}

bool GpuRigidBodyView::gatherVelocityColumnOvStage(const PxVec3* rdSrc,
                                                   const PxVec3* linkSrc,
                                                   const PxU32* outRecordIdxDev,
                                                   const PxU32 numOutputs,
                                                   const TensorDesc* dstTensor) const
{
    // dstTensor was validated by the caller, before it did any engine work.
    return fetchRbVelocityColumnOvStage(static_cast<float*>(dstTensor->data), rdSrc, linkSrc, mRbRecordsDev,
                                        outRecordIdxDev, numOutputs, mGpuSimData->mMaxLinks);
}

// ADR-0012: the ovstage column WRITE, mirroring getPoseColumnOvStage above -- and, like it,
// carrying NO host block. The count PhysX needs is numOutputs, known before anything launches, so
// nothing has to be read back from the device; that is what a flags-and-compaction scatter cannot
// do, since its count comes off a thrust iterator on the host.
//
// Reads the covered bodies' current poses first, because PxRigidDynamicGPUAPIWriteType carries one
// eGLOBAL_POSE and a session writes one attribute -- the half not being written has to come from
// somewhere, and the engine is the only place it exists. The read is numOutputs-wide, not
// scene-wide: it goes through the same packed index list the write does.
bool GpuRigidBodyView::setPoseColumnOvStage(const char* const attribName,
                                            const TensorDesc* const srcTensor,
                                            const bool wantOrientation,
                                            const PxU32* outRecordIdx,
                                            const PxU32 numOutputs,
                                            const uint64_t rowsToken)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!srcTensor || !srcTensor->data)
        return false;
    if (!checkTensorDevice(*srcTensor, mDevice, attribName, __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, attribName, __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, numOutputs * (wantOrientation ? 4u : 3u), attribName, __FUNCTION__))
    {
        return false;
    }
    if (!refreshRdGpuIndices())
        return false;
    if (numOutputs == 0)
        return true;

    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    const PxU32* outRecordIdxDev = nullptr;
    PxTransform* packedPose = nullptr;
    PxRigidDynamicGPUIndex* packedIdx = nullptr;
    if (!ovStageWriteBuffers(attribName, outRecordIdx, numOutputs, rowsToken, outRecordIdxDev,
                             reinterpret_cast<void*&>(packedPose), packedIdx))
        return false;

    // Current poses for exactly the covered rows. Status IS checked: PhysX refuses these calls in
    // states the caller cannot see (pre-step being the documented one) and writes NOTHING when it
    // does; the scratch is not zeroed, so an unchecked refusal would overlay the caller's component
    // onto stale contents and publish that as poses.
    //
    // startEvent orders PhysX behind the index build (see ovStageWriteBuffers). This read used to pass
    // NULL and was the TestOvstageWriteScatter context-poisoning flake.
    CUevent rdEvent = mGpuSimData->mCopyEvents[CopyEvent::eRdData];
    if (!scene->getDirectGPUAPI().getRigidDynamicData((void*)packedPose, packedIdx,
                                                      PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE, numOutputs,
                                                      mOvStageIdxReadyEvent, rdEvent))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::getRigidDynamicData(eGLOBAL_POSE) refused the read that preserves "
                       "the component this write does not set",
                       attribName);
        return false;
    }
    // Stream-ordered, not blocking: the scatter below waits for the fill on the device.
    CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(rdEvent), 0, nullptr));

    if (!submitRbPackedPoseOvStage(packedPose, static_cast<const float*>(srcTensor->data), mRbRecordsDev,
                                    outRecordIdxDev, numOutputs, wantOrientation))
    {
        CARB_LOG_ERROR("%s: scatter failed", attribName);
        return false;
    }

    // Record-then-pass-as-startEvent: PhysX is ordered behind our scatter on the device, with no
    // host round trip. This is the convention the articulation write already follows
    // (GpuArticulationView::setRootTransforms); the legacy rigid write is the one that blocks
    // instead, which is what ADR-0008 flagged as left for this work.
    //
    // The buffers are private to this view, so this is the WHOLE of the ordering: nothing here
    // touches mRdPoseDev, so there is no shared scene buffer for a sibling read to race on.
    CHECK_CU(getCudaShim()->eventRecord(
        reinterpret_cast<uintptr_t>(mGpuSimData->mApplyWaitEvents[ApplyEvent::eRdData]), uintptr_t(0), nullptr));
    scene->getDirectGPUAPI().setRigidDynamicData((void*)packedPose, packedIdx,
                                                 PxRigidDynamicGPUAPIWriteType::eGLOBAL_POSE, numOutputs,
                                                 mGpuSimData->mApplyWaitEvents[ApplyEvent::eRdData],
                                                 mGpuSimData->mApplySignalEvents[ApplyEvent::eRdData]);
    return true;
}

// As above for a velocity component, and simpler in the way that matters: eLINEAR_VELOCITY and
// eANGULAR_VELOCITY are separate write types, so there is nothing to preserve and no pre-read.
//
// The caller's column is ALREADY the packed payload -- a dense [numOutputs, 3] float column is
// exactly PxVec3[numOutputs] -- so it is handed to PhysX directly. No scatter kernel, no copy.
bool GpuRigidBodyView::setVelocityColumnOvStage(const char* const attribName,
                                                const TensorDesc* const srcTensor,
                                                const PxRigidDynamicGPUAPIWriteType::Enum rdType,
                                                const PxU32* outRecordIdx,
                                                const PxU32 numOutputs,
                                                const uint64_t rowsToken)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!srcTensor || !srcTensor->data)
        return false;
    if (!checkTensorDevice(*srcTensor, mDevice, attribName, __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, attribName, __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, numOutputs * 3u, attribName, __FUNCTION__))
    {
        return false;
    }
    if (!refreshRdGpuIndices())
        return false;
    if (numOutputs == 0)
        return true;

    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    const PxU32* outRecordIdxDev = nullptr;
    void* packedData = nullptr; // unused on this path; the caller's column IS the payload
    PxRigidDynamicGPUIndex* packedIdx = nullptr;
    if (!ovStageWriteBuffers(attribName, outRecordIdx, numOutputs, rowsToken, outRecordIdxDev, packedData,
                             packedIdx))
        return false;

    // Only the index build ran on our stream, but the ordering obligation is identical: PhysX must
    // not read the index list before that kernel finishes.
    CHECK_CU(getCudaShim()->eventRecord(
        reinterpret_cast<uintptr_t>(mGpuSimData->mApplyWaitEvents[ApplyEvent::eRdData]), uintptr_t(0), nullptr));
    scene->getDirectGPUAPI().setRigidDynamicData(srcTensor->data, packedIdx, rdType, numOutputs,
                                                 mGpuSimData->mApplyWaitEvents[ApplyEvent::eRdData],
                                                 mGpuSimData->mApplySignalEvents[ApplyEvent::eRdData]);
    return true;
}

void GpuRigidBodyView::waitOvStageWriteDone() const
{
    if (!mGpuSimData)
        return;
    CUevent done = mGpuSimData->mApplySignalEvents[ApplyEvent::eRdData];
    if (!done)
        return;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    // Waits for whatever last recorded this event, which is at least as late as our last write. That
    // is over-waiting when a sibling raced in, never under-waiting, which is the safe direction for
    // a guard standing between an in-flight read and a free.
    CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(done), nullptr));
}

bool GpuRigidBodyView::setPositionsOvStage(const TensorDesc* srcTensor,
                                           const PxU32* outRecordIdx,
                                           const PxU32 numOutputs,
                                           const uint64_t rowsToken)
{
    return setPoseColumnOvStage("position", srcTensor, /*wantOrientation*/ false, outRecordIdx, numOutputs, rowsToken);
}

bool GpuRigidBodyView::setOrientationsOvStage(const TensorDesc* srcTensor,
                                              const PxU32* outRecordIdx,
                                              const PxU32 numOutputs,
                                              const uint64_t rowsToken)
{
    return setPoseColumnOvStage("orientation", srcTensor, /*wantOrientation*/ true, outRecordIdx, numOutputs,
                                rowsToken);
}

bool GpuRigidBodyView::ovStageRowsAreLinks(const PxU32* rows, const PxU32 numOutputs) const
{
    if (numOutputs == 0)
        return false;
    const PxU32 rec = rows ? rows[0] : 0;
    if (rec >= rbRecords.size())
        return false; // let the rigid path report the out-of-range row, where that check already lives
    return rbRecords[rec].physxLinkIdx != 0xffffffff;
}

bool GpuRigidBodyView::setLinkWrenchOvStage(const char* const attribName,
                                           const TensorDesc* const srcTensor,
                                           const PxU32* const outRecordIdx,
                                           const PxU32 numOutputs,
                                           const uint64_t rowsToken,
                                           const PxU32 comps)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (!srcTensor || !srcTensor->data)
        return false;
    if (numOutputs == 0)
        return true;
    if (!checkTensorDevice(*srcTensor, mDevice, attribName, __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, attribName, __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, numOutputs * comps, attribName, __FUNCTION__))
    {
        return false;
    }
    if (mNumArtis == 0 || mGpuSimData->mMaxLinks == 0)
    {
        CARB_LOG_ERROR("%s: this view holds no articulation links", attribName);
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    // Rows only. ovStageWriteBuffers is deliberately NOT used here: its packed index build reads
    // tensorRdIdx, which a link does not have, and its validation rejects exactly this case.
    const PxU32* rowsDev = ovStageRowsDevice(outRecordIdx, numOutputs, rowsToken);
    if (!rowsDev)
        return false;

    if (!getComsCacheStateValid())
        updateCMassData();

    const PxU32 blockLinks = mNumArtis * mGpuSimData->mMaxLinks;

    // ZERO FIRST, and this is the one place the link route's semantics differ from the rigid one.
    //
    // The rigid path writes only the rows it packed. Here the data block covers the whole view, so a
    // link this query did not match would otherwise carry whatever the block held. There is no
    // read-modify-write available to preserve it either: a force is write-only, so PhysX offers no
    // read to recover a current value from.
    //
    // Zero is the honest fill BECAUSE the attribute is a per-step control input that PhysX clears
    // after every step: a link written zero is in the state it would have been in had nothing been
    // applied. The caveat is real and worth stating -- if something else applied a force to an
    // unmatched link earlier in the SAME step, this clears it.
    CHECK_CUDA(cudaMemsetAsync(mGpuSimData->mLinkForcesDev, 0, size_t(blockLinks) * sizeof(PxVec3), nullptr));
    if (comps == 9)
        CHECK_CUDA(cudaMemsetAsync(mGpuSimData->mLinkTorquesDev, 0, size_t(blockLinks) * sizeof(PxVec3), nullptr));

    // Link poses, needed only by the wrench conversion. Read into the shared link-transform buffer,
    // so this takes the ADR-0008 Decision 7 start event that buffer's protocol requires.
    if (comps == 9)
    {
        CUevent poseEvent = nullptr;
        if (!scene->getDirectGPUAPI().getArticulationData(
                (void*)mGpuSimData->mLinkOrRootTransformsDev, mArtiIndicesDev,
                PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, mNumArtis,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eLinkOrRootTransforms), poseEvent))
        {
            CARB_LOG_ERROR("%s: PxDirectGPUAPI::getArticulationData refused the link-pose read the "
                           "torque-about-COM conversion needs",
                           attribName);
            return false;
        }
        if (poseEvent)
            CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(poseEvent), 0, nullptr));
    }

    if (!submitLinkWrenchOvStage(mGpuSimData->mLinkForcesDev, mGpuSimData->mLinkTorquesDev,
                                  static_cast<const float*>(srcTensor->data),
                                  mGpuSimData->mLinkOrRootTransformsDev, cMassLocalPosePosDev, mRbRecordsDev,
                                  rowsDev, numOutputs, mGpuSimData->mMaxLinks, comps))
    {
        CARB_LOG_ERROR("%s: ovstage link scatter failed", attribName);
        return false;
    }
    if (comps == 9)
        mGpuSimData->recordKernelDone(SharedDeviceBuffer::eLinkOrRootTransforms);

    // mArtiIndicesDev and mNumArtis: the whole view, so the count is known on the host and no
    // compaction runs. The tensor binding's equivalent calls fillArtiFT, which takes its count off
    // the device -- the host block this path exists without.
    CHECK_CU(getCudaShim()->eventRecord(
        reinterpret_cast<uintptr_t>(mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiLinkForces]), uintptr_t(0),
        nullptr));
    scene->getDirectGPUAPI().setArticulationData(
        (void*)mGpuSimData->mLinkForcesDev, mArtiIndicesDev, PxArticulationGPUAPIWriteType::eLINK_FORCE, mNumArtis,
        mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiLinkForces],
        mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkForces]);
    if (comps == 9)
    {
        scene->getDirectGPUAPI().setArticulationData(
            (void*)mGpuSimData->mLinkTorquesDev, mArtiIndicesDev, PxArticulationGPUAPIWriteType::eLINK_TORQUE,
            mNumArtis, mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiLinkTorques],
            mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkTorques]);
    }

    // PhysX's applies above read mLinkForcesDev/mLinkTorquesDev ASYNC (finishEvent = mApplySignalEvents).
    // The next setLinkWrenchOvStage zeros those same shared buffers with cudaMemsetAsync on our stream,
    // so order our stream behind the applies here -- without this the memset could clear them mid-apply
    // (ADR-0008 D7, the reverse direction the host block used to cover).
    CHECK_CU(getCudaShim()->streamWaitEvent(
        uintptr_t(0), reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkForces]), 0,
        nullptr));
    if (comps == 9)
        CHECK_CU(getCudaShim()->streamWaitEvent(
            uintptr_t(0), reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkTorques]),
            0, nullptr));
    return true;
}

bool GpuRigidBodyView::setWrenchesOvStage(const TensorDesc* const srcTensor,
                                         const PxU32* const outRecordIdx,
                                         const PxU32 numOutputs,
                                         const uint64_t rowsToken)
{
    // A LINK query cannot use the rigid-dynamic packed path -- a link has no rd index to pack, and
    // ovStageWriteBuffers rejects exactly that. Detected from the first row's record rather than
    // passed in: a query is one object type, so the rows are all links or all rigid dynamics.
    if (ovStageRowsAreLinks(outRecordIdx, numOutputs))
        return setLinkWrenchOvStage("wrench", srcTensor, outRecordIdx, numOutputs, rowsToken, 9u);

    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (!srcTensor || !srcTensor->data)
        return false;
    if (!checkTensorDevice(*srcTensor, mDevice, "wrench", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "wrench", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, numOutputs * 9u, "wrench", __FUNCTION__))
    {
        return false;
    }
    if (!refreshRdGpuIndices())
        return false;
    if (numOutputs == 0)
        return true;

    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    const PxU32* rowsDev = nullptr;
    void* packed = nullptr;
    PxRigidDynamicGPUIndex* packedIdx = nullptr;
    if (!ovStageWriteBuffers("wrench", outRecordIdx, numOutputs, rowsToken, rowsDev, packed, packedIdx))
        return false;
    if (!packed)
        return false;

    // The COM table the conversion needs. Refreshed on the same condition the binding's force path
    // uses -- setCMassLocalPose invalidates it and nothing else does.
    if (!getComsCacheStateValid())
        updateCMassData();

    if (numOutputs > mOvStageWrenchCapacity)
    {
        // The old buffer may still be read by an in-flight wrench apply: setRigidDynamicData(eFORCE/
        // eTORQUE) below signals eRdForces and does NOT block, so a later group needing more rows can
        // reach this free while PhysX is still reading it. Freeing device memory out from under an
        // in-flight read is cudaErrorIllegalAddress, which poisons the context for every later CUDA
        // call rather than failing here -- so wait for the wrench apply first, as ovStageWriteBuffers
        // does for its pose/index buffers (but on the wrench signal, eRdForces, not eRdData).
        CUevent wrenchDone = mGpuSimData->mApplySignalEvents[ApplyEvent::eRdForces];
        if (wrenchDone)
            CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(wrenchDone), nullptr));
        CHECK_CUDA(cudaFree(mOvStageWrenchDev));
        mOvStageWrenchDev = nullptr;
        mOvStageWrenchCapacity = 0;
        if (!prepareDeviceData((void**)&mOvStageWrenchDev, nullptr, size_t(numOutputs) * 2 * sizeof(PxVec3),
                               "mOvStageWrenchDev"))
            return false;
        mOvStageWrenchCapacity = numOutputs;
    }
    PxVec3* outForces = mOvStageWrenchDev;
    PxVec3* outTorques = mOvStageWrenchDev + numOutputs;

    // Poses for exactly the addressed rows, read through the same packed index list, so the kernel
    // indexes them by output slot rather than by record.
    //
    // startEvent orders PhysX behind the index build (see ovStageWriteBuffers). That is the only edge
    // this read needs: mOvStagePackedDev belongs to THIS view and this write is its only user, so the
    // ADR-0008 Decision 7 protocol for the SHARED buffers does not apply here.
    //
    // The completion event is a local, since CopyEvent indexes articulation copies only -- the rigid
    // paths in this file take PhysX's out-event the same way.
    PxTransform* poses = static_cast<PxTransform*>(packed);
    CUevent readEvent = nullptr;
    if (!scene->getDirectGPUAPI().getRigidDynamicData(poses, packedIdx, PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE,
                                                      numOutputs, mOvStageIdxReadyEvent, readEvent))
    {
        CARB_LOG_ERROR("wrench: PxDirectGPUAPI::getRigidDynamicData refused the pose read the "
                       "torque-about-COM conversion needs");
        return false;
    }
    if (readEvent)
        CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(readEvent), 0, nullptr));

    if (!submitRbWrenchOvStage(outForces, outTorques, static_cast<const float*>(srcTensor->data), poses,
                                cMassLocalPosePosDev, rowsDev, numOutputs))
    {
        CARB_LOG_ERROR("wrench: ovstage scatter failed");
        return false;
    }

    // Two write types, one packed index list, and no compaction anywhere: numOutputs IS the count,
    // known before the launch. The tensor binding's equivalent flags every body and takes the count
    // off the device through thrust::copy_if, which is the host block this path exists without.
    CHECK_CU(getCudaShim()->eventRecord(
        reinterpret_cast<uintptr_t>(mGpuSimData->mApplyWaitEvents[ApplyEvent::eRdForces]), uintptr_t(0), nullptr));
    scene->getDirectGPUAPI().setRigidDynamicData(outForces, packedIdx, PxRigidDynamicGPUAPIWriteType::eFORCE,
                                                 numOutputs, mGpuSimData->mApplyWaitEvents[ApplyEvent::eRdForces],
                                                 mGpuSimData->mApplySignalEvents[ApplyEvent::eRdForces]);
    scene->getDirectGPUAPI().setRigidDynamicData(outTorques, packedIdx, PxRigidDynamicGPUAPIWriteType::eTORQUE,
                                                 numOutputs, mGpuSimData->mApplyWaitEvents[ApplyEvent::eRdForces],
                                                 mGpuSimData->mApplySignalEvents[ApplyEvent::eRdForces]);
    return true;
}

bool GpuRigidBodyView::setForcesOvStage(const TensorDesc* srcTensor,
                                       const PxU32* outRecordIdx,
                                       const PxU32 numOutputs,
                                       const uint64_t rowsToken)
{
    // A LINK query cannot use the rigid-dynamic packed path -- a link has no rd index to pack, and
    // ovStageWriteBuffers rejects exactly that. Detected from the first row's record rather than
    // passed in: a query is one object type, so the rows are all links or all rigid dynamics.
    if (ovStageRowsAreLinks(outRecordIdx, numOutputs))
        return setLinkWrenchOvStage("force", srcTensor, outRecordIdx, numOutputs, rowsToken, 3u);

    // Routed through the vec3-column path, whose name says "velocity" only because velocity was its
    // first caller: what it actually does is hand a dense [N,3] column and a packed index list to one
    // PxRigidDynamicGPUAPIWriteType, which is exactly what a force write is.
    //
    // NOT through the tensor binding's applyForces machinery, deliberately. That path flags every
    // body, compacts with thrust::copy_if and takes the resulting COUNT off the device
    // (fillRdFT returns an iterator difference), which is a host block -- the same one the ovstage
    // pose path was rewritten to remove. Here the row list IS the set being written, so the count is
    // numOutputs, known before the launch, and no compaction is needed at all.
    return setVelocityColumnOvStage("force", srcTensor, PxRigidDynamicGPUAPIWriteType::eFORCE, outRecordIdx,
                                    numOutputs, rowsToken);
}

bool GpuRigidBodyView::setLinearVelocitiesOvStage(const TensorDesc* srcTensor,
                                                  const PxU32* outRecordIdx,
                                                  const PxU32 numOutputs,
                                                  const uint64_t rowsToken)
{
    return setVelocityColumnOvStage("linearVelocity", srcTensor, PxRigidDynamicGPUAPIWriteType::eLINEAR_VELOCITY,
                                    outRecordIdx, numOutputs, rowsToken);
}

bool GpuRigidBodyView::setAngularVelocitiesOvStage(const TensorDesc* srcTensor,
                                                   const PxU32* outRecordIdx,
                                                   const PxU32 numOutputs,
                                                   const uint64_t rowsToken)
{
    return setVelocityColumnOvStage("angularVelocity", srcTensor, PxRigidDynamicGPUAPIWriteType::eANGULAR_VELOCITY,
                                    outRecordIdx, numOutputs, rowsToken);
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

    if (!refreshRdGpuIndices())
        return false;
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

    if (!refreshRdGpuIndices())
        return false;
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
    if (!refreshRdGpuIndices())
        return false;

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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
    if (!refreshRdGpuIndices())
        return false;

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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
    if (!refreshRdGpuIndices())
        return false;
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
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
        // Both completion events are deliberately null here, so PhysX records nothing and there is
        // no producer to drain -- the waits below never fire and SYNCHRONIZE_CUDA does the ordering.
        // Only the refusal status is actionable, and it matters: these poses are what the force
        // submission below transforms, so a refused read applies forces at stale positions.
        if (mNumRds > 0 &&
            !scene->getDirectGPUAPI().getRigidDynamicData(
                (void*)mGpuSimData->mRdPoseDev, mRdGpuIndicesDev, PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE, mNumRds,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eRdPose), rdCopyEvent))
        {
            CARB_LOG_ERROR("%s: PxDirectGPUAPI refused the rigid-dynamic pose read", __FUNCTION__);
            return false;
        }

        if (mNumArtis > 0 && !scene->getDirectGPUAPI().getArticulationData(
                                 (void*)mGpuSimData->mLinkOrRootTransformsDev, mArtiIndicesDev,
                                 PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, mNumArtis,
                                 mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eLinkOrRootTransforms), artiCopyEvent))
        {
            CARB_LOG_ERROR("%s: PxDirectGPUAPI refused the articulation link pose read", __FUNCTION__);
            return false;
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
    const bool forcesSubmitted = submitRbForces(
        mGpuSimData->mRdForcesDev, mGpuSimData->mRdTorquesDev, mGpuSimData->mLinkForcesDev, mGpuSimData->mLinkTorquesDev,
        mRdDirtyFlagsDev, mArtiDirtyFlagsDev, mArtiLinksDirtyFlagsDev, mGpuSimData->mLinkOrRootTransformsDev,
        mGpuSimData->mRdPoseDev, cMassLocalPosePosDev, forceData, torqueData, positionData, indices, numIndices,
        mGpuSimData->mMaxLinks, mRbRecordsDev, isGlobal, validForceTensor, validTorqueTensor, validPositionTensor);

    // submitRbForces reads both pose buffers, so the next DirectGPU fill of either has to wait for
    // it (ADR-0008 Decision 7). Recorded before the branch: the submit can fail after launching.
    mGpuSimData->recordKernelDone(SharedDeviceBuffer::eRdPose);
    mGpuSimData->recordKernelDone(SharedDeviceBuffer::eLinkOrRootTransforms);
    if (!forcesSubmitted)
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

// Non-const masked setters (DirectGPU paths -- GPU mask/indices)
GPU_RB_MASKED_SETTER(setKinematicTargets, )
GPU_RB_MASKED_SETTER(setTransforms, )
GPU_RB_MASKED_SETTER(setVelocities, )
GPU_RB_MASKED_SETTER(applyForces, )
GPU_RB_MASKED_SETTER(setMasses, )
GPU_RB_MASKED_SETTER(setCOMs, )
GPU_RB_MASKED_SETTER(setInertias, )
// setDisable*/material/rest/contact/compliant Masked: BaseRigidBodyView

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

// ============================================================================
// OMPE-103213: CPU-only PhysX property APIs -- refuse GPU tensors rather than
// silently staging them to host. BaseRigidBodyView already requires device==-1.
// Masked wrappers for those APIs live on BaseRigidBodyView.
// ============================================================================

namespace
{
bool requireHostTensor(const TensorDesc* desc, const char* tensorName, const char* funcName)
{
    if (desc && desc->data && desc->device >= 0)
    {
        CARB_LOG_ERROR("%s: %s tensor must be on host (CPU); GPU tensors not supported", funcName, tensorName);
        return false;
    }
    return true;
}
} // namespace


// setDisable*/material/rest/contact/compliant Masked: BaseRigidBodyView


// setDisableGravities keeps a DirectGPU wake-for-refresh after the CPU flag write.
// OMPE-103213: require host tensors -- no silent GPU staging.
bool GpuRigidBodyView::setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    // Host-index wake used as a DirectGPU body-sim refresh, not a public wake_up.
    // Skip eKINEMATIC: kinematics ignore gravity, and PxRigidDynamic::wakeUp emits
    // a checked-build error ("Body must be non-kinematic!"). Skip sleeping bodies:
    // they are not integrated, so a stale GPU disableGravity is irrelevant until
    // they wake. Also skip eDISABLE_SIMULATION (same as BaseRigidBodyView::wakeUp).
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

    if (!requireHostTensor(srcTensor, "src", "setDisableGravities"))
        return false;
    if (!requireHostTensor(indexTensor, "index", "setDisableGravities"))
        return false;
    if (indexTensor && indexTensor->data && !checkIndexTensorSize(*indexTensor, getCount(), "setDisableGravities"))
        return false;

    const TensorDesc* idxArg = (indexTensor && indexTensor->data) ? indexTensor : nullptr;
    const bool ok = BaseRigidBodyView::setDisableGravities(srcTensor, idxArg);
    if (!ok)
        return false;

    wakeForGravityRefresh(idxArg);
    return true;
}

bool GpuRigidBodyView::setDisableSimulations(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!requireHostTensor(srcTensor, "src", "setDisableSimulations"))
        return false;
    if (!requireHostTensor(indexTensor, "index", "setDisableSimulations"))
        return false;
    if (indexTensor && indexTensor->data && !checkIndexTensorSize(*indexTensor, getCount(), "setDisableSimulations"))
        return false;

    const bool ok = BaseRigidBodyView::setDisableSimulations(srcTensor, indexTensor);
    if (!ok)
        return false;

    // Sibling views on the same scene need an epoch bump so their next refresh
    // sees the membership change (OMPE-94459).
    markRdDisableDirty();

    // A fixed-membership tensor view still becomes stale. The ovstage superset keeps sentinel rows
    // so a disable cannot take unrelated reads/writes -- or its own re-enable write -- down.
    if (mInvalidateOnDisabledRd)
    {
        for (const RigidBodyEntry& entry : mEntries)
        {
            if (entry.type != RigidBodyType::eRigidDynamic || !entry.body)
                continue;
            if (entry.body->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION))
            {
                invalidateMappingForDisabledRd("setDisableSimulations left a rigid dynamic disabled");
                break;
            }
        }
    }
    return true;
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
        // Conservative in both directions: this is called for enables too, and resolving which one
        // happened costs the walk the hint exists to avoid. A spurious true is one scan, retired by
        // the superset's next refresh; a missed true is a device read against a freed island node.
        mGpuSimData->mMayHaveDisabledRd = true;
    }
}

} // namespace tensors
} // namespace physx
} // namespace omni

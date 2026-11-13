// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-CCT-001
 * @covers AC-3
 *
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
 * @covers AC-1, AC-5, AC-30
 *
 * @implements REQ-TENSOR-INDEX-001
 * @covers AC-1 AC-2 AC-3
 * @implements REQ-TENSOR-CPU-ONLY-001
 * @covers AC-1 AC-2 AC-4
 *
 * @implements REQ-INPUT-DEVICE-001
 * @covers AC-1 AC-1b AC-2 AC-3 AC-8 AC-9 AC-14
 *
 * @implements REQ-INPUT-CORE-001
 * @covers AC-4 AC-5 AC-10
 *
 * @implements REQ-TENSOR-VIEW-001
 * @covers AC-16
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

#include <cstring>
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
        rb.orientationFixup = mEntries[i].orientationFixup;

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
    mNumLinkRecords = PxU32(linkMap.size());

    // Every device buffer of the view lives in the scene's CUDA context, which need not be current on the caller's
    // thread.
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

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

    prepareDeviceData((void**)&mRbIndicesDev, rbIndices.data(), numBodies * sizeof(PxU32), "mRbIndicesDev");
    // rb indexing data
    prepareDeviceData((void**)&mRbRecordsDev, rbRecords.data(), numBodies * sizeof(GpuRigidBodyRecord), "mRbRecordsDev");
    {
        std::vector<PxVec3> worldScales(numBodies);
        for (PxU32 i = 0; i < numBodies; i++)
            worldScales[i] = mEntries[i].worldScale;
        // A failed copy leaves the buffer allocated but unwritten. Freed, so the eWorldScale gather finds no buffer
        // and fails the read instead of publishing whatever the allocation held.
        if (!prepareDeviceData((void**)&mWorldScalesDev, worldScales.data(), numBodies * sizeof(PxVec3),
                               "mWorldScalesDev") &&
            mWorldScalesDev)
        {
            CHECK_CUDA(cudaFree(mWorldScalesDev));
            mWorldScalesDev = nullptr;
        }
    }

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
            uploadToDevice(
                mRdGpuIndicesDev, rdGpuIndices.data(), rdGpuIndices.size() * sizeof(PxRigidDynamicGPUIndex));
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
        // Strided by the simulation data's links, as the force submission and fillArtiFT index it, not the view's.
        prepareDeviceData((void**)&mArtiLinksDirtyFlagsDev, nullptr,
                          mNumArtis * mGpuSimData->mMaxLinks * sizeof(PxArticulationCacheFlags),
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

        // Drain before freeing: an ovstage gather launched on the null stream returns without
        // synchronizing, so a view destroyed in the same frame as its last read can free records the
        // kernel is still reading. cudaFree has historically synchronized implicitly; that is not a
        // guarantee to rely on. A stream-ordered read or commit reads them on its caller's stream
        // instead, which the drain does not cover: it is waited for as well.
        CHECK_CUDA(cudaStreamSynchronize(nullptr));
        mGpuSimData->waitForTableReaders();

        // PhysX consumes the ovstage write buffers on its own streams, which the drain above does not
        // cover: wait for the last pose/velocity and wrench applies before freeing what they read.
        if (CUevent applyDone = mGpuSimData->mApplySignalEvents[ApplyEvent::eRdData])
            CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(applyDone), nullptr));
        if (CUevent wrenchDone = mGpuSimData->mApplySignalEvents[ApplyEvent::eRdForces])
            CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(wrenchDone), nullptr));

        CHECK_CUDA(cudaFree(mRbIndicesDev));
        CHECK_CUDA(cudaFree(mRdGpuIndicesDev));
        CHECK_CUDA(cudaFree(mArtiIndicesDev));
        CHECK_CUDA(cudaFree(mRbRecordsDev));
        CHECK_CUDA(cudaFree(mWorldScalesDev));
        for (OvStageRowsSlot& rowsSlot : mOvStageRows)
            CHECK_CUDA(cudaFree(rowsSlot.dev));
        CHECK_CUDA(cudaFree(mOvStageWrenchDev));
        CHECK_CUDA(cudaFree(mOvStageLinkWrenchArtisDev));
        CHECK_CUDA(cudaFree(mOvStageLinkWrenchRowArtiDev));
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
                mArtiLinksDirtyFlagsDev, 0, mNumArtis * mGpuSimData->mMaxLinks * sizeof(PxArticulationCacheFlags))))
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, mNumArtis > 0, false);

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
        "eDISABLE_SIMULATION rigid dynamics. getValid() is now false; recreate the "
        "simulation view / tensor binding for the enabled set.",
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
    // The upload needs the view's CUDA context current; callers acquire
    // PhysxCudaContextGuard later for their own DirectGPU calls, but this
    // helper runs before that point in getTransforms/getVelocities/etc.
    // Match the convention used by other CUDA entry points in this file.
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    bool uploaded = true;
    if (mNumRds > 0)
    {
        uploaded = mGpuSimData->rewriteTable(
            mRdGpuIndicesDev, mRdGpuIndicesHost.data(), mNumRds * sizeof(PxRigidDynamicGPUIndex));
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
        uploaded = mGpuSimData->rewriteTable(
                       mRbRecordsDev + first, rbRecords.data() + first, rangeCount * sizeof(GpuRigidBodyRecord)) &&
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
                                           const OvStageDeviceWriteArgs& deviceArgs,
                                           const size_t payloadBytes,
                                           const PxU32*& outRowsDev,
                                           void*& outPacked,
                                           PxRigidDynamicGPUIndex*& outIdx)
{
    if (numOutputs == 0)
        return false;

    PxRigidDynamicGPUIndex* idxDev = nullptr;
    if (deviceArgs.sessionOwned())
    {
        // A session's own row list (uploaded at open), index list and payload, so a captured replay reads
        // nothing another operation can resize or overwrite.
        OvStageScratch scratch(deviceArgs.scratch);
        outRowsDev = scratch.take<PxU32>(numOutputs);
        idxDev = scratch.take<PxRigidDynamicGPUIndex>(numOutputs);
        outPacked = static_cast<char*>(deviceArgs.scratch) + scratch.used();
        if (!deviceArgs.scratchHolds(static_cast<char*>(outPacked) + payloadBytes, attribName))
            return false;
    }
    else
    {
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
            // A wrench write reads the same index list and signals eRdForces instead, so wait for both.
            //
            // A drain reaches this with a live write in flight whenever a later group needs MORE rows
            // than an earlier one.
            const CUevent wrenchDone = mGpuSimData ? mGpuSimData->mApplySignalEvents[ApplyEvent::eRdForces] : nullptr;
            if (!waitOvStageWriteDone() ||
                (wrenchDone &&
                 !CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(wrenchDone), nullptr))))
                return false;
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
        idxDev = mOvStageIdxDev;
        outPacked = mOvStagePackedDev;
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
    // On the write's stream: every PhysX call that reads the list is forked from that stream after this
    // kernel (physxOnStream), so PhysX never gathers through an unbuilt list.
    if (!submitRbPackedIndicesOvStage(idxDev, mRdGpuIndicesDev, mRbRecordsDev, outRowsDev, numOutputs,
                                      deviceArgs.stream))
    {
        CARB_LOG_ERROR("%s: packed index build failed", attribName);
        return false;
    }
    outIdx = idxDev;
    return true;
}

OvStageDeviceWriteArgs GpuRigidBodyView::drainDeviceWriteArgs(const int applyEvent, const CUstream stream) const
{
    OvStageDeviceWriteArgs deviceArgs;
    // Run on the stream the read handed back (the producer's), so the drain's kernels and the PhysX call it forks
    // share one stream -- ordered by the stream, no extra event. null (the current synchronous read) is the
    // legacy default stream, the drain's behaviour before deferred sync.
    deviceArgs.stream = stream;
    deviceArgs.fork = mGpuSimData->mApplyWaitEvents[applyEvent];
    deviceArgs.join = mGpuSimData->mApplySignalEvents[applyEvent];
    return deviceArgs;
}

bool GpuRigidBodyView::rdIndicesRefreshPending() const
{
    return mRdIndexDirty || mDeviceUploadPending ||
           (mGpuSimData && mGpuSimData->mRdDisableEpoch != mLastSeenRdDisableEpoch);
}

bool GpuRigidBodyView::refreshRdIndicesForWrite(const char* const attribName,
                                                const OvStageDeviceWriteArgs& deviceArgs) const
{
    if (deviceArgs.capturing && rdIndicesRefreshPending())
    {
        CARB_LOG_ERROR("%s: a rigid body was enabled or disabled since this write was opened, and a captured "
                       "commit cannot upload the refreshed GPU index map. Reopen the write and capture again.",
                       attribName);
        return false;
    }
    return refreshRdGpuIndices();
}

size_t GpuRigidBodyView::ovStageWriteScratchBytes(const OvStageRigidWrite kind, const PxU32 numOutputs) const
{
    // The order every session setter carves its scratch in: the row list first, then the kind's arrays.
    OvStageScratch layout(nullptr);
    layout.take<PxU32>(numOutputs);
    const PxU32 linkBlock = mNumArtis * (mGpuSimData ? mGpuSimData->mMaxLinks : 0u);
    switch (kind)
    {
    case OvStageRigidWrite::ePose:
    case OvStageRigidWrite::eInstancerPose:
        layout.take<PxRigidDynamicGPUIndex>(numOutputs);
        layout.take<PxTransform>(numOutputs);
        break;
    case OvStageRigidWrite::eVec3:
    case OvStageRigidWrite::eInstancerVelocity:
        layout.take<PxRigidDynamicGPUIndex>(numOutputs);
        layout.take<PxVec3>(numOutputs);
        break;
    case OvStageRigidWrite::eWrench:
        layout.take<PxRigidDynamicGPUIndex>(numOutputs);
        layout.take<PxTransform>(numOutputs);
        layout.take<PxVec3>(numOutputs);
        layout.take<PxVec3>(numOutputs);
        break;
    case OvStageRigidWrite::eLinkForce:
    case OvStageRigidWrite::eLinkWrench:
        layout.take<PxU32>(numOutputs); // row -> applied-articulation slot
        layout.take<PxArticulationGPUIndex>(mNumArtis);
        layout.take<PxVec3>(linkBlock);
        if (kind == OvStageRigidWrite::eLinkWrench)
        {
            layout.take<PxVec3>(linkBlock);
            layout.take<PxTransform>(linkBlock);
        }
        break;
    }
    return layout.used();
}

bool GpuRigidBodyView::prepareOvStageWrite(const OvStageRigidWrite kind,
                                           const PxU32* const rows,
                                           const PxU32 numOutputs,
                                           const bool wholeSet,
                                           OvStageDeviceWriteArgs& deviceArgs)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    const bool wrench = kind == OvStageRigidWrite::eWrench || kind == OvStageRigidWrite::eLinkWrench;
    // The COM table a wrench reads is refreshed on the host, which a captured commit cannot do.
    if (wrench && !getComsCacheStateValid() && !updateCMassData())
        return false;
    if (kind != OvStageRigidWrite::eLinkForce && kind != OvStageRigidWrite::eLinkWrench)
        return true;
    if (mNumArtis == 0 || mGpuSimData->mMaxLinks == 0)
    {
        CARB_LOG_ERROR("link force: this view holds no articulation links");
        return false;
    }

    // A whole-set write (`wholeSet`) that names every link record in this view writes every articulation, so it
    // applies the whole view and skips the dedup. `wholeSet` alone is not enough: an articulation not yet in a
    // scene when the write was opened is left out of it and joins this view at the next step. Otherwise the
    // written articulations are resolved here from the query's records and, when they are a subset, only they
    // are applied, so every other articulation's input from earlier this step survives.
    deviceArgs.applyWholeView = wholeSet && numOutputs == mNumLinkRecords;
    deviceArgs.applyCount = mNumArtis;
    if (deviceArgs.applyWholeView)
        return true;

    std::vector<PxU32> artiToCompact(mNumArtis, 0xffffffffu); // tensorArtiIdx -> compact slot
    std::vector<PxArticulationGPUIndex> writtenArtis; // compact slot -> GPU index
    std::vector<PxU32> rowCompactArti(numOutputs); // written row -> compact slot
    for (PxU32 i = 0; i < numOutputs; ++i)
    {
        const GpuRigidBodyRecord& r = rbRecords[rows[i]];
        PxU32 slot = (r.tensorArtiIdx < mNumArtis) ? artiToCompact[r.tensorArtiIdx] : 0xffffffffu;
        if (slot == 0xffffffffu)
        {
            slot = static_cast<PxU32>(writtenArtis.size());
            if (r.tensorArtiIdx < mNumArtis)
                artiToCompact[r.tensorArtiIdx] = slot;
            writtenArtis.push_back(r.physxArtiIdx);
        }
        rowCompactArti[i] = slot;
    }
    deviceArgs.applyCount = static_cast<PxU32>(writtenArtis.size());
    if (deviceArgs.applyCount == mNumArtis)
    {
        // Every articulation is written after all: the whole-view defaults stand and nothing is uploaded.
        deviceArgs.applyWholeView = true;
        return true;
    }
    if (deviceArgs.applyCount == 0)
        return true;

    OvStageScratch scratch(deviceArgs.scratch);
    scratch.take<PxU32>(numOutputs); // the row list, uploaded by the caller
    PxU32* const rowCompactDev = scratch.take<PxU32>(numOutputs);
    PxArticulationGPUIndex* const artisDev = scratch.take<PxArticulationGPUIndex>(mNumArtis);
    if (!deviceArgs.scratchHolds(artisDev + mNumArtis, "link force"))
        return false;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    return uploadToDevice(rowCompactDev, rowCompactArti.data(), size_t(numOutputs) * sizeof(PxU32)) &&
           uploadToDevice(artisDev, writtenArtis.data(), writtenArtis.size() * sizeof(PxArticulationGPUIndex));
}

const PxU32* GpuRigidBodyView::ovStageRowsDevice(const PxU32* rows, uint32_t count, uint64_t token) const
{
    if (!rows || count == 0)
        return nullptr;

    CudaContextGuard ctxGuard(mGpuSimData ? mGpuSimData->mCtx : nullptr);

    // Hit: a slot already holds this exact list. The token must identify the row list ITSELF -- the
    // caller mints a version whenever it builds a new list, or hashes the list's contents into the
    // other half of the token space (OvStageRowsVersion.h). A cheaper token will not do: one built from
    // view generation, object type and scope leaves all three unchanged across two different queries of
    // the same type and scope. Count is compared too -- it bounds the memcpy and the slot's count,
    // whatever the caller derives its token from. Touching the slot refreshes its recency so a list
    // written every frame is never the eviction victim.
    for (OvStageRowsSlot& hit : mOvStageRows)
    {
        if (hit.token != 0 && hit.token == token && hit.count == count)
        {
            hit.tick = ++mOvStageRowsClock;
            return hit.dev;
        }
    }

    // Miss: evict the least-recently-used slot (an empty slot, tick 0, wins first) so every
    // concurrently-live write list keeps its own device copy; once they exceed kOvStageRowsSlots (a
    // many-instancer write burst) a list still in use can be the victim.
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
    if (!uploadToDevice(s.dev, rows, count * sizeof(PxU32)))
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

namespace
{
// The PhysX copy a rigid read column slices. Position and orientation share the pose.
enum RigidReadSource : uint32_t
{
    kRigidReadPose,
    kRigidReadLinearVelocity,
    kRigidReadAngularVelocity,
    kRigidReadLinearAcceleration,
    kRigidReadAngularAcceleration,
    kRigidReadSourceCount
};

RigidReadSource rigidReadSource(const GpuRigidBodyView::OvStageRigidRead kind)
{
    using Read = GpuRigidBodyView::OvStageRigidRead;
    switch (kind)
    {
    case Read::ePosition:
    case Read::eOrientation:
        return kRigidReadPose;
    case Read::eLinearVelocity:
        return kRigidReadLinearVelocity;
    case Read::eAngularVelocity:
        return kRigidReadAngularVelocity;
    case Read::eLinearAcceleration:
        return kRigidReadLinearAcceleration;
    case Read::eAngularAcceleration:
        return kRigidReadAngularAcceleration;
    case Read::eWorldScale: // stored, never copied: see rigidReadIsStored
    case Read::eNone:
        break;
    }
    return kRigidReadSourceCount;
}

// A column gathered from a copy this view holds rather than from a PhysX copy the read makes.
bool rigidReadIsStored(const GpuRigidBodyView::OvStageRigidRead kind)
{
    return kind == GpuRigidBodyView::OvStageRigidRead::eWorldScale;
}

constexpr PxRigidDynamicGPUAPIReadType::Enum kRigidDynamicReadType[kRigidReadSourceCount] = {
    PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE, PxRigidDynamicGPUAPIReadType::eLINEAR_VELOCITY,
    PxRigidDynamicGPUAPIReadType::eANGULAR_VELOCITY, PxRigidDynamicGPUAPIReadType::eLINEAR_ACCELERATION,
    PxRigidDynamicGPUAPIReadType::eANGULAR_ACCELERATION
};
constexpr PxArticulationGPUAPIReadType::Enum kLinkReadType[kRigidReadSourceCount] = {
    PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, PxArticulationGPUAPIReadType::eLINK_LINEAR_VELOCITY,
    PxArticulationGPUAPIReadType::eLINK_ANGULAR_VELOCITY, PxArticulationGPUAPIReadType::eLINK_LINEAR_ACCELERATION,
    PxArticulationGPUAPIReadType::eLINK_ANGULAR_ACCELERATION
};
} // namespace

bool GpuRigidBodyView::planOvStageRead(const PxU32* const rows,
                                       const PxU32 numOutputs,
                                       const uint64_t rowsToken,
                                       const OvStageRigidReadColumn* const columns,
                                       const PxU32 columnCount,
                                       OvStageRigidReadPlan& plan,
                                       std::shared_ptr<const std::vector<uint8_t>>& head,
                                       size_t& scratchBytes) const
{
    plan = OvStageRigidReadPlan();
    head.reset();
    scratchBytes = 0;
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    if (!rows || numOutputs == 0 || !columns || columnCount == 0)
        return false;
    // The columns shape the plan only through the copies they need; a stored column needs none.
    uint32_t sources = 0;
    for (PxU32 c = 0; c < columnCount; ++c)
    {
        if (rigidReadIsStored(columns[c].kind))
            continue;
        const RigidReadSource source = rigidReadSource(columns[c].kind);
        if (source == kRigidReadSourceCount)
            return false;
        sources |= 1u << source;
    }
    // The view's rigid-dynamic count decides the fetch, and moves when a body outside the rows is enabled or disabled.
    const std::vector<uint64_t> signature{ sources, numOutputs, mNumRds };
    if (rowsToken)
    {
        if (const OvStageReadPlans<OvStageRigidReadPlan>::Entry* const kept =
                mOvStageReadPlans.find(rowsToken, 0, signature))
        {
            plan = kept->plan;
            head = kept->head;
            scratchBytes = kept->scratchBytes;
            return true;
        }
    }
    plan.sources = sources;
    // Once per plan, on the host: the gathers index records[rows[i]] with no bound of their own.
    if (!checkRecordIndices(rows, numOutputs, mEntries.size(), "ovstage record list", __FUNCTION__))
        return false;

    plan.numOutputs = numOutputs;
    plan.links = ovStageRowsAreLinks(rows, numOutputs);
    std::vector<PxArticulationGPUIndex> artis;
    std::vector<PxU32> artiSlot;
    if (plan.links)
    {
        if (mNumArtis == 0 || mGpuSimData->mMaxLinks == 0)
        {
            CARB_LOG_ERROR("ovstage link read: this view holds no articulation links");
            return false;
        }
        // The articulations the rows belong to, each fetched once, in the order the rows first name them.
        std::vector<PxU32> slotOfArti(mNumArtis, 0xffffffffu);
        artiSlot.resize(numOutputs);
        for (PxU32 i = 0; i < numOutputs; ++i)
        {
            const GpuRigidBodyRecord& r = rbRecords[rows[i]];
            if (r.physxLinkIdx == 0xffffffff || r.tensorArtiIdx >= mNumArtis)
            {
                CARB_LOG_ERROR("ovstage link read: row %u is not an articulation link of this view", i);
                return false;
            }
            PxU32& slot = slotOfArti[r.tensorArtiIdx];
            if (slot == 0xffffffffu)
            {
                slot = static_cast<PxU32>(artis.size());
                artis.push_back(r.physxArtiIdx);
            }
            artiSlot[i] = slot;
        }
        // Rows touching every articulation fetch them all through the view's own list, which costs the same.
        plan.wholeView = artis.size() == mNumArtis;
        plan.numFetched = plan.wholeView ? mNumArtis : static_cast<PxU32>(artis.size());
    }
    else
    {
        // A link among rigid-dynamic rows would be gathered from a link copy the read does not make.
        for (PxU32 i = 0; i < numOutputs; ++i)
        {
            if (rbRecords[rows[i]].physxLinkIdx != 0xffffffff)
            {
                CARB_LOG_ERROR("ovstage rigid read: row %u is an articulation link among rigid dynamics", i);
                return false;
            }
        }
        // The view's index list holds every enabled rigid dynamic, so it serves any read; a read of fewer
        // fetches only its own rows. A read's rows never hold a disabled body, which has no index: the read omits
        // disabled bodies from its rows.
        plan.wholeView = numOutputs >= mNumRds;
        plan.numFetched = plan.wholeView ? mNumRds : numOutputs;
    }

    // The head readOvStageColumns finds at the start of its scratch, laid out as it carves it.
    OvStageScratch layout(nullptr);
    layout.take<PxU32>(numOutputs);
    size_t artisAt = 0;
    size_t slotsAt = 0;
    if (plan.links && !plan.wholeView)
    {
        artisAt = layout.used();
        layout.take<PxArticulationGPUIndex>(plan.numFetched);
        slotsAt = layout.used();
        layout.take<PxU32>(numOutputs);
    }
    std::vector<uint8_t> built(layout.used(), 0);
    std::memcpy(built.data(), rows, size_t(numOutputs) * sizeof(PxU32));
    if (plan.links && !plan.wholeView)
    {
        std::memcpy(built.data() + artisAt, artis.data(), artis.size() * sizeof(PxArticulationGPUIndex));
        std::memcpy(built.data() + slotsAt, artiSlot.data(), artiSlot.size() * sizeof(PxU32));
    }
    if (!plan.links && !plan.wholeView)
        layout.take<PxRigidDynamicGPUIndex>(numOutputs);
    const PxU32 perCopy = plan.links ? plan.numFetched * mGpuSimData->mMaxLinks : plan.numFetched;
    for (uint32_t source = 0; source < kRigidReadSourceCount; ++source)
    {
        if (!(plan.sources & (1u << source)))
            continue;
        if (source == kRigidReadPose)
            layout.take<PxTransform>(perCopy);
        else
            layout.take<PxVec3>(perCopy);
    }
    scratchBytes = layout.used();
    head = std::make_shared<const std::vector<uint8_t>>(std::move(built));
    if (rowsToken)
        mOvStageReadPlans.keep(rowsToken, 0, signature, plan, head, scratchBytes);
    return true;
}

bool GpuRigidBodyView::readOvStageColumns(const OvStageDeviceReadArgs& args,
                                          const OvStageRigidReadPlan& plan,
                                          const OvStageRigidReadColumn* const columns,
                                          const PxU32 columnCount) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, plan.links, false);
    if (!args.scratch || !args.fork || !args.join || plan.numOutputs == 0 || !columns)
        return false;
    // The plan addresses the GPU index map as it was when the plan was made, in this same call.
    if (rdIndicesRefreshPending())
    {
        CARB_LOG_ERROR("ovstage rigid read: a rigid body was enabled or disabled after the read was planned");
        return false;
    }

    PxScene* const scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    const PxU32 n = plan.numOutputs;
    OvStageScratch scratch(args.scratch);
    const PxU32* const rowsDev = scratch.take<PxU32>(n);
    const PxArticulationGPUIndex* artisDev = nullptr;
    const PxU32* artiSlotDev = nullptr;
    PxRigidDynamicGPUIndex* idxDev = nullptr;
    if (plan.links && !plan.wholeView)
    {
        artisDev = scratch.take<PxArticulationGPUIndex>(plan.numFetched);
        artiSlotDev = scratch.take<PxU32>(n);
    }
    if (!plan.links && !plan.wholeView)
        idxDev = scratch.take<PxRigidDynamicGPUIndex>(n);
    const PxU32 perCopy = plan.links ? plan.numFetched * mGpuSimData->mMaxLinks : plan.numFetched;
    void* staging[kRigidReadSourceCount] = {};
    for (uint32_t source = 0; source < kRigidReadSourceCount; ++source)
    {
        if (!(plan.sources & (1u << source)))
            continue;
        staging[source] = source == kRigidReadPose ? static_cast<void*>(scratch.take<PxTransform>(perCopy)) :
                                                     static_cast<void*>(scratch.take<PxVec3>(perCopy));
    }
    if (!args.scratchHolds(static_cast<char*>(args.scratch) + scratch.used(), "ovstage rigid read"))
        return false;

    // Everything below reads this view's tables on `args.stream`; the host rewrites them only once it is done.
    const auto noteReader = [&]() { return args.capturing || mGpuSimData->noteTableReader(args.stream); };

    if (idxDev && !submitRbPackedIndicesOvStage(idxDev, mRdGpuIndicesDev, mRbRecordsDev, rowsDev, n, args.stream))
    {
        noteReader();
        CARB_LOG_ERROR("ovstage rigid read: packed index build failed");
        return false;
    }
    // Status IS checked: PhysX refuses these copies in states the caller cannot see and writes nothing when it does,
    // and a gather over the scratch would then publish whatever it last held.
    bool fetched = true;
    for (uint32_t source = 0; source < kRigidReadSourceCount && fetched; ++source)
    {
        if (!staging[source])
            continue;
        fetched = physxOnStream(args, [&](CUevent start, CUevent finish) {
            if (plan.links)
                return scene->getDirectGPUAPI().getArticulationData(
                    staging[source], plan.wholeView ? mArtiIndicesDev : artisDev, kLinkReadType[source],
                    plan.numFetched, start, finish);
            return scene->getDirectGPUAPI().getRigidDynamicData(
                staging[source], plan.wholeView ? mRdGpuIndicesDev : idxDev, kRigidDynamicReadType[source],
                plan.numFetched, start, finish);
        });
        if (!fetched)
            CARB_LOG_ERROR("ovstage rigid read: PxDirectGPUAPI refused the %s copy",
                           plan.links ? "articulation link" : "rigid dynamic");
    }
    bool gathered = fetched;
    const bool packedRd = !plan.links && !plan.wholeView;
    for (PxU32 c = 0; c < columnCount && gathered; ++c)
    {
        if (rigidReadIsStored(columns[c].kind))
        {
            gathered = columns[c].dst && mWorldScalesDev &&
                       fetchRbWorldScaleColumnOvStage(columns[c].dst, mWorldScalesDev, rowsDev, n, args.stream);
            continue;
        }
        const RigidReadSource source = rigidReadSource(columns[c].kind);
        if (source == kRigidReadSourceCount || !staging[source] || !columns[c].dst)
        {
            gathered = false;
            break;
        }
        const void* const rdSource = plan.links ? nullptr : staging[source];
        const void* const linkSource = plan.links ? staging[source] : nullptr;
        if (source == kRigidReadPose)
            gathered = fetchRbPoseColumnOvStage(columns[c].dst, static_cast<const PxTransform*>(rdSource),
                                                static_cast<const PxTransform*>(linkSource), mRbRecordsDev, rowsDev, n,
                                                mGpuSimData->mMaxLinks,
                                                columns[c].kind == OvStageRigidRead::eOrientation, args.stream,
                                                packedRd, artiSlotDev, plan.numFetched);
        else
            gathered = fetchRbVelocityColumnOvStage(columns[c].dst, static_cast<const PxVec3*>(rdSource),
                                                    static_cast<const PxVec3*>(linkSource), mRbRecordsDev, rowsDev, n,
                                                    mGpuSimData->mMaxLinks, args.stream, packedRd, artiSlotDev,
                                                    plan.numFetched);
    }
    return noteReader() && gathered;
}

// The PhysX copies an instancer read's columns need, each holding every rigid dynamic of the view.
static void instancerReadScratchLayout(OvStageScratch& layout,
                                       const GpuRigidBodyView::OvStageInstancerReadColumn* columns,
                                       const PxU32 columnCount,
                                       const PxU32 numRds,
                                       void* staging[kRigidReadSourceCount])
{
    uint32_t sources = 0;
    for (PxU32 c = 0; c < columnCount; ++c)
        sources |= 1u << rigidReadSource(columns[c].kind);
    for (uint32_t source = 0; source < kRigidReadSourceCount; ++source)
    {
        staging[source] = nullptr;
        if (!(sources & (1u << source)))
            continue;
        staging[source] = source == kRigidReadPose ? static_cast<void*>(layout.take<PxTransform>(numRds)) :
                                                     static_cast<void*>(layout.take<PxVec3>(numRds));
    }
}

size_t GpuRigidBodyView::ovStageInstancerReadScratchBytes(const OvStageInstancerReadColumn* columns,
                                                          const PxU32 columnCount) const
{
    OvStageScratch layout(nullptr);
    void* staging[kRigidReadSourceCount];
    instancerReadScratchLayout(layout, columns, columnCount, mNumRds, staging);
    return layout.used();
}

bool GpuRigidBodyView::readOvStageInstancerColumns(const OvStageDeviceReadArgs& args,
                                                   const OvStageInstancerReadColumn* columns,
                                                   const PxU32 columnCount,
                                                   const GpuPointInstancerRecord* recordsDev,
                                                   const InstancerAffine* inversesDev,
                                                   const PxU32 numInstances) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (!args.scratch || !args.fork || !args.join || !columns || !recordsDev || !inversesDev)
        return false;
    if (numInstances == 0 || mNumRds == 0)
        return true;
    // The copies address the GPU index map as it was when the read was planned, in this same call.
    if (rdIndicesRefreshPending())
    {
        CARB_LOG_ERROR("ovstage instancer read: a rigid body was enabled or disabled after the read was planned");
        return false;
    }

    PxScene* const scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    OvStageScratch scratch(args.scratch);
    void* staging[kRigidReadSourceCount];
    instancerReadScratchLayout(scratch, columns, columnCount, mNumRds, staging);
    if (!args.scratchHolds(static_cast<char*>(args.scratch) + scratch.used(), "ovstage instancer read"))
        return false;

    // Status IS checked: a refused copy writes nothing, and a gather would publish whatever the scratch held.
    bool fetched = true;
    for (uint32_t source = 0; source < kRigidReadSourceCount && fetched; ++source)
    {
        if (!staging[source])
            continue;
        fetched = physxOnStream(args, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().getRigidDynamicData(staging[source], mRdGpuIndicesDev,
                                                                kRigidDynamicReadType[source], mNumRds, start,
                                                                finish);
        });
        if (!fetched)
            CARB_LOG_ERROR("ovstage instancer read: PxDirectGPUAPI refused the rigid dynamic copy");
    }
    bool gathered = fetched;
    for (PxU32 c = 0; c < columnCount && gathered; ++c)
    {
        const RigidReadSource source = rigidReadSource(columns[c].kind);
        if (source == kRigidReadSourceCount || !columns[c].dst || !columns[c].offsetsDev)
        {
            gathered = false;
            break;
        }
        if (source == kRigidReadPose)
            gathered = fetchInstancerPoseColumnOvStage(
                columns[c].dst, static_cast<const PxTransform*>(staging[source]), mRbRecordsDev, recordsDev,
                inversesDev, columns[c].offsetsDev, numInstances, columns[c].kind == OvStageRigidRead::eOrientation,
                args.stream);
        else
            gathered = fetchInstancerVelocityColumnOvStage(columns[c].dst, static_cast<const PxVec3*>(staging[source]),
                                                           mRbRecordsDev, recordsDev, columns[c].offsetsDev,
                                                           numInstances, args.stream);
    }
    // The gathers read this view's tables and the instancer's frames on `args.stream`.
    const bool noted = args.capturing || mGpuSimData->noteTableReader(args.stream);
    return noted && gathered;
}

bool GpuRigidBodyView::setInstancerPoseColumnOvStage(const bool wantOrientation,
                                                     const float* const srcDev,
                                                     const GpuPointInstancerRecord* const recordsDev,
                                                     const InstancerAffine* const instancerInversesDev,
                                                     const InstancerAffine* const instancerForwardsDev,
                                                     const PxU32* const rbRows,
                                                     const PxU32 numOutputs,
                                                     const PxU32 instancerIdx,
                                                     const uint64_t rowsToken,
                                                     const OvStageDeviceWriteArgs& sessionDeviceArgs)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (numOutputs == 0)
        return true;
    if (!srcDev || !recordsDev || !instancerInversesDev || !instancerForwardsDev || !rbRows)
        return false;

    const char* const attribName =
        wantOrientation ? "instancer orientation (ovstage write)" : "instancer position (ovstage write)";
    const bool session = sessionDeviceArgs.sessionOwned();
    const OvStageDeviceWriteArgs deviceArgs = session ? sessionDeviceArgs : drainDeviceWriteArgs(ApplyEvent::eRdData);
    if (!refreshRdIndicesForWrite(attribName, deviceArgs))
        return false;
    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    const PxU32* rowsDev = nullptr;
    void* packed = nullptr;
    PxRigidDynamicGPUIndex* packedIdx = nullptr;
    if (!ovStageWriteBuffers(attribName, rbRows, numOutputs, rowsToken, deviceArgs,
                             size_t(numOutputs) * sizeof(PxTransform), rowsDev, packed, packedIdx))
        return false;
    if (!packed)
        return false;

    // The bodies' CURRENT poses, for the read-modify-write. A session reads exactly its instances through
    // its own index list into its own block, which the scatter then overwrites in place. The drain reads
    // every body into the shared rd-pose buffer the gather uses, so it takes that buffer's ordering event
    // (ADR-0008 Decision 7).
    const PxTransform* current = nullptr;
    if (session)
    {
        current = static_cast<const PxTransform*>(packed);
        if (!physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
                return scene->getDirectGPUAPI().getRigidDynamicData(packed, packedIdx,
                                                                    PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE,
                                                                    numOutputs, start, finish);
            }))
        {
            CARB_LOG_ERROR("%s: PxDirectGPUAPI::getRigidDynamicData(eGLOBAL_POSE) refused the read that "
                           "preserves the pose half this column does not carry",
                           attribName);
            return false;
        }
    }
    else if (mNumRds > 0)
    {
        // awaitDirectGpuFetch, not a bare call: the finish event here is the non-null shared rd-pose
        // event, so PhysX may have queued work and recorded it even on refusal. The ordering edge --
        // and the producer drain plus buffer quarantine when that edge cannot be established -- has to
        // run either way.
        CUevent rdEvent = mGpuSimData->mCopyEvents[CopyEvent::eRdData];
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
        instancerInversesDev, instancerForwardsDev, numOutputs, instancerIdx, wantOrientation, deviceArgs.stream,
        deviceArgs.mask, current);
    if (!session)
        mGpuSimData->recordKernelDone(SharedDeviceBuffer::eRdPose);
    if (!scattered)
    {
        CARB_LOG_ERROR("%s: ovstage instancer scatter failed", attribName);
        return false;
    }

    if (!physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().setRigidDynamicData(packed, packedIdx,
                                                                PxRigidDynamicGPUAPIWriteType::eGLOBAL_POSE,
                                                                numOutputs, start, finish);
        }))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::setRigidDynamicData(eGLOBAL_POSE) refused the write", attribName);
        return false;
    }
    return true;
}

bool GpuRigidBodyView::setInstancerVelocityColumnOvStage(const bool wantAngular,
                                                         const float* const srcDev,
                                                         const GpuPointInstancerRecord* const recordsDev,
                                                         const PxU32* const rbRows,
                                                         const PxU32 numOutputs,
                                                         const uint64_t rowsToken,
                                                         const OvStageDeviceWriteArgs& sessionDeviceArgs)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (numOutputs == 0)
        return true;
    if (!srcDev || !recordsDev || !rbRows)
        return false;

    const char* const attribName = wantAngular ? "instancer angularVelocity (ovstage write)" :
                                                 "instancer linearVelocity (ovstage write)";
    const OvStageDeviceWriteArgs deviceArgs =
        sessionDeviceArgs.sessionOwned() ? sessionDeviceArgs : drainDeviceWriteArgs(ApplyEvent::eRdData);
    if (!refreshRdIndicesForWrite(attribName, deviceArgs))
        return false;
    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    const PxU32* rowsDev = nullptr;
    void* packed = nullptr;
    PxRigidDynamicGPUIndex* packedIdx = nullptr;
    if (!ovStageWriteBuffers(attribName, rbRows, numOutputs, rowsToken, deviceArgs,
                             size_t(numOutputs) * sizeof(PxVec3), rowsDev, packed, packedIdx))
        return false;
    if (!packed)
        return false;

    // No pre-read: linear and angular velocity are separate PhysX write types, so neither has to
    // preserve the other -- the same reason the standalone velocity path needs no RMW. A masked commit
    // does read the current velocities back, for the instances it does not select.
    //
    // The drain's packed block is PxTransform-sized, which is larger than the PxVec3[N] this needs;
    // reusing it rather than allocating a second buffer is what the standalone paths do too.
    if (deviceArgs.mask &&
        !physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().getRigidDynamicData(
                packed, packedIdx,
                wantAngular ? PxRigidDynamicGPUAPIReadType::eANGULAR_VELOCITY :
                              PxRigidDynamicGPUAPIReadType::eLINEAR_VELOCITY,
                numOutputs, start, finish);
        }))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::getRigidDynamicData refused the read that keeps the instances this "
                       "masked commit does not select",
                       attribName);
        return false;
    }
    if (!submitInstancerVelocityColumnOvStage(static_cast<PxVec3*>(packed), srcDev, recordsDev, numOutputs,
                                              deviceArgs.stream, deviceArgs.mask))
    {
        CARB_LOG_ERROR("%s: ovstage instancer velocity scatter failed", attribName);
        return false;
    }

    if (!physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().setRigidDynamicData(
                packed, packedIdx,
                wantAngular ? PxRigidDynamicGPUAPIWriteType::eANGULAR_VELOCITY :
                              PxRigidDynamicGPUAPIWriteType::eLINEAR_VELOCITY,
                numOutputs, start, finish);
        }))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::setRigidDynamicData refused the write", attribName);
        return false;
    }
    return true;
}

// ADR-0012: the ovstage column WRITE, carrying NO host block, as the read carries none. The count
// PhysX needs is numOutputs, known before anything launches, so nothing has to be read back from the
// device; that is what a flags-and-compaction scatter cannot do, since its count comes off a thrust
// iterator on the host.
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
                                            const uint64_t rowsToken,
                                            const OvStageDeviceWriteArgs& sessionDeviceArgs)
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
    const OvStageDeviceWriteArgs deviceArgs =
        sessionDeviceArgs.sessionOwned() ? sessionDeviceArgs : drainDeviceWriteArgs(ApplyEvent::eRdData);
    if (!refreshRdIndicesForWrite(attribName, deviceArgs))
        return false;
    if (numOutputs == 0)
        return true;

    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    const PxU32* outRecordIdxDev = nullptr;
    void* payload = nullptr;
    PxRigidDynamicGPUIndex* packedIdx = nullptr;
    if (!ovStageWriteBuffers(attribName, outRecordIdx, numOutputs, rowsToken, deviceArgs,
                             size_t(numOutputs) * sizeof(PxTransform), outRecordIdxDev, payload, packedIdx))
        return false;
    PxTransform* const packedPose = static_cast<PxTransform*>(payload);

    // Current poses for exactly the covered rows. Status IS checked: PhysX refuses these calls in
    // states the caller cannot see (pre-step being the documented one) and writes NOTHING when it
    // does; the scratch is not zeroed, so an unchecked refusal would overlay the caller's component
    // onto stale contents and publish that as poses.
    if (!physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().getRigidDynamicData((void*)packedPose, packedIdx,
                                                                PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE, numOutputs,
                                                                start, finish);
        }))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::getRigidDynamicData(eGLOBAL_POSE) refused the read that preserves "
                       "the component this write does not set",
                       attribName);
        return false;
    }

    // An unselected row of a masked commit keeps the pose just read.
    if (!submitRbPackedPoseOvStage(packedPose, static_cast<const float*>(srcTensor->data), mRbRecordsDev,
                                    outRecordIdxDev, numOutputs, wantOrientation, deviceArgs.stream, deviceArgs.mask))
    {
        CARB_LOG_ERROR("%s: scatter failed", attribName);
        return false;
    }

    // Ordered behind the scatter on the device, with no host round trip. The buffers are private to
    // this write, so there is no shared scene buffer for a sibling read to race on.
    if (!physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().setRigidDynamicData((void*)packedPose, packedIdx,
                                                                PxRigidDynamicGPUAPIWriteType::eGLOBAL_POSE,
                                                                numOutputs, start, finish);
        }))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::setRigidDynamicData(eGLOBAL_POSE) refused the write", attribName);
        return false;
    }
    return true;
}

// As above for a velocity component, and simpler in the way that matters: eLINEAR_VELOCITY and
// eANGULAR_VELOCITY are separate write types, so there is nothing to preserve and no pre-read.
//
// The caller's column is ALREADY the packed payload -- a dense [numOutputs, 3] float column is
// exactly PxVec3[numOutputs] -- so it is handed to PhysX directly. A masked commit (ADR-0052) merges
// instead: the selected rows from the column, the rest read back first (a velocity) or zero (a force or
// torque, which PhysX cannot read back), in the session's own buffer.
bool GpuRigidBodyView::setVelocityColumnOvStage(const char* const attribName,
                                                const TensorDesc* const srcTensor,
                                                const PxRigidDynamicGPUAPIWriteType::Enum rdType,
                                                const PxU32* outRecordIdx,
                                                const PxU32 numOutputs,
                                                const uint64_t rowsToken,
                                                const OvStageDeviceWriteArgs& sessionDeviceArgs)
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
    // The drain carries the producer's stream in sessionDeviceArgs.stream (scratch null, so not sessionOwned);
    // run on that stream so the scale and this write share it. Session commits bring their own full args.
    const OvStageDeviceWriteArgs deviceArgs =
        sessionDeviceArgs.sessionOwned() ? sessionDeviceArgs
                                         : drainDeviceWriteArgs(ApplyEvent::eRdData, sessionDeviceArgs.stream);
    if (!refreshRdIndicesForWrite(attribName, deviceArgs))
        return false;
    if (numOutputs == 0)
        return true;

    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    const PxU32* outRecordIdxDev = nullptr;
    void* payload = nullptr;
    PxRigidDynamicGPUIndex* packedIdx = nullptr;
    if (!ovStageWriteBuffers(attribName, outRecordIdx, numOutputs, rowsToken, deviceArgs,
                             size_t(numOutputs) * sizeof(PxVec3), outRecordIdxDev, payload, packedIdx))
        return false;

    const void* data = srcTensor->data;
    if (deviceArgs.mask)
    {
        PxVec3* const merged = static_cast<PxVec3*>(payload);
        const bool readable = rdType == PxRigidDynamicGPUAPIWriteType::eLINEAR_VELOCITY ||
                              rdType == PxRigidDynamicGPUAPIWriteType::eANGULAR_VELOCITY;
        if (readable &&
            !physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
                return scene->getDirectGPUAPI().getRigidDynamicData(
                    (void*)merged, packedIdx,
                    rdType == PxRigidDynamicGPUAPIWriteType::eLINEAR_VELOCITY ?
                        PxRigidDynamicGPUAPIReadType::eLINEAR_VELOCITY :
                        PxRigidDynamicGPUAPIReadType::eANGULAR_VELOCITY,
                    numOutputs, start, finish);
            }))
        {
            CARB_LOG_ERROR("%s: PxDirectGPUAPI::getRigidDynamicData refused the read that keeps the rows this "
                           "masked commit does not select",
                           attribName);
            return false;
        }
        if (!overlayVec3ColumnOvStage(merged, static_cast<const float*>(srcTensor->data), deviceArgs.mask, numOutputs,
                                      !readable, deviceArgs.stream))
        {
            CARB_LOG_ERROR("%s: masked merge failed", attribName);
            return false;
        }
        data = merged;
    }

    // Only our kernels ran on the write's stream, but the ordering obligation is identical: PhysX must
    // not read the index list or the column before they finish.
    if (!physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().setRigidDynamicData(data, packedIdx, rdType, numOutputs, start, finish);
        }))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::setRigidDynamicData refused the write", attribName);
        return false;
    }
    return true;
}

bool GpuRigidBodyView::waitOvStageWriteDone() const
{
    if (!mGpuSimData)
        return true;
    CUevent done = mGpuSimData->mApplySignalEvents[ApplyEvent::eRdData];
    if (!done)
        return true;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    // Waits for whatever last recorded this event, which is at least as late as our last write. That
    // is over-waiting when a sibling raced in, never under-waiting, which is the safe direction for
    // a guard standing between an in-flight read and a free.
    return CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(done), nullptr));
}

bool GpuRigidBodyView::setPositionsOvStage(const TensorDesc* srcTensor,
                                           const PxU32* outRecordIdx,
                                           const PxU32 numOutputs,
                                           const uint64_t rowsToken, bool,
                                           const OvStageDeviceWriteArgs& deviceArgs)
{
    return setPoseColumnOvStage("position", srcTensor, /*wantOrientation*/ false, outRecordIdx, numOutputs, rowsToken,
                                deviceArgs);
}

bool GpuRigidBodyView::setOrientationsOvStage(const TensorDesc* srcTensor,
                                              const PxU32* outRecordIdx,
                                              const PxU32 numOutputs,
                                              const uint64_t rowsToken, bool,
                                              const OvStageDeviceWriteArgs& deviceArgs)
{
    return setPoseColumnOvStage("orientation", srcTensor, /*wantOrientation*/ true, outRecordIdx, numOutputs,
                                rowsToken, deviceArgs);
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
                                           const PxU32 comps, bool wholeSet,
                                           const OvStageDeviceWriteArgs& sessionDeviceArgs)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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

    const bool session = sessionDeviceArgs.sessionOwned();
    const OvStageDeviceWriteArgs deviceArgs =
        session ? sessionDeviceArgs : drainDeviceWriteArgs(ApplyEvent::eArtiLinkForces);
    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    const PxU32 linkBlock = mNumArtis * mGpuSimData->mMaxLinks;

    // Rows only. ovStageWriteBuffers is deliberately NOT used here: its packed index build reads
    // tensorRdIdx, which a link does not have, and its validation rejects exactly this case.
    const PxU32* rowsDev = nullptr;
    PxArticulationGPUIndex* applyIndices = mArtiIndicesDev;
    PxU32 applyCount = mNumArtis;
    const PxU32* rowCompactArg = nullptr;
    PxVec3* linkForces = mGpuSimData->mLinkForcesDev;
    PxVec3* linkTorques = mGpuSimData->mLinkTorquesDev;
    PxTransform* linkPoses = mGpuSimData->mLinkOrRootTransformsDev;
    if (session)
    {
        // Everything resolved at open (prepareOvStageWrite) lives in the session's scratch, laid out as
        // ovStageWriteScratchBytes describes.
        OvStageScratch scratch(deviceArgs.scratch);
        rowsDev = scratch.take<PxU32>(numOutputs);
        PxU32* const rowCompactDev = scratch.take<PxU32>(numOutputs);
        PxArticulationGPUIndex* const artisDev = scratch.take<PxArticulationGPUIndex>(mNumArtis);
        linkForces = scratch.take<PxVec3>(linkBlock);
        if (comps == 9)
        {
            linkTorques = scratch.take<PxVec3>(linkBlock);
            linkPoses = scratch.take<PxTransform>(linkBlock);
        }
        const void* const end = comps == 9 ? static_cast<const void*>(linkPoses + linkBlock) :
                                             static_cast<const void*>(linkForces + linkBlock);
        if (!deviceArgs.scratchHolds(end, attribName))
            return false;
        if (!deviceArgs.applyWholeView)
        {
            applyIndices = artisDev;
            applyCount = deviceArgs.applyCount;
            rowCompactArg = rowCompactDev;
        }
        if (applyCount == 0)
            return true; // no resolvable articulation among the written rows -- nothing to apply
        if (comps == 9 && !getComsCacheStateValid())
        {
            if (deviceArgs.capturing)
            {
                CARB_LOG_ERROR("%s: a centre of mass changed since this write was opened, and a captured commit "
                               "cannot refresh it. Reopen the write and capture again.",
                               attribName);
                return false;
            }
            updateCMassData();
        }
    }
    else
    {
        rowsDev = ovStageRowsDevice(outRecordIdx, numOutputs, rowsToken);
        if (!rowsDev)
            return false;

        if (!getComsCacheStateValid())
            updateCMassData();

        // A whole-set write (`wholeSet`) that names every link record in this view writes every articulation, so
        // it applies the whole view (rowCompactArg == nullptr) and skips the host dedup. `wholeSet` alone is not
        // enough: an articulation not yet in a scene when the write was opened is left out of it and joins this
        // view at the next step. Otherwise the written articulations are resolved on the host from the query's
        // records and, when they are a subset, only they are applied, so every other articulation's input from
        // earlier this step survives. Scratch is cached on the view.
        const bool coversView = wholeSet && numOutputs == mNumLinkRecords;
        if (!coversView)
        {
            std::vector<PxU32>& artiToCompact = mLinkWrenchArtiToCompact;                 // tensorArtiIdx -> compact slot
            std::vector<PxArticulationGPUIndex>& writtenArtis = mLinkWrenchWrittenArtis;  // compact slot -> GPU index
            std::vector<PxU32>& rowCompactArti = mLinkWrenchRowCompact;                   // written row -> compact slot
            artiToCompact.assign(mNumArtis, 0xffffffffu);
            writtenArtis.clear();
            rowCompactArti.resize(numOutputs);
            for (PxU32 i = 0; i < numOutputs; ++i)
            {
                const GpuRigidBodyRecord& r = rbRecords[outRecordIdx[i]];
                PxU32 slot = (r.tensorArtiIdx < mNumArtis) ? artiToCompact[r.tensorArtiIdx] : 0xffffffffu;
                if (slot == 0xffffffffu)
                {
                    slot = static_cast<PxU32>(writtenArtis.size());
                    if (r.tensorArtiIdx < mNumArtis)
                        artiToCompact[r.tensorArtiIdx] = slot;
                    writtenArtis.push_back(r.physxArtiIdx);
                }
                rowCompactArti[i] = slot;
            }
            const PxU32 numWrittenArtis = static_cast<PxU32>(writtenArtis.size());
            if (numWrittenArtis == 0)
                return true; // no resolvable articulation among the written rows -- nothing to apply

            // A true subset: upload the per-row compact slots and the written-articulation index list
            // (host-known, so a plain upload, not the device->host readback the binding's masked path pays).
            // Grown-and-cached like mOvStageWrenchDev; freed in the destructor. A grow of the index buffer
            // waits on the link-force apply signal first, since PhysX reads it asynchronously (ADR-0008 D7). If
            // the dedup finds EVERY articulation written, the whole-view defaults above already stand and no
            // upload is needed.
            if (numWrittenArtis != mNumArtis)
            {
                if (numOutputs > mOvStageLinkWrenchRowArtiCapacity)
                {
                    if (mOvStageLinkWrenchRowArtiDev)
                        CHECK_CUDA(cudaFree(mOvStageLinkWrenchRowArtiDev));
                    mOvStageLinkWrenchRowArtiDev = nullptr;
                    mOvStageLinkWrenchRowArtiCapacity = 0;
                    if (!CHECK_CUDA(cudaMalloc((void**)&mOvStageLinkWrenchRowArtiDev,
                                               size_t(numOutputs) * sizeof(PxU32))))
                        return false;
                    mOvStageLinkWrenchRowArtiCapacity = numOutputs;
                }
                if (numWrittenArtis > mOvStageLinkWrenchArtisCapacity)
                {
                    CUevent forcesDone = mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkForces];
                    if (mOvStageLinkWrenchArtisDev && forcesDone)
                        CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(forcesDone), nullptr));
                    if (mOvStageLinkWrenchArtisDev)
                        CHECK_CUDA(cudaFree(mOvStageLinkWrenchArtisDev));
                    mOvStageLinkWrenchArtisDev = nullptr;
                    mOvStageLinkWrenchArtisCapacity = 0;
                    if (!CHECK_CUDA(cudaMalloc((void**)&mOvStageLinkWrenchArtisDev,
                                               size_t(numWrittenArtis) * sizeof(PxArticulationGPUIndex))))
                        return false;
                    mOvStageLinkWrenchArtisCapacity = numWrittenArtis;
                }
                CHECK_CUDA(cudaMemcpyAsync(mOvStageLinkWrenchRowArtiDev, rowCompactArti.data(),
                                           size_t(numOutputs) * sizeof(PxU32), cudaMemcpyHostToDevice, nullptr));
                CHECK_CUDA(cudaMemcpyAsync(mOvStageLinkWrenchArtisDev, writtenArtis.data(),
                                           size_t(numWrittenArtis) * sizeof(PxArticulationGPUIndex),
                                           cudaMemcpyHostToDevice, nullptr));
                applyIndices = mOvStageLinkWrenchArtisDev;
                applyCount = numWrittenArtis;
                rowCompactArg = mOvStageLinkWrenchRowArtiDev;
            }
        }
    }

    // Zero only the blocks the apply pushes: the whole view, or the written articulations' compact blocks.
    // A written articulation's unnamed links are zeroed with it, as are the unselected links of a masked
    // commit; an unwritten articulation is not applied at all.
    const PxU32 zeroLinks = applyCount * mGpuSimData->mMaxLinks;
    CHECK_CUDA(cudaMemsetAsync(linkForces, 0, size_t(zeroLinks) * sizeof(PxVec3), deviceArgs.stream));
    if (comps == 9)
        CHECK_CUDA(cudaMemsetAsync(linkTorques, 0, size_t(zeroLinks) * sizeof(PxVec3), deviceArgs.stream));

    // Link poses, needed only by the wrench conversion. The drain reads them into the shared
    // link-transform buffer, which is why it releases that buffer after the scatter (ADR-0008 Decision 7).
    if (comps == 9 &&
        !physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().getArticulationData((void*)linkPoses, mArtiIndicesDev,
                                                                PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE,
                                                                mNumArtis, start, finish);
        }))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::getArticulationData refused the link-pose read the "
                       "torque-about-COM conversion needs",
                       attribName);
        return false;
    }

    if (!submitLinkWrenchOvStage(linkForces, linkTorques, static_cast<const float*>(srcTensor->data), linkPoses,
                                  cMassLocalPosePosDev, mRbRecordsDev, rowsDev, rowCompactArg, numOutputs,
                                  mGpuSimData->mMaxLinks, comps, deviceArgs.stream, deviceArgs.mask))
    {
        CARB_LOG_ERROR("%s: ovstage link scatter failed", attribName);
        return false;
    }
    if (comps == 9 && !session)
        mGpuSimData->recordKernelDone(SharedDeviceBuffer::eLinkOrRootTransforms);

    // applyIndices[slot] is the PhysX GPU index of the articulation whose (compact or whole-view) block the
    // scatter filled, and applyCount is the host-known number applied. On the sparse path PhysX touches ONLY
    // the written articulations' link forces and leaves every other articulation's input from earlier this
    // step in place; on the write-all path this is the whole view. Either way, no device->host readback --
    // the count came from the host-side query, not a device compaction (the binding's fillArtiFT block).
    //
    // The write's stream waits for both applies, so the next write cannot clear these blocks mid-apply.
    if (!physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
            bool applied = scene->getDirectGPUAPI().setArticulationData(
                (void*)linkForces, applyIndices, PxArticulationGPUAPIWriteType::eLINK_FORCE, applyCount, start, finish);
            if (comps == 9)
                applied = scene->getDirectGPUAPI().setArticulationData((void*)linkTorques, applyIndices,
                                                                       PxArticulationGPUAPIWriteType::eLINK_TORQUE,
                                                                       applyCount, start, finish) &&
                          applied;
            return applied;
        }))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::setArticulationData refused the link force write", attribName);
        return false;
    }
    return true;
}

bool GpuRigidBodyView::setWrenchesOvStage(const TensorDesc* const srcTensor,
                                         const PxU32* const outRecordIdx,
                                         const PxU32 numOutputs,
                                         const uint64_t rowsToken, bool wholeSet,
                                         const OvStageDeviceWriteArgs& sessionDeviceArgs)
{
    // A LINK query cannot use the rigid-dynamic packed path -- a link has no rd index to pack, and
    // ovStageWriteBuffers rejects exactly that. Detected from the first row's record rather than
    // passed in: a query is one object type, so the rows are all links or all rigid dynamics.
    if (ovStageRowsAreLinks(outRecordIdx, numOutputs))
        return setLinkWrenchOvStage("wrench", srcTensor, outRecordIdx, numOutputs, rowsToken, 9u, wholeSet,
                                    sessionDeviceArgs);

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
    const bool session = sessionDeviceArgs.sessionOwned();
    const OvStageDeviceWriteArgs deviceArgs = session ? sessionDeviceArgs : drainDeviceWriteArgs(ApplyEvent::eRdForces);
    if (!refreshRdIndicesForWrite("wrench", deviceArgs))
        return false;
    if (numOutputs == 0)
        return true;

    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    const PxU32* rowsDev = nullptr;
    void* payload = nullptr;
    PxRigidDynamicGPUIndex* packedIdx = nullptr;
    if (!ovStageWriteBuffers("wrench", outRecordIdx, numOutputs, rowsToken, deviceArgs, 0, rowsDev, payload,
                             packedIdx))
        return false;
    if (!payload)
        return false;

    // The COM table the conversion needs, stale once any centre of mass changed since it was built
    // (centerOfMassEpoch). A session refreshed it at open.
    if (!getComsCacheStateValid())
    {
        if (deviceArgs.capturing)
        {
            CARB_LOG_ERROR("wrench: a centre of mass changed since this write was opened, and a captured commit "
                           "cannot refresh it. Reopen the write and capture again.");
            return false;
        }
        updateCMassData();
    }

    PxTransform* poses = static_cast<PxTransform*>(payload);
    PxVec3* outForces = nullptr;
    PxVec3* outTorques = nullptr;
    if (session)
    {
        OvStageScratch scratch(payload);
        poses = scratch.take<PxTransform>(numOutputs);
        outForces = scratch.take<PxVec3>(numOutputs);
        outTorques = scratch.take<PxVec3>(numOutputs);
        if (!deviceArgs.scratchHolds(outTorques + numOutputs, "wrench"))
            return false;
    }
    else
    {
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
        outForces = mOvStageWrenchDev;
        outTorques = mOvStageWrenchDev + numOutputs;
    }

    // Poses for exactly the addressed rows, read through the same packed index list, so the kernel
    // indexes them by output slot rather than by record. The pose block belongs to this write alone.
    if (!physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().getRigidDynamicData(poses, packedIdx,
                                                                PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE,
                                                                numOutputs, start, finish);
        }))
    {
        CARB_LOG_ERROR("wrench: PxDirectGPUAPI::getRigidDynamicData refused the pose read the "
                       "torque-about-COM conversion needs");
        return false;
    }

    // An unselected row of a masked commit applies no load: PhysX cannot read one back to keep.
    if (!submitRbWrenchOvStage(outForces, outTorques, static_cast<const float*>(srcTensor->data), poses,
                                cMassLocalPosePosDev, rowsDev, numOutputs, deviceArgs.stream, deviceArgs.mask))
    {
        CARB_LOG_ERROR("wrench: ovstage scatter failed");
        return false;
    }

    // Two write types, one packed index list, and no compaction anywhere: numOutputs IS the count,
    // known before the launch. The tensor binding's equivalent flags every body and takes the count
    // off the device through thrust::copy_if, which is the host block this path exists without.
    if (!physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
            const bool forces = scene->getDirectGPUAPI().setRigidDynamicData(
                outForces, packedIdx, PxRigidDynamicGPUAPIWriteType::eFORCE, numOutputs, start, finish);
            const bool torques = scene->getDirectGPUAPI().setRigidDynamicData(
                outTorques, packedIdx, PxRigidDynamicGPUAPIWriteType::eTORQUE, numOutputs, start, finish);
            return forces && torques;
        }))
    {
        CARB_LOG_ERROR("wrench: PxDirectGPUAPI::setRigidDynamicData refused the write");
        return false;
    }
    return true;
}

bool GpuRigidBodyView::setForcesOvStage(const TensorDesc* srcTensor,
                                       const PxU32* outRecordIdx,
                                       const PxU32 numOutputs,
                                       const uint64_t rowsToken, bool wholeSet,
                                       const OvStageDeviceWriteArgs& deviceArgs)
{
    // A LINK query cannot use the rigid-dynamic packed path -- a link has no rd index to pack, and
    // ovStageWriteBuffers rejects exactly that. Detected from the first row's record rather than
    // passed in: a query is one object type, so the rows are all links or all rigid dynamics.
    if (ovStageRowsAreLinks(outRecordIdx, numOutputs))
        return setLinkWrenchOvStage("force", srcTensor, outRecordIdx, numOutputs, rowsToken, 3u, wholeSet, deviceArgs);

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
                                    numOutputs, rowsToken, deviceArgs);
}

bool GpuRigidBodyView::setLinearVelocitiesOvStage(const TensorDesc* srcTensor,
                                                  const PxU32* outRecordIdx,
                                                  const PxU32 numOutputs,
                                                  const uint64_t rowsToken, bool,
                                                  const OvStageDeviceWriteArgs& deviceArgs)
{
    return setVelocityColumnOvStage("linearVelocity", srcTensor, PxRigidDynamicGPUAPIWriteType::eLINEAR_VELOCITY,
                                    outRecordIdx, numOutputs, rowsToken, deviceArgs);
}

bool GpuRigidBodyView::setAngularVelocitiesOvStage(const TensorDesc* srcTensor,
                                                   const PxU32* outRecordIdx,
                                                   const PxU32 numOutputs,
                                                   const uint64_t rowsToken, bool,
                                                   const OvStageDeviceWriteArgs& deviceArgs)
{
    return setVelocityColumnOvStage("angularVelocity", srcTensor, PxRigidDynamicGPUAPIWriteType::eANGULAR_VELOCITY,
                                    outRecordIdx, numOutputs, rowsToken, deviceArgs);
}

bool GpuRigidBodyView::getVelocities(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, mNumArtis > 0, false);

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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, mNumArtis > 0, false);

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

bool GpuRigidBodyView::setTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, mNumArtis > 0, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, mNumArtis > 0, false);
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
    CARB_LOG_WARN("Deprecated function IRigidBodyView::applyForces, please use IRigidBodyView::applyForcesAndTorquesAtPosition instead.");
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
    if (!mGpuSimData->rewriteTable(cMassLocalPosePosDev, cMassLocalPosePos.data(), mEntries.size() * sizeof(PxVec3)))
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, mNumArtis > 0, false);
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
GPU_RB_MASKED_SETTER(setTransforms, )
GPU_RB_MASKED_SETTER(setVelocities, )
GPU_RB_MASKED_SETTER(applyForces, )
GPU_RB_MASKED_SETTER(setMasses, )
GPU_RB_MASKED_SETTER(setCOMs, )
GPU_RB_MASKED_SETTER(setInertias, )
// setKinematicTargets/setDisable*/material/rest/contact/compliant Masked: BaseRigidBodyView

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


// setKinematicTargets/setDisable*/material/rest/contact/compliant Masked: BaseRigidBodyView


// OMPE-103213: require host tensors -- no silent GPU staging. The base setter owns the
// device-independent solver refresh, so DirectGPU does not carry a separate behavior path.
bool GpuRigidBodyView::setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!requireHostTensor(srcTensor, "src", "setDisableGravities"))
        return false;
    if (!requireHostTensor(indexTensor, "index", "setDisableGravities"))
        return false;
    if (indexTensor && indexTensor->data && !checkIndexTensorSize(*indexTensor, getCount(), "setDisableGravities"))
        return false;

    const TensorDesc* idxArg = (indexTensor && indexTensor->data) ? indexTensor : nullptr;
    return BaseRigidBodyView::setDisableGravities(srcTensor, idxArg);
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

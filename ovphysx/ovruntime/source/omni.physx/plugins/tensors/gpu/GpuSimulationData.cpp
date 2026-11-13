// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-VIEWS-001
// @covers AC-1 AC-3

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-5
 *
 * @implements REQ-TENSOR-CONTACT-005
 * @covers AC-3 AC-5
 *
 * @implements REQ-TENSOR-VIEW-001
 * @covers AC-14 AC-15 AC-16 AC-17
 */

// @implements REQ-INPUT-DEVICE-001
// @covers AC-14
//
// @implements REQ-INPUT-COVERAGE-001
// @covers AC-16

// clang-format off
// clang-format on

#include "AttachmentObjectLookup.h"
#include "internal/Internal.h" // recordLifetimeEpoch
#include "tensors/gpu/GpuSimulationData.h"

#include "tensors/CommonTypes.h"
#include "tensors/GlobalsAreBad.h"
#include "tensors/SimulationBackend.h"
#include "tensors/gpu/CudaKernels.h"

#include "PhysXTools.h"
#include "usdLoad/AttachedStage.h"
#include "usdLoad/LoadUsd.h"

#include <omni/physx/IPhysx.h>

#include <PxPhysicsAPI.h>

#include <algorithm>

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

GpuSimulationData::GpuSimulationData(SimulationBackend& backend,
                                     omni::physics::tensors::AttachHandle attachHandle,
                                     std::shared_ptr<GpuRdDisableState> rdDisableState)
    : mBackend(backend)
    , mAttachHandle(attachHandle)
    , mRdDisableState(std::move(rdDisableState))
    , mRdDisableEpoch(mRdDisableState->epoch)
    , mMayHaveDisabledRd(mRdDisableState->mayHaveDisabledRd)
{
}

bool GpuSimulationData::drainDirectGpuFinish(CUevent finishEvent, const char* label) const
{
    if (getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(finishEvent), 0, nullptr))
    {
        return true;
    }

    // PhysX may have queued work and recorded the finish event even when the DirectGPU call reports
    // failure. If the asynchronous dependency cannot be queued, drain that producer before the
    // caller is allowed to release the output or submit another user of the scratch.
    CARB_LOG_ERROR("Failed to queue the completion wait for %s; draining the producer event", label);
    if (!getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(finishEvent), nullptr))
    {
        CARB_LOG_ERROR("Failed to synchronize the completion event for %s; draining the CUDA device", label);
        if (!CHECK_CUDA(cudaDeviceSynchronize()))
        {
            // Last fallback, and it failed too. Nothing has established that the producer stopped
            // writing, so no later user of this scratch can be given a safe ordering edge either --
            // quarantine every shared buffer rather than let the next producer proceed on an
            // assumption that has just been disproved. Fail closed: a stalled read is recoverable,
            // a gather racing a producer is not.
            quarantineAllSharedBuffers("device drain after a failed completion wait");
        }
    }
    return false;
}


bool GpuSimulationData::awaitDirectGpuFetch(bool issued, CUevent finishEvent, const char* label) const
{
    if (!issued)
    {
        CARB_LOG_ERROR("PxDirectGPUAPI refused the read for %s", label);
        drainDirectGpuFinish(finishEvent, label);
        return false;
    }
    return drainDirectGpuFinish(finishEvent, label);
}


void GpuSimulationData::quarantineAllSharedBuffers(const char* reason) const
{
    for (PxU32 i = 0; i < PxU32(SharedDeviceBuffer::eCOUNT); ++i)
        mSharedBufferQuarantined[i] = true;
    CARB_LOG_ERROR_ONCE("Shared device scratch quarantined (%s): the completion protocol lost its "
                        "ordering edge and could not re-establish it. Reads over this scene's scratch "
                        "are refused from here on.",
                        reason ? reason : "unknown");
}

bool GpuSimulationData::recordKernelDone(PxU32 buf)
{
    if (buf >= SharedDeviceBuffer::eCOUNT)
        return false;

    // Postcondition callers rely on: on return the buffer is either guarded by an event that signals
    // when our kernel finishes, or the kernel has already finished. A missing edge must therefore be
    // made unnecessary by draining the stream -- handing the next Fetch a nullptr startEvent is the
    // overwrite-under-gather race this protocol closes.
    int status = 0;
    const bool recorded =
        mKernelDoneEvents[buf] &&
        getCudaShim()->eventRecord(reinterpret_cast<uintptr_t>(mKernelDoneEvents[buf]), uintptr_t(0), &status);
    if (recorded)
        return true;

    CARB_LOG_ERROR_ONCE("cuEventRecord failed for shared device buffer %u (CUresult %d); falling back to a "
                        "synchronous drain, which is correct but stalls the host.",
                        buf, status);
    if (CHECK_CU(getCudaShim()->streamSynchronize(uintptr_t(0), nullptr)))
        return true; // no edge, but the kernel has demonstrably finished -- the postcondition holds

    // BOTH legs failed, so the postcondition above is false: nothing proves our kernel stopped
    // touching this buffer. The result used to be discarded here, which handed the next DirectGPU
    // producer a startEvent that was never recorded -- and CUDA treats a never-recorded event as
    // already complete, so the producer would wait for nothing and overwrite scratch mid-gather.
    //
    // Latched rather than merely returned: this function is called from gatherThenRelease AFTER the
    // kernel is enqueued, so the immediate caller can no longer undo anything. What has to change is
    // what happens NEXT, and only a sticky flag reaches those callers.
    mSharedBufferQuarantined[buf] = true;
    CARB_LOG_ERROR_ONCE("Shared device buffer %u quarantined: neither the completion event nor the "
                        "fallback drain could establish that our kernel finished with it.",
                        buf);
    return false;
}

bool GpuSimulationData::noteTableReader(CUstream stream)
{
    // An operation whose event has completed no longer reads the tables.
    for (size_t i = 0; i < mTableReaders.size();)
    {
        if (cudaEventQuery(mTableReaders[i]) == cudaSuccess)
        {
            mIdleTableReaders.push_back(mTableReaders[i]);
            mTableReaders[i] = mTableReaders.back();
            mTableReaders.pop_back();
        }
        else
        {
            ++i;
        }
    }

    IOptionalCuda* const cuda = getCudaShim();
    uintptr_t event = 0;
    if (!mIdleTableReaders.empty())
    {
        event = reinterpret_cast<uintptr_t>(mIdleTableReaders.back());
        mIdleTableReaders.pop_back();
    }
    int status = 0;
    if ((event || cuda->eventCreate(&event, CU_EVENT_DISABLE_TIMING, &status)) &&
        cuda->eventRecord(event, reinterpret_cast<uintptr_t>(stream), &status))
    {
        mTableReaders.push_back(reinterpret_cast<CUevent>(event));
        return true;
    }
    if (event)
        mIdleTableReaders.push_back(reinterpret_cast<CUevent>(event));

    // Without the record a later rewrite cannot tell when this operation stops reading, so wait for it now.
    CARB_LOG_ERROR_ONCE("Recording a stream-ordered commit or read for the table rewrites failed (CUresult %d); "
                        "waiting for its stream instead, which stalls the host.",
                        status);
    return CHECK_CU(cuda->streamSynchronize(reinterpret_cast<uintptr_t>(stream), nullptr));
}

bool GpuSimulationData::waitForTableReaders()
{
    while (!mTableReaders.empty())
    {
        if (!CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(mTableReaders.back()), nullptr)))
            return false;
        mIdleTableReaders.push_back(mTableReaders.back());
        mTableReaders.pop_back();
    }
    return true;
}

unsigned long long* GpuSimulationData::keptPinnedTargetsCounter()
{
    if (mKeptPinnedTargetsDev || !mCudaContextManager)
        return mKeptPinnedTargetsDev;
    PxCudaContext* const cu = mCudaContextManager->getCudaContext();
    void* host = nullptr;
    CUdeviceptr dev = 0;
    if (!cu ||
        cu->memHostAlloc(&host, sizeof(unsigned long long), CU_MEMHOSTALLOC_PORTABLE | CU_MEMHOSTALLOC_DEVICEMAP) !=
            CUDA_SUCCESS ||
        !host)
    {
        CARB_LOG_ERROR("Allocating the kinematic-target kept-pinned count failed.");
        return nullptr;
    }
    if (cu->memHostGetDevicePointer(&dev, host, 0) != CUDA_SUCCESS || !dev)
    {
        CARB_LOG_ERROR("Mapping the kinematic-target kept-pinned count for the device failed.");
        cu->memFreeHost(host);
        return nullptr;
    }
    mKeptPinnedTargetsHost = static_cast<unsigned long long*>(host);
    *mKeptPinnedTargetsHost = 0;
    mKeptPinnedTargetsDev = reinterpret_cast<unsigned long long*>(dev);
    return mKeptPinnedTargetsDev;
}

void logKeptPinnedTargets(unsigned long long count)
{
    if (count == 0)
        return;
    CARB_LOG_ERROR("Kinematic-target writes marked %llu pinned volume-deformable node(s) free; they stayed pinned at "
                   "their last target. A pinned node cannot be freed: PhysX does not restore the inverse mass it "
                   "zeroed to pin it.",
                   count);
}

void GpuSimulationData::reportKeptPinnedTargets()
{
    if (!mKeptPinnedTargetsHost)
        return;
    const unsigned long long count = *static_cast<volatile unsigned long long*>(mKeptPinnedTargetsHost);
    if (count <= mKeptPinnedTargetsReported)
        return;
    logKeptPinnedTargets(count - mKeptPinnedTargetsReported);
    mKeptPinnedTargetsReported = count;
}

bool GpuSimulationData::rewriteTable(void* dst, const void* src, size_t bytes)
{
    if (!waitForTableReaders())
        return false;
    // A copy from pageable memory may return before it reaches the device, and a non-blocking stream does not
    // wait for the legacy default stream it runs on.
    return CHECK_CUDA(cudaMemcpy(dst, src, bytes, cudaMemcpyHostToDevice)) &&
           CHECK_CUDA(cudaStreamSynchronize(nullptr));
}

namespace
{
// Resolves a PhysX actor userData id to the legacy asInt(ObjectKey)-encoded path id
// (see CommonTypes.h's asInt()/keyFromLegacyId() comment) that GpuActorPathIdPair::pathId
// and raw-contact consumers expect. Same encoding as ContactReport.cpp's
// keyToLegacyPathInt, factored out once here rather than repeating this at each
// of the three actor-collection call sites below (rigid dynamics / statics /
// articulation links).
bool legacyPathIdForObject(omni::physics::AttachHandle attach, size_t objectId, uint64_t& pathIdOut)
{
    omni::physics::parse::ObjectKey objectKey = attachmentLookup::keyForObject(usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach), objectId);
    if (!objectKey.valid())
    {
        return false;
    }
    pathIdOut = asInt(objectKey);
    return true;
}
} // namespace

// Raise a DirectGPU scene's live per-articulation counts to the strides PhysX lays its DirectGPU articulation blocks
// out with: the largest counts it has seen in the scene, never lowered on removal, so once the largest articulation
// is removed the live counts alone misplace every articulation but the first. PhysX reports them only after the
// scene's first step; before that, on a non-DirectGPU scene and on one without articulations, the counts are
// unchanged. The larger of the two is kept, since the live counts can lead PhysX's between an articulation's
// insertion and the next simulate.
static void raiseToPhysxArticulationStrides(PxScene* scene,
                                            PxU32& maxLinks,
                                            PxU32& maxDofs,
                                            PxU32& maxFixedTendons,
                                            PxU32& maxSpatialTendons)
{
    // PhysX refuses the count query (with an error) until its DirectGPU API is initialized at the end of the first
    // step. The scene timestamp starts at 1 and moves in fetchResults, after that, so above 1 means a step completed.
    // No articulation, no stride: left at zero.
    if (!scene || !scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API) || scene->getTimestamp() <= 1 ||
        scene->getNbArticulations() == 0)
        return;
    const PxArticulationGPUAPIMaxCounts counts = scene->getDirectGPUAPI().getArticulationGPUAPIMaxCounts();
    maxLinks = PxMax(maxLinks, counts.maxLinks);
    maxDofs = PxMax(maxDofs, counts.maxDofs);
    maxFixedTendons = PxMax(maxFixedTendons, counts.maxFixedTendons);
    maxSpatialTendons = PxMax(maxSpatialTendons, counts.maxSpatialTendons);
}

void articulationStrides(
    PxScene* scene, PxU32& maxLinks, PxU32& maxDofs, PxU32& maxFixedTendons, PxU32& maxSpatialTendons)
{
    maxLinks = 0;
    maxDofs = 0;
    maxFixedTendons = 0;
    maxSpatialTendons = 0;
    const PxU32 numArtis = scene->getNbArticulations();
    std::vector<PxArticulationReducedCoordinate*> artis(numArtis);
    scene->getArticulations(artis.data(), numArtis);
    for (PxArticulationReducedCoordinate* arti : artis)
    {
        if (!arti)
            continue;
        // getDofs() is the joint DOF total (the links' inbound DOFs; a floating base's six are not joint DOFs), and
        // 0xffffffff for an articulation not in a scene.
        const PxU32 numDofs = arti->getDofs();
        maxLinks = PxMax(maxLinks, arti->getNbLinks());
        if (numDofs != 0xffffffff)
            maxDofs = PxMax(maxDofs, numDofs);
        maxFixedTendons = PxMax(maxFixedTendons, arti->getNbFixedTendons());
        maxSpatialTendons = PxMax(maxSpatialTendons, arti->getNbSpatialTendons());
    }
    raiseToPhysxArticulationStrides(scene, maxLinks, maxDofs, maxFixedTendons, maxSpatialTendons);
}

bool GpuSimulationData::init(PxScene* scene)
{
    if (!scene)
    {
        CARB_LOG_ERROR("Failed to prepare GPU data: null scene");
        return false;
    }

    PxSceneFlags sceneFlags = scene->getFlags();
    if (!sceneFlags.isSet(PxSceneFlag::eENABLE_GPU_DYNAMICS))
    {
        CARB_LOG_ERROR("Failed to prepare GPU data: simulation is running on CPU");
        return false;
    }
    if (!sceneFlags.isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API))
    {
        CARB_LOG_ERROR("Failed to prepare GPU data: direct GPU API is not enabled");
        return false;
    }

    // Sampled before the walk, so a record change racing it costs a rebuild rather than a stale map.
    mRecordEpoch = internal::recordLifetimeEpoch(scene);

    PxU32 maxLinks = 0;
    PxU32 maxDofs = 0;
    PxU32 maxFixedTendons = 0;
    PxU32 maxSpatialTendons = 0;
    articulationStrides(scene, maxLinks, maxDofs, maxFixedTendons, maxSpatialTendons);

    // Every articulation under its root link's island node, with its GPU index.
    PxU32 numArtis = scene->getNbArticulations();
    uint32_t maxArtiNodeIdx = 0;
    std::vector<PxArticulationReducedCoordinate*> artis(numArtis);
    scene->getArticulations(artis.data(), numArtis);
    for (PxArticulationReducedCoordinate* arti : artis)
    {
        PxArticulationLink* root = nullptr;
        arti->getLinks(&root, 1);
        const PxNodeIndex nodeIdx = root->getInternalIslandNodeIndex();
        mNodeIdx2ArtiIdxMap[nodeIdx.index()] = arti->getGPUIndex();
        maxArtiNodeIdx = PxMax(maxArtiNodeIdx, nodeIdx.index());
    }

    std::vector<uint32_t> nodeIdx2ArtiGpuIdx(maxArtiNodeIdx + 1, 0xffffffff);
    for(auto&item : mNodeIdx2ArtiIdxMap){
        nodeIdx2ArtiGpuIdx[item.first] = item.second;
    }

    PxU32 numRds = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
    mScene = scene;
    mCudaContextManager = scene->getCudaContextManager();
    mNumArtis = numArtis;
    mNumRds = numRds;
    mMaxLinks = maxLinks;
    mMaxDofs = maxDofs;
    mMaxFixedTendons = maxFixedTendons;
    mMaxSpatialTendons = maxSpatialTendons;
    mLinkBufSize = numArtis * maxLinks;
    mDofBufSize = numArtis * maxDofs;
    mFixedTendonBufSize = numArtis * maxFixedTendons;
    mSpatialTendonBufSize = numArtis * maxSpatialTendons;

    PhysxCudaContextGuard ctxGuarg(mCudaContextManager);

    SHIM_CU_CTX_GET_CURRENT(&mCtx);

    VALIDATE_CUDA_CONTEXT();

    // determine the device ordinal that PhysX is using
    {
        int deviceCount = 0;
        if (!CHECK_CU(getCudaShim()->deviceGetCount(&deviceCount, nullptr)))
        {
            CARB_LOG_ERROR("Failed to get device count");
            return false;
        }

        std::map<CUdevice, int> deviceMap;
        for (int i = 0; i < deviceCount; i++)
        {
            int dev = -1;
            if (CHECK_CU(getCudaShim()->deviceGet(&dev, i, nullptr)))
            {
                deviceMap[static_cast<CUdevice>(dev)] = i;
            }
        }

        int devOrdinal = -1;
        if (CHECK_CU(getCudaShim()->ctxGetDevice(&devOrdinal, nullptr)))
        {
            auto it = deviceMap.find(static_cast<CUdevice>(devOrdinal));
            if (it != deviceMap.end())
            {
                mDevice = it->second;
            }
            else
            {
                CARB_LOG_ERROR("Failed to determine PhysX CUDA device ordinal");
                return false;
            }
        }
        else
        {
            CARB_LOG_ERROR("Failed to determine PhysX CUDA device");
            return false;
        }

        CARB_LOG_INFO("Physics using context %p, device %d", mCtx, mDevice);
    }

    // A refused device allocation fails the build, so the backend keeps nothing half-built and the next acquire
    // builds the data again (REQ-TENSOR-VIEW-001 AC-15). What was allocated is freed with the object. The inverse
    // dynamics scratch below is the one exception.
    if (numArtis > 0)
    {
        // node indices to global articulation indices map
        if (!prepareDeviceData((void**)&mNodeIdx2ArtiGpuIdxDev, nodeIdx2ArtiGpuIdx.data(),
                               (size_t)nodeIdx2ArtiGpuIdx.size() * sizeof(PxU32), "mNodeIdx2ArtiGpuIdxDev"))
            return false;
        mNodeIdx2ArtiGpuIdxCount = PxU32(nodeIdx2ArtiGpuIdx.size());
    }

    if (mMaxDofs > 0)
    {
        // arti DOF positions/velocity/targets/etc
        if (!prepareDeviceData((void**)&mDofScalarsDev, nullptr, mDofBufSize * sizeof(float), "mDofScalarsDev"))
            return false;
        // arti DOF actuation forces
        if (!prepareDeviceData(
                 (void**)&mDofActuationForcesDev, nullptr, mDofBufSize * sizeof(float), "mDofActuationForcesDev"))
            return false;
    }

    // The inverse dynamics matrices are ROOT-INCLUSIVE, so they exist for a zero-dof articulation and are
    // sized by numArtis rather than by mMaxDofs. Every size below is (6 + mMaxDofs)-based: at zero
    // dofs that is 6, a 6x6 mass matrix and Jacobian, six-component Coriolis/gravity forces and a
    // 6x7 centroidal matrix -- all meaningful for a floating body, all closed-form.
    //
    // These used to sit inside `if (mMaxDofs > 0)` with the two genuinely DOF-only buffers above.
    // A floating single-link articulation therefore reached the device with null scratch, and the
    // read had to decline rather than hand computeArticulationData a null buffer (which terminates
    // the process). The CPU path served the same scene, so one requirement answered two ways
    // depending on the backend.
    //
    // The scratch is optional: a scene whose state fits on the device but not this scratch keeps its other reads,
    // and every inverse dynamics read refuses (checkInverseDynamicsScratch). It is kept whole or not at all, so no
    // inverse dynamics read works while another refuses (REQ-TENSOR-VIEW-001 AC-15).
    if (numArtis > 0)
    {
        mJacobianMaxCols = 6 + mMaxDofs;
        mJacobianMaxRows = 6 + (mMaxLinks - 1) * 6;
        uint32_t jacobianSize = mJacobianMaxCols * mJacobianMaxRows;
        uint32_t jacobianBufSize = numArtis * jacobianSize;
        uint32_t coriolisGravityBuffSize = numArtis * (6 + mMaxDofs);
        uint32_t massMatrixBuffSize = numArtis * (6 + mMaxDofs) * (6 + mMaxDofs);
        uint32_t sizeMassMatrices = numArtis * (maxDofs + 6) * (maxDofs + 6);
        uint32_t sizeCoriolisForces = numArtis * (maxDofs + 6);
        uint32_t sizeCentroidalMomentumMatrices = numArtis * 6 * (maxDofs + 6);
        uint32_t sizeCentroidalMomentumBiasForces = numArtis * 6;
        uint32_t sizeCentroidalMomentumData =
            sizeMassMatrices + sizeCoriolisForces + sizeCentroidalMomentumMatrices + sizeCentroidalMomentumBiasForces;
        const bool scratchAllocated =
            prepareDeviceData(
                (void**)&mJacobianDataDev, nullptr, jacobianBufSize * sizeof(float), "mJacobianDataDev") &&
            prepareDeviceData((void**)&mCoriolisGravityDataDev, nullptr, coriolisGravityBuffSize * sizeof(float),
                              "mCoriolisGravityDataDev") &&
            prepareDeviceData(
                (void**)&mMassMatrixDataDev, nullptr, massMatrixBuffSize * sizeof(float), "mMassMatrixDataDev") &&
            prepareDeviceData((void**)&mCentroidalMomentumDataDev, nullptr, sizeCentroidalMomentumData * sizeof(float),
                              "mCentroidalMomentumDataDev");
        if (!scratchAllocated)
        {
            for (float** scratch :
                 { &mJacobianDataDev, &mCoriolisGravityDataDev, &mMassMatrixDataDev, &mCentroidalMomentumDataDev })
            {
                if (*scratch)
                    CHECK_CUDA(cudaFree(*scratch));
                *scratch = nullptr;
            }
        }
    }
    
    if (mMaxLinks > 0)
    {
        if (!prepareDeviceData((void**)&mLinkOrRootTransformsDev, nullptr, mLinkBufSize * sizeof(PxTransform),
                               "mLinkOrRootTransformsDev"))
            return false;

        if (!prepareDeviceData((void**)&mLinkOrRootLinearVelAccDev, nullptr, mLinkBufSize * sizeof(PxVec3),
                               "mLinkOrRootLinearVelAccDev"))
            return false;

        if (!prepareDeviceData((void**)&mLinkOrRootAngularVelAccDev, nullptr, mLinkBufSize * sizeof(PxVec3),
                               "mLinkOrRootAngularVelAccDev"))
            return false;
        if (!prepareDeviceData((void**)&mLinkForcesDev, nullptr, mLinkBufSize * sizeof(PxVec3), "mLinkForcesDev"))
            return false;

        if (!prepareDeviceData((void**)&mLinkTorquesDev, nullptr, mLinkBufSize * sizeof(PxVec3), "mLinkTorquesDev"))
            return false;

        if (!prepareDeviceData((void**)&mLinkIncomingJointForceDev, nullptr,
                               mLinkBufSize * sizeof(PhysxGpuSpatialForces), "mLinkIncomingJointForceDev"))
            return false;
    }

    if (numRds > 0)
    {
        // Global index of rigid dynamics (0..numRds): getInternalIslandNodeIndex mixes RDs and articulations,
        // so this separate index is needed to size GPU staging buffers correctly.
        std::vector<PxActor*> rdActors(numRds);
        std::vector<PxRigidDynamicGPUIndex> rdGpuIndices(numRds);
        scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, rdActors.data(), numRds);
        for (PxU32 i = 0; i < numRds; i++)
        {
            PxRigidDynamic* rd = static_cast<PxRigidDynamic*>(rdActors[i]);
            mActor2RdIndexMap[rd] = i;
            rdGpuIndices[i] = rd->getGPUIndex();
            // Seed the scene-wide disable hint off the index this loop already read: PhysX returns
            // the invalid node handle for a body with no island node, which is what
            // eDISABLE_SIMULATION leaves behind. Catches bodies authored disabled -- they are
            // disabled before any view exists, so no epoch bump ever describes them.
            if (PxU32(rdGpuIndices[i]) == 0xffffffffu)
            {
                mMayHaveDisabledRd = true;
            }
        }

        if (!prepareDeviceData((void**)&mRdPoseDev, nullptr, mNumRds * sizeof(PxTransform), "mRdPoseDev"))
            return false;
        if (!prepareDeviceData((void**)&mRdLinearVelAccDev, nullptr, mNumRds * sizeof(PxVec3), "mRdLinearVelAccDev"))
            return false;
        if (!prepareDeviceData((void**)&mRdAngularVelAccDev, nullptr, mNumRds * sizeof(PxVec3), "mRdAngularVelAccDev"))
            return false;
        if (!prepareDeviceData((void**)&mRdForcesDev, nullptr, mNumRds * sizeof(PxVec3), "mRdForcesDev"))
            return false;
        if (!prepareDeviceData((void**)&mRdTorquesDev, nullptr, mNumRds * sizeof(PxVec3), "mRdTorquesDev"))
            return false;
    }
    // fixed tendons
    if (mMaxFixedTendons > 0)
    {
        if (!prepareDeviceData((void**)&mFixedTendonPropertiesDev, nullptr,
                               mFixedTendonBufSize * sizeof(PxGpuFixedTendonData), "mFixedTendonPropertiesDev"))
            return false;
    }
    // spatial tendons
    if (mMaxSpatialTendons > 0)
    {
        if (!prepareDeviceData((void**)&mSpatialTendonPropertiesDev, nullptr,
                               mSpatialTendonBufSize * sizeof(PxGpuSpatialTendonData), "mSpatialTendonPropertiesDev"))
            return false;
    }


    // contacts

    // PxSceneDesc doesn't expose the GPU dynamics config, so read it from the source
    // instead, resolved through the attach handle rather than the USD stage cache
    // (ADR-0013) -- a stageless attach has no stage-cache id to look up.
    usdparser::AttachedStage* attachedStage =
        usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(mAttachHandle);
    if (attachedStage && g_physx)
    {
        size_t sceneObjectId = reinterpret_cast<size_t>(scene->userData);
        // Source-backed read (IPhysicsSource::getAttributeAtTime via PhysXTools.h's
        // pxr-free getValue overload), not USD-specific -- works identically on the
        // UsdSource backend.
        omni::physics::parse::ObjectKey sceneKey = attachmentLookup::keyForObject(attachedStage, sceneObjectId);
        if (sceneKey.valid())
        {
            unsigned value = 0;
            omni::physics::parse::KnownTokens tok;
            if (const omni::physics::parse::IPhysicsSource* src = attachedStage->getSource())
                tok.intern(*src);
            if (internal::getValue(*attachedStage, sceneKey, tok.physxSceneGpuMaxRigidContactCount,
                                   omni::physics::parse::ReadTime::defaultTime(), value))
            {
                mMaxGpuContactPairs = value;
            }
        }
    }
    else
    {
        CARB_LOG_WARN("Unable to determine max GPU contact pairs, using default %u", mMaxGpuContactPairs);
    }

    if (!prepareDeviceData((void**)&mGpuContactPairsDev, nullptr,
                           (size_t)mMaxGpuContactPairs * sizeof(PxGpuContactPair), "mGpuContactPairsDev"))
        return false;
    if (!prepareDeviceData((void**)&mGpuContactPairCountDev, nullptr, sizeof(PxU32),
                           "mGpuContactPairCountDev"))
        return false;

    // Build global actor-to-pathId lookup for raw contact data, so contact views can identify which
    // actor a contact is with on GPU (userData holds an object ID, which is converted to a path ID here).
    // pathId uses the legacy asInt(ObjectKey) encoding (see CommonTypes.h's asInt()/
    // keyFromLegacyId()) so raw contact actor ids stay consistent with the rest of the
    // tensor API's legacy id encoding (RigidContactSensorEntry::referentId).
    if (g_physx)
    {
        std::vector<GpuActorPathIdPair> actorPathLookup;

        // Add all rigid dynamics
        if (numRds > 0)
        {
            std::vector<PxActor*> rdActors(numRds);
            scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, rdActors.data(), numRds);
            for (PxActor* actor : rdActors)
            {
                if (actor && actor->userData)
                {
                    size_t objectId = reinterpret_cast<size_t>(actor->userData);
                    uint64_t pathId;
                    if (legacyPathIdForObject(mAttachHandle, objectId, pathId))
                    {
                        GpuActorPathIdPair pair;
                        pair.actor = actor;
                        pair.pathId = pathId;
                        actorPathLookup.push_back(pair);
                    }
                }
            }
        }

        // Add all rigid statics (ground plane, etc.)
        PxU32 numStatics = scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC);
        if (numStatics > 0)
        {
            std::vector<PxActor*> staticActors(numStatics);
            scene->getActors(PxActorTypeFlag::eRIGID_STATIC, staticActors.data(), numStatics);
            for (PxActor* actor : staticActors)
            {
                if (actor && actor->userData)
                {
                    size_t objectId = reinterpret_cast<size_t>(actor->userData);
                    uint64_t pathId;
                    if (legacyPathIdForObject(mAttachHandle, objectId, pathId))
                    {
                        GpuActorPathIdPair pair;
                        pair.actor = actor;
                        pair.pathId = pathId;
                        actorPathLookup.push_back(pair);
                    }
                }
            }
        }

        // Add all articulation links
        for (PxArticulationReducedCoordinate* arti : artis)
        {
            if (arti)
            {
                PxU32 numLinks = arti->getNbLinks();
                std::vector<PxArticulationLink*> links(numLinks);
                arti->getLinks(links.data(), numLinks);
                for (PxArticulationLink* link : links)
                {
                    if (link && link->userData)
                    {
                        size_t objectId = reinterpret_cast<size_t>(link->userData);
                        uint64_t pathId;
                        if (legacyPathIdForObject(mAttachHandle, objectId, pathId))
                        {
                            GpuActorPathIdPair pair;
                            pair.actor = link;
                            pair.pathId = pathId;
                            actorPathLookup.push_back(pair);
                        }
                    }
                }
            }
        }

        // Sort for binary search on GPU
        std::sort(actorPathLookup.begin(), actorPathLookup.end(), GpuActorPathIdPair::LessThan());
        mNumActorPathPairs = PxU32(actorPathLookup.size());

        if (mNumActorPathPairs > 0)
        {
            if (!prepareDeviceData((void**)&mActorPathLookupDev, actorPathLookup.data(),
                                   mNumActorPathPairs * sizeof(GpuActorPathIdPair), "mActorPathLookupDev"))
                return false;
        }
    }

    // Synchronization events. Each carries an ordering edge, so a failed creation is fatal to init:
    // a null event does not degrade to a slower-but-correct path, it silently removes the edge.
    bool eventsOk = true;
    for (PxU32 i = 0; i < PxU32(CopyEvent::eCOUNT); i++)
    {
        eventsOk = SHIM_CU_EVENT_CREATE(&mCopyEvents[i], CU_EVENT_DISABLE_TIMING) && eventsOk;
        mCopyEventPointers[i] = mCopyEvents[i] ? &mCopyEvents[i] : nullptr;
    }
    for (PxU32 i = 0; i < PxU32(ApplyEvent::eCOUNT); i++)
    {
        eventsOk = SHIM_CU_EVENT_CREATE(&mApplyWaitEvents[i], CU_EVENT_DISABLE_TIMING) && eventsOk;
        eventsOk = SHIM_CU_EVENT_CREATE(&mApplySignalEvents[i], CU_EVENT_DISABLE_TIMING) && eventsOk;
    }

    for (PxU32 i = 0; i < PxU32(SharedDeviceBuffer::eCOUNT); i++)
    {
        eventsOk = SHIM_CU_EVENT_CREATE(&mKernelDoneEvents[i], CU_EVENT_DISABLE_TIMING) && eventsOk;
    }

    eventsOk = SHIM_CU_EVENT_CREATE(&mContactReadEvent, CU_EVENT_DISABLE_TIMING) && eventsOk;

    // Every creation is attempted before bailing, so the shim logs each refusal, not only the first.
    if (!eventsOk)
    {
        CARB_LOG_ERROR("Failed to prepare GPU data: could not create the CUDA synchronization events");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

GpuSimulationData::~GpuSimulationData()
{
    CudaContextGuard ctxGuard(mCtx);
    // Only what exists is freed: a build that failed part-way, out of device memory say, frees what it allocated and
    // makes no CUDA call for the rest.
    if (mNodeIdx2ArtiGpuIdxDev)
        CHECK_CUDA(cudaFree(mNodeIdx2ArtiGpuIdxDev));
    if (mGpuContactPairsDev)
        CHECK_CUDA(cudaFree(mGpuContactPairsDev));
    if (mGpuContactPairCountDev)
        CHECK_CUDA(cudaFree(mGpuContactPairCountDev));
    if (mActorPathLookupDev)
        CHECK_CUDA(cudaFree(mActorPathLookupDev));

    if (mLinkOrRootLinearVelAccDev)
        CHECK_CUDA(cudaFree(mLinkOrRootLinearVelAccDev));
    if (mLinkOrRootAngularVelAccDev)
        CHECK_CUDA(cudaFree(mLinkOrRootAngularVelAccDev));
    if (mLinkOrRootTransformsDev)
        CHECK_CUDA(cudaFree(mLinkOrRootTransformsDev));
    if (mDofScalarsDev)
        CHECK_CUDA(cudaFree(mDofScalarsDev));
    if (mLinkForcesDev)
        CHECK_CUDA(cudaFree(mLinkForcesDev));
    if (mLinkTorquesDev)
        CHECK_CUDA(cudaFree(mLinkTorquesDev));
    if (mDofActuationForcesDev)
        CHECK_CUDA(cudaFree(mDofActuationForcesDev));
    if (mLinkIncomingJointForceDev)
        CHECK_CUDA(cudaFree(mLinkIncomingJointForceDev));
    if (mFixedTendonPropertiesDev)
        CHECK_CUDA(cudaFree(mFixedTendonPropertiesDev));
    if (mSpatialTendonPropertiesDev)
        CHECK_CUDA(cudaFree(mSpatialTendonPropertiesDev));


    if (mRdPoseDev)
        CHECK_CUDA(cudaFree(mRdPoseDev));
    if (mRdLinearVelAccDev)
        CHECK_CUDA(cudaFree(mRdLinearVelAccDev));
    if (mRdAngularVelAccDev)
        CHECK_CUDA(cudaFree(mRdAngularVelAccDev));
    if (mRdForcesDev)
        CHECK_CUDA(cudaFree(mRdForcesDev));
    if (mRdTorquesDev)
        CHECK_CUDA(cudaFree(mRdTorquesDev));

    if (mJacobianDataDev)
        CHECK_CUDA(cudaFree(mJacobianDataDev));
    if (mMassMatrixDataDev)
        CHECK_CUDA(cudaFree(mMassMatrixDataDev));
    if (mCoriolisGravityDataDev)
        CHECK_CUDA(cudaFree(mCoriolisGravityDataDev));
    if (mCentroidalMomentumDataDev)
        CHECK_CUDA(cudaFree(mCentroidalMomentumDataDev));


    // The frees above waited for the device, so no commit still adds to the count.
    if (mKeptPinnedTargetsHost)
    {
        reportKeptPinnedTargets();
        if (mCudaContextManager->getCudaContext()->memFreeHost(mKeptPinnedTargetsHost) != CUDA_SUCCESS)
            CARB_LOG_ERROR("Freeing the kinematic-target kept-pinned count failed.");
        mKeptPinnedTargetsHost = nullptr;
        mKeptPinnedTargetsDev = nullptr;
    }

    // deformable bodies
    for (auto record : mDeformableBodyToRecord)
        DeformableBodyBufferManager::freeRecordBuffers(record.second);
    mDeformableBodyToRecord.clear();

    // synchronization events
    for (PxU32 i = 0; i < PxU32(CopyEvent::eCOUNT); i++)
    {
        if (mCopyEvents[i])
        {
            CHECK_CU(getCudaShim()->eventDestroy(reinterpret_cast<uintptr_t>(mCopyEvents[i]), nullptr));
        }
    }
    for (PxU32 i = 0; i < PxU32(ApplyEvent::eCOUNT); i++)
    {
        if (mApplyWaitEvents[i])
        {
            CHECK_CU(getCudaShim()->eventDestroy(reinterpret_cast<uintptr_t>(mApplyWaitEvents[i]), nullptr));
        }
        if (mApplySignalEvents[i])
        {
            CHECK_CU(getCudaShim()->eventDestroy(reinterpret_cast<uintptr_t>(mApplySignalEvents[i]), nullptr));
        }
    }
    for (PxU32 i = 0; i < PxU32(SharedDeviceBuffer::eCOUNT); i++)
    {
        if (mKernelDoneEvents[i])
        {
            CHECK_CU(getCudaShim()->eventDestroy(reinterpret_cast<uintptr_t>(mKernelDoneEvents[i]), nullptr));
        }
    }
    if (mContactReadEvent)
    {
        CHECK_CU(getCudaShim()->eventDestroy(reinterpret_cast<uintptr_t>(mContactReadEvent), nullptr));
    }
    for (const std::vector<CUevent>* events : { &mTableReaders, &mIdleTableReaders })
    {
        for (CUevent event : *events)
            CHECK_CU(getCudaShim()->eventDestroy(reinterpret_cast<uintptr_t>(event), nullptr));
    }
}

bool GpuSimulationData::checkApiReady(const char* function) const
{
    if (mBackend.getStepCount(mAttachHandle) < 1)
    {
        CARB_LOG_ERROR("GPU tensor function %s can only be called after at least one simulation step was performed!", function);
        return false;
    }
    return true;
}

bool GpuSimulationData::checkArticulationStrides(const char* function) const
{
    // Every DirectGPU articulation buffer is laid out with these strides, so they must be PhysX's. They differ between
    // the insertion of an articulation larger than any PhysX has simulated and the next step, when PhysX learns it
    // (data built in that window keeps the larger live counts), and for data built before such an insertion once that
    // step has raised PhysX's counts past it.
    if (mNumArtis > 0 && mScene && mScene->getTimestamp() > 1)
    {
        const PxArticulationGPUAPIMaxCounts counts = mScene->getDirectGPUAPI().getArticulationGPUAPIMaxCounts();
        if (counts.maxLinks != mMaxLinks || counts.maxDofs != mMaxDofs || counts.maxFixedTendons != mMaxFixedTendons ||
            counts.maxSpatialTendons != mMaxSpatialTendons)
        {
            // PhysX above the data is permanent for this data: only new data (a new simulation view) has its strides.
            // The data above PhysX is the insertion window, which the next step closes -- unless the inserted
            // articulation is removed before it is simulated, so the remedy names both.
            const bool physxLarger = counts.maxLinks > mMaxLinks || counts.maxDofs > mMaxDofs ||
                                     counts.maxFixedTendons > mMaxFixedTendons ||
                                     counts.maxSpatialTendons > mMaxSpatialTendons;
            CARB_LOG_ERROR("GPU tensor function %s: the articulation strides of this simulation view's data (links %u, "
                           "DOFs %u, fixed tendons %u, spatial tendons %u) do not match PhysX's (links %u, DOFs %u, "
                           "fixed tendons %u, spatial tendons %u). %s",
                           function, mMaxLinks, mMaxDofs, mMaxFixedTendons, mMaxSpatialTendons, counts.maxLinks,
                           counts.maxDofs, counts.maxFixedTendons, counts.maxSpatialTendons,
                           physxLarger ? "PhysX simulated a larger articulation after the simulation view was created: "
                                         "create a new simulation view and recreate the views from it." :
                                         "An articulation larger than any simulated was inserted and not simulated "
                                         "yet: step the simulation once; if this persists, create a new simulation "
                                         "view and recreate the views from it.");
            return false;
        }
    }
    return true;
}

bool GpuSimulationData::describesArticulation(const PxArticulationReducedCoordinate& arti) const
{
    // init() records every articulation of the scene under its root link's island node, with its GPU index.
    PxArticulationLink* root = nullptr;
    if (arti.getLinks(&root, 1) != 1 || !root)
        return false;
    const std::unordered_map<uint32_t, PxArticulationGPUIndex>::const_iterator it =
        mNodeIdx2ArtiIdxMap.find(root->getInternalIslandNodeIndex().index());
    return it != mNodeIdx2ArtiIdxMap.end() && it->second == arti.getGPUIndex();
}

void GpuSimulationData::clearForces()
{
    if (mRdForcesApplied)
    {
        CHECK_CUDA(cudaMemset(mRdForcesDev, 0, mNumRds * sizeof(PxVec3)));
        // make sure the cleared forces get applied on the next flush
        mRdForcesApplied = false;
    }

    if (mRdTorquesApplied)
    {
        CHECK_CUDA(cudaMemset(mRdTorquesDev, 0, mNumRds * sizeof(PxVec3)));
        mRdTorquesApplied = false;
    }

    if (mLinkForcesApplied)
    {
        CHECK_CUDA(cudaMemset(mLinkForcesDev, 0, mLinkBufSize * sizeof(PxVec3)));
        mLinkForcesApplied = false;
    }

    if (mLinkTorquesApplied)
    {
        CHECK_CUDA(cudaMemset(mLinkTorquesDev, 0, mLinkBufSize * sizeof(PxVec3)));
        mLinkTorquesApplied = false;
    }

    if (mArtiDofForcesApplied)
    {
        CHECK_CUDA(cudaMemset(mDofActuationForcesDev, 0,  mDofBufSize * sizeof(float)));
        mArtiDofForcesApplied = false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));
}


bool GpuSimulationData::flush()
{
    // nothing to do anymore
    return true;
}

void GpuSimulationData::enableGpuUsageWarnings(bool enable)
{
    mGenerateWarning = enable;
}


void GpuSimulationData::updateContactReports()
{
    // avoid processing contacts multiple times during a step
    int64_t timestamp = mBackend.getTimestamp(mAttachHandle);
    if (mRigidContactTimestamp >= timestamp)
    {
        return;
    }
    mRigidContactTimestamp = timestamp;

    PhysxCudaContextGuard ctxGuarg(mCudaContextManager);

    // copyContactData writes nothing for a step without contact pairs, which would keep the previous step's pairs.
    // The count is cleared first, and the clear finishes before PhysX's own stream can write it.
    if (!CHECK_CUDA(cudaMemsetAsync(mGpuContactPairCountDev, 0, sizeof(PxU32), nullptr)) ||
        !CHECK_CUDA(cudaStreamSynchronize(nullptr)))
    {
        mNumContactPairs = 0;
        return;
    }

    mScene->getDirectGPUAPI().copyContactData(mGpuContactPairsDev, mGpuContactPairCountDev, mMaxGpuContactPairs, NULL, mContactReadEvent);
    CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(mContactReadEvent), 0, nullptr));

    //dumpGpuContactData(mGpuContactPairsDev, mGpuContactPairCountDev);

    CHECK_CUDA(cudaMemcpy(&mNumContactPairs, mGpuContactPairCountDev, sizeof(PxU32), cudaMemcpyDeviceToHost));

}


void GpuSimulationData::dumpGpuContactData(PxGpuContactPair* contactPairsDev, PxU32* numContactPairsDev)
{
    printf("-----\n");

    PxU32 numContactPairs = 0;
    CHECK_CUDA(cudaMemcpy(&numContactPairs, numContactPairsDev, sizeof(numContactPairs), cudaMemcpyDeviceToHost));
    printf("Got %u contact pairs\n", numContactPairs);

    if (numContactPairs > mMaxGpuContactPairs)
    {
        CARB_LOG_WARN("Num contact pairs (%u) exeeded max contact pairs (%u), some contacts will be lost.",
                        numContactPairs, mMaxGpuContactPairs);
        // TODO: CARB_LOG_WARN("Increase max contact pairs in sim params.");
        numContactPairs = mMaxGpuContactPairs;
    }

    if (numContactPairs > 0)
    {
        std::vector<PxGpuContactPair> cpairs(numContactPairs);
        CHECK_CUDA(cudaMemcpy(
            cpairs.data(), contactPairsDev, numContactPairs * sizeof(*contactPairsDev), cudaMemcpyDeviceToHost));

        // find the extents of the patch, contact, and force arrays
        PxU8* patchPtrBase = (PxU8*)std::numeric_limits<size_t>::max();
        PxU8* patchPtrEnd = nullptr;
        PxU8* contactPtrBase = (PxU8*)std::numeric_limits<size_t>::max();
        PxU8* contactPtrEnd = nullptr;
        PxReal* forcePtrBase = (PxReal*)std::numeric_limits<size_t>::max();
        PxReal* forcePtrEnd = nullptr;

        for (PxU32 i = 0; i < numContactPairs; i++)
        {
            auto& cp = cpairs[i];
            printf("CP %u:\n", i);
            printf("  Node0:\n");
            printf("    Index:       %u\n", cp.nodeIndex0.index());
            printf("    Static?      %u\n", cp.nodeIndex0.isStaticBody());
            printf("    Arti?        %u\n", cp.nodeIndex0.isArticulation());
            printf("    Link index:  %u\n", cp.nodeIndex0.articulationLinkId());
            printf("    Shape ref:   %u\n", cp.transformCacheRef0);
            printf("  Node1:\n");
            printf("    Index:       %u\n", cp.nodeIndex1.index());
            printf("    Static?      %u\n", cp.nodeIndex1.isStaticBody());
            printf("    Arti?        %u\n", cp.nodeIndex1.isArticulation());
            printf("    Link index:  %u\n", cp.nodeIndex1.articulationLinkId());
            printf("    Shape ref:   %u\n", cp.transformCacheRef1);
            printf("  NumContacts: %u\n", cp.nbContacts);
            printf("  NumPatches:  %u\n", cp.nbPatches);


            printf("  Forces ptr:   %p\n", cp.contactForces);
            if (cp.contactForces < forcePtrBase)
            {
                forcePtrBase = cp.contactForces;
            }
            if (cp.contactForces + 1 > forcePtrEnd)
            {
                forcePtrEnd = cp.contactForces + 1;
            }

            printf("  Patches ptr:  %p\n", cp.contactPatches);
            if (cp.contactPatches < patchPtrBase)
            {
                patchPtrBase = cp.contactPatches;
            }
            if (cp.contactPatches + cp.nbPatches * sizeof(PxContactPatch) > patchPtrEnd)
            {
                patchPtrEnd = cp.contactPatches + cp.nbPatches * sizeof(PxContactPatch);
            }

            printf("  Contact ptr: %p\n", cp.contactPoints);
            if (cp.contactPoints < contactPtrBase)
            {
                contactPtrBase = cp.contactPoints;
            }
            if (cp.contactPoints + cp.nbContacts * sizeof(PxContact) > contactPtrEnd)
            {
                contactPtrEnd = cp.contactPoints + cp.nbContacts * sizeof(PxContact);
            }
        }

        PxU32 numForces = PxU32(forcePtrEnd - forcePtrBase);
        PxU32 numPatches = PxU32((PxContactPatch*)patchPtrEnd - (PxContactPatch*)patchPtrBase);
        PxU32 numContacts = PxU32((PxContact*)contactPtrEnd - (PxContact*)contactPtrBase);
        printf("Num forces:   %u\n", numForces);
        printf("Num patches:  %u\n", numPatches);
        printf("Num contacts: %u\n", numContacts);

        printf("Forces ptr:   %p\n", forcePtrBase);
        printf("Patches ptr:  %p\n", patchPtrBase);
        printf("Contacts ptr: %p\n", contactPtrBase);

        std::vector<PxReal> forces(numForces);
        CHECK_CUDA(cudaMemcpy(forces.data(), forcePtrBase, numForces * sizeof(PxReal), cudaMemcpyDeviceToHost));

        std::vector<PxContactPatch> patches(numPatches);
        memset(patches.data(), 0, numPatches * sizeof(PxContactPatch));
        CHECK_CUDA(
            cudaMemcpy(patches.data(), patchPtrBase, numPatches * sizeof(PxContactPatch), cudaMemcpyDeviceToHost));

        std::vector<PxContact> contacts(numContacts);
        memset(contacts.data(), 0, numContacts * sizeof(PxContact));
        CHECK_CUDA(
            cudaMemcpy(contacts.data(), contactPtrBase, numContacts * sizeof(PxContact), cudaMemcpyDeviceToHost));

        printf("Forces:");
        for (PxU32 i = 0; i < numForces; i++)
        {
            printf(" %.3f", forces[i]);
        }
        printf("\n");

        printf("Patches:\n");
        for (PxU32 i = 0; i < numPatches; i++)
        {
            auto& patch = patches[i];
            printf("  Patch %u\n", i);
            printf("    Num contacts: %u\n", patch.nbContacts);
            printf("    Contact idx:  %u\n", patch.startContactIndex);
            printf("    Normal:       (%f, %f, %f)\n", patch.normal.x, patch.normal.y, patch.normal.z);
        }

        printf("Contacts:\n");
        for (PxU32 i = 0; i < numContacts; i++)
        {
            auto& contact = contacts[i];
            printf("  Contact %u\n", i);
            printf("    Point:       (%f, %f, %f)\n", contact.contact.x, contact.contact.y, contact.contact.z);
            printf("    Separation:  %f\n", contact.separation);
        }
    }
}


bool DeformableBodyBufferManager::allocDevMemAndCopyH2D(void** dst, const void* src, size_t size, const char* name)
{
    if (!CHECK_CUDA(cudaMalloc(dst, size)))
    {
        CARB_LOG_ERROR("Unable to allocate memory of size %u for %s", (::physx::PxU32)size, name);
        cudaGetLastError(); // reported through the result, as prepareDeviceData does
        return false;
    }
    if (!CHECK_CUDA(cudaMemcpy(*dst, src, size, cudaMemcpyHostToDevice)))
    {
        CARB_LOG_ERROR("Unable to copy data from host to device for %s", name);
        cudaGetLastError();
        return false;
    }
    return true;
}

void DeformableBodyBufferManager::freeRecordBuffers(GpuDeformableBodyRecord& record)
{
    for (void** buffer : { (void**)&record.simElementIndices, (void**)&record.simNodalKinematicTargets,
                           (void**)&record.restNodalPositions, (void**)&record.collElementIndices })
    {
        if (*buffer)
            CHECK_CUDA(cudaFree(*buffer));
        *buffer = nullptr;
    }
}

void DeformableBodyBufferManager::resetDeviceMem()
{
    for (size_t i = 0; i < buffersH.size(); i++)
    {
        if (!CHECK_CUDA(cudaMemset(buffersH[i], 0, bufferSizesH[i])))
        {
            return;
        }
    }
}

void DeformableBodyBufferManager::setMaxBufferSize()
{
    maxBufferSize = *std::max_element(bufferSizesH.begin(), bufferSizesH.end());
}

bool DeformableBodyBufferManager::uploadDeviceData(std::string debugMessage)
{
    if (buffersD)
        CHECK_CUDA(cudaFree(buffersD));
    buffersD = nullptr;
    if (!allocDevMemAndCopyH2D(
            (void**)&buffersD, buffersH.data(), sizeof(void*) * buffersH.size(), (debugMessage + " buffer").c_str()))
        return false;

    if (bufferSizesD)
        CHECK_CUDA(cudaFree(bufferSizesD));
    bufferSizesD = nullptr;
    return allocDevMemAndCopyH2D((void**)&bufferSizesD, bufferSizesH.data(),
                                 sizeof(::physx::PxU32) * bufferSizesH.size(), (debugMessage + " sizes").c_str());
}

void DeformableBodyBufferManager::releaseDeviceMem()
{
    CHECK_CUDA(cudaFree(buffersD));
    CHECK_CUDA(cudaFree(bufferSizesD));
}

}
}
}

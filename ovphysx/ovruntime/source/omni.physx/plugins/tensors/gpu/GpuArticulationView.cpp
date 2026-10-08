// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-INPUT-CORE-001
 * @covers AC-3 AC-4
 *
 * @implements REQ-READ-CORE-001
 * @covers AC-6
 *
 * @implements REQ-READ-TENDON-001
 * @covers AC-4
 *
 * @implements REQ-TENSOR-PATH-001
 * @covers AC-4
 *
 * @implements REQ-TENSOR-INDEX-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-TENSOR-CPU-ONLY-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-READ-ARTICULATION-001
 * @covers AC-3 AC-4 AC-9
 *
 * @implements REQ-READ-ATTRS-001
 * @covers AC-15, AC-17
 *
 * @implements REQ-READ-INVDYN-001
 * @covers AC-1, AC-8, AC-10
 *
 * @implements REQ-INPUT-COVERAGE-001
 * @covers AC-11
 *
 * @implements REQ-INPUT-DEVICE-001
 * @covers AC-1 AC-2
 */

#include "tensors/gpu/CudaKernels.h"
#include "tensors/gpu/GpuArticulationView.h"
#include "tensors/gpu/GpuSimulationView.h"

#include "tensors/GlobalsAreBad.h"
#include "tensors/CommonTypes.h"
#include "tensors/SimulationBackend.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

#include <omni/physics/tensors/TensorUtils.h>

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

GpuArticulationView::GpuArticulationView(GpuSimulationView* sim, const std::vector<ArticulationEntry>& entries, int device)
    : BaseArticulationView(sim, entries), mDevice(device)
{
    if (sim)
    {
        mGpuSimData = sim->getGpuSimulationData();
    }

    PxU32 numArtis = getCount();
    mLinkBufSize = numArtis * mMaxLinks;
    mDofBufSize = numArtis * mMaxDofs;
    mFixedTendonBufSize = numArtis * mMaxFixedTendons;
    mSpatialTendonBufSize = numArtis * mMaxSpatialTendons;

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    if (!SHIM_CU_EVENT_CREATE(&mOvStageSelectionReadyEvent, CU_EVENT_DISABLE_TIMING))
    {
        CARB_LOG_ERROR("Failed to create the ovstage articulation selection ready event");
    }

    // physx arti indices
    mArtiIndices.resize(numArtis);

    // articulation view indices
    std::vector<::physx::PxU32> mViewIndices;
    mViewIndices.resize(numArtis);

    for (PxU32 i = 0; i < numArtis; i++)
    {
        mArtiIndices[i] = mEntries[i].arti->getGPUIndex();
        mViewIndices[i] = i;
    }
    prepareDeviceData((void**)&mArtiGpuIndicesDev, (void*)mArtiIndices.data(),
                      numArtis * sizeof(PxArticulationGPUIndex), "mArtiGpuIndicesDev");

    prepareDeviceData((void**)&mViewIndicesDev, (void*)mViewIndices.data(), numArtis * sizeof(PxU32), "mViewIndicesDev");

    prepareDeviceData((void**)&mDirtyArtiGpuIndicesDev, nullptr, numArtis * sizeof(PxArticulationGPUIndex),
                      "mDirtyArtiGpuIndicesDev");


    // root data
    std::vector<GpuArticulationRootRecord> rootRecords(numArtis);
    for (PxU32 i = 0; i < numArtis; i++)
    {
        rootRecords[i].physxArtiIdx = mArtiIndices[i];
        if (mEntries[i].subspace)
        {
            const carb::Float3& o = mEntries[i].subspace->origin;
            rootRecords[i].origin = { o.x, o.y, o.z };
        }
        else
        {
            rootRecords[i].origin = { 0.0f, 0.0f, 0.0f };
        }
    }
    prepareDeviceData((void**)&mRootRecordsDev, (void*)rootRecords.data(), numArtis * sizeof(GpuArticulationRootRecord),
                      "mRootRecordsDev");
    // DOF data
    std::vector<GpuArticulationDofRecord> dofRecords(mDofBufSize);
    for (PxU32 i = 0; i < numArtis; i++)
    {
        for (PxU32 j = 0; j < mMaxDofs; j++)
        {
            GpuArticulationDofRecord& data = dofRecords[i * mMaxDofs + j];
            data.physxArtiIdx = mArtiIndices[i];
            data.physxDofIdx = data.physxArtiIdx * mGpuSimData->mMaxDofs + j;
            data.body0IsParent =
                (j < mEntries[i].numDofs) ? mEntries[i].metatype->isDofBody0Parent(j) : true;
        }
    }
    if (mMaxDofs > 0)
    {
        prepareDeviceData((void**)&mDofRecordsDev, (void*)dofRecords.data(),
                          mDofBufSize * sizeof(GpuArticulationDofRecord), "mDofRecordsDev");
    }
    // link data
    // NOTE: mGpuSimData->mMaxLinks is the global scene maxLink,
    // while mMaxLinks defined in the base class is the max links of the view class
    // similarly for mGpuSimData->mMaxDofs vs mMaxDofs. physx indices are w.r.t to global maximums
    std::vector<GpuArticulationLinkRecord> linkData(mLinkBufSize);
    for (PxU32 i = 0; i < numArtis; i++)
    {
        PxU32 artiIdx = mArtiIndices[i];
        PxVec3 origin;
        if (mEntries[i].subspace)
        {
            const carb::Float3& o = mEntries[i].subspace->origin;
            origin = { o.x, o.y, o.z };
        }
        else
        {
            origin = { 0.0f, 0.0f, 0.0f };
        }

        for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
        {
            GpuArticulationLinkRecord& data = linkData[i * mMaxLinks + j];
            data.physxLinkIdx = artiIdx * mGpuSimData->mMaxLinks + j;
            data.physxArtiIdx = artiIdx;
            data.origin = origin;

            data.physxToUsdJointRotation = mEntries[i].incomingJointPhysxToUsdRotations[j];
            data.dofOffset = i * mMaxDofs + mEntries[i].dofStarts[j];
            data.D6RotationAxes = mEntries[i].freeD6Axes[j];
            PxArticulationJointReducedCoordinate* joint = mEntries[i].links[j]->getInboundJoint();
            if (joint)
            {
                data.incomingJointType = joint->getJointType();
                data.body0IsParent = mEntries[i].isIncomingJointBody0Parent[j];
                data.jointChild = mEntries[i].jointChild[j];
                data.jointParent = mEntries[i].jointParent[j];
                if (mEntries[i].parentIndices[j] != 0xffffffff)
                    // TODO: double check this, since it is used to get an index in the link transform buffer
                    data.incomingLinkIdx = i * mGpuSimData->mMaxLinks + mEntries[i].parentIndices[j];  
                else
                    data.incomingLinkIdx = 0xffffffff;
            }
        }
    }

    // articulation links center of mass
    prepareDeviceData((void**)&cMassLocalPosePosDev, nullptr, mLinkBufSize * sizeof(PxVec3), "cMassLocalPosePosDev");

    cMassLocalPosePos.resize(mLinkBufSize, { 0.0f, 0.0f, 0.0f });
    updateCMassData();

    prepareDeviceData((void**)&mLinkRecordsDev, (void*)linkData.data(),
                      mLinkBufSize * sizeof(GpuArticulationLinkRecord), "mLinkRecordsDev");
    // fixed tendons
    if (mMaxFixedTendons > 0)
    {
        std::vector<GpuArticulationFixedTendonRecord> tendonRecords(numArtis * mMaxFixedTendons);
        for (PxU32 i = 0; i < numArtis; i++)
        {
            PxU32 artiIdx = mArtiIndices[i];
            for (PxU32 j = 0; j < mMaxFixedTendons; j++)
            {
                GpuArticulationFixedTendonRecord& data = tendonRecords[i * mMaxFixedTendons + j];
                data.physxTendonIdx = artiIdx * mGpuSimData->mMaxFixedTendons + j;
                data.physxArtiIdx = artiIdx;
            }
        }
        prepareDeviceData((void**)&mFixedTendonRecordsDev, (void*)tendonRecords.data(),
                          mFixedTendonBufSize * sizeof(GpuArticulationFixedTendonRecord), "mFixedTendonRecordsDev");
    }

    // spatial tendons
    if (mMaxSpatialTendons > 0)
    {
        std::vector<GpuArticulationSpatialTendonRecord> spatialTendonRecords(numArtis * mMaxSpatialTendons);
        for (PxU32 i = 0; i < numArtis; i++)
        {
            PxU32 artiIdx = mArtiIndices[i];
            for (PxU32 j = 0; j < mMaxSpatialTendons; j++)
            {
                GpuArticulationSpatialTendonRecord& data = spatialTendonRecords[i * mMaxSpatialTendons + j];
                data.physxTendonIdx = artiIdx * mGpuSimData->mMaxSpatialTendons + j;
                data.physxArtiIdx = artiIdx;
            }
        }
        prepareDeviceData((void**)&mSpatialTendonRecordsDev, (void*)spatialTendonRecords.data(),
                          mSpatialTendonBufSize * sizeof(GpuArticulationSpatialTendonRecord), "mSpatialTendonRecordsDev");
    }

    VALIDATE_CUDA_CONTEXT();
}

GpuArticulationView::~GpuArticulationView()
{
    if (mGpuSimData)
    {
        CudaContextGuard ctxGuard(mGpuSimData->mCtx);

        // Drain ONCE, before any free below. Every ovstage gather this view launches runs on the
        // null stream and returns without synchronizing, so a view destroyed in the same frame as
        // its last read can free memory a kernel is still reading. cudaFree has historically
        // synchronized implicitly; that is not a guarantee to rely on.
        //
        // Unconditional and hoisted, not per buffer. The drain used to sit inside the
        // `mOvStageSelectionDev` branch, which covered one allocation and left the rest exposed --
        // and the rest are not incidental: mLinkRecordsDev, mDofRecordsDev, mArtiGpuIndicesDev and
        // the private gather scratch are all *inputs* to those same kernels.
        // mLinkIncomingJointForceScratchDev is the sharpest case, because being privately owned is
        // exactly why nothing else protects it: it is handed back through no shared-buffer protocol,
        // so this drain is the only thing between the free and a live kernel.
        //
        // Same shape as GpuPointSetReadView::~GpuPointSetReadView, which drains once for the same
        // reason.
        CHECK_CUDA(cudaStreamSynchronize(nullptr));

        if (mOvStageSelectionDev)
        {
            CHECK_CUDA(cudaFree(mOvStageSelectionDev));
        }
        if (mOvStageSelectionReadyEvent)
        {
            CHECK_CU(getCudaShim()->eventDestroy(reinterpret_cast<uintptr_t>(mOvStageSelectionReadyEvent), nullptr));
        }

        CHECK_CUDA(cudaFree(mArtiGpuIndicesDev));
        CHECK_CUDA(cudaFree(mViewIndicesDev));
        CHECK_CUDA(cudaFree(mDofRecordsDev));
        CHECK_CUDA(cudaFree(mOvStageDofRecordsDev));
        CHECK_CUDA(cudaFree(mOvStageFixedTendonRecordsDev));
        CHECK_CUDA(cudaFree(mOvStageSpatialTendonRecordsDev));
        CHECK_CUDA(cudaFree(mOvStageLinkForceRecordsDev));
        CHECK_CUDA(cudaFree(mRootRecordsDev));
        CHECK_CUDA(cudaFree(mLinkRecordsDev));
        CHECK_CUDA(cudaFree(mFixedTendonRecordsDev));
        CHECK_CUDA(cudaFree(mSpatialTendonRecordsDev));
        CHECK_CUDA(cudaFree(mDirtyArtiGpuIndicesDev));
        CHECK_CUDA(cudaFree(mOvStageRowsDev));
        CHECK_CUDA(cudaFree(cMassLocalPosePosDev));
        if (mMaskIndicesDev)
            CHECK_CUDA(cudaFree(mMaskIndicesDev));
        if (mMaskAllocPolicy.mBuffer)
            CHECK_CUDA(cudaFree(mMaskAllocPolicy.mBuffer));
        if (mLinkIncomingJointForceScratchDev)
            CHECK_CUDA(cudaFree(mLinkIncomingJointForceScratchDev));
    }
}

bool GpuArticulationView::getLinkTransforms(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "link transform", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "link transform", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxLinks * 7u, "link transform", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    SYNCHRONIZE_CUDA();

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiLinkTransforms];

    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)mGpuSimData->mLinkOrRootTransformsDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, numArtis,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eLinkOrRootTransforms), copyEvent),
            copyEvent, "articulation link transforms"))
    {
        return false;
    }

    SYNCHRONIZE_CUDA();

    // Our kernel is done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7).
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eLinkOrRootTransforms, [&] {
            return fetchArtiLinkTransforms(static_cast<TensorTransform*>(dstTensor->data),
                                           mGpuSimData->mLinkOrRootTransformsDev, numArtis * mMaxLinks, mMaxLinks,
                                           mGpuSimData->mMaxLinks, mLinkRecordsDev);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch articulation link transforms");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

static bool getVelAcc(const TensorDesc* dstTensor,
                      PxVec3* linkDataLinearDev,
                      PxVec3* linkDataAngularDev,
                      const PxU32* defaultArtiIndicesDev,
                      const PxArticulationGPUIndex* artiGpuIndicesDev,
                      const PxArticulationGPUAPIReadType::Enum linkLinearType,
                      const PxArticulationGPUAPIReadType::Enum linkAngularType,
                      const PxU32 numArtis,
                      const PxU32 maxLinks,
                      const int device,
                      const char* tensorName,
                      const char* funcName,
                      GpuSimulationDataPtr gpuSimData)
{
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, device, tensorName, funcName) ||
        !checkTensorFloat32(*dstTensor, tensorName, funcName) ||
        !checkTensorSizeExact(*dstTensor, numArtis * maxLinks * 6u, tensorName, funcName))
    {
        return false;
    }

    PxScene* scene = gpuSimData->mScene;
    PhysxCudaContextGuard ctxGuarg(gpuSimData->mCudaContextManager);

    SYNCHRONIZE_CUDA();

    CUevent artiCopyEventLin = gpuSimData->mCopyEvents[CopyEvent::eArtiLinkLinearVelocities];
    CUevent artiCopyEventAng = gpuSimData->mCopyEvents[CopyEvent::eArtiLinkAngularVelocities];

    // Each fetch is checked against its OWN finish event; pairing one with the other's would order
    // the gather behind the wrong copy.
    if (!gpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)linkDataLinearDev, artiGpuIndicesDev, linkLinearType, numArtis,
                gpuSimData->kernelDoneEvent(SharedDeviceBuffer::eLinkOrRootLinearVel), artiCopyEventLin),
            artiCopyEventLin, __FUNCTION__))
    {
        return false;
    }
    if (!gpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)linkDataAngularDev, artiGpuIndicesDev, linkAngularType, numArtis,
                gpuSimData->kernelDoneEvent(SharedDeviceBuffer::eLinkOrRootAngularVel), artiCopyEventAng),
            artiCopyEventAng, __FUNCTION__))
    {
        return false;
    }

    SYNCHRONIZE_CUDA();

    // Our kernel is done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded before the branch, because the gather can fail AFTER launching
    // and returning first would leave the next writer free to overwrite a buffer still being read.
    if (!gpuSimData->gatherThenRelease(
            SharedDeviceBuffer::eLinkOrRootLinearVel, SharedDeviceBuffer::eLinkOrRootAngularVel, [&] {
                return fetchArtiLinkVelocitiesAccelerations(static_cast<TensorVelAcc*>(dstTensor->data),
                                                            linkDataLinearDev, linkDataAngularDev, numArtis * maxLinks,
                                                            maxLinks, gpuSimData->mMaxLinks);
            }))
    {
        CARB_LOG_ERROR("Failed to fetch articulation link velocities or accelerations");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

bool GpuArticulationView::getLinkVelocities(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    return getVelAcc(dstTensor,  mGpuSimData->mLinkOrRootLinearVelAccDev,  mGpuSimData->mLinkOrRootAngularVelAccDev, mViewIndicesDev,
                     mArtiGpuIndicesDev, PxArticulationGPUAPIReadType::Enum::eLINK_LINEAR_VELOCITY,
                     PxArticulationGPUAPIReadType::Enum::eLINK_ANGULAR_VELOCITY, getCount(), mMaxLinks, mDevice,
                     "link velocity", __FUNCTION__, mGpuSimData);
}

bool GpuArticulationView::getLinkAccelerations(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    return getVelAcc(dstTensor,  mGpuSimData->mLinkOrRootLinearVelAccDev,  mGpuSimData->mLinkOrRootAngularVelAccDev, mViewIndicesDev,
                     mArtiGpuIndicesDev, PxArticulationGPUAPIReadType::Enum::eLINK_LINEAR_ACCELERATION,
                     PxArticulationGPUAPIReadType::Enum::eLINK_ANGULAR_ACCELERATION, getCount(), mMaxLinks, mDevice,
                     "link acceleration", __FUNCTION__, mGpuSimData);
}

bool GpuArticulationView::getRootTransforms(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "root transform", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "root transform", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 7u, "root transform", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    SYNCHRONIZE_CUDA();

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiRootTransforms];
    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)mGpuSimData->mLinkOrRootTransformsDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIReadType::eROOT_GLOBAL_POSE, numArtis,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eLinkOrRootTransforms), copyEvent),
            copyEvent, __FUNCTION__))
    {
        return false;
    }

    SYNCHRONIZE_CUDA();

    // Our kernel is done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded unconditionally: every kernel that touches a buffer records it.
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eLinkOrRootTransforms, [&] {
            return fetchArtiRootTransforms(static_cast<TensorTransform*>(dstTensor->data),
                                           mGpuSimData->mLinkOrRootTransformsDev, numArtis, mRootRecordsDev);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch articulation root tranforms");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    SYNCHRONIZE_CUDA();

    return true;
}

bool GpuArticulationView::setRootTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, mDevice, "root transform", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "root transform", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * 7u, "root transform", __FUNCTION__))
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
        indices = mViewIndicesDev;
        numIndices = getCount();
    }

    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    CHECK_CUDA(cudaMemset(mDirtyArtiGpuIndicesDev, 0, getCount() * sizeof(PxArticulationGPUIndex)));
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    SYNCHRONIZE_CUDA();
    if (!submitArtiRootTransforms( mGpuSimData->mLinkOrRootTransformsDev, static_cast<const TensorTransform*>(srcTensor->data),
                                  indices, mDirtyArtiGpuIndicesDev, numIndices, getCount(), mRootRecordsDev))
    {
        CARB_LOG_ERROR("Failed to submit articulation root transforms");
        return false;
    }

    // CHECK_CUDA(cudaStreamSynchronize(nullptr));
    CHECK_CU(getCudaShim()->eventRecord(reinterpret_cast<uintptr_t>(mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiRootTransforms]), uintptr_t(0), nullptr));
    scene->getDirectGPUAPI().setArticulationData((void*) mGpuSimData->mLinkOrRootTransformsDev, mDirtyArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIWriteType::eROOT_GLOBAL_POSE, numIndices,
                                                 mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiRootTransforms],
                                                 mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootTransforms]);
    CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootTransforms]), nullptr));
    SYNCHRONIZE_CUDA();

    return true;
}

bool GpuArticulationView::getRootVelocities(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "root velocity", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "root velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 6u, "root velocity", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    SYNCHRONIZE_CUDA();

    CUevent artiCopyEventLin = nullptr;
    CUevent artiCopyEventAng = nullptr;

    // Use link buffers to avoid more memory usage.
    //
    // Both finish events are deliberately null, so PhysX records nothing and there is no producer to
    // drain -- the SYNCHRONIZE_CUDA below is what orders the gather. Only the refusal status is
    // actionable: PhysX writes NOTHING when it refuses, and the scratch is not zeroed, so an
    // unchecked refusal is gathered as whatever it last held (ADR-0008 Decision 10).
    if (!scene->getDirectGPUAPI().getArticulationData(
            (void*)mGpuSimData->mLinkOrRootLinearVelAccDev, mArtiGpuIndicesDev,
            PxArticulationGPUAPIReadType::eROOT_LINEAR_VELOCITY, numArtis,
            mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eLinkOrRootLinearVel), artiCopyEventLin))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI refused the root linear velocity read", __FUNCTION__);
        return false;
    }
    if (!scene->getDirectGPUAPI().getArticulationData(
            (void*)mGpuSimData->mLinkOrRootAngularVelAccDev, mArtiGpuIndicesDev,
            PxArticulationGPUAPIReadType::eROOT_ANGULAR_VELOCITY, numArtis,
            mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eLinkOrRootAngularVel), artiCopyEventAng))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI refused the root angular velocity read", __FUNCTION__);
        return false;
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

    // Our kernel is done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded unconditionally: every kernel that touches a buffer records it.
    if (!mGpuSimData->gatherThenRelease(
            SharedDeviceBuffer::eLinkOrRootLinearVel, SharedDeviceBuffer::eLinkOrRootAngularVel, [&] {
                return fetchArtiRootVelocities(static_cast<TensorVelAcc*>(dstTensor->data),
                                               mGpuSimData->mLinkOrRootLinearVelAccDev,
                                               mGpuSimData->mLinkOrRootAngularVelAccDev, numArtis);
            }))
    {
        CARB_LOG_ERROR("Failed to fetch articulation root velocities");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

bool GpuArticulationView::ensureOvStageSelection(const PxU32* rows,
                                                 PxU32 count,
                                                 uint64_t rowsToken,
                                                 const PxArticulationGPUIndex*& gpuIndicesDev) const
{
    // Bounds are checked on the miss, not the hit: a hit means this exact list was validated when it
    // was uploaded. That rests on the token contract in the header -- if a token were ever reused for
    // a different list, this is where the check would be skipped.
    const bool selectionChanged =
        !mOvStageSelectionValid || mOvStageSelectionCount != count || mOvStageSelectionToken != rowsToken;
    if (selectionChanged && !checkRecordIndices(rows, count, mEntries.size(), "ovstage articulation rows", __FUNCTION__))
    {
        return false;
    }

    if (count > mOvStageSelectionCapacity)
    {
        // Every consumer leaves its finish wait and gather on the null stream, so this one
        // structural synchronization makes the old allocation safe to free.
        if (mOvStageSelectionDev && !CHECK_CUDA(cudaStreamSynchronize(nullptr)))
        {
            return false;
        }
        if (mOvStageSelectionDev && !CHECK_CUDA(cudaFree(mOvStageSelectionDev)))
        {
            return false;
        }
        mOvStageSelectionDev = nullptr;
        mOvStageGpuIndicesDev = nullptr;
        mOvStageSelectionCapacity = 0;
        mOvStageSelectionValid = false;

        void* allocation = nullptr;
        if (!CHECK_CUDA(cudaMalloc(&allocation, size_t(count) * sizeof(PxArticulationGPUIndex))))
        {
            CARB_LOG_ERROR("Failed to allocate ovstage articulation selection for %u rows", count);
            return false;
        }
        mOvStageSelectionDev = allocation;
        mOvStageGpuIndicesDev = static_cast<PxArticulationGPUIndex*>(allocation);
        mOvStageSelectionCapacity = count;
    }

    if (selectionChanged)
    {
        // Synchronous, like GpuRigidBodyView::ovStageRowsDevice: an async copy would need its source
        // kept alive until the upload retired, and buys nothing because this copy and every consumer
        // of it are on the null stream. REQ-READ-DEVICE-001's "asynchronous" is about the RESULT
        // path; this is host-to-device. Not a cold path: under kOvxActive the producer mints a fresh
        // token every read, so this runs per read on that scope.
        mOvStageSelectionHost.resize(count);
        for (PxU32 i = 0; i < count; i++)
        {
            const PxU32 row = rows ? rows[i] : i;
            mOvStageSelectionHost[i] = mArtiIndices[row];
        }

        if (!CHECK_CUDA(cudaMemcpy(mOvStageGpuIndicesDev, mOvStageSelectionHost.data(),
                                   size_t(count) * sizeof(PxArticulationGPUIndex), cudaMemcpyHostToDevice)))
        {
            mOvStageSelectionValid = false;
            return false;
        }

        mOvStageSelectionCount = count;
        mOvStageSelectionToken = rowsToken;
        mOvStageSelectionValid = true;
    }

    gpuIndicesDev = mOvStageGpuIndicesDev;
    return true;
}

// Shared body for the two cached ovstage record uploads. Grows the buffer when the
// byte size exceeds capacity and re-uploads only when the token or count changes, exactly like
// ensureOvStageSelection above. No index validation: this is a behaviour-preserving move of the
// reader's former per-read memAlloc + memcpyHtoD, which validated nothing either. Called with the
// view's CUDA context already current -- the reader holds the scene's PxScopedCudaLock across the
// build -- so it takes no guard of its own, matching ensureOvStageSelection.
static const void* uploadOvStageRecordsCached(void*& dev, size_t& capBytes, uint32_t& storedCount,
                                              uint64_t& storedToken, const void* records, PxU32 count,
                                              size_t recordSize, uint64_t token, const char* label)
{
    if (!records || count == 0)
        return nullptr;
    const size_t bytes = size_t(count) * recordSize;
    if (bytes > capBytes)
    {
        // Every consumer leaves its gather on the null stream, so one structural synchronisation
        // makes the old allocation safe to free -- the same argument ensureOvStageSelection makes.
        if (dev && !CHECK_CUDA(cudaStreamSynchronize(nullptr)))
            return nullptr;
        if (dev && !CHECK_CUDA(cudaFree(dev)))
            return nullptr;
        dev = nullptr;
        capBytes = 0;
        storedToken = 0;
        if (!CHECK_CUDA(cudaMalloc(&dev, bytes)))
        {
            CARB_LOG_ERROR("Failed to allocate ovstage %s records (%zu bytes)", label, bytes);
            dev = nullptr;
            return nullptr;
        }
        capBytes = bytes;
    }

    // `token == 0` forces a re-upload: a fresh view holds storedToken 0, and the reader's own record
    // cache is likewise gated on generation != 0, so the two agree on "not yet built".
    const bool changed = !dev || storedToken != token || storedCount != count || token == 0;
    if (changed)
    {
        if (!CHECK_CUDA(cudaMemcpy(dev, records, bytes, cudaMemcpyHostToDevice)))
        {
            storedToken = 0; // a failed upload must not be mistaken for a held copy
            return nullptr;
        }
        storedToken = token;
        storedCount = count;
    }
    return dev;
}

const void* GpuArticulationView::ovStageDofRecordsDevice(const void* records, PxU32 count, uint64_t token) const
{
    return uploadOvStageRecordsCached(mOvStageDofRecordsDev, mOvStageDofRecordsCapBytes, mOvStageDofRecordsCount,
                                      mOvStageDofRecordsToken, records, count, sizeof(ArticulationDofOvStageRecord),
                                      token, "DOF");
}

const void* GpuArticulationView::ovStageTendonRecordsDevice(const void* records, PxU32 count, uint64_t token,
                                                           bool fixed) const
{
    // Fixed and spatial keep separate buffers: the record struct is identical and the counts can
    // match, so sharing one buffer aliased the two on an equal-count read.
    if (fixed)
        return uploadOvStageRecordsCached(mOvStageFixedTendonRecordsDev, mOvStageFixedTendonRecordsCapBytes,
                                          mOvStageFixedTendonRecordsCount, mOvStageFixedTendonRecordsToken, records,
                                          count, sizeof(ArticulationTendonOvStageRecord), token, "fixed tendon");
    return uploadOvStageRecordsCached(mOvStageSpatialTendonRecordsDev, mOvStageSpatialTendonRecordsCapBytes,
                                      mOvStageSpatialTendonRecordsCount, mOvStageSpatialTendonRecordsToken, records,
                                      count, sizeof(ArticulationTendonOvStageRecord), token, "spatial tendon");
}
const void* GpuArticulationView::ovStageLinkForceRecordsDevice(const void* records, PxU32 count, uint64_t token) const
{
    return uploadOvStageRecordsCached(mOvStageLinkForceRecordsDev, mOvStageLinkForceRecordsCapBytes,
                                      mOvStageLinkForceRecordsCount, mOvStageLinkForceRecordsToken, records, count,
                                      sizeof(ArticulationLinkOvStageRecord), token, "link force");
}

bool GpuArticulationView::recordOvStageReady(CUevent waitEvent) const
{
    if (!mOvStageSelectionReadyEvent)
    {
        CARB_LOG_ERROR("The ovstage articulation selection ready event is unavailable");
        return false;
    }
    if (waitEvent &&
        !CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(waitEvent), 0, nullptr)))
    {
        return false;
    }
    return CHECK_CU(
        getCudaShim()->eventRecord(reinterpret_cast<uintptr_t>(mOvStageSelectionReadyEvent), uintptr_t(0), nullptr));
}

bool GpuArticulationView::waitForDirectGpuFinish(CUevent finishEvent, const char* label) const
{
    return mGpuSimData->drainDirectGpuFinish(finishEvent, label);
}

// Every requested root-state column, with each DirectGPU fetch issued once. Three fetches serve the
// four columns: eROOT_GLOBAL_POSE fills both pose columns, while linear and angular velocity are
// distinct read types landing in distinct scratch buffers.
//
// Each fetch is followed immediately by the gathers that consume it, before the next fetch is
// issued: the scratch buffers are SHARED (SharedDeviceBuffer::*, guarded by kernelDoneEvent /
// recordKernelDone), so a fetch may not be held across another one.
bool GpuArticulationView::getRootStateColumnsOvStage(
    const RootStateColumn* columns, PxU32 numColumns, const PxU32* rows, PxU32 count, uint64_t rowsToken) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    // A zero-column or zero-row request is nothing to emit, not a failure -- as on the CPU path.
    if (numColumns == 0 || count == 0)
        return true; // nothing to emit -- not a failure
    if (!columns)
    {
        return false;
    }

    // An empty destination means an empty READ, not an empty column: every column in one call is
    // sized off the same `count`, so one empty implies all empty. Decided ONCE here rather than with
    // PASS_EMPTY_TENSOR in the loop below, which expands to a function-level `return (true)` and
    // would skip validation of every later column and all three fetches. A only-PARTLY empty set
    // falls through to that loop and is refused there.
    bool allEmpty = true;
    for (PxU32 c = 0; c < numColumns && allEmpty; ++c)
    {
        const TensorDesc* const dst = columns[c].dst;
        allEmpty = dst && !dst->data && getTensorTotalSize(*dst) == 0;
    }
    if (allEmpty)
    {
        return true;
    }

    uint32_t sources = 0;
    for (PxU32 c = 0; c < numColumns; ++c)
    {
        const TensorDesc* const dst = columns[c].dst;
        const char* const label = rootStateLabel(columns[c].quantity);
        if (!dst || !dst->data)
        {
            return false;
        }
        if (!checkTensorDevice(*dst, mDevice, label, __FUNCTION__) ||
            !checkTensorFloat32(*dst, label, __FUNCTION__) ||
            !checkTensorSizeExact(*dst, count * rootStateComponents(columns[c].quantity), label, __FUNCTION__))
        {
            return false;
        }
        sources |= rootStateSource(columns[c].quantity);
    }

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    const PxArticulationGPUIndex* gpuIndicesDev = nullptr;
    if (!ensureOvStageSelection(rows, count, rowsToken, gpuIndicesDev))
    {
        return false;
    }
    PxScene* const scene = mGpuSimData->mScene;

    // One source: fetch into its scratch, then gather each column that reads that scratch.
    const auto fetchAndGather = [&](uint32_t source, PxArticulationGPUAPIReadType::Enum readType, void* scratch,
                                    PxU32 buffer, PxU32 copyEvent, const char* label) -> bool
    {
        const CUevent priorKernel = mGpuSimData->kernelDoneEvent(buffer);
        const CUevent finishEvent = mGpuSimData->mCopyEvents[copyEvent];
        if (!priorKernel || !finishEvent || !recordOvStageReady(priorKernel))
        {
            CARB_LOG_ERROR("Missing synchronization event for %s", label);
            return false;
        }
        if (!scene->getDirectGPUAPI().getArticulationData(
                scratch, gpuIndicesDev, readType, count, mOvStageSelectionReadyEvent, finishEvent))
        {
            CARB_LOG_ERROR("Failed to fetch %s", label);
            waitForDirectGpuFinish(finishEvent, label);
            return false;
        }
        if (!waitForDirectGpuFinish(finishEvent, label))
        {
            return false;
        }

        bool gathered = true;
        for (PxU32 c = 0; c < numColumns && gathered; ++c)
        {
            if (rootStateSource(columns[c].quantity) != source)
                continue;
            float* const dst = static_cast<float*>(columns[c].dst->data);
            gathered = source == eRootSrcPose ?
                           fetchArtiRootPoseColumnOvStage(dst, static_cast<::physx::PxTransform*>(scratch), count,
                                                          columns[c].quantity == RootStateQuantity::eOrientation) :
                           fetchArtiRootVelocityColumnOvStage(dst, static_cast<PxVec3*>(scratch), count);
            if (!gathered)
                CARB_LOG_ERROR("Failed to gather %s", rootStateLabel(columns[c].quantity));
        }
        // Recorded whether or not the gathers succeeded: the buffer was handed to a kernel either
        // way, and skipping this on failure would let the next reader of this scratch overlap it.
        mGpuSimData->recordKernelDone(buffer);
        return gathered;
    };

    if ((sources & eRootSrcPose) &&
        !fetchAndGather(eRootSrcPose, PxArticulationGPUAPIReadType::eROOT_GLOBAL_POSE,
                        static_cast<void*>(mGpuSimData->mLinkOrRootTransformsDev),
                        SharedDeviceBuffer::eLinkOrRootTransforms, CopyEvent::eArtiRootTransforms,
                        "articulation root pose"))
    {
        return false;
    }
    if ((sources & eRootSrcLinearVel) &&
        !fetchAndGather(eRootSrcLinearVel, PxArticulationGPUAPIReadType::eROOT_LINEAR_VELOCITY,
                        static_cast<void*>(mGpuSimData->mLinkOrRootLinearVelAccDev),
                        SharedDeviceBuffer::eLinkOrRootLinearVel, CopyEvent::eArtiRootLinVelocities,
                        "articulation root linear velocity"))
    {
        return false;
    }
    if ((sources & eRootSrcAngularVel) &&
        !fetchAndGather(eRootSrcAngularVel, PxArticulationGPUAPIReadType::eROOT_ANGULAR_VELOCITY,
                        static_cast<void*>(mGpuSimData->mLinkOrRootAngularVelAccDev),
                        SharedDeviceBuffer::eLinkOrRootAngularVel, CopyEvent::eArtiRootAngVelocities,
                        "articulation root angular velocity"))
    {
        return false;
    }
    return true;
}

// ------------------------------------------------------------------------------------------------
// Inverse dynamics columns (REQ-READ-INVDYN-001).
//
// Each is: resolve the cohort's PhysX indices, ask DirectGPU to compute for exactly those, then
// de-stride the result into the caller's packed column. computeArticulationData writes result slot i
// for gpuIndices[i], so a cohort's rows land packed at 0..count-1 in the scratch. What changes per
// cohort is the packed width; the scratch stride stays the scene-wide maximum.
// ------------------------------------------------------------------------------------------------

// The inverse dynamics scratch is allocated whenever the scene holds at least one articulation
// (GpuSimulationData: `if (numArtis > 0)`), zero-dof articulations included -- their matrices are
// root-inclusive, so a free-floating single-link body has a meaningful 6x6 mass matrix and IS served
// on the device path. These buffers are therefore null only when the scene has no articulation at all
// or a device allocation failed. Handing a null `data` to computeArticulationData ABORTS the process
// rather than returning an error, so every getter that submits into this scratch must check first.
// getArticulationMassCenter is exempt: it computes straight into the caller's tensor.
bool GpuArticulationView::checkInverseDynamicsScratch(
    const void* scratch, const char* label, const char* funcName) const
{
    if (scratch)
        return true;
    // WARN_ONCE keys on a static at this expansion site, so all nine callers share one line ever.
    // Hence the wording: the message describes the scene, not just the column that asked first.
    CARB_LOG_WARN_ONCE(
        "%s: %s -- and every other inverse dynamics column -- is unavailable on this DirectGPU scene: the "
        "shared inverse dynamics scratch was not allocated -- the scene holds no articulation, or its "
        "device buffer allocation failed.",
        funcName, label);
    return false;
}

bool GpuArticulationView::getJacobiansOvStage(const TensorDesc* dstTensor,
                                              const PxU32* rows,
                                              PxU32 count,
                                              uint64_t rowsToken) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    const char* label = "articulation jacobian";
    PxU32 width = 0;
    if (!checkTensorDevice(*dstTensor, mDevice, label, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, label, __FUNCTION__) ||
        !checkInverseDynamicsColumn(*dstTensor, rows, count, InverseDynamicsColumn::eJacobian, width, label,
                                    __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, count * width, label, __FUNCTION__))
    {
        return false;
    }
    if (count == 0)
    {
        return true;
    }

    const PxU32 firstEntryIndex = rows ? rows[0] : 0u;
    const PxU32 rootDofs = mEntries[firstEntryIndex].metatype->getFixedBase() ? 0u : 6u;
    const PxU32 jacobianCols = rootDofs + mEntries[firstEntryIndex].numDofs;
    const PxU32 dofRecordBase = firstEntryIndex * mMaxDofs;
    const bool applyBodyOrderSign = mEntries[firstEntryIndex].metatype->hasReversedDofBodyOrder();

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    const PxArticulationGPUIndex* gpuIndicesDev = nullptr;
    if (!ensureOvStageSelection(rows, count, rowsToken, gpuIndicesDev) || !recordOvStageReady(nullptr))
    {
        return false;
    }

    if (!checkInverseDynamicsScratch(mGpuSimData->mJacobianDataDev, label, __FUNCTION__))
    {
        return false;
    }
    const CUevent finishEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiJacobians];
    PxScene* const scene = mGpuSimData->mScene;
    // The return is checked: a refused DirectGPU compute writes nothing and leaves the shared scratch
    // holding the PREVIOUS read's values (ADR-0008 Decision 10), so discarding it would gather stale
    // numbers and publish them as this read's answer. The ovstage error contract is atomic and needs
    // the submission failure to reach it.
    if (!scene->getDirectGPUAPI().computeArticulationData(
            (void*)mGpuSimData->mJacobianDataDev, gpuIndicesDev,
            PxArticulationGPUAPIComputeType::eDENSE_JACOBIANS, count, mOvStageSelectionReadyEvent,
            finishEvent))
    {
        CARB_LOG_ERROR("Failed to compute %s", label);
        waitForDirectGpuFinish(finishEvent, label);
        return false;
    }
    if (!waitForDirectGpuFinish(finishEvent, label))
    {
        return false;
    }

    const bool fetched =
        fetchArtiJacobian(static_cast<float*>(dstTensor->data), mGpuSimData->mJacobianDataDev, count * width, width,
                          mGpuSimData->mJacobianMaxRows * mGpuSimData->mJacobianMaxCols, jacobianCols, rootDofs,
                          dofRecordBase, mDofRecordsDev, applyBodyOrderSign);
    mGpuSimData->recordKernelDone(SharedDeviceBuffer::eJacobianData);
    if (!fetched)
    {
        CARB_LOG_ERROR("Failed to fetch %s", label);
        return false;
    }
    // No host sync: the gather is stream-ordered on the null stream, the consumer waits on the
    // per-context event the reader records after every column (REQ-READ-DEVICE-001, ADR-0008), and
    // the next writer of this shared scratch is held off by recordKernelDone above.
    return true;
}

bool GpuArticulationView::getMassMatricesOvStage(const TensorDesc* dstTensor,
                                                 const PxU32* rows,
                                                 PxU32 count,
                                                 uint64_t rowsToken) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    const char* label = "articulation mass matrix";
    PxU32 width = 0;
    if (!checkTensorDevice(*dstTensor, mDevice, label, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, label, __FUNCTION__) ||
        !checkInverseDynamicsColumn(*dstTensor, rows, count, InverseDynamicsColumn::eMassMatrix, width, label,
                                    __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, count * width, label, __FUNCTION__))
    {
        return false;
    }
    if (count == 0)
    {
        return true;
    }

    const PxU32 firstEntryIndex = rows ? rows[0] : 0u;
    const PxU32 rootDofs = mEntries[firstEntryIndex].metatype->getFixedBase() ? 0u : 6u;
    const PxU32 generalizedCoords = rootDofs + mEntries[firstEntryIndex].numDofs;
    const PxU32 dofRecordBase = firstEntryIndex * mMaxDofs;
    const bool applyBodyOrderSign = mEntries[firstEntryIndex].metatype->hasReversedDofBodyOrder();

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    const PxArticulationGPUIndex* gpuIndicesDev = nullptr;
    if (!ensureOvStageSelection(rows, count, rowsToken, gpuIndicesDev) || !recordOvStageReady(nullptr))
    {
        return false;
    }

    if (!checkInverseDynamicsScratch(mGpuSimData->mMassMatrixDataDev, label, __FUNCTION__))
    {
        return false;
    }
    const CUevent finishEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiMassMatrices];
    PxScene* const scene = mGpuSimData->mScene;
    if (!scene->getDirectGPUAPI().computeArticulationData(
            (void*)mGpuSimData->mMassMatrixDataDev, gpuIndicesDev,
            PxArticulationGPUAPIComputeType::eMASS_MATRICES, count, mOvStageSelectionReadyEvent, finishEvent))
    {
        CARB_LOG_ERROR("Failed to compute %s", label);
        waitForDirectGpuFinish(finishEvent, label);
        return false;
    }
    if (!waitForDirectGpuFinish(finishEvent, label))
    {
        return false;
    }

    const PxU32 simMassMatrixSize = (mGpuSimData->mMaxDofs + 6) * (mGpuSimData->mMaxDofs + 6);
    const bool fetched = fetchArtiMassMatrices(static_cast<float*>(dstTensor->data), mGpuSimData->mMassMatrixDataDev,
                                               count * width, width, simMassMatrixSize, generalizedCoords, rootDofs,
                                               dofRecordBase, mDofRecordsDev, applyBodyOrderSign);
    mGpuSimData->recordKernelDone(SharedDeviceBuffer::eMassMatrixData);
    if (!fetched)
    {
        CARB_LOG_ERROR("Failed to fetch %s", label);
        return false;
    }
    // No host sync: the gather is stream-ordered on the null stream, the consumer waits on the
    // per-context event the reader records after every column (REQ-READ-DEVICE-001, ADR-0008), and
    // the next writer of this shared scratch is held off by recordKernelDone above.
    return true;
}

bool GpuArticulationView::getGeneralizedForceColumnOvStage(const TensorDesc* dstTensor,
                                                           const PxU32* rows,
                                                           PxU32 count,
                                                           uint64_t rowsToken,
                                                           bool gravity) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    const char* label = gravity ? "articulation gravity force" : "articulation coriolis force";
    PxU32 width = 0;
    if (!checkTensorDevice(*dstTensor, mDevice, label, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, label, __FUNCTION__) ||
        !checkInverseDynamicsColumn(*dstTensor, rows, count, InverseDynamicsColumn::eGeneralizedForce, width, label,
                                    __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, count * width, label, __FUNCTION__) || count == 0)
    {
        return false;
    }

    // The sign block comes from the cohort's FIRST row, valid only because every row in a cohort
    // shares a metatype and therefore the same per-dof parentage. checkInverseDynamicsColumn enforces that
    // by comparing interned metatype pointers, not widths: two topologies can agree on generalized
    // coordinate count yet differ in parentage.
    const PxU32 firstEntryIndex = rows ? rows[0] : 0u;
    const bool isFixedBase = mEntries[firstEntryIndex].metatype->getFixedBase();
    const PxU32 rootDofs = isFixedBase ? 0u : 6u;
    const PxU32 dofRecordBase = firstEntryIndex * mMaxDofs;

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    const PxArticulationGPUIndex* gpuIndicesDev = nullptr;
    if (!ensureOvStageSelection(rows, count, rowsToken, gpuIndicesDev) || !recordOvStageReady(nullptr))
    {
        return false;
    }

    if (!checkInverseDynamicsScratch(mGpuSimData->mCoriolisGravityDataDev, label, __FUNCTION__))
    {
        return false;
    }
    const CUevent finishEvent = mGpuSimData->mCopyEvents[gravity ? CopyEvent::eArtiGeneralizedGravity :
                                                                  CopyEvent::eArtiCoriolisCentrifugal];
    PxScene* const scene = mGpuSimData->mScene;
    if (!scene->getDirectGPUAPI().computeArticulationData(
            (void*)mGpuSimData->mCoriolisGravityDataDev, gpuIndicesDev,
            gravity ? PxArticulationGPUAPIComputeType::eGRAVITY_COMPENSATION :
                      PxArticulationGPUAPIComputeType::eCORIOLIS_AND_CENTRIFUGAL_COMPENSATION,
            count, mOvStageSelectionReadyEvent, finishEvent))
    {
        CARB_LOG_ERROR("Failed to compute %s", label);
        waitForDirectGpuFinish(finishEvent, label);
        return false;
    }
    if (!waitForDirectGpuFinish(finishEvent, label))
    {
        return false;
    }

    // Not a dof count: the scene-wide dof maximum plus the floating base's six.
    const PxU32 simGeneralizedCoords = mGpuSimData->mMaxDofs + 6;
    const bool fetched = fetchArtiGeneralizedForceColumnOvStage(static_cast<float*>(dstTensor->data),
                                                                mGpuSimData->mCoriolisGravityDataDev, count, width,
                                                                rootDofs, simGeneralizedCoords, dofRecordBase, mDofRecordsDev);
    mGpuSimData->recordKernelDone(SharedDeviceBuffer::eCoriolisGravityData);
    if (!fetched)
    {
        CARB_LOG_ERROR("Failed to fetch %s", label);
        return false;
    }
    // No host sync: the gather is stream-ordered on the null stream, the consumer waits on the
    // per-context event the reader records after every column (REQ-READ-DEVICE-001, ADR-0008), and
    // the next writer of this shared scratch is held off by recordKernelDone above.
    return true;
}

bool GpuArticulationView::getCoriolisForcesOvStage(const TensorDesc* dstTensor,
                                                   const PxU32* rows,
                                                   PxU32 count,
                                                   uint64_t rowsToken) const
{
    return getGeneralizedForceColumnOvStage(dstTensor, rows, count, rowsToken, false);
}

bool GpuArticulationView::getGravityForcesOvStage(const TensorDesc* dstTensor,
                                                  const PxU32* rows,
                                                  PxU32 count,
                                                  uint64_t rowsToken) const
{
    return getGeneralizedForceColumnOvStage(dstTensor, rows, count, rowsToken, true);
}

bool GpuArticulationView::getCentroidalMomentaOvStage(const TensorDesc* dstTensor,
                                                      const PxU32* rows,
                                                      PxU32 count,
                                                      uint64_t rowsToken) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    const char* label = "articulation centroidal momentum";
    PxU32 width = 0;
    if (!checkTensorDevice(*dstTensor, mDevice, label, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, label, __FUNCTION__) ||
        !checkInverseDynamicsColumn(*dstTensor, rows, count, InverseDynamicsColumn::eCentroidalMomentum, width, label,
                                    __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, count * width, label, __FUNCTION__) || count == 0)
    {
        return false;
    }

    // Read off the cohort's first row: checkInverseDynamicsColumn rejects any row needing a different width,
    // so every row here shares a metatype and therefore a dof count.
    const PxU32 firstEntryIndex = rows ? rows[0] : 0u;

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    const PxArticulationGPUIndex* gpuIndicesDev = nullptr;
    if (!ensureOvStageSelection(rows, count, rowsToken, gpuIndicesDev))
    {
        return false;
    }

    if (!checkInverseDynamicsScratch(mGpuSimData->mCentroidalMomentumDataDev, label, __FUNCTION__))
    {
        return false;
    }

    // The centroidal matrix is defined against the mass matrix and the Coriolis term, so all three
    // are computed into one scratch block, in that order. Each fill re-arms the start event, because
    // the previous fill consumed it.
    PxScene* const scene = mGpuSimData->mScene;
    // Not a dof count: the scene-wide dof maximum plus the floating base's six.
    const PxU32 simGeneralizedCoords = mGpuSimData->mMaxDofs + 6;
    // `count`, not getCount(): the submissions below ask for this cohort's rows, so PhysX lays the
    // scratch out for that many articulations. Sizing these offsets off the view instead reads past
    // everything written -- zeros, not garbage, so the answer looks plausible.
    const PxU32 simStartCoriolisForces = count * simGeneralizedCoords * simGeneralizedCoords;
    const PxArticulationGPUAPIComputeType::Enum stages[3] = {
        PxArticulationGPUAPIComputeType::eMASS_MATRICES,
        PxArticulationGPUAPIComputeType::eCORIOLIS_AND_CENTRIFUGAL_COMPENSATION,
        PxArticulationGPUAPIComputeType::eCENTROIDAL_MOMENTUM_MATRICES,
    };
    // CopyEvent's enum is unnamed, so its constants are plain ints in that scope.
    const uint32_t stageEvents[3] = { CopyEvent::eArtiMassMatrices, CopyEvent::eArtiCoriolisCentrifugal,
                                      CopyEvent::eArtiCentroidalMomentum };
    for (int stage = 0; stage < 3; ++stage)
    {
        if (!recordOvStageReady(nullptr))
        {
            return false;
        }
        float* const target = (stage == 1) ? mGpuSimData->mCentroidalMomentumDataDev + simStartCoriolisForces :
                                             mGpuSimData->mCentroidalMomentumDataDev;
        const CUevent finishEvent = mGpuSimData->mCopyEvents[stageEvents[stage]];
        // Each stage feeds the next -- the centroidal result is defined against the mass matrix and
        // the Coriolis term -- so a refused stage would leave the one after it computing from stale
        // scratch rather than merely omitting its own block.
        if (!scene->getDirectGPUAPI().computeArticulationData((void*)target, gpuIndicesDev, stages[stage], count,
                                                              mOvStageSelectionReadyEvent, finishEvent))
        {
            CARB_LOG_ERROR("Failed to compute %s", label);
            waitForDirectGpuFinish(finishEvent, label);
            return false;
        }
        if (!waitForDirectGpuFinish(finishEvent, label))
        {
            return false;
        }
    }

    // The kernel's `maxDofs` is the PACKED dof count -- it strides the destination by (maxDofs + 7)
    // and the source by (maxDofs + 6) -- so for a cohort it is that cohort's dofs, not the view
    // maximum. The sim-side blocks describe one scratch holding the mass matrices, then the coriolis
    // forces, then the centroidal matrices, then the bias forces, each sized for the SUBMITTED count;
    // the centroidal matrices therefore start past the first two rather than at zero.
    const PxU32 cohortDofs = mEntries[firstEntryIndex].numDofs;
    const PxU32 dofRecordBase = firstEntryIndex * mMaxDofs;
    const bool applyBodyOrderSign = mEntries[firstEntryIndex].metatype->hasReversedDofBodyOrder();
    const PxU32 blockSize = width; // 6 * (cohortDofs + 7), this cohort's packed block
    const PxU32 simCentroidalBlockSize = 6u * simGeneralizedCoords;
    const PxU32 simMassMatrixBlockSize = simGeneralizedCoords * simGeneralizedCoords;
    const PxU32 simCoriolisBlockSize = simGeneralizedCoords;
    const PxU32 simStartCentroidalMomentumMatrix = (simMassMatrixBlockSize + simCoriolisBlockSize) * count;
    const PxU32 startSimBiasForceBlock = simCentroidalBlockSize * count;
    const bool fetched = fetchArtiCentroidalMomentumMatrices(
        static_cast<float*>(dstTensor->data),
        mGpuSimData->mCentroidalMomentumDataDev + simStartCentroidalMomentumMatrix, count * blockSize, cohortDofs,
        blockSize, simCentroidalBlockSize, startSimBiasForceBlock, dofRecordBase, mDofRecordsDev, applyBodyOrderSign);
    mGpuSimData->recordKernelDone(SharedDeviceBuffer::eCentroidalMomentumData);
    if (!fetched)
    {
        CARB_LOG_ERROR("Failed to fetch %s", label);
        return false;
    }
    // No host sync: the gather is stream-ordered on the null stream, the consumer waits on the
    // per-context event the reader records after every column (REQ-READ-DEVICE-001, ADR-0008), and
    // the next writer of this shared scratch is held off by recordKernelDone above.
    return true;
}

bool GpuArticulationView::getMassCentersOvStage(
    const TensorDesc* dstTensor, const PxU32* rows, PxU32 count, uint64_t rowsToken, bool localFrame) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    const char* label = localFrame ? "articulation local mass center" : "articulation world mass center";
    if (!checkTensorDevice(*dstTensor, mDevice, label, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, label, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, count * 3u, label, __FUNCTION__))
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    const PxArticulationGPUIndex* gpuIndicesDev = nullptr;
    if (!ensureOvStageSelection(rows, count, rowsToken, gpuIndicesDev))
    {
        return false;
    }
    // Zero first: the PhysX COM kernel accumulates into this buffer rather than writing it.
    // computeArtiCOM (physx/source/gpuarticulation/src/CUDA/inverseDynamic.cu) atomically adds each
    // link's mass into dst.x, reads it back as the total, then adds the mass-weighted positions into
    // all three components, initialising none of them -- so a non-zero buffer divides by a corrupted
    // total mass. PxDirectGPUAPI.h does not state the requirement; the kernel is the contract.
    // Zeroed on the stream that records the start event, so it necessarily precedes the compute.
    if (!CHECK_CUDA(cudaMemsetAsync(dstTensor->data, 0, size_t(count) * sizeof(PxVec3), nullptr)) ||
        !recordOvStageReady(nullptr))
    {
        return false;
    }

    const CUevent finishEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiMassCenter];
    if (!finishEvent)
    {
        CARB_LOG_ERROR("Missing synchronization event for %s", label);
        return false;
    }
    PxScene* const scene = mGpuSimData->mScene;
    if (!scene->getDirectGPUAPI().computeArticulationData(
            dstTensor->data, gpuIndicesDev,
            localFrame ? PxArticulationGPUAPIComputeType::eARTICULATION_COMS_ROOT_FRAME :
                         PxArticulationGPUAPIComputeType::eARTICULATION_COMS_WORLD_FRAME,
            count, mOvStageSelectionReadyEvent, finishEvent))
    {
        CARB_LOG_ERROR("Failed to compute %s", label);
        waitForDirectGpuFinish(finishEvent, label);
        return false;
    }
    if (!waitForDirectGpuFinish(finishEvent, label))
    {
        return false;
    }

    return true;
}

// The four entry points the attribute table names, each a ONE-column case of the gather above.
bool GpuArticulationView::getPositionsOvStage(const TensorDesc* dstTensor,
                                              const PxU32* rows,
                                              PxU32 count,
                                              uint64_t rowsToken) const
{
    const RootStateColumn column{ RootStateQuantity::ePosition, dstTensor };
    return getRootStateColumnsOvStage(&column, 1, rows, count, rowsToken);
}

bool GpuArticulationView::getOrientationsOvStage(const TensorDesc* dstTensor,
                                                 const PxU32* rows,
                                                 PxU32 count,
                                                 uint64_t rowsToken) const
{
    const RootStateColumn column{ RootStateQuantity::eOrientation, dstTensor };
    return getRootStateColumnsOvStage(&column, 1, rows, count, rowsToken);
}

bool GpuArticulationView::getLinearVelocitiesOvStage(const TensorDesc* dstTensor,
                                                     const PxU32* rows,
                                                     PxU32 count,
                                                     uint64_t rowsToken) const
{
    const RootStateColumn column{ RootStateQuantity::eLinearVelocity, dstTensor };
    return getRootStateColumnsOvStage(&column, 1, rows, count, rowsToken);
}

bool GpuArticulationView::getAngularVelocitiesOvStage(const TensorDesc* dstTensor,
                                                      const PxU32* rows,
                                                      PxU32 count,
                                                      uint64_t rowsToken) const
{
    const RootStateColumn column{ RootStateQuantity::eAngularVelocity, dstTensor };
    return getRootStateColumnsOvStage(&column, 1, rows, count, rowsToken);
}

bool GpuArticulationView::getMassCentersWorldOvStage(const TensorDesc* dstTensor,
                                                     const PxU32* rows,
                                                     PxU32 count,
                                                     uint64_t rowsToken) const
{
    return getMassCentersOvStage(dstTensor, rows, count, rowsToken, false);
}

bool GpuArticulationView::getMassCentersLocalOvStage(const TensorDesc* dstTensor,
                                                     const PxU32* rows,
                                                     PxU32 count,
                                                     uint64_t rowsToken) const
{
    return getMassCentersOvStage(dstTensor, rows, count, rowsToken, true);
}

bool GpuArticulationView::setRootVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, mDevice, "root velocity", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "root velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * 6u, "root velocity", __FUNCTION__))
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
        indices = mViewIndicesDev;
        numIndices = getCount();
    }

    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    CHECK_CUDA(cudaMemset(mDirtyArtiGpuIndicesDev, 0, getCount() * sizeof(PxArticulationGPUIndex)));
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    if (!submitArtiRootVelocities( mGpuSimData->mLinkOrRootLinearVelAccDev,  mGpuSimData->mLinkOrRootAngularVelAccDev,
                                  static_cast<const TensorVelAcc*>(srcTensor->data), indices, mDirtyArtiGpuIndicesDev,
                                  numIndices, (PxU32)getCount(), mRootRecordsDev))
    {
        CARB_LOG_ERROR("Failed to submit articulation root velocities");
        return false;
    }
    // CHECK_CUDA(cudaStreamSynchronize(nullptr));
    CHECK_CU(getCudaShim()->eventRecord(reinterpret_cast<uintptr_t>(mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiRootLinVelocities]), uintptr_t(0), nullptr));
    CHECK_CU(getCudaShim()->eventRecord(reinterpret_cast<uintptr_t>(mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiRootAngVelocities]), uintptr_t(0), nullptr));
    scene->getDirectGPUAPI().setArticulationData((void*) mGpuSimData->mLinkOrRootLinearVelAccDev, mDirtyArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIWriteType::eROOT_LINEAR_VELOCITY, numIndices,
                                                 mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiRootLinVelocities],
                                                 mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootLinVelocities]);
    scene->getDirectGPUAPI().setArticulationData((void*) mGpuSimData->mLinkOrRootAngularVelAccDev, mDirtyArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIWriteType::eROOT_ANGULAR_VELOCITY, numIndices,
                                                 mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiRootAngVelocities],
                                                 mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootAngVelocities]);

    CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootLinVelocities]), nullptr));
    CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootAngVelocities]), nullptr));

    SYNCHRONIZE_CUDA();
    return true;
}

bool GpuArticulationView::getDofAttribute(const char* attribName,
                                          const TensorDesc* dstTensor,
                                          const PxArticulationGPUAPIReadType::Enum attribFlag,
                                          CUevent syncEvent) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (mMaxDofs==0)
    {
        CARB_LOG_WARN("Articulation has no DOF");
        return true;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, attribName, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, attribName, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, attribName, __FUNCTION__))
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    SYNCHRONIZE_CUDA();

    if (!mGpuSimData->awaitDirectGpuFetch(scene->getDirectGPUAPI().getArticulationData(
                                              (void*)mGpuSimData->mDofScalarsDev, mArtiGpuIndicesDev, attribFlag, numArtis,
                                              mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eDofScalars), syncEvent),
                                          syncEvent, __FUNCTION__))
    {
        return false;
    }

    SYNCHRONIZE_CUDA();

    // Our kernel is done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded unconditionally: every kernel that touches a buffer records it.
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eDofScalars, [&] {
            return fetchArtiDofAttribute(static_cast<float*>(dstTensor->data), mGpuSimData->mDofScalarsDev, mDofBufSize,
                                         mMaxDofs, mGpuSimData->mMaxDofs, mDofRecordsDev);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch %s attribute", attribName);
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

bool GpuArticulationView::getDofAttributeOvStage(const char* attribName,
                                                 const TensorDesc* dstTensor,
                                                 const PxArticulationGPUAPIReadType::Enum attribFlag,
                                                 const DofScalePolicy policy,
                                                 const ArticulationDofOvStageRecord* recordsDev,
                                                 PxU32 numOutputs,
                                                 CUevent syncEvent) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }
    if (numOutputs == 0)
    {
        return true; // nothing to emit -- not a failure
    }
    if (mMaxDofs == 0)
    {
        CARB_LOG_WARN("Articulation has no DOF");
        return false; // caller asked for outputs but there are no DOFs -> record map is invalid
    }
    if (!recordsDev)
    {
        CARB_LOG_ERROR("%s: null ovstage DOF records", attribName);
        return false;
    }
    if (!checkTensorDevice(*dstTensor, mDevice, attribName, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, attribName, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, numOutputs, attribName, __FUNCTION__))
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    SYNCHRONIZE_CUDA();

    // Fill ALL dofs of the view's articulations into the shared scratch (PhysX layout, stride
    // mGpuSimData->mMaxDofs); the ovstage gather then picks/scales the (joint-prim, enabled-axis)
    // subset it needs, so no intermediate dense [numArti x maxDofs] buffer.
    // startEvent, not nullptr: this buffer is shared, and a gather from a PREVIOUS read of it may
    // still be running on our stream -- without it PhysX overwrites the buffer underneath that
    // gather, giving plausible wrong values and no crash (ADR-0008 Decision 7).
    if (!mGpuSimData->awaitDirectGpuFetch(scene->getDirectGPUAPI().getArticulationData(
                                              (void*)mGpuSimData->mDofScalarsDev, mArtiGpuIndicesDev, attribFlag, numArtis,
                                              mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eDofScalars), syncEvent),
                                          syncEvent, attribName))
    {
        return false;
    }

    SYNCHRONIZE_CUDA();

    // The buffer stops being read here, so this is where the next DirectGPU call may take over.
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eDofScalars, [&] {
            return fetchArtiDofAttributeOvStage(static_cast<float*>(dstTensor->data), mGpuSimData->mDofScalarsDev,
                                                numOutputs, mGpuSimData->mMaxDofs, policy, recordsDev);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch %s attribute", attribName);
        return false;
    }

    // No host block: the ovstage read issues every column on the null stream and synchronizes once at
    // the end (ADR-0008), and the streamWaitEvent above already orders the fill ahead of the gather.
    return true;
}

bool GpuArticulationView::setDofAttributeOvStage(const char* const attribName,
                                                 const TensorDesc* const srcTensor,
                                                 const PxArticulationGPUAPIReadType::Enum readFlag,
                                                 const PxArticulationGPUAPIWriteType::Enum writeFlag,
                                                 const ArticulationDofOvStageRecord* const recordsDev,
                                                 const PxU32 numOutputs,
                                                 const int copyEvent,
                                                 const int applyEvent,
                                                 const DofScalePolicy policy)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (!srcTensor || !srcTensor->data)
        return false;
    if (numOutputs == 0)
        return true; // nothing to publish -- not a failure
    if (mMaxDofs == 0)
    {
        CARB_LOG_WARN("Articulation has no DOF");
        return false;
    }
    if (!recordsDev)
    {
        CARB_LOG_ERROR("%s: null ovstage DOF records", attribName);
        return false;
    }
    if (!checkTensorDevice(*srcTensor, mDevice, attribName, __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, attribName, __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, numOutputs, attribName, __FUNCTION__))
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    PxScene* scene = mGpuSimData->mScene;
    const PxU32 numArtis = getCount();

    // Read-modify-write. Status IS checked: PhysX refuses these calls in states the caller cannot
    // see and writes NOTHING when it does; the scratch is not zeroed, so an unchecked refusal would
    // overlay the caller's DOFs onto stale contents and publish that as joint state.
    CUevent readEvent = mGpuSimData->mCopyEvents[copyEvent];
    if (!scene->getDirectGPUAPI().getArticulationData((void*)mGpuSimData->mDofScalarsDev, mArtiGpuIndicesDev,
                                                      readFlag, numArtis,
                                                      mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eDofScalars),
                                                      readEvent))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::getArticulationData refused the read that preserves the DOFs "
                       "this write does not address",
                       attribName);
        return false;
    }
    CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(readEvent), 0, nullptr));

    if (!submitArtiDofAttributeOvStage(mGpuSimData->mDofScalarsDev, static_cast<const float*>(srcTensor->data),
                                       numOutputs, mGpuSimData->mMaxDofs, recordsDev, policy))
    {
        CARB_LOG_ERROR("%s: ovstage DOF scatter failed", attribName);
        return false;
    }

    // Ordered on the device, not by a host block: record our completion and hand it to PhysX as the
    // start event, the convention the rest of this class already follows.
    CHECK_CU(getCudaShim()->eventRecord(
        reinterpret_cast<uintptr_t>(mGpuSimData->mApplyWaitEvents[applyEvent]), uintptr_t(0), nullptr));
    scene->getDirectGPUAPI().setArticulationData((void*)mGpuSimData->mDofScalarsDev, mArtiGpuIndicesDev, writeFlag,
                                                 numArtis, mGpuSimData->mApplyWaitEvents[applyEvent],
                                                 mGpuSimData->mApplySignalEvents[applyEvent]);
    // PhysX's apply above reads the buffer ASYNC (finishEvent = mApplySignalEvents); recordKernelDone
    // only captures our scatter, so order our stream behind the apply before releasing -- otherwise the
    // next producer, gated on kernelDoneEvent, would overwrite the buffer mid-apply (ADR-0008 D7, the
    // reverse direction the host block used to cover).
    CHECK_CU(getCudaShim()->streamWaitEvent(
        uintptr_t(0), reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[applyEvent]), 0, nullptr));
    mGpuSimData->recordKernelDone(SharedDeviceBuffer::eDofScalars);
    return true;
}

bool GpuArticulationView::setDofPositionsOvStage(const TensorDesc* srcTensor,
                                                 const ArticulationDofOvStageRecord* recordsDev,
                                                 const PxU32 numOutputs)
{
    return setDofAttributeOvStage("jointPosition", srcTensor, PxArticulationGPUAPIReadType::eJOINT_POSITION,
                                  PxArticulationGPUAPIWriteType::eJOINT_POSITION, recordsDev, numOutputs,
                                  CopyEvent::eArtiDofPositions, ApplyEvent::eArtiDofPositions,
                                  DofScalePolicy::eAngularSigned);
}

bool GpuArticulationView::setDofVelocitiesOvStage(const TensorDesc* srcTensor,
                                                  const ArticulationDofOvStageRecord* recordsDev,
                                                  const PxU32 numOutputs)
{
    return setDofAttributeOvStage("jointVelocity", srcTensor, PxArticulationGPUAPIReadType::eJOINT_VELOCITY,
                                  PxArticulationGPUAPIWriteType::eJOINT_VELOCITY, recordsDev, numOutputs,
                                  CopyEvent::eArtiDofVelocities, ApplyEvent::eArtiDofVelocities,
                                  DofScalePolicy::eAngularSigned);
}

// The three drive INPUTS (ADR-0012). Same machinery as the state pair above -- the difference that
// matters is the scale policy: an actuation force is a joint effort, so it takes the sign but NOT
// the rad->deg fold the other four take.
bool GpuArticulationView::setDofPositionTargetsOvStage(const TensorDesc* srcTensor,
                                                       const ArticulationDofOvStageRecord* recordsDev,
                                                       const PxU32 numOutputs)
{
    return setDofAttributeOvStage("jointPositionTarget", srcTensor,
                                  PxArticulationGPUAPIReadType::eJOINT_TARGET_POSITION,
                                  PxArticulationGPUAPIWriteType::eJOINT_TARGET_POSITION, recordsDev, numOutputs,
                                  CopyEvent::eArtiDofPositionTargets, ApplyEvent::eArtiDofPositionTargets,
                                  DofScalePolicy::eAngularSigned);
}

bool GpuArticulationView::setDofVelocityTargetsOvStage(const TensorDesc* srcTensor,
                                                       const ArticulationDofOvStageRecord* recordsDev,
                                                       const PxU32 numOutputs)
{
    return setDofAttributeOvStage("jointVelocityTarget", srcTensor,
                                  PxArticulationGPUAPIReadType::eJOINT_TARGET_VELOCITY,
                                  PxArticulationGPUAPIWriteType::eJOINT_TARGET_VELOCITY, recordsDev, numOutputs,
                                  CopyEvent::eArtiDofVelocityTargets, ApplyEvent::eArtiDofVelocityTargets,
                                  DofScalePolicy::eAngularSigned);
}

bool GpuArticulationView::setDofActuationForcesOvStage(const TensorDesc* srcTensor,
                                                       const ArticulationDofOvStageRecord* recordsDev,
                                                       const PxU32 numOutputs)
{
    return setDofAttributeOvStage("jointActuationForce", srcTensor, PxArticulationGPUAPIReadType::eJOINT_FORCE,
                                  PxArticulationGPUAPIWriteType::eJOINT_FORCE, recordsDev, numOutputs,
                                  CopyEvent::eArtiDofActuationForces, ApplyEvent::eArtiDofForces,
                                  DofScalePolicy::eSigned);
}

const PxU32* GpuArticulationView::ovStageRowsDevice(const PxU32* rows, uint32_t count, uint64_t token) const
{
    if (!rows || count == 0)
        return nullptr;

    CudaContextGuard ctxGuard(mGpuSimData ? mGpuSimData->mCtx : nullptr);

    if (count > mOvStageRowsCapacity)
    {
        CHECK_CUDA(cudaFree(mOvStageRowsDev));
        mOvStageRowsDev = nullptr;
        mOvStageRowsCapacity = 0;
        mOvStageRowsToken = 0;
        if (!prepareDeviceData((void**)&mOvStageRowsDev, nullptr, count * sizeof(PxU32), "mOvStageRowsDev"))
            return nullptr;
        mOvStageRowsCapacity = count;
    }

    // Re-uploaded when the token changes; the count is compared too, and is not redundant -- it
    // bounds the memcpy and mOvStageRowsCount independently of what the caller derives its token
    // from. See GpuRigidBodyView::ovStageRowsDevice for why the token must identify the row list's
    // CONTENTS and cannot be derived from the generation, type and scope.
    if (mOvStageRowsToken != token || mOvStageRowsCount != count)
    {
        // Validated inside the upload gate rather than per write: the scatter kernels index
        // rootBlock[rows[i]] with no bound of their own, so an out-of-range row is a silent
        // out-of-bounds device WRITE -- worse than the read the rigid view guards, because it
        // corrupts another articulation's root state instead of returning a wrong value.
        if (!checkRecordIndices(rows, count, mEntries.size(), "ovstage articulation row list", __FUNCTION__))
        {
            mOvStageRowsToken = 0;
            return nullptr;
        }
        if (!CHECK_CUDA(cudaMemcpy(mOvStageRowsDev, rows, count * sizeof(PxU32), cudaMemcpyHostToDevice)))
        {
            mOvStageRowsToken = 0; // a failed upload must not be mistaken for a held copy
            return nullptr;
        }
        mOvStageRowsToken = token;
        mOvStageRowsCount = count;
    }
    return mOvStageRowsDev;
}

bool GpuArticulationView::setRootAttributeOvStage(const char* const attribName,
                                                  const TensorDesc* const srcTensor,
                                                  const PxU32* const rows,
                                                  const PxU32 numOutputs,
                                                  const uint64_t rowsToken,
                                                  const bool angular,
                                                  const bool pose)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (!srcTensor || !srcTensor->data || !rows)
        return false;
    if (numOutputs == 0)
        return true; // nothing to publish -- not a failure

    // Only the ORIENTATION is a quaternion. `angular` selects the rotational half of whichever pair
    // this attribute belongs to -- orientation within a pose, angular velocity within a velocity --
    // and an angular VELOCITY is still a vec3, so the width comes from both flags and not from one.
    const PxU32 comp = (pose && angular) ? 4u : 3u;
    if (!checkTensorDevice(*srcTensor, mDevice, attribName, __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, attribName, __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, numOutputs * comp, attribName, __FUNCTION__))
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    const PxU32* rowsDev = ovStageRowsDevice(rows, numOutputs, rowsToken);
    if (!rowsDev)
        return false;

    PxScene* scene = mGpuSimData->mScene;
    const PxU32 numArtis = getCount();

    // Which of the three root blocks this attribute lives in. Pose is one block PhysX reads and
    // writes whole; linear and angular velocity are separate write types with a block each, so
    // neither has to preserve the other.
    const PxArticulationGPUAPIReadType::Enum readFlag =
        pose    ? PxArticulationGPUAPIReadType::eROOT_GLOBAL_POSE :
        angular ? PxArticulationGPUAPIReadType::eROOT_ANGULAR_VELOCITY :
                  PxArticulationGPUAPIReadType::eROOT_LINEAR_VELOCITY;
    const PxArticulationGPUAPIWriteType::Enum writeFlag =
        pose    ? PxArticulationGPUAPIWriteType::eROOT_GLOBAL_POSE :
        angular ? PxArticulationGPUAPIWriteType::eROOT_ANGULAR_VELOCITY :
                  PxArticulationGPUAPIWriteType::eROOT_LINEAR_VELOCITY;
    const PxU32 sharedBuf = pose    ? SharedDeviceBuffer::eLinkOrRootTransforms :
                            angular ? SharedDeviceBuffer::eLinkOrRootAngularVel :
                                      SharedDeviceBuffer::eLinkOrRootLinearVel;
    const int copyEvent = pose    ? CopyEvent::eArtiRootTransforms :
                          angular ? CopyEvent::eArtiRootAngVelocities :
                                    CopyEvent::eArtiRootLinVelocities;
    const int applyEvent = pose    ? ApplyEvent::eArtiRootTransforms :
                           angular ? ApplyEvent::eArtiRootAngVelocities :
                                     ApplyEvent::eArtiRootLinVelocities;
    void* const block = pose    ? static_cast<void*>(mGpuSimData->mLinkOrRootTransformsDev) :
                        angular ? static_cast<void*>(mGpuSimData->mLinkOrRootAngularVelAccDev) :
                                  static_cast<void*>(mGpuSimData->mLinkOrRootLinearVelAccDev);

    // Read-modify-write. Status IS checked: PhysX refuses these calls in states the caller cannot
    // see and writes NOTHING when it does; the block is not zeroed, so an unchecked refusal would
    // overlay the caller's values onto stale contents and publish that as root state.
    CUevent readEvent = mGpuSimData->mCopyEvents[copyEvent];
    if (!scene->getDirectGPUAPI().getArticulationData(block, mArtiGpuIndicesDev, readFlag, numArtis,
                                                      mGpuSimData->kernelDoneEvent(sharedBuf), readEvent))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::getArticulationData refused the read that preserves the root "
                       "state this write does not address",
                       attribName);
        return false;
    }
    CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(readEvent), 0, nullptr));

    const bool scattered =
        pose ? submitArtiRootPoseOvStage(static_cast<PxTransform*>(block),
                                          static_cast<const float*>(srcTensor->data), rowsDev, mRootRecordsDev,
                                          numOutputs, angular) :
               submitArtiRootVelocityOvStage(static_cast<PxVec3*>(block),
                                              static_cast<const float*>(srcTensor->data), rowsDev, numOutputs);
    if (!scattered)
    {
        CARB_LOG_ERROR("%s: ovstage articulation root scatter failed", attribName);
        return false;
    }

    // Ordered on the device, not by a host block: record our completion and hand it to PhysX as the
    // start event, the convention the rest of this class follows.
    CHECK_CU(getCudaShim()->eventRecord(
        reinterpret_cast<uintptr_t>(mGpuSimData->mApplyWaitEvents[applyEvent]), uintptr_t(0), nullptr));
    // mArtiGpuIndicesDev, not a compacted list: the whole view is pushed, and the articulations this
    // query did not match are written their own current values -- a no-op on their state, and the
    // reason the RMW above covers the whole view rather than just the matched rows. A subset write
    // would need a compacted index list whose LENGTH comes off the host, which is the host block the
    // read carries none of.
    scene->getDirectGPUAPI().setArticulationData(block, mArtiGpuIndicesDev, writeFlag, numArtis,
                                                 mGpuSimData->mApplyWaitEvents[applyEvent],
                                                 mGpuSimData->mApplySignalEvents[applyEvent]);
    // PhysX's apply above reads the block ASYNC (finishEvent = mApplySignalEvents); recordKernelDone
    // only captures our scatter, so order our stream behind the apply before releasing -- otherwise the
    // next producer, gated on kernelDoneEvent, would overwrite the block mid-apply (ADR-0008 D7, the
    // reverse direction the host block used to cover).
    CHECK_CU(getCudaShim()->streamWaitEvent(
        uintptr_t(0), reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[applyEvent]), 0, nullptr));
    mGpuSimData->recordKernelDone(sharedBuf);
    return true;
}

bool GpuArticulationView::setRootPositionsOvStage(const TensorDesc* srcTensor,
                                                  const PxU32* rows,
                                                  const PxU32 numOutputs,
                                                  const uint64_t rowsToken)
{
    return setRootAttributeOvStage("position", srcTensor, rows, numOutputs, rowsToken, false, true);
}

bool GpuArticulationView::setRootOrientationsOvStage(const TensorDesc* srcTensor,
                                                     const PxU32* rows,
                                                     const PxU32 numOutputs,
                                                     const uint64_t rowsToken)
{
    return setRootAttributeOvStage("orientation", srcTensor, rows, numOutputs, rowsToken, true, true);
}

bool GpuArticulationView::setRootLinearVelocitiesOvStage(const TensorDesc* srcTensor,
                                                         const PxU32* rows,
                                                         const PxU32 numOutputs,
                                                         const uint64_t rowsToken)
{
    return setRootAttributeOvStage("linearVelocity", srcTensor, rows, numOutputs, rowsToken, false, false);
}

bool GpuArticulationView::setRootAngularVelocitiesOvStage(const TensorDesc* srcTensor,
                                                          const PxU32* rows,
                                                          const PxU32 numOutputs,
                                                          const uint64_t rowsToken)
{
    return setRootAttributeOvStage("angularVelocity", srcTensor, rows, numOutputs, rowsToken, true, false);
}

bool GpuArticulationView::setDofAttribute(const char* attribName,
                                          const TensorDesc* srcTensor,
                                          const TensorDesc* indexTensor,
                                          const PxArticulationGPUAPIWriteType::Enum attribFlag,
                                          CUevent mApplyWaitEvents,
                                          CUevent mApplySignalEvents)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (mMaxDofs==0)
    {
        CARB_LOG_WARN("Articulation has no DOF");
        return true;
    }
    if (!checkTensorDevice(*srcTensor, mDevice, attribName, __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, attribName, __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs, attribName, __FUNCTION__))
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
        indices = mViewIndicesDev;
        numIndices = getCount();
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    PxScene* scene = mGpuSimData->mScene;

    CHECK_CUDA(cudaMemset(mDirtyArtiGpuIndicesDev, 0, getCount() * sizeof(PxArticulationGPUIndex)));
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    if (!submitArtiDofAttribute(mGpuSimData->mDofScalarsDev, static_cast<const float*>(srcTensor->data), indices,
                                mDirtyArtiGpuIndicesDev, numIndices * mMaxDofs, mMaxDofs, mGpuSimData->mMaxDofs, getCount(), mDofRecordsDev))
    {
        CARB_LOG_ERROR("Failed to submit %s attribute", attribName);
        return false;
    }
    CHECK_CU(getCudaShim()->eventRecord(reinterpret_cast<uintptr_t>(mApplyWaitEvents), uintptr_t(0), nullptr));
    scene->getDirectGPUAPI().setArticulationData(
        (void*) mGpuSimData->mDofScalarsDev, mDirtyArtiGpuIndicesDev, attribFlag, numIndices, mApplyWaitEvents, mApplySignalEvents);
    CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(mApplySignalEvents), nullptr));

    SYNCHRONIZE_CUDA();

    return true;
}

bool GpuArticulationView::getDofPositions(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    return getDofAttribute("DOF position", dstTensor, PxArticulationGPUAPIReadType::Enum::eJOINT_POSITION,
                           mGpuSimData->mCopyEvents[CopyEvent::eArtiDofPositions]);
}

bool GpuArticulationView::setDofPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(srcTensor);

    return setDofAttribute("DOF position", srcTensor, indexTensor, PxArticulationGPUAPIWriteType::Enum::eJOINT_POSITION,
                           mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiDofPositions],
                           mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiDofPositions]);
}

bool GpuArticulationView::getDofVelocities(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    return getDofAttribute("DOF velocity", dstTensor, PxArticulationGPUAPIReadType::Enum::eJOINT_VELOCITY,
                           mGpuSimData->mCopyEvents[CopyEvent::eArtiDofVelocities]);
}

bool GpuArticulationView::getDofPositionsOvStage(const ArticulationDofOvStageRecord* recordsDev,
                                                 PxU32 numOutputs, const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    return getDofAttributeOvStage("DOF position (ovstage)", dstTensor,
                                  PxArticulationGPUAPIReadType::Enum::eJOINT_POSITION,
                                  DofScalePolicy::eAngularSigned, recordsDev, numOutputs,
                                  mGpuSimData->mCopyEvents[CopyEvent::eArtiDofPositions]);
}

bool GpuArticulationView::getDofVelocitiesOvStage(const ArticulationDofOvStageRecord* recordsDev,
                                                  PxU32 numOutputs, const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    return getDofAttributeOvStage("DOF velocity (ovstage)", dstTensor,
                                  PxArticulationGPUAPIReadType::Enum::eJOINT_VELOCITY,
                                  DofScalePolicy::eAngularSigned, recordsDev, numOutputs,
                                  mGpuSimData->mCopyEvents[CopyEvent::eArtiDofVelocities]);
}

// The two drive TARGETS are the same generalized coordinate and its derivative as the two state
// scalars above, so they take the same fold: degrees on an angular axis, and the body-order sign.
bool GpuArticulationView::getDofPositionTargetsOvStage(const ArticulationDofOvStageRecord* recordsDev,
                                                       PxU32 numOutputs, const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    return getDofAttributeOvStage("DOF position target (ovstage)", dstTensor,
                                  PxArticulationGPUAPIReadType::Enum::eJOINT_TARGET_POSITION,
                                  DofScalePolicy::eAngularSigned, recordsDev, numOutputs,
                                  mGpuSimData->mCopyEvents[CopyEvent::eArtiDofPositionTargets]);
}

bool GpuArticulationView::getDofVelocityTargetsOvStage(const ArticulationDofOvStageRecord* recordsDev,
                                                       PxU32 numOutputs, const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    return getDofAttributeOvStage("DOF velocity target (ovstage)", dstTensor,
                                  PxArticulationGPUAPIReadType::Enum::eJOINT_TARGET_VELOCITY,
                                  DofScalePolicy::eAngularSigned, recordsDev, numOutputs,
                                  mGpuSimData->mCopyEvents[CopyEvent::eArtiDofVelocityTargets]);
}

// The actuation force takes the SIGN but not the degree fold: it is the generalized force on the
// axis -- a newton on a prismatic axis, a newton-metre on a revolute one -- and neither is an angle.
// Applying the position fold here would scale every torque by 57.295.
bool GpuArticulationView::getDofActuationForcesOvStage(const ArticulationDofOvStageRecord* recordsDev,
                                                       PxU32 numOutputs, const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    return getDofAttributeOvStage("DOF actuation force (ovstage)", dstTensor,
                                  PxArticulationGPUAPIReadType::Enum::eJOINT_FORCE, DofScalePolicy::eSigned,
                                  recordsDev, numOutputs,
                                  mGpuSimData->mCopyEvents[CopyEvent::eArtiDofActuationForces]);
}

bool GpuArticulationView::getDofProjectedForcesOvStage(const ArticulationDofOvStageRecord* recordsDev,
                                                       PxU32 numOutputs, const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }
    if (numOutputs == 0)
    {
        return true; // nothing to emit -- not a failure
    }
    if (mMaxDofs == 0)
    {
        CARB_LOG_WARN("Articulation has no DOF");
        return false; // caller asked for outputs but there are no DOFs -> record map is invalid
    }
    const char* label = "jointProjectedForce";
    if (!recordsDev)
    {
        CARB_LOG_ERROR("%s: %s null ovstage DOF records", __FUNCTION__, label);
        return false;
    }
    if (!checkTensorDevice(*dstTensor, mDevice, label, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, label, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, numOutputs, label, __FUNCTION__))
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    // Staged through the SHARED DOF buffer: it is sized for the scene while this needs the view's
    // (getCount() x mMaxDofs), so it always fits, and it cannot alias what the projection reads (the
    // link-shaped incoming-force and pose buffers). Staged at all because the projection SCATTERS --
    // one thread per link writing that link's inbound DOFs -- so there is no per-output-row
    // formulation that could write the caller's tensor directly.
    const size_t denseFloats = size_t(getCount()) * mMaxDofs;
    if (!mGpuSimData->mDofScalarsDev)
    {
        CARB_LOG_ERROR("%s: %s no shared DOF buffer", __FUNCTION__, label);
        return false;
    }
    // Zeroed before the dense pass because that pass writes only the DOFs of links that HAVE an
    // inbound joint -- a fixed joint contributes none -- and the buffer is shared, so an unwritten
    // slot would otherwise hand back whatever the last read of it left there.
    if (!CHECK_CUDA(cudaMemset(mGpuSimData->mDofScalarsDev, 0, denseFloats * sizeof(float))))
    {
        return false;
    }

    TensorDesc denseDesc;
    denseDesc.device = mDevice;
    denseDesc.dtype = omni::physics::tensors::TensorDataType::eFloat32;
    denseDesc.numDims = 1;
    denseDesc.dims[0] = static_cast<int64_t>(denseFloats);
    denseDesc.data = mGpuSimData->mDofScalarsDev;
    if (!getDofProjectedJointForces(&denseDesc))
    {
        return false;
    }

    // The dense row is strided by the VIEW's maxDofs (that is what its kernel's dofOffset counts in),
    // not the scene's, so the gather is the same one the DOF-buffer attributes use with a different
    // stride. No fold: the projection already resolved the joint frame and the body order.
    // The shared buffer stops being read here, so this is where the next DirectGPU call may take over.
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eDofScalars, [&] {
            return fetchArtiDofAttributeOvStage(static_cast<float*>(dstTensor->data), mGpuSimData->mDofScalarsDev,
                                                numOutputs, mMaxDofs, DofScalePolicy::eNone, recordsDev);
        }))
    {
        CARB_LOG_ERROR("%s: failed to fetch %s", __FUNCTION__, label);
        return false;
    }
    return true;
}

bool GpuArticulationView::getLinkIncomingJointForcesOvStage(const ArticulationLinkOvStageRecord* recordsDev,
                                                            PxU32 numOutputs, const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }
    if (numOutputs == 0)
    {
        return true;
    }
    if (mMaxLinks == 0)
    {
        CARB_LOG_WARN("Articulation has no links");
        return false;
    }
    const char* label = "linkIncomingJointForce";
    if (!recordsDev)
    {
        CARB_LOG_ERROR("%s: %s null ovstage link records", __FUNCTION__, label);
        return false;
    }
    if (!checkTensorDevice(*dstTensor, mDevice, label, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, label, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, numOutputs * 6u, label, __FUNCTION__))
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    // A dense staging row, then a gather, so this path calls the same dense getter the tensor API
    // exposes; see mLinkIncomingJointForceScratchDev for why that outweighs the buffer it costs.
    const size_t denseFloats = size_t(getCount()) * mMaxLinks * 6u;
    if (!mLinkIncomingJointForceScratchDev)
    {
        prepareDeviceData((void**)&mLinkIncomingJointForceScratchDev, nullptr, denseFloats * sizeof(float),
                          "mLinkIncomingJointForceScratchDev");
        if (!mLinkIncomingJointForceScratchDev)
        {
            return false;
        }
    }
    // NOT zeroed, unlike getDofProjectedForcesOvStage above: fetchArtiLinkIncomingJointForce writes
    // every one of its getCount() * mMaxLinks slots, while the projection writes nothing for a fixed
    // joint and skips a locked axis. (CpuArticulationView zeroes BOTH rows -- its dense link pass
    // leaves padding untouched.) Padding here is written but meaningless; the read is safe only
    // because no emitted record can name a padding slot. A column whose records can needs the memset.

    TensorDesc denseDesc;
    denseDesc.device = mDevice;
    denseDesc.dtype = omni::physics::tensors::TensorDataType::eFloat32;
    denseDesc.numDims = 1;
    denseDesc.dims[0] = static_cast<int64_t>(denseFloats);
    denseDesc.data = mLinkIncomingJointForceScratchDev;
    if (!getLinkIncomingJointForce(&denseDesc))
    {
        return false;
    }

    if (!fetchArtiLinkVectorOvStage(static_cast<float*>(dstTensor->data), mLinkIncomingJointForceScratchDev, numOutputs,
                                    mMaxLinks, 6u, recordsDev))
    {
        CARB_LOG_ERROR("%s: failed to fetch %s", __FUNCTION__, label);
        return false;
    }
    // No recordKernelDone: the gather reads mLinkIncomingJointForceScratchDev, which this view owns
    // privately, so there is no shared buffer to hand back. The dense getter that FILLED that scratch
    // records its own shared buffers (ADR-0008 Decision 7) before returning.
    return true;
}

bool GpuArticulationView::getTendonPropertyOvStage(const char* attribName,
                                                   const TensorDesc* dstTensor,
                                                   const PxArticulationGPUAPIReadType::Enum attribFlag,
                                                   PxU32 buffer,
                                                   void* scratchDev,
                                                   PxU32 structFloats,
                                                   PxU32 maxTendons,
                                                   TendonProperty prop,
                                                   const ArticulationTendonOvStageRecord* recordsDev,
                                                   PxU32 numOutputs,
                                                   CUevent syncEvent) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }
    if (numOutputs == 0)
    {
        return true; // nothing to emit -- not a failure
    }
    if (maxTendons == 0 || !scratchDev)
    {
        // The caller enumerated tendons, so a scene with no tendon slot means the records name
        // something this view cannot address.
        CARB_LOG_WARN("%s: articulation view has no tendon storage", attribName);
        return false;
    }
    if (!recordsDev)
    {
        CARB_LOG_ERROR("%s: null ovstage tendon records", attribName);
        return false;
    }
    const PxU32 comp = tendonPropertyComponents(prop);
    // Stated as a bounds check rather than as "spatial tendons have no limit": the gather resolves a
    // source at fieldOffset..+comp within a structFloats-wide element, so ANY property that does not
    // fit silently reads a neighbouring tendon's data. This guard covers whatever the table adds
    // without needing to know which properties those are.
    if (static_cast<PxU32>(prop) + comp > structFloats)
    {
        CARB_LOG_ERROR("%s: property at float %u+%u does not fit a %u-float tendon struct", attribName,
                       static_cast<PxU32>(prop), comp, structFloats);
        return false;
    }
    if (!checkTensorDevice(*dstTensor, mDevice, attribName, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, attribName, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, numOutputs * comp, attribName, __FUNCTION__))
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    // Both SYNCHRONIZE_CUDA calls here and below are no-ops unless g_forceCudaDeviceSync is set; they
    // exist so this gather can be bisected with that flag like every other one in the file.
    SYNCHRONIZE_CUDA();

    // One DirectGPU fill of the whole tendon block for the view, then a gather of the requested
    // property. Several tendon attributes refill the same buffer once per attribute rather than
    // sharing one fill, which keeps one entry point per attribute; tendon counts are far below DOF
    // counts, so share the fill only if a tendon-heavy scene ever says otherwise.
    //
    // startEvent, not nullptr: the buffer is shared, and a gather from a previous read of it may
    // still be running on our stream (ADR-0008 Decision 7).
    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                scratchDev, mArtiGpuIndicesDev, attribFlag, numArtis, mGpuSimData->kernelDoneEvent(buffer), syncEvent),
            syncEvent, __FUNCTION__))
    {
        return false;
    }

    SYNCHRONIZE_CUDA();

    // The buffer stops being read here, so this is where the next DirectGPU call may take over.
    if (!mGpuSimData->gatherThenRelease(buffer, [&] {
            return fetchArtiTendonPropertyOvStage(static_cast<float*>(dstTensor->data),
                                                  static_cast<const float*>(scratchDev), numOutputs, maxTendons,
                                                  structFloats, static_cast<PxU32>(prop), comp, recordsDev);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch %s attribute", attribName);
        return false;
    }

    // No host block, for the same reason as the DOF ovstage path: the read synchronizes once at the
    // end, and the streamWaitEvent above already orders the fill ahead of this gather (ADR-0008).
    return true;
}

// One event per PROPERTY, shared by both tendon kinds -- eArtiTendonStiffnesses serves a fixed and a
// spatial stiffness read alike, and the CopyEvent enum has no per-kind slots.
//
// The event only hands THIS fill's completion to our stream, and a read issues fill -> wait -> gather
// in program order on one thread, so two fills are never in flight together. What keeps a fill off a
// buffer another gather is still reading is the per-buffer kernelDoneEvent / recordKernelDone pair
// (ADR-0008 Decision 7), which IS keyed by buffer.
static PxU32 tendonCopyEvent(TendonProperty prop)
{
    switch (prop)
    {
    case TendonProperty::eStiffness:
        return CopyEvent::eArtiTendonStiffnesses;
    case TendonProperty::eDamping:
        return CopyEvent::eArtiTendonDampings;
    case TendonProperty::eLimitStiffness:
        return CopyEvent::eArtiTendonLimitStiffnesses;
    case TendonProperty::eLimit:
        return CopyEvent::eArtiTendonLimits;
    case TendonProperty::eRestLength:
        return CopyEvent::eArtiTendonRestLengths;
    case TendonProperty::eOffset:
        return CopyEvent::eArtiTendonOffsets;
    }
    return CopyEvent::eArtiTendonStiffnesses;
}

bool GpuArticulationView::getFixedTendonPropertiesOvStage(const ArticulationTendonOvStageRecord* recordsDev,
                                                          PxU32 numOutputs, TendonProperty prop,
                                                          const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    return getTendonPropertyOvStage("fixed tendon property (ovstage)", dstTensor,
                                    PxArticulationGPUAPIReadType::Enum::eFIXED_TENDON,
                                    SharedDeviceBuffer::eFixedTendonProperties,
                                    (void*)mGpuSimData->mFixedTendonPropertiesDev,
                                    sizeof(PxGpuFixedTendonData) / sizeof(float),
                                    mGpuSimData->mMaxFixedTendons, prop, recordsDev, numOutputs,
                                    mGpuSimData->mCopyEvents[tendonCopyEvent(prop)]);
}

bool GpuArticulationView::getSpatialTendonPropertiesOvStage(const ArticulationTendonOvStageRecord* recordsDev,
                                                            PxU32 numOutputs, TendonProperty prop,
                                                            const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    return getTendonPropertyOvStage("spatial tendon property (ovstage)", dstTensor,
                                    PxArticulationGPUAPIReadType::Enum::eSPATIAL_TENDON,
                                    SharedDeviceBuffer::eSpatialTendonProperties,
                                    (void*)mGpuSimData->mSpatialTendonPropertiesDev,
                                    sizeof(PxGpuSpatialTendonData) / sizeof(float),
                                    mGpuSimData->mMaxSpatialTendons, prop, recordsDev, numOutputs,
                                    mGpuSimData->mCopyEvents[tendonCopyEvent(prop)]);
}

bool GpuArticulationView::setTendonPropertyOvStage(const char* const attribName,
                                                  const TensorDesc* const srcTensor,
                                                  const PxArticulationGPUAPIReadType::Enum readFlag,
                                                  const PxArticulationGPUAPIWriteType::Enum writeFlag,
                                                  const PxU32 buffer,
                                                  void* const scratchDev,
                                                  const PxU32 structFloats,
                                                  const PxU32 maxTendons,
                                                  const TendonProperty prop,
                                                  const ArticulationTendonOvStageRecord* const recordsDev,
                                                  const PxU32 numOutputs)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (!srcTensor || !srcTensor->data)
        return false;
    if (numOutputs == 0)
        return true; // nothing to publish -- not a failure
    if (maxTendons == 0 || !scratchDev)
    {
        CARB_LOG_WARN("%s: articulation view has no tendon storage", attribName);
        return false;
    }
    if (!recordsDev)
    {
        CARB_LOG_ERROR("%s: null ovstage tendon records", attribName);
        return false;
    }
    const PxU32 comp = tendonPropertyComponents(prop);
    // The same bounds check the gather makes, and it matters MORE here: a property that does not fit
    // the struct would have the scatter write over a NEIGHBOURING tendon's data rather than merely
    // read it. That covers limit and rest length on a spatial tendon -- the case the schema makes
    // impossible -- and anything a future table edit adds, without this needing to know which.
    if (static_cast<PxU32>(prop) + comp > structFloats)
    {
        CARB_LOG_ERROR("%s: property at float %u+%u does not fit a %u-float tendon struct", attribName,
                       static_cast<PxU32>(prop), comp, structFloats);
        return false;
    }
    if (!checkTensorDevice(*srcTensor, mDevice, attribName, __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, attribName, __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, numOutputs * comp, attribName, __FUNCTION__))
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    PxScene* scene = mGpuSimData->mScene;
    const PxU32 numArtis = getCount();

    // Read-modify-write. Status IS checked: PhysX refuses these calls in states the caller cannot
    // see and writes NOTHING when it does; the scratch is not zeroed, so an unchecked refusal would
    // overlay the caller's values onto stale contents and publish that as tendon state.
    CUevent readEvent = mGpuSimData->mCopyEvents[tendonCopyEvent(prop)];
    if (!scene->getDirectGPUAPI().getArticulationData(scratchDev, mArtiGpuIndicesDev, readFlag, numArtis,
                                                      mGpuSimData->kernelDoneEvent(buffer), readEvent))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::getArticulationData refused the read that preserves the tendon "
                       "properties sharing a struct with this one",
                       attribName);
        return false;
    }
    CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(readEvent), 0, nullptr));

    if (!submitArtiTendonPropertyOvStage(static_cast<float*>(scratchDev),
                                         static_cast<const float*>(srcTensor->data), numOutputs, maxTendons,
                                         structFloats, static_cast<PxU32>(prop), comp, recordsDev))
    {
        CARB_LOG_ERROR("%s: ovstage tendon scatter failed", attribName);
        return false;
    }

    // Ordered on the device, not by a host block: record our completion and hand it to PhysX as the
    // start event, the convention the rest of this class follows.
    CHECK_CU(getCudaShim()->eventRecord(
        reinterpret_cast<uintptr_t>(mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiTendonProperties]),
        uintptr_t(0), nullptr));
    // mArtiGpuIndicesDev, not a compacted list: the whole view is pushed and the articulations this
    // query did not match are written their own current values, which is why the RMW above covers
    // the whole view. A subset write would need an index list whose LENGTH comes off the host.
    scene->getDirectGPUAPI().setArticulationData(
        scratchDev, mArtiGpuIndicesDev, writeFlag, numArtis,
        mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiTendonProperties],
        mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiTendonProperties]);
    // PhysX's apply above reads the buffer ASYNC (finishEvent = mApplySignalEvents); recordKernelDone
    // only captures our scatter, so order our stream behind the apply before releasing -- otherwise the
    // next producer, gated on kernelDoneEvent, would overwrite the buffer mid-apply (ADR-0008 D7, the
    // reverse direction the host block used to cover).
    CHECK_CU(getCudaShim()->streamWaitEvent(
        uintptr_t(0),
        reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiTendonProperties]), 0, nullptr));
    mGpuSimData->recordKernelDone(buffer);
    return true;
}

bool GpuArticulationView::setFixedTendonPropertiesOvStage(const TensorDesc* srcTensor,
                                                          const ArticulationTendonOvStageRecord* recordsDev,
                                                          const PxU32 numOutputs, const TendonProperty prop)
{
    return setTendonPropertyOvStage("fixed tendon property (ovstage write)", srcTensor,
                                    PxArticulationGPUAPIReadType::Enum::eFIXED_TENDON,
                                    PxArticulationGPUAPIWriteType::Enum::eFIXED_TENDON,
                                    SharedDeviceBuffer::eFixedTendonProperties,
                                    (void*)mGpuSimData->mFixedTendonPropertiesDev,
                                    sizeof(PxGpuFixedTendonData) / sizeof(float),
                                    mGpuSimData->mMaxFixedTendons, prop, recordsDev, numOutputs);
}

bool GpuArticulationView::setSpatialTendonPropertiesOvStage(const TensorDesc* srcTensor,
                                                            const ArticulationTendonOvStageRecord* recordsDev,
                                                            const PxU32 numOutputs, const TendonProperty prop)
{
    return setTendonPropertyOvStage("spatial tendon property (ovstage write)", srcTensor,
                                    PxArticulationGPUAPIReadType::Enum::eSPATIAL_TENDON,
                                    PxArticulationGPUAPIWriteType::Enum::eSPATIAL_TENDON,
                                    SharedDeviceBuffer::eSpatialTendonProperties,
                                    (void*)mGpuSimData->mSpatialTendonPropertiesDev,
                                    sizeof(PxGpuSpatialTendonData) / sizeof(float),
                                    mGpuSimData->mMaxSpatialTendons, prop, recordsDev, numOutputs);
}

bool GpuArticulationView::setDofVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(srcTensor);

    return setDofAttribute("DOF velocity", srcTensor, indexTensor, PxArticulationGPUAPIWriteType::Enum::eJOINT_VELOCITY,
                           mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiDofVelocities],
                           mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiDofVelocities]);
}

bool GpuArticulationView::setDofActuationForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(srcTensor);
    if (setDofAttribute("DOF actuation force", srcTensor, indexTensor, PxArticulationGPUAPIWriteType::Enum::eJOINT_FORCE,
                        mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiDofForces],
                        mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiDofForces]))
    {
        mGpuSimData->mArtiDofForcesApplied = true;
    }
    else
    {
        return false;
    }

    return true;
}

bool GpuArticulationView::getDofPositionTargets(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    return getDofAttribute("DOF position target", dstTensor,
                           PxArticulationGPUAPIReadType::Enum::eJOINT_TARGET_POSITION,
                           mGpuSimData->mCopyEvents[CopyEvent::eArtiDofPositionTargets]);
}

bool GpuArticulationView::setDofPositionTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(srcTensor);

    return setDofAttribute("DOF position target", srcTensor, indexTensor,
                           PxArticulationGPUAPIWriteType::Enum::eJOINT_TARGET_POSITION,
                           mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiDofPositionTargets],
                           mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiDofPositionTargets]);
}

bool GpuArticulationView::getDofVelocityTargets(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    return getDofAttribute("DOF velocity target", dstTensor,
                           PxArticulationGPUAPIReadType::Enum::eJOINT_TARGET_VELOCITY,
                           mGpuSimData->mCopyEvents[CopyEvent::eArtiDofVelocityTargets]);
}

bool GpuArticulationView::setDofVelocityTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(srcTensor);

    return setDofAttribute("DOF velocity target", srcTensor, indexTensor,
                           PxArticulationGPUAPIWriteType::Enum::eJOINT_TARGET_VELOCITY,
                           mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiDofVelocityTargets],
                           mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiDofVelocityTargets]);
}

bool GpuArticulationView::getDofActuationForces(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);
    // NOTE: for now it was decided to use the staging buffer to get the applied actuation forces.
    // because the PxArticulationGpuDataType::eJOINT_FORCE handles a write-only physX buffer.
    // However, this needs to be addressed with the state reset issue later on.
    return getDofAttribute("DOF actuation force", dstTensor, 
                           PxArticulationGPUAPIReadType::Enum::eJOINT_FORCE,
                           mGpuSimData->mCopyEvents[CopyEvent::eArtiDofActuationForces]);
}

bool GpuArticulationView::updateCMassData()
{
    PxU32 numArtis = getCount();
    for (PxU32 i = 0; i < numArtis; i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
        {

            cMassLocalPosePos[i * mMaxLinks + j] = mEntries[i].links[j]->getCMassLocalPose().p;
        }
    }

    if (!CHECK_CUDA(cudaMemcpy(
            cMassLocalPosePosDev, cMassLocalPosePos.data(), mLinkBufSize * sizeof(PxVec3), cudaMemcpyHostToDevice)))
    {
        return false;
    }
    setComsCacheStateValid(true);
    return true;
}

bool GpuArticulationView::applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                                          const TensorDesc* srcTorqueTensor,
                                                          const TensorDesc* srcPositionTensor,
                                                          const TensorDesc* indexTensor,
                                                          const bool isGlobal)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);

    bool validForceTensor = false;
    bool validTorqueTensor = false;
    bool validPositionTensor = false;
    bool hasForce = srcForceTensor ? srcForceTensor->data != nullptr : false;
    bool hasTorque = srcTorqueTensor ? srcTorqueTensor->data != nullptr : false;
    bool hasPosition = srcPositionTensor ? srcPositionTensor->data != nullptr : false;
    const PxVec3* forceData = nullptr;
    const PxVec3* torqueData = nullptr;
    const PxVec3* positionData = nullptr;
    PxU32 numArtis = getCount();

    if (!hasForce && !hasTorque)
    {
        return false;
    }
    // skip if tensors are defined but are ill-defined
    if (hasForce)
    {
        validForceTensor = checkTensorDevice(*srcForceTensor, mDevice, "force", __FUNCTION__) &&
                           checkTensorFloat32(*srcForceTensor, "force", __FUNCTION__) &&
                           checkTensorSizeExact(*srcForceTensor, getCount() * mMaxLinks * 3u, "force", __FUNCTION__);
        if (!validForceTensor)
            return false;
        forceData = static_cast<const PxVec3*>(srcForceTensor->data);
    }

    if (hasTorque)
    {
        validTorqueTensor = checkTensorDevice(*srcTorqueTensor, mDevice, "torque", __FUNCTION__) &&
                            checkTensorFloat32(*srcTorqueTensor, "torque", __FUNCTION__) &&
                            checkTensorSizeExact(*srcTorqueTensor, getCount() * mMaxLinks * 3u, "torque", __FUNCTION__);
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
        validPositionTensor =
            checkTensorDevice(*srcPositionTensor, mDevice, "torque", __FUNCTION__) &&
            checkTensorFloat32(*srcPositionTensor, "torque", __FUNCTION__) &&
            checkTensorSizeExact(*srcPositionTensor, getCount() * mMaxLinks * 3u, "torque", __FUNCTION__);
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
        indices = mViewIndicesDev;
        numIndices = getCount();
    }

    PxScene* scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    // copy body data if forces are in local coordinates. We need the body coordinates for local->global transformation
    if (!isGlobal || (validPositionTensor && validForceTensor))
    {
        CUevent artiCopyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiLinkTransforms];
        if (!mGpuSimData->awaitDirectGpuFetch(
                scene->getDirectGPUAPI().getArticulationData(
                    (void*)mGpuSimData->mLinkOrRootTransformsDev, mArtiGpuIndicesDev,
                    PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, numArtis,
                    mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eLinkOrRootTransforms), artiCopyEvent),
                artiCopyEvent, __FUNCTION__))
        {
            return false;
        }
    }

    // will keep this until direct GPU API for articulation link mass properties is available
    // This does not have to be done if setCMassLocalPose is not called
    if (validPositionTensor && validForceTensor && numArtis > 0 && !getComsCacheStateValid())
        updateCMassData();

    if (!CHECK_CUDA(cudaMemset(mDirtyArtiGpuIndicesDev, 0, numArtis * sizeof(PxArticulationGPUIndex))))
    {
        return false;
    }
    mGpuSimData->clearForces();

    SYNCHRONIZE_CUDA();
    // submitArtiLinkForces reads the link transforms, so the next DirectGPU fill of that buffer has
    // to wait for it (ADR-0008 Decision 7). Unconditional -- see recordKernelDone.
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eLinkOrRootTransforms, [&] {
            return submitArtiLinkForces(mGpuSimData->mLinkForcesDev, mGpuSimData->mLinkTorquesDev,
                                        mDirtyArtiGpuIndicesDev, mGpuSimData->mLinkOrRootTransformsDev,
                                        cMassLocalPosePosDev, forceData, torqueData, positionData, indices, numIndices,
                                        numIndices * mMaxLinks, mGpuSimData->mMaxLinks, mLinkRecordsDev, isGlobal,
                                        validForceTensor, validTorqueTensor, validPositionTensor);
        }))
    {
        CARB_LOG_ERROR("Failed to submit articulation link forces");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    SYNCHRONIZE_CUDA();
    if (validForceTensor)
    {
        CHECK_CU(getCudaShim()->eventRecord(reinterpret_cast<uintptr_t>(mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiLinkForces]), uintptr_t(0), nullptr));
        scene->getDirectGPUAPI().setArticulationData((void*) mGpuSimData->mLinkForcesDev, mDirtyArtiGpuIndicesDev,
                                                     PxArticulationGPUAPIWriteType::eLINK_FORCE, numIndices,
                                                     mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiLinkForces],
                                                     mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkForces]);
        CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkForces]), nullptr));
        mGpuSimData->mLinkForcesApplied = true;
    }
    if (validPositionTensor || validTorqueTensor)
    {
        CHECK_CU(getCudaShim()->eventRecord(reinterpret_cast<uintptr_t>(mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiLinkTorques]), uintptr_t(0), nullptr));
        scene->getDirectGPUAPI().setArticulationData((void*) mGpuSimData->mLinkTorquesDev, mDirtyArtiGpuIndicesDev,
                                                     PxArticulationGPUAPIWriteType::eLINK_TORQUE, numIndices,
                                                     mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiLinkTorques],
                                                     mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkTorques]);
        CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkTorques]), nullptr));
        mGpuSimData->mLinkTorquesApplied = true;
    }
    SYNCHRONIZE_CUDA();

    return true;
}


bool GpuArticulationView::getJacobians(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    uint32_t jacobianRows = 0;
    uint32_t jacobianCols = 0;

    // this will fail if view is not homogeneous
    if (!getJacobianShape(&jacobianRows, &jacobianCols))
    {
        return false;
    }

    uint32_t jacobianSize = jacobianRows * jacobianCols;
    const PxU32 rootDofs = mEntries[0].metatype->getFixedBase() ? 0u : 6u;
    const bool applyBodyOrderSign = mEntries[0].metatype->hasReversedDofBodyOrder();

    if (!checkTensorDevice(*dstTensor, mDevice, "Jacobian", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Jacobian", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * jacobianSize, "Jacobian", __FUNCTION__))
    {
        return false;
    }

    if (!checkInverseDynamicsScratch(mGpuSimData->mJacobianDataDev, "articulation jacobian", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    SYNCHRONIZE_CUDA();

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiJacobians];

    if (!mGpuSimData->awaitDirectGpuFetch(scene->getDirectGPUAPI().computeArticulationData(
                                              (void*)mGpuSimData->mJacobianDataDev, mArtiGpuIndicesDev,
                                              PxArticulationGPUAPIComputeType::eDENSE_JACOBIANS, getCount(),
                                              mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eJacobianData), copyEvent),
                                          copyEvent, __FUNCTION__))
    {
        return false;
    }
    SYNCHRONIZE_CUDA();
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eJacobianData, [&] {
            return fetchArtiJacobian(static_cast<float*>(dstTensor->data), mGpuSimData->mJacobianDataDev,
                                     getCount() * jacobianRows * jacobianCols, jacobianRows * jacobianCols,
                                     mGpuSimData->mJacobianMaxRows * mGpuSimData->mJacobianMaxCols, jacobianCols,
                                     rootDofs, 0u, mDofRecordsDev, applyBodyOrderSign);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch Jacobian tensor");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

bool GpuArticulationView::getGeneralizedMassMatrices(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    const bool isFixedBase = mEntries[0].metatype->getFixedBase();
    const PxU32 simMassMatrixSize = (mGpuSimData->mMaxDofs + 6) * (mGpuSimData->mMaxDofs + 6);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    uint32_t massMatrixRows = 0;
    uint32_t massMatrixCols = 0;

    // this will fail if view is not homogeneous
    if (!getGeneralizedMassMatrixShape(&massMatrixRows, &massMatrixCols))
    {
        return false;
    }

    uint32_t massMatrixSize = massMatrixRows * massMatrixCols;
    const bool applyBodyOrderSign = mEntries[0].metatype->hasReversedDofBodyOrder();

    if (!checkTensorDevice(*dstTensor, mDevice, "Mass Matrix", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Mass Matrix", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * massMatrixSize, "Mass Matrix", __FUNCTION__))
    {
        return false;
    }

    if (!checkInverseDynamicsScratch(mGpuSimData->mMassMatrixDataDev, "articulation mass matrix", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    SYNCHRONIZE_CUDA();

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiMassMatrices];

    // Through the helper, like every other producer: checking the status alone left two holes --
    // returning on refusal without draining an event PhysX may already have recorded, and treating a
    // failed streamWaitEvent as a log line while the gather ran anyway.
    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().computeArticulationData(
                (void*)mGpuSimData->mMassMatrixDataDev, mArtiGpuIndicesDev, PxArticulationGPUAPIComputeType::eMASS_MATRICES,
                getCount(), mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eMassMatrixData), copyEvent),
            copyEvent, __FUNCTION__))
    {
        CARB_LOG_ERROR("Failed to compute generalized mass matrices");
        return false;
    }

    SYNCHRONIZE_CUDA();

    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eMassMatrixData, [&] {
            return fetchArtiMassMatrices(static_cast<float*>(dstTensor->data), mGpuSimData->mMassMatrixDataDev,
                                         getCount() * massMatrixSize, massMatrixSize, simMassMatrixSize, massMatrixCols,
                                         isFixedBase ? 0u : 6u, 0u, mDofRecordsDev, applyBodyOrderSign);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch generalized mass matrices attribute");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

bool GpuArticulationView::getArticulationMassCenter(const TensorDesc* dstTensor, bool localFrame) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }


    if (!checkTensorDevice(*dstTensor, mDevice, "Articulation Mass Center", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Articulation Mass Center", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 3, "Articulation Mass Center", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;
    
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    SYNCHRONIZE_CUDA();

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiMassCenter];
    if (!copyEvent)
    {
        CARB_LOG_ERROR("Missing synchronization event for articulation mass centers");
        return false;
    }
    // Zeroed for the reason spelled out in getMassCentersOvStage above: the PhysX COM kernel
    // accumulates into its destination. Same stream as the start event, so the zeroing precedes the
    // DirectGPU compute.
    if (!CHECK_CUDA(cudaMemsetAsync(dstTensor->data, 0, size_t(getCount()) * sizeof(PxVec3), nullptr)) ||
        !recordOvStageReady(nullptr))
    {
        return false;
    }
    if (!scene->getDirectGPUAPI().computeArticulationData(
            (void*)(dstTensor->data), mArtiGpuIndicesDev,
            localFrame ? PxArticulationGPUAPIComputeType::eARTICULATION_COMS_ROOT_FRAME :
                         PxArticulationGPUAPIComputeType::eARTICULATION_COMS_WORLD_FRAME,
            getCount(), mOvStageSelectionReadyEvent, copyEvent))
    {
        CARB_LOG_ERROR("Failed to compute articulation mass centers");
        waitForDirectGpuFinish(copyEvent, "articulation mass centers");
        return false;
    }

    // The COM work runs on PhysX's own non-blocking stream and its completion is carried by copyEvent,
    // so draining the null stream says nothing about it: without this wait the call returns before
    // dstTensor is filled.
    if (!waitForDirectGpuFinish(copyEvent, "articulation mass centers"))
    {
        return false;
    }

    if (!localFrame &&
        !applySubspaceOriginArtiMassCentersOvStage(static_cast<PxVec3*>(dstTensor->data), getCount(), mRootRecordsDev))
    {
        CARB_LOG_ERROR("Failed to apply subspace origin to articulation mass centers");
        return false;
    }

    if (!CHECK_CUDA(cudaStreamSynchronize(nullptr)))
        return false;

    return true;
}

bool GpuArticulationView::getArticulationCentroidalMomentum(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);


    if (!requireUniformBaseType(__FUNCTION__))
    {
        return false;
    }

    // Heterogeneous views are not supported here, and unlike the per-DOF readers this one cannot
    // degrade to padding: PhysX sizes centroidalMomentumMatrix 6 x (own dofs + 6), so a view whose
    // articulations differ has no single row stride to read it with. getGeneralizedMassMatrices
    // refuses such a view for the same reason, via getGeneralizedMassMatrixShape.
    if (!isHomogeneous())
    {
        CARB_LOG_ERROR("%s: the articulations in this view are not homogeneous. Centroidal momentum is read with one row stride for the whole view, which cannot describe articulations of differing size. Build one view per articulation type.",
                       __FUNCTION__);
        return false;
    }

    const bool isFixedBase = mEntries[0].metatype->getFixedBase();
    if (isFixedBase)
    {
        CARB_LOG_ERROR("Articulation has fixed base, centroidal momentum is not defined");
        return false;
    }

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "Articulation Centroidal Momentum", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Articulation Centroidal Momentum", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 6 * (mMaxDofs + 7), "Articulation Centroidal Momentum", __FUNCTION__))
    {
        return false;
    }

    if (!checkInverseDynamicsScratch(mGpuSimData->mCentroidalMomentumDataDev, "articulation centroidal momentum",
                                     __FUNCTION__))
    {
        return false;
    }

    const PxU32 centroidalMomentumMatricesBlockSize = 6 * (mMaxDofs + 7);
    const PxU32 simCentroidalMomentumMatricesBlockSize = 6 * (mGpuSimData->mMaxDofs + 6);
    const PxU32 simMassMatricesBlockSize = (mGpuSimData->mMaxDofs + 6) * (mGpuSimData->mMaxDofs + 6);
    const PxU32 simCoriolisForcesBlockSize = mGpuSimData->mMaxDofs + 6;
    const PxU32 simStartCoriolisForces = simMassMatricesBlockSize * getCount();
    const PxU32 simStartCentroidalMomentumMatrix = (simMassMatricesBlockSize + simCoriolisForcesBlockSize) * getCount();
	const PxU32 startSimBiasForceBlock = simCentroidalMomentumMatricesBlockSize * getCount();
    const bool applyBodyOrderSign = mEntries[0].metatype->hasReversedDofBodyOrder();

    PxScene* scene = mGpuSimData->mScene;
    
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    SYNCHRONIZE_CUDA();

    CUevent copyEventMass = mGpuSimData->mCopyEvents[CopyEvent::eArtiMassMatrices];
    if (!scene->getDirectGPUAPI().computeArticulationData(
            (void*)mGpuSimData->mCentroidalMomentumDataDev, mArtiGpuIndicesDev,
            PxArticulationGPUAPIComputeType::eMASS_MATRICES, getCount(),
            mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eCentroidalMomentumData), copyEventMass))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI refused the eMASS_MATRICES compute", __FUNCTION__);
        // Drained even on refusal: PhysX may have queued work and recorded the event anyway, and
        // the scratch is shared with the two fills around this one.
        CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(copyEventMass), nullptr));
        return false;
    }
    CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(copyEventMass), nullptr));
    CUevent copyEventCoriolis = mGpuSimData->mCopyEvents[CopyEvent::eArtiCoriolisCentrifugal];
    if (!scene->getDirectGPUAPI().computeArticulationData(
            (void*)(mGpuSimData->mCentroidalMomentumDataDev + simStartCoriolisForces), mArtiGpuIndicesDev,
            PxArticulationGPUAPIComputeType::eCORIOLIS_AND_CENTRIFUGAL_COMPENSATION, getCount(),
            mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eCentroidalMomentumData), copyEventCoriolis))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI refused the eCORIOLIS_AND_CENTRIFUGAL_COMPENSATION compute", __FUNCTION__);
        // Drained even on refusal: PhysX may have queued work and recorded the event anyway, and
        // the scratch is shared with the two fills around this one.
        CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(copyEventCoriolis), nullptr));
        return false;
    }
    CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(copyEventCoriolis), nullptr));
    // Each fill signals its own event, so the host blocks between fills can be dropped later without
    // first untangling which event carries which fill's completion.
    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiCentroidalMomentum];
    if (!scene->getDirectGPUAPI().computeArticulationData(
            (void*)mGpuSimData->mCentroidalMomentumDataDev, mArtiGpuIndicesDev,
            PxArticulationGPUAPIComputeType::eCENTROIDAL_MOMENTUM_MATRICES, getCount(),
            mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eCentroidalMomentumData), copyEvent))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI refused the eCENTROIDAL_MOMENTUM_MATRICES compute", __FUNCTION__);
        // Drained even on refusal: PhysX may have queued work and recorded the event anyway, and
        // the scratch is shared with the two fills around this one.
        CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(copyEvent), nullptr));
        return false;
    }
    CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(copyEvent), nullptr));

    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eCentroidalMomentumData, [&] {
            return fetchArtiCentroidalMomentumMatrices(
                static_cast<float*>(dstTensor->data),
                mGpuSimData->mCentroidalMomentumDataDev + simStartCentroidalMomentumMatrix,
                getCount() * centroidalMomentumMatricesBlockSize, mMaxDofs, centroidalMomentumMatricesBlockSize,
                simCentroidalMomentumMatricesBlockSize, startSimBiasForceBlock, 0u, mDofRecordsDev, applyBodyOrderSign);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch centroidal momentum data");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

bool GpuArticulationView::getCoriolisAndCentrifugalCompensationForces(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!requireUniformBaseType(__FUNCTION__))
    {
        return false;
    }

    const bool isFixedBase = mEntries[0].metatype->getFixedBase();
    const PxU32 maxDofs = isFixedBase ? mMaxDofs : mMaxDofs + 6;
    // Not a dof count: the scene-wide dof maximum plus the floating base's six.
    const PxU32 simGeneralizedCoords = mGpuSimData->mMaxDofs + 6;
    const bool hasRootDofs = !isFixedBase;

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "Coriolis and centrifugal forces", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Coriolis and centrifugal forces", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * maxDofs, "Coriolis and centrifugal forces", __FUNCTION__))
    {
        return false;
    }

    if (!checkInverseDynamicsScratch(mGpuSimData->mCoriolisGravityDataDev,
                                     "articulation coriolis and centrifugal forces", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;

    SYNCHRONIZE_CUDA();

    // Context guard must be acquired before calling computeArticulationData below.
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiCoriolisCentrifugal];

    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().computeArticulationData(
                (void*)mGpuSimData->mCoriolisGravityDataDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIComputeType::eCORIOLIS_AND_CENTRIFUGAL_COMPENSATION, getCount(),
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eCoriolisGravityData), copyEvent),
            copyEvent, __FUNCTION__))
    {
        return false;
    }

    SYNCHRONIZE_CUDA();

    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eCoriolisGravityData, [&] {
            return fetchArtiDofAttributeGravityAndCoriolis(static_cast<float*>(dstTensor->data),
                                                           mGpuSimData->mCoriolisGravityDataDev, getCount() * maxDofs,
                                                           maxDofs, simGeneralizedCoords, mDofRecordsDev, hasRootDofs);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch coriolis and centrifugal forces attribute");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

bool GpuArticulationView::getGravityCompensationForces(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!requireUniformBaseType(__FUNCTION__))
    {
        return false;
    }

    const bool isFixedBase = mEntries[0].metatype->getFixedBase();
    const PxU32 maxDofs = isFixedBase ? mMaxDofs : mMaxDofs + 6;
    // Not a dof count: the scene-wide dof maximum plus the floating base's six.
    const PxU32 simGeneralizedCoords = mGpuSimData->mMaxDofs + 6;
    const bool hasRootDofs = !isFixedBase;

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "gravity compensation forces", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "gravity compensation forces", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * maxDofs, "gravity compensation forces", __FUNCTION__))
    {
        return false;
    }

    if (!checkInverseDynamicsScratch(mGpuSimData->mCoriolisGravityDataDev,
                                     "articulation gravity compensation forces", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    SYNCHRONIZE_CUDA();

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiGeneralizedGravity];

    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().computeArticulationData(
                (void*)mGpuSimData->mCoriolisGravityDataDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIComputeType::eGRAVITY_COMPENSATION, getCount(),
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eCoriolisGravityData), copyEvent),
            copyEvent, __FUNCTION__))
    {
        return false;
    }

    SYNCHRONIZE_CUDA();

    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eCoriolisGravityData, [&] {
            return fetchArtiDofAttributeGravityAndCoriolis(static_cast<float*>(dstTensor->data),
                                                           mGpuSimData->mCoriolisGravityDataDev, getCount() * maxDofs,
                                                           maxDofs, simGeneralizedCoords, mDofRecordsDev, hasRootDofs);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch gravity compensation forces attribute");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

bool GpuArticulationView::getLinkIncomingJointForce(const TensorDesc* dstTensor) const
{
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "link incoming joint force", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "link incoming joint force", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxLinks * 6u, "link incoming joint force", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    // Pre-simulate (step count == 0), the GPU linkIncomingJointForces buffer is
    // uninitialized; mirror the CPU cache path's dt==0 PxMemZero so the readback
    // returns zeros until the first simulate() populates the buffer.
    if (SimulationBackend* backend = GetSimulationBackend())
    {
        if (backend->getStepCount() == 0)
        {
            if (!CHECK_CUDA(cudaMemset(dstTensor->data, 0,
                                       getCount() * mMaxLinks * 6u * sizeof(float))))
            {
                return false;
            }
            return true;
        }
    }

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiLinkIncomingJointForce];
    CUevent copyEventTransforms = mGpuSimData->mCopyEvents[CopyEvent::eArtiLinkTransforms];
    // Two producers into two buffers, each checked against its OWN finish event: pairing the pose
    // fetch with the force event would order the gather behind the wrong copy.
    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)mGpuSimData->mLinkIncomingJointForceDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIReadType::eLINK_INCOMING_JOINT_FORCE, numArtis,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eLinkIncomingJointForce), copyEvent),
            copyEvent, __FUNCTION__))
    {
        return false;
    }
    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)mGpuSimData->mLinkOrRootTransformsDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, numArtis,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eLinkOrRootTransforms), copyEventTransforms),
            copyEventTransforms, __FUNCTION__))
    {
        return false;
    }
    CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(copyEventTransforms), 0, nullptr));
    SYNCHRONIZE_CUDA();

    // Our kernel is done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded unconditionally: every kernel that touches a buffer records it.
    if (!mGpuSimData->gatherThenRelease(
            SharedDeviceBuffer::eLinkIncomingJointForce, SharedDeviceBuffer::eLinkOrRootTransforms, [&] {
                return fetchArtiLinkIncomingJointForce(static_cast<PhysxGpuSpatialForces*>(dstTensor->data),
                                                       mGpuSimData->mLinkIncomingJointForceDev,
                                                       mGpuSimData->mLinkOrRootTransformsDev, getCount() * mMaxLinks,
                                                       mMaxLinks, mGpuSimData->mMaxLinks, mLinkRecordsDev);
            }))
    {
        CARB_LOG_ERROR("Failed to fetch articulation link incoming joint forces");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

bool GpuArticulationView::getDofProjectedJointForces(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "dof projected joint force", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "dof projected joint force", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "dof projected joint force", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    // Pre-simulate (step count == 0), the GPU spatial-force source buffer is
    // uninitialized; mirror CPU's dt==0 PxMemZero so the readback returns
    // zeros until the first simulate() populates the buffer.
    if (SimulationBackend* backend = GetSimulationBackend())
    {
        if (backend->getStepCount() == 0)
        {
            if (!CHECK_CUDA(cudaMemset(dstTensor->data, 0,
                                       getCount() * mMaxDofs * sizeof(float))))
            {
                return false;
            }
            return true;
        }
    }

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiLinkIncomingJointForce];
    CUevent copyEventTransforms = mGpuSimData->mCopyEvents[CopyEvent::eArtiLinkTransforms];
    // Two producers into two buffers, each checked against its OWN finish event: pairing the pose
    // fetch with the force event would order the gather behind the wrong copy.
    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)mGpuSimData->mLinkIncomingJointForceDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIReadType::eLINK_INCOMING_JOINT_FORCE, numArtis,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eLinkIncomingJointForce), copyEvent),
            copyEvent, __FUNCTION__))
    {
        return false;
    }
    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)mGpuSimData->mLinkOrRootTransformsDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, numArtis,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eLinkOrRootTransforms), copyEventTransforms),
            copyEventTransforms, __FUNCTION__))
    {
        return false;
    }
    CHECK_CU(getCudaShim()->streamWaitEvent(uintptr_t(0), reinterpret_cast<uintptr_t>(copyEventTransforms), 0, nullptr));
    SYNCHRONIZE_CUDA();

    // Our kernel is done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded unconditionally: every kernel that touches a buffer records it.
    if (!mGpuSimData->gatherThenRelease(
            SharedDeviceBuffer::eLinkIncomingJointForce, SharedDeviceBuffer::eLinkOrRootTransforms, [&] {
                return fetchDofProjectionForce(static_cast<float*>(dstTensor->data),
                                               mGpuSimData->mLinkIncomingJointForceDev,
                                               mGpuSimData->mLinkOrRootTransformsDev, getCount() * mMaxLinks, mMaxLinks,
                                               mGpuSimData->mMaxLinks, mLinkRecordsDev);
            }))
    {
        CARB_LOG_ERROR("Failed to fetch dof projected joint force");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
}

bool GpuArticulationView::getFixedTendonStiffnesses(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "tendon stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "tendon stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons, "tendon stiffness", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiTendonStiffnesses];
    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)mGpuSimData->mFixedTendonPropertiesDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIReadType::eFIXED_TENDON, numArtis,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eFixedTendonProperties), copyEvent),
            copyEvent, __FUNCTION__))
    {
        return false;
    }
    SYNCHRONIZE_CUDA();

    // Our kernel is done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded unconditionally: every kernel that touches a buffer records it.
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eFixedTendonProperties, [&] {
            return fetchFixedTendonStiffness(static_cast<float*>(dstTensor->data),
                                             mGpuSimData->mFixedTendonPropertiesDev, getCount() * mMaxFixedTendons,
                                             mMaxFixedTendons, mGpuSimData->mMaxFixedTendons);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch fixed tendon stiffness");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
}

bool GpuArticulationView::getFixedTendonDampings(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "tendon damping", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "tendon damping", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons, "tendon damping", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiTendonDampings];
    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)mGpuSimData->mFixedTendonPropertiesDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIReadType::eFIXED_TENDON, numArtis,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eFixedTendonProperties), copyEvent),
            copyEvent, __FUNCTION__))
    {
        return false;
    }
    SYNCHRONIZE_CUDA();

    // Our kernel is done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded unconditionally: every kernel that touches a buffer records it.
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eFixedTendonProperties, [&] {
            return fetchFixedTendonDamping(static_cast<float*>(dstTensor->data), mGpuSimData->mFixedTendonPropertiesDev,
                                           getCount() * mMaxFixedTendons, mMaxFixedTendons,
                                           mGpuSimData->mMaxFixedTendons);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch fixed tendon damping");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
}

bool GpuArticulationView::getFixedTendonLimitStiffnesses(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "tendon limit stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "tendon limit stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons, "tendon limit stiffness", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiTendonLimitStiffnesses];
    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)mGpuSimData->mFixedTendonPropertiesDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIReadType::eFIXED_TENDON, numArtis,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eFixedTendonProperties), copyEvent),
            copyEvent, __FUNCTION__))
    {
        return false;
    }
    SYNCHRONIZE_CUDA();

    // Our kernel is done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded unconditionally: every kernel that touches a buffer records it.
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eFixedTendonProperties, [&] {
            return fetchFixedTendonLimitStiffness(static_cast<float*>(dstTensor->data),
                                                  mGpuSimData->mFixedTendonPropertiesDev, getCount() * mMaxFixedTendons,
                                                  mMaxFixedTendons, mGpuSimData->mMaxFixedTendons);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch fixed tendon limit stiffness");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
}

bool GpuArticulationView::getFixedTendonLimits(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "tendon limits", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "tendon limits", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons * 2u, "tendon limits", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiTendonLimits];
    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)mGpuSimData->mFixedTendonPropertiesDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIReadType::eFIXED_TENDON, numArtis,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eFixedTendonProperties), copyEvent),
            copyEvent, __FUNCTION__))
    {
        return false;
    }
    SYNCHRONIZE_CUDA();

    // Our kernel is done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded unconditionally: every kernel that touches a buffer records it.
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eFixedTendonProperties, [&] {
            return fetchFixedTendonLimits(static_cast<float*>(dstTensor->data), mGpuSimData->mFixedTendonPropertiesDev,
                                          getCount() * mMaxFixedTendons, mMaxFixedTendons, mGpuSimData->mMaxFixedTendons);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch fixed tendon limit");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
}

bool GpuArticulationView::getFixedTendonfixedSpringRestLengths(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "tendon rest lengths", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "tendon rest lengths", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons, "tendon rest lengths", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiTendonRestLengths];
    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)mGpuSimData->mFixedTendonPropertiesDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIReadType::eFIXED_TENDON, numArtis,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eFixedTendonProperties), copyEvent),
            copyEvent, __FUNCTION__))
    {
        return false;
    }
    SYNCHRONIZE_CUDA();

    // Our kernel is done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded unconditionally: every kernel that touches a buffer records it.
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eFixedTendonProperties, [&] {
            return fetchFixedTendonRestLength(static_cast<float*>(dstTensor->data),
                                              mGpuSimData->mFixedTendonPropertiesDev, getCount() * mMaxFixedTendons,
                                              mMaxFixedTendons, mGpuSimData->mMaxFixedTendons);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch fixed tendon rest length");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
}

bool GpuArticulationView::getFixedTendonOffsets(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "tendon offset", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "tendon offset", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons, "tendon offset", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiTendonOffsets];
    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)mGpuSimData->mFixedTendonPropertiesDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIReadType::eFIXED_TENDON, numArtis,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eFixedTendonProperties), copyEvent),
            copyEvent, __FUNCTION__))
    {
        return false;
    }
    SYNCHRONIZE_CUDA();

    // Our kernel is done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded unconditionally: every kernel that touches a buffer records it.
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eFixedTendonProperties, [&] {
            return fetchFixedTendonOffset(static_cast<float*>(dstTensor->data), mGpuSimData->mFixedTendonPropertiesDev,
                                          getCount() * mMaxFixedTendons, mMaxFixedTendons, mGpuSimData->mMaxFixedTendons);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch fixed tendon offset");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
}

bool GpuArticulationView::getSpatialTendonStiffnesses(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "tendon stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "tendon stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxSpatialTendons, "tendon stiffness", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiTendonStiffnesses];
    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)mGpuSimData->mSpatialTendonPropertiesDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIReadType::eSPATIAL_TENDON, numArtis,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eSpatialTendonProperties), copyEvent),
            copyEvent, __FUNCTION__))
    {
        return false;
    }
    SYNCHRONIZE_CUDA();

    // Our kernel is done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded unconditionally: every kernel that touches a buffer records it.
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eSpatialTendonProperties, [&] {
            return fetchSpatialTendonStiffness(static_cast<float*>(dstTensor->data),
                                               mGpuSimData->mSpatialTendonPropertiesDev, getCount() * mMaxSpatialTendons,
                                               mMaxSpatialTendons, mGpuSimData->mMaxSpatialTendons);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch spatial tendon stiffness");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
}

bool GpuArticulationView::getSpatialTendonDampings(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "tendon damping", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "tendon damping", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxSpatialTendons, "tendon damping", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiTendonDampings];
    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)mGpuSimData->mSpatialTendonPropertiesDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIReadType::eSPATIAL_TENDON, numArtis,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eSpatialTendonProperties), copyEvent),
            copyEvent, __FUNCTION__))
    {
        return false;
    }
    SYNCHRONIZE_CUDA();

    // Our kernel is done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded unconditionally: every kernel that touches a buffer records it.
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eSpatialTendonProperties, [&] {
            return fetchSpatialTendonDamping(static_cast<float*>(dstTensor->data),
                                             mGpuSimData->mSpatialTendonPropertiesDev, getCount() * mMaxSpatialTendons,
                                             mMaxSpatialTendons, mGpuSimData->mMaxSpatialTendons);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch spatial tendon damping");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
}

bool GpuArticulationView::getSpatialTendonLimitStiffnesses(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "tendon limit stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "tendon limit stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxSpatialTendons, "tendon limit stiffness", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiTendonLimitStiffnesses];
    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)mGpuSimData->mSpatialTendonPropertiesDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIReadType::eSPATIAL_TENDON, numArtis,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eSpatialTendonProperties), copyEvent),
            copyEvent, __FUNCTION__))
    {
        return false;
    }
    SYNCHRONIZE_CUDA();

    // Our kernel is done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded unconditionally: every kernel that touches a buffer records it.
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eSpatialTendonProperties, [&] {
            return fetchSpatialTendonLimitStiffness(
                static_cast<float*>(dstTensor->data), mGpuSimData->mSpatialTendonPropertiesDev,
                getCount() * mMaxSpatialTendons, mMaxSpatialTendons, mGpuSimData->mMaxSpatialTendons);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch spatial tendon limit stiffness");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
}

bool GpuArticulationView::getSpatialTendonOffsets(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "tendon offset", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "tendon offset", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxSpatialTendons, "tendon offset", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;
    PxU32 numArtis = getCount();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiTendonOffsets];
    if (!mGpuSimData->awaitDirectGpuFetch(
            scene->getDirectGPUAPI().getArticulationData(
                (void*)mGpuSimData->mSpatialTendonPropertiesDev, mArtiGpuIndicesDev,
                PxArticulationGPUAPIReadType::eSPATIAL_TENDON, numArtis,
                mGpuSimData->kernelDoneEvent(SharedDeviceBuffer::eSpatialTendonProperties), copyEvent),
            copyEvent, __FUNCTION__))
    {
        return false;
    }
    SYNCHRONIZE_CUDA();

    // Our kernel is done with these buffers; the next DirectGPU call on one may take over
    // (ADR-0008 Decision 7). Recorded unconditionally: every kernel that touches a buffer records it.
    if (!mGpuSimData->gatherThenRelease(SharedDeviceBuffer::eSpatialTendonProperties, [&] {
            return fetchSpatialTendonOffset(static_cast<float*>(dstTensor->data),
                                            mGpuSimData->mSpatialTendonPropertiesDev, getCount() * mMaxSpatialTendons,
                                            mMaxSpatialTendons, mGpuSimData->mMaxSpatialTendons);
        }))
    {
        CARB_LOG_ERROR("Failed to fetch spatial tendon offset");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
}

bool GpuArticulationView::setFixedTendonProperties(const TensorDesc* stiffnesses,
                                                   const TensorDesc* dampings,
                                                   const TensorDesc* limitStiffnesses,
                                                   const TensorDesc* limits,
                                                   const TensorDesc* restLengths,
                                                   const TensorDesc* offsets,
                                                   const TensorDesc* indexTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(stiffnesses);
    PASS_EMPTY_TENSOR(dampings);
    PASS_EMPTY_TENSOR(limitStiffnesses);
    PASS_EMPTY_TENSOR(limits);
    PASS_EMPTY_TENSOR(restLengths);
    PASS_EMPTY_TENSOR(offsets);

    if (!stiffnesses || !stiffnesses->data)
    {
        return false;
    }
    if (!dampings || !dampings->data)
    {
        return false;
    }
    if (!limitStiffnesses || !limitStiffnesses->data)
    {
        return false;
    }
    if (!limits || !limits->data)
    {
        return false;
    }
    if (!restLengths || !restLengths->data)
    {
        return false;
    }
    if (!offsets || !offsets->data)
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
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
        indices = mViewIndicesDev;
        numIndices = getCount();
    }

    if (!checkTensorDevice(*stiffnesses, mDevice, "tendon stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*stiffnesses, "tendon stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*stiffnesses, getCount() * mMaxFixedTendons, "tendon stiffness", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*dampings, mDevice, "tendon damping", __FUNCTION__) ||
        !checkTensorFloat32(*dampings, "tendon damping", __FUNCTION__) ||
        !checkTensorSizeExact(*dampings, getCount() * mMaxFixedTendons, "tendon damping", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*limitStiffnesses, mDevice, "tendon limit stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*limitStiffnesses, "tendon limit stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*limitStiffnesses, getCount() * mMaxFixedTendons, "tendon limit stiffness", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*limits, mDevice, "tendon limits", __FUNCTION__) ||
        !checkTensorFloat32(*limits, "tendon limits", __FUNCTION__) ||
        !checkTensorSizeExact(*limits, getCount() * mMaxFixedTendons * 2u, "tendon limits", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*restLengths, mDevice, "tendon rest length", __FUNCTION__) ||
        !checkTensorFloat32(*restLengths, "tendon rest length", __FUNCTION__) ||
        !checkTensorSizeExact(*restLengths, getCount() * mMaxFixedTendons, "tendon rest length", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*offsets, mDevice, "tendon offset", __FUNCTION__) ||
        !checkTensorFloat32(*offsets, "tendon offset", __FUNCTION__) ||
        !checkTensorSizeExact(*offsets, getCount() * mMaxFixedTendons, "tendon offset", __FUNCTION__))
    {
        return false;
    }
    CHECK_CUDA(cudaMemset(mDirtyArtiGpuIndicesDev, 0, getCount() * sizeof(PxArticulationGPUIndex)));
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    if (!submitArtiFixedTendonProperties(
             mGpuSimData->mFixedTendonPropertiesDev, static_cast<const float*>(stiffnesses->data),
            static_cast<const float*>(dampings->data), static_cast<const float*>(limitStiffnesses->data),
            static_cast<const float*>(limits->data), static_cast<const float*>(restLengths->data),
            static_cast<const float*>(offsets->data), indices, mDirtyArtiGpuIndicesDev, numIndices, mMaxFixedTendons,
            mGpuSimData->mMaxFixedTendons, mFixedTendonRecordsDev))
    {
        CARB_LOG_ERROR("Failed to submit fixed tendon properties");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    SYNCHRONIZE_CUDA();
    PxScene* scene = mGpuSimData->mScene;
    scene->getDirectGPUAPI().setArticulationData(
        (void*) mGpuSimData->mFixedTendonPropertiesDev, mDirtyArtiGpuIndicesDev, PxArticulationGPUAPIWriteType::eFIXED_TENDON,
        numIndices, mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiTendonProperties],
        mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiTendonProperties]);
    CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiTendonProperties]), nullptr));

    SYNCHRONIZE_CUDA();
    return true;
}

bool GpuArticulationView::setSpatialTendonProperties(const TensorDesc* stiffnesses,
                                                   const TensorDesc* dampings,
                                                   const TensorDesc* limitStiffnesses,
                                                   const TensorDesc* offsets,
                                                   const TensorDesc* indexTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(stiffnesses);
    PASS_EMPTY_TENSOR(dampings);
    PASS_EMPTY_TENSOR(limitStiffnesses);
    PASS_EMPTY_TENSOR(offsets);

    if (!stiffnesses || !stiffnesses->data)
    {
        return false;
    }
    if (!dampings || !dampings->data)
    {
        return false;
    }
    if (!limitStiffnesses || !limitStiffnesses->data)
    {
        return false;
    }
    if (!offsets || !offsets->data)
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
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
        indices = mViewIndicesDev;
        numIndices = getCount();
    }

    if (!checkTensorDevice(*stiffnesses, mDevice, "tendon stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*stiffnesses, "tendon stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*stiffnesses, getCount() * mMaxSpatialTendons, "tendon stiffness", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*dampings, mDevice, "tendon damping", __FUNCTION__) ||
        !checkTensorFloat32(*dampings, "tendon damping", __FUNCTION__) ||
        !checkTensorSizeExact(*dampings, getCount() * mMaxSpatialTendons, "tendon damping", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*limitStiffnesses, mDevice, "tendon limit stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*limitStiffnesses, "tendon limit stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*limitStiffnesses, getCount() * mMaxSpatialTendons, "tendon limit stiffness", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*offsets, mDevice, "tendon offset", __FUNCTION__) ||
        !checkTensorFloat32(*offsets, "tendon offset", __FUNCTION__) ||
        !checkTensorSizeExact(*offsets, getCount() * mMaxSpatialTendons, "tendon offset", __FUNCTION__))
    {
        return false;
    }
    CHECK_CUDA(cudaMemset(mDirtyArtiGpuIndicesDev, 0, getCount() * sizeof(PxArticulationGPUIndex)));
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    if (!submitArtiSpatialTendonProperties(
             mGpuSimData->mSpatialTendonPropertiesDev, static_cast<const float*>(stiffnesses->data),
            static_cast<const float*>(dampings->data), static_cast<const float*>(limitStiffnesses->data),
            static_cast<const float*>(offsets->data), indices, mDirtyArtiGpuIndicesDev, numIndices, mMaxSpatialTendons,
            mGpuSimData->mMaxSpatialTendons, mSpatialTendonRecordsDev))
    {
        CARB_LOG_ERROR("Failed to submit spatial tendon properties");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    SYNCHRONIZE_CUDA();
    PxScene* scene = mGpuSimData->mScene;
    scene->getDirectGPUAPI().setArticulationData(
        (void*) mGpuSimData->mSpatialTendonPropertiesDev, mDirtyArtiGpuIndicesDev, PxArticulationGPUAPIWriteType::eSPATIAL_TENDON,
        numIndices, mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiTendonProperties],
        mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiTendonProperties]);
    CHECK_CU(getCudaShim()->eventSynchronize(reinterpret_cast<uintptr_t>(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiTendonProperties]), nullptr));

    SYNCHRONIZE_CUDA();
    return true;
}

// ---------------------------------------------------------------------------
// Mask support
// ---------------------------------------------------------------------------

bool GpuArticulationView::resolveMask(const TensorDesc* maskTensor, PxU32& outK) const
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

// Full-body macro for simple 2-param masked setters: resolve mask, build index desc, forward.
// IsConst should be empty or 'const'. Hand-write methods with non-standard signatures below.
#define GPU_ARTI_MASKED_SETTER(MethodName, IsConst)                                         \
bool GpuArticulationView::MethodName##Masked(const TensorDesc* src, const TensorDesc* mask) IsConst \
{                                                                                           \
    PxU32 K;                                                                                \
    if (!resolveMask(mask, K)) return false;                                                \
    if (K == 0) return true;                                                                \
    if (K == getCount()) return MethodName(src, nullptr);                                   \
    TensorDesc idx{};                                                                       \
    idx.device = mDevice;                                                                   \
    idx.dtype  = omni::physics::tensors::TensorDataType::eUint32;                           \
    idx.numDims = 1;                                                                        \
    idx.dims[0] = (int)K;                                                                   \
    idx.data   = mMaskIndicesDev;                                                           \
    return MethodName(src, &idx);                                                           \
}

// Non-const masked setters
GPU_ARTI_MASKED_SETTER(setRootTransforms, )
GPU_ARTI_MASKED_SETTER(setRootVelocities, )
GPU_ARTI_MASKED_SETTER(setDofPositions, )
GPU_ARTI_MASKED_SETTER(setDofVelocities, )
GPU_ARTI_MASKED_SETTER(setDofActuationForces, )
GPU_ARTI_MASKED_SETTER(setDofPositionTargets, )
GPU_ARTI_MASKED_SETTER(setDofVelocityTargets, )
GPU_ARTI_MASKED_SETTER(setDofLimits, )
GPU_ARTI_MASKED_SETTER(setDofStiffnesses, )
GPU_ARTI_MASKED_SETTER(setDofDampings, )
GPU_ARTI_MASKED_SETTER(setDofMaxForces, )
GPU_ARTI_MASKED_SETTER(setDofDriveModelProperties, )
GPU_ARTI_MASKED_SETTER(setDofFrictionCoefficients, )
GPU_ARTI_MASKED_SETTER(setDofFrictionProperties, )
GPU_ARTI_MASKED_SETTER(setDofMaxVelocities, )
GPU_ARTI_MASKED_SETTER(setDofArmatures, )
GPU_ARTI_MASKED_SETTER(setMasses, )
GPU_ARTI_MASKED_SETTER(setCOMs, )
GPU_ARTI_MASKED_SETTER(setInertias, )
// setDisableGravities/material/rest/contact/compliant Masked: BaseArticulationView

#undef GPU_ARTI_MASKED_SETTER

// Helper macro to build a TensorDesc for the compacted index buffer (used by hand-written methods below)
#define GPU_ARTI_MASK_IDX_DESC(K)                                       \
    TensorDesc idx{};                                                   \
    idx.device = mDevice;                                               \
    idx.dtype  = omni::physics::tensors::TensorDataType::eUint32;       \
    idx.numDims = 1;                                                    \
    idx.dims[0] = (int)(K);                                             \
    idx.data   = mMaskIndicesDev;

// Hand-written: applyForcesAndTorquesAtPositionMasked (5-param signature)
bool GpuArticulationView::applyForcesAndTorquesAtPositionMasked(const TensorDesc* force,
                                                                 const TensorDesc* torque,
                                                                 const TensorDesc* position,
                                                                 const TensorDesc* mask,
                                                                 const bool isGlobal)
{
    PxU32 K;
    if (!resolveMask(mask, K)) return false;
    if (K == 0) return true;
    if (K == getCount()) return applyForcesAndTorquesAtPosition(force, torque, position, nullptr, isGlobal);
    GPU_ARTI_MASK_IDX_DESC(K);
    return applyForcesAndTorquesAtPosition(force, torque, position, &idx, isGlobal);
}

// Hand-written: tendon setters (multi-param signatures)
bool GpuArticulationView::setFixedTendonPropertiesMasked(const TensorDesc* stiffnesses,
                                                          const TensorDesc* dampings,
                                                          const TensorDesc* limitStiffnesses,
                                                          const TensorDesc* limits,
                                                          const TensorDesc* restLengths,
                                                          const TensorDesc* offsets,
                                                          const TensorDesc* mask) const
{
    PxU32 K;
    if (!resolveMask(mask, K)) return false;
    if (K == 0) return true;
    if (K == getCount()) return setFixedTendonProperties(stiffnesses, dampings, limitStiffnesses, limits, restLengths, offsets, nullptr);
    GPU_ARTI_MASK_IDX_DESC(K);
    return setFixedTendonProperties(stiffnesses, dampings, limitStiffnesses, limits, restLengths, offsets, &idx);
}

bool GpuArticulationView::setSpatialTendonPropertiesMasked(const TensorDesc* stiffnesses,
                                                            const TensorDesc* dampings,
                                                            const TensorDesc* limitStiffnesses,
                                                            const TensorDesc* offsets,
                                                            const TensorDesc* mask) const
{
    PxU32 K;
    if (!resolveMask(mask, K)) return false;
    if (K == 0) return true;
    if (K == getCount()) return setSpatialTendonProperties(stiffnesses, dampings, limitStiffnesses, offsets, nullptr);
    GPU_ARTI_MASK_IDX_DESC(K);
    return setSpatialTendonProperties(stiffnesses, dampings, limitStiffnesses, offsets, &idx);
}

#undef GPU_ARTI_MASK_IDX_DESC

} // namespace tensors
} // namespace physx
} // namespace omni

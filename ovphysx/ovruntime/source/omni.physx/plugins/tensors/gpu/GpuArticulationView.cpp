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
 * @implements REQ-SIM-OVSTAGE-WRITEAPPLY-001
 * @covers AC-21
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
 *
 * @implements REQ-TENSOR-VIEW-001
 * @covers AC-7 AC-16
 *
 * @implements REQ-READ-DEVICE-001
 * @covers AC-12
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

    if (!SHIM_CU_EVENT_CREATE(&mMassCenterStartEvent, CU_EVENT_DISABLE_TIMING))
    {
        CARB_LOG_ERROR("Failed to create the articulation mass center start event");
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

        // Drain ONCE, before any free below. A gather this view launches on the null stream returns
        // without synchronizing, so a view destroyed in the same frame as its last read can free memory
        // a kernel is still reading. cudaFree has historically synchronized implicitly; that is not a
        // guarantee to rely on.
        //
        // Unconditional and hoisted, not per buffer: mLinkRecordsDev, mDofRecordsDev and
        // mArtiGpuIndicesDev are all *inputs* to those same kernels. Same shape as
        // GpuPointSetReadView::~GpuPointSetReadView, which drains once for the same reason. A
        // stream-ordered read or commit reads them on its caller's stream instead, which the drain
        // does not cover: it is waited for as well.
        CHECK_CUDA(cudaStreamSynchronize(nullptr));
        mGpuSimData->waitForTableReaders();

        if (mMassCenterStartEvent)
        {
            CHECK_CU(getCudaShim()->eventDestroy(reinterpret_cast<uintptr_t>(mMassCenterStartEvent), nullptr));
        }

        CHECK_CUDA(cudaFree(mArtiGpuIndicesDev));
        CHECK_CUDA(cudaFree(mViewIndicesDev));
        CHECK_CUDA(cudaFree(mDofRecordsDev));
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
    }
}

bool GpuArticulationView::getLinkTransforms(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);

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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
    return getVelAcc(dstTensor,  mGpuSimData->mLinkOrRootLinearVelAccDev,  mGpuSimData->mLinkOrRootAngularVelAccDev, mViewIndicesDev,
                     mArtiGpuIndicesDev, PxArticulationGPUAPIReadType::Enum::eLINK_LINEAR_VELOCITY,
                     PxArticulationGPUAPIReadType::Enum::eLINK_ANGULAR_VELOCITY, getCount(), mMaxLinks, mDevice,
                     "link velocity", __FUNCTION__, mGpuSimData);
}

bool GpuArticulationView::getLinkAccelerations(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
    return getVelAcc(dstTensor,  mGpuSimData->mLinkOrRootLinearVelAccDev,  mGpuSimData->mLinkOrRootAngularVelAccDev, mViewIndicesDev,
                     mArtiGpuIndicesDev, PxArticulationGPUAPIReadType::Enum::eLINK_LINEAR_ACCELERATION,
                     PxArticulationGPUAPIReadType::Enum::eLINK_ANGULAR_ACCELERATION, getCount(), mMaxLinks, mDevice,
                     "link acceleration", __FUNCTION__, mGpuSimData);
}

bool GpuArticulationView::getRootTransforms(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);

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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);

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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);

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

bool GpuArticulationView::waitForDirectGpuFinish(CUevent finishEvent, const char* label) const
{
    return mGpuSimData->drainDirectGpuFinish(finishEvent, label);
}

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
    // WARN_ONCE keys on a static at this expansion site, so every caller shares one line ever.
    // Hence the wording: the message describes the scene, not just the column that asked first.
    CARB_LOG_WARN_ONCE(
        "%s: %s -- and every other inverse dynamics column -- is unavailable on this DirectGPU scene: the "
        "shared inverse dynamics scratch was not allocated -- the scene holds no articulation, or its "
        "device buffer allocation failed.",
        funcName, label);
    return false;
}

namespace
{
using ArtiRead = GpuArticulationView::OvStageArtiRead;

// The PhysX copy or compute one articulation read column needs, one bit each. The root pose serves both pose
// columns; every other column has its own.
enum ArtiReadSource : uint32_t
{
    kArtiReadPose = 1u << 0,
    kArtiReadLinearVelocity = 1u << 1,
    kArtiReadAngularVelocity = 1u << 2,
    kArtiReadMassCenterWorld = 1u << 3,
    kArtiReadMassCenterLocal = 1u << 4,
    kArtiReadJacobian = 1u << 5,
    kArtiReadMassMatrix = 1u << 6,
    kArtiReadCoriolis = 1u << 7,
    kArtiReadGravity = 1u << 8,
    kArtiReadCentroidal = 1u << 9
};

uint32_t artiReadSource(const ArtiRead kind)
{
    switch (kind)
    {
    case ArtiRead::eRootPosition:
    case ArtiRead::eRootOrientation:
        return kArtiReadPose;
    case ArtiRead::eRootLinearVelocity:
        return kArtiReadLinearVelocity;
    case ArtiRead::eRootAngularVelocity:
        return kArtiReadAngularVelocity;
    case ArtiRead::eMassCenterWorld:
        return kArtiReadMassCenterWorld;
    case ArtiRead::eMassCenterLocal:
        return kArtiReadMassCenterLocal;
    case ArtiRead::eJacobian:
        return kArtiReadJacobian;
    case ArtiRead::eMassMatrix:
        return kArtiReadMassMatrix;
    case ArtiRead::eCoriolisForce:
        return kArtiReadCoriolis;
    case ArtiRead::eGravityForce:
        return kArtiReadGravity;
    case ArtiRead::eCentroidalMomentum:
        return kArtiReadCentroidal;
    case ArtiRead::eNone:
        break;
    }
    return 0;
}

// The inverse dynamics column a read column is, or eNone for root state and centre of mass.
BaseArticulationView::InverseDynamicsColumn inverseDynamicsColumnOf(const ArtiRead kind)
{
    using Column = BaseArticulationView::InverseDynamicsColumn;
    switch (kind)
    {
    case ArtiRead::eJacobian:
        return Column::eJacobian;
    case ArtiRead::eMassMatrix:
        return Column::eMassMatrix;
    case ArtiRead::eCoriolisForce:
        return Column::eCoriolisForce;
    case ArtiRead::eGravityForce:
        return Column::eGravityForce;
    case ArtiRead::eCentroidalMomentum:
        return Column::eCentroidalMomentum;
    default:
        return Column::eNone;
    }
}
} // namespace

// The device scratch a read's sources need, in bit order after the GPU index list. computeArticulationData writes
// result slot i for gpuIndices[i], so a cohort's rows land packed at 0..count-1, each at the scene-wide stride; the
// centre of mass computes straight into its column.
static size_t articulationReadScratchLayout(OvStageScratch& layout,
                                            const uint32_t sources,
                                            const PxU32 count,
                                            const GpuSimulationData& sim,
                                            void* staging[10])
{
    const size_t generalized = size_t(sim.mMaxDofs) + 6;
    const auto take = [&](uint32_t bit, size_t floats, size_t index)
    {
        staging[index] = (sources & bit) ? static_cast<void*>(layout.take<float>(floats)) : nullptr;
    };
    staging[0] = (sources & kArtiReadPose) ? static_cast<void*>(layout.take<PxTransform>(count)) : nullptr;
    staging[1] = (sources & kArtiReadLinearVelocity) ? static_cast<void*>(layout.take<PxVec3>(count)) : nullptr;
    staging[2] = (sources & kArtiReadAngularVelocity) ? static_cast<void*>(layout.take<PxVec3>(count)) : nullptr;
    staging[3] = staging[4] = nullptr;
    take(kArtiReadJacobian, size_t(count) * sim.mJacobianMaxRows * sim.mJacobianMaxCols, 5);
    take(kArtiReadMassMatrix, size_t(count) * generalized * generalized, 6);
    take(kArtiReadCoriolis, size_t(count) * generalized, 7);
    take(kArtiReadGravity, size_t(count) * generalized, 8);
    // The mass matrices, the Coriolis term, then the centroidal matrices and their bias forces: the centroidal
    // compute reads the first two from where the first two computes left them.
    take(kArtiReadCentroidal, size_t(count) * (generalized * generalized + generalized + 6 * generalized + 6), 9);
    return layout.used();
}

bool GpuArticulationView::planOvStageRead(const PxU32* rows,
                                          const PxU32 count,
                                          const uint64_t rowsToken,
                                          const uint64_t rowSet,
                                          const OvStageArtiReadColumn* columns,
                                          const PxU32 columnCount,
                                          OvStageArtiReadPlan& plan,
                                          std::shared_ptr<const std::vector<uint8_t>>& head,
                                          size_t& scratchBytes) const
{
    plan = OvStageArtiReadPlan();
    head.reset();
    scratchBytes = 0;
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    if (!rows || count == 0 || !columns || columnCount == 0)
        return false;
    std::vector<uint64_t> signature{ count };
    for (PxU32 c = 0; c < columnCount; ++c)
        signature.push_back((uint64_t(columns[c].kind) << 32) | columns[c].width);
    if (rowsToken)
    {
        if (const OvStageReadPlans<OvStageArtiReadPlan>::Entry* const kept =
                mOvStageReadPlans.find(rowsToken, rowSet, signature))
        {
            plan = kept->plan;
            head = kept->head;
            scratchBytes = kept->scratchBytes;
            return true;
        }
    }
    // Once per plan, on the host: the GPU index list below and the gathers index by these rows.
    if (!checkRecordIndices(rows, count, mEntries.size(), "ovstage articulation rows", __FUNCTION__))
        return false;
    for (PxU32 c = 0; c < columnCount; ++c)
    {
        const uint32_t source = artiReadSource(columns[c].kind);
        if (!source)
            return false;
        const BaseArticulationView::InverseDynamicsColumn dynamics = inverseDynamicsColumnOf(columns[c].kind);
        if (dynamics != BaseArticulationView::InverseDynamicsColumn::eNone)
        {
            // Every row shares the first row's metatype, whose topology the gathers apply to all of them.
            TensorDesc shape;
            shape.numDims = 2;
            shape.dims[0] = count;
            shape.dims[1] = columns[c].width;
            PxU32 width = 0;
            if (!checkInverseDynamicsColumn(shape, rows, count, dynamics, width, "inverse dynamics read", __FUNCTION__))
                return false;
        }
        else if (columns[c].width != (columns[c].kind == ArtiRead::eRootOrientation ? 4u : 3u))
        {
            return false;
        }
        plan.sources |= source;
    }
    plan.count = count;
    plan.firstEntryIndex = rows[0];

    OvStageScratch layout(nullptr);
    layout.take<PxArticulationGPUIndex>(count);
    std::vector<uint8_t> built(layout.used(), 0);
    PxArticulationGPUIndex* const indices = reinterpret_cast<PxArticulationGPUIndex*>(built.data());
    for (PxU32 i = 0; i < count; ++i)
        indices[i] = mArtiIndices[rows[i]];
    void* staging[10];
    scratchBytes = articulationReadScratchLayout(layout, plan.sources, count, *mGpuSimData, staging);
    head = std::make_shared<const std::vector<uint8_t>>(std::move(built));
    if (rowsToken)
        mOvStageReadPlans.keep(rowsToken, rowSet, signature, plan, head, scratchBytes);
    return true;
}

bool GpuArticulationView::readOvStageColumns(const OvStageDeviceReadArgs& args,
                                             const OvStageArtiReadPlan& plan,
                                             const OvStageArtiReadColumn* columns,
                                             const PxU32 columnCount) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
    if (!args.scratch || !args.fork || !args.join || plan.count == 0 || !columns)
        return false;

    PxScene* const scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    const PxU32 count = plan.count;
    OvStageScratch scratch(args.scratch);
    const PxArticulationGPUIndex* const gpuIndicesDev = scratch.take<PxArticulationGPUIndex>(count);
    void* staging[10];
    articulationReadScratchLayout(scratch, plan.sources, count, *mGpuSimData, staging);
    if (!args.scratchHolds(static_cast<char*>(args.scratch) + scratch.used(), "ovstage articulation read"))
        return false;

    // The PhysX copy or compute for one source, ordered on the read's stream. Its status IS checked: PhysX refuses
    // these in states the caller cannot see and writes nothing when it does, and a gather would then publish
    // whatever the scratch last held.
    const auto copy = [&](void* data, PxArticulationGPUAPIReadType::Enum type, const char* label)
    {
        const bool issued = physxOnStream(args, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().getArticulationData(data, gpuIndicesDev, type, count, start, finish);
        });
        if (!issued)
            CARB_LOG_ERROR("ovstage articulation read: PxDirectGPUAPI refused the %s copy", label);
        return issued;
    };
    const auto compute = [&](void* data, PxArticulationGPUAPIComputeType::Enum type, const char* label)
    {
        const bool issued = physxOnStream(args, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().computeArticulationData(data, gpuIndicesDev, type, count, start, finish);
        });
        if (!issued)
            CARB_LOG_ERROR("ovstage articulation read: PxDirectGPUAPI refused the %s compute", label);
        return issued;
    };

    // The cohort's topology, read off its first row: the plan checked every row shares it.
    const ArticulationEntry& entry = mEntries[plan.firstEntryIndex];
    const PxU32 rootDofs = entry.metatype->getFixedBase() ? 0u : 6u;
    const PxU32 dofRecordBase = plan.firstEntryIndex * mMaxDofs;
    const bool applyBodyOrderSign = entry.metatype->hasReversedDofBodyOrder();
    // Not a dof count: the scene-wide dof maximum plus the floating base's six.
    const PxU32 simGeneralizedCoords = mGpuSimData->mMaxDofs + 6;

    uint32_t copied = 0;
    bool ok = true;
    for (PxU32 c = 0; c < columnCount && ok; ++c)
    {
        float* const dst = columns[c].dst;
        const PxU32 width = columns[c].width;
        if (!dst)
        {
            ok = false;
            break;
        }
        switch (columns[c].kind)
        {
        case ArtiRead::eRootPosition:
        case ArtiRead::eRootOrientation:
            if (!(copied & kArtiReadPose))
                ok = copy(staging[0], PxArticulationGPUAPIReadType::eROOT_GLOBAL_POSE, "root pose");
            copied |= kArtiReadPose;
            ok = ok && fetchArtiRootPoseColumnOvStage(dst, static_cast<const PxTransform*>(staging[0]), count,
                                                      columns[c].kind == ArtiRead::eRootOrientation, args.stream);
            break;
        case ArtiRead::eRootLinearVelocity:
        case ArtiRead::eRootAngularVelocity:
        {
            const bool linear = columns[c].kind == ArtiRead::eRootLinearVelocity;
            const uint32_t bit = linear ? kArtiReadLinearVelocity : kArtiReadAngularVelocity;
            void* const source = staging[linear ? 1 : 2];
            if (!(copied & bit))
                ok = copy(source,
                          linear ? PxArticulationGPUAPIReadType::eROOT_LINEAR_VELOCITY :
                                   PxArticulationGPUAPIReadType::eROOT_ANGULAR_VELOCITY,
                          linear ? "root linear velocity" : "root angular velocity");
            copied |= bit;
            ok = ok && fetchArtiRootVelocityColumnOvStage(dst, static_cast<const PxVec3*>(source), count, args.stream);
            break;
        }
        case ArtiRead::eMassCenterWorld:
        case ArtiRead::eMassCenterLocal:
            // Zeroed first: PhysX's centre-of-mass kernel accumulates into its output rather than writing it -- it
            // adds each link's mass into x, reads that back as the total and adds the mass-weighted positions --
            // so a non-zero column divides by a corrupted total mass.
            ok = CHECK_CUDA(cudaMemsetAsync(dst, 0, size_t(count) * sizeof(PxVec3), args.stream)) &&
                 compute(dst,
                         columns[c].kind == ArtiRead::eMassCenterLocal ?
                             PxArticulationGPUAPIComputeType::eARTICULATION_COMS_ROOT_FRAME :
                             PxArticulationGPUAPIComputeType::eARTICULATION_COMS_WORLD_FRAME,
                         "centre of mass");
            break;
        case ArtiRead::eJacobian:
            ok = compute(staging[5], PxArticulationGPUAPIComputeType::eDENSE_JACOBIANS, "jacobian") &&
                 fetchArtiJacobian(dst, static_cast<const float*>(staging[5]), count * width, width,
                                   mGpuSimData->mJacobianMaxRows * mGpuSimData->mJacobianMaxCols,
                                   rootDofs + entry.numDofs, rootDofs, dofRecordBase, mDofRecordsDev,
                                   applyBodyOrderSign, args.stream);
            break;
        case ArtiRead::eMassMatrix:
            ok = compute(staging[6], PxArticulationGPUAPIComputeType::eMASS_MATRICES, "mass matrix") &&
                 fetchArtiMassMatrices(dst, static_cast<const float*>(staging[6]), count * width, width,
                                       simGeneralizedCoords * simGeneralizedCoords, rootDofs + entry.numDofs, rootDofs,
                                       dofRecordBase, mDofRecordsDev, applyBodyOrderSign, args.stream);
            break;
        case ArtiRead::eCoriolisForce:
        case ArtiRead::eGravityForce:
        {
            const bool gravity = columns[c].kind == ArtiRead::eGravityForce;
            void* const source = staging[gravity ? 8 : 7];
            ok = compute(source,
                         gravity ? PxArticulationGPUAPIComputeType::eGRAVITY_COMPENSATION :
                                   PxArticulationGPUAPIComputeType::eCORIOLIS_AND_CENTRIFUGAL_COMPENSATION,
                         gravity ? "gravity compensation" : "coriolis compensation") &&
                 fetchArtiGeneralizedForceColumnOvStage(dst, static_cast<const float*>(source), count, width, rootDofs,
                                                        simGeneralizedCoords, dofRecordBase, mDofRecordsDev,
                                                        args.stream);
            break;
        }
        case ArtiRead::eCentroidalMomentum:
        {
            // The centroidal matrix is defined against the mass matrix and the Coriolis term, so all three are
            // computed into one block, in that order; each stage feeds the next, so a refused one stops the read.
            float* const block = static_cast<float*>(staging[9]);
            const PxU32 massBlock = simGeneralizedCoords * simGeneralizedCoords;
            ok = compute(block, PxArticulationGPUAPIComputeType::eMASS_MATRICES, "mass matrix") &&
                 compute(block + size_t(count) * massBlock,
                         PxArticulationGPUAPIComputeType::eCORIOLIS_AND_CENTRIFUGAL_COMPENSATION,
                         "coriolis compensation") &&
                 compute(block, PxArticulationGPUAPIComputeType::eCENTROIDAL_MOMENTUM_MATRICES, "centroidal momentum");
            // The kernel's `maxDofs` is the cohort's PACKED dof count -- it strides the destination by
            // (maxDofs + 7) and the source by (maxDofs + 6). The centroidal matrices start past the mass matrices
            // and the Coriolis terms, and their bias forces past the centroidal matrices.
            const PxU32 simCentroidalBlock = 6u * simGeneralizedCoords;
            ok = ok && fetchArtiCentroidalMomentumMatrices(
                           dst, block + size_t(count) * (massBlock + simGeneralizedCoords), count * width,
                           entry.numDofs, width, simCentroidalBlock, simCentroidalBlock * count, dofRecordBase,
                           mDofRecordsDev, applyBodyOrderSign, args.stream);
            break;
        }
        case ArtiRead::eNone:
            ok = false;
            break;
        }
    }
    // The gathers read this view's tables on `args.stream`; the host rewrites them only once it is done.
    const bool noted = args.capturing || mGpuSimData->noteTableReader(args.stream);
    return noted && ok;
}

namespace
{
using RecordRead = GpuArticulationView::OvStageRecordRead;

// The PhysX copies and derived passes a record read column needs, one bit each.
enum RecordReadSource : uint32_t
{
    kRecordReadDofPosition = 1u << 0,
    kRecordReadDofVelocity = 1u << 1,
    kRecordReadDofPositionTarget = 1u << 2,
    kRecordReadDofVelocityTarget = 1u << 3,
    kRecordReadDofActuationForce = 1u << 4,
    // The link incoming joint force and the link poses it is resolved in, copied together.
    kRecordReadLinkForce = 1u << 5,
    // The dense passes over the view's links: forces projected onto each DOF, and forces in each joint frame.
    kRecordReadDofProjection = 1u << 6,
    kRecordReadLinkJointFrame = 1u << 7,
    kRecordReadFixedTendon = 1u << 8,
    kRecordReadSpatialTendon = 1u << 9
};
constexpr uint32_t kRecordReadSourceCount = 10;

uint32_t recordReadSources(const RecordRead kind)
{
    switch (kind)
    {
    case RecordRead::eDofPosition:
        return kRecordReadDofPosition;
    case RecordRead::eDofVelocity:
        return kRecordReadDofVelocity;
    case RecordRead::eDofPositionTarget:
        return kRecordReadDofPositionTarget;
    case RecordRead::eDofVelocityTarget:
        return kRecordReadDofVelocityTarget;
    case RecordRead::eDofActuationForce:
        return kRecordReadDofActuationForce;
    case RecordRead::eDofProjectedForce:
        return kRecordReadLinkForce | kRecordReadDofProjection;
    case RecordRead::eLinkIncomingJointForce:
        return kRecordReadLinkForce | kRecordReadLinkJointFrame;
    case RecordRead::eFixedTendon:
        return kRecordReadFixedTendon;
    case RecordRead::eSpatialTendon:
        return kRecordReadSpatialTendon;
    case RecordRead::eNone:
        break;
    }
    return 0;
}

// The record type a column's kind takes.
size_t recordBytesOf(const RecordRead kind)
{
    switch (kind)
    {
    case RecordRead::eLinkIncomingJointForce:
        return sizeof(ArticulationLinkOvStageRecord);
    case RecordRead::eFixedTendon:
    case RecordRead::eSpatialTendon:
        return sizeof(ArticulationTendonOvStageRecord);
    default:
        return sizeof(ArticulationDofOvStageRecord);
    }
}

// The DirectGPU read type and the unit/sign fold of a DOF scalar column.
void dofReadOf(const RecordRead kind, PxArticulationGPUAPIReadType::Enum& type, DofScalePolicy& policy)
{
    // Position, velocity and their drive targets are a generalized coordinate and its derivatives: degrees on an
    // angular axis and the body-order sign. The actuation force is an effort -- a newton-metre is not an angle --
    // so it takes the sign alone.
    policy = DofScalePolicy::eAngularSigned;
    switch (kind)
    {
    case RecordRead::eDofVelocity:
        type = PxArticulationGPUAPIReadType::eJOINT_VELOCITY;
        return;
    case RecordRead::eDofPositionTarget:
        type = PxArticulationGPUAPIReadType::eJOINT_TARGET_POSITION;
        return;
    case RecordRead::eDofVelocityTarget:
        type = PxArticulationGPUAPIReadType::eJOINT_TARGET_VELOCITY;
        return;
    case RecordRead::eDofActuationForce:
        type = PxArticulationGPUAPIReadType::eJOINT_FORCE;
        policy = DofScalePolicy::eSigned;
        return;
    default:
        type = PxArticulationGPUAPIReadType::eJOINT_POSITION;
        return;
    }
}
} // namespace

// The device scratch a record read needs after its records, in source-bit order. Every copy holds the `numArtis`
// articulations it fetches, at the scene-wide strides PhysX lays its buffers out in; the dense passes hold them at the
// view's own.
static void recordReadScratchLayout(OvStageScratch& layout,
                                    const uint32_t sources,
                                    const PxU32 numArtis,
                                    const PxU32 viewMaxDofs,
                                    const PxU32 viewMaxLinks,
                                    const GpuSimulationData& sim,
                                    void* staging[kRecordReadSourceCount + 1])
{
    for (uint32_t bit = 0; bit < kRecordReadSourceCount + 1; ++bit)
        staging[bit] = nullptr;
    for (uint32_t bit = 0; bit < 5; ++bit)
        if (sources & (1u << bit))
            staging[bit] = layout.take<float>(size_t(numArtis) * sim.mMaxDofs);
    if (sources & kRecordReadLinkForce)
    {
        staging[5] = layout.take<PhysxGpuSpatialForces>(size_t(numArtis) * sim.mMaxLinks);
        staging[kRecordReadSourceCount] = layout.take<PxTransform>(size_t(numArtis) * sim.mMaxLinks);
    }
    if (sources & kRecordReadDofProjection)
        staging[6] = layout.take<float>(size_t(numArtis) * viewMaxDofs);
    if (sources & kRecordReadLinkJointFrame)
        staging[7] = layout.take<PhysxGpuSpatialForces>(size_t(numArtis) * viewMaxLinks);
    if (sources & kRecordReadFixedTendon)
        staging[8] = layout.take<PxGpuFixedTendonData>(size_t(numArtis) * sim.mMaxFixedTendons);
    if (sources & kRecordReadSpatialTendon)
        staging[9] = layout.take<PxGpuSpatialTendonData>(size_t(numArtis) * sim.mMaxSpatialTendons);
}

bool GpuArticulationView::planOvStageRecordRead(const PxU32 numRecords,
                                                const size_t recordBytes,
                                                const OvStageRecordReadColumn* columns,
                                                const PxU32 columnCount,
                                                const PxU32* fetchRows,
                                                const PxU32 fetchCount,
                                                OvStageRecordReadPlan& plan,
                                                std::vector<uint8_t>& fetchHead,
                                                size_t& scratchBytes) const
{
    plan = OvStageRecordReadPlan();
    fetchHead.clear();
    scratchBytes = 0;
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    if (numRecords == 0 || !columns || columnCount == 0 || (!fetchRows) != (fetchCount == 0))
        return false;
    if (fetchRows && !checkRecordIndices(fetchRows, fetchCount, mEntries.size(), "ovstage articulation fetch rows",
                                         __FUNCTION__))
        return false;
    for (PxU32 c = 0; c < columnCount; ++c)
    {
        const RecordRead kind = columns[c].kind;
        const uint32_t sources = recordReadSources(kind);
        if (!sources || recordBytesOf(kind) != recordBytes)
            return false;
        // A view with no slot for what a record names would gather out of bounds.
        const bool links = kind == RecordRead::eLinkIncomingJointForce || kind == RecordRead::eDofProjectedForce;
        const bool dofs = kind >= RecordRead::eDofPosition && kind <= RecordRead::eDofProjectedForce;
        if ((links && (mMaxLinks == 0 || mGpuSimData->mMaxLinks == 0)) || (dofs && mMaxDofs == 0))
        {
            CARB_LOG_WARN("ovstage articulation read: the articulation view has no %s", links ? "links" : "DOF");
            return false;
        }
        // A projected force, link force or tendon gathers through a pass over every articulation of the view.
        if (fetchRows && !(kind >= RecordRead::eDofPosition && kind <= RecordRead::eDofActuationForce))
            return false;
        if (kind == RecordRead::eFixedTendon || kind == RecordRead::eSpatialTendon)
        {
            const bool fixed = kind == RecordRead::eFixedTendon;
            const PxU32 maxTendons = fixed ? mGpuSimData->mMaxFixedTendons : mGpuSimData->mMaxSpatialTendons;
            const PxU32 structFloats =
                PxU32((fixed ? sizeof(PxGpuFixedTendonData) : sizeof(PxGpuSpatialTendonData)) / sizeof(float));
            // The gather reads a property at its float offset within a tendon struct, so one that does not fit
            // would read a neighbouring tendon's data.
            const PxU32 prop = static_cast<PxU32>(columns[c].tendonProperty);
            if (maxTendons == 0 || prop + tendonPropertyComponents(columns[c].tendonProperty) > structFloats)
            {
                CARB_LOG_ERROR("ovstage tendon read: the view has no %s tendon slot for property %u",
                               fixed ? "fixed" : "spatial", prop);
                return false;
            }
        }
        plan.sources |= sources;
    }
    plan.numRecords = numRecords;
    plan.numFetched = fetchCount;

    OvStageScratch layout(nullptr);
    layout.take<uint8_t>(size_t(numRecords) * recordBytes);
    if (fetchRows)
    {
        layout.take<PxArticulationGPUIndex>(fetchCount);
        fetchHead.resize(size_t(fetchCount) * sizeof(PxArticulationGPUIndex));
        PxArticulationGPUIndex* const indices = reinterpret_cast<PxArticulationGPUIndex*>(fetchHead.data());
        for (PxU32 i = 0; i < fetchCount; ++i)
            indices[i] = mArtiIndices[fetchRows[i]];
    }
    void* staging[kRecordReadSourceCount + 1];
    recordReadScratchLayout(layout, plan.sources, fetchRows ? fetchCount : getCount(), mMaxDofs, mMaxLinks,
                            *mGpuSimData, staging);
    scratchBytes = layout.used();
    return true;
}

bool GpuArticulationView::readOvStageRecordColumns(const OvStageDeviceReadArgs& args,
                                                   const OvStageRecordReadPlan& plan,
                                                   const OvStageRecordReadColumn* columns,
                                                   const PxU32 columnCount) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
    if (!args.scratch || !args.fork || !args.join || plan.numRecords == 0 || !columns || columnCount == 0)
        return false;

    PxScene* const scene = mGpuSimData->mScene;
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    // The articulations each copy holds: the whole view, or the ones a DOF state read's records name, whose GPU
    // indices follow the records. Only a whole-view read reaches the dense passes below (planOvStageRecordRead).
    const PxU32 numArtis = plan.numFetched ? plan.numFetched : getCount();
    const PxU32 numRecords = plan.numRecords;
    OvStageScratch scratch(args.scratch);
    const void* const recordsDev = scratch.take<uint8_t>(size_t(numRecords) * recordBytesOf(columns[0].kind));
    const PxArticulationGPUIndex* const fetchIndices =
        plan.numFetched ? scratch.take<PxArticulationGPUIndex>(plan.numFetched) : mArtiGpuIndicesDev;
    void* staging[kRecordReadSourceCount + 1];
    recordReadScratchLayout(scratch, plan.sources, numArtis, mMaxDofs, mMaxLinks, *mGpuSimData, staging);
    if (!args.scratchHolds(static_cast<char*>(args.scratch) + scratch.used(), "ovstage articulation record read"))
        return false;

    // One PhysX copy per source, made the first time a column needs it (`done`). Its status IS checked: a refused
    // copy writes nothing, and a gather would then publish whatever the scratch last held.
    uint32_t done = 0;
    const auto copy = [&](void* data, PxArticulationGPUAPIReadType::Enum type)
    {
        const bool issued = physxOnStream(args, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().getArticulationData(data, fetchIndices, type, numArtis, start, finish);
        });
        if (!issued)
            CARB_LOG_ERROR("ovstage articulation read: PxDirectGPUAPI refused a copy of read type %d", int(type));
        return issued;
    };
    const auto copyOnce = [&](uint32_t bit, void* data, PxArticulationGPUAPIReadType::Enum type)
    {
        if (!(done & bit) && copy(data, type))
            done |= bit;
        return (done & bit) != 0;
    };
    // The link incoming joint forces and the link poses they are resolved in.
    const auto copyLinkForces = [&]()
    {
        if (!(done & kRecordReadLinkForce) &&
            copy(staging[5], PxArticulationGPUAPIReadType::eLINK_INCOMING_JOINT_FORCE) &&
            copy(staging[kRecordReadSourceCount], PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE))
            done |= kRecordReadLinkForce;
        return (done & kRecordReadLinkForce) != 0;
    };

    bool ok = true;
    for (PxU32 c = 0; c < columnCount && ok; ++c)
    {
        const RecordRead kind = columns[c].kind;
        float* const dst = columns[c].dst;
        if (!dst)
        {
            ok = false;
            break;
        }
        switch (kind)
        {
        case RecordRead::eDofPosition:
        case RecordRead::eDofVelocity:
        case RecordRead::eDofPositionTarget:
        case RecordRead::eDofVelocityTarget:
        case RecordRead::eDofActuationForce:
        {
            // The five DOF scalar kinds and their copies are in the same order.
            const uint32_t index = static_cast<uint32_t>(kind) - static_cast<uint32_t>(RecordRead::eDofPosition);
            PxArticulationGPUAPIReadType::Enum type;
            DofScalePolicy policy;
            dofReadOf(kind, type, policy);
            ok = copyOnce(1u << index, staging[index], type) &&
                 fetchArtiDofAttributeOvStage(dst, static_cast<const float*>(staging[index]), numRecords,
                                              mGpuSimData->mMaxDofs, policy,
                                              static_cast<const ArticulationDofOvStageRecord*>(recordsDev),
                                              args.stream);
            break;
        }
        case RecordRead::eDofProjectedForce:
            ok = copyLinkForces();
            // Zeroed before the dense pass, which writes only the DOFs of links that HAVE an inbound joint -- a
            // fixed joint contributes none. Strided by the VIEW's maxDofs, which is what that pass counts in, and
            // already in the joint frame and body order, so the gather applies no fold.
            if (ok && !(done & kRecordReadDofProjection))
            {
                ok = CHECK_CUDA(cudaMemsetAsync(staging[6], 0, size_t(numArtis) * mMaxDofs * sizeof(float),
                                                args.stream)) &&
                     fetchDofProjectionForce(static_cast<float*>(staging[6]),
                                             static_cast<const PhysxGpuSpatialForces*>(staging[5]),
                                             static_cast<const PxTransform*>(staging[kRecordReadSourceCount]),
                                             numArtis * mMaxLinks, mMaxLinks, mGpuSimData->mMaxLinks, mLinkRecordsDev,
                                             args.stream);
                done |= ok ? kRecordReadDofProjection : 0u;
            }
            ok = ok && fetchArtiDofAttributeOvStage(dst, static_cast<const float*>(staging[6]), numRecords, mMaxDofs,
                                                    DofScalePolicy::eNone,
                                                    static_cast<const ArticulationDofOvStageRecord*>(recordsDev),
                                                    args.stream);
            break;
        case RecordRead::eLinkIncomingJointForce:
            ok = copyLinkForces();
            // The dense pass writes every one of the view's link slots, so it needs no zeroing; no record names a
            // padding slot.
            if (ok && !(done & kRecordReadLinkJointFrame))
            {
                ok = fetchArtiLinkIncomingJointForce(static_cast<PhysxGpuSpatialForces*>(staging[7]),
                                                     static_cast<const PhysxGpuSpatialForces*>(staging[5]),
                                                     static_cast<const PxTransform*>(staging[kRecordReadSourceCount]),
                                                     numArtis * mMaxLinks, mMaxLinks, mGpuSimData->mMaxLinks,
                                                     mLinkRecordsDev, args.stream);
                done |= ok ? kRecordReadLinkJointFrame : 0u;
            }
            ok = ok && fetchArtiLinkVectorOvStage(dst, static_cast<const float*>(staging[7]), numRecords, mMaxLinks, 6u,
                                                  static_cast<const ArticulationLinkOvStageRecord*>(recordsDev),
                                                  args.stream);
            break;
        case RecordRead::eFixedTendon:
        case RecordRead::eSpatialTendon:
        {
            const bool fixed = kind == RecordRead::eFixedTendon;
            const uint32_t bit = fixed ? kRecordReadFixedTendon : kRecordReadSpatialTendon;
            void* const source = staging[fixed ? 8 : 9];
            const TendonProperty prop = columns[c].tendonProperty;
            ok = copyOnce(bit, source,
                          fixed ? PxArticulationGPUAPIReadType::eFIXED_TENDON :
                                  PxArticulationGPUAPIReadType::eSPATIAL_TENDON) &&
                 fetchArtiTendonPropertyOvStage(
                     dst, static_cast<const float*>(source), numRecords,
                     fixed ? mGpuSimData->mMaxFixedTendons : mGpuSimData->mMaxSpatialTendons,
                     PxU32((fixed ? sizeof(PxGpuFixedTendonData) : sizeof(PxGpuSpatialTendonData)) / sizeof(float)),
                     static_cast<PxU32>(prop), tendonPropertyComponents(prop),
                     static_cast<const ArticulationTendonOvStageRecord*>(recordsDev), args.stream);
            break;
        }
        case RecordRead::eNone:
            ok = false;
            break;
        }
    }
    // The gathers read this view's tables on `args.stream`; the host rewrites them only once it is done.
    const bool noted = args.capturing || mGpuSimData->noteTableReader(args.stream);
    return noted && ok;
}

bool GpuArticulationView::setRootVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);

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

bool GpuArticulationView::setDofAttributeOvStage(const char* const attribName,
                                                 const TensorDesc* const srcTensor,
                                                 const PxArticulationGPUAPIReadType::Enum readFlag,
                                                 const PxArticulationGPUAPIWriteType::Enum writeFlag,
                                                 const ArticulationDofOvStageRecord* const recordsDev,
                                                 const PxU32 numOutputs,
                                                 const int applyEvent,
                                                 const DofScalePolicy policy,
                                                 const OvStageDeviceWriteArgs& sessionDeviceArgs)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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

    const bool session = sessionDeviceArgs.sessionOwned();
    const OvStageDeviceWriteArgs deviceArgs = session ? sessionDeviceArgs : drainDeviceWriteArgs(applyEvent);
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    PxScene* scene = mGpuSimData->mScene;
    const PxU32 numArtis = getCount();

    // A session's DOF block is its own. The drain uses the scene's shared DOF scratch, which it releases after
    // the apply (ADR-0008 Decision 7).
    float* dofScalars = mGpuSimData->mDofScalarsDev;
    if (session)
    {
        OvStageScratch scratch(deviceArgs.scratch);
        dofScalars = scratch.take<float>(size_t(numArtis) * mGpuSimData->mMaxDofs);
        if (!deviceArgs.scratchHolds(dofScalars + size_t(numArtis) * mGpuSimData->mMaxDofs, attribName))
            return false;
    }

    // Read-modify-write. Status IS checked: PhysX refuses these calls in states the caller cannot
    // see and writes NOTHING when it does; the scratch is not zeroed, so an unchecked refusal would
    // overlay the caller's DOFs onto stale contents and publish that as joint state.
    if (!physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().getArticulationData((void*)dofScalars, mArtiGpuIndicesDev, readFlag,
                                                                numArtis, start, finish);
        }))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::getArticulationData refused the read that preserves the DOFs "
                       "this write does not address",
                       attribName);
        return false;
    }

    // A DOF a masked commit does not select keeps the value just read.
    if (!submitArtiDofAttributeOvStage(dofScalars, static_cast<const float*>(srcTensor->data), numOutputs,
                                       mGpuSimData->mMaxDofs, recordsDev, policy, deviceArgs.stream, deviceArgs.mask))
    {
        CARB_LOG_ERROR("%s: ovstage DOF scatter failed", attribName);
        return false;
    }

    // Ordered on the device. The write's stream waits for the apply too, so the next producer of the
    // block cannot overwrite it mid-apply (ADR-0008 D7).
    if (!physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().setArticulationData((void*)dofScalars, mArtiGpuIndicesDev, writeFlag,
                                                                numArtis, start, finish);
        }))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::setArticulationData refused the write", attribName);
        return false;
    }
    if (!session)
        mGpuSimData->recordKernelDone(SharedDeviceBuffer::eDofScalars);
    return true;
}

bool GpuArticulationView::setDofPositionsOvStage(const TensorDesc* srcTensor,
                                                 const ArticulationDofOvStageRecord* recordsDev,
                                                 const PxU32 numOutputs,
                                                 const OvStageDeviceWriteArgs& deviceArgs)
{
    return setDofAttributeOvStage("jointPosition", srcTensor, PxArticulationGPUAPIReadType::eJOINT_POSITION,
                                  PxArticulationGPUAPIWriteType::eJOINT_POSITION, recordsDev, numOutputs,
                                  ApplyEvent::eArtiDofPositions, DofScalePolicy::eAngularSigned, deviceArgs);
}

bool GpuArticulationView::setDofVelocitiesOvStage(const TensorDesc* srcTensor,
                                                  const ArticulationDofOvStageRecord* recordsDev,
                                                  const PxU32 numOutputs,
                                                  const OvStageDeviceWriteArgs& deviceArgs)
{
    return setDofAttributeOvStage("jointVelocity", srcTensor, PxArticulationGPUAPIReadType::eJOINT_VELOCITY,
                                  PxArticulationGPUAPIWriteType::eJOINT_VELOCITY, recordsDev, numOutputs,
                                  ApplyEvent::eArtiDofVelocities, DofScalePolicy::eAngularSigned, deviceArgs);
}

// The three drive INPUTS (ADR-0012). Same machinery as the state pair above -- the difference that
// matters is the scale policy: an actuation force is a joint effort, so it takes the sign but NOT
// the rad->deg fold the other four take.
bool GpuArticulationView::setDofPositionTargetsOvStage(const TensorDesc* srcTensor,
                                                       const ArticulationDofOvStageRecord* recordsDev,
                                                       const PxU32 numOutputs,
                                                       const OvStageDeviceWriteArgs& deviceArgs)
{
    return setDofAttributeOvStage("jointPositionTarget", srcTensor,
                                  PxArticulationGPUAPIReadType::eJOINT_TARGET_POSITION,
                                  PxArticulationGPUAPIWriteType::eJOINT_TARGET_POSITION, recordsDev, numOutputs,
                                  ApplyEvent::eArtiDofPositionTargets, DofScalePolicy::eAngularSigned, deviceArgs);
}

bool GpuArticulationView::setDofVelocityTargetsOvStage(const TensorDesc* srcTensor,
                                                       const ArticulationDofOvStageRecord* recordsDev,
                                                       const PxU32 numOutputs,
                                                       const OvStageDeviceWriteArgs& deviceArgs)
{
    return setDofAttributeOvStage("jointVelocityTarget", srcTensor,
                                  PxArticulationGPUAPIReadType::eJOINT_TARGET_VELOCITY,
                                  PxArticulationGPUAPIWriteType::eJOINT_TARGET_VELOCITY, recordsDev, numOutputs,
                                  ApplyEvent::eArtiDofVelocityTargets, DofScalePolicy::eAngularSigned, deviceArgs);
}

bool GpuArticulationView::setDofActuationForcesOvStage(const TensorDesc* srcTensor,
                                                       const ArticulationDofOvStageRecord* recordsDev,
                                                       const PxU32 numOutputs,
                                                       const OvStageDeviceWriteArgs& deviceArgs)
{
    return setDofAttributeOvStage("jointActuationForce", srcTensor, PxArticulationGPUAPIReadType::eJOINT_FORCE,
                                  PxArticulationGPUAPIWriteType::eJOINT_FORCE, recordsDev, numOutputs,
                                  ApplyEvent::eArtiDofForces, DofScalePolicy::eSigned, deviceArgs);
}

OvStageDeviceWriteArgs GpuArticulationView::drainDeviceWriteArgs(const int applyEvent) const
{
    OvStageDeviceWriteArgs deviceArgs;
    deviceArgs.fork = mGpuSimData->mApplyWaitEvents[applyEvent];
    deviceArgs.join = mGpuSimData->mApplySignalEvents[applyEvent];
    return deviceArgs;
}

size_t GpuArticulationView::ovStageWriteScratchBytes(const OvStageArtiWrite kind, const PxU32 numOutputs) const
{
    // The order the session setters carve their scratch in.
    OvStageScratch layout(nullptr);
    const size_t numArtis = getCount();
    switch (kind)
    {
    case OvStageArtiWrite::eRootPose:
        layout.take<PxU32>(numOutputs);
        layout.take<PxTransform>(numArtis);
        break;
    case OvStageArtiWrite::eRootVelocity:
        layout.take<PxU32>(numOutputs);
        layout.take<PxVec3>(numArtis);
        break;
    case OvStageArtiWrite::eDof:
        layout.take<float>(numArtis * (mGpuSimData ? mGpuSimData->mMaxDofs : 0u));
        break;
    case OvStageArtiWrite::eFixedTendon:
        layout.take<PxGpuFixedTendonData>(numArtis * (mGpuSimData ? mGpuSimData->mMaxFixedTendons : 0u));
        break;
    case OvStageArtiWrite::eSpatialTendon:
        layout.take<PxGpuSpatialTendonData>(numArtis * (mGpuSimData ? mGpuSimData->mMaxSpatialTendons : 0u));
        break;
    }
    return layout.used();
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
        if (!uploadToDevice(mOvStageRowsDev, rows, count * sizeof(PxU32)))
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
                                                  const bool pose,
                                                  const OvStageDeviceWriteArgs& sessionDeviceArgs)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    const int applyEvent = pose    ? ApplyEvent::eArtiRootTransforms :
                           angular ? ApplyEvent::eArtiRootAngVelocities :
                                     ApplyEvent::eArtiRootLinVelocities;

    const bool session = sessionDeviceArgs.sessionOwned();
    const OvStageDeviceWriteArgs deviceArgs = session ? sessionDeviceArgs : drainDeviceWriteArgs(applyEvent);
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    PxScene* scene = mGpuSimData->mScene;
    const PxU32 numArtis = getCount();

    // A session's row list (uploaded at open) and root block are its own. The drain uses the view's row
    // copy and the scene's shared root block, which it releases after the apply (ADR-0008 Decision 7).
    const PxU32* rowsDev = nullptr;
    void* block = nullptr;
    if (session)
    {
        OvStageScratch scratch(deviceArgs.scratch);
        rowsDev = scratch.take<PxU32>(numOutputs);
        block = pose ? static_cast<void*>(scratch.take<PxTransform>(numArtis)) :
                       static_cast<void*>(scratch.take<PxVec3>(numArtis));
        if (!deviceArgs.scratchHolds(
                static_cast<char*>(block) + size_t(numArtis) * (pose ? sizeof(PxTransform) : sizeof(PxVec3)),
                attribName))
            return false;
    }
    else
    {
        rowsDev = ovStageRowsDevice(rows, numOutputs, rowsToken);
        if (!rowsDev)
            return false;
        block = pose    ? static_cast<void*>(mGpuSimData->mLinkOrRootTransformsDev) :
                angular ? static_cast<void*>(mGpuSimData->mLinkOrRootAngularVelAccDev) :
                          static_cast<void*>(mGpuSimData->mLinkOrRootLinearVelAccDev);
    }

    // Read-modify-write. Status IS checked: PhysX refuses these calls in states the caller cannot
    // see and writes NOTHING when it does; the block is not zeroed, so an unchecked refusal would
    // overlay the caller's values onto stale contents and publish that as root state.
    if (!physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().getArticulationData(block, mArtiGpuIndicesDev, readFlag, numArtis,
                                                                start, finish);
        }))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::getArticulationData refused the read that preserves the root "
                       "state this write does not address",
                       attribName);
        return false;
    }

    // An articulation a masked commit does not select keeps the state just read.
    const bool scattered =
        pose ? submitArtiRootPoseOvStage(static_cast<PxTransform*>(block),
                                          static_cast<const float*>(srcTensor->data), rowsDev, mRootRecordsDev,
                                          numOutputs, angular, deviceArgs.stream, deviceArgs.mask) :
               submitArtiRootVelocityOvStage(static_cast<PxVec3*>(block),
                                              static_cast<const float*>(srcTensor->data), rowsDev, numOutputs,
                                              deviceArgs.stream, deviceArgs.mask);
    if (!scattered)
    {
        CARB_LOG_ERROR("%s: ovstage articulation root scatter failed", attribName);
        return false;
    }

    // mArtiGpuIndicesDev, not a compacted list: the whole view is pushed, and the articulations this
    // query did not match, or its mask leaves out, are written their own current values. That is not a
    // no-op: PhysX refreshes their kinematics, so their link accelerations and incoming joint forces read
    // zero until the next step. It is the reason the RMW above covers the whole view rather than just the
    // matched rows. A subset write
    // would need a compacted index list whose LENGTH comes off the host, which is the host block the
    // read carries none of. The write's stream waits for the apply, so the next producer of the block
    // cannot overwrite it mid-apply.
    if (!physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().setArticulationData(block, mArtiGpuIndicesDev, writeFlag, numArtis,
                                                                start, finish);
        }))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::setArticulationData refused the write", attribName);
        return false;
    }
    if (!session)
        mGpuSimData->recordKernelDone(sharedBuf);
    return true;
}

bool GpuArticulationView::setRootPositionsOvStage(const TensorDesc* srcTensor,
                                                  const PxU32* rows,
                                                  const PxU32 numOutputs,
                                                  const uint64_t rowsToken,
                                                  const OvStageDeviceWriteArgs& deviceArgs)
{
    return setRootAttributeOvStage("position", srcTensor, rows, numOutputs, rowsToken, false, true, deviceArgs);
}

bool GpuArticulationView::setRootOrientationsOvStage(const TensorDesc* srcTensor,
                                                     const PxU32* rows,
                                                     const PxU32 numOutputs,
                                                     const uint64_t rowsToken,
                                                     const OvStageDeviceWriteArgs& deviceArgs)
{
    return setRootAttributeOvStage("orientation", srcTensor, rows, numOutputs, rowsToken, true, true, deviceArgs);
}

bool GpuArticulationView::setRootLinearVelocitiesOvStage(const TensorDesc* srcTensor,
                                                         const PxU32* rows,
                                                         const PxU32 numOutputs,
                                                         const uint64_t rowsToken,
                                                         const OvStageDeviceWriteArgs& deviceArgs)
{
    return setRootAttributeOvStage("linearVelocity", srcTensor, rows, numOutputs, rowsToken, false, false, deviceArgs);
}

bool GpuArticulationView::setRootAngularVelocitiesOvStage(const TensorDesc* srcTensor,
                                                          const PxU32* rows,
                                                          const PxU32 numOutputs,
                                                          const uint64_t rowsToken,
                                                          const OvStageDeviceWriteArgs& deviceArgs)
{
    return setRootAttributeOvStage("angularVelocity", srcTensor, rows, numOutputs, rowsToken, true, false, deviceArgs);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
    PASS_EMPTY_TENSOR(dstTensor);

    return getDofAttribute("DOF position", dstTensor, PxArticulationGPUAPIReadType::Enum::eJOINT_POSITION,
                           mGpuSimData->mCopyEvents[CopyEvent::eArtiDofPositions]);
}

bool GpuArticulationView::setDofPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
    PASS_EMPTY_TENSOR(srcTensor);

    return setDofAttribute("DOF position", srcTensor, indexTensor, PxArticulationGPUAPIWriteType::Enum::eJOINT_POSITION,
                           mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiDofPositions],
                           mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiDofPositions]);
}

bool GpuArticulationView::getDofVelocities(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
    PASS_EMPTY_TENSOR(dstTensor);

    return getDofAttribute("DOF velocity", dstTensor, PxArticulationGPUAPIReadType::Enum::eJOINT_VELOCITY,
                           mGpuSimData->mCopyEvents[CopyEvent::eArtiDofVelocities]);
}

bool GpuArticulationView::setTendonPropertyOvStage(const char* const attribName,
                                                  const TensorDesc* const srcTensor,
                                                  const PxArticulationGPUAPIReadType::Enum readFlag,
                                                  const PxArticulationGPUAPIWriteType::Enum writeFlag,
                                                  const PxU32 buffer,
                                                  void* const sharedScratchDev,
                                                  const PxU32 structFloats,
                                                  const PxU32 maxTendons,
                                                  const TendonProperty prop,
                                                  const ArticulationTendonOvStageRecord* const recordsDev,
                                                  const PxU32 numOutputs,
                                                  const OvStageDeviceWriteArgs& sessionDeviceArgs,
                                                  const TendonFieldWrite mode)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
    if (!srcTensor || !srcTensor->data)
        return false;
    if (numOutputs == 0)
        return true; // nothing to publish -- not a failure
    if (maxTendons == 0 || !sharedScratchDev)
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
    // A one-sided limit write addresses the pair but carries one value per row.
    if (mode != TendonFieldWrite::eSet && prop != TendonProperty::eLimit)
    {
        CARB_LOG_ERROR("%s: a one-sided limit write needs the limit property", attribName);
        return false;
    }
    const PxU32 srcComp = mode == TendonFieldWrite::eSet ? comp : 1u;
    if (!checkTensorDevice(*srcTensor, mDevice, attribName, __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, attribName, __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, numOutputs * srcComp, attribName, __FUNCTION__))
    {
        return false;
    }

    const bool session = sessionDeviceArgs.sessionOwned();
    const OvStageDeviceWriteArgs deviceArgs =
        session ? sessionDeviceArgs : drainDeviceWriteArgs(ApplyEvent::eArtiTendonProperties);
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    PxScene* scene = mGpuSimData->mScene;
    const PxU32 numArtis = getCount();
    // A session's tendon block is its own; the drain uses the scene's shared one, which it releases after
    // the apply (ADR-0008 Decision 7).
    const size_t scratchFloats = size_t(numArtis) * maxTendons * structFloats;
    void* const scratchDev =
        session ? static_cast<void*>(OvStageScratch(deviceArgs.scratch).take<float>(scratchFloats)) : sharedScratchDev;
    if (session && !deviceArgs.scratchHolds(static_cast<float*>(scratchDev) + scratchFloats, attribName))
        return false;

    // Read-modify-write. Status IS checked: PhysX refuses these calls in states the caller cannot
    // see and writes NOTHING when it does; the scratch is not zeroed, so an unchecked refusal would
    // overlay the caller's values onto stale contents and publish that as tendon state.
    if (!physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().getArticulationData(scratchDev, mArtiGpuIndicesDev, readFlag, numArtis,
                                                                start, finish);
        }))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::getArticulationData refused the read that preserves the tendon "
                       "properties sharing a struct with this one",
                       attribName);
        return false;
    }

    // A tendon a masked commit does not select keeps the struct just read.
    if (!submitArtiTendonPropertyOvStage(static_cast<float*>(scratchDev),
                                         static_cast<const float*>(srcTensor->data), numOutputs, maxTendons,
                                         structFloats, static_cast<PxU32>(prop), comp, recordsDev, deviceArgs.stream,
                                         deviceArgs.mask, mode))
    {
        CARB_LOG_ERROR("%s: ovstage tendon scatter failed", attribName);
        return false;
    }

    // mArtiGpuIndicesDev, not a compacted list: the whole view is pushed and the articulations this
    // query did not match are written their own current values, which is why the RMW above covers
    // the whole view. A subset write would need an index list whose LENGTH comes off the host. The
    // write's stream waits for the apply, so the next producer of the block cannot overwrite it mid-apply.
    if (!physxOnStream(deviceArgs, [&](CUevent start, CUevent finish) {
            return scene->getDirectGPUAPI().setArticulationData(scratchDev, mArtiGpuIndicesDev, writeFlag, numArtis,
                                                                start, finish);
        }))
    {
        CARB_LOG_ERROR("%s: PxDirectGPUAPI::setArticulationData refused the write", attribName);
        return false;
    }
    if (!session)
        mGpuSimData->recordKernelDone(buffer);
    return true;
}

bool GpuArticulationView::setFixedTendonPropertiesOvStage(const TensorDesc* srcTensor,
                                                          const ArticulationTendonOvStageRecord* recordsDev,
                                                          const PxU32 numOutputs, const TendonProperty prop,
                                                          const OvStageDeviceWriteArgs& deviceArgs,
                                                          const TendonFieldWrite mode)
{
    return setTendonPropertyOvStage("fixed tendon property (ovstage write)", srcTensor,
                                    PxArticulationGPUAPIReadType::Enum::eFIXED_TENDON,
                                    PxArticulationGPUAPIWriteType::Enum::eFIXED_TENDON,
                                    SharedDeviceBuffer::eFixedTendonProperties,
                                    (void*)mGpuSimData->mFixedTendonPropertiesDev,
                                    sizeof(PxGpuFixedTendonData) / sizeof(float),
                                    mGpuSimData->mMaxFixedTendons, prop, recordsDev, numOutputs, deviceArgs, mode);
}

bool GpuArticulationView::setSpatialTendonPropertiesOvStage(const TensorDesc* srcTensor,
                                                            const ArticulationTendonOvStageRecord* recordsDev,
                                                            const PxU32 numOutputs, const TendonProperty prop,
                                                            const OvStageDeviceWriteArgs& deviceArgs)
{
    return setTendonPropertyOvStage("spatial tendon property (ovstage write)", srcTensor,
                                    PxArticulationGPUAPIReadType::Enum::eSPATIAL_TENDON,
                                    PxArticulationGPUAPIWriteType::Enum::eSPATIAL_TENDON,
                                    SharedDeviceBuffer::eSpatialTendonProperties,
                                    (void*)mGpuSimData->mSpatialTendonPropertiesDev,
                                    sizeof(PxGpuSpatialTendonData) / sizeof(float),
                                    mGpuSimData->mMaxSpatialTendons, prop, recordsDev, numOutputs, deviceArgs,
                                    TendonFieldWrite::eSet);
}

bool GpuArticulationView::setDofVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
    PASS_EMPTY_TENSOR(srcTensor);

    return setDofAttribute("DOF velocity", srcTensor, indexTensor, PxArticulationGPUAPIWriteType::Enum::eJOINT_VELOCITY,
                           mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiDofVelocities],
                           mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiDofVelocities]);
}

bool GpuArticulationView::setDofActuationForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
    PASS_EMPTY_TENSOR(dstTensor);

    return getDofAttribute("DOF position target", dstTensor,
                           PxArticulationGPUAPIReadType::Enum::eJOINT_TARGET_POSITION,
                           mGpuSimData->mCopyEvents[CopyEvent::eArtiDofPositionTargets]);
}

bool GpuArticulationView::setDofPositionTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
    PASS_EMPTY_TENSOR(srcTensor);

    return setDofAttribute("DOF position target", srcTensor, indexTensor,
                           PxArticulationGPUAPIWriteType::Enum::eJOINT_TARGET_POSITION,
                           mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiDofPositionTargets],
                           mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiDofPositionTargets]);
}

bool GpuArticulationView::getDofVelocityTargets(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
    PASS_EMPTY_TENSOR(dstTensor);

    return getDofAttribute("DOF velocity target", dstTensor,
                           PxArticulationGPUAPIReadType::Enum::eJOINT_TARGET_VELOCITY,
                           mGpuSimData->mCopyEvents[CopyEvent::eArtiDofVelocityTargets]);
}

bool GpuArticulationView::setDofVelocityTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
    PASS_EMPTY_TENSOR(srcTensor);

    return setDofAttribute("DOF velocity target", srcTensor, indexTensor,
                           PxArticulationGPUAPIWriteType::Enum::eJOINT_TARGET_VELOCITY,
                           mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiDofVelocityTargets],
                           mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiDofVelocityTargets]);
}

bool GpuArticulationView::getDofActuationForces(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);

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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    // Zeroed for the reason spelled out in readOvStageColumns: the PhysX COM kernel accumulates into its
    // destination. Same stream as the start event, so the zeroing precedes the DirectGPU compute.
    if (!mMassCenterStartEvent ||
        !CHECK_CUDA(cudaMemsetAsync(dstTensor->data, 0, size_t(getCount()) * sizeof(PxVec3), nullptr)) ||
        !CHECK_CU(getCudaShim()->eventRecord(reinterpret_cast<uintptr_t>(mMassCenterStartEvent), uintptr_t(0),
                                             nullptr)))
    {
        return false;
    }
    if (!scene->getDirectGPUAPI().computeArticulationData(
            (void*)(dstTensor->data), mArtiGpuIndicesDev,
            localFrame ? PxArticulationGPUAPIComputeType::eARTICULATION_COMS_ROOT_FRAME :
                         PxArticulationGPUAPIComputeType::eARTICULATION_COMS_WORLD_FRAME,
            getCount(), mMassCenterStartEvent, copyEvent))
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);

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
        if (backend->getStepCount(mGpuSimData->mAttachHandle) == 0)
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
        if (backend->getStepCount(mGpuSimData->mAttachHandle) == 0)
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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
    GPUAPI_CHECK_READY_ARTICULATIONS(mGpuSimData, true, false);
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

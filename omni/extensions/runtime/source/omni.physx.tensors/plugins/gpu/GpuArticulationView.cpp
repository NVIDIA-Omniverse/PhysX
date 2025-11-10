// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "CudaKernels.h"
#include "GpuArticulationView.h"
#include "GpuSimulationView.h"

#include "../GlobalsAreBad.h"
#include "../CommonTypes.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

#include <omni/physics/tensors/TensorUtils.h>

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

    // physx arti indices
    mArtiIndices.resize(numArtis);

    // articulation view indices
    std::vector<::physx::PxU32> mViewIndices;
    mViewIndices.resize(numArtis);

    // printf("%u articulation indices:", numArtis);
    for (PxU32 i = 0; i < numArtis; i++)
    {
        mArtiIndices[i] = mEntries[i].arti->getGPUIndex();
        mViewIndices[i] = i;
        // printf(" %u", mArtiIndices[i]);
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
        // printf("+++ Arti idx %u, root node index: %u\n", rootRecords[i].physxArtiIdx,
        // mEntries[i].links[0]->getInternalIslandNodeIndex().index());
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
            data.body0IsParent = mEntries[i].metatype->isDofBody0Parent(j);
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

        CHECK_CUDA(cudaFree(mArtiGpuIndicesDev));
        CHECK_CUDA(cudaFree(mViewIndicesDev));
        CHECK_CUDA(cudaFree(mDofRecordsDev));
        CHECK_CUDA(cudaFree(mRootRecordsDev));
        CHECK_CUDA(cudaFree(mLinkRecordsDev));
        CHECK_CUDA(cudaFree(mFixedTendonRecordsDev));
        CHECK_CUDA(cudaFree(mSpatialTendonRecordsDev));
        CHECK_CUDA(cudaFree(mDirtyArtiGpuIndicesDev));
        CHECK_CUDA(cudaFree(cMassLocalPosePosDev));


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


    // printf("this= %p numArtis= %d, mLinkOrRootTransformsDev= %p, mArtiGpuIndicesDev= %p\n", this, numArtis,
    //        mLinkOrRootTransformsDev, mArtiGpuIndicesDev);
    // printData(mLinkOrRootTransformsDev, mArtiGpuIndicesDev, getCount() * mMaxLinks, mMaxLinks);

    scene->getDirectGPUAPI().getArticulationData((void*) mGpuSimData->mLinkOrRootTransformsDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, numArtis, nullptr,
                                                 copyEvent);


    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));

    SYNCHRONIZE_CUDA();

    if (!fetchArtiLinkTransforms(static_cast<TensorTransform*>(dstTensor->data),  mGpuSimData->mLinkOrRootTransformsDev,
                                 numArtis * mMaxLinks, mMaxLinks, mGpuSimData->mMaxLinks, mLinkRecordsDev))
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

    scene->getDirectGPUAPI().getArticulationData(
        (void*)linkDataLinearDev, artiGpuIndicesDev, linkLinearType, numArtis, nullptr, artiCopyEventLin);
    scene->getDirectGPUAPI().getArticulationData(
        (void*)linkDataAngularDev, artiGpuIndicesDev, linkAngularType, numArtis, nullptr, artiCopyEventAng);

    CHECK_CU(cuStreamWaitEvent(nullptr, artiCopyEventLin, 0));
    CHECK_CU(cuStreamWaitEvent(nullptr, artiCopyEventAng, 0));

    SYNCHRONIZE_CUDA();

    if (!fetchArtiLinkVelocitiesAccelerations(static_cast<TensorVelAcc*>(dstTensor->data), linkDataLinearDev,
                                              linkDataAngularDev, numArtis * maxLinks, maxLinks, gpuSimData->mMaxLinks))
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
    scene->getDirectGPUAPI().getArticulationData((void*) mGpuSimData->mLinkOrRootTransformsDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eROOT_GLOBAL_POSE, numArtis, nullptr,
                                                 copyEvent);
    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));

    SYNCHRONIZE_CUDA();

    if (!fetchArtiRootTransforms(
            static_cast<TensorTransform*>(dstTensor->data),  mGpuSimData->mLinkOrRootTransformsDev, numArtis, mRootRecordsDev))
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
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
    CHECK_CU(cuEventRecord(mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiRootTransforms], nullptr));
    scene->getDirectGPUAPI().setArticulationData((void*) mGpuSimData->mLinkOrRootTransformsDev, mDirtyArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIWriteType::eROOT_GLOBAL_POSE, numIndices,
                                                 mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiRootTransforms],
                                                 mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootTransforms]);
    CHECK_CU(cuEventSynchronize(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootTransforms]));
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

    // Use link buffers to avoid more memory usage
    scene->getDirectGPUAPI().getArticulationData((void*) mGpuSimData->mLinkOrRootLinearVelAccDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eROOT_LINEAR_VELOCITY, numArtis, nullptr,
                                                 artiCopyEventLin);

    scene->getDirectGPUAPI().getArticulationData((void*) mGpuSimData->mLinkOrRootAngularVelAccDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eROOT_ANGULAR_VELOCITY, numArtis,
                                                 nullptr, artiCopyEventAng);
    if (artiCopyEventLin)
    {
        CHECK_CU(cuStreamWaitEvent(nullptr, artiCopyEventLin, 0));
    }
    if (artiCopyEventAng)
    {
        CHECK_CU(cuStreamWaitEvent(nullptr, artiCopyEventAng, 0));
    }
    SYNCHRONIZE_CUDA();

    if (!fetchArtiRootVelocities(static_cast<TensorVelAcc*>(dstTensor->data),  mGpuSimData->mLinkOrRootLinearVelAccDev,
                                  mGpuSimData->mLinkOrRootAngularVelAccDev, numArtis))
    {
        CARB_LOG_ERROR("Failed to fetch articulation root velocities");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
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
    if (!submitArtiRootVelocities( mGpuSimData->mLinkOrRootLinearVelAccDev,  mGpuSimData->mLinkOrRootAngularVelAccDev,
                                  static_cast<const TensorVelAcc*>(srcTensor->data), indices, mDirtyArtiGpuIndicesDev,
                                  numIndices, (PxU32)getCount(), mRootRecordsDev))
    {
        CARB_LOG_ERROR("Failed to submit articulation root velocities");
        return false;
    }
    // CHECK_CUDA(cudaStreamSynchronize(nullptr));
    CHECK_CU(cuEventRecord(mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiRootLinVelocities], nullptr));
    CHECK_CU(cuEventRecord(mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiRootAngVelocities], nullptr));
    scene->getDirectGPUAPI().setArticulationData((void*) mGpuSimData->mLinkOrRootLinearVelAccDev, mDirtyArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIWriteType::eROOT_LINEAR_VELOCITY, numIndices,
                                                 mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiRootLinVelocities],
                                                 mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootLinVelocities]);
    scene->getDirectGPUAPI().setArticulationData((void*) mGpuSimData->mLinkOrRootAngularVelAccDev, mDirtyArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIWriteType::eROOT_ANGULAR_VELOCITY, numIndices,
                                                 mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiRootAngVelocities],
                                                 mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootAngVelocities]);

    CHECK_CU(cuEventSynchronize(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootLinVelocities]));
    CHECK_CU(cuEventSynchronize(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootAngVelocities]));

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

    scene->getDirectGPUAPI().getArticulationData(
        (void*) mGpuSimData->mDofScalarsDev, mArtiGpuIndicesDev, attribFlag, numArtis, nullptr, syncEvent);

    CHECK_CU(cuStreamWaitEvent(nullptr, syncEvent, 0));

    SYNCHRONIZE_CUDA();

    if (!fetchArtiDofAttribute(
            static_cast<float*>(dstTensor->data),  mGpuSimData->mDofScalarsDev, mDofBufSize, mMaxDofs, mGpuSimData->mMaxDofs, mDofRecordsDev))
    {
        CARB_LOG_ERROR("Failed to fetch %s attribute", attribName);
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
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
    CHECK_CU(cuEventRecord(mApplyWaitEvents, nullptr));
    scene->getDirectGPUAPI().setArticulationData(
        (void*) mGpuSimData->mDofScalarsDev, mDirtyArtiGpuIndicesDev, attribFlag, numIndices, mApplyWaitEvents, mApplySignalEvents);
    CHECK_CU(cuEventSynchronize(mApplySignalEvents));

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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
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
        scene->getDirectGPUAPI().getArticulationData((void*) mGpuSimData->mLinkOrRootTransformsDev, mArtiGpuIndicesDev,
                                                     PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, numArtis, nullptr,
                                                     artiCopyEvent);
        CHECK_CU(cuStreamWaitEvent(nullptr, artiCopyEvent, 0));
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
    if (!submitArtiLinkForces(mGpuSimData->mLinkForcesDev, mGpuSimData->mLinkTorquesDev, mDirtyArtiGpuIndicesDev,
                              mGpuSimData->mLinkOrRootTransformsDev, cMassLocalPosePosDev, forceData, torqueData,
                              positionData, indices, numIndices, numIndices * mMaxLinks, mGpuSimData->mMaxLinks,
                              mLinkRecordsDev, isGlobal, validForceTensor, validTorqueTensor, validPositionTensor))
    {
        CARB_LOG_ERROR("Failed to submit articulation link forces");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    SYNCHRONIZE_CUDA();
    if (validForceTensor)
    {
        CHECK_CU(cuEventRecord(mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiLinkForces], nullptr));
        scene->getDirectGPUAPI().setArticulationData((void*) mGpuSimData->mLinkForcesDev, mDirtyArtiGpuIndicesDev,
                                                     PxArticulationGPUAPIWriteType::eLINK_FORCE, numIndices,
                                                     mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiLinkForces],
                                                     mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkForces]);
        CHECK_CU(cuEventSynchronize(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkForces]));
        mGpuSimData->mLinkForcesApplied = true;
    }
    if (validPositionTensor || validTorqueTensor)
    {
        CHECK_CU(cuEventRecord(mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiLinkTorques], nullptr));
        scene->getDirectGPUAPI().setArticulationData((void*) mGpuSimData->mLinkTorquesDev, mDirtyArtiGpuIndicesDev,
                                                     PxArticulationGPUAPIWriteType::eLINK_TORQUE, numIndices,
                                                     mGpuSimData->mApplyWaitEvents[ApplyEvent::eArtiLinkTorques],
                                                     mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkTorques]);
        CHECK_CU(cuEventSynchronize(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkTorques]));
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

    if (!checkTensorDevice(*dstTensor, mDevice, "Jacobian", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Jacobian", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * jacobianSize, "Jacobian", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;

    SYNCHRONIZE_CUDA();

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiJacobians];

    scene->getDirectGPUAPI().computeArticulationData((void*)mGpuSimData->mJacobianDataDev, mArtiGpuIndicesDev,
                                                     PxArticulationGPUAPIComputeType::eDENSE_JACOBIANS, getCount(),
                                                     nullptr, copyEvent);
    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));
    SYNCHRONIZE_CUDA();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    if (!fetchArtiJacobian(static_cast<float*>(dstTensor->data), mGpuSimData->mJacobianDataDev,
                           getCount() * jacobianRows * jacobianCols, jacobianRows * jacobianCols,
                           mGpuSimData->mJacobianMaxRows * mGpuSimData->mJacobianMaxCols))
    {
        CARB_LOG_ERROR("Failed to fetch Jacobian tensor");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

// DEPRECATED
bool GpuArticulationView::getMassMatrices(const TensorDesc* dstTensor) const
{
    CARB_LOG_WARN("DEPRECATED: Please use getGeneralizedMassMatrices instead.");
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    uint32_t massMatrixRows = 0;
    uint32_t massMatrixCols = 0;

    // this will fail if view is not homogeneous
    if (!getMassMatrixShape(&massMatrixRows, &massMatrixCols))
    {
        return false;
    }

    uint32_t massMatrixSize = massMatrixRows * massMatrixCols;

    if (!checkTensorDevice(*dstTensor, mDevice, "Mass Matrix", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Mass Matrix", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * massMatrixSize, "Mass Matrix", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;

    SYNCHRONIZE_CUDA();

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiMassMatrices];

    scene->getDirectGPUAPI().computeArticulationData((void*)mGpuSimData->mMassMatrixDataDev, mArtiGpuIndicesDev,
                                                     PxArticulationGPUAPIComputeType::eGENERALIZED_MASS_MATRICES,
                                                     getCount(), nullptr, copyEvent);

    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));

    SYNCHRONIZE_CUDA();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    SYNCHRONIZE_CUDA();

    if (!fetchArtiMassMatrices(static_cast<float*>(dstTensor->data), mGpuSimData->mMassMatrixDataDev,
                                                 getCount() * massMatrixSize, massMatrixSize,
                                                 mGpuSimData->mMaxDofs * mGpuSimData->mMaxDofs))
    {
        CARB_LOG_ERROR("Failed to fetch generalized mass matrices attribute");
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
    if (!getMassMatrixShape(&massMatrixRows, &massMatrixCols))
    {
        return false;
    }

    if (!isFixedBase)
    {
        massMatrixRows += 6;
        massMatrixCols += 6;
    }
    uint32_t massMatrixSize = massMatrixRows * massMatrixCols;

    if (!checkTensorDevice(*dstTensor, mDevice, "Mass Matrix", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Mass Matrix", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * massMatrixSize, "Mass Matrix", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;

    SYNCHRONIZE_CUDA();

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiMassMatrices];

    scene->getDirectGPUAPI().computeArticulationData((void*)mGpuSimData->mMassMatrixDataDev, mArtiGpuIndicesDev,
                                                     PxArticulationGPUAPIComputeType::eMASS_MATRICES,
                                                     getCount(), nullptr, copyEvent);

    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));

    SYNCHRONIZE_CUDA();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    SYNCHRONIZE_CUDA();

    if (!fetchArtiMassMatrices(static_cast<float*>(dstTensor->data), mGpuSimData->mMassMatrixDataDev,
                                                 getCount() * massMatrixSize, massMatrixSize, simMassMatrixSize))
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
    SYNCHRONIZE_CUDA();

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiMassCenter];
    scene->getDirectGPUAPI().computeArticulationData(
        (void*)(dstTensor->data), mArtiGpuIndicesDev,
        localFrame ? PxArticulationGPUAPIComputeType::eARTICULATION_COMS_ROOT_FRAME :
                     PxArticulationGPUAPIComputeType::eARTICULATION_COMS_WORLD_FRAME,
        getCount(), nullptr, copyEvent);
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

bool GpuArticulationView::getArticulationCentroidalMomentum(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);


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

    const PxU32 centroidalMomentumMatricesBlockSize = 6 * (mMaxDofs + 7);
    const PxU32 simCentroidalMomentumMatricesBlockSize = 6 * (mGpuSimData->mMaxDofs + 6);
    const PxU32 simMassMatricesBlockSize = (mGpuSimData->mMaxDofs + 6) * (mGpuSimData->mMaxDofs + 6);
    const PxU32 simCoriolisForcesBlockSize = mGpuSimData->mMaxDofs + 6;
    const PxU32 simStartCoriolisForces = simMassMatricesBlockSize * getCount();
    const PxU32 simStartCentroidalMomentumMatrix = (simMassMatricesBlockSize + simCoriolisForcesBlockSize) * getCount();
	const PxU32 startSimBiasForceBlock = simCentroidalMomentumMatricesBlockSize * getCount();

    PxScene* scene = mGpuSimData->mScene;
    SYNCHRONIZE_CUDA();

    CUevent copyEventMass = mGpuSimData->mCopyEvents[CopyEvent::eArtiMassMatrices];
    scene->getDirectGPUAPI().computeArticulationData(
        (void*)mGpuSimData->mCentroidalMomentumDataDev, mArtiGpuIndicesDev,
        PxArticulationGPUAPIComputeType::eMASS_MATRICES, getCount(), nullptr, copyEventMass);
    CHECK_CU(cuEventSynchronize(copyEventMass));
    CUevent copyEventCoriolis = mGpuSimData->mCopyEvents[CopyEvent::eArtiMassMatrices];
    scene->getDirectGPUAPI().computeArticulationData(
        (void*)(mGpuSimData->mCentroidalMomentumDataDev + simStartCoriolisForces), mArtiGpuIndicesDev,
        PxArticulationGPUAPIComputeType::eCORIOLIS_AND_CENTRIFUGAL_COMPENSATION, getCount(), nullptr, copyEventCoriolis);
    CHECK_CU(cuEventSynchronize(copyEventCoriolis));
    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiCoriolisCentrifugal];
    scene->getDirectGPUAPI().computeArticulationData(
        (void*)mGpuSimData->mCentroidalMomentumDataDev, mArtiGpuIndicesDev,
        PxArticulationGPUAPIComputeType::eCENTROIDAL_MOMENTUM_MATRICES, getCount(), nullptr, copyEvent);
    CHECK_CU(cuEventSynchronize(copyEvent));

    if (!fetchArtiCentroidalMomentumMatrices(static_cast<float*>(dstTensor->data),
                                             mGpuSimData->mCentroidalMomentumDataDev + simStartCentroidalMomentumMatrix,
                                             getCount() * centroidalMomentumMatricesBlockSize, mMaxDofs,
                                             mGpuSimData->mMaxDofs, centroidalMomentumMatricesBlockSize,
                                             simCentroidalMomentumMatricesBlockSize, startSimBiasForceBlock))
    {
        CARB_LOG_ERROR("Failed to fetch centroidal momentum data");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

// DEPRECATED
bool GpuArticulationView::getCoriolisAndCentrifugalForces(const TensorDesc* dstTensor) const
{
    CARB_LOG_WARN("DEPRECATED: Please use getCoriolisAndCentrifugalCompensationForces instead.");
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "Coriolis and centrifugal forces", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Coriolis and centrifugal forces", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "Coriolis and centrifugal forces", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;

    SYNCHRONIZE_CUDA();

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiCoriolisCentrifugal];

    scene->getDirectGPUAPI().computeArticulationData((void*)mGpuSimData->mCoriolisGravityDataDev, mArtiGpuIndicesDev,
                                                     PxArticulationGPUAPIComputeType::eCORIOLIS_AND_CENTRIFUGAL_FORCES,
                                                     getCount(), nullptr, copyEvent);

    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));

    SYNCHRONIZE_CUDA();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    SYNCHRONIZE_CUDA();
    if (!fetchArtiDofAttributeGravityAndCoriolis(static_cast<float*>(dstTensor->data), mGpuSimData->mCoriolisGravityDataDev,
                                                 getCount() * mMaxDofs, mMaxDofs, mGpuSimData->mMaxDofs, mDofRecordsDev, false))
    {
        CARB_LOG_ERROR("Failed to fetch coriolis and centrifugal forces attribute");
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

    const bool isFixedBase = mEntries[0].metatype->getFixedBase();
    const PxU32 maxDofs = isFixedBase ? mMaxDofs : mMaxDofs + 6;
    const PxU32 simMaxDofs = mGpuSimData->mMaxDofs + 6;
    const bool rootDofs = !isFixedBase;

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

    PxScene* scene = mGpuSimData->mScene;

    SYNCHRONIZE_CUDA();

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiCoriolisCentrifugal];

    scene->getDirectGPUAPI().computeArticulationData((void*)mGpuSimData->mCoriolisGravityDataDev, mArtiGpuIndicesDev,
                                                     PxArticulationGPUAPIComputeType::eCORIOLIS_AND_CENTRIFUGAL_COMPENSATION,
                                                     getCount(), nullptr, copyEvent);

    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));

    SYNCHRONIZE_CUDA();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    SYNCHRONIZE_CUDA();

    if (!fetchArtiDofAttributeGravityAndCoriolis(static_cast<float*>(dstTensor->data), mGpuSimData->mCoriolisGravityDataDev,
                                                 getCount() * maxDofs, maxDofs, simMaxDofs, mDofRecordsDev, rootDofs))
    {
        CARB_LOG_ERROR("Failed to fetch coriolis and centrifugal forces attribute");
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

// DEPRECATED
bool GpuArticulationView::getGeneralizedGravityForces(const TensorDesc* dstTensor) const
{
    CARB_LOG_WARN("DEPRECATED: Please use getGravityCompensationForces instead.");
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "generalized gravity forces", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "generalized gravity forces", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "generalized gravity forces", __FUNCTION__))
    {
        return false;
    }

    PxScene* scene = mGpuSimData->mScene;

    SYNCHRONIZE_CUDA();

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiGeneralizedGravity];
    scene->getDirectGPUAPI().computeArticulationData((void*)mGpuSimData->mCoriolisGravityDataDev, mArtiGpuIndicesDev,
                                                     PxArticulationGPUAPIComputeType::eGENERALIZED_GRAVITY_FORCES,
                                                     getCount(), nullptr, copyEvent);

    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    SYNCHRONIZE_CUDA();
    if (!fetchArtiDofAttributeGravityAndCoriolis(static_cast<float*>(dstTensor->data), mGpuSimData->mCoriolisGravityDataDev,
                                                 getCount() * mMaxDofs, mMaxDofs, mGpuSimData->mMaxDofs, mDofRecordsDev, 0))
    {
        CARB_LOG_ERROR("Failed to fetch generalized gravity forces forces attribute");
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

    const bool isFixedBase = mEntries[0].metatype->getFixedBase();
    const PxU32 maxDofs = isFixedBase ? mMaxDofs : mMaxDofs + 6;
    const PxU32 simMaxDofs = mGpuSimData->mMaxDofs + 6;
    const bool rootDofs = !isFixedBase;

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

    PxScene* scene = mGpuSimData->mScene;

    SYNCHRONIZE_CUDA();

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiGeneralizedGravity];

    scene->getDirectGPUAPI().computeArticulationData((void*)mGpuSimData->mCoriolisGravityDataDev, mArtiGpuIndicesDev,
                                                     PxArticulationGPUAPIComputeType::eGRAVITY_COMPENSATION,
                                                     getCount(), nullptr, copyEvent);

    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));

    SYNCHRONIZE_CUDA();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    SYNCHRONIZE_CUDA();

    if (!fetchArtiDofAttributeGravityAndCoriolis(static_cast<float*>(dstTensor->data), mGpuSimData->mCoriolisGravityDataDev,
                                                 getCount() * maxDofs, maxDofs, simMaxDofs, mDofRecordsDev, rootDofs))
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

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiLinkIncomingJointForce];
    CUevent copyEventTransforms = mGpuSimData->mCopyEvents[CopyEvent::eArtiLinkTransforms];
    scene->getDirectGPUAPI().getArticulationData((void*) mGpuSimData->mLinkIncomingJointForceDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eLINK_INCOMING_JOINT_FORCE, numArtis,
                                                 nullptr, copyEvent);
    scene->getDirectGPUAPI().getArticulationData((void*) mGpuSimData->mLinkOrRootTransformsDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, numArtis, nullptr,
                                                 copyEventTransforms);

    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));
    CHECK_CU(cuStreamWaitEvent(nullptr, copyEventTransforms, 0));
    SYNCHRONIZE_CUDA();

    if (!fetchArtiLinkIncomingJointForce(static_cast<PhysxGpuSpatialForces*>(dstTensor->data),
                                         mGpuSimData->mLinkIncomingJointForceDev, mGpuSimData->mLinkOrRootTransformsDev,
                                         getCount() * mMaxLinks, mMaxLinks, mGpuSimData->mMaxLinks, mLinkRecordsDev))
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

    CUevent copyEvent = mGpuSimData->mCopyEvents[CopyEvent::eArtiLinkIncomingJointForce];
    CUevent copyEventTransforms = mGpuSimData->mCopyEvents[CopyEvent::eArtiLinkTransforms];
    scene->getDirectGPUAPI().getArticulationData((void*) mGpuSimData->mLinkIncomingJointForceDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eLINK_INCOMING_JOINT_FORCE, numArtis,
                                                 nullptr, copyEvent);
    scene->getDirectGPUAPI().getArticulationData((void*) mGpuSimData->mLinkOrRootTransformsDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, numArtis, nullptr,
                                                 copyEventTransforms);
    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));
    CHECK_CU(cuStreamWaitEvent(nullptr, copyEventTransforms, 0));
    SYNCHRONIZE_CUDA();

    if (!fetchDofProjectionForce(static_cast<float*>(dstTensor->data), mGpuSimData->mLinkIncomingJointForceDev,
                                 mGpuSimData->mLinkOrRootTransformsDev, getCount() * mMaxLinks, mMaxLinks,
                                 mGpuSimData->mMaxLinks, mLinkRecordsDev))
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
    scene->getDirectGPUAPI().getArticulationData((void*)mGpuSimData->mFixedTendonPropertiesDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eFIXED_TENDON, numArtis, nullptr,
                                                 copyEvent);
    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));
    SYNCHRONIZE_CUDA();

    if (!fetchFixedTendonStiffness(static_cast<float*>(dstTensor->data), mGpuSimData->mFixedTendonPropertiesDev,
                                   getCount() * mMaxFixedTendons, mMaxFixedTendons, mGpuSimData->mMaxFixedTendons))
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
    scene->getDirectGPUAPI().getArticulationData((void*)mGpuSimData->mFixedTendonPropertiesDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eFIXED_TENDON, numArtis, nullptr,
                                                 copyEvent);
    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));
    SYNCHRONIZE_CUDA();

    if (!fetchFixedTendonDamping(static_cast<float*>(dstTensor->data), mGpuSimData->mFixedTendonPropertiesDev,
                                 getCount() * mMaxFixedTendons, mMaxFixedTendons, mGpuSimData->mMaxFixedTendons))
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
    scene->getDirectGPUAPI().getArticulationData((void*)mGpuSimData->mFixedTendonPropertiesDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eFIXED_TENDON, numArtis, nullptr,
                                                 copyEvent);
    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));
    SYNCHRONIZE_CUDA();

    if (!fetchFixedTendonLimitStiffness(static_cast<float*>(dstTensor->data), mGpuSimData->mFixedTendonPropertiesDev,
                                        getCount() * mMaxFixedTendons, mMaxFixedTendons, mGpuSimData->mMaxFixedTendons))
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
    scene->getDirectGPUAPI().getArticulationData((void*)mGpuSimData->mFixedTendonPropertiesDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eFIXED_TENDON, numArtis, nullptr,
                                                 copyEvent);
    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));
    SYNCHRONIZE_CUDA();

    if (!fetchFixedTendonLimits(static_cast<float*>(dstTensor->data), mGpuSimData->mFixedTendonPropertiesDev,
                                getCount() * mMaxFixedTendons, mMaxFixedTendons, mGpuSimData->mMaxFixedTendons))
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
    scene->getDirectGPUAPI().getArticulationData((void*)mGpuSimData->mFixedTendonPropertiesDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eFIXED_TENDON, numArtis, nullptr,
                                                 copyEvent);
    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));
    SYNCHRONIZE_CUDA();

    if (!fetchFixedTendonRestLength(static_cast<float*>(dstTensor->data), mGpuSimData->mFixedTendonPropertiesDev,
                                    getCount() * mMaxFixedTendons, mMaxFixedTendons, mGpuSimData->mMaxFixedTendons))
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
    scene->getDirectGPUAPI().getArticulationData((void*)mGpuSimData->mFixedTendonPropertiesDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eFIXED_TENDON, numArtis, nullptr,
                                                 copyEvent);
    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));
    SYNCHRONIZE_CUDA();

    if (!fetchFixedTendonOffset(static_cast<float*>(dstTensor->data), mGpuSimData->mFixedTendonPropertiesDev,
                                getCount() * mMaxFixedTendons, mMaxFixedTendons, mGpuSimData->mMaxFixedTendons))
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
    scene->getDirectGPUAPI().getArticulationData((void*)mGpuSimData->mSpatialTendonPropertiesDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eSPATIAL_TENDON, numArtis, nullptr,
                                                 copyEvent);
    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));
    SYNCHRONIZE_CUDA();

    if (!fetchSpatialTendonStiffness(static_cast<float*>(dstTensor->data), mGpuSimData->mSpatialTendonPropertiesDev,
                                     getCount() * mMaxSpatialTendons, mMaxSpatialTendons, mGpuSimData->mMaxSpatialTendons))
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
    scene->getDirectGPUAPI().getArticulationData((void*)mGpuSimData->mSpatialTendonPropertiesDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eSPATIAL_TENDON, numArtis, nullptr,
                                                 copyEvent);
    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));
    SYNCHRONIZE_CUDA();

    if (!fetchSpatialTendonDamping(static_cast<float*>(dstTensor->data), mGpuSimData->mSpatialTendonPropertiesDev,
                                   getCount() * mMaxSpatialTendons, mMaxSpatialTendons, mGpuSimData->mMaxSpatialTendons))
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
    scene->getDirectGPUAPI().getArticulationData((void*)mGpuSimData->mSpatialTendonPropertiesDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eSPATIAL_TENDON, numArtis, nullptr,
                                                 copyEvent);
    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));
    SYNCHRONIZE_CUDA();

    if (!fetchSpatialTendonLimitStiffness(static_cast<float*>(dstTensor->data),
                                          mGpuSimData->mSpatialTendonPropertiesDev, getCount() * mMaxSpatialTendons,
                                          mMaxSpatialTendons, mGpuSimData->mMaxSpatialTendons))
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
    scene->getDirectGPUAPI().getArticulationData((void*)mGpuSimData->mSpatialTendonPropertiesDev, mArtiGpuIndicesDev,
                                                 PxArticulationGPUAPIReadType::eSPATIAL_TENDON, numArtis, nullptr,
                                                 copyEvent);
    CHECK_CU(cuStreamWaitEvent(nullptr, copyEvent, 0));
    SYNCHRONIZE_CUDA();

    if (!fetchSpatialTendonOffset(static_cast<float*>(dstTensor->data), mGpuSimData->mSpatialTendonPropertiesDev,
                                  getCount() * mMaxSpatialTendons, mMaxSpatialTendons, mGpuSimData->mMaxSpatialTendons))
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
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
    CHECK_CU(cuEventSynchronize(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiTendonProperties]));

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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
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
    CHECK_CU(cuEventSynchronize(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiTendonProperties]));

    SYNCHRONIZE_CUDA();
    return true;
}

} // namespace tensors
} // namespace physx
} // namespace omni

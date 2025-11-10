// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "CudaKernels.h"
#include "GpuRigidBodyView.h"
#include "GpuSimulationView.h"

#include "../GlobalsAreBad.h"
#include "../CommonTypes.h"
#include "../SimulationBackend.h"

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
    // std::vector<PxU32> artiRootIndices;  // indices of articulations whose root links are in this view
    PxU32 numArtiRoots = 0;
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
            PxRigidDynamic* rd = static_cast<PxRigidDynamic*>(mEntries[i].body);
            PxU32 nodeIdx = rd->getInternalIslandNodeIndex().index();
            // get RD index in global staging buffer
            auto it = mGpuSimData->mNode2RdIndexMap.find(nodeIdx);
            if (it != mGpuSimData->mNode2RdIndexMap.end())
            {
                PxU32 rdIdx = it->second;
                rb.physxRdIdx = rdIdx;
                rb.tensorRdIdx = PxU32(rdGpuIndices.size());
                rdGpuIndices.push_back(rd->getGPUIndex());
                // printf("  rbIdx: %u, rdIdx: %u, nodeIdx: %u\n", i, rdIndexPair.mSrcIndex,
                // rdIndexPair.mNodeIndex.index());
            }
            else
            {
                // shouldn't happen
                CARB_LOG_ERROR("Internal error: Unresolved rigid dynamic index!");
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
    // std::sort(artiRootIndices.begin(), artiRootIndices.end());
    // mNumArtiRoots = PxU32(artiRootIndices.size());
    mNumArtiRoots = numArtiRoots;
    if (mNumRds > 0 || mNumArtis > 0)
    {
        // rd  + articulation links center of mass
        prepareDeviceData((void**)&cMassLocalPosePosDev, nullptr, (mNumArtis * mMaxLinks + mNumRds) * sizeof(PxVec3),
                          "cMassLocalPosePosDev");
    }
    cMassLocalPosePos.resize(mNumRds + mNumArtis * mMaxLinks, { 0.0f, 0.0f, 0.0f });
    updateCMassData();

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    // rb indices
    prepareDeviceData((void**)&mRbIndicesDev, rbIndices.data(), numBodies * sizeof(PxU32), "mRbIndicesDev");
    // rb indexing data
    prepareDeviceData((void**)&mRbRecordsDev, rbRecords.data(), numBodies * sizeof(GpuRigidBodyRecord), "mRbRecordsDev");

    if (mNumRds > 0)
    {
        prepareDeviceData((void**)&mRdGpuIndicesDev, rdGpuIndices.data(), mNumRds * sizeof(PxRigidDynamicGPUIndex),
                          "mRdGpuIndicesDev");
        prepareDeviceData(
            (void**)&mDirtyRdGpuIndices, nullptr, mNumRds * sizeof(PxRigidDynamicGPUIndex), "mDirtyRdGpuIndices");
        prepareDeviceData((void**)&mRdDirtyFlagsDev, nullptr, mNumRds * sizeof(ActorGpuFlags), "mRdDirtyFlagsDev");
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
        CHECK_CUDA(cudaFree(mDirtyArtiGpuIndices));
        CHECK_CUDA(cudaFree(mDirtyRdGpuIndices));
        CHECK_CUDA(cudaFree(mRdDirtyFlagsDev));
        CHECK_CUDA(cudaFree(mArtiDirtyFlagsDev));
        CHECK_CUDA(cudaFree(mArtiLinksDirtyFlagsDev));
        CHECK_CUDA(cudaFree(cMassLocalPosePosDev));

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
        CHECK_CU(cuStreamWaitEvent(nullptr, rdCopyEvent, 0));
    }

    if (artiCopyEvent)
    {
        CHECK_CU(cuStreamWaitEvent(nullptr, artiCopyEvent, 0));
    }

    SYNCHRONIZE_CUDA();

    if (!fetchRbTransforms(static_cast<TensorTransform*>(dstTensor->data),  mGpuSimData->mRdPoseDev,  mGpuSimData->mLinkOrRootTransformsDev,
                           getCount(),  mGpuSimData->mMaxLinks, mRbRecordsDev))
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
        CHECK_CU(cuStreamWaitEvent(nullptr, rdCopyEventLin, 0));
    }
    if (rdCopyEventAng)
    {
        CHECK_CU(cuStreamWaitEvent(nullptr, rdCopyEventAng, 0));
    }

    if (artiCopyEventLin)
    {
        CHECK_CU(cuStreamWaitEvent(nullptr, artiCopyEventLin, 0));
    }
    if (artiCopyEventAng)
    {
        CHECK_CU(cuStreamWaitEvent(nullptr, artiCopyEventAng, 0));
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

    // warn if there are non-root articulation links in this view, because their transforms cannot be set directly
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

    CHECK_CU(cuEventSynchronize(mGpuSimData->mApplySignalEvents[ApplyEvent::eRdData]));
    CHECK_CU(cuEventSynchronize(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootTransforms]));
    SYNCHRONIZE_CUDA();

    return true;
}

bool GpuRigidBodyView::setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);
    GPUAPI_CHECK_READY(mGpuSimData, false);

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

    // warn if there are non-root articulation links in this view, because their velocities cannot be set directly
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

    // printf("numDirtyArtiIndices =%d numDirtyRdIndices=%d\n", numDirtyArtiIndices, numDirtyRdIndices);

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
    CHECK_CU(cuEventSynchronize(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootLinVelocities]));
    CHECK_CU(cuEventSynchronize(mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiRootAngVelocities]));
    CHECK_CU(cuEventSynchronize(mGpuSimData->mApplySignalEvents[ApplyEvent::eRdLinVelocities]));
    CHECK_CU(cuEventSynchronize(mGpuSimData->mApplySignalEvents[ApplyEvent::eRdAngVelocities]));
    SYNCHRONIZE_CUDA();

    return true;
}

bool GpuRigidBodyView::applyForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CARB_LOG_WARN("Deprecated function IArticulationView::applyForces, please use IArticulationView::applyForcesAndTorquesAtPosition instead.");
    return applyForcesAndTorquesAtPosition(srcTensor, nullptr, nullptr, indexTensor, true);
}

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
        CHECK_CU(cuStreamWaitEvent(nullptr, rdCopyEvent, 0));
    }

    if (artiCopyEvent)
    {
        CHECK_CU(cuStreamWaitEvent(nullptr, artiCopyEvent, 0));
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
        // PxVec3 tmp = mEntries[idx].body->getCMassLocalPose().p;
        // printf("rbRecords idx %u mass pos = %f,%f,%f\n", idx, tmp.x, tmp.y, tmp.z);
    }
    if (!CHECK_CUDA(cudaMemcpy(cMassLocalPosePosDev, cMassLocalPosePos.data(),
                               (mNumArtis * mMaxLinks + mNumRds) * sizeof(PxVec3), cudaMemcpyHostToDevice)))
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
            CHECK_CU(cuStreamWaitEvent(nullptr, rdCopyEvent, 0));
        }

        if (artiCopyEvent)
        {
            CHECK_CU(cuStreamWaitEvent(nullptr, artiCopyEvent, 0));
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
        CHECK_CU(cuStreamWaitEvent(nullptr, mGpuSimData->mApplySignalEvents[ApplyEvent::eRdForces], 0));
        CHECK_CU(cuStreamWaitEvent(nullptr, mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkForces], 0));

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
        CHECK_CU(cuStreamWaitEvent(nullptr, mGpuSimData->mApplySignalEvents[ApplyEvent::eRdTorques], 0));
        CHECK_CU(cuStreamWaitEvent(nullptr, mGpuSimData->mApplySignalEvents[ApplyEvent::eArtiLinkTorques], 0));
        CHECK_CUDA(cudaStreamSynchronize(nullptr));
    }
    SYNCHRONIZE_CUDA();

    return true;
}
} // namespace tensors
} // namespace physx
} // namespace omni

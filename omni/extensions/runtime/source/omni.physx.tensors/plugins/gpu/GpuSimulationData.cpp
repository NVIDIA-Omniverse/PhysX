// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "GpuSimulationData.h"

#include "../GlobalsAreBad.h"
#include "../SimulationBackend.h"
#include "CudaKernels.h"

#include <omni/physx/IPhysx.h>

#include <PxPhysicsAPI.h>

using namespace physx;
using namespace pxr;

namespace omni
{
namespace physx
{
namespace tensors
{

GpuSimulationData::GpuSimulationData(SimulationBackend& backend, long stageId)
    : mBackend(backend)
    , mStageId(stageId)
{
}

bool GpuSimulationData::init(PxScene* scene)
{
    if (!scene)
    {
        CARB_LOG_ERROR("Failed to prepare GPU data: null scene");
        return false;
    }

    // check if sim is running on GPU and if direct GPU API is enabled
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

    // find maxLinks and maxDofs
    PxU32 maxLinks = 0;
    PxU32 maxDofs = 0;
    PxU32 maxFixedTendons = 0;
    PxU32 maxSpatialTendons = 0;
    PxU32 numArtis = scene->getNbArticulations();
    PxArticulationGPUIndex maxArtiIndex = numArtis; // AD: must be at least as large as the number of articulations.
    uint32_t maxArtiNodeIdx = 0;

    std::vector<PxArticulationReducedCoordinate*> artis(numArtis);
    scene->getArticulations(artis.data(), numArtis);
    for (PxU32 i = 0; i < numArtis; i++)
    {
        PxArticulationReducedCoordinate* arti = artis[i];

        // get link count
        PxU32 numLinks = arti->getNbLinks();

        // get dof count
        std::vector<PxArticulationLink*> links(numLinks);
        arti->getLinks(links.data(), numLinks);
        PxU32 numDofs = 0;
        for (PxU32 j = 0; j < numLinks; j++)
        {
            PxU32 inboundDofCount = links[j]->getInboundJointDof();
            // printf("Link %u @ %p: Inbound dofs %u\n", j, links[j], inboundDofCount);
            if (inboundDofCount != 0xffffffff)
            {
                numDofs += inboundDofCount;
            }
        }

        PxNodeIndex nodeIdx = links[0]->getInternalIslandNodeIndex();
        mNodeIdx2ArtiIdxMap[nodeIdx.index()] = arti->getGPUIndex();
        maxArtiNodeIdx = nodeIdx.index() > maxArtiNodeIdx ? nodeIdx.index() : maxArtiNodeIdx;

        const PxArticulationGPUIndex artiIdx = arti->getGPUIndex();
        PxU32 numFixedTendons = arti->getNbFixedTendons();
        PxU32 numSpatialTendons = arti->getNbSpatialTendons();

        if (numLinks > maxLinks)
        {
            maxLinks = numLinks;
        }
        if (numDofs > maxDofs)
        {
            maxDofs = numDofs;
        }
        if (numFixedTendons > maxFixedTendons)
        {
            maxFixedTendons = numFixedTendons;
        }
        if (numSpatialTendons > maxSpatialTendons)
        {
            maxSpatialTendons = numSpatialTendons;
        }
        if (artiIdx > maxArtiIndex)
        {
            maxArtiIndex = artiIdx;
        }
    }



    std::vector<uint32_t> nodeIdx2ArtiGpuIdx(maxArtiNodeIdx + 1, 0xffffffff);
    for(auto&item : mNodeIdx2ArtiIdxMap){
        nodeIdx2ArtiGpuIdx[item.first] = item.second;
    }

    // printf("~!~!~! %s: Max links: %u\n", __FUNCTION__, maxLinks);
    // printf("~!~!~! %s: Max DOFs: %u\n", __FUNCTION__, maxDofs);

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
    mMaxArtiIndex = maxArtiIndex;

    /*
    {
        CUcontext _ctx = 0;
        CHECK_CU(cuCtxGetCurrent(&_ctx));
        printf(":=:=:=:=:=:=:=: ctx @ %p (init entry)\n", _ctx);
    }
    */

    PhysxCudaContextGuard ctxGuarg(mCudaContextManager);

    /*
    {
        CUcontext _ctx = 0;
        CHECK_CU(cuCtxGetCurrent(&_ctx));
        printf(":=:=:=:=:=:=:=: ctx @ %p (init guarded)\n", _ctx);
    }
    */

    CHECK_CU(cuCtxGetCurrent(&mCtx));

    VALIDATE_CUDA_CONTEXT();

    // determine the device ordinal that PhysX is using
    {
        int deviceCount = 0;
        if (!CHECK_CU(cuDeviceGetCount(&deviceCount)))
        {
            CARB_LOG_ERROR("Failed to get device count");
            return false;
        }

        std::map<CUdevice, int> deviceMap;
        for (int i = 0; i < deviceCount; i++)
        {
            CUdevice device;
            if (CHECK_CU(cuDeviceGet(&device, i)))
            {
                deviceMap[device] = i;
            }
        }

        CUdevice device;
        if (CHECK_CU(cuCtxGetDevice(&device)))
        {
            auto it = deviceMap.find(device);
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

    if (numArtis > 0)
    {
        // node indices to global articulation indices map
        prepareDeviceData((void**)&mNodeIdx2ArtiGpuIdxDev, nodeIdx2ArtiGpuIdx.data(),
                          (size_t)nodeIdx2ArtiGpuIdx.size() * sizeof(PxU32), "mNodeIdx2ArtiGpuIdxDev");
    }

    if (mMaxDofs > 0)
    {
        // arti DOF positions/velocity/targets/etc
        prepareDeviceData((void**)&mDofScalarsDev, nullptr, mDofBufSize * sizeof(float), "mDofScalarsDev");
        // arti DOF actuation forces
        prepareDeviceData(
            (void**)&mDofActuationForcesDev, nullptr, mDofBufSize * sizeof(float), "mDofActuationForcesDev");

        mJacobianMaxCols = 6 + mMaxDofs;
        mJacobianMaxRows = 6 + (mMaxLinks - 1) * 6;
        uint32_t jacobianSize = mJacobianMaxCols * mJacobianMaxRows;
        uint32_t jacobianBufSize = numArtis * jacobianSize;
        prepareDeviceData((void**)&mJacobianDataDev, nullptr, jacobianBufSize * sizeof(float), "mJacobianDataDev");

        uint32_t coriolisGravityBuffSize = numArtis * (6 + mMaxDofs);
        prepareDeviceData((void**)&mCoriolisGravityDataDev, nullptr, coriolisGravityBuffSize * sizeof(float), "mCoriolisGravityDataDev");

        uint32_t massMatrixBuffSize = numArtis * (6 + mMaxDofs) * (6 + mMaxDofs);
        prepareDeviceData((void**)&mMassMatrixDataDev, nullptr, massMatrixBuffSize * sizeof(float), "mMassMatrixDataDev");
        uint32_t sizeMassMatrices = numArtis * (maxDofs + 6) * (maxDofs + 6);
        uint32_t sizeCoriolisForces = numArtis * (maxDofs + 6);
        uint32_t sizeCentroidalMomentumMatrices = numArtis * 6 * (maxDofs + 6);
        uint32_t sizeCentroidalMomentumBiasForces = numArtis * 6;
        uint32_t sizeCentroidalMomentumData =
            sizeMassMatrices + sizeCoriolisForces + sizeCentroidalMomentumMatrices + sizeCentroidalMomentumBiasForces;
        prepareDeviceData((void**)&mCentroidalMomentumDataDev, nullptr, sizeCentroidalMomentumData * sizeof(float),
                          "mCentroidalMomentumDataDev");

    }
    
    if (mMaxLinks > 0)
    {
        prepareDeviceData(
            (void**)&mLinkOrRootTransformsDev, nullptr, mLinkBufSize * sizeof(PxTransform), "mLinkOrRootTransformsDev");

        prepareDeviceData(
            (void**)&mLinkOrRootLinearVelAccDev, nullptr, mLinkBufSize * sizeof(PxVec3), "mLinkOrRootLinearVelAccDev");

        prepareDeviceData((void**)&mLinkOrRootAngularVelAccDev, nullptr, mLinkBufSize * sizeof(PxVec3),
                          "mLinkOrRootAngularVelAccDev");
        prepareDeviceData((void**)&mLinkForcesDev, nullptr, mLinkBufSize * sizeof(PxVec3), "mLinkForcesDev");

        prepareDeviceData((void**)&mLinkTorquesDev, nullptr, mLinkBufSize * sizeof(PxVec3), "mLinkTorquesDev");

        prepareDeviceData((void**)&mLinkIncomingJointForceDev, nullptr, mLinkBufSize * sizeof(PhysxGpuSpatialForces),
                          "mLinkIncomingJointForceDev");
    }

    if (numRds > 0)
    {
        // Create a global index of rigid dynamics from 0 to numRds.
        // Relying on getInternalIslandNodeIndex is not great, because rigid dynamics and articulations get a node
        // index. We need a global index of rigid dynamics so that we can apply GPU data with appropriately sized
        // staging buffers.
        std::vector<PxActor*> rdActors(numRds);
        std::vector<PxRigidDynamicGPUIndex> rdGpuIndices(numRds);
        scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, rdActors.data(), numRds);
        PxU32 maxRdIndex = 0;
        for (PxU32 i = 0; i < numRds; i++)
        {
            PxRigidDynamic* rd = static_cast<PxRigidDynamic*>(rdActors[i]);
            PxNodeIndex nodeIdx = rd->getInternalIslandNodeIndex();
            mNode2RdIndexMap[nodeIdx.index()] = i;
            rdGpuIndices[i] = rd->getGPUIndex();

            PxU32 rdIdx = nodeIdx.index();
            if (rdIdx > maxRdIndex)
            {
                maxRdIndex = rdIdx;
            }
        }

        mMaxRdIndex = maxRdIndex;

        prepareDeviceData((void**)&mRdPoseDev, nullptr, mNumRds * sizeof(PxTransform), "mRdPoseDev");
        prepareDeviceData((void**)&mRdLinearVelAccDev, nullptr, mNumRds * sizeof(PxVec3), "mRdLinearVelAccDev");
        prepareDeviceData((void**)&mRdAngularVelAccDev, nullptr, mNumRds * sizeof(PxVec3), "mRdAngularVelAccDev");
        prepareDeviceData((void**)&mRdForcesDev, nullptr, mNumRds * sizeof(PxVec3), "mRdForcesDev");
        prepareDeviceData((void**)&mRdTorquesDev, nullptr, mNumRds * sizeof(PxVec3), "mRdTorquesDev");
    }
    // fixed tendons
    if (mMaxFixedTendons > 0)
    {
        prepareDeviceData((void**)&mFixedTendonPropertiesDev, nullptr,
                          mFixedTendonBufSize * sizeof(PxGpuFixedTendonData), "mFixedTendonPropertiesDev");
    }
    // spatial tendons
    if (mMaxSpatialTendons > 0)
    {
        prepareDeviceData((void**)&mSpatialTendonPropertiesDev, nullptr,
                          mSpatialTendonBufSize * sizeof(PxGpuSpatialTendonData), "mSpatialTendonPropertiesDev");
    }


    // contacts

    // HMMM, we can't get the GPU dynamics config from PxSceneDesc, so try to get it from USD
    UsdStageRefPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(mStageId));
    if (stage && g_physx)
    {
        size_t sceneObjectId = reinterpret_cast<size_t>(scene->userData);
        SdfPath scenePath = g_physx->getPhysXObjectUsdPath(sceneObjectId);
        if (!scenePath.IsEmpty())
        {
            UsdPrim scenePrim = stage->GetPrimAtPath(scenePath);
            if (scenePrim)
            {
                PhysxSchemaPhysxSceneAPI physxSceneApi(scenePrim);
                if (physxSceneApi)
                {
                    unsigned value = 0;
                    if (physxSceneApi.GetGpuMaxRigidContactCountAttr().Get(&value))
                    {
                        mMaxGpuContactPairs = value;
                    }
                }
            }
        }
    }
    else
    {
        CARB_LOG_WARN("Unable to determine max GPU contact pairs, using default %u", mMaxGpuContactPairs);
    }

    prepareDeviceData((void**)&mGpuContactPairsDev, nullptr, (size_t)mMaxGpuContactPairs * sizeof(PxGpuContactPair),
                      "mGpuContactPairsDev");
    prepareDeviceData((void**)&mGpuContactPairCountDev, nullptr, sizeof(PxU32),
                      "mGpuContactPairCountDev");
    // synchronization events
    for (PxU32 i = 0; i < PxU32(CopyEvent::eCOUNT); i++)
    {
        CHECK_CU(cuEventCreate(&mCopyEvents[i], CU_EVENT_DISABLE_TIMING));
        mCopyEventPointers[i] = mCopyEvents[i] ? &mCopyEvents[i] : nullptr;
    }
    for (PxU32 i = 0; i < PxU32(ApplyEvent::eCOUNT); i++)
    {
        CHECK_CU(cuEventCreate(&mApplyWaitEvents[i], CU_EVENT_DISABLE_TIMING));
        CHECK_CU(cuEventCreate(&mApplySignalEvents[i], CU_EVENT_DISABLE_TIMING));
    }

    CHECK_CU(cuEventCreate(&mContactReadEvent, CU_EVENT_DISABLE_TIMING));

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

GpuSimulationData::~GpuSimulationData()
{
    // printf("+++++ Deleting GpuSimulationData\n");

    CudaContextGuard ctxGuard(mCtx);

    // CHECK_CUDA(cudaFree(allArtiIndicesDev));
    CHECK_CUDA(cudaFree(mNodeIdx2ArtiGpuIdxDev));
    CHECK_CUDA(cudaFree(mGpuContactPairsDev));
    CHECK_CUDA(cudaFree(mGpuContactPairCountDev));

    CHECK_CUDA(cudaFree(mLinkOrRootLinearVelAccDev));
    CHECK_CUDA(cudaFree(mLinkOrRootAngularVelAccDev));
    CHECK_CUDA(cudaFree(mLinkOrRootTransformsDev));
    CHECK_CUDA(cudaFree(mDofScalarsDev));
    CHECK_CUDA(cudaFree(mLinkForcesDev));
    CHECK_CUDA(cudaFree(mLinkTorquesDev));
    CHECK_CUDA(cudaFree(mDofActuationForcesDev));
    CHECK_CUDA(cudaFree(mLinkIncomingJointForceDev));
    CHECK_CUDA(cudaFree(mFixedTendonPropertiesDev));


    CHECK_CUDA(cudaFree(mRdPoseDev));
    CHECK_CUDA(cudaFree(mRdLinearVelAccDev));
    CHECK_CUDA(cudaFree(mRdAngularVelAccDev));
    CHECK_CUDA(cudaFree(mRdForcesDev));
    CHECK_CUDA(cudaFree(mRdTorquesDev));

    CHECK_CUDA(cudaFree(mJacobianDataDev));
    CHECK_CUDA(cudaFree(mMassMatrixDataDev));
    CHECK_CUDA(cudaFree(mCoriolisGravityDataDev));
    CHECK_CUDA(cudaFree(mCentroidalMomentumDataDev));


    // MR: alternatively soft view objects should clean up after themselves, but if we keep it here
    // memory allocated for soft bodies remain valid even after the view is destructed. The idea is some other view may
    // need access to the same body later on, and this avoid memory allocation.
    for (auto  record : mGlobalIndex2SbRecords)
    {
        CHECK_CUDA(cudaFree(record.second.nodalPositions));
        CHECK_CUDA(cudaFree(record.second.elementRestPose));
        CHECK_CUDA(cudaFree(record.second.elementRotation));
        CHECK_CUDA(cudaFree(record.second.elementIndices));
        CHECK_CUDA(cudaFree(record.second.simNodalValues));
        CHECK_CUDA(cudaFree(record.second.simKinematicTargets));
        CHECK_CUDA(cudaFree(record.second.simElementIndices));
        CHECK_CUDA(cudaFree(record.second.simElementRestPose));
        CHECK_CUDA(cudaFree(record.second.simElementRotation));
    }
    mGlobalIndex2SbRecords.clear();

    // same for deformable bodies
    for (auto record : mDeformableBodyToRecord)
    {
        CHECK_CUDA(cudaFree(record.second.simElementIndices));
        CHECK_CUDA(cudaFree(record.second.simNodalKinematicTargets));
        CHECK_CUDA(cudaFree(record.second.collElementIndices));
    }
    mDeformableBodyToRecord.clear();

    // synchronization events
    for (PxU32 i = 0; i < PxU32(CopyEvent::eCOUNT); i++)
    {
        if (mCopyEvents[i])
        {
            CHECK_CU(cuEventDestroy(mCopyEvents[i]));
        }
    }
    for (PxU32 i = 0; i < PxU32(ApplyEvent::eCOUNT); i++)
    {
        if (mApplyWaitEvents[i])
        {
            CHECK_CU(cuEventDestroy(mApplyWaitEvents[i]));
        }
        if (mApplySignalEvents[i])
        {
            CHECK_CU(cuEventDestroy(mApplySignalEvents[i]));
        }
    }
    if (mContactReadEvent)
    {
        CHECK_CU(cuEventDestroy(mContactReadEvent));
    }
}

bool GpuSimulationData::checkApiReady(const char* function) const
{
    //printf("~!~!~! Calling %s at timestamp %d, step %d\n", function, int(mBackend.getTimestamp()), int(mBackend.getStepCount()));
    if (mBackend.getStepCount() < 1)
    {
        CARB_LOG_ERROR("GPU tensor function %s can only be called after at least one simulation step was performed!", function);
        return false;
    }
    return true;
}

void GpuSimulationData::clearForces()
{
    if (mRdForcesApplied)
    {
        // printf("CLEARING RD FORCES\n");
        // clear forces in staging buffer
        CHECK_CUDA(cudaMemset(mRdForcesDev, 0, mNumRds * sizeof(PxVec3)));
        // make sure the cleared forces get applied on the next flush
        mRdForcesApplied = false;
    }

    if (mRdTorquesApplied)
    {
        // printf("CLEARING RD TORQUE\n");
        CHECK_CUDA(cudaMemset(mRdTorquesDev, 0, mNumRds * sizeof(PxVec3)));
        mRdTorquesApplied = false;
    }

    if (mLinkForcesApplied)
    {
        // printf("CLEARING LINK FORCES\n");
        // clear forces in staging buffer
        CHECK_CUDA(cudaMemset(mLinkForcesDev, 0, mLinkBufSize * sizeof(PxVec3)));
        mLinkForcesApplied = false;
    }

    if (mLinkTorquesApplied)
    {
        // printf("CLEARING LINK TORQUES\n");
        CHECK_CUDA(cudaMemset(mLinkTorquesDev, 0, mLinkBufSize * sizeof(PxVec3)));
        mLinkTorquesApplied = false;
    }

    if (mArtiDofForcesApplied)
    {
        // printf("CLEARING ARTICULATION DOF FORCES\n");
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
    int64_t timestamp = mBackend.getTimestamp();
    if (mRigidContactTimestamp >= timestamp)
    {
        return;
    }
    mRigidContactTimestamp = timestamp;

    PhysxCudaContextGuard ctxGuarg(mCudaContextManager);

    mScene->getDirectGPUAPI().copyContactData(mGpuContactPairsDev, mGpuContactPairCountDev, mMaxGpuContactPairs, NULL, mContactReadEvent);
    CHECK_CU(cuStreamWaitEvent(nullptr, mContactReadEvent, 0));

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
        // *** TODO:
        // CARB_LOG_WARN("Increase max contact pairs in sim params.");
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


void SoftBodyBufferManager::allocDevMemAndCopyH2D(void** dst, const void* src, size_t size, const char* name)
{
    if (!CHECK_CUDA(cudaMalloc(dst, size)))
    {
        CARB_LOG_ERROR("Unable to allocate memory of size %u for %s", (::physx::PxU32)size, name);
        return;
    }
    if (!CHECK_CUDA(cudaMemcpy(*dst, src, size, cudaMemcpyHostToDevice)))
    {
        CARB_LOG_ERROR("Unable to copy data from host to device for %s", name);
        return;
    }
}

void SoftBodyBufferManager::resizeDeviceData(::physx::PxU32 previousSize,
                                             ::physx::PxU32 currentSize,
                                             ::physx::PxU32 elementSize,
                                             void* previousBufferDev)
{
    ::physx::PxU32 numNewElements = currentSize - previousSize;
    ::physx::PxU32* temp = nullptr;
    if (!CHECK_CUDA(cudaMalloc(&temp, currentSize * elementSize)))
    {
        return;
    }
    if (previousBufferDev)
    {
        if (!CHECK_CUDA(cudaMemcpy(temp, previousBufferDev, previousSize * elementSize, cudaMemcpyDeviceToDevice)))
        {
            return;
        }
        SYNCHRONIZE_CUDA(); // D2D needs a sync
    }
    if (!CHECK_CUDA(cudaMemset(temp + previousSize, 0, numNewElements * elementSize)))
    {
        return;
    }
    // free old memory + set the pointer to new buffer
    if (previousBufferDev)
        CHECK_CUDA(cudaFree(previousBufferDev));
    previousBufferDev = temp;
}

void SoftBodyBufferManager::resetDeviceMem()
{
    for (size_t i = 0; i < buffersH.size(); i++)
    {
        if (!CHECK_CUDA(cudaMemset(buffersH[i], 0, bufferSizesH[i])))
        {
            return;
        }
    }
}

void SoftBodyBufferManager::setMaxBufferSize()
{
    maxBufferSize = *std::max_element(bufferSizesH.begin(), bufferSizesH.end());
}

void SoftBodyBufferManager::uploadDeviceData(std::string debugMessage)
{
    if (buffersD)
        CHECK_CUDA(cudaFree(buffersD));
    allocDevMemAndCopyH2D(
        (void**)&buffersD, buffersH.data(), sizeof(void*) * buffersH.size(), (debugMessage + " buffer").c_str());

    if (bufferSizesD)
        CHECK_CUDA(cudaFree(bufferSizesD));
    allocDevMemAndCopyH2D((void**)&bufferSizesD, bufferSizesH.data(), sizeof(::physx::PxU32) * bufferSizesH.size(),
                          (debugMessage + " sizes").c_str());
}

void SoftBodyBufferManager::releaseDeviceMem()
{
    CHECK_CUDA(cudaFree(buffersD));
    CHECK_CUDA(cudaFree(bufferSizesD));
}

}
}
}

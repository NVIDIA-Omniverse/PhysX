// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include <string>

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include <PxPhysicsAPI.h>
#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

#include "GpuSoftBodyView.h"
#include "GpuSimulationView.h"
#include "CudaKernels.h"


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

GpuSoftBodyView::GpuSoftBodyView(GpuSimulationView* sim, const std::vector<SoftBodyEntry>& entries, int device)
    : BaseSoftBodyView(sim, entries), mDevice(device), mSbIndicesD(0)
{
    // NOTE: we assume that the simulation, collision mesh indices, and rest poses are constant throughout the simulation to avoid multiple data copy.
    if (sim)
        mGpuSimData = sim->getGpuSimulationData();

    uint32_t numBodies = getCount();
    PxU32 maxElements = 0;
    PxU32 maxVertices = 0;
    PxU32 maxSimVertices = 0;
    PxU32 maxSimElements = 0;
    mViewIndices.resize(numBodies);
    mSbIndices.resize(numBodies);

    // helper data for GpuSimData update if needed.
    PxU32 initNumGpuSb = mGpuSimData->mNumSoftBodies;
    PxU32 numSoftBodyCounter = initNumGpuSb;

    for (PxU32 i = 0; i < numBodies; i++)
    {
        PxSoftBody* softBody = entries[i].body;
        PxU32 softBodyIdx = softBody->getGpuSoftBodyIndex();
        mSbIndices[i] = softBodyIdx;
        mViewIndices[i] = i;
        GpuSoftBodyRecord sbRecord;

        if (mEntries[i].subspace)
        {
            const carb::Float3& origin = mEntries[i].subspace->origin;
            sbRecord.origin = { origin.x, origin.y, origin.z };
        }
        else
        {
            sbRecord.origin = { 0.0f, 0.0f, 0.0f };
        }
        PxTetrahedronMesh* collisionTetMesh = softBody->getCollisionMesh();
        PxTetrahedronMesh* SimulationTetMesh = softBody->getSimulationMesh();
        sbRecord.numVertices = collisionTetMesh->getNbVertices();
        sbRecord.numElements = collisionTetMesh->getNbTetrahedrons();
        sbRecord.numSimVertices = SimulationTetMesh->getNbVertices();
        sbRecord.numSimElements = SimulationTetMesh->getNbTetrahedrons();
        sbRecord.globalIndex = softBody->getGpuSoftBodyIndex();

        CARB_ASSERT(sbRecord.numSimElements * 4 == entries[i].simIndices.size());
        CARB_ASSERT(sbRecord.numSimElements == entries[i].simRestPoses.size());
        softBody->setSoftBodyFlag(PxSoftBodyFlag::ePARTIALLY_KINEMATIC, true);

        const PxU32 simNodalValuesBufferSize = sbRecord.numSimVertices * sizeof(PxVec4);
        const PxU32 simElementIndicesBufferSize = sbRecord.numSimElements * sizeof(PxU32) * 4; // for tetrahedron
        const PxU32 elementIndicesBufferSize = sbRecord.numElements * sizeof(PxU32) * 4; // for tetrahedron
        const PxU32 nodalPositionBufferSize = sbRecord.numVertices * sizeof(PxVec4);
        const PxU32 elementRestPoseBufferSize = sbRecord.numElements * sizeof(PxMat33);
        const PxU32 elementRotationBufferSize = sbRecord.numElements * sizeof(PxQuat);
        const PxU32 simElementRotationBufferSize = sbRecord.numSimElements * sizeof(PxQuat);
        const PxU32 simOrderedElementIndicesBufferSize = sbRecord.numSimElements * sizeof(PxU32);
        const PxU32 SimElementRestPoseBufferSize = sbRecord.numSimElements * sizeof(PxMat33);

        // check for existing record
        // update the records and buffers only if no other views have done so yet
        auto it = mGpuSimData->mGlobalIndex2SbRecords.find(softBodyIdx);
        if (it == mGpuSimData->mGlobalIndex2SbRecords.end())
        {
            // std::cout << "GpuSoftBodyView :: allocating memory for softBody : " << softBodyIdx << std::endl;
            if (!CHECK_CUDA(cudaMalloc(&(sbRecord.simKinematicTargets), simNodalValuesBufferSize)))
                return;
            if (!CHECK_CUDA(cudaMalloc(&(sbRecord.simNodalValues), simNodalValuesBufferSize)))
                return;
            if (!CHECK_CUDA(cudaMalloc(&(sbRecord.elementIndices), elementIndicesBufferSize)))
                return;
            if (!CHECK_CUDA(cudaMalloc(&(sbRecord.nodalPositions), nodalPositionBufferSize)))
                return;
            if (!CHECK_CUDA(cudaMalloc(&(sbRecord.elementRotation), elementRotationBufferSize)))
                return;
            if (!CHECK_CUDA(cudaMalloc(&(sbRecord.simElementRotation), simElementRotationBufferSize)))
                return;

            if (!CHECK_CUDA(cudaMalloc(&(sbRecord.elementRestPose), elementRestPoseBufferSize)))
                return;
            //
            if (!CHECK_CUDA(cudaMalloc(&(sbRecord.simElementRestPose), SimElementRestPoseBufferSize)))
                return;

            if (!CHECK_CUDA(cudaMemcpy(sbRecord.simElementRestPose, entries[i].simRestPoses.data(),
                                       SimElementRestPoseBufferSize, cudaMemcpyHostToDevice)))
            {
                CARB_LOG_ERROR("Unable to copy data from host to device for SimElementRestPoseBuffer\n");
                return;
            }
            if (!CHECK_CUDA(cudaMalloc(&(sbRecord.simElementIndices), simElementIndicesBufferSize)))
                return;
            if (!CHECK_CUDA(cudaMemcpy(sbRecord.simElementIndices, entries[i].simIndices.data(),
                                       simElementIndicesBufferSize, cudaMemcpyHostToDevice)))
            {
                CARB_LOG_ERROR("Unable to copy data from host to device for simElementIndicesBuffer\n");
                return;
            }
            // std::cout << "GpuSoftBodyView :: softBodyIdx " << softBodyIdx
            //           << " usage count : " << mGpuSimData->mGlobalIndexRefCount[softBodyIdx] << std::endl;
            mGpuSimData->mGlobalIndex2SbRecords[softBodyIdx] = sbRecord;
            numSoftBodyCounter++;
        }
        else
        {
            sbRecord = mGpuSimData->mGlobalIndex2SbRecords[softBodyIdx];
        }

        mSimKinematicTargets.bufferSizesH.push_back(simNodalValuesBufferSize);
        mSimKinematicTargets.buffersH.push_back(sbRecord.simKinematicTargets);
        mSimNodalValues.bufferSizesH.push_back(simNodalValuesBufferSize);
        mSimNodalValues.buffersH.push_back(sbRecord.simNodalValues);
        mSimElementIndices.bufferSizesH.push_back(simElementIndicesBufferSize);
        mSimElementIndices.buffersH.push_back(sbRecord.simElementIndices);
        mSimElementRotations.bufferSizesH.push_back(simElementRotationBufferSize);
        mSimElementRotations.buffersH.push_back(sbRecord.simElementRotation);

        mNodalPositions.bufferSizesH.push_back(nodalPositionBufferSize);
        mNodalPositions.buffersH.push_back(sbRecord.nodalPositions);
        mElementIndices.bufferSizesH.push_back(elementIndicesBufferSize);
        mElementIndices.buffersH.push_back(sbRecord.elementIndices);
        mElementRestPoses.bufferSizesH.push_back(elementRestPoseBufferSize);
        mElementRestPoses.buffersH.push_back(sbRecord.elementRestPose);
        mElementRotations.bufferSizesH.push_back(elementRotationBufferSize);
        mElementRotations.buffersH.push_back(sbRecord.elementRotation);

        // increase the usage count of the buffer managing the softBodyIdx
        if (mGpuSimData->mGlobalIndexRefCount.find(softBodyIdx) == mGpuSimData->mGlobalIndexRefCount.end())
            mGpuSimData->mGlobalIndexRefCount[softBodyIdx] = 1;
        else
            mGpuSimData->mGlobalIndexRefCount[softBodyIdx]++;

        sbRecord.softBody = softBody;

        mSbRecords.push_back(sbRecord);
        maxElements = PxMax(maxElements, sbRecord.numElements);
        maxVertices = PxMax(maxVertices, sbRecord.numVertices);
        maxSimVertices = PxMax(maxSimVertices, sbRecord.numSimVertices);
        maxSimElements = PxMax(maxSimElements, sbRecord.numSimElements);
    }

    mMaxElementsPerBody = maxElements;
    mMaxVerticesPerBody = maxVertices;
    mMaxSimElementsPerBody = maxSimElements;
    mMaxSimVerticesPerBody = maxSimVertices;

    mSimElementIndices.setMaxBufferSize();
    mSimKinematicTargets.setMaxBufferSize();
    mSimNodalValues.setMaxBufferSize();
    mSimElementRotations.setMaxBufferSize();
    mNodalPositions.setMaxBufferSize();
    mElementRestPoses.setMaxBufferSize();
    mElementRotations.setMaxBufferSize();
    mElementIndices.setMaxBufferSize();
    mSimKinematicTargets.uploadDeviceData("mSimKinematicTargets");
    mSimNodalValues.uploadDeviceData("mSimNodalValues");
    mSimElementIndices.uploadDeviceData("mSimElementIndices");
    mElementIndices.uploadDeviceData("mElementIndices");
    mSimElementRotations.uploadDeviceData("mSimElementRotations");
    mElementRotations.uploadDeviceData("mElementRotations");
    mNodalPositions.uploadDeviceData("mNodalPositions");
    mElementRestPoses.uploadDeviceData("mElementRestPoses");


    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    // soft body indices
    SoftBodyBufferManager::allocDevMemAndCopyH2D(
        (void**)&mSbIndicesD, mSbIndices.data(), sizeof(PxU32) * numBodies, "SoftBodyIndices");
    // view indices
    SoftBodyBufferManager::allocDevMemAndCopyH2D(
        (void**)&mViewIndicesD, mViewIndices.data(), sizeof(PxU32) * numBodies, "ViewIndices");
    // soft body records
    SoftBodyBufferManager::allocDevMemAndCopyH2D(
        (void**)&mSbRecordsDev, mSbRecords.data(), sizeof(GpuSoftBodyRecord) * numBodies, "GpuSoftBodyRecord");

    if (!CHECK_CUDA(cudaMalloc(&materialPropertiesDev, sizeof(PxVec4) * numBodies)))
    {
        CARB_LOG_ERROR("Unable to allocate memory for  material properties");
        return;
    }
    if (!CHECK_CUDA(cudaMalloc(&materialModelsDev, sizeof(PxFEMSoftBodyMaterialModel) * numBodies)))
    {
        CARB_LOG_ERROR("Unable to allocate memory for material model");
        return;
    }

    // cache the indices and rest poses
    copySoftBodyData(PxSoftBodyGpuDataFlag::eTET_REST_POSES, CopyEvent::eSbElementRestPoseBuffer,
                     (void**)mElementRestPoses.buffersD, mElementRestPoses.bufferSizesD, mSbIndicesD, (PxU32)getCount(),
                     mElementRestPoses.maxBufferSize);
    copySoftBodyData(PxSoftBodyGpuDataFlag::eTET_INDICES, CopyEvent::eSbElementIndicesBuffer,
                     (void**)mElementIndices.buffersD, mElementIndices.bufferSizesD, mSbIndicesD, (PxU32)getCount(),
                     mElementIndices.maxBufferSize);
    // Also fetch the inital masses with positions, in case a submit command is issued
    copySoftBodyData(PxSoftBodyGpuDataFlag::eSIM_POSITION_INV_MASS, SoftBodyData::eSimNodalPosition,
                     (void**)mSimNodalValues.buffersD, mSimNodalValues.bufferSizesD, mSbIndicesD, (PxU32)getCount(),
                     mSimNodalValues.maxBufferSize);
}

GpuSoftBodyView::~GpuSoftBodyView()
{
    if (mGpuSimData)
    {
        CudaContextGuard ctxGuard(mGpuSimData->mCtx);

        CHECK_CUDA(cudaFree(mSbIndicesD));
        CHECK_CUDA(cudaFree(mViewIndicesD));
        CHECK_CUDA(cudaFree(mSbRecordsDev));
        CHECK_CUDA(cudaFree(materialModelsDev));
        CHECK_CUDA(cudaFree(materialPropertiesDev));

        mSimKinematicTargets.releaseDeviceMem();
        mSimNodalValues.releaseDeviceMem();
        mSimElementIndices.releaseDeviceMem();
        mSimElementRotations.releaseDeviceMem();
        mElementRestPoses.releaseDeviceMem();
        mElementRotations.releaseDeviceMem();
        mElementIndices.releaseDeviceMem();
        mNodalPositions.releaseDeviceMem();
        // NOTE: should we remove the device buffer allocated for a specific soft body as soon as they are not managed
        // by any views or should we keep them around maybe some new view are going to manage them?
        // former is more memory efficient, but later is more performant if the users keep creating view objects
    }
}

void GpuSoftBodyView::copySoftBodyData(PxSoftBodyGpuDataFlag::Enum flag,
                                       PxU32 copyEvent,
                                       void** mBuffersD,
                                       PxU32* mBufferSizesD,
                                       PxU32* mSbIndicesD,
                                       PxU32 size,
                                       PxU32 mMaxBufferSize) const
{
    PxScene* scene = mGpuSimData->mScene;
    CUevent signalEvent = mGpuSimData->mCopyEvents[copyEvent];
    CHECK_CU(cuStreamWaitEvent(nullptr, signalEvent, 0));
    scene->copySoftBodyData(
        mBuffersD, (void*)mBufferSizesD, (void*)mSbIndicesD, flag, size, mMaxBufferSize, signalEvent);
    CHECK_CU(cuStreamWaitEvent(nullptr, signalEvent, 0));
}

bool GpuSoftBodyView::submitPxValues(const TensorDesc* srcTensor,
                                     const TensorDesc* indexTensor,
                                     PxSoftBodyGpuDataFlag::Enum dataType,
                                     const char* callingFunction,
                                     const char* tensorName,
                                     bool transform,
                                     bool kinematicTarget)
{
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }
    PxU32 expectedSize = mMaxSimVerticesPerBody * getCount() * (kinematicTarget ? 4u : 3u);

    if (!checkTensorDevice(*srcTensor, mDevice, tensorName, callingFunction) ||
        !checkTensorFloat32(*srcTensor, tensorName, callingFunction) ||
        !checkTensorSizeExact(*srcTensor,expectedSize, tensorName, callingFunction))
    {
        return false;
    }
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    const PxU32* viewIndicesD = nullptr;
    PxU32* SbIndicesD = nullptr;
    SoftBodyBufferManager tmpNodalBuffer; // needed in case of re-batching
    bool requireCleaning = false;
    PxU32 numIndices = 0;
    std::vector<PxU32> viewIndicesH;

    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, mDevice, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        viewIndicesD = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));

        // need to re-batch the soft body pointers to only apply data to the ones required modification
        std::vector<PxU32> SbIndicesH(numIndices);
        viewIndicesH.resize(numIndices);
        if (!CHECK_CUDA(cudaMemcpy(viewIndicesH.data(), viewIndicesD, numIndices * sizeof(PxU32), cudaMemcpyDeviceToHost)))
        {
            CARB_LOG_ERROR("Unable to copy data from device to host for viewIndicesD to viewIndicesH\n");
            return false;
        }
        for (PxU32 i = 0; i < numIndices; i++)
        {
            PxU32 viewIdx = viewIndicesH[i];
            SbIndicesH[i] = mSbIndices[viewIdx];
            const PxU32 bufferSize = mSbRecords[viewIdx].numSimVertices * sizeof(PxVec4);
            tmpNodalBuffer.bufferSizesH.push_back(bufferSize);
            tmpNodalBuffer.buffersH.push_back(mSbRecords[viewIdx].simNodalValues);
        }
        tmpNodalBuffer.setMaxBufferSize();
        tmpNodalBuffer.uploadDeviceData("mSimNodalBuffer");
        SoftBodyBufferManager::allocDevMemAndCopyH2D(
            (void**)&SbIndicesD, SbIndicesH.data(), sizeof(PxU32) * SbIndicesH.size(), "SbIndicesD");
        requireCleaning = true;
    }
    else
    {
        tmpNodalBuffer = mSimNodalValues;
        viewIndicesD = mViewIndicesD;
        viewIndicesH = mViewIndices;
        SbIndicesD = mSbIndicesD;
        numIndices = getCount();
    }


    if (!kinematicTarget)
    {
        if (!submitSbNodalValues(static_cast<PxVec3*>(srcTensor->data), viewIndicesD, mSbRecordsDev, numIndices,
                                 mMaxSimVerticesPerBody, transform))
        {
            CARB_LOG_ERROR("Failed to submit soft body nodal values");
            return false;
        }
        CUevent waitEvent = mGpuSimData->mApplyWaitEvents[ApplyEvent::eSbSimNodalValueBuffer];
        CHECK_CU(cuEventRecord(waitEvent, nullptr));
        CHECK_CU(cuEventSynchronize(waitEvent));
        PxScene* scene = mGpuSimData->mScene;
        SYNCHRONIZE_CUDA();
        // cuCtxSynchronize(); // TODO: (why) is this necessary?
        scene->applySoftBodyData((void**)tmpNodalBuffer.buffersD, (void*)tmpNodalBuffer.bufferSizesD, (void*)SbIndicesD,
                                 dataType, numIndices, tmpNodalBuffer.maxBufferSize, waitEvent);
        CHECK_CUDA(cudaStreamSynchronize(nullptr));
        if (requireCleaning)
        {
            CHECK_CUDA(cudaFree(SbIndicesD));
            tmpNodalBuffer.releaseDeviceMem();
        }
    }
    else
    {
        if (!submitSbKinematicTargets(static_cast<PxVec4*>(srcTensor->data), viewIndicesD, mSbRecordsDev, numIndices,
                                      mMaxSimVerticesPerBody, transform))
        {
            CARB_LOG_ERROR("Failed to submit kinematic targets");
            return false;
        }
        PxScene* scene = mGpuSimData->mScene;
        CHECK_CUDA(cudaStreamSynchronize(nullptr));
        SYNCHRONIZE_CUDA();

        for (size_t i = 0; i < numIndices; i++)
        {
            PxU32 viewIdx = viewIndicesH[i];
            GpuSoftBodyRecord& record = mSbRecords[viewIdx];
            record.softBody->setKinematicTargetBufferD(record.simKinematicTargets, PxSoftBodyFlag::ePARTIALLY_KINEMATIC);
        }
    }

    return true;
}

bool GpuSoftBodyView::fetchPxValues(const TensorDesc* dstTensor,
                                    PxSoftBodyGpuDataFlag::Enum flag,
                                    SoftBodyData::Enum dataType,
                                    PxU32 copyEvent,
                                    const char* tensorName,
                                    PxU32 expectedSize,
                                    void** mBuffersD,
                                    PxU32* mBufferSizesD,
                                    PxU32* mSbIndicesD,
                                    PxU32 numSb,
                                    PxU32 mMaxBufferSize,
                                    PxU32 mMaxElementsOrVerticesPerBody,
                                    const char* callingFunction,
                                    checkTensorType callBackTensorCheck) const
{
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }
    if (!checkTensorDevice(*dstTensor, mDevice, tensorName, callingFunction) ||
        !callBackTensorCheck(*dstTensor, tensorName, callingFunction) ||
        !checkTensorSizeExact(*dstTensor, expectedSize, tensorName, callingFunction))
    {
        return false;
    }

    if (dataType != SoftBodyData::eSimKinematicTarget)
        copySoftBodyData(flag, copyEvent, mBuffersD, mBufferSizesD, mSbIndicesD, numSb, mMaxBufferSize);

    bool result = false;
    switch (dataType)
    {
    case SoftBodyData::eElementRotation:
        result = fetchSbElementRotations(
            static_cast<PxQuat*>(dstTensor->data), mSbRecordsDev, numSb, mMaxElementsOrVerticesPerBody, false);
        break;

    case SoftBodyData::eSimElementRotation:
        result = fetchSbElementRotations(
            static_cast<PxQuat*>(dstTensor->data), mSbRecordsDev, numSb, mMaxElementsOrVerticesPerBody, true);
        break;

    case SoftBodyData::eElementRestPose:
        result = fetchSbElementRestPoses(
            static_cast<PxMat33*>(dstTensor->data), mSbRecordsDev, numSb, mMaxElementsOrVerticesPerBody, false);
        break;

    case SoftBodyData::eElementIndices:
        result = fetchSbElementIndices(
            static_cast<PxU32*>(dstTensor->data), mSbRecordsDev, numSb, mMaxElementsOrVerticesPerBody, false);
        break;

    case SoftBodyData::eSimElementIndices:
        result = fetchSbElementIndices(
            static_cast<PxU32*>(dstTensor->data), mSbRecordsDev, numSb, mMaxElementsOrVerticesPerBody, true);
        break;

    case SoftBodyData::eNodalPosition:
        result = fetchSbNodalValues(
            static_cast<PxVec3*>(dstTensor->data), mSbRecordsDev, numSb, mMaxElementsOrVerticesPerBody, false, true);
        break;

    case SoftBodyData::eSimNodalPosition:
        result = fetchSbNodalValues(
            static_cast<PxVec3*>(dstTensor->data), mSbRecordsDev, numSb, mMaxElementsOrVerticesPerBody, true, true);
        break;

    case SoftBodyData::eSimNodalVelocity:
        result = fetchSbNodalValues(
            static_cast<PxVec3*>(dstTensor->data), mSbRecordsDev, numSb, mMaxElementsOrVerticesPerBody, true, false);
        break;

    case SoftBodyData::eSimKinematicTarget:
        result = fetchSbKinematicTargets(
            static_cast<PxVec4*>(dstTensor->data), mSbRecordsDev, numSb, mMaxElementsOrVerticesPerBody, true);
        break;

    default:
        CARB_LOG_ERROR("Invalid SoftBodyData encountered.");
        break;
    }

    if (!result)
    {
        CARB_LOG_ERROR("Failed to fetch soft body %s", tensorName);
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    SYNCHRONIZE_CUDA();
    return true;
};

bool GpuSoftBodyView::setSimNodalPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return submitPxValues(
        srcTensor, indexTensor, PxSoftBodyGpuDataFlag::eSIM_POSITION_INV_MASS, "nodalPositions", __FUNCTION__, true, false);
}

bool GpuSoftBodyView::setSimNodalVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return submitPxValues(
        srcTensor, indexTensor, PxSoftBodyGpuDataFlag::eSIM_VELOCITY_INV_MASS, "nodalVelocities", __FUNCTION__, false, false);
}

bool GpuSoftBodyView::setSimKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return submitPxValues(srcTensor, indexTensor, PxSoftBodyGpuDataFlag::eSIM_POSITION_INV_MASS, "kinematicTargets", __FUNCTION__, true, true);
}

bool GpuSoftBodyView::getSimNodalPositions(const TensorDesc* dstTensor) const
{
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    return fetchPxValues(dstTensor, PxSoftBodyGpuDataFlag::eSIM_POSITION_INV_MASS, SoftBodyData::eSimNodalPosition,
                         CopyEvent::eSbSimNodalValueBuffer, "nodalPositions", mMaxSimVerticesPerBody * getCount() * 3u,
                         (void**)mSimNodalValues.buffersD, mSimNodalValues.bufferSizesD, mSbIndicesD, (PxU32)getCount(),
                         mSimNodalValues.maxBufferSize, mMaxSimVerticesPerBody, __FUNCTION__);
}

bool GpuSoftBodyView::getSimNodalVelocities(const TensorDesc* dstTensor) const
{
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    return fetchPxValues(dstTensor, PxSoftBodyGpuDataFlag::eSIM_VELOCITY_INV_MASS, SoftBodyData::eSimNodalVelocity,
                         CopyEvent::eSbSimNodalValueBuffer, "NodalVelocities", mMaxSimVerticesPerBody * getCount() * 3u,
                         (void**)mSimNodalValues.buffersD, mSimNodalValues.bufferSizesD, mSbIndicesD, (PxU32)getCount(),
                         mSimNodalValues.maxBufferSize, mMaxSimVerticesPerBody, __FUNCTION__);
}

bool GpuSoftBodyView::getSimKinematicTargets(const TensorDesc* dstTensor) const
{
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    return fetchPxValues(dstTensor, PxSoftBodyGpuDataFlag::eSIM_POSITION_INV_MASS, SoftBodyData::eSimKinematicTarget,
                         CopyEvent::eSbSimNodalValueBuffer, "KinematicTargets", mMaxSimVerticesPerBody * getCount() * 4u,
                         (void**)mSimKinematicTargets.buffersD, mSimKinematicTargets.bufferSizesD, mSbIndicesD,
                         (PxU32)getCount(), mSimKinematicTargets.maxBufferSize, mMaxSimVerticesPerBody, __FUNCTION__);
}

bool GpuSoftBodyView::getNodalPositions(const TensorDesc* dstTensor) const
{
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    return fetchPxValues(dstTensor, PxSoftBodyGpuDataFlag::eTET_POSITION_INV_MASS, SoftBodyData::eNodalPosition,
                         CopyEvent::eSbNodalPositionBuffer, "nodalPosition", mMaxVerticesPerBody * getCount() * 3u,
                         (void**)mNodalPositions.buffersD, mNodalPositions.bufferSizesD, mSbIndicesD, (PxU32)getCount(),
                         mNodalPositions.maxBufferSize, mMaxVerticesPerBody, __FUNCTION__);
}

bool GpuSoftBodyView::getElementRestPoses(const TensorDesc* dstTensor) const
{
    // PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    // return fetchPxValues(dstTensor, PxSoftBodyGpuDataFlag::eTET_REST_POSES, SoftBodyData::eElementRestPose,
    //                      CopyEvent::eSbElementRestPoseBuffer, "mElementRestPoses", mMaxElementsPerBody * getCount() * 9u,
    //                      (void**)mElementRestPoses.buffersD, mElementRestPoses.bufferSizesD, mSbIndicesD, (PxU32)getCount(),
    //                      mElementRestPoses.maxBufferSize, mMaxElementsPerBody, __FUNCTION__);

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }
    if (!checkTensorDevice(*dstTensor, mDevice, "elementRestPoses", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "elementRestPoses", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, mMaxElementsPerBody * getCount() * 9u, "elementRestPoses", __FUNCTION__))
    {
        return false;
    }

    if (!fetchSbElementRestPoses(
            static_cast<PxMat33*>(dstTensor->data), mSbRecordsDev, (PxU32)getCount(), mMaxElementsPerBody, false))
    {
        CARB_LOG_ERROR("Failed to fetch soft body simElementRestPose");
        return false;
    }
    return true;
}

bool GpuSoftBodyView::getSimElementRestPoses(const TensorDesc* dstTensor) const
{
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }
    if (!checkTensorDevice(*dstTensor, mDevice, "simElementRestPose", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "simElementRestPose", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, mMaxSimElementsPerBody * getCount() * 9u, "simElementRestPose", __FUNCTION__))
    {
        return false;
    }

    if (!fetchSbElementRestPoses(
            static_cast<PxMat33*>(dstTensor->data), mSbRecordsDev, (PxU32)getCount(), mMaxSimElementsPerBody, true))
    {
        CARB_LOG_ERROR("Failed to fetch soft body simElementRestPose");
        return false;
    }
    return true;
}

bool GpuSoftBodyView::getElementRotations(const TensorDesc* dstTensor) const
{
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    return fetchPxValues(dstTensor, PxSoftBodyGpuDataFlag::eTET_ROTATIONS, SoftBodyData::eElementRotation,
                         CopyEvent::eSbElementRotationBuffer, "elementRotations", mMaxElementsPerBody * getCount() * 4u,
                         (void**)mElementRotations.buffersD, mElementRotations.bufferSizesD, mSbIndicesD,
                         (PxU32)getCount(), mElementRotations.maxBufferSize, mMaxElementsPerBody, __FUNCTION__);
}

bool GpuSoftBodyView::getSimElementRotations(const TensorDesc* dstTensor) const
{
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    return fetchPxValues(dstTensor, PxSoftBodyGpuDataFlag::eSIM_TET_ROTATIONS, SoftBodyData::eSimElementRotation,
                         CopyEvent::eSbSimElementRotationBuffer, "simElementRotations", mMaxSimElementsPerBody * getCount() * 4u,
                         (void**)mSimElementRotations.buffersD, mSimElementRotations.bufferSizesD, mSbIndicesD,
                         (PxU32)getCount(), mSimElementRotations.maxBufferSize, mMaxSimElementsPerBody, __FUNCTION__);
}

bool GpuSoftBodyView::getElementIndices(const TensorDesc* dstTensor) const
{    

    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }
    if (!checkTensorDevice(*dstTensor, mDevice, "elementIndices", __FUNCTION__) ||
        !checkTensorInt32(*dstTensor, "elementIndices", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, mMaxElementsPerBody * getCount() * 4u, "elementIndices", __FUNCTION__))
    {
        return false;
    }
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    if (!fetchSbElementIndices(
            static_cast<PxU32*>(dstTensor->data), mSbRecordsDev, (PxU32)getCount(), mMaxElementsPerBody, false))
    {
        CARB_LOG_ERROR("Failed to fetch soft body deformation gradient");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
    // No need to copy returns the cache indices
    // PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    // return fetchPxValues(dstTensor, PxSoftBodyGpuDataFlag::eTET_INDICES, SoftBodyData::eElementIndices,
    //                      CopyEvent::eSbElementIndicesBuffer, "elementIndices", mMaxElementsPerBody * getCount() * 4u,
    //                      (void**)mElementIndices.buffersD, mElementIndices.bufferSizesD, mSbIndicesD, (PxU32)getCount(),
    //                      mElementIndices.maxBufferSize, mMaxElementsPerBody, __FUNCTION__, checkTensorInt32);
}

bool GpuSoftBodyView::getSimElementIndices(const TensorDesc* dstTensor) const
{
    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }
    if (!checkTensorDevice(*dstTensor, mDevice, "simElementIndices", __FUNCTION__) ||
        !checkTensorInt32(*dstTensor, "simElementIndices", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, mMaxSimElementsPerBody * getCount() * 4u, "simElementIndices", __FUNCTION__))
    {
        return false;
    }
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    if (!fetchSbElementIndices(
            static_cast<PxU32*>(dstTensor->data), mSbRecordsDev, (PxU32)getCount(), mMaxSimElementsPerBody, true))
    {
        CARB_LOG_ERROR("Failed to fetch soft body deformation gradient");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
    // No need to copy returns the cache indices
    // return fetchPxValues(dstTensor, PxSoftBodyGpuDataFlag::eSIM_TET_INDICES, SoftBodyData::eSimElementIndices,
    //                      CopyEvent::eSbSimElementIndicesBuffer, "simElementIndices",
    //                      mMaxSimElementsPerBody * getCount() * 4u, (void**)mSimElementIndices.buffersD,
    //                      mSimElementIndices.bufferSizesD, mSbIndicesD, (PxU32)getCount(),
    //                      mSimElementIndices.maxBufferSize, mMaxSimElementsPerBody, __FUNCTION__, checkTensorInt32);
}

bool GpuSoftBodyView::getElementDeformationGradients(const TensorDesc* dstTensor) const
{
    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }
    if (!checkTensorDevice(*dstTensor, mDevice, "deformationGradient", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "deformationGradient", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, mMaxElementsPerBody * getCount() * 9u, "deformationGradient", __FUNCTION__))
    {
        return false;
    }
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    copySoftBodyData(PxSoftBodyGpuDataFlag::eTET_POSITION_INV_MASS, CopyEvent::eSbNodalPositionBuffer,
                     (void**)mNodalPositions.buffersD, mNodalPositions.bufferSizesD, mSbIndicesD, (PxU32)getCount(),
                     mNodalPositions.maxBufferSize);
    copySoftBodyData(PxSoftBodyGpuDataFlag::eTET_ROTATIONS, CopyEvent::eSbElementRotationBuffer,
                     (void**)mElementRotations.buffersD, mElementRotations.bufferSizesD, mSbIndicesD,
                     (PxU32)getCount(), mElementRotations.maxBufferSize);

    if (!computeSbElementDeformationGradients(
            static_cast<PxMat33*>(dstTensor->data), mSbRecordsDev, (PxU32)getCount(), mMaxElementsPerBody, false))
    {
        CARB_LOG_ERROR("Failed to fetch soft body deformation gradient");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
}


bool GpuSoftBodyView::getSimElementDeformationGradients(const TensorDesc* dstTensor) const
{
    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }
    if (!checkTensorDevice(*dstTensor, mDevice, "simDeformationGradient", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "simDeformationGradient", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, mMaxSimElementsPerBody * getCount() * 9u, "simDeformationGradient", __FUNCTION__))
    {
        return false;
    }
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    copySoftBodyData(PxSoftBodyGpuDataFlag::eSIM_POSITION_INV_MASS, CopyEvent::eSbSimNodalValueBuffer,
                     (void**)mSimNodalValues.buffersD, mSimNodalValues.bufferSizesD, mSbIndicesD, (PxU32)getCount(),
                     mSimNodalValues.maxBufferSize);
    copySoftBodyData(PxSoftBodyGpuDataFlag::eSIM_TET_ROTATIONS, CopyEvent::eSbSimElementRotationBuffer,
                     (void**)mSimElementRotations.buffersD, mSimElementRotations.bufferSizesD, mSbIndicesD,
                     (PxU32)getCount(), mSimElementRotations.maxBufferSize);

    if (!computeSbElementDeformationGradients(
            static_cast<PxMat33*>(dstTensor->data), mSbRecordsDev, (PxU32)getCount(), mMaxSimElementsPerBody, true))
    {
        CARB_LOG_ERROR("Failed to fetch soft body deformation gradient");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
}

bool GpuSoftBodyView::getElementStresses(const TensorDesc* dstTensor) const
{
    // PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    // return fetchPxValues(dstTensor, PxSoftBodyGpuDataFlag::eTET_STRESS, SoftBodyData::eElementTensor,
    //                      CopyEvent::eSbElementTensorBuffer, "elementCauchyStresses",
    //                      mMaxElementsPerBody * getCount() * 9u, (void**)mElementRestPoses.buffersD,
    //                      mElementRestPoses.bufferSizesD, mSbIndicesD, (PxU32)getCount(), mElementRestPoses.maxBufferSize,
    //                      mMaxElementsPerBody, __FUNCTION__);
    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }
    if (!checkTensorDevice(*dstTensor, mDevice, "deformationGradient", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "deformationGradient", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, mMaxElementsPerBody * getCount() * 9u, "deformationGradient", __FUNCTION__))
    {
        return false;
    }
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    std::vector<PxVec4> matProperties(getCount());
    std::vector<PxFEMSoftBodyMaterialModel::Enum> matModel(getCount());
    for (PxU32 i = 0; i < getCount(); i++)
    {
        PxFEMSoftBodyMaterial* material = nullptr;
        mSbRecords[i].softBody->getShape()->getSoftBodyMaterials(&material, 1);
        matProperties[i] = { material->getYoungsModulus(), material->getPoissons(), material->getDamping(),
                             material->getDampingScale() };
        matModel[i] = material->getMaterialModel();
    }

    if (!CHECK_CUDA(cudaMemcpy(
            materialPropertiesDev, matProperties.data(), getCount() * sizeof(PxVec4), cudaMemcpyHostToDevice)))
    {
        CARB_LOG_ERROR("Unable to copy data from host to device for material properties");
        return false;
    }
    if (!CHECK_CUDA(cudaMemcpy(materialModelsDev, matModel.data(), getCount() * sizeof(PxFEMSoftBodyMaterialModel),
                               cudaMemcpyHostToDevice)))
    {
        CARB_LOG_ERROR("Unable to copy data from host to device for material models");
        return false;
    }

    copySoftBodyData(PxSoftBodyGpuDataFlag::eTET_POSITION_INV_MASS, CopyEvent::eSbNodalPositionBuffer,
                     (void**)mNodalPositions.buffersD, mNodalPositions.bufferSizesD, mSbIndicesD, (PxU32)getCount(),
                     mNodalPositions.maxBufferSize);
    copySoftBodyData(PxSoftBodyGpuDataFlag::eTET_ROTATIONS, CopyEvent::eSbElementRotationBuffer,
                     (void**)mElementRotations.buffersD, mElementRotations.bufferSizesD, mSbIndicesD, (PxU32)getCount(),
                     mElementRotations.maxBufferSize);

    if (!computeSbElementStresses(static_cast<PxMat33*>(dstTensor->data), mSbRecordsDev, materialPropertiesDev,
                                  materialModelsDev, (PxU32)getCount(), mMaxElementsPerBody, false))
    {
        CARB_LOG_ERROR("Failed to fetch soft body deformation gradient");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
}

bool GpuSoftBodyView::getSimElementStresses(const TensorDesc* dstTensor) const
{
    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }
    if (!checkTensorDevice(*dstTensor, mDevice, "simElementStresses", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "simElementStresses", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, mMaxSimElementsPerBody * getCount() * 9u, "simElementStresses", __FUNCTION__))
    {
        return false;
    }
    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    std::vector<PxVec4> matProperties(getCount());
    std::vector<PxFEMSoftBodyMaterialModel::Enum> matModel(getCount());
    for (PxU32 i = 0; i < getCount(); i++)
    {
        PxFEMSoftBodyMaterial* material = nullptr;
        mSbRecords[i].softBody->getShape()->getSoftBodyMaterials(&material, 1);
        matProperties[i] = { material->getYoungsModulus(), material->getPoissons(), material->getDamping(),
                             material->getDampingScale() };
        matModel[i] = material->getMaterialModel();
    }

    if (!CHECK_CUDA(cudaMemcpy(
            materialPropertiesDev, matProperties.data(), getCount() * sizeof(PxVec4), cudaMemcpyHostToDevice)))
    {
        CARB_LOG_ERROR("Unable to copy data from host to device for material properties");
        return false;
    }
    if (!CHECK_CUDA(cudaMemcpy(materialModelsDev, matModel.data(), getCount() * sizeof(PxFEMSoftBodyMaterialModel),
                               cudaMemcpyHostToDevice)))
    {
        CARB_LOG_ERROR("Unable to copy data from host to device for material models");
        return false;
    }

    copySoftBodyData(PxSoftBodyGpuDataFlag::eSIM_POSITION_INV_MASS, CopyEvent::eSbSimNodalValueBuffer,
                     (void**)mSimNodalValues.buffersD, mSimNodalValues.bufferSizesD, mSbIndicesD, (PxU32)getCount(),
                     mSimNodalValues.maxBufferSize);
    copySoftBodyData(PxSoftBodyGpuDataFlag::eSIM_TET_ROTATIONS, CopyEvent::eSbSimElementRotationBuffer,
                     (void**)mSimElementRotations.buffersD, mSimElementRotations.bufferSizesD, mSbIndicesD, (PxU32)getCount(),
                     mSimElementRotations.maxBufferSize);

    if (!computeSbElementStresses(static_cast<PxMat33*>(dstTensor->data), mSbRecordsDev, materialPropertiesDev,
                                  materialModelsDev, (PxU32)getCount(), mMaxSimElementsPerBody, true))
    {
        CARB_LOG_ERROR("Failed to fetch soft body deformation gradient");
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return true;
}

} // namespace tensors
} // namespace physx
} // namespace omni

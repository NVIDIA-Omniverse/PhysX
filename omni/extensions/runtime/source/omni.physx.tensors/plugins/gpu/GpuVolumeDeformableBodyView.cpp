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

#include "GpuVolumeDeformableBodyView.h"
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

GpuVolumeDeformableBodyView::GpuVolumeDeformableBodyView(GpuSimulationView* sim, const std::vector<DeformableBodyEntry>& entries, int device)
    : BaseVolumeDeformableBodyView(sim, entries), mDevice(device)
{
    // NOTE: we assume that the simulation and collision mesh indices are constant throughout the simulation to avoid multiple data copy.
    if (sim)
        mGpuSimData = sim->getGpuSimulationData();

    uint32_t numBodies = getCount();
    PxU32 maxSimElements = 0;
    PxU32 maxSimNodes = 0;
    PxU32 maxRestNodes = 0;
    PxU32 maxCollElements = 0;
    PxU32 maxCollNodes = 0;
    mViewIndices.resize(numBodies);

    for (PxU32 i = 0; i < numBodies; i++)
    {
        PxDeformableVolume* deformableBody = static_cast<PxDeformableVolume*>(entries[i].body);
        mViewIndices[i] = i;
        GpuDeformableBodyRecord deformableBodyRecord;

        if (mEntries[i].subspace)
        {
            const carb::Float3& origin = mEntries[i].subspace->origin;
            deformableBodyRecord.origin = { origin.x, origin.y, origin.z };
        }
        else
        {
            deformableBodyRecord.origin = { 0.0f, 0.0f, 0.0f };
        }
        PxTetrahedronMesh* simulationTetMesh = deformableBody->getSimulationMesh();
        deformableBodyRecord.numSimNodes = simulationTetMesh->getNbVertices();
        deformableBodyRecord.numSimElements = simulationTetMesh->getNbTetrahedrons();
        CARB_ASSERT(deformableBodyRecord.numSimElements * 4 == entries[i].simIndices.size());
        deformableBody->setDeformableVolumeFlag(PxDeformableVolumeFlag::ePARTIALLY_KINEMATIC, true);

        // currently only supporting identical topology to sim mesh
        deformableBodyRecord.numRestNodes = deformableBodyRecord.numSimNodes;

        PxTetrahedronMesh* collisionTetMesh = deformableBody->getCollisionMesh();
        deformableBodyRecord.numCollNodes = collisionTetMesh->getNbVertices();
        deformableBodyRecord.numCollElements = collisionTetMesh->getNbTetrahedrons();
        CARB_ASSERT(deformableBodyRecord.numCollElements * 4 == entries[i].collIndices.size());

        const PxU32 simElementIndicesBufferSize = deformableBodyRecord.numSimElements * sizeof(PxU32) * 4;
        const PxU32 collElementIndicesBufferSize = deformableBodyRecord.numCollElements * sizeof(PxU32) * 4;
        const PxU32 simNodalVec4BufferSize = deformableBodyRecord.numSimNodes * sizeof(PxVec4);
        const PxU32 restNodalVec3BufferSize = deformableBodyRecord.numRestNodes * sizeof(PxVec3);
        const PxU32 collNodalVec4BufferSize = deformableBodyRecord.numCollNodes * sizeof(PxVec4);

        // check for existing record
        // update the records and buffers only if no other views have done so yet
        auto it = mGpuSimData->mDeformableBodyToRecord.find(deformableBody);
        if (it == mGpuSimData->mDeformableBodyToRecord.end())
        {
            // directly references to PxDeformableVolume buffers
            deformableBodyRecord.simNodalPositions = deformableBody->getSimPositionInvMassBufferD();
            deformableBodyRecord.simNodalVelocities = deformableBody->getSimVelocityBufferD();
            deformableBodyRecord.collNodalPositions = deformableBody->getPositionInvMassBufferD();

            // tensor lib managed data
            if (!CHECK_CUDA(cudaMalloc(&(deformableBodyRecord.simElementIndices), simElementIndicesBufferSize)))
                return;

            if (!CHECK_CUDA(cudaMemcpy(deformableBodyRecord.simElementIndices, entries[i].simIndices.data(),
                                       simElementIndicesBufferSize, cudaMemcpyHostToDevice)))
            {
                CARB_LOG_ERROR("Unable to copy data from host to device for simElementIndicesBuffer\n");
                return;
            }

            if (!CHECK_CUDA(cudaMalloc(&(deformableBodyRecord.simNodalKinematicTargets), simNodalVec4BufferSize)))
                return;

            if (!CHECK_CUDA(cudaMalloc(&(deformableBodyRecord.restNodalPositions), restNodalVec3BufferSize)))
                return;

            if (!CHECK_CUDA(cudaMemcpy(deformableBodyRecord.restNodalPositions, entries[i].restPositions.data(),
                                       restNodalVec3BufferSize, cudaMemcpyHostToDevice)))
            {
                CARB_LOG_ERROR("Unable to copy data from host to device for restNodalPositionsBuffer\n");
                return;
            }

            if (!CHECK_CUDA(cudaMalloc(&(deformableBodyRecord.collElementIndices), collElementIndicesBufferSize)))
                return;

            if (!CHECK_CUDA(cudaMemcpy(deformableBodyRecord.collElementIndices, entries[i].collIndices.data(),
                                       collElementIndicesBufferSize, cudaMemcpyHostToDevice)))
            {
                CARB_LOG_ERROR("Unable to copy data from host to device for collElementIndicesBuffer\n");
                return;
            }

            mGpuSimData->mDeformableBodyToRecord[deformableBody] = deformableBodyRecord;
        }
        else
        {
            deformableBodyRecord = mGpuSimData->mDeformableBodyToRecord[deformableBody];
        }

        mSimElementIndices.bufferSizesH.push_back(simElementIndicesBufferSize);
        mSimElementIndices.buffersH.push_back(deformableBodyRecord.simElementIndices);
        mSimNodalKinematicTargets.bufferSizesH.push_back(simNodalVec4BufferSize);
        mSimNodalKinematicTargets.buffersH.push_back(deformableBodyRecord.simNodalKinematicTargets);
        mRestNodalPositions.bufferSizesH.push_back(restNodalVec3BufferSize);
        mRestNodalPositions.buffersH.push_back(deformableBodyRecord.restNodalPositions);
        mCollElementIndices.bufferSizesH.push_back(collElementIndicesBufferSize);
        mCollElementIndices.buffersH.push_back(deformableBodyRecord.collElementIndices);

        deformableBodyRecord.deformableBody = deformableBody;

        mDeformableBodyRecords.push_back(deformableBodyRecord);
        maxSimElements = PxMax(maxSimElements, deformableBodyRecord.numSimElements);
        maxSimNodes = PxMax(maxSimNodes, deformableBodyRecord.numSimNodes);
        maxRestNodes = PxMax(maxRestNodes, deformableBodyRecord.numRestNodes);
        maxCollElements = PxMax(maxCollElements, deformableBodyRecord.numCollElements);
        maxCollNodes = PxMax(maxCollNodes, deformableBodyRecord.numCollNodes);
    }

    mMaxSimElementsPerBody = maxSimElements;
    mMaxSimNodesPerBody = maxSimNodes;
    mMaxRestNodesPerBody = maxRestNodes;
    mMaxCollElementsPerBody = maxCollElements;
    mMaxCollNodesPerBody = maxCollNodes;

    mSimElementIndices.setMaxBufferSize();
    mCollElementIndices.setMaxBufferSize();
    mSimNodalKinematicTargets.setMaxBufferSize();
    mRestNodalPositions.setMaxBufferSize();

    mSimElementIndices.uploadDeviceData("mSimElementIndices");
    mCollElementIndices.uploadDeviceData("mCollElementIndices");
    mSimNodalKinematicTargets.uploadDeviceData("mSimNodalKinematicTargets");
    mRestNodalPositions.uploadDeviceData("mRestNodalPositions");

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    // view indices
    DeformableBodyBufferManager::allocDevMemAndCopyH2D((void**)&mViewIndicesD, mViewIndices.data(), sizeof(PxU32) * numBodies, "ViewIndices");
    // deformable body records
    DeformableBodyBufferManager::allocDevMemAndCopyH2D((void**)&mDeformableBodyRecordsD, mDeformableBodyRecords.data(), sizeof(GpuDeformableBodyRecord) * numBodies, "GpuDeformableBodyRecord");
}

GpuVolumeDeformableBodyView::~GpuVolumeDeformableBodyView()
{
    if (mGpuSimData)
    {
        CudaContextGuard ctxGuard(mGpuSimData->mCtx);

        CHECK_CUDA(cudaFree(mViewIndicesD));
        CHECK_CUDA(cudaFree(mDeformableBodyRecordsD));

        mSimNodalKinematicTargets.releaseDeviceMem();
        mSimElementIndices.releaseDeviceMem();
        mCollElementIndices.releaseDeviceMem();
        // NOTE: should we remove the device buffer allocated for a specific deformable body as soon as they are not managed
        // by any views or should we keep them around maybe some new view are going to manage them?
        // former is more memory efficient, but later is more performant if the users keep creating view objects
    }
}

bool GpuVolumeDeformableBodyView::submitData(const TensorDesc* srcTensor,
                                             const TensorDesc* indexTensor,
                                             const DeformableBodyData::Enum dataFlag,
                                             const uint32_t maxItemsPerBody,
                                             const uint32_t itemSize,
                                             const char* tensorName,
                                             const char* callingFunctionName,
                                             const checkTensorType checkTensorBaseType)
{
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    PxU32 expectedSize = maxItemsPerBody * getCount() * itemSize;
    if (!checkTensorDevice(*srcTensor, mDevice, tensorName, callingFunctionName) ||
        !checkTensorBaseType(*srcTensor, tensorName, callingFunctionName) ||
        !checkTensorSizeExact(*srcTensor, expectedSize, tensorName, callingFunctionName))
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    const PxU32* viewIndicesD = nullptr;
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

        // need to re-batch the deformable body pointers to only apply data to the ones required modification
        viewIndicesH.resize(numIndices);
        if (!CHECK_CUDA(cudaMemcpy(viewIndicesH.data(), viewIndicesD, numIndices * sizeof(PxU32), cudaMemcpyDeviceToHost)))
        {
            CARB_LOG_ERROR("Unable to copy data from device to host for viewIndicesD to viewIndicesH\n");
            return false;
        }
    }
    else
    {
        viewIndicesD = mViewIndicesD;
        viewIndicesH = mViewIndices;
        numIndices = getCount();
    }

    {
        bool result = false;
        switch (dataFlag)
        {
        case DeformableBodyData::eSimNodalKinematicTarget:
            result = submitDeformableBodyVec4Data(static_cast<PxVec4*>(srcTensor->data), viewIndicesD,
                                                  mDeformableBodyRecordsD, dataFlag, numIndices, maxItemsPerBody);
            break;
        case DeformableBodyData::eSimNodalPosition:
        case DeformableBodyData::eSimNodalVelocity:
            result = submitDeformableBodyVec3Data(static_cast<PxVec3*>(srcTensor->data), viewIndicesD,
                                                  mDeformableBodyRecordsD, dataFlag, numIndices, maxItemsPerBody);
            break;
        default:
            CARB_LOG_ERROR("Invalid DeformableBodyData encountered.");
            break;
        }

        if (!result)
        {
            CARB_LOG_ERROR("Failed to submit deformable body data");
            return false;
        }

        CHECK_CUDA(cudaStreamSynchronize(nullptr));
        SYNCHRONIZE_CUDA();

        for (size_t i = 0; i < numIndices; i++)
        {
            PxU32 viewIdx = viewIndicesH[i];
            GpuDeformableBodyRecord& record = mDeformableBodyRecords[viewIdx];
            PxDeformableVolume* deformableVolume = static_cast<PxDeformableVolume*>(record.deformableBody);

            switch (dataFlag)
            {
            case DeformableBodyData::eSimNodalKinematicTarget:
                deformableVolume->setKinematicTargetBufferD(record.simNodalKinematicTargets);
                break;
            case DeformableBodyData::eSimNodalPosition:
                deformableVolume->markDirty(PxDeformableVolumeDataFlag::eSIM_POSITION_INVMASS);
                break;
            case DeformableBodyData::eSimNodalVelocity:
                deformableVolume->markDirty(PxDeformableVolumeDataFlag::eSIM_VELOCITY);
                break;
            }
        }
    }

    return true;
}

bool GpuVolumeDeformableBodyView::fetchData(const TensorDesc* dstTensor,
                                            const DeformableBodyData::Enum dataFlag,
                                            const uint32_t maxItemsPerBody,
                                            const uint32_t itemSize,
                                            const char* tensorName,
                                            const char* callingFunctionName,
                                            const checkTensorType checkTensorBaseType) const
{
    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    PxU32 expectedSize = maxItemsPerBody * getCount() * itemSize;
    if (!checkTensorDevice(*dstTensor, mDevice, tensorName, callingFunctionName) ||
        !checkTensorBaseType(*dstTensor, tensorName, callingFunctionName) ||
        !checkTensorSizeExact(*dstTensor, expectedSize, tensorName, callingFunctionName))
    {
        return false;
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    bool result = false;
    PxU32 numBodies = getCount();
    switch (dataFlag)
    {
    case DeformableBodyData::eCollElementIndices:
    case DeformableBodyData::eSimElementIndices:
        result = fetchDeformableBodyUInt4Data(static_cast<PxU32*>(dstTensor->data), mDeformableBodyRecordsD, dataFlag,
                                              numBodies, maxItemsPerBody);
        break;
    case DeformableBodyData::eSimNodalPosition:
    case DeformableBodyData::eSimNodalVelocity:
    case DeformableBodyData::eRestNodalPosition:
    case DeformableBodyData::eCollNodalPosition:
        result = fetchDeformableBodyVec3Data(static_cast<PxVec3*>(dstTensor->data), mDeformableBodyRecordsD, dataFlag,
                                             numBodies, maxItemsPerBody);
        break;
    case DeformableBodyData::eSimNodalKinematicTarget:
        result = fetchDeformableBodyVec4Data(static_cast<PxVec4*>(dstTensor->data), mDeformableBodyRecordsD, dataFlag,
                                             numBodies, maxItemsPerBody);
        break;
    default:
        CARB_LOG_ERROR("Invalid DeformableBodyData encountered.");
        break;
    }

    if (!result)
    {
        CARB_LOG_ERROR("Failed to fetch deformable body %s", tensorName);
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    SYNCHRONIZE_CUDA();
    return true;
};

bool GpuVolumeDeformableBodyView::getSimulationElementIndices(const TensorDesc* dstTensor) const
{
    return fetchData(dstTensor, DeformableBodyData::eSimElementIndices, mMaxSimElementsPerBody, 4u,
                     "simulationElementIndices", __FUNCTION__, checkTensorInt32);
}

bool GpuVolumeDeformableBodyView::getSimulationNodalPositions(const TensorDesc* dstTensor) const
{
    return fetchData(dstTensor, DeformableBodyData::eSimNodalPosition, mMaxSimNodesPerBody, 3u,
                     "simulationNodalPositions", __FUNCTION__, checkTensorFloat32);
}

bool GpuVolumeDeformableBodyView::setSimulationNodalPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return submitData(srcTensor, indexTensor, DeformableBodyData::Enum::eSimNodalPosition, mMaxSimNodesPerBody,
                      3u, "simulationNodalPositions", __FUNCTION__, checkTensorFloat32);
}

bool GpuVolumeDeformableBodyView::getSimulationNodalVelocities(const TensorDesc* dstTensor) const
{
    return fetchData(dstTensor, DeformableBodyData::eSimNodalVelocity, mMaxSimNodesPerBody, 3u,
                     "simulationNodalVelocities", __FUNCTION__, checkTensorFloat32);
}

bool GpuVolumeDeformableBodyView::setSimulationNodalVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return submitData(srcTensor, indexTensor, DeformableBodyData::Enum::eSimNodalVelocity, mMaxSimNodesPerBody,
                      3u, "simulationNodalVelocities", __FUNCTION__, checkTensorFloat32);
}

bool GpuVolumeDeformableBodyView::getSimulationNodalKinematicTargets(const TensorDesc* dstTensor) const
{
    return fetchData(dstTensor, DeformableBodyData::eSimNodalKinematicTarget, mMaxSimNodesPerBody, 4u,
                     "simulationNodalKinematicTargets", __FUNCTION__, checkTensorFloat32);
}

bool GpuVolumeDeformableBodyView::setSimulationNodalKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return submitData(srcTensor, indexTensor, DeformableBodyData::Enum::eSimNodalKinematicTarget, mMaxSimNodesPerBody,
                      4u, "simulationNodalKinematicTargets", __FUNCTION__, checkTensorFloat32);
}

bool GpuVolumeDeformableBodyView::getRestElementIndices(const TensorDesc* dstTensor) const
{
    // right now we only support rest element indices identical to sim element indices
    return fetchData(dstTensor, DeformableBodyData::eSimElementIndices, mMaxSimElementsPerBody, 4u,
                     "restElementIndices", __FUNCTION__, checkTensorInt32);
}

bool GpuVolumeDeformableBodyView::getRestNodalPositions(const TensorDesc* dstTensor) const
{
    return fetchData(dstTensor, DeformableBodyData::eRestNodalPosition, mMaxRestNodesPerBody,
                     3u, "restNodalPositions", __FUNCTION__, checkTensorFloat32);
}

bool GpuVolumeDeformableBodyView::getCollisionElementIndices(const TensorDesc* dstTensor) const
{
    return fetchData(dstTensor, DeformableBodyData::eCollElementIndices, mMaxCollElementsPerBody, 4u,
                     "collisionElementIndices", __FUNCTION__, checkTensorInt32);
}

bool GpuVolumeDeformableBodyView::getCollisionNodalPositions(const TensorDesc* dstTensor) const
{
    return fetchData(dstTensor, DeformableBodyData::eCollNodalPosition, mMaxCollNodesPerBody,
                     3u, "collisionNodalPositions", __FUNCTION__, checkTensorFloat32);
}

} // namespace tensors
} // namespace physx
} // namespace omni

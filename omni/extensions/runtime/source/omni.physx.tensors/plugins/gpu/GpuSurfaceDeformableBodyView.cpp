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

#include "GpuSurfaceDeformableBodyView.h"
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

GpuSurfaceDeformableBodyView::GpuSurfaceDeformableBodyView(GpuSimulationView* sim,
                                                           const std::vector<DeformableBodyEntry>& entries,
                                                           int device)
    : BaseSurfaceDeformableBodyView(sim, entries), mDevice(device)
{
    // NOTE: we assume that the simulation and collision mesh indices are constant throughout the simulation to avoid
    // multiple data copy.
    if (sim)
        mGpuSimData = sim->getGpuSimulationData();

    uint32_t numBodies = getCount();
    PxU32 maxSimElements = 0;
    PxU32 maxSimNodes = 0;
    PxU32 maxRestNodes = 0;
    mViewIndices.resize(numBodies);

    for (PxU32 i = 0; i < numBodies; i++)
    {
        PxDeformableSurface* deformableBody = static_cast<PxDeformableSurface*>(entries[i].body);
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

        const PxShape* shape = deformableBody->getShape();
        if (!shape || shape->getGeometry().getType() != PxGeometryType::eTRIANGLEMESH)
            return;

        const PxTriangleMeshGeometry& triangleMeshGeom = static_cast<const PxTriangleMeshGeometry&>(shape->getGeometry());
        if (!triangleMeshGeom.triangleMesh)
            return;

        const PxTriangleMesh& triangleMesh = *triangleMeshGeom.triangleMesh;

        deformableBodyRecord.numSimNodes = triangleMesh.getNbVertices();
        deformableBodyRecord.numSimElements = triangleMesh.getNbTriangles();

        // currently only supporting identical topology to sim mesh
        deformableBodyRecord.numRestNodes = deformableBodyRecord.numSimNodes;

        deformableBodyRecord.numCollNodes = 0;
        deformableBodyRecord.numCollElements = 0;

        CARB_ASSERT(deformableBodyRecord.numSimElements * 4 == entries[i].simIndices.size());
        CARB_ASSERT(0 == entries[i].collIndices.size());

        const PxU32 simElementIndicesBufferSize = deformableBodyRecord.numSimElements * sizeof(PxU32) * 3;
        const PxU32 simNodalVec4BufferSize = deformableBodyRecord.numSimNodes * sizeof(PxVec4);
        const PxU32 restNodalVec3BufferSize = deformableBodyRecord.numRestNodes * sizeof(PxVec3);

        // check for existing record
        // update the records and buffers only if no other views have done so yet
        auto it = mGpuSimData->mDeformableBodyToRecord.find(deformableBody);
        if (it == mGpuSimData->mDeformableBodyToRecord.end())
        {
            // directly references to PxDeformableSurface buffers
            deformableBodyRecord.simNodalPositions = deformableBody->getPositionInvMassBufferD();
            deformableBodyRecord.simNodalVelocities = deformableBody->getVelocityBufferD();
            deformableBodyRecord.collNodalPositions = nullptr;

            // tensor lib managed data
            if (!CHECK_CUDA(cudaMalloc(&(deformableBodyRecord.simElementIndices), simElementIndicesBufferSize)))
                return;

            if (!CHECK_CUDA(cudaMemcpy(deformableBodyRecord.simElementIndices, entries[i].simIndices.data(),
                                       simElementIndicesBufferSize, cudaMemcpyHostToDevice)))
            {
                CARB_LOG_ERROR("Unable to copy data from host to device for simElementIndicesBuffer\n");
                return;
            }

            deformableBodyRecord.collElementIndices = nullptr;
            deformableBodyRecord.simNodalKinematicTargets = nullptr;

            if (!CHECK_CUDA(cudaMalloc(&(deformableBodyRecord.restNodalPositions), restNodalVec3BufferSize)))
                return;

            if (!CHECK_CUDA(cudaMemcpy(deformableBodyRecord.restNodalPositions, entries[i].restPositions.data(),
                                       restNodalVec3BufferSize, cudaMemcpyHostToDevice)))
            {
                CARB_LOG_ERROR("Unable to copy data from host to device for restNodalPositionsBuffer\n");
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
        mRestNodalPositions.bufferSizesH.push_back(restNodalVec3BufferSize);
        mRestNodalPositions.buffersH.push_back(deformableBodyRecord.restNodalPositions);

        deformableBodyRecord.deformableBody = deformableBody;

        mDeformableBodyRecords.push_back(deformableBodyRecord);
        maxSimElements = PxMax(maxSimElements, deformableBodyRecord.numSimElements);
        maxSimNodes = PxMax(maxSimNodes, deformableBodyRecord.numSimNodes);
        maxRestNodes = PxMax(maxRestNodes, deformableBodyRecord.numRestNodes);
    }

    mMaxSimElementsPerBody = maxSimElements;
    mMaxSimNodesPerBody = maxSimNodes;
    mMaxRestNodesPerBody = maxRestNodes;

    mSimElementIndices.setMaxBufferSize();
    mRestNodalPositions.setMaxBufferSize();

    mSimElementIndices.uploadDeviceData("mSimElementIndices");
    mRestNodalPositions.uploadDeviceData("mRestNodalPositions");

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    // view indices
    DeformableBodyBufferManager::allocDevMemAndCopyH2D(
        (void**)&mViewIndicesD, mViewIndices.data(), sizeof(PxU32) * numBodies, "ViewIndices");

    // deformable body records
    DeformableBodyBufferManager::allocDevMemAndCopyH2D((void**)&mDeformableBodyRecordsD, mDeformableBodyRecords.data(),
                                                       sizeof(GpuDeformableBodyRecord) * numBodies,
                                                       "GpuDeformableBodyRecord");
}

GpuSurfaceDeformableBodyView::~GpuSurfaceDeformableBodyView()
{
    if (mGpuSimData)
    {
        CudaContextGuard ctxGuard(mGpuSimData->mCtx);

        CHECK_CUDA(cudaFree(mViewIndicesD));
        CHECK_CUDA(cudaFree(mDeformableBodyRecordsD));

        mSimElementIndices.releaseDeviceMem();
        mRestNodalPositions.releaseDeviceMem();
        // NOTE: should we remove the device buffer allocated for a specific deformable body as soon as they are not
        // managed by any views or should we keep them around maybe some new view are going to manage them? former is
        // more memory efficient, but later is more performant if the users keep creating view objects
    }
}

bool GpuSurfaceDeformableBodyView::submitData(const TensorDesc* srcTensor,
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
            PxDeformableSurface* deformableSurface = static_cast<PxDeformableSurface*>(record.deformableBody);

            switch (dataFlag)
            {
            case DeformableBodyData::eSimNodalPosition:
                deformableSurface->markDirty(PxDeformableSurfaceDataFlag::ePOSITION_INVMASS);
                break;
            case DeformableBodyData::eSimNodalVelocity:
                deformableSurface->markDirty(PxDeformableSurfaceDataFlag::eVELOCITY);
                break;
            }
        }
    }

    return true;
}

bool GpuSurfaceDeformableBodyView::fetchData(const TensorDesc* dstTensor,
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
    case DeformableBodyData::eSimElementIndices:
        result = fetchDeformableBodyUInt3Data(
            static_cast<PxU32*>(dstTensor->data), mDeformableBodyRecordsD, dataFlag, numBodies, maxItemsPerBody);
        break;
    case DeformableBodyData::eSimNodalPosition:
    case DeformableBodyData::eSimNodalVelocity:
    case DeformableBodyData::eRestNodalPosition:
        result = fetchDeformableBodyVec3Data(
            static_cast<PxVec3*>(dstTensor->data), mDeformableBodyRecordsD, dataFlag, numBodies, maxItemsPerBody);
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

bool GpuSurfaceDeformableBodyView::getSimulationElementIndices(const TensorDesc* dstTensor) const
{
    return fetchData(dstTensor, DeformableBodyData::eSimElementIndices, mMaxSimElementsPerBody, 3u,
                     "simulationElementIndices", __FUNCTION__, checkTensorInt32);
}

bool GpuSurfaceDeformableBodyView::getSimulationNodalPositions(const TensorDesc* dstTensor) const
{
    return fetchData(dstTensor, DeformableBodyData::eSimNodalPosition, mMaxSimNodesPerBody, 3u,
                     "simulationNodalPositions", __FUNCTION__, checkTensorFloat32);
}

bool GpuSurfaceDeformableBodyView::setSimulationNodalPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return submitData(srcTensor, indexTensor, DeformableBodyData::Enum::eSimNodalPosition, mMaxSimNodesPerBody, 3u,
                      "simulationNodalPositions", __FUNCTION__, checkTensorFloat32);
}

bool GpuSurfaceDeformableBodyView::getSimulationNodalVelocities(const TensorDesc* dstTensor) const
{
    return fetchData(dstTensor, DeformableBodyData::eSimNodalVelocity, mMaxSimNodesPerBody, 3u,
                     "simulationNodalVelocities", __FUNCTION__, checkTensorFloat32);
}

bool GpuSurfaceDeformableBodyView::setSimulationNodalVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return submitData(srcTensor, indexTensor, DeformableBodyData::Enum::eSimNodalVelocity, mMaxSimNodesPerBody, 3u,
                      "simulationNodalVelocities", __FUNCTION__, checkTensorFloat32);
}

bool GpuSurfaceDeformableBodyView::getSimulationNodalKinematicTargets(const TensorDesc* dstTensor) const
{
    CARB_LOG_WARN("GpuSurfaceDeformableBodyView::getSimulationNodalKinematicTargets is not supported yet");
    return false;
}

bool GpuSurfaceDeformableBodyView::setSimulationNodalKinematicTargets(const TensorDesc* srcTensor,
                                                                     const TensorDesc* indexTensor)
{
    CARB_LOG_WARN("GpuSurfaceDeformableBodyView::setSimulationNodalKinematicTargets is not supported yet");
    return false;
}

bool GpuSurfaceDeformableBodyView::getRestElementIndices(const TensorDesc* dstTensor) const
{
    // right now we only support rest element indices identical to sim element indices
    return fetchData(dstTensor, DeformableBodyData::eSimElementIndices, mMaxSimElementsPerBody, 3u,
                     "restElementIndices", __FUNCTION__, checkTensorInt32);
}

bool GpuSurfaceDeformableBodyView::getRestNodalPositions(const TensorDesc* dstTensor) const
{
    return fetchData(dstTensor, DeformableBodyData::eRestNodalPosition, mMaxRestNodesPerBody, 3u,
                     "restNodalPositions", __FUNCTION__, checkTensorFloat32);
}

bool GpuSurfaceDeformableBodyView::getCollisionElementIndices(const TensorDesc* dstTensor) const
{
    return getSimulationElementIndices(dstTensor);
}

bool GpuSurfaceDeformableBodyView::getCollisionNodalPositions(const TensorDesc* dstTensor) const
{
    return getSimulationNodalPositions(dstTensor);
}

} // namespace tensors
} // namespace physx
} // namespace omni

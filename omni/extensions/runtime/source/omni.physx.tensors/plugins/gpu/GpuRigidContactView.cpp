// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "GpuRigidContactView.h"
#include "GpuSimulationView.h"
#include "CudaKernels.h"

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
using namespace pxr;

namespace omni
{
namespace physx
{
namespace tensors
{


GpuRigidContactView::GpuRigidContactView(GpuSimulationView* sim,
                                         const std::vector<RigidContactSensorEntry>& entries,
                                         uint32_t numFilters,
                                         uint32_t maxContactDataCount,
                                         int device)
    : BaseRigidContactView(sim, entries, numFilters, maxContactDataCount), mDevice(device)
{
    mGpuSimData = sim->getGpuSimulationData();

    PxU32 numSensors = PxU32(mEntries.size());

    PxU32 linkBufSize = (mGpuSimData->mMaxArtiIndex + 1) * mGpuSimData->mMaxLinks;
    PxU32 rdBufSize = mGpuSimData->mMaxRdIndex + 1;

    std::vector<PxU32> linkContactIndices(linkBufSize, 0xffffffff);
    std::vector<PxU32> rdContactIndices(rdBufSize, 0xffffffff);

    std::vector<GpuRigidContactFilterIdPair> filterLookup(numSensors * numFilters);

    for (PxU32 i = 0; i < numSensors; i++)
    {
        auto& entry = mEntries[i];

        if (entry.link)
        {
            //printf("~!~!~! RigidContactView entry %u is an articulation link @ %p\n", i, entry.link);
            PxArticulationReducedCoordinate& arti = entry.link->getArticulation();
            PxArticulationGPUIndex artiIdx = arti.getGPUIndex();
            PxU32 linkIdx = entry.link->getLinkIndex();
            PxU32 globalLinkIdx = artiIdx * mGpuSimData->mMaxLinks + linkIdx;
            linkContactIndices[globalLinkIdx] = i;
        }
        else if (entry.rd)
        {
            //printf("~!~!~! RigidContactView entry %u is a rigid dynamic @ %p\n", i, entry.rd);
            PxU32 rdIdx = entry.rd->getInternalIslandNodeIndex().index();
            rdContactIndices[rdIdx] = i;
        }
        else if (entry.shape)
        {
            // TODO!!! - handle shapes
            CARB_LOG_WARN("GPU contact info for collider '%s' is not supported", entry.path.GetText());
        }

        if (numFilters > 0)
        {
            // filter lookup table per referent
            GpuRigidContactFilterIdPair* filterIdPairs = filterLookup.data() + i * numFilters;
            PxU32 j = 0;
            for (auto& srcPair : entry.filterIndexMap)
            {
                const SdfPath& filterPath = intToPath(srcPair.first);

                PxActor* actor = static_cast<PxActor*>(g_physx->getPhysXPtr(filterPath, omni::physx::ePTActor));
                if (!actor)
                {
                    actor = static_cast<PxArticulationLink*>(g_physx->getPhysXPtr(filterPath, omni::physx::ePTLink));
                }

                if (actor)
                {
                    filterIdPairs[j].actor = actor;
                    filterIdPairs[j].filterIndex = srcPair.second;
                    ++j;
                }
                else
                {
                    // TODO!!! - handle shapes
                    CARB_LOG_WARN("GPU contact filter for collider '%s' is not supported", filterPath.GetText());
                }
            }

            std::sort(filterIdPairs, filterIdPairs + numFilters, GpuRigidContactFilterIdPair::LessThan());
        }
    }

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    if (mGpuSimData->mNumArtis > 0)
    {
        if (!CHECK_CUDA(cudaMalloc(&mLinkContactIndicesDev, linkBufSize * sizeof(PxU32))))
        {
            return;
        }
        if (!CHECK_CUDA(cudaMemcpy(mLinkContactIndicesDev, linkContactIndices.data(), linkBufSize * sizeof(PxU32), cudaMemcpyHostToDevice)))
        {
            return;
        }
    }

    if (mGpuSimData->mNumRds > 0)
    {
        if (!CHECK_CUDA(cudaMalloc(&mRdContactIndicesDev, rdBufSize * sizeof(PxU32))))
        {
            return;
        }
        if (!CHECK_CUDA(cudaMemcpy(mRdContactIndicesDev, rdContactIndices.data(), rdBufSize * sizeof(PxU32), cudaMemcpyHostToDevice)))
        {
            return;
        }
    }

    if (numFilters > 0)
    {
        if (!CHECK_CUDA(cudaMalloc(&mFilterLookupDev, numSensors * numFilters * sizeof(GpuRigidContactFilterIdPair))))
        {
            return;
        }
        if (!CHECK_CUDA(cudaMemcpy(mFilterLookupDev, filterLookup.data(), numSensors * numFilters * sizeof(GpuRigidContactFilterIdPair), cudaMemcpyHostToDevice)))
        {
            return;
        }
    }
}

GpuRigidContactView::~GpuRigidContactView()
{
    if (mGpuSimData)
    {
        PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

        CHECK_CUDA(cudaFree(mLinkContactIndicesDev));
        CHECK_CUDA(cudaFree(mFilterLookupDev));
        if (getFilterCount() > 0)
        {
            CHECK_CUDA(cudaFree(mRdContactIndicesDev));
        }
    }
}

bool GpuRigidContactView::getNetContactForces(const TensorDesc* dstTensor, float dt) const
{
    if (!mGpuSimData)
    {
        return false;
    }

    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "net contact forces", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "net contact forces", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getSensorCount() * 3, "net contact forces", __FUNCTION__))
    {
        return false;
    }

    PxVec3* dstForce = static_cast<PxVec3*>(dstTensor->data);

    // make sure we have the latest contact reports
    mGpuSimData->updateContactReports();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    float timeStepInv = 1.0f / dt;

    CHECK_CUDA(cudaMemset(dstForce, 0, getSensorCount() * sizeof(PxVec3)));
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    if (!fetchNetRigidContactForces(
            dstForce,
            mGpuSimData->mGpuContactPairsDev,
            mGpuSimData->mNumContactPairs,
            mGpuSimData->mMaxLinks,
            timeStepInv,
            mGpuSimData->mNodeIdx2ArtiGpuIdxDev,
            mRdContactIndicesDev,
            mLinkContactIndicesDev))
    {
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

bool GpuRigidContactView::getContactForceMatrix(const TensorDesc* dstTensor, float dt) const
{
    if (!mGpuSimData)
    {
        return false;
    }

    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, mDevice, "contact force matrix", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "contact force matrix", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getSensorCount() * getFilterCount() * 3, "contact force matrix", __FUNCTION__))
    {
        return false;
    }

    PxVec3* dstForce = static_cast<PxVec3*>(dstTensor->data);

    // make sure we have the latest contact reports
    mGpuSimData->updateContactReports();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);

    float timeStepInv = 1.0f / dt;

    CHECK_CUDA(cudaMemset(dstForce, 0, getSensorCount() * getFilterCount() * sizeof(PxVec3)));
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    if (!fetchRigidContactForceMatrix(
            dstForce,
            mGpuSimData->mGpuContactPairsDev,
            mGpuSimData->mNumContactPairs,
            mNumFilters,
            mGpuSimData->mMaxLinks,
            timeStepInv,
            mGpuSimData->mNodeIdx2ArtiGpuIdxDev,
            mRdContactIndicesDev,
            mLinkContactIndicesDev,
            mFilterLookupDev))
    {
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

bool GpuRigidContactView::getContactData(const TensorDesc* contactForceTensor,
                                         const TensorDesc* contactPointTensor,
                                         const TensorDesc* contactNormalTensor,
                                         const TensorDesc* contactSeparationTensor,
                                         const TensorDesc* contactCountTensor,
                                         const TensorDesc* contactStartIndicesTensor,
                                         float dt) const
{
    if (!mGpuSimData)
    {
        return false;
    }

    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (!contactForceTensor || !contactForceTensor->data || !contactPointTensor || !contactPointTensor->data ||
        !contactNormalTensor || !contactNormalTensor->data || !contactSeparationTensor ||
        !contactSeparationTensor->data || !contactCountTensor || !contactCountTensor->data ||
        !contactStartIndicesTensor || !contactStartIndicesTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*contactForceTensor, mDevice, "contact force buffer", __FUNCTION__) ||
        !checkTensorFloat32(*contactForceTensor, "contact force buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactForceTensor, getMaxContactDataCount(), "contact force buffer", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactPointTensor, mDevice, "contact point buffer", __FUNCTION__) ||
        !checkTensorFloat32(*contactPointTensor, "contact point buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactPointTensor, getMaxContactDataCount() * 3,
                              "contact point buffer", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactNormalTensor, mDevice, "contact normal buffer", __FUNCTION__) ||
        !checkTensorFloat32(*contactNormalTensor, "contact normal buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactNormalTensor,  getMaxContactDataCount() * 3,
                              "contact normal buffer", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*contactSeparationTensor, mDevice, "contact separation buffer", __FUNCTION__) ||
        !checkTensorFloat32(*contactSeparationTensor, "contact separation buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactSeparationTensor, getMaxContactDataCount(),
                              "contact separation buffer", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactCountTensor, mDevice, "contact count matrix", __FUNCTION__) ||
        !checkTensorInt32(*contactCountTensor, "contact count matrix", __FUNCTION__) ||
        !checkTensorSizeExact(
            *contactCountTensor, getSensorCount() * getFilterCount(), "contact count matrix", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactStartIndicesTensor, mDevice, "contact start indices matrix", __FUNCTION__) ||
        !checkTensorInt32(*contactStartIndicesTensor, "contact start indices matrix", __FUNCTION__) ||
        !checkTensorSizeExact(*contactStartIndicesTensor, getSensorCount() * getFilterCount(),
                              "contact start indices matrix", __FUNCTION__))
    {
        return false;
    }

    PxReal* dstForces = static_cast<PxReal*>(contactForceTensor->data);
    PxVec3* dstPoints = static_cast<PxVec3*>(contactPointTensor->data);
    PxVec3* dstNormals = static_cast<PxVec3*>(contactNormalTensor->data);
    PxReal* dstSeparations = static_cast<PxReal*>(contactSeparationTensor->data);
    PxU32* dstCounts = static_cast<PxU32*>(contactCountTensor->data);
    PxU32* dstStartIndices = static_cast<PxU32*>(contactStartIndicesTensor->data);

    // make sure we have the latest contact reports
    mGpuSimData->updateContactReports();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    CHECK_CUDA(cudaMemset(dstForces, 0, getMaxContactDataCount()* sizeof(PxReal)));
    CHECK_CUDA(cudaMemset(dstPoints, 0, getMaxContactDataCount() * sizeof(PxVec3)));
    CHECK_CUDA(cudaMemset(dstNormals, 0, getMaxContactDataCount()* sizeof(PxVec3)));
    CHECK_CUDA(cudaMemset(dstSeparations, 0, getMaxContactDataCount() * sizeof(PxReal)));
    CHECK_CUDA(cudaMemset(dstCounts, 0, getSensorCount() * getFilterCount() * sizeof(PxU32)));
    CHECK_CUDA(cudaMemset(dstStartIndices, 0, getSensorCount() * getFilterCount() * sizeof(PxU32)));
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    float timeStepInv = 1.0f / dt;

    if (!fetchRigidContactCount(dstCounts, mGpuSimData->mGpuContactPairsDev, mGpuSimData->mNumContactPairs, mNumFilters,
                                mGpuSimData->mMaxLinks, mGpuSimData->mNodeIdx2ArtiGpuIdxDev, mRdContactIndicesDev,
                                mLinkContactIndicesDev, mFilterLookupDev))
    {
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    exclusiveScan(dstCounts, dstStartIndices, getSensorCount() * getFilterCount());
    {
        PxU32 lastCount = 0;
        PxU32 lastStartIdx = 0;
        if (!CHECK_CUDA(cudaMemcpy(&lastCount, &dstCounts[getSensorCount() * getFilterCount() - 1], sizeof(PxU32),
                                   cudaMemcpyDeviceToHost)))
        {
            return false;
        }
        if (!CHECK_CUDA(cudaMemcpy(&lastStartIdx, &dstStartIndices[getSensorCount() * getFilterCount() - 1],
                                   sizeof(PxU32), cudaMemcpyDeviceToHost)))
        {
            return false;
        }
        if (lastStartIdx + lastCount > getMaxContactDataCount())
            CARB_LOG_WARN(
                "Incomplete contact data is reported in GpuRigidContactView::getContactData because there are more contact data points than specified maxContactDataCount = %u.",
                getMaxContactDataCount());
    }

    CHECK_CUDA(cudaMemset(dstCounts, 0, getSensorCount() * getFilterCount() * sizeof(PxU32)));
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    if (!fetchRigidContactData(dstForces, dstPoints, dstNormals, dstSeparations, dstCounts, dstStartIndices,
                               mGpuSimData->mGpuContactPairsDev, mGpuSimData->mNumContactPairs, mNumFilters,
                               getMaxContactDataCount(), mGpuSimData->mMaxLinks, timeStepInv, mGpuSimData->mNodeIdx2ArtiGpuIdxDev,
                               mRdContactIndicesDev, mLinkContactIndicesDev, mFilterLookupDev))
    {
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

bool GpuRigidContactView::getFrictionData(const TensorDesc* FrictionForceTensor,
                                          const TensorDesc* contactPointTensor,
                                          const TensorDesc* contactCountTensor,
                                          const TensorDesc* contactStartIndicesTensor,
                                          float dt) const
{
    if (!mGpuSimData)
    {
        return false;
    }

    GPUAPI_CHECK_READY(mGpuSimData, false);
    if (!FrictionForceTensor || !FrictionForceTensor->data || !contactPointTensor || !contactPointTensor->data ||
        !contactCountTensor || !contactCountTensor->data || !contactStartIndicesTensor ||
        !contactStartIndicesTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*FrictionForceTensor, mDevice, "friction force buffer", __FUNCTION__) ||
        !checkTensorFloat32(*FrictionForceTensor, "friction force buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*FrictionForceTensor, getMaxContactDataCount() * 3, "friction force buffer", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactPointTensor, mDevice, "contact point buffer", __FUNCTION__) ||
        !checkTensorFloat32(*contactPointTensor, "contact point buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactPointTensor, getMaxContactDataCount() * 3, "contact point buffer", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*contactCountTensor, mDevice, "contact count matrix", __FUNCTION__) ||
        !checkTensorInt32(*contactCountTensor, "contact count matrix", __FUNCTION__) ||
        !checkTensorSizeExact(
            *contactCountTensor, getSensorCount() * getFilterCount(), "contact count matrix", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactStartIndicesTensor, mDevice, "contact start indices matrix", __FUNCTION__) ||
        !checkTensorInt32(*contactStartIndicesTensor, "contact start indices matrix", __FUNCTION__) ||
        !checkTensorSizeExact(*contactStartIndicesTensor, getSensorCount() * getFilterCount(),
                              "contact start indices matrix", __FUNCTION__))
    {
        return false;
    }

    PxVec3* dstForces = static_cast<PxVec3*>(FrictionForceTensor->data);
    PxVec3* dstPoints = static_cast<PxVec3*>(contactPointTensor->data);
    PxU32* dstCounts = static_cast<PxU32*>(contactCountTensor->data);
    PxU32* dstStartIndices = static_cast<PxU32*>(contactStartIndicesTensor->data);

    // make sure we have the latest contact reports
    mGpuSimData->updateContactReports();

    PhysxCudaContextGuard ctxGuarg(mGpuSimData->mCudaContextManager);
    CHECK_CUDA(cudaMemset(dstForces, 0, getMaxContactDataCount()* sizeof(PxVec3)));
    CHECK_CUDA(cudaMemset(dstPoints, 0, getMaxContactDataCount() * sizeof(PxVec3)));
    CHECK_CUDA(cudaMemset(dstCounts, 0, getSensorCount() * getFilterCount() * sizeof(PxU32)));
    CHECK_CUDA(cudaMemset(dstStartIndices, 0, getSensorCount() * getFilterCount() * sizeof(PxU32)));
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    float timeStepInv = 1.0f / dt;

    if (!fetchFrictionCount(dstCounts, mGpuSimData->mGpuContactPairsDev, mGpuSimData->mNumContactPairs, mNumFilters,
                            mGpuSimData->mMaxLinks, mGpuSimData->mNodeIdx2ArtiGpuIdxDev, mRdContactIndicesDev,
                            mLinkContactIndicesDev, mFilterLookupDev))
    {
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    exclusiveScan(dstCounts, dstStartIndices, getSensorCount() * getFilterCount());
    {
        PxU32 lastCount = 0;
        PxU32 lastStartIdx = 0;
        if (!CHECK_CUDA(cudaMemcpy(&lastCount, &dstCounts[getSensorCount() * getFilterCount() - 1], sizeof(PxU32),
                                   cudaMemcpyDeviceToHost)))
        {
            return false;
        }
        if (!CHECK_CUDA(cudaMemcpy(&lastStartIdx, &dstStartIndices[getSensorCount() * getFilterCount() - 1],
                                   sizeof(PxU32), cudaMemcpyDeviceToHost)))
        {
            return false;
        }
        if (lastStartIdx + lastCount > getMaxContactDataCount())
            CARB_LOG_WARN(
                "Incomplete contact data is reported in GpuRigidContactView::getFrictionData because there are more contact data points than specified maxContactDataCount = %u.",
                getMaxContactDataCount());
    }

    CHECK_CUDA(cudaMemset(dstCounts, 0, getSensorCount() * getFilterCount() * sizeof(PxU32)));
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    if (!fetchRigidFrictionData(dstForces, dstPoints, dstCounts, dstStartIndices, mGpuSimData->mGpuContactPairsDev,
                                mGpuSimData->mNumContactPairs, mNumFilters, getMaxContactDataCount(),
                                mGpuSimData->mMaxLinks, timeStepInv, mGpuSimData->mNodeIdx2ArtiGpuIdxDev,
                                mRdContactIndicesDev, mLinkContactIndicesDev, mFilterLookupDev))
    {
        return false;
    }

    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    return true;
}

}
}
}

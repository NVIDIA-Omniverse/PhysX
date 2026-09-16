// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-TENSOR-CONTACT-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-TENSOR-ATTACH-001
 * @covers AC-1
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-40
 */

#include "tensors/gpu/GpuRigidContactView.h"
#include "tensors/gpu/GpuSimulationView.h"
#include "tensors/gpu/CudaKernels.h"
#include "usdLoad/AttachedStage.h"

#include "tensors/GlobalsAreBad.h"
#include "tensors/CommonTypes.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

#include <omni/physics/tensors/TensorUtils.h>

#include <unordered_map>
#include <utility>

using omni::physics::tensors::checkTensorDevice;
using omni::physics::tensors::checkTensorFloat32;
using omni::physics::tensors::checkTensorInt32;
using omni::physics::tensors::checkTensorInt64;
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


GpuRigidContactView::GpuRigidContactView(GpuSimulationView* sim,
                                         std::vector<RigidContactSensorEntry>&& entries,
                                         uint32_t numFilters,
                                         uint32_t maxContactDataCount,
                                         int device)
    : BaseRigidContactView(sim, std::move(entries), numFilters, maxContactDataCount), mDevice(device)
{
    mGpuSimData = sim->getGpuSimulationData();

    PxU32 numSensors = PxU32(mEntries.size());

    PxU32 linkBufSize = (mGpuSimData->mMaxArtiIndex + 1) * mGpuSimData->mMaxLinks;
    PxU32 rdBufSize = mGpuSimData->mMaxRdIndex + 1;

    std::vector<PxU32> linkContactIndices(linkBufSize, 0xffffffff);
    std::vector<PxU32> rdContactIndices(rdBufSize, 0xffffffff);

    std::vector<GpuRigidContactFilterIdPair> filterLookup(numSensors * numFilters);

    usdparser::AttachedStage* attachedStage = sim->getAttachedStage();
    std::unordered_map<omni::physics::parse::ObjectKey, PxActor*, omni::physics::parse::ObjectKey::Hash>
        resolvedFilterActors;
    resolvedFilterActors.reserve(numFilters);
    for (PxU32 i = 0; i < numSensors; i++)
    {
        auto& entry = mEntries[i];

        if (entry.link)
        {
            PxArticulationReducedCoordinate& arti = entry.link->getArticulation();
            PxArticulationGPUIndex artiIdx = arti.getGPUIndex();
            PxU32 linkIdx = entry.link->getLinkIndex();
            PxU32 globalLinkIdx = artiIdx * mGpuSimData->mMaxLinks + linkIdx;
            linkContactIndices[globalLinkIdx] = i;
        }
        else if (entry.rd)
        {
            PxU32 rdIdx = entry.rd->getInternalIslandNodeIndex().index();
            // A disabled rigid dynamic has no island node -- index 0xffffffff, the same sentinel that
            // no longer sizes rdBufSize above. It owns no contact row, and writing at the sentinel is a
            // ~16 GB out-of-bounds heap write. The buffer is pre-filled with 0xffffffff ("no sensor"),
            // so skipping the disabled body already leaves the correct downstream state.
            if (rdIdx != 0xffffffffu)
                rdContactIndices[rdIdx] = i;
        }
        else if (entry.shape)
        {
            // TODO!!! - handle shapes
            CARB_LOG_WARN("GPU contact info for collider '%s' is not supported", entry.path.c_str());
        }

        if (numFilters > 0)
        {
            // filter lookup table per referent
            GpuRigidContactFilterIdPair* filterIdPairs = filterLookup.data() + i * numFilters;
            PxU32 j = 0;
            for (auto& srcPair : entry.filterIndexMap)
            {
                const omni::physics::parse::ObjectKey filterKey = entry.filterKeys[srcPair.second];
                auto insertion = resolvedFilterActors.emplace(filterKey, nullptr);
                if (insertion.second)
                {
                    insertion.first->second = static_cast<PxActor*>(
                        BaseSimulationView::resolvePhysXPtr(attachedStage, filterKey, omni::physx::ePTActor));
                    if (!insertion.first->second)
                    {
                        insertion.first->second = static_cast<PxArticulationLink*>(
                            BaseSimulationView::resolvePhysXPtr(attachedStage, filterKey, omni::physx::ePTLink));
                    }
                }
                PxActor* actor = insertion.first->second;

                if (actor)
                {
                    filterIdPairs[j].actor = actor;
                    filterIdPairs[j].filterIndex = srcPair.second;
                    ++j;
                }
                else
                {
                    // TODO!!! - handle shapes
                    CARB_LOG_WARN("GPU contact filter for collider '%s' is not supported",
                                  entry.filterPaths[srcPair.second].c_str());
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

    if (numSensors > 0)
    {
        if (!CHECK_CUDA(cudaMalloc(&mRawLayoutScratchDev, 2 * numSensors * sizeof(PxU32))))
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
        CudaContextGuard ctxGuard(mGpuSimData->mCtx);

        CHECK_CUDA(cudaFree(mLinkContactIndicesDev));
        CHECK_CUDA(cudaFree(mRdContactIndicesDev));
        CHECK_CUDA(cudaFree(mFilterLookupDev));
        CHECK_CUDA(cudaFree(mRawLayoutScratchDev));
    }
}

bool GpuRigidContactView::getNetContactForces(const TensorDesc* dstTensor, float dt) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);

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
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);

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
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);

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
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);

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

bool GpuRigidContactView::getRawContactData(const TensorDesc* contactForceTensor,
                                            const TensorDesc* contactPointTensor,
                                            const TensorDesc* contactNormalTensor,
                                            const TensorDesc* contactSeparationTensor,
                                            const TensorDesc* sensorLayoutTensor,
                                            const TensorDesc* actorIdsTensor,
                                            float dt) const
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, mSim, false);

    GPUAPI_CHECK_READY(mGpuSimData, false);

    if (!contactForceTensor || !contactForceTensor->data || !contactPointTensor || !contactPointTensor->data ||
        !contactNormalTensor || !contactNormalTensor->data || !contactSeparationTensor ||
        !contactSeparationTensor->data || !sensorLayoutTensor || !sensorLayoutTensor->data ||
        !actorIdsTensor || !actorIdsTensor->data)
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
        !checkTensorSizeExact(*contactPointTensor, getMaxContactDataCount() * 3, "contact point buffer", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*contactNormalTensor, mDevice, "contact normal buffer", __FUNCTION__) ||
        !checkTensorFloat32(*contactNormalTensor, "contact normal buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactNormalTensor, getMaxContactDataCount() * 3, "contact normal buffer", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*contactSeparationTensor, mDevice, "contact separation buffer", __FUNCTION__) ||
        !checkTensorFloat32(*contactSeparationTensor, "contact separation buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactSeparationTensor, getMaxContactDataCount(), "contact separation buffer", __FUNCTION__))
    {
        return false;
    }

    // Per-sensor layout is (numSensors, 2): column 0 count, column 1 start index. No
    // filter dimension.
    if (!checkTensorDevice(*sensorLayoutTensor, mDevice, "sensor layout buffer", __FUNCTION__) ||
        !checkTensorInt32(*sensorLayoutTensor, "sensor layout buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*sensorLayoutTensor, getSensorCount() * 2, "sensor layout buffer", __FUNCTION__))
    {
        return false;
    }

    // Per-contact identities are (maxContactDataCount, 2): column 0 the reporting
    // sensor's actor, column 1 the actor it contacted.
    if (!checkTensorDevice(*actorIdsTensor, mDevice, "actor IDs buffer", __FUNCTION__) ||
        !checkTensorInt64(*actorIdsTensor, "actor IDs buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*actorIdsTensor, getMaxContactDataCount() * 2, "actor IDs buffer", __FUNCTION__))
    {
        return false;
    }

    PxReal* dstForces = static_cast<PxReal*>(contactForceTensor->data);
    PxVec3* dstPoints = static_cast<PxVec3*>(contactPointTensor->data);
    PxVec3* dstNormals = static_cast<PxVec3*>(contactNormalTensor->data);
    PxReal* dstSeparations = static_cast<PxReal*>(contactSeparationTensor->data);
    PxU32* dstSensorLayout = static_cast<PxU32*>(sensorLayoutTensor->data);
    uint64_t* dstActorIds = static_cast<uint64_t*>(actorIdsTensor->data);

    // Counts and starts stay contiguous in scratch for the kernels below; the caller's
    // interleaved tensor is filled once at the end.
    PxU32* dstCounts = mRawLayoutScratchDev;
    PxU32* dstStartIndices = mRawLayoutScratchDev ? mRawLayoutScratchDev + getSensorCount() : nullptr;

    mGpuSimData->updateContactReports();

    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);

    CHECK_CUDA(cudaMemset(dstForces, 0, getMaxContactDataCount() * sizeof(PxReal)));
    CHECK_CUDA(cudaMemset(dstPoints, 0, getMaxContactDataCount() * sizeof(PxVec3)));
    CHECK_CUDA(cudaMemset(dstNormals, 0, getMaxContactDataCount() * sizeof(PxVec3)));
    CHECK_CUDA(cudaMemset(dstSeparations, 0, getMaxContactDataCount() * sizeof(PxReal)));
    CHECK_CUDA(cudaMemset(dstSensorLayout, 0, getSensorCount() * 2 * sizeof(PxU32)));
    CHECK_CUDA(cudaMemset(dstActorIds, 0, getMaxContactDataCount() * 2 * sizeof(uint64_t)));
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    if (getSensorCount() == 0)
        return true;

    if (!mRawLayoutScratchDev)
        return false;

    // No sync after this memset: it and the count/scan/fill kernels below all run on the
    // default stream, which orders them. The scratch is device-only and never read by the
    // host, so there is nothing to wait for here.
    CHECK_CUDA(cudaMemset(mRawLayoutScratchDev, 0, 2 * getSensorCount() * sizeof(PxU32)));

    float timeStepInv = 1.0f / dt;

    // First pass: count contacts per sensor
    if (!fetchRawRigidContactCount(dstCounts, mGpuSimData->mGpuContactPairsDev, mGpuSimData->mNumContactPairs,
                                   mGpuSimData->mMaxLinks, mGpuSimData->mNodeIdx2ArtiGpuIdxDev, mRdContactIndicesDev,
                                   mLinkContactIndicesDev))
    {
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    exclusiveScan(dstCounts, dstStartIndices, getSensorCount());

    {
        PxU32 lastCount = 0;
        PxU32 lastStartIdx = 0;
        if (!CHECK_CUDA(cudaMemcpy(&lastCount, &dstCounts[getSensorCount() - 1], sizeof(PxU32),
                                   cudaMemcpyDeviceToHost)))
        {
            return false;
        }
        if (!CHECK_CUDA(cudaMemcpy(&lastStartIdx, &dstStartIndices[getSensorCount() - 1],
                                   sizeof(PxU32), cudaMemcpyDeviceToHost)))
        {
            return false;
        }
        if (lastStartIdx + lastCount > getMaxContactDataCount())
        {
            CARB_LOG_WARN(
                "Incomplete raw contact data in GpuRigidContactView::getRawContactData because there are more "
                "contact data points than specified maxContactDataCount = %u.",
                getMaxContactDataCount());
        }
    }

    // Reset counts for second pass (data kernel will atomically increment)
    CHECK_CUDA(cudaMemset(dstCounts, 0, getSensorCount() * sizeof(PxU32)));
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    // Second pass: fill contact data and both actor IDs
    if (!fetchRawRigidContactData(dstForces, dstPoints, dstNormals, dstSeparations, dstActorIds,
                                  dstCounts, dstStartIndices, mGpuSimData->mGpuContactPairsDev,
                                  mGpuSimData->mNumContactPairs, getMaxContactDataCount(), mGpuSimData->mMaxLinks,
                                  timeStepInv, mGpuSimData->mNodeIdx2ArtiGpuIdxDev, mRdContactIndicesDev,
                                  mLinkContactIndicesDev, mGpuSimData->mActorPathLookupDev,
                                  mGpuSimData->mNumActorPathPairs))
    {
        return false;
    }
    CHECK_CUDA(cudaStreamSynchronize(nullptr));

    // Correct per-sensor counts to reflect only what was actually written.
    if (!clampContactLayout(dstCounts, dstStartIndices, getSensorCount(), getMaxContactDataCount()))
        return false;
    if (!CHECK_CUDA(cudaStreamSynchronize(nullptr)))
        return false;

    // Interleave scratch into the caller's (numSensors, 2) layout tensor. These last two
    // steps are the ones that populate the caller's layout tensor, so their status is
    // returned rather than dropped: reporting success here with the sync having failed
    // would hand back the memset zeros as if the step genuinely had no contacts.
    if (!packSensorLayout(dstSensorLayout, dstCounts, dstStartIndices, getSensorCount()))
        return false;
    if (!CHECK_CUDA(cudaStreamSynchronize(nullptr)))
        return false;

    return true;
}

}
}
}

// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "VolumeDeformableBodyManager.h"
#include "FabricManager.h"

#include <cstdint>

// A safety net to ensure that kCuEventDisableTiming has the correct value
#if defined(__has_include)
#    if __has_include(<cuda.h>)
#        define OMNI_PHYSX_FABRIC_HAS_CUDA_H 1
#        include <cuda.h>
static_assert(CU_EVENT_DISABLE_TIMING == 0x02u, "Unexpected CU_EVENT_DISABLE_TIMING value");
#    endif
#endif

namespace
{
// CUDA Driver API: CU_EVENT_DISABLE_TIMING == 0x02.
constexpr unsigned int kCuEventDisableTiming = 0x02u;
} // namespace

#include <omni/fabric/FabricUSD.h>
#include <omni/physx/IPhysxFabric.h>
#include <omni/physx/PhysXConversions.h>

#include "extensions/PxCudaHelpersExt.h"

// There are 2 possible implementation for CPU fabric:
// 1. Compute the USD points on CPU fabric buffers
// 2. Use GPU kernels to compute the USD points and then doing device to host memory transfers to CPU fabric buffers
// (1) would work well for small number of deformable bodies and (2) could potentially be faster for large number of deformables.
// We can potentially use a hybrid approach in the future based on heuristics once we have sufficient profiling data
#define USE_GPU_CODE_PATH_FOR_CPU_FABRIC 0

using namespace PXR_NS;
using namespace physx;
using namespace physx::Ext;
using namespace carb;
using namespace omni::physx;

VolumeDeformableBodySet::VolumeDeformableBodySet()
{
    using omni::fabric::AttributeRole;
    using omni::fabric::BaseDataType;

    mPointsToken = omni::fabric::Token::createImmortal("points");
    mTypeFloat3Array = omni::fabric::Type(BaseDataType::eFloat, 3, 1, AttributeRole::ePosition);

    carb::settings::ISettings* iSettings = carb::getCachedInterface<carb::settings::ISettings>();
    mGPUInterop = iSettings->getAsBool(kSettingFabricUseGPUInterop);
}

VolumeDeformableBodySet::~VolumeDeformableBodySet()
{
    releaseBuffers();
}

void VolumeDeformableBodySet::releaseBuffers()
{
    PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

    if (!cudaContextManager)
        return;

    PxScopedCudaLock _lock(*cudaContextManager);

    for (size_t i = 0; i < mVolumeDeformableBodies.size(); ++i)
    {
        VolumeDeformableBody& volumeDeformableBody = *mVolumeDeformableBodies[i];

        {
            PXR_NS::SdfPath path = volumeDeformableBody.data.simMeshPrim.GetPath();
            omni::fabric::Path pathHandle = omni::fabric::convertToPathType<omni::fabric::Path>(mFabricId, path);

            if (volumeDeformableBody.stagingPointsDevMap[pathHandle])
                PxCudaHelpersExt::freeDeviceBuffer(*cudaContextManager, volumeDeformableBody.stagingPointsDevMap[pathHandle]);
        }

        if (volumeDeformableBody.hasCollMesh)
        {
            PXR_NS::SdfPath path = volumeDeformableBody.data.collMeshPrim.GetPath();
            omni::fabric::Path pathHandle = omni::fabric::convertToPathType<omni::fabric::Path>(mFabricId, path);

            if (volumeDeformableBody.stagingPointsDevMap[pathHandle])
                PxCudaHelpersExt::freeDeviceBuffer(*cudaContextManager, volumeDeformableBody.stagingPointsDevMap[pathHandle]);
        }

        for (size_t i = 0; i < volumeDeformableBody.skinMeshPrims.size(); ++i)
        {
            PXR_NS::SdfPath path = volumeDeformableBody.skinMeshPrims[i].GetPath();
            omni::fabric::Path pathHandle = omni::fabric::convertToPathType<omni::fabric::Path>(mFabricId, path);

            if (volumeDeformableBody.stagingPointsDevMap[pathHandle])
                PxCudaHelpersExt::freeDeviceBuffer(*cudaContextManager, volumeDeformableBody.stagingPointsDevMap[pathHandle]);
        }
    }

    if (mDeformableBodyGPUDataH)
        PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mDeformableBodyGPUDataH);

    if (mPointsCopyEvent)
        cudaContextManager->getCudaContext()->eventDestroy(mPointsCopyEvent);

    if (mDeformableBodyCopyStream)
        cudaContextManager->getCudaContext()->streamDestroy(mDeformableBodyCopyStream);
}

void VolumeDeformableBodySet::createFabricAttribute(omni::fabric::StageReaderWriter& srw, PxCudaContextManager* cudaContextManager, PXR_NS::SdfPath path, uint32_t numVertices, std::unordered_map<omni::fabric::Path, float3*>& stagingPointsDevMap)
{
    omni::fabric::Path pathHandle = omni::fabric::convertToPathType<omni::fabric::Path>(srw.getFabricId(), path);

    srw.createPrim(pathHandle);
    srw.createAttribute(pathHandle, mPointsToken, mTypeFloat3Array);
    srw.setArrayAttributeSize(pathHandle, mPointsToken, numVertices);

    mMaxPoints = mMaxPoints < numVertices ? numVertices : mMaxPoints;

    if (!mGPUInterop)
    {
        float3* stagingPoints = PxCudaHelpersExt::allocDeviceBuffer<float3>(*cudaContextManager, numVertices);
        stagingPointsDevMap[pathHandle] = stagingPoints;
    }
}

void VolumeDeformableBodySet::prepareBuffers(omni::fabric::StageReaderWriter& srw)
{
    mFabricId = srw.getFabricId();

    size_t numVolumeDeformableBodies = mVolumeDeformableBodies.size();
    if (numVolumeDeformableBodies)
    {
        PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

        if (!cudaContextManager)
            return;

        PxScopedCudaLock _lock(*cudaContextManager);

        releaseBuffers();

        // Calculate the number of fabric attributes to create
        mNumFabricAttributes = 0;
        for (VolumeDeformableBody* const it : mVolumeDeformableBodies)
        {
            // simMeshPrim + skinMeshPrims + collMeshPrim
            mNumFabricAttributes += 1 + (uint32_t)it->data.skinMeshPrims.size() + it->hasCollMesh;
        }

        mDeformableBodyGPUDataH = PxCudaHelpersExt::allocPinnedHostBuffer<DeformableBodyGPUData>(*cudaContextManager, (PxU32)mNumFabricAttributes);
        if (!mDeformableBodyGPUDataH)
            return;

        // Synchronization event
        cudaContextManager->getCudaContext()->streamCreate(&mDeformableBodyCopyStream, 0x1);
        cudaContextManager->getCudaContext()->eventCreate(&mPointsCopyEvent, kCuEventDisableTiming);

        mMaxPoints = 0;
        for (VolumeDeformableBody* const it : mVolumeDeformableBodies)
        {
            {
                createFabricAttribute(srw, cudaContextManager, it->data.simMeshPrim.GetPath(), it->data.numSimMeshVertices, it->stagingPointsDevMap);
            }

            if (it->hasCollMesh)
            {
                createFabricAttribute(srw, cudaContextManager, it->data.collMeshPrim.GetPath(), it->data.numCollMeshVertices, it->stagingPointsDevMap);
            }

            for (size_t i = 0; i < it->data.skinMeshPrims.size(); ++i)
            {
                createFabricAttribute(srw, cudaContextManager, it->data.skinMeshPrims[i].GetPath(), it->data.skinMeshRanges[i].y, it->stagingPointsDevMap);
            }
        }
    }
}

void VolumeDeformableBodySet::updateInternalVolumeDeformableBodyData()
{
    PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

    if (!cudaContextManager)
        return;

    PxScopedCudaLock _lock(*cudaContextManager);

    cudaContextManager->getCudaContext()->eventRecord(mPointsCopyEvent, mDeformableBodyCopyStream);

    for (VolumeDeformableBody* const it : mVolumeDeformableBodies)
    {
        if (!it->physXPtr)
        {
            CARB_LOG_WARN_ONCE("VolumeDeformableBodySet::updateInternalVolumeDeformableBodyData: physXPtr is null, skipping.");
            continue;
        }

        if (!it->physXPtr->isSleeping())
        {
            if (!mGPUInterop)
            {
#if !USE_GPU_CODE_PATH_FOR_CPU_FABRIC
                // Update simMeshPositionInvMassH if GPU code path is not used for CPU fabric
                PxCudaHelpersExt::copyDToHAsync(*cudaContextManager, it->data.simMeshPositionInvMassH, it->physXPtr->getSimPositionInvMassBufferD(), it->data.numSimMeshVertices, mDeformableBodyCopyStream);

                // Update allSkinnedVerticesH
                if (it->data.numSkinMeshVertices > 0)
                    PxCudaHelpersExt::copyDToHAsync(*cudaContextManager, it->data.allSkinnedVerticesH, it->data.allSkinnedVerticesD, it->data.numSkinMeshVertices, mDeformableBodyCopyStream);
#endif
            }

            // Update collMeshPositionInvMassH so that picking can work correctly 
            PxCudaHelpersExt::copyDToHAsync(*cudaContextManager, it->data.collMeshPositionInvMassH, it->physXPtr->getPositionInvMassBufferD(), it->data.numCollMeshVertices, mDeformableBodyCopyStream);
        }
    }
}

bool VolumeDeformableBodySet::prepareInteropData(omni::fabric::StageReaderWriter& srw, PXR_NS::SdfPath path, DeformableBodyGPUData& gpuData,
                                                 void* src, uint32_t numVertices, PXR_NS::GfMatrix4f worldToDeformableSurface,
                                                 std::unordered_map<omni::fabric::Path, float3*>& stagingPointsDevMap, std::unordered_map<omni::fabric::Path, float3*>& fabricPointsCPUMap, const int srcPointsElemSize)
{
    omni::fabric::Path pathHandle = omni::fabric::convertToPathType<omni::fabric::Path>(srw.getFabricId(), path);

    if (mGPUInterop)
    {
        float3** cudaPtr = srw.getAttributeWrGpu<float3*>(pathHandle, mPointsToken);
        if (cudaPtr && *cudaPtr)
        {
            gpuData.dstPoints = *cudaPtr;
        }
        else
        {
            return false;
        }
    }
    else
    {
        float3** cpuPtr = srw.getAttributeWr<float3*>(pathHandle, mPointsToken);
        gpuData.dstPoints = stagingPointsDevMap[pathHandle];
        if (cpuPtr && *cpuPtr)
        {         
            fabricPointsCPUMap[pathHandle] = *cpuPtr;
        }
        else
        {
            return false;
        }
    }

    float4* worldToDeformableFloat4 = reinterpret_cast<float4*>(&worldToDeformableSurface);
    gpuData.transform[0] = worldToDeformableFloat4[0];
    gpuData.transform[1] = worldToDeformableFloat4[1];
    gpuData.transform[2] = worldToDeformableFloat4[2];
    gpuData.transform[3] = worldToDeformableFloat4[3];

    gpuData.srcPoints = src;
    gpuData.nbPoints = (int)numVertices;

    gpuData.srcPointsElemSize = srcPointsElemSize;

    return true;
}

void VolumeDeformableBodySet::updateDeformableBodies(omni::fabric::StageReaderWriter& srw)
{
    PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

    if (!cudaContextManager)
        return;

    PxScopedCudaLock _lock(*cudaContextManager);
    PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

    size_t fabricAttributeCount = 0;
    for (PxU32 i = 0; i < mVolumeDeformableBodies.size(); ++i)
    {
        VolumeDeformableBody* hostData = mVolumeDeformableBodies[i];

        if (!hostData->physXPtr)
        {
            CARB_LOG_WARN_ONCE("VolumeDeformableBodySet::updateDeformableBodies: physXPtr is null, skipping.");
            continue;
        }

        {
            DeformableBodyGPUData& gpuData = mDeformableBodyGPUDataH[fabricAttributeCount++];

            if (!prepareInteropData
            (
                srw, hostData->data.simMeshPrim.GetPath(), gpuData,
                hostData->physXPtr->getSimPositionInvMassBufferD(), hostData->data.numSimMeshVertices, hostData->data.worldToSimMesh,
                hostData->stagingPointsDevMap, hostData->fabricPointsCPUMap, 4
            ))
            {
                CARB_LOG_ERROR("prepareInteropData in updateDeformableBodies failed!");
                return;
            }
        }

        if (hostData->hasCollMesh)
        {
            DeformableBodyGPUData& gpuData = mDeformableBodyGPUDataH[fabricAttributeCount++];

            if (!prepareInteropData
            (
                srw, hostData->data.collMeshPrim.GetPath(), gpuData,
                hostData->physXPtr->getPositionInvMassBufferD(), hostData->data.numCollMeshVertices, hostData->data.worldToCollMesh,
                hostData->stagingPointsDevMap, hostData->fabricPointsCPUMap, 4
            ))
            {
                CARB_LOG_ERROR("prepareInteropData in updateDeformableBodies failed!");
                return;
            }
        }

        for (size_t i = 0; i < hostData->data.skinMeshPrims.size(); ++i)
        {
            DeformableBodyGPUData& gpuData = mDeformableBodyGPUDataH[fabricAttributeCount++];

            if (!prepareInteropData
            (
                srw, hostData->data.skinMeshPrims[i].GetPath(), gpuData,
                hostData->data.allSkinnedVerticesD + hostData->data.skinMeshRanges[i].x, hostData->data.skinMeshRanges[i].y, hostData->data.worldToSkinMeshTransforms[i],
                hostData->stagingPointsDevMap, hostData->fabricPointsCPUMap, 3
            ))
            {
                CARB_LOG_ERROR("prepareInteropData in updateDeformableBodies failed!");
                return;
            }
        }
    }

    CARB_ASSERT(fabricAttributeCount == mNumFabricAttributes);

#if USE_GPU_CODE_PATH_FOR_CPU_FABRIC
    convertToVec3fBlockDeformableBody(mDeformableBodyGPUDataH,
                                      (int)fabricAttributeCount,
                                      mMaxPoints,
                                      omni::physx::FabricCudaStreamHandle{ reinterpret_cast<uintptr_t>(mDeformableBodyCopyStream) });

    if (!mGPUInterop)
    {
        for (PxU32 i = 0; i < mVolumeDeformableBodies.size(); ++i)
        {
            VolumeDeformableBody* hostData = mVolumeDeformableBodies[i];

            {
                PXR_NS::SdfPath path = hostData->data.simMeshPrim.GetPath();
                omni::fabric::PathC pathHandle = omni::fabric::asInt(path);
                uint32_t numVertices = hostData->data.numSimMeshVertices;

                PxCudaHelpersExt::copyDToHAsync(*cudaContextManager, hostData->fabricPointsCPUMap[pathHandle], hostData->stagingPointsDevMap[pathHandle], numVertices, mDeformableBodyCopyStream);
            }

            if (hostData->hasCollMesh)
            {
                PXR_NS::SdfPath path = hostData->data.collMeshPrim.GetPath();
                omni::fabric::PathC pathHandle = omni::fabric::asInt(path);
                uint32_t numVertices = hostData->data.numCollMeshVertices;

                PxCudaHelpersExt::copyDToHAsync(*cudaContextManager, hostData->fabricPointsCPUMap[pathHandle], hostData->stagingPointsDevMap[pathHandle], numVertices, mDeformableBodyCopyStream);
            }

            for (size_t i = 0; i < hostData->data.skinMeshPrims.size(); ++i)
            {
                PXR_NS::SdfPath path = hostData->data.skinMeshPrims[i].GetPath();
                omni::fabric::PathC pathHandle = omni::fabric::asInt(path);
                uint32_t numVertices = hostData->data.skinMeshRanges[i].y;

                PxCudaHelpersExt::copyDToHAsync(*cudaContextManager, hostData->fabricPointsCPUMap[pathHandle], hostData->stagingPointsDevMap[pathHandle], numVertices, mDeformableBodyCopyStream);
            }
        }
    }

    cudaContextManager->getCudaContext()->streamWaitEvent(nullptr, mPointsCopyEvent, 0);
#else
    if (mGPUInterop)
    {
        convertToVec3fBlockDeformableBody(mDeformableBodyGPUDataH,
                                          (int)fabricAttributeCount,
                                          mMaxPoints,
                                          omni::physx::FabricCudaStreamHandle{ reinterpret_cast<uintptr_t>(mDeformableBodyCopyStream) });
    }

    cudaContextManager->getCudaContext()->streamWaitEvent(nullptr, mPointsCopyEvent, 0);

    if (!mGPUInterop)
    {
        for (PxU32 i = 0; i < mVolumeDeformableBodies.size(); ++i)
        {
            VolumeDeformableBody* hostData = mVolumeDeformableBodies[i];

            {
                PXR_NS::SdfPath path = hostData->data.simMeshPrim.GetPath();
                omni::fabric::Path pathHandle = omni::fabric::convertToPathType<omni::fabric::Path>(srw.getFabricId(), path);

                copyBuffer(hostData->fabricPointsCPUMap[pathHandle], hostData->data.simMeshPositionInvMassH, (unsigned int)hostData->data.numSimMeshVertices, hostData->data.worldToSimMesh);
            }

            if (hostData->hasCollMesh)
            {
                PXR_NS::SdfPath path = hostData->data.collMeshPrim.GetPath();
                omni::fabric::Path pathHandle = omni::fabric::convertToPathType<omni::fabric::Path>(srw.getFabricId(), path);

                copyBuffer(hostData->fabricPointsCPUMap[pathHandle], hostData->data.collMeshPositionInvMassH, (unsigned int)hostData->data.numCollMeshVertices, hostData->data.worldToCollMesh);
            }

            for (size_t i = 0; i < hostData->data.skinMeshPrims.size(); ++i)
            {
                PXR_NS::SdfPath path = hostData->data.skinMeshPrims[i].GetPath();
                omni::fabric::Path pathHandle = omni::fabric::convertToPathType<omni::fabric::Path>(srw.getFabricId(), path);

                copyBuffer(hostData->fabricPointsCPUMap[pathHandle], hostData->data.allSkinnedVerticesH + hostData->data.skinMeshRanges[i].x, (unsigned int)hostData->data.skinMeshRanges[i].y, hostData->data.worldToSkinMeshTransforms[i]);
            }
        }
    }
#endif
}

VolumeDeformableBodyManager::VolumeDeformableBodyManager()
{
    using omni::fabric::AttributeRole;
    using omni::fabric::BaseDataType;

    mTypeAppliedSchema = omni::fabric::Type(BaseDataType::eTag, 1, 0, AttributeRole::eAppliedSchema);

    mTypeFloat3 = omni::fabric::Type(BaseDataType::eFloat, 3, 0, AttributeRole::eNone);
    mTypeFloat3Array = omni::fabric::Type(BaseDataType::eFloat, 3, 1, AttributeRole::ePosition);
    
    mPointsToken = omni::fabric::Token::createImmortal("points");
}

VolumeDeformableBodyManager::~VolumeDeformableBodyManager()
{
    clear();
}

void VolumeDeformableBodyManager::registerDeformableBody(PXR_NS::UsdGeomXformCache& xfCache, uint64_t stageId, omni::fabric::IStageReaderWriter* iStageReaderWriter, omni::fabric::StageReaderWriterId stageInProgress, const PXR_NS::UsdPrim& prim)
{
    PXR_NS::SdfPath path = prim.GetPath();
    omni::fabric::Path pathHandle = omni::fabric::convertToPathType<omni::fabric::Path>(iStageReaderWriter->getFabricId(stageInProgress), path);

    // Check if deformable body is a valid volume deformable
    IPhysx* iPhysX = carb::getCachedInterface<IPhysx>();
    PxDeformableVolume* deformableVolume = static_cast<PxDeformableVolume*>(iPhysX->getPhysXPtr(path, omni::physx::ePTDeformableVolume));
    if (deformableVolume == nullptr)
    {
        return;
    }

    IPhysxPrivate* iPhysXPrivate = carb::getCachedInterface<IPhysxPrivate>();

    VolumeDeformableBody& deformableBody = mVolumeDeformableBodies[pathHandle];
    InternalVolumeDeformableBodyData& data = deformableBody.data;
    iPhysXPrivate->getInternalVolumeDeformableBodyData((const usdparser::ObjectId)deformableVolume->userData, data);

    VtArray<GfVec3f> simMeshPoints;
    VtArray<GfVec3f> collMeshPoints;
    VtArray<GfVec3f> allSkinMeshPoints;

    UsdGeomPointBased simMesh(data.simMeshPrim);
    simMesh.GetPointsAttr().Get(&simMeshPoints);

    // add tag for hydra to pick this prim up
    constexpr omni::fabric::Type primTypeType(omni::fabric::BaseDataType::eTag, 1, 0, omni::fabric::AttributeRole::ePrimTypeName);
    const omni::fabric::Token primType("Deformable");
    omni::fabric::Path simMeshPathHandle = omni::fabric::convertToPathType<omni::fabric::Path>(
        iStageReaderWriter->getFabricId(stageInProgress), data.simMeshPrim.GetPath());
    iStageReaderWriter->createAttribute(stageInProgress, simMeshPathHandle, primType, omni::fabric::TypeC(primTypeType));

    PositionCache::const_iterator fit = mInitialPositions.find(simMeshPathHandle);
    if (fit == mInitialPositions.end())
    {
        mInitialPositions[simMeshPathHandle] = simMeshPoints;
    }

    UsdGeomPointBased collMesh(data.collMeshPrim);
    if (collMesh)
    {
        if (collMesh.GetPrim() != simMesh.GetPrim())
        {
            collMesh.GetPointsAttr().Get(&collMeshPoints);

            // add tag for hydra to pick this prim up
            omni::fabric::Path collMeshPathHandle = omni::fabric::convertToPathType<omni::fabric::Path>(
                iStageReaderWriter->getFabricId(stageInProgress), data.collMeshPrim.GetPath());
            iStageReaderWriter->createAttribute(
                stageInProgress, collMeshPathHandle, primType, omni::fabric::TypeC(primTypeType));

            PositionCache::const_iterator fit = mInitialPositions.find(collMeshPathHandle);
            if (fit == mInitialPositions.end())
            {
                mInitialPositions[collMeshPathHandle] = collMeshPoints;
            }

            deformableBody.hasCollMesh = true;
        }
    }

    allSkinMeshPoints.reserve(data.numSkinMeshVertices);
    for (size_t i = 0; i < data.skinMeshPrims.size(); ++i)
    {
        UsdGeomPointBased skinMesh(data.skinMeshPrims[i]);
        if (skinMesh)
        {
            PXR_NS::VtArray<PXR_NS::GfVec3f> points;
            skinMesh.GetPointsAttr().Get(&points);
            for (const PXR_NS::GfVec3f& point : points)
            {
                allSkinMeshPoints.push_back(point);
            }

            // add tag for hydra to pick this prim up
            omni::fabric::Path skinMeshPathHandle = omni::fabric::convertToPathType<omni::fabric::Path>(
                iStageReaderWriter->getFabricId(stageInProgress), data.skinMeshPrims[i].GetPath());
            iStageReaderWriter->createAttribute(stageInProgress, skinMeshPathHandle, primType, omni::fabric::TypeC(primTypeType));

            PositionCache::const_iterator fit = mInitialPositions.find(skinMeshPathHandle);
            if (fit == mInitialPositions.end())
            {
                mInitialPositions[skinMeshPathHandle] = points;
            }
        }
    }

    // data.skinMeshPrims points to the internal deformable body array which can be destroyed prior to fabric cleanup is called. Make a hard copy of data.skinMeshPrims.
    deformableBody.skinMeshPrims.assign(data.skinMeshPrims.begin(), data.skinMeshPrims.end());

    mIsDirty = true;
}

bool VolumeDeformableBodyManager::prepareBuffers(omni::fabric::StageReaderWriter& srw)
{
    IPhysx* iPhysX = carb::getCachedInterface<IPhysx>();

    if (mIsDirty)
    {
        mVolumeDeformableBodySets.clear();

        std::vector<omni::fabric::Path> emptyPaths;

        for (auto& it : mVolumeDeformableBodies)
        {
            const SdfPath path = omni::fabric::toSdfPath(it.first);

            PxDeformableVolume* deformableVolume = static_cast<PxDeformableVolume*>(iPhysX->getPhysXPtr(path, omni::physx::ePTDeformableVolume));
            if (deformableVolume)
            {
                it.second.physXPtr = deformableVolume;

                PxScene* scene = deformableVolume->getScene();
                if (scene)
                {
                    VolumeDeformableBodySet& set = mVolumeDeformableBodySets[scene];

                    set.mScene = scene;
                    set.mVolumeDeformableBodies.push_back(&it.second);
                }
            }
            else
                emptyPaths.push_back(it.first);
        }

        for (auto& it : emptyPaths)
        {
            mVolumeDeformableBodies.erase(it);
        }

        for (auto& it : mVolumeDeformableBodySets)
        {
            it.second.prepareBuffers(srw);
        }

        mIsDirty = false;
    }

    return true;
}

void VolumeDeformableBodyManager::clear()
{
    mVolumeDeformableBodySets.clear();
    mVolumeDeformableBodies.clear();
    mInitialPositions.clear();
}

void VolumeDeformableBodyManager::update(omni::fabric::StageReaderWriter& srw)
{
    // get current settings
    carb::settings::ISettings* iSettings = carb::getCachedInterface<carb::settings::ISettings>();
           
    bool updatePoints = iSettings->getAsBool(kSettingFabricUpdatePoints);
    if (updatePoints)
    {
        if (!prepareBuffers(srw))
        {
            return;
        }

        updateVolumeDeformableBodies(srw);
    }
}

void VolumeDeformableBodyManager::updateVolumeDeformableBodies(omni::fabric::StageReaderWriter& srw)
{
    for (auto& it : mVolumeDeformableBodySets)
    {
        if (it.second.mVolumeDeformableBodies.size() > 0)
        {
            it.second.updateInternalVolumeDeformableBodyData();
            it.second.updateDeformableBodies(srw);
        }
    }
}

void VolumeDeformableBodyManager::setInitialTransformation(omni::fabric::StageReaderWriter& stage)
{
    for (const auto& initial : mInitialPositions)
    {
        const omni::fabric::Path pathHandle = initial.first;
        // The prim may have been removed from Fabric between registration and stop()
        // (FabricManager::removePrim does not evict from mInitialPositions). Skip
        // dead entries — getArrayAttributeWr<GfVec3f> on a missing attribute trips
        // a Fabric type-compatibility validation that calls std::terminate.
        if (!stage.attributeExists(pathHandle, mPointsToken))
            continue;
        const PXR_NS::VtArray<PXR_NS::GfVec3f>& src = initial.second;
        // Use the typed write API instead of setArrayAttribute(PXR_NS::VtValue): Kit Fabric
        // stores the VtValue by value in its ImportedDataSource, and the VtValue's _info
        // pointer aims at this DLL's per-DLL _TypeInfoImpl<VtArray<GfVec3f>> static. On
        // omni.physx.fabric.plugin unload that pointer dangles, and the next USD->Fabric
        // flush crashes on the planted VtValue's destructor.
        stage.setArrayAttributeSize(pathHandle, mPointsToken, src.size());
        auto dst = stage.getArrayAttributeWr<PXR_NS::GfVec3f>(pathHandle, mPointsToken);
        std::copy(src.cbegin(), src.cend(), dst.begin());
    }

    clear();
}

void VolumeDeformableBodyManager::saveToUsd(omni::fabric::StageReaderWriter& stage, PXR_NS::UsdStageRefPtr& usdStage)
{
    omni::fabric::StageReaderWriterUsd stageUsd{ stage.getId() };
    for (const auto& initial : mInitialPositions)
    {
        const omni::fabric::Path pathHandle = initial.first;
        PXR_NS::VtValue value = stageUsd.getArrayAttributeRd(pathHandle, mPointsToken);
        if (!value.IsEmpty())
        {
            PXR_NS::VtArray<PXR_NS::GfVec3f> points = value.Get<PXR_NS::VtArray<PXR_NS::GfVec3f>>();
            const size_t size = points.size();
            if (size == initial.second.size())
            {
                const PXR_NS::SdfPath primPath = omni::fabric::toSdfPath(pathHandle);
                UsdPrim prim = usdStage->GetPrimAtPath(primPath);
                UsdAttribute pointsAttr = prim.GetAttribute(UsdGeomTokens->points);
                pointsAttr.Set(points);
            }
        }
    }

    clear();
}

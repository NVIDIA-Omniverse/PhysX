// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "VolumeDeformableBodyManager.h"
#include "FabricManager.h"

#if !CARB_AARCH64

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

using namespace pxr;
using namespace physx;
using namespace physx::Ext;
using namespace carb;
using namespace omni::physx;

VolumeDeformableBodySet::VolumeDeformableBodySet()
{
    using omni::fabric::AttributeRole;
    using omni::fabric::BaseDataType;

    omni::fabric::IToken* iToken = carb::getCachedInterface<omni::fabric::IToken>();

    mPointsToken = iToken->getHandle("points");
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
            pxr::SdfPath path = volumeDeformableBody.data.simMeshPrim.GetPath();
            omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

            if (volumeDeformableBody.stagingPointsDevMap[pathHandle])
                PxCudaHelpersExt::freeDeviceBuffer(*cudaContextManager, volumeDeformableBody.stagingPointsDevMap[pathHandle]);
        }

        if (volumeDeformableBody.hasCollMesh)
        {
            pxr::SdfPath path = volumeDeformableBody.data.collMeshPrim.GetPath();
            omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

            if (volumeDeformableBody.stagingPointsDevMap[pathHandle])
                PxCudaHelpersExt::freeDeviceBuffer(*cudaContextManager, volumeDeformableBody.stagingPointsDevMap[pathHandle]);
        }

        for (size_t i = 0; i < volumeDeformableBody.skinMeshPrims.size(); ++i)
        {
            pxr::SdfPath path = volumeDeformableBody.skinMeshPrims[i].GetPath();
            omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

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

void VolumeDeformableBodySet::createFabricAttribute(omni::fabric::StageReaderWriter& srw, PxCudaContextManager* cudaContextManager, pxr::SdfPath path, uint32_t numVertices, std::unordered_map<omni::fabric::PathC, float3*>& stagingPointsDevMap)
{
    omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

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
        cudaContextManager->getCudaContext()->eventCreate(&mPointsCopyEvent, CU_EVENT_DISABLE_TIMING);

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
        if (!it->physXPtr->isSleeping())
        {
            if (!mGPUInterop)
            {
#if !USE_GPU_CODE_PATH_FOR_CPU_FABRIC
                // Update simMeshPositionInvMassH if GPU code path is not used for CPU fabric
                PxCudaHelpersExt::copyDToHAsync(*cudaContextManager, it->data.simMeshPositionInvMassH, it->physXPtr->getSimPositionInvMassBufferD(), it->data.numSimMeshVertices, mDeformableBodyCopyStream);

                // Update allSkinnedVerticesH
                PxCudaHelpersExt::copyDToHAsync(*cudaContextManager, it->data.allSkinnedVerticesH, it->data.allSkinnedVerticesD, it->data.numSkinMeshVertices, mDeformableBodyCopyStream);
#endif
            }

            // Update collMeshPositionInvMassH so that picking can work correctly 
            PxCudaHelpersExt::copyDToHAsync(*cudaContextManager, it->data.collMeshPositionInvMassH, it->physXPtr->getPositionInvMassBufferD(), it->data.numCollMeshVertices, mDeformableBodyCopyStream);
        }
    }
}

void VolumeDeformableBodySet::prepareInteropData(omni::fabric::StageReaderWriter& srw, pxr::SdfPath path, DeformableBodyGPUData& gpuData,
                                                 void* src, uint32_t numVertices, pxr::GfMatrix4f worldToDeformableSurface,
                                                 std::unordered_map<omni::fabric::PathC, float3*>& stagingPointsDevMap, std::unordered_map<omni::fabric::PathC, float3*>& fabricPointsCPUMap, const int srcPointsElemSize)
{
    omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

    if (mGPUInterop)
    {
        float3** cudaPtr = srw.getAttributeWrGpu<float3*>(pathHandle, mPointsToken);
        gpuData.dstPoints = *cudaPtr;
    }
    else
    {
        float3** cpuPtr = srw.getAttributeWr<float3*>(pathHandle, mPointsToken);
        gpuData.dstPoints = stagingPointsDevMap[pathHandle];
        fabricPointsCPUMap[pathHandle] = *cpuPtr;
    }

    float4* worldToDeformableFloat4 = reinterpret_cast<float4*>(&worldToDeformableSurface);
    gpuData.transform[0] = worldToDeformableFloat4[0];
    gpuData.transform[1] = worldToDeformableFloat4[1];
    gpuData.transform[2] = worldToDeformableFloat4[2];
    gpuData.transform[3] = worldToDeformableFloat4[3];

    gpuData.srcPoints = src;
    gpuData.nbPoints = (int)numVertices;

    gpuData.srcPointsElemSize = srcPointsElemSize;
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

        {
            DeformableBodyGPUData& gpuData = mDeformableBodyGPUDataH[fabricAttributeCount++];

            prepareInteropData
            (
                srw, hostData->data.simMeshPrim.GetPath(), gpuData,
                hostData->physXPtr->getSimPositionInvMassBufferD(), hostData->data.numSimMeshVertices, hostData->data.worldToSimMesh,
                hostData->stagingPointsDevMap, hostData->fabricPointsCPUMap, 4
            );
        }

        if (hostData->hasCollMesh)
        {
            DeformableBodyGPUData& gpuData = mDeformableBodyGPUDataH[fabricAttributeCount++];

            prepareInteropData
            (
                srw, hostData->data.collMeshPrim.GetPath(), gpuData,
                hostData->physXPtr->getPositionInvMassBufferD(), hostData->data.numCollMeshVertices, hostData->data.worldToCollMesh,
                hostData->stagingPointsDevMap, hostData->fabricPointsCPUMap, 4
            );
        }

        for (size_t i = 0; i < hostData->data.skinMeshPrims.size(); ++i)
        {
            DeformableBodyGPUData& gpuData = mDeformableBodyGPUDataH[fabricAttributeCount++];

            prepareInteropData
            (
                srw, hostData->data.skinMeshPrims[i].GetPath(), gpuData,
                hostData->data.allSkinnedVerticesD + hostData->data.skinMeshRanges[i].x, hostData->data.skinMeshRanges[i].y, hostData->data.worldToSkinMeshTransforms[i],
                hostData->stagingPointsDevMap, hostData->fabricPointsCPUMap, 3
            );
        }
    }

    CARB_ASSERT(fabricAttributeCount == mNumFabricAttributes);

#if USE_GPU_CODE_PATH_FOR_CPU_FABRIC
    convertToVec3fBlockDeformableBody(mDeformableBodyGPUDataH, (int)fabricAttributeCount, mMaxPoints, mDeformableBodyCopyStream);

    if (!mGPUInterop)
    {
        for (PxU32 i = 0; i < mVolumeDeformableBodies.size(); ++i)
        {
            VolumeDeformableBody* hostData = mVolumeDeformableBodies[i];

            {
                pxr::SdfPath path = hostData->data.simMeshPrim.GetPath();
                omni::fabric::PathC pathHandle = omni::fabric::asInt(path);
                uint32_t numVertices = hostData->data.numSimMeshVertices;

                PxCudaHelpersExt::copyDToHAsync(*cudaContextManager, hostData->fabricPointsCPUMap[pathHandle], hostData->stagingPointsDevMap[pathHandle], numVertices, mDeformableBodyCopyStream);
            }

            if (hostData->hasCollMesh)
            {
                pxr::SdfPath path = hostData->data.collMeshPrim.GetPath();
                omni::fabric::PathC pathHandle = omni::fabric::asInt(path);
                uint32_t numVertices = hostData->data.numCollMeshVertices;

                PxCudaHelpersExt::copyDToHAsync(*cudaContextManager, hostData->fabricPointsCPUMap[pathHandle], hostData->stagingPointsDevMap[pathHandle], numVertices, mDeformableBodyCopyStream);
            }

            for (size_t i = 0; i < hostData->data.skinMeshPrims.size(); ++i)
            {
                pxr::SdfPath path = hostData->data.skinMeshPrims[i].GetPath();
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
        convertToVec3fBlockDeformableBody(mDeformableBodyGPUDataH, (int)fabricAttributeCount, mMaxPoints, mDeformableBodyCopyStream);
    }

    cudaContextManager->getCudaContext()->streamWaitEvent(nullptr, mPointsCopyEvent, 0);

    if (!mGPUInterop)
    {
        for (PxU32 i = 0; i < mVolumeDeformableBodies.size(); ++i)
        {
            VolumeDeformableBody* hostData = mVolumeDeformableBodies[i];

            {
                pxr::SdfPath path = hostData->data.simMeshPrim.GetPath();
                omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

                copyBuffer(hostData->fabricPointsCPUMap[pathHandle], hostData->data.simMeshPositionInvMassH, (unsigned int)hostData->data.numSimMeshVertices, hostData->data.worldToSimMesh);
            }

            if (hostData->hasCollMesh)
            {
                pxr::SdfPath path = hostData->data.collMeshPrim.GetPath();
                omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

                copyBuffer(hostData->fabricPointsCPUMap[pathHandle], hostData->data.collMeshPositionInvMassH, (unsigned int)hostData->data.numCollMeshVertices, hostData->data.worldToCollMesh);
            }

            for (size_t i = 0; i < hostData->data.skinMeshPrims.size(); ++i)
            {
                pxr::SdfPath path = hostData->data.skinMeshPrims[i].GetPath();
                omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

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

    omni::fabric::IToken* iToken = carb::getCachedInterface<omni::fabric::IToken>();
    mPointsToken = iToken->getHandle("points");
}

VolumeDeformableBodyManager::~VolumeDeformableBodyManager()
{
    clear();
}

void VolumeDeformableBodyManager::registerDeformableBody(pxr::UsdGeomXformCache& xfCache, uint64_t stageId, omni::fabric::IStageReaderWriter* iStageReaderWriter, omni::fabric::StageReaderWriterId stageInProgress, const pxr::UsdPrim& prim)
{
    pxr::SdfPath path = prim.GetPath();
    omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

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
    omni::fabric::PathC simMeshPathHandle = omni::fabric::asInt(data.simMeshPrim.GetPath());
    iStageReaderWriter->createAttribute(stageInProgress, simMeshPathHandle, primType, omni::fabric::TypeC(primTypeType));

    PositionCache::const_iterator fit = mInitialPositions.find(simMeshPathHandle.path);
    if (fit == mInitialPositions.end())
    {
        pxr::VtArray<carb::Float3> simMeshPositions;
        copyBuffer(simMeshPositions, simMeshPoints.data(), (unsigned int)simMeshPoints.size());
        mInitialPositions[simMeshPathHandle.path] = simMeshPositions;
    }

    UsdGeomPointBased collMesh(data.collMeshPrim);
    if (collMesh)
    {
        if (collMesh.GetPrim() != simMesh.GetPrim())
        {
            collMesh.GetPointsAttr().Get(&collMeshPoints);

            // add tag for hydra to pick this prim up
            omni::fabric::PathC collMeshPathHandle = omni::fabric::asInt(data.collMeshPrim.GetPath());
            iStageReaderWriter->createAttribute(
                stageInProgress, collMeshPathHandle, primType, omni::fabric::TypeC(primTypeType));

            PositionCache::const_iterator fit = mInitialPositions.find(collMeshPathHandle.path);
            if (fit == mInitialPositions.end())
            {
                pxr::VtArray<carb::Float3> collMeshPositions;
                copyBuffer(collMeshPositions, collMeshPoints.data(), (unsigned int)collMeshPoints.size());
                mInitialPositions[collMeshPathHandle.path] = collMeshPositions;
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
            pxr::VtArray<pxr::GfVec3f> points;
            skinMesh.GetPointsAttr().Get(&points);
            for (const pxr::GfVec3f& point : points)
            {
                allSkinMeshPoints.push_back(point);
            }

            // add tag for hydra to pick this prim up
            omni::fabric::PathC skinMeshPathHandle = omni::fabric::asInt(data.skinMeshPrims[i].GetPath());
            iStageReaderWriter->createAttribute(stageInProgress, skinMeshPathHandle, primType, omni::fabric::TypeC(primTypeType));

            PositionCache::const_iterator fit = mInitialPositions.find(skinMeshPathHandle.path);
            if (fit == mInitialPositions.end())
            {

                pxr::VtArray<carb::Float3> skinMeshPositions;
                copyBuffer(skinMeshPositions, points.data(), (unsigned int)points.size());
                mInitialPositions[skinMeshPathHandle.path] = skinMeshPositions;
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

    for (auto& it : mVolumeDeformableBodies)
    {
        const SdfPath path = omni::fabric::toSdfPath(it.first);

        PxDeformableVolume* deformableVolume = static_cast<PxDeformableVolume*>(iPhysX->getPhysXPtr(path, omni::physx::ePTDeformableVolume));
        if (deformableVolume == nullptr)
        {
            mIsDirty = true;
        }
    }

    if (mIsDirty)
    {
        mVolumeDeformableBodySets.clear();

        std::vector<omni::fabric::PathC> emptyPaths;

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
        omni::fabric::PathC pathHandle = omni::fabric::PathC(initial.first);

        // AD: attention, we will get a pointer to the pointer here. not really intuitive.
        auto fabricAttr = stage.getAttributeWr<carb::Float3*>(pathHandle, mPointsToken);
        if (fabricAttr)
        {
            carb::Float3* positions = *fabricAttr;
            if (positions)
            {
                for (size_t j = 0; j < initial.second.size(); ++j)
                {
                    positions[j] = initial.second[j];
                }
            }
        }
    }

    clear();
}

void VolumeDeformableBodyManager::saveToUsd(omni::fabric::StageReaderWriter& stage, pxr::UsdStageRefPtr& usdStage)
{
    for (const auto& initial : mInitialPositions)
    {
        omni::fabric::PathC pathHandle = omni::fabric::PathC(initial.first);

        const size_t size = stage.getArrayAttributeSize(pathHandle, mPointsToken);
        auto fabricAttr = stage.getAttributeRd<carb::Float3*>(pathHandle, mPointsToken);
        if (fabricAttr && size == initial.second.size())
        {
            carb::Float3* positions = *fabricAttr;
            if (positions)
            {
                const pxr::SdfPath primPath = omni::fabric::toSdfPath(pathHandle);
                UsdPrim prim = usdStage->GetPrimAtPath(primPath);
                UsdAttribute pointsAttr = prim.GetAttribute(UsdGeomTokens->points);

                pxr::VtArray<pxr::GfVec3f> points(size);
                for (size_t j = 0; j < size; ++j)
                {
                    carb::Float3 tmp = positions[j];
                    points[j] = pxr::GfVec3f{ tmp.x, tmp.y, tmp.z };
                }
                pointsAttr.Set(points);
            }
        }
    }

    clear();
}

#endif

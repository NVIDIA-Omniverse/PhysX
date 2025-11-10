// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "SurfaceDeformableBodyManager.h"
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

SurfaceDeformableBodySet::SurfaceDeformableBodySet()
{
    using omni::fabric::AttributeRole;
    using omni::fabric::BaseDataType;

    omni::fabric::IToken* iToken = carb::getCachedInterface<omni::fabric::IToken>();

    mPointsToken = iToken->getHandle("points");
    mTypeFloat3Array = omni::fabric::Type(BaseDataType::eFloat, 3, 1, AttributeRole::ePosition);

    carb::settings::ISettings* iSettings = carb::getCachedInterface<carb::settings::ISettings>();
    mGPUInterop = iSettings->getAsBool(kSettingFabricUseGPUInterop);
}

SurfaceDeformableBodySet::~SurfaceDeformableBodySet()
{
    releaseBuffers();
}

void SurfaceDeformableBodySet::releaseBuffers()
{
    PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

    if (!cudaContextManager)
        return;

    PxScopedCudaLock _lock(*cudaContextManager);

    for (size_t i = 0; i < mSurfaceDeformableBodies.size(); ++i)
    {
        SurfaceDeformableBody& surfaceDeformableBody = *mSurfaceDeformableBodies[i];

        {
            pxr::SdfPath path = surfaceDeformableBody.data.simMeshPrim.GetPath();
            omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

            if (surfaceDeformableBody.stagingPointsDevMap[pathHandle])
                PxCudaHelpersExt::freeDeviceBuffer(*cudaContextManager, surfaceDeformableBody.stagingPointsDevMap[pathHandle]);
        }

        for (size_t i = 0; i < surfaceDeformableBody.skinMeshPrims.size(); ++i)
        {
            pxr::SdfPath path = surfaceDeformableBody.skinMeshPrims[i].GetPath();
            omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

            if (surfaceDeformableBody.stagingPointsDevMap[pathHandle])
                PxCudaHelpersExt::freeDeviceBuffer(*cudaContextManager, surfaceDeformableBody.stagingPointsDevMap[pathHandle]);
        }
    }

    if (mDeformableBodyGPUDataH)
        PxCudaHelpersExt::freePinnedHostBuffer(*cudaContextManager, mDeformableBodyGPUDataH);

    if (mPointsCopyEvent)
        cudaContextManager->getCudaContext()->eventDestroy(mPointsCopyEvent);

    if (mDeformableBodyCopyStream)
        cudaContextManager->getCudaContext()->streamDestroy(mDeformableBodyCopyStream);
}

void SurfaceDeformableBodySet::createFabricAttribute(omni::fabric::StageReaderWriter& srw, PxCudaContextManager* cudaContextManager, pxr::SdfPath path, uint32_t numVertices, std::unordered_map<omni::fabric::PathC, float3*>& stagingPointsDevMap)
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

void SurfaceDeformableBodySet::prepareBuffers(omni::fabric::StageReaderWriter& srw)
{
    size_t numSurfaceDeformableBodies = mSurfaceDeformableBodies.size();
    if (numSurfaceDeformableBodies)
    {
        PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

        if (!cudaContextManager)
            return;

        PxScopedCudaLock _lock(*cudaContextManager);

        releaseBuffers();

        // Calculate the number of fabric attributes to create
        mNumFabricAttributes = 0;
        for (SurfaceDeformableBody* const it : mSurfaceDeformableBodies)
        {
            // simMeshPrim + skinMeshPrims
            mNumFabricAttributes += 1 + (uint32_t)it->data.skinMeshPrims.size();
        }

        mDeformableBodyGPUDataH = PxCudaHelpersExt::allocPinnedHostBuffer<DeformableBodyGPUData>(*cudaContextManager, (PxU32)mNumFabricAttributes);
        if (!mDeformableBodyGPUDataH)
            return;

        // Synchronization event
        cudaContextManager->getCudaContext()->streamCreate(&mDeformableBodyCopyStream, 0x1);
        cudaContextManager->getCudaContext()->eventCreate(&mPointsCopyEvent, CU_EVENT_DISABLE_TIMING);

        mMaxPoints = 0;
        for (SurfaceDeformableBody* const it: mSurfaceDeformableBodies)
        {
            {
                createFabricAttribute(srw, cudaContextManager, it->data.simMeshPrim.GetPath(), it->data.numSimMeshVertices, it->stagingPointsDevMap);
            }

            for (size_t i = 0; i < it->data.skinMeshPrims.size(); ++i)
            {
                createFabricAttribute(srw, cudaContextManager, it->data.skinMeshPrims[i].GetPath(), it->data.skinMeshRanges[i].y, it->stagingPointsDevMap);
            }
        }
    }
}

void SurfaceDeformableBodySet::updateInternalSurfaceDeformableBodyData()
{
    PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

    if (!cudaContextManager)
        return;

    PxScopedCudaLock _lock(*cudaContextManager);

    cudaContextManager->getCudaContext()->eventRecord(mPointsCopyEvent, mDeformableBodyCopyStream);

    for (SurfaceDeformableBody* const it : mSurfaceDeformableBodies)
    {
        if (!it->physXPtr->isSleeping())
        {
            // Update simMeshPositionInvMassH so that picking can work correctly 
            PxCudaHelpersExt::copyDToHAsync(*cudaContextManager, it->data.simMeshPositionInvMassH, it->physXPtr->getPositionInvMassBufferD(), it->data.numSimMeshVertices, mDeformableBodyCopyStream);

            // Update allSkinnedVerticesH
            PxCudaHelpersExt::copyDToHAsync(*cudaContextManager, it->data.allSkinnedVerticesH, it->data.allSkinnedVerticesD, it->data.numSkinMeshVertices, mDeformableBodyCopyStream);
        }
    }
}

void SurfaceDeformableBodySet::prepareInteropData(omni::fabric::StageReaderWriter& srw, pxr::SdfPath path, DeformableBodyGPUData& gpuData,
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

void SurfaceDeformableBodySet::updateSurfaceDeformableBodies(omni::fabric::StageReaderWriter& srw)
{
    PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

    if (!cudaContextManager)
        return;

    PxScopedCudaLock _lock(*cudaContextManager);

    size_t fabricAttributeCount = 0;
    for (PxU32 i = 0; i < mSurfaceDeformableBodies.size(); ++i)
    {
        SurfaceDeformableBody* hostData = mSurfaceDeformableBodies[i];

        {
            DeformableBodyGPUData& gpuData = mDeformableBodyGPUDataH[fabricAttributeCount++];

            prepareInteropData
            (
                srw, hostData->data.simMeshPrim.GetPath(), gpuData,
                hostData->physXPtr->getPositionInvMassBufferD(), hostData->data.numSimMeshVertices, hostData->data.worldToSimMesh,
                hostData->stagingPointsDevMap, hostData->fabricPointsCPUMap, 4
            );
        }

        // Current implementation limitation for surface deformable
        if (hostData->data.skinMeshPrims.size() > 1)
        {
            CARB_LOG_WARN_ONCE("No multiple skin mesh support for surface deformables yet.");
            continue;
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
        for (PxU32 i = 0; i < mSurfaceDeformableBodies.size(); ++i)
        {
            SurfaceDeformableBody* hostData = mSurfaceDeformableBodies[i];

            {
                pxr::SdfPath path = hostData->data.simMeshPrim.GetPath();
                omni::fabric::PathC pathHandle = omni::fabric::asInt(path);
                uint32_t numVertices = hostData->data.numSimMeshVertices;

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
        for (PxU32 i = 0; i < mSurfaceDeformableBodies.size(); ++i)
        {
            SurfaceDeformableBody* hostData = mSurfaceDeformableBodies[i];

            {
                pxr::SdfPath path = hostData->data.simMeshPrim.GetPath();
                omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

                copyBuffer(hostData->fabricPointsCPUMap[pathHandle], hostData->data.simMeshPositionInvMassH, (unsigned int)hostData->data.numSimMeshVertices,
                           hostData->data.worldToSimMesh);
            }

            for (size_t i = 0; i < hostData->data.skinMeshPrims.size(); ++i)
            {
                pxr::SdfPath path = hostData->data.skinMeshPrims[i].GetPath();
                omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

                copyBuffer(hostData->fabricPointsCPUMap[pathHandle], hostData->data.allSkinnedVerticesH + hostData->data.skinMeshRanges[i].x, (unsigned int)hostData->data.skinMeshRanges[i].y,
                            hostData->data.worldToSkinMeshTransforms[i]);
            }
        }
    }
#endif
}

SurfaceDeformableBodyManager::SurfaceDeformableBodyManager()
{
    using omni::fabric::AttributeRole;
    using omni::fabric::BaseDataType;

    mTypeAppliedSchema = omni::fabric::Type(BaseDataType::eTag, 1, 0, AttributeRole::eAppliedSchema);

    mTypeFloat3 = omni::fabric::Type(BaseDataType::eFloat, 3, 0, AttributeRole::eNone);
    mTypeFloat3Array = omni::fabric::Type(BaseDataType::eFloat, 3, 1, AttributeRole::ePosition);

    omni::fabric::IToken* iToken = carb::getCachedInterface<omni::fabric::IToken>();
    mPointsToken = iToken->getHandle("points");
}

SurfaceDeformableBodyManager::~SurfaceDeformableBodyManager()
{
    clear();
}

void SurfaceDeformableBodyManager::registerDeformableBody(pxr::UsdGeomXformCache& xfCache, uint64_t stageId, omni::fabric::IStageReaderWriter* iStageReaderWriter, omni::fabric::StageReaderWriterId stageInProgress, const pxr::UsdPrim& prim)
{
    pxr::SdfPath path = prim.GetPath();
    omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

    // Check if deformable body is a valid surface deformable
    IPhysx* iPhysX = carb::getCachedInterface<IPhysx>();
    PxDeformableSurface* deformableSurface = static_cast<PxDeformableSurface*>(iPhysX->getPhysXPtr(path, omni::physx::ePTDeformableSurface));
    if (deformableSurface == nullptr)
    {
        return;
    }

    IPhysxPrivate* iPhysXPrivate = carb::getCachedInterface<IPhysxPrivate>();

    SurfaceDeformableBody& deformableBody = mSurfaceDeformableBodies[pathHandle];
    InternalSurfaceDeformableBodyData& data = deformableBody.data;
    iPhysXPrivate->getInternalSurfaceDeformableBodyData((const usdparser::ObjectId)deformableSurface->userData, data);

    VtArray<GfVec3f> simMeshPoints;
    VtArray<GfVec3f> allSkinMeshPoints;

    UsdGeomMesh simMesh(data.simMeshPrim);
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

bool SurfaceDeformableBodyManager::prepareBuffers(omni::fabric::StageReaderWriter& srw)
{
    IPhysx* iPhysX = carb::getCachedInterface<IPhysx>();

    for (auto const &it : mSurfaceDeformableBodies)
    {
        const SdfPath path = omni::fabric::toSdfPath(it.first);

        PxDeformableSurface* deformableSurface = static_cast<PxDeformableSurface*>(iPhysX->getPhysXPtr(path, omni::physx::ePTDeformableSurface));
        if (deformableSurface == nullptr)
        {
            mIsDirty = true;
        }
    }

    if (mIsDirty)
    {
        mSurfaceDeformableBodySets.clear();

        std::vector<omni::fabric::PathC> emptyPaths;

        for (auto &it : mSurfaceDeformableBodies)
        {
            const SdfPath path = omni::fabric::toSdfPath(it.first);

            PxDeformableSurface* deformableSurface = static_cast<PxDeformableSurface*>(iPhysX->getPhysXPtr(path, omni::physx::ePTDeformableSurface));
            if (deformableSurface)
            {
                it.second.physXPtr = deformableSurface;

                PxScene* scene = deformableSurface->getScene();
                if (scene)
                {
                    SurfaceDeformableBodySet& set = mSurfaceDeformableBodySets[scene];

                    set.mScene = scene;
                    set.mSurfaceDeformableBodies.push_back(&it.second);
                }
            }
            else
                emptyPaths.push_back(it.first);
        }

        for (auto const &it : emptyPaths)
        {
            mSurfaceDeformableBodies.erase(it);
        }

        for (auto &it : mSurfaceDeformableBodySets)
        {
            it.second.prepareBuffers(srw);
        }

        mIsDirty = false;
    }

    return true;
}
 
void SurfaceDeformableBodyManager::clear()
{
    mSurfaceDeformableBodySets.clear();
    mSurfaceDeformableBodies.clear();
    mInitialPositions.clear();
}

void SurfaceDeformableBodyManager::update(omni::fabric::StageReaderWriter& srw)
{
    carb::settings::ISettings* iSettings = carb::getCachedInterface<carb::settings::ISettings>();
           
    bool updatePoints = iSettings->getAsBool(kSettingFabricUpdatePoints);
    if (updatePoints)
    {
        if (!prepareBuffers(srw))
        {
            return;
        }

        updateSurfaceDeformableBodies(srw);
    }
}

void SurfaceDeformableBodyManager::updateSurfaceDeformableBodies(omni::fabric::StageReaderWriter& srw)
{
    for (auto &it : mSurfaceDeformableBodySets)
    {
        if (it.second.mSurfaceDeformableBodies.size() > 0)
        {
            it.second.updateInternalSurfaceDeformableBodyData();
            it.second.updateSurfaceDeformableBodies(srw);
        }
    }
}

void SurfaceDeformableBodyManager::setInitialTransformation(omni::fabric::StageReaderWriter& stage)
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

void SurfaceDeformableBodyManager::saveToUsd(omni::fabric::StageReaderWriter& stage, pxr::UsdStageRefPtr& usdStage)
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

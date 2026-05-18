// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "DeformableSurfaceManagerDeprecated.h"
#include "FabricManager.h"

#include <omni/fabric/FabricUSD.h>
#include <carb/settings/ISettings.h>

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxFabric.h>
#include <omni/physx/PhysXConversions.h>
#include <pxr/usd/usdPhysics/tokens.h>
#include <physxSchema/tokens.h>

#include <PxPhysicsAPI.h>

#include <vector_types.h>

#include "PxDeformableSurfaceFlag.h"
#include "cudamanager/PxCudaContext.h"

using namespace pxr;
using namespace physx;
using namespace carb;

namespace omni
{
namespace physx
{

DeformableSurfaceSetDeprecated::DeformableSurfaceSetDeprecated()
{
    using omni::fabric::AttributeRole;
    using omni::fabric::BaseDataType;

    mPointsToken = omni::fabric::Token::createImmortal("points");
    mTypeFloat3Array = omni::fabric::Type(BaseDataType::eFloat, 3, 1, AttributeRole::ePosition);

    carb::settings::ISettings* iSettings = carb::getCachedInterface<carb::settings::ISettings>();
    mGPUInterop = iSettings->getAsBool(kSettingFabricUseGPUInterop);
}

DeformableSurfaceSetDeprecated::~DeformableSurfaceSetDeprecated()
{
    releaseBuffers();
}

void DeformableSurfaceSetDeprecated::releaseBuffers()
{
    PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

    if (!cudaContextManager)
        return;

    PxScopedCudaLock _lock(*cudaContextManager);

    for (size_t i = 0; i < mDeformableSurfaces.size(); ++i)
    {
        DeformableSurfaceDataDeprecated& data = *mDeformableSurfaces[i];
        if (data.stagingPointsDev)
            PX_DEVICE_FREE(cudaContextManager, data.stagingPointsDev);
    }

    if (mDeformableSurfaceGPUDataH)
        PX_PINNED_HOST_FREE(cudaContextManager, mDeformableSurfaceGPUDataH);

}

void DeformableSurfaceSetDeprecated::prepareBuffers(omni::fabric::StageReaderWriter& srw)
{
    size_t numCloths = mDeformableSurfaces.size();
    if (numCloths)
    {
        PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

        if (!cudaContextManager)
            return;

        PxScopedCudaLock _lock(*cudaContextManager);

        releaseBuffers();

        mDeformableSurfaceGPUDataH = PX_PINNED_HOST_ALLOC_T(DeformableSurfaceGPUDataDeprecated, cudaContextManager, (PxU32)numCloths);
        if (!mDeformableSurfaceGPUDataH)
            return;

        mMaxPoints = 0;
        for (DeformableSurfaceDataDeprecated* const it: mDeformableSurfaces)
        {
            const SdfPath path = it->prim.GetPath();
            const omni::fabric::Path pathHandle = omni::fabric::convertToPathType<omni::fabric::Path>(srw.getFabricId(), path);

            srw.createPrim(pathHandle);
            srw.createAttribute(pathHandle, mPointsToken, mTypeFloat3Array);
            srw.setArrayAttributeSize(pathHandle, mPointsToken, it->numVerts);

            GfMatrix4d parentToWorld = GfMatrix4d(UsdGeomXform(it->prim).ComputeParentToWorldTransform(UsdTimeCode::Default()));
            it->worldToDeformableSurface = (GfMatrix4d(it->initialPrimToParent) * parentToWorld).GetInverse();

            mMaxPoints = mMaxPoints < (uint32_t)it->numVerts ? (uint32_t)it->numVerts : mMaxPoints;

            if (!mGPUInterop)
            {
                float3* stagingPoints = PX_DEVICE_ALLOC_T(float3, cudaContextManager, (uint32_t)it->numVerts);
                it->stagingPointsDev = stagingPoints;
            }
        }
    }
}

void DeformableSurfaceSetDeprecated::updateDeformableSurfaces(omni::fabric::StageReaderWriter& srw) const
{
    PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

    if (!cudaContextManager)
        return;

    PxScopedCudaLock _lock(*cudaContextManager);
    PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

    for (PxU32 i = 0; i < mDeformableSurfaces.size(); ++i)
    {
        DeformableSurfaceGPUDataDeprecated& data = mDeformableSurfaceGPUDataH[i];

        DeformableSurfaceDataDeprecated* hostData = mDeformableSurfaces[i];
        SdfPath path = hostData->prim.GetPath();
        const omni::fabric::Path pathHandle = omni::fabric::convertToPathType<omni::fabric::Path>(srw.getFabricId(), path);

        if (mGPUInterop)
        {
            float3** cudaPtr = srw.getAttributeWrGpu<float3*>(pathHandle, mPointsToken);
            if (cudaPtr && *cudaPtr)
            {
                data.dstPoints = *cudaPtr;
            }
            else
            {
                CARB_LOG_ERROR("Failed to get valid GPU points attribute!");
                return;
            }
        }
        else
        {
            float3** cpuPtr = srw.getAttributeWr<float3*>(pathHandle, mPointsToken);
            data.dstPoints = hostData->stagingPointsDev;
            if (cpuPtr && *cpuPtr)
            {
                hostData->fabricPointsCPU = *cpuPtr;
            }
            else
            {
                CARB_LOG_ERROR("Failed to get valid CPU points attribute!");
                return;
            }
        }

        GfMatrix4f parentToWorld = GfMatrix4f(UsdGeomXform(hostData->prim).ComputeParentToWorldTransform(UsdTimeCode::Default()));
        GfMatrix4f worldToDeformableSurface = (GfMatrix4f(hostData->initialPrimToParent) * parentToWorld).GetInverse();

        float4* worldToDeformableFloat4 = reinterpret_cast<float4*>(&worldToDeformableSurface);
        data.transform[0] = worldToDeformableFloat4[0];
        data.transform[1] = worldToDeformableFloat4[1];
        data.transform[2] = worldToDeformableFloat4[2];
        data.transform[3] = worldToDeformableFloat4[3];

        data.srcPoints = reinterpret_cast<float4*>(hostData->physXPtr->getPositionInvMassBufferD());
        data.nbPoints = (int)hostData->numVerts;
    }

    convertVec4ftoVec3fBlockDeformableSurfaceDeprecated(mDeformableSurfaceGPUDataH, (int)mDeformableSurfaces.size(), mMaxPoints);

    if (!mGPUInterop)
    {
        // TODO: async. Would also be awesome to just get a pinned host memory pointer for the fabric buffer, then
        // we could just use that one as destination buffer in the kernel above.
        for (PxU32 i = 0; i < mDeformableSurfaces.size(); ++i)
        {
            DeformableSurfaceDataDeprecated* data = mDeformableSurfaces[i];
            cudaContext->memcpyDtoH(data->fabricPointsCPU, reinterpret_cast<CUdeviceptr>(data->stagingPointsDev), data->numVerts * sizeof(float3));
        }
    }
}

DeformableSurfaceManagerDeprecated::DeformableSurfaceManagerDeprecated()
{
    using omni::fabric::AttributeRole;
    using omni::fabric::BaseDataType;

    mTypeAppliedSchema = omni::fabric::Type(BaseDataType::eTag, 1, 0, AttributeRole::eAppliedSchema);

    mTypeFloat3 = omni::fabric::Type(BaseDataType::eFloat, 3, 0, AttributeRole::eNone);
    mTypeFloat3Array = omni::fabric::Type(BaseDataType::eFloat, 3, 1, AttributeRole::ePosition);

    mDeformableSurfaceSchemaToken = omni::fabric::Token::createImmortal("PhysxDeformableSurfaceAPI");
    mPointsToken = omni::fabric::Token::createImmortal("points");
          
    mPhysxSimulationInterface = carb::getCachedInterface<omni::physx::IPhysxSimulation>();
}

DeformableSurfaceManagerDeprecated::~DeformableSurfaceManagerDeprecated()
{
}

void DeformableSurfaceManagerDeprecated::registerDeformableSurface(UsdGeomXformCache& xfCache, uint64_t stageId, omni::fabric::IStageReaderWriter* iStageReaderWriter,
    omni::fabric::StageReaderWriterId stageInProgress, const UsdPrim& prim)
{
    const SdfPath path = prim.GetPath();
    omni::fabric::Path pathHandle = omni::fabric::convertToPathType<omni::fabric::Path>(iStageReaderWriter->getFabricId(stageInProgress), path);

    UsdGeomPointBased pointBased(prim);
    VtArray<GfVec3f> points;
    pointBased.GetPointsAttr().Get(&points);
    
    const omni::fabric::UsdStageId ustageId = { uint64_t(stageId) };
    
    // add tag for hydra to pick this prim up
    constexpr omni::fabric::Type primTypeType(
        omni::fabric::BaseDataType::eTag, 1, 0, omni::fabric::AttributeRole::ePrimTypeName);
    const omni::fabric::Token primType("Deformable");
    iStageReaderWriter->createAttribute(stageInProgress, pathHandle, primType, omni::fabric::TypeC(primTypeType));

    DeformableSurfaceDataDeprecated& dd = mDeformableSurfaces[pathHandle];
    dd.prim = prim;
    dd.numVerts = points.size();

    PositionCache::const_iterator fit = mInitialPositions.find(pathHandle);
    if (fit == mInitialPositions.end())
    {
        bool resetsXformStack;
        UsdGeomXform(prim).GetLocalTransformation(&dd.initialPrimToParent, &resetsXformStack, UsdTimeCode::Default());
        mInitialPositions[pathHandle] = points;
    }

    mIsDirty = true;
}

bool DeformableSurfaceManagerDeprecated::prepareBuffers(omni::fabric::StageReaderWriter& srw)
{
    IPhysx* iPhysX = carb::getCachedInterface<IPhysx>();
    std::vector<omni::fabric::Path> emptyPaths;

    for (auto const &it : mDeformableSurfaces)
    {
        const SdfPath path = omni::fabric::toSdfPath(it.first);

        PxDeformableSurface* deformableSurface = static_cast<PxDeformableSurface*>(iPhysX->getPhysXPtr(path, omni::physx::ePTFEMClothDeprecated));
        if (deformableSurface == nullptr)
        {
            mIsDirty = true;
        }
    }

    if (mIsDirty)
    {
        mDeformableSurfacesSet.clear();

        std::vector<omni::fabric::Path> emptyPaths;

        for (auto &it : mDeformableSurfaces)
        {
            const SdfPath path = omni::fabric::toSdfPath(it.first);

            PxDeformableSurface* deformableSurface = static_cast<PxDeformableSurface*>(iPhysX->getPhysXPtr(path, omni::physx::ePTFEMClothDeprecated));
            if (deformableSurface)
            {
                it.second.physXPtr = deformableSurface;

                PxScene* scene = deformableSurface->getScene();
                if (scene)
                {
                    DeformableSurfaceSetDeprecated& set = mDeformableSurfacesSet[scene];

                    set.mScene = scene;
                    set.mDeformableSurfaces.push_back(&it.second);
                }
            }
            else
                emptyPaths.push_back(it.first);
        }

        for (auto const &it : emptyPaths)
        {
            mDeformableSurfaces.erase(it);
        }

        for (auto &it : mDeformableSurfacesSet)
        {
            it.second.prepareBuffers(srw);
        }

        mIsDirty = false;
    }

    return true;
}

 
void DeformableSurfaceManagerDeprecated::clear()
{
    for (auto &it : mDeformableSurfacesSet)
    {
        it.second.releaseBuffers();
    }
}

void DeformableSurfaceManagerDeprecated::update(omni::fabric::StageReaderWriter& srw)
{
    carb::settings::ISettings* iSettings = carb::getCachedInterface<carb::settings::ISettings>();
           
    bool updatePoints = iSettings->getAsBool(kSettingFabricUpdatePoints);
    if (updatePoints)
    {
        if (!prepareBuffers(srw))
        {
            return;
        }

        updateDeformableSurfaces(srw);
    }
}

void DeformableSurfaceManagerDeprecated::updateDeformableSurfaces(omni::fabric::StageReaderWriter& srw)
{
    for (auto const &it : mDeformableSurfacesSet)
    {
        if (it.second.mDeformableSurfaces.size() > 0)
        {
            it.second.updateDeformableSurfaces(srw);
        }
    }
}

void DeformableSurfaceManagerDeprecated::setInitialTransformation(omni::fabric::StageReaderWriter& stage)
{
    omni::fabric::StageReaderWriterUsd stageUsd{ stage.getId() };
    const omni::fabric::set<omni::fabric::AttrNameAndType> requiredAll = { omni::fabric::AttrNameAndType(mTypeAppliedSchema, mDeformableSurfaceSchemaToken) };
    const omni::fabric::set<omni::fabric::AttrNameAndType> requiredAny = { omni::fabric::AttrNameAndType(mTypeFloat3Array, mPointsToken) };

    omni::fabric::PrimBucketList primBuckets = stage.findPrims(requiredAll, requiredAny);
    size_t bucketCount = primBuckets.bucketCount();
    for (size_t i = 0; i != bucketCount; i++)
    {
        gsl::span<const omni::fabric::Path> paths = stage.getPathArray(primBuckets, i);
        for (const omni::fabric::Path& path : paths)
        {
            PositionCache::const_iterator fit = mInitialPositions.find(path);
            if (fit != mInitialPositions.end())
            {
                pxr::VtValue value{ fit->second };
                stageUsd.setArrayAttribute(path, mPointsToken, value);
            }
        }
    }

    mInitialPositions.clear();
}

void DeformableSurfaceManagerDeprecated::saveToUsd(omni::fabric::StageReaderWriter& stage, UsdStageRefPtr& usdStage)
{
   const omni::fabric::set<omni::fabric::AttrNameAndType> requiredAll = { omni::fabric::AttrNameAndType(mTypeAppliedSchema, mDeformableSurfaceSchemaToken) };
   const omni::fabric::set<omni::fabric::AttrNameAndType> requiredAny = { omni::fabric::AttrNameAndType(mTypeFloat3Array, mPointsToken) };

    omni::fabric::PrimBucketList primBuckets = stage.findPrims(requiredAll, requiredAny);
    size_t bucketCount = primBuckets.bucketCount();
    for (size_t i = 0; i < bucketCount; i++)
    {
        gsl::span<const omni::fabric::Path> paths = stage.getPathArray(primBuckets, i);
        for (const omni::fabric::Path& path : paths)
        {
            const size_t size = stage.getArrayAttributeSize(path, mPointsToken);
            carb::Float3* positions = *(stage.getAttributeRd<carb::Float3*>(path, mPointsToken));

            const SdfPath primPath = omni::fabric::toSdfPath(path);
            UsdPrim prim = usdStage->GetPrimAtPath(primPath);

            {
                UsdAttribute pointsAttr = prim.GetAttribute(UsdGeomTokens->points);
                if (pointsAttr)
                {
                    VtArray<GfVec3f> points(size);
                    for (size_t j = 0; j < size; ++j)
                    {
                        carb::Float3 tmp = positions[j];
                        points[j] = GfVec3f{ tmp.x, tmp.y, tmp.z };
                    }
                    pointsAttr.Set(points);
                }
            }
        }
    }
}

} // namespace physx
} // namespace omni

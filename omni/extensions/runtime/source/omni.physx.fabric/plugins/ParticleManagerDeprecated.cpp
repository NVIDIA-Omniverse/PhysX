// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "ParticleManagerDeprecated.h"
#include "FabricKernels.h"
#include "FabricManager.h"

#if !CARB_AARCH64

#include <carb/logging/Log.h>
#include <carb/InterfaceUtils.h>
#include <omni/fabric/IFabric.h>
#include <omni/fabric/FabricUSD.h>
#include <carb/settings/ISettings.h>

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxFabric.h>
#include <pxr/usd/usdPhysics/tokens.h>
#include <physxSchema/tokens.h>
#include <cudamanager/PxCudaContext.h>

#include <PxPhysicsAPI.h>

#include <cuda.h>

#include <algorithm>
#include <unordered_set>
#include <vector>
#include "driver_types.h"

#include <common/foundation/TypeCast.h>

using namespace pxr;
using namespace physx;
using namespace carb;

#define PARTICLE_CLOTH_DEBUG   0

namespace omni
{
namespace physx
{

static const TfToken physxParticleWeldedTriangleIndicesToken{ "physxParticle:weldedTriangleIndices" };
static const TfToken physxParticleWeldedVerticesRemapToWeldToken{ "physxParticle:weldedVerticesRemapToWeld" };

ParticleClothSetDeprecated::ParticleClothSetDeprecated()
{
    using omni::fabric::AttributeRole;
    using omni::fabric::BaseDataType;

    omni::fabric::IToken* iToken = carb::getCachedInterface<omni::fabric::IToken>();

    mParticleClothSchemaToken = iToken->getHandle("PhysxParticleClothAPI");
    mPointsToken = iToken->getHandle("points");
    mTypeFloat3Array = omni::fabric::Type(BaseDataType::eFloat, 3, 1, AttributeRole::ePosition);

    carb::settings::ISettings* iSettings = carb::getCachedInterface<carb::settings::ISettings>();
    mGPUInterop = iSettings->getAsBool(kSettingFabricUseGPUInterop);
}

ParticleClothSetDeprecated::~ParticleClothSetDeprecated()
{
    releaseBuffers();
}

void ParticleClothSetDeprecated::releaseBuffers()
{
    PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

    if (!cudaContextManager)
        return;

    PxScopedCudaLock _lock(*cudaContextManager);

    PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

    size_t numCloths = mParticleCloths.size();
    if (numCloths)
    {
        for (size_t i = 0; i < numCloths; ++i)
        {
            ParticleClothDataDeprecated* cloth = mParticleCloths[i];
            if (cloth->remapTable)
            {
                cudaContext->memFree(cloth->remapTable);
                cloth->remapTable = 0;
            }
        }

        if (!mGPUInterop && mFabricHost)
        {
            for (uint32_t i = 0; i < numCloths; ++i)
            {
                cudaContext->memFree(mFabricHost[i]);
                mFabricHost[i] = 0;
            }
        }

        if (mParticleClothPointsDev)
        {
            cudaContext->memFree(mParticleClothPointsDev);
            mParticleClothPointsDev = 0;
        }

        if (mParticleClothSizesDev)
        {
            cudaContext->memFree(mParticleClothSizesDev);
            mParticleClothSizesDev = 0;
        }

        if (mParticleClothMtxDevPtrs)
        {
            cudaContext->memFree(mParticleClothMtxDevPtrs);
            mParticleClothMtxDevPtrs = 0;
        }

        if (mFabricPointsDev)
        {
            cudaContext->memFree(mFabricPointsDev);
            mFabricPointsDev = 0;
        }

        if (mParticleClothRemapTablesDev)
        {
            cudaContext->memFree(mParticleClothRemapTablesDev);
            mParticleClothRemapTablesDev = 0;
        }

        if (mWorldToParticleClothHost)
        {
            cudaContext->memFreeHost(mWorldToParticleClothHost);
            mWorldToParticleClothHost = 0;
        }

        if (mFabricHost)
        {
            cudaContext->memFreeHost(mFabricHost);
            mFabricHost = 0;
        }

        if (mParticleClothPointsDevPtrsHost)
        {
            cudaContext->memFreeHost(mParticleClothPointsDevPtrsHost);
            mParticleClothPointsDevPtrsHost = 0;
        }

        if (mParticleClothSizesHost)
        {
            cudaContext->memFreeHost(mParticleClothSizesHost);
            mParticleClothSizesHost = 0;
        }

        if (mParticleClothRemapTablesDevPtrsHost)
        {
            cudaContext->memFreeHost(mParticleClothPointsDevPtrsHost);
            mParticleClothPointsDevPtrsHost = 0;
        }

        if (mFabricPointsCPU)
        {
            cudaContext->memFreeHost(mFabricPointsCPU);
            mFabricPointsCPU = 0;
        }
    }
}

void ParticleClothSetDeprecated::prepareBuffers(omni::fabric::StageReaderWriter& srw)
{

    size_t numCloths = (PxU32)mParticleCloths.size();
    if (numCloths)
    {
        PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

        if (!cudaContextManager)
            return;

        PxScopedCudaLock _lock(*cudaContextManager);

        PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

        releaseBuffers();

        IPhysx* iPhysX = carb::getCachedInterface<IPhysx>();
         
        PxCUresult result = cudaContext->memHostAlloc((void**)&mParticleClothSizesHost, sizeof(PxU32) * numCloths, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
        PxU32 count = 0;

        for (auto& it : mParticleCloths)
        {
            const SdfPath& path = it->prim.GetPath();

            PxParticleClothBuffer* pc = static_cast<PxParticleClothBuffer*>(iPhysX->getPhysXPtr(path, omni::physx::ePTParticleClothDeprecated));

            it->points = pc->getPositionInvMasses();
                    
            PxU32 pointsSize = (PxU32)(it->numVerts);
            mParticleClothSizesHost[count] = pointsSize;
            count++;
        }

        result = cudaContext->memAlloc(&mParticleClothMtxDevPtrs, sizeof(float) * 16 * numCloths);
        result = cudaContext->memAlloc(&mFabricPointsDev, sizeof(CUdeviceptr) * numCloths);
        result = cudaContext->memAlloc(&mParticleClothPointsDev, numCloths * sizeof(CUdeviceptr));
        result = cudaContext->memAlloc(&mParticleClothSizesDev, numCloths * sizeof(int));
        result = cudaContext->memAlloc(&mParticleClothRemapTablesDev, numCloths * sizeof(CUdeviceptr));

        result = cudaContext->memHostAlloc((void**)&mWorldToParticleClothHost, sizeof(GfMatrix4f) * numCloths, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
        result = cudaContext->memHostAlloc((void**)&mFabricHost, sizeof(CUdeviceptr) * numCloths, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
        result = cudaContext->memHostAlloc((void**)&mParticleClothPointsDevPtrsHost, sizeof(CUdeviceptr) * numCloths, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
        result = cudaContext->memHostAlloc((void**)&mParticleClothRemapTablesDevPtrsHost, sizeof(CUdeviceptr) * numCloths, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
        result = cudaContext->memHostAlloc((void**)&mFabricPointsCPU, sizeof(float3*) * numCloths, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);

        mMaxPoints = 0;
        for (PxU32 i = 0; i < mParticleCloths.size(); ++i)
        {
            ParticleClothDataDeprecated* it = mParticleCloths[i];

            SdfPath path = it->prim.GetPath();
            omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

            srw.createPrim(pathHandle);
            srw.createAttribute(pathHandle, mPointsToken, mTypeFloat3Array);
            srw.setArrayAttributeSize(pathHandle, mPointsToken, it->numVerts);

            // prepare a device staging buffer if CPU interop
            if (!mGPUInterop)
            {
                CUdeviceptr pointsGPU;
                result = cudaContext->memAlloc(&pointsGPU, it->numVerts * sizeof(float3));
                mFabricHost[i] = pointsGPU;
            }

            GfMatrix4f parentToWorld = GfMatrix4f(UsdGeomXform(it->prim).ComputeParentToWorldTransform(UsdTimeCode::Default()));
            GfMatrix4f worldToParticleCloth = (GfMatrix4f(it->initialPrimToParent) * parentToWorld).GetInverse();

            mWorldToParticleClothHost[i] = worldToParticleCloth;
            mParticleClothPointsDevPtrsHost[i] = reinterpret_cast<CUdeviceptr>(it->points);

            // check if welded
            bool isWelded = false;
            VtArray<uint32_t> weldedTriangleIndices;
            UsdAttribute weldedTriangleIndicesAttr = it->prim.GetAttribute(physxParticleWeldedTriangleIndicesToken);
            if (weldedTriangleIndicesAttr.HasAuthoredValue())
            {
                weldedTriangleIndicesAttr.Get(&weldedTriangleIndices);
            }

            PhysxSchemaPhysxAutoParticleClothAPI autoParticleCloth(it->prim);
            bool enableWelding = !!autoParticleCloth;
            if (enableWelding)
            {
                bool disableWelding;
                autoParticleCloth.GetDisableMeshWeldingAttr().Get(&disableWelding);
                enableWelding = !disableWelding;
            }
            isWelded = enableWelding && weldedTriangleIndices.size();

            if (isWelded)
            {
                uint32_t* ptr;
                result = cudaContext->memAlloc(&it->remapTable, sizeof(int) * it->numVerts);
                result = cudaContext->memHostAlloc((void**)&ptr, sizeof(int) * it->numVerts, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);

                // copy remapTable from USD to host buffer
                VtArray<uint32_t> verticesRemapToWeld;
                UsdAttribute verticesRemapToWeldAttr = it->prim.GetAttribute(physxParticleWeldedVerticesRemapToWeldToken);
                if (verticesRemapToWeldAttr.HasAuthoredValue())
                {
                    verticesRemapToWeldAttr.Get(&verticesRemapToWeld);
                }

                for (size_t j = 0; j < it->numVerts; ++j)
                {
                    ptr[j] = verticesRemapToWeld[j];
                }

                result = cudaContext->memcpyHtoD(it->remapTable, ptr, sizeof(int) * it->numVerts);
                result = cudaContext->memFreeHost(ptr);
            }

            mParticleClothRemapTablesDevPtrsHost[i] = it->remapTable;
           
#if PARTICLE_CLOTH_DEBUG
            cudaPointerAttributes attributes;
            cudaPointerGetAttributes(&attributes, (void*)mFabricHost[i]);

            printf("Fabric Device pointer %p on device %i\n", (void*)mFabricHost[i], attributes.device);  
#endif
            if (mMaxPoints < it->numVerts)
            {
                mMaxPoints = (int)it->numVerts;
            }
        }


        result = cudaContext->memcpyHtoD(mParticleClothPointsDev, mParticleClothPointsDevPtrsHost, numCloths * sizeof(CUdeviceptr));
        result = cudaContext->memcpyHtoD(mParticleClothSizesDev, mParticleClothSizesHost, numCloths * sizeof(int));
        result = cudaContext->memcpyHtoD(mParticleClothMtxDevPtrs, mWorldToParticleClothHost, sizeof(float) * 16 * numCloths);
        result = cudaContext->memcpyHtoD(mFabricPointsDev, mFabricHost, sizeof(CUdeviceptr) * numCloths);
        result = cudaContext->memcpyHtoD(mParticleClothRemapTablesDev, mParticleClothRemapTablesDevPtrsHost, sizeof(CUdeviceptr) * numCloths);
                
#if PARTICLE_CLOTH_DEBUG
        if (result != CUDA_SUCCESS)
        {
            int bob = 0;
        }
#endif
    }
}


void ParticleClothSetDeprecated::updateParticleCloths(omni::fabric::StageReaderWriter& srw) const
{
    // make sure the results are ready.
    mScene->fetchResultsParticleSystem();

    PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

    if (!cudaContextManager)
        return;

    PxScopedCudaLock _lock(*cudaContextManager);
    PxCudaContext* cudaContext = cudaContextManager->getCudaContext();  

    // need to get the current pointers
    for (PxU32 i = 0; i < mParticleCloths.size(); ++i)
    {
        ParticleClothDataDeprecated* it = mParticleCloths[i];

        SdfPath path = it->prim.GetPath();
        omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

        if (mGPUInterop)
        {
            float3** cudaPtr = srw.getAttributeWrGpu<float3*>(pathHandle, mPointsToken);
            mFabricHost[i] = (CUdeviceptr)*cudaPtr;
        }
        else
        {
            float3** cpuPtr = srw.getAttributeWr<float3*>(pathHandle, mPointsToken);
            mFabricPointsCPU[i] = *cpuPtr;
        }
    }

    if (mGPUInterop)
    {
        PxCUresult result = cudaContext->memcpyHtoD(mFabricPointsDev, mFabricHost, sizeof(CUdeviceptr) * mParticleCloths.size());
    }

    convertVec4ftoVec3fBlockParticleClothDeprecated((float3**)mFabricPointsDev, (const float4**)mParticleClothPointsDev, (const int*)mParticleClothSizesDev, (const float*)mParticleClothMtxDevPtrs, (const int**)mParticleClothRemapTablesDev, mParticleCloths.size(), mMaxPoints);

    if (!mGPUInterop)
    {
        // Read data from the GPU and populate the CPU-side Fabric buffers
        for (PxU32 i = 0; i < mParticleCloths.size(); ++i)
        {
            ParticleClothDataDeprecated* it = mParticleCloths[i];
            PxCUresult result = cudaContext->memcpyDtoH((void*)mFabricPointsCPU[i], mFabricHost[i], it->numVerts * sizeof(float3));
        }
    }
          
#if PARTICLE_CLOTH_DEBUG
    cudaPointerAttributes attributes;
    cudaPointerGetAttributes(&attributes, (void*)mParticleClothPointsDev);

    printf("Particle cloth Device pointer %p on device %i\n", (void*)mParticleClothPointsDev, attributes.device);
    PxCUresult result = cudaContext->streamSynchronize(0);

    if (result != CUDA_SUCCESS)
    {
        int bob = 0;
    }
#endif
}

ParticleManagerDeprecated::ParticleManagerDeprecated()
{
    using omni::fabric::AttributeRole;
    using omni::fabric::BaseDataType;

    mTypeAppliedSchema = omni::fabric::Type(BaseDataType::eTag, 1, 0, AttributeRole::eAppliedSchema);

    mTypeFloat3 = omni::fabric::Type(BaseDataType::eFloat, 3, 0, AttributeRole::eNone);
    mTypeFloat3Array = omni::fabric::Type(BaseDataType::eFloat, 3, 1, AttributeRole::ePosition);

    omni::fabric::IToken* iToken = carb::getCachedInterface<omni::fabric::IToken>();
    mParticleClothSchemaToken = iToken->getHandle("PhysxParticleClothAPI");
    mPointsToken = iToken->getHandle("points");
          
    mPhysxSimulationInterface = carb::getCachedInterface<omni::physx::IPhysxSimulation>();
}

ParticleManagerDeprecated::~ParticleManagerDeprecated()
{
}

void ParticleManagerDeprecated::registerParticleCloth(UsdGeomXformCache& xfCache, uint64_t stageId, omni::fabric::IStageReaderWriter* iStageReaderWriter,
    omni::fabric::StageReaderWriterId stageInProgress, const UsdPrim& prim)
{
    SdfPath path = prim.GetPath();
    omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

    UsdGeomPointBased pointBased(prim);
    VtArray<GfVec3f> points;
    pointBased.GetPointsAttr().Get(&points);
    
    const omni::fabric::UsdStageId ustageId = { uint64_t(stageId) };

    // add tag for hydra to pick this prim up
    constexpr omni::fabric::Type primTypeType(
        omni::fabric::BaseDataType::eTag, 1, 0, omni::fabric::AttributeRole::ePrimTypeName);
    const omni::fabric::Token primType("Deformable");
    iStageReaderWriter->createAttribute(stageInProgress, pathHandle, primType, omni::fabric::TypeC(primTypeType));

    ParticleClothDataDeprecated& pc = mParticleCloths[pathHandle];
    pc.prim = prim;
    pc.numVerts = points.size();
    //printf("+++ Registering GPU ParticleCloth %s numVerts=%zu\n", path.GetText(), pc.numVerts);

    PositionCache::const_iterator fit = mInitialPositions.find(pathHandle.path);
    if (fit == mInitialPositions.end())
    {
        bool resetsXformStack;
        UsdGeomXform(prim).GetLocalTransformation(&pc.initialPrimToParent, &resetsXformStack, UsdTimeCode::Default());

        VtArray<carb::Float3> initPos;
        initPos.resize(points.size());
        for (uint32_t i = 0; i < points.size(); ++i)
        {
            initPos[i] = toFloat3(points[i]);
        }

        mInitialPositions[pathHandle.path] = initPos;
    }

    mIsDirty = true;
}

bool ParticleManagerDeprecated::prepareBuffers(omni::fabric::StageReaderWriter& srw)
{
    IPhysx* iPhysX = carb::getCachedInterface<IPhysx>();

    for (auto& it : mParticleCloths)
    {
        const SdfPath path = omni::fabric::toSdfPath(it.first);

        PxParticleClothBuffer* pc = static_cast<PxParticleClothBuffer*>(iPhysX->getPhysXPtr(path, omni::physx::ePTParticleClothDeprecated));
        if (pc == nullptr)
        {
            mIsDirty = true;
        }
    }

    if (mIsDirty)
    {
        mParticleClothsSet.clear();

        std::vector<omni::fabric::PathC> emptyPaths;

        for (auto& it : mParticleCloths)
        {
            const SdfPath path = omni::fabric::toSdfPath(it.first);

            PxParticleClothBuffer* pc = static_cast<PxParticleClothBuffer*>(iPhysX->getPhysXPtr(path, omni::physx::ePTParticleClothDeprecated));

            // to get the scene - we need to get the simulation owner particle system and call the getScene method there.
            // because particle buffers are not tied to a scene.
            SdfPath particleSystemPath;
            UsdRelationship rel = PhysxSchemaPhysxParticleAPI(it.second.prim).GetParticleSystemRel();
            if (rel)
            {
                SdfPathVector particleSystemPaths;
                rel.GetTargets(&particleSystemPaths);
                if (!particleSystemPaths.empty())
                {
                    particleSystemPath = particleSystemPaths[0];
                }
            }

            PxParticleSystem* ps = static_cast<PxParticleSystem*>(iPhysX->getPhysXPtr(particleSystemPath, omni::physx::ePTParticleSystem));

            if (pc && ps)
            {
                PxScene* scene = ps->getScene();
                if (scene)
                {
                    ParticleClothSetDeprecated& set = mParticleClothsSet[scene];
                    set.mScene = scene;
                    set.mParticleCloths.push_back(&it.second);
                }
            }
            else
                emptyPaths.push_back(it.first);
        }

        for (auto& it : emptyPaths)
        {
            mParticleCloths.erase(it);
        }

        for (auto& it : mParticleClothsSet)
        {

            it.second.prepareBuffers(srw);
        }

        mIsDirty = false;
    }

    return true;
}

 
void ParticleManagerDeprecated::clear()
{
    for (auto& it : mParticleClothsSet)
    {
        it.second.releaseBuffers();
    }
}

void ParticleManagerDeprecated::update(omni::fabric::StageReaderWriter& srw)
{
    carb::settings::ISettings* iSettings = carb::getCachedInterface<carb::settings::ISettings>();
           
    bool updatePoints = iSettings->getAsBool(kSettingFabricUpdatePoints);
    if (updatePoints)
    {
        if (!prepareBuffers(srw))
        {
            return;
        }

        updateParticleCloths(srw);
    }
}

void ParticleManagerDeprecated::updateParticleCloths(omni::fabric::StageReaderWriter& srw)
{
    for (auto& it : mParticleClothsSet)
    {
        if (it.second.mParticleCloths.size() > 0)
        {
            it.second.updateParticleCloths(srw);
        }
    }
}

void ParticleManagerDeprecated::setInitialTransformation(omni::fabric::StageReaderWriter& stage)
{
   const omni::fabric::set<omni::fabric::AttrNameAndType_v2> requiredAll = { omni::fabric::AttrNameAndType_v2(mTypeAppliedSchema, mParticleClothSchemaToken) };
   const omni::fabric::set<omni::fabric::AttrNameAndType_v2> requiredAny = { omni::fabric::AttrNameAndType_v2(mTypeFloat3Array, mPointsToken) };

    omni::fabric::PrimBucketList primBuckets = stage.findPrims(requiredAll, requiredAny);
    size_t bucketCount = primBuckets.bucketCount();
    for (size_t i = 0; i != bucketCount; i++)
    {
        gsl::span<const omni::fabric::Path> paths = stage.getPathArray(primBuckets, i);
        for (const omni::fabric::Path& path : paths)
        {
            // AD: attention, we will get a pointer to the pointer here. not really intuitive.
            carb::Float3* positions = *(stage.getAttributeWr<carb::Float3*>(path, mPointsToken));

            PositionCache::const_iterator fit = mInitialPositions.find(omni::fabric::PathC(path).path);
            if (fit != mInitialPositions.end() && positions)
            {
                for (size_t j = 0; j < fit->second.size(); ++j)
                {
                    positions[j] = fit->second[j];
                }
            }
        }
    }

    mInitialPositions.clear();
}

void ParticleManagerDeprecated::saveToUsd(omni::fabric::StageReaderWriter& stage, UsdStageRefPtr& usdStage)
{
   const omni::fabric::set<omni::fabric::AttrNameAndType_v2> requiredAll = { omni::fabric::AttrNameAndType_v2(mTypeAppliedSchema, mParticleClothSchemaToken) };
   const omni::fabric::set<omni::fabric::AttrNameAndType_v2> requiredAny = { omni::fabric::AttrNameAndType_v2(mTypeFloat3Array, mPointsToken) };

    omni::fabric::PrimBucketList primBuckets = stage.findPrims(requiredAll, requiredAny);
    size_t bucketCount = primBuckets.bucketCount();
    for (size_t i = 0; i != bucketCount; i++)
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
                        points[j] = toVec3f(positions[j]);
                    }
                    pointsAttr.Set(points);
                }
            }
        }
    }
}

} // namespace physx
} // namespace omni

#endif

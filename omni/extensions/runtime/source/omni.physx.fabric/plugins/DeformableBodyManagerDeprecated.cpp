// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "DeformableBodyManagerDeprecated.h"
#include "FabricKernels.h"
#include "FabricManager.h"

#if !CARB_AARCH64

#include <carb/logging/Log.h>
#include <carb/InterfaceUtils.h>
#include <omni/fabric/FabricUSD.h>
#include <carb/settings/ISettings.h>

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxFabric.h>
#include <omni/physx/PhysxTokens.h>
#include <pxr/usd/usdPhysics/tokens.h>
#include <physxSchema/tokens.h>
#include <cudamanager/PxCudaContext.h>

#include <PxPhysicsAPI.h>

#include <cuda.h>

#include <algorithm>
#include <unordered_set>
#include <vector>
#include "driver_types.h"
#include "cuda_runtime_api.h"

#include <common/foundation/TypeCast.h>

using namespace pxr;
using namespace physx;
using namespace carb;

#define DEFORMABLE_BODY_DEBUG   0

namespace omni
{
    namespace physx
    {
        DeformableBodySetDeprecated::DeformableBodySetDeprecated()
        {
            using omni::fabric::AttributeRole;
            using omni::fabric::BaseDataType;

            omni::fabric::IToken* iToken = carb::getCachedInterface<omni::fabric::IToken>();

            mSoftBodySchemaToken = iToken->getHandle("PhysxDeformableBodyAPI");
            mPointsToken = iToken->getHandle("points");
            mTypeFloat3Array = omni::fabric::Type(BaseDataType::eFloat, 3, 1, AttributeRole::ePosition);

            carb::settings::ISettings* iSettings = carb::getCachedInterface<carb::settings::ISettings>();
            mGPUInterop = iSettings->getAsBool(kSettingFabricUseGPUInterop);
        }

        DeformableBodySetDeprecated::~DeformableBodySetDeprecated()
        {
            releaseBuffers();
        }

        void DeformableBodySetDeprecated::memHostAllocAndCheck(PxCudaContext* cudaContext, void** hptr, size_t byteSize, unsigned int flags)
        {
            if (cudaContext)
            {
                PxCUresult result = cudaContext->memHostAlloc(hptr, byteSize, flags);
                if (result != CUDA_SUCCESS)
                {
                    CARB_LOG_ERROR("Failed to allocate host memory!");
                    releaseBuffers();
                }
            }
        }

        void DeformableBodySetDeprecated::memDeviceAllocAndCheck(PxCudaContext* cudaContext, CUdeviceptr* dptr, size_t byteSize)
        {
            if (cudaContext)
            {
                PxCUresult result = cudaContext->memAlloc(dptr, byteSize);
                if (result != CUDA_SUCCESS)
                {
                    CARB_LOG_ERROR("Failed to allocate device memory!");
                    releaseBuffers();
                }
            }
        }

        void DeformableBodySetDeprecated::memcpyHtoDAndCheck(PxCudaContext* cudaContext, CUdeviceptr dstDevice, const void* srcHost, size_t byteSize)
        {
            if (cudaContext)
            {
                PxCUresult result = cudaContext->memcpyHtoD(dstDevice, srcHost, byteSize);
                if (result != CUDA_SUCCESS)
                {
                    CARB_LOG_ERROR("Failed to copy data from host to device!");
                    releaseBuffers();
                }
            }
        }

        void DeformableBodySetDeprecated::releaseBuffers()
        {
            PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

            if (!cudaContextManager)
                return;

            PxScopedCudaLock _lock(*cudaContextManager);

            PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

            if (mNumSds)
            {
                if (!mGPUInterop && mFabricHost)
                {
                    for (uint32_t i = 0; i < mNumSds; ++i)
                    {
                        cudaContext->memFree(mFabricHost[i]);
                        mFabricHost[i] = 0;
                    }
                }

                if (mSoftBodyPointsBufferDev)
                {
                    cudaContext->memFree(mSoftBodyPointsBufferDev);
                    mSoftBodyPointsBufferDev = 0;
                }

                if (mSoftBodyPointsDev)
                {
                    cudaContext->memFree(mSoftBodyPointsDev);
                    mSoftBodyPointsDev = 0;
                }

                if (mSoftBodyIndicesDev)
                {
                    cudaContext->memFree(mSoftBodyIndicesDev);
                    mSoftBodyIndicesDev = 0;
                }

                if (mSoftBodySizesDev)
                {
                    cudaContext->memFree(mSoftBodySizesDev);
                    mSoftBodySizesDev = 0;
                }

                if (mSoftBodyMtxDevPtrs)
                {
                    cudaContext->memFree(mSoftBodyMtxDevPtrs);
                    mSoftBodyMtxDevPtrs = 0;
                }

                if (mFabricPointsDev)
                {
                    cudaContext->memFree(mFabricPointsDev);
                    mFabricPointsDev = 0;
                }

                if (mOffsetsDev)
                {
                    cudaContext->memFree(mOffsetsDev);
                    mOffsetsDev = 0;
                }

                if (mWorldToSoftbodyHost)
                {
                    cudaContext->memFreeHost(mWorldToSoftbodyHost);
                    mWorldToSoftbodyHost = 0;
                }

                if (mOffsetHost)
                {
                    cudaContext->memFreeHost(mOffsetHost);
                    mOffsetHost = 0;
                }

                if (mFabricHost)
                {
                    cudaContext->memFreeHost(mOffsetHost);
                    mFabricHost = 0;
                }

                if (mSoftBodyPointsDevPtrsHost)
                {
                    cudaContext->memFreeHost(mSoftBodyPointsDevPtrsHost);
                    mSoftBodyPointsDevPtrsHost = 0;
                }

                if (mSoftBodyIndicesHost)
                {
                    cudaContext->memFreeHost(mSoftBodyIndicesHost);
                    mSoftBodyIndicesHost = 0;
                }

                if (mSoftBodySizesHost)
                {
                    cudaContext->memFreeHost(mSoftBodySizesHost);
                    mSoftBodySizesHost = 0;
                }

                if (mFabricPointsCPU)
                {
                    cudaContext->memFreeHost(mFabricPointsCPU);
                    mFabricPointsCPU = nullptr;
                }

                if (mPointsCopyEvent)
                {
                    cudaContext->eventDestroy(mPointsCopyEvent);
                    mPointsCopyEvent = nullptr;
                }
            }
        }

        void DeformableBodySetDeprecated::prepareBuffers(omni::fabric::StageReaderWriter& srw)
        {

            mNumSds = (PxU32)mSoftBodies.size();
            if (mNumSds)
            {
                PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();
                
                if (!cudaContextManager)
                    return;

                PxScopedCudaLock _lock(*cudaContextManager);

                PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

                releaseBuffers();

                IPhysx* iPhysX = carb::getCachedInterface<IPhysx>();

                mMaxPointSize = 0;
                mTotalPointSize = 0;
         
                memHostAllocAndCheck(cudaContext, (void**)&mSoftBodyIndicesHost, sizeof(PxU32) * mNumSds, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
                memHostAllocAndCheck(cudaContext, (void**)&mSoftBodySizesHost, sizeof(PxU32) * mNumSds, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
                if (!mSoftBodyIndicesHost || !mSoftBodySizesHost)
                    return;

                PxU32 count = 0;
                for (auto& it : mSoftBodies)
                {
                    const SdfPath& path = it->prim.GetPath();

                    PxSoftBody* sd = static_cast<PxSoftBody*>(iPhysX->getPhysXPtr(path, omni::physx::ePTSoftBodyDeprecated));

                    PxU32 pointsSize = (PxU32)(it->numVerts + it->numCollVerts) * sizeof(float) * 4;

                    mTotalPointSize += pointsSize;

                    if (mMaxPointSize < pointsSize)
                    {
                        mMaxPointSize = pointsSize;
                    }
                    it->idx = sd->getGpuSoftBodyIndex();

                    mSoftBodyIndicesHost[count] = it->idx;
                    mSoftBodySizesHost[count] = pointsSize;
                    count++;
                }

                memDeviceAllocAndCheck(cudaContext, &mSoftBodyMtxDevPtrs, sizeof(float) * 16 * mNumSds);
                memDeviceAllocAndCheck(cudaContext, &mFabricPointsDev, sizeof(CUdeviceptr) * mNumSds);
                memDeviceAllocAndCheck(cudaContext, &mOffsetsDev, sizeof(int2) * mNumSds);
                memDeviceAllocAndCheck(cudaContext, &mSoftBodyPointsBufferDev, mTotalPointSize);
                memDeviceAllocAndCheck(cudaContext, &mSoftBodyPointsDev, mNumSds * sizeof(CUdeviceptr));
                memDeviceAllocAndCheck(cudaContext, &mSoftBodyIndicesDev, mNumSds * sizeof(PxU32));
                memDeviceAllocAndCheck(cudaContext, &mSoftBodySizesDev, mNumSds * sizeof(PxU32));
                if (!mSoftBodyMtxDevPtrs || !mFabricPointsDev || !mOffsetsDev || !mSoftBodyPointsBufferDev || !mSoftBodyPointsDev || !mSoftBodyIndicesDev || !mSoftBodySizesDev)
                    return;

                // synchronization event
                cudaContext->eventCreate(&mPointsCopyEvent, CU_EVENT_DISABLE_TIMING);

                //printf("~*~*~*   %u softBodies, maxSize=%u\n", mNumSds, mMaxPointSize);

            // printf("~*~*~*\n");

                memHostAllocAndCheck(cudaContext, (void**)&mWorldToSoftbodyHost, sizeof(GfMatrix4f) * mNumSds, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
                memHostAllocAndCheck(cudaContext, (void**)&mOffsetHost, sizeof(int2) * mNumSds, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
                memHostAllocAndCheck(cudaContext, (void**)&mFabricHost, sizeof(CUdeviceptr) * mNumSds, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
                memHostAllocAndCheck(cudaContext, (void**)&mSoftBodyPointsDevPtrsHost, sizeof(CUdeviceptr) * mNumSds, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
                memHostAllocAndCheck(cudaContext, (void**)&mFabricPointsCPU, sizeof(float3*) * mNumSds, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
                if (!mWorldToSoftbodyHost || !mOffsetHost || !mFabricHost || !mSoftBodyPointsDevPtrsHost || !mFabricPointsCPU)
                    return;

                mMaxSkinMeshPoints = 0;
                CUdeviceptr pointsDev = mSoftBodyPointsBufferDev;

                for (PxU32 i = 0; i < mSoftBodies.size(); ++i)
                {
                    DeformableBodyDataDeprecated* it = mSoftBodies[i];

                    SdfPath path = it->prim.GetPath();
                    omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

                    GfMatrix4f parentToWorld = GfMatrix4f(UsdGeomXform(it->prim).ComputeParentToWorldTransform(UsdTimeCode::Default()));
                    GfMatrix4f worldToSoftbody = (GfMatrix4f(it->initialPrimToParent) * parentToWorld).GetInverse();

                    mWorldToSoftbodyHost[i] = worldToSoftbody;
                    mOffsetHost[i].x = (int)it->numVerts; //number of skin mesh
                    mOffsetHost[i].y = (int)it->numCollVerts;//the offset to skin mesh verts in soft body position array

                    srw.createPrim(pathHandle);
                    srw.createAttribute(pathHandle, mPointsToken, mTypeFloat3Array);
                    srw.setArrayAttributeSize(pathHandle, mPointsToken, it->numVerts);

                    // prepare a device staging buffer if CPU interop
                    if (!mGPUInterop)
                    {
                        CUdeviceptr pointsGPU;
                        memDeviceAllocAndCheck(cudaContext, &pointsGPU, it->numVerts * sizeof(float3));
                        mFabricHost[i] = pointsGPU;
                    }

                    mSoftBodyPointsDevPtrsHost[i] = pointsDev;
                    pointsDev += mSoftBodySizesHost[i];


#if DEFORMABLE_BODY_DEBUG
                    cudaPointerAttributes attributes;
                    cudaPointerGetAttributes(&attributes, (void*)mFabricHost[i]);

                    printf("Fabric Device pointer %p on device %i\n", (void*)mFabricHost[i], attributes.device);
#endif
                    if (mMaxSkinMeshPoints < it->numVerts)
                    {
                        mMaxSkinMeshPoints = (int)it->numVerts;
                    }
                }

                memcpyHtoDAndCheck(cudaContext, mSoftBodyPointsDev, mSoftBodyPointsDevPtrsHost, mNumSds * sizeof(CUdeviceptr));
                memcpyHtoDAndCheck(cudaContext, mSoftBodyIndicesDev, mSoftBodyIndicesHost, mNumSds * sizeof(PxU32));
                memcpyHtoDAndCheck(cudaContext, mSoftBodySizesDev, mSoftBodySizesHost, mNumSds * sizeof(PxU32));
                memcpyHtoDAndCheck(cudaContext, mSoftBodyMtxDevPtrs, mWorldToSoftbodyHost, sizeof(float) * 16 * mNumSds);
                memcpyHtoDAndCheck(cudaContext, mOffsetsDev, mOffsetHost, sizeof(int2) * mNumSds);
                memcpyHtoDAndCheck(cudaContext, mFabricPointsDev, mFabricHost, sizeof(CUdeviceptr) * mNumSds);
            }
        }

        void DeformableBodySetDeprecated::fetchSoftBodyData()
        {
            PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

            if (!cudaContextManager)
                return;

            PxScopedCudaLock _lock(*cudaContextManager);

            PxCudaContext* cudaContext = cudaContextManager->getCudaContext();
            //
            // launch PhysX kernels to copy state data
            //
            mScene->copySoftBodyData((void**)mSoftBodyPointsDev, (void*)mSoftBodySizesDev, (void*)mSoftBodyIndicesDev,
                PxSoftBodyGpuDataFlag::eTET_POSITION_INV_MASS, mNumSds,
                mMaxPointSize, mPointsCopyEvent);

            cudaContext->streamWaitEvent(nullptr, mPointsCopyEvent, 0);

            //cudaPointerAttributes attributesSoftBody;
            //cudaPointerGetAttributes(&attributesSoftBody, (void*)mSoftBodyPointsDev);

            //cudaPointerAttributes attributesFabric;
            //cudaPointerGetAttributes(&attributesFabric, (void*)mFabricHost[0]);

            //if (attributesSoftBody.device != attributesFabric.device)
            //{
            //    cudaSetDevice(attributesFabric.device);

            //    cuMemAlloc(&mSoftBodyPointsFabricBufferDev, mTotalPointSize);
            //    cuMemcpyDtoD(mSoftBodyPointsFabricBufferDev, mSoftBodyPointsBufferDev, mTotalPointSize);

            //    CUdeviceptr pointsDev = mSoftBodyPointsFabricBufferDev;
            //    mSoftBodyPointsDevPtrs.clear();
            //    // SoftBody CUDA work
            //    for (auto size : mSoftBodySizes)
            //    {

            //        //result = cudaContext->memAlloc(&pointsDev, size);

            //        mSoftBodyPointsDevPtrs.push_back(pointsDev);

            //        pointsDev += size;
            //    }

            //    cuMemAlloc(&mSoftBodyPointsFabricDev, mNumSds * sizeof(PxU32));
            //    cuMemcpyHtoD(mSoftBodyPointsFabricDev, mSoftBodyPointsDevPtrs.data(),
            //        mSoftBodyPointsDevPtrs.size() * sizeof(CUdeviceptr));
            //}
            //else
            {
                mSoftBodyPointsFabricDev = mSoftBodyPointsDev;
            }
        }


        void DeformableBodySetDeprecated::updateSoftBodies(omni::fabric::StageReaderWriter& srw) const
        {
            PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

            if (!cudaContextManager)
                return;

            PxScopedCudaLock _lock(*cudaContextManager);

            PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

            for (uint32_t i = 0; i < mNumSds; ++i)
            {
                DeformableBodyDataDeprecated* it = mSoftBodies[i];

                SdfPath path = it->prim.GetPath();
                omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

                if (mGPUInterop)
                {
                    // If we're using GPU interop then get a writeable GPU pointer from Fabric
                    float3** cudaPtr = srw.getAttributeWrGpu<float3*>(pathHandle, mPointsToken);
                    mFabricHost[i] = (CUdeviceptr)*cudaPtr;
                }
                else
                {
                    // If we're not using GPU interop then get a writeable CPU pointer from Fabric
                    float3** cpuPtr = srw.getAttributeWr<float3*>(pathHandle, mPointsToken);
                    mFabricPointsCPU[i] = *cpuPtr;
                }
            }

            if (mGPUInterop)
            {
                PxCUresult result = cudaContext->memcpyHtoD(mFabricPointsDev, mFabricHost, sizeof(CUdeviceptr) * mNumSds);
            }

            convertVec4fToVec3fBlock((float3**)mFabricPointsDev, (const float4**)mSoftBodyPointsFabricDev, (const int2*)mOffsetsDev, (const float*)mSoftBodyMtxDevPtrs, mSoftBodies.size(), mMaxSkinMeshPoints);

            if (!mGPUInterop)
            {
                // Read data from the GPU and populate the CPU-side Fabric buffers
                for (PxU32 i = 0; i < mSoftBodies.size(); ++i)
                {
                    DeformableBodyDataDeprecated* it = mSoftBodies[i];
                    cudaContext->memcpyDtoH((void*)mFabricPointsCPU[i], mFabricHost[i], it->numVerts * sizeof(float3));
                }
            }

#if DEFORMABLE_BODY_DEBUG
           cudaPointerAttributes attributes;
           cudaPointerGetAttributes(&attributes, (void*)mSoftBodyPointsDev);

           printf("SoftBody Device pointer %p on device %i\n", (void*)mSoftBodyPointsFabricDev, attributes.device);
           PxCUresult result = cudaContext->streamSynchronize(0);

           if (result != CUDA_SUCCESS)
           {
              int bob = 0;
           }
#endif
        }

        DeformableBodyManagerDeprecated::DeformableBodyManagerDeprecated()
        {

            using omni::fabric::AttributeRole;
            using omni::fabric::BaseDataType;

            mTypeAppliedSchema = omni::fabric::Type(BaseDataType::eTag, 1, 0, AttributeRole::eAppliedSchema);

            mTypeFloat3 = omni::fabric::Type(BaseDataType::eFloat, 3, 0, AttributeRole::eNone);
            mTypeDouble3 = omni::fabric::Type(BaseDataType::eDouble, 3, 0, AttributeRole::eNone);
            mTypeQuat = omni::fabric::Type(BaseDataType::eFloat, 4, 0, AttributeRole::eQuaternion);
            mTypeFloat3Array = omni::fabric::Type(BaseDataType::eFloat, 3, 1, AttributeRole::ePosition);

            omni::fabric::Token::iToken = getFramework()->tryAcquireInterface<omni::fabric::IToken>();

            mWorldMatrixToken = omni::fabric::Token::iToken->getHandle(gWorldMatrixTokenString);
            mSoftBodySchemaToken = omni::fabric::Token::iToken->getHandle("PhysxDeformableBodyAPI");
            mPointsToken = omni::fabric::Token::iToken->getHandle("points");

            mPhysxSimulationInterface = carb::getCachedInterface<omni::physx::IPhysxSimulation>();
        }

        DeformableBodyManagerDeprecated::~DeformableBodyManagerDeprecated()
        {
        }

        void DeformableBodyManagerDeprecated::registerSoftBody(UsdGeomXformCache& xfCache, uint64_t stageId, omni::fabric::IStageReaderWriter* iStageReaderWriter,
            omni::fabric::StageReaderWriterId stageInProgress, const UsdPrim& prim)
        {
            SdfPath path = prim.GetPath();
            omni::fabric::PathC pathHandle = omni::fabric::asInt(path);

            UsdGeomPointBased pointBased(prim);
            VtArray<GfVec3f> points;
            pointBased.GetPointsAttr().Get(&points);
            VtArray<GfVec3f> collPoints;
            PhysxSchemaPhysxDeformableBodyAPI(prim).GetCollisionPointsAttr().Get(&collPoints);

            const omni::fabric::UsdStageId ustageId = { uint64_t(stageId) };

            // add tag for hydra to pick this prim up
            constexpr omni::fabric::Type primTypeType(
                omni::fabric::BaseDataType::eTag, 1, 0, omni::fabric::AttributeRole::ePrimTypeName);
            const omni::fabric::Token primType("Deformable");
            iStageReaderWriter->createAttribute(stageInProgress, pathHandle, primType, omni::fabric::TypeC(primTypeType));

            //const GfMatrix4d worldPose = xfCache.GetLocalToWorldTransform(prim);
            //const GfTransform tr(worldPose);
            //const GfVec3d wPos = tr.GetTranslation();
            //Float3 pos = { float(wPos[0]), float(wPos[1]), float(wPos[2]) };
            //const GfQuatd wRot = tr.GetRotation().GetQuat();
            //Float4 orient = { float(wRot.GetImaginary()[0]), float(wRot.GetImaginary()[1]), float(wRot.GetImaginary()[2]),
            //                  float(wRot.GetReal()) };
            //const float transf[] = { orient.x, orient.y, orient.z, orient.w, pos.x, pos.y, pos.z }; // must match memory layout
            //                                                                                        // of PxTransform

            //// create position attribute (somewhat abused, we're saving a 7-element pose in it)
            //constexpr omni::fabric::Type worldPosType(omni::fabric::BaseDataType::eFloat, 1, 1);
            //iStageReaderWriter->createAttribute(stageInProgress, pathHandle, mWorldPosToken, uint32_t(worldPosType));
            //iStageReaderWriter->setArrayAttributeSize(stageInProgress, pathHandle, mWorldPosToken, 7);
            //const omni::fabric::SpanC worldPosBuf = iStageReaderWriter->getArrayAttributeWr(stageInProgress, pathHandle, mWorldPosToken);
            //std::copy(transf, transf + 7, reinterpret_cast<float*>(worldPosBuf.ptr));

            DeformableBodyDataDeprecated& sd = mSoftBodies[pathHandle];
            sd.prim = prim;
            sd.numVerts = points.size();
            sd.numCollVerts = collPoints.size();
           // printf("+++ Registering GPU SoftBody %s numVerts=%zu numCollVerts=%zu\n", path.GetText(), sd.numVerts, sd.numCollVerts);

            PositionCache::const_iterator fit = mInitialPositions.find(pathHandle.path);
            if (fit == mInitialPositions.end())
            {
                bool resetsXformStack;
                UsdGeomXform(prim).GetLocalTransformation(
                    &sd.initialPrimToParent, &resetsXformStack, UsdTimeCode::Default());

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

        bool DeformableBodyManagerDeprecated::prepareBuffers(omni::fabric::StageReaderWriter& srw)
        {
            IPhysx* iPhysX = carb::getCachedInterface<IPhysx>();

            for (auto& it : mSoftBodies)
            {
                const SdfPath path = omni::fabric::toSdfPath(it.first);

                PxSoftBody* sd = static_cast<PxSoftBody*>(iPhysX->getPhysXPtr(path, omni::physx::ePTSoftBodyDeprecated));
                if (sd == nullptr)
                {
                    mIsDirty = true;
                }
            }

            if (mIsDirty)
            {
                mSoftBodiesSet.clear();

                std::vector<omni::fabric::PathC> emptyPaths;

                for (auto& it : mSoftBodies)
                {
                    const SdfPath path = omni::fabric::toSdfPath(it.first);

                    PxSoftBody* sd = static_cast<PxSoftBody*>(iPhysX->getPhysXPtr(path, omni::physx::ePTSoftBodyDeprecated));
                    if (sd)
                    {
                        PxScene* scene = sd->getScene();
                        if (scene)
                        {
                            DeformableBodySetDeprecated& set = mSoftBodiesSet[scene];
                            set.mScene = scene;
                            set.mSoftBodies.push_back(&it.second);
                            set.mMaxPointSize = 0;
                            set.mNumSds = 0;
                        }
                    }
                    else
                        emptyPaths.push_back(it.first);
                }

                for (auto& it : emptyPaths)
                {
                    mSoftBodies.erase(it);
                }

                for (auto& it : mSoftBodiesSet)
                {

                    it.second.prepareBuffers(srw);
                }

                mIsDirty = false;
            }

            return true;
        }


        void DeformableBodyManagerDeprecated::clear()
        {
            for (auto& it : mSoftBodiesSet)
            {
                it.second.releaseBuffers();
            }
        }

        void DeformableBodyManagerDeprecated::update(omni::fabric::StageReaderWriter& srw)
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

                updateSoftBodies(srw);
            }
        }



        void DeformableBodyManagerDeprecated::updateSoftBodies(omni::fabric::StageReaderWriter& srw)
        {
            for (auto& it : mSoftBodiesSet)
            {
                if (it.second.mNumSds > 0)
                {
                    it.second.fetchSoftBodyData();
                    it.second.updateSoftBodies(srw);
                }
            }
        }

        void DeformableBodyManagerDeprecated::setInitialTransformation(omni::fabric::StageReaderWriter& stage)
        {
            const omni::fabric::set<omni::fabric::AttrNameAndType_v2> requiredAll = { omni::fabric::AttrNameAndType_v2(mTypeAppliedSchema, mSoftBodySchemaToken) };
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

        void DeformableBodyManagerDeprecated::saveToUsd(omni::fabric::StageReaderWriter& stage, UsdStageRefPtr& usdStage)
        {
            const omni::fabric::set<omni::fabric::AttrNameAndType_v2> requiredAll = { omni::fabric::AttrNameAndType_v2(mTypeAppliedSchema, mSoftBodySchemaToken) };
            const omni::fabric::set<omni::fabric::AttrNameAndType_v2> requiredAny = { omni::fabric::AttrNameAndType_v2(mTypeFloat3Array, mPointsToken) };

            omni::fabric::PrimBucketList primBuckets = stage.findPrims(requiredAll, requiredAny);
            size_t bucketCount = primBuckets.bucketCount();
            for (size_t i = 0; i != bucketCount; i++)
            {
                gsl::span<const omni::fabric::Path> paths = stage.getPathArray(primBuckets, i);
                for (const omni::fabric::Path& path : paths)
                {
                    carb::Float3* positions = *(stage.getAttributeRd<carb::Float3*>(path, mPointsToken));

                    const SdfPath primPath = omni::fabric::toSdfPath(path);
                    UsdPrim prim = usdStage->GetPrimAtPath(primPath);

                    size_t size = mSoftBodies[omni::fabric::PathC(path)].numVerts;
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

// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Defines.h>

#if !CARB_AARCH64

#    include <omni/fabric/SimStageWithHistory.h>

#    include <foundation/PxTransform.h>
#    include <cudamanager/PxCudaTypes.h>

#    include <unordered_map>
#    include <vector_types.h>

namespace physx
{
class PxScene;
class PxCudaContext;
} // namespace physx

namespace omni
{
namespace physx
{
struct IPhysxSimulation;

using PositionCache = std::unordered_map<uint64_t, pxr::VtArray<carb::Float3>>;

struct DeformableBodyDataDeprecated
{
    uint32_t idx = 0xffffffff; // softbody GPU index
    size_t numVerts = 0;
    size_t numCollVerts = 0;
    pxr::UsdPrim prim;
    pxr::GfMatrix4d initialPrimToParent;
};

// Each scene has a list of deformable bodies
class DeformableBodySetDeprecated
{
public:
    DeformableBodySetDeprecated();
    ~DeformableBodySetDeprecated();

    void memHostAllocAndCheck(::physx::PxCudaContext* cudaContext, void** hptr, size_t byteSize, unsigned int flags);
    void memDeviceAllocAndCheck(::physx::PxCudaContext* cudaContext, CUdeviceptr* dptr, size_t byteSize);
    void memcpyHtoDAndCheck(::physx::PxCudaContext* cudaContext,
                            CUdeviceptr dstDevice,
                            const void* srcHost,
                            size_t byteSize);
    void releaseBuffers();
    void prepareBuffers(omni::fabric::StageReaderWriter& srw);
    void fetchSoftBodyData();
    void updateSoftBodies(omni::fabric::StageReaderWriter& srw) const;

    uint32_t mNumSds = 0;
    uint32_t mMaxPointSize = 0;
    uint32_t mTotalPointSize = 0;
    uint32_t mMaxSkinMeshPoints = 0;


    // std::vector<CUdeviceptr> mSoftBodyPointsDevPtrs;
    CUdeviceptr mSoftBodyPointsBufferDev = 0;
    CUdeviceptr mSoftBodyPointsFabricBufferDev = 0;
    CUdeviceptr mSoftBodyMtxDevPtrs = 0;
    CUdeviceptr mFabricPointsDev = 0;
    CUdeviceptr mOffsetsDev = 0;
    CUdeviceptr mSoftBodyPointsDev = 0;
    CUdeviceptr mSoftBodyPointsFabricDev = 0;
    CUdeviceptr mSoftBodyIndicesDev = 0;
    CUdeviceptr mSoftBodySizesDev = 0;
    CUevent mPointsCopyEvent = nullptr;

    // pinned host memory
    pxr::GfMatrix4f* mWorldToSoftbodyHost = 0;
    int2* mOffsetHost = 0;
    CUdeviceptr* mFabricHost = 0;
    CUdeviceptr* mSoftBodyPointsDevPtrsHost = 0;
    ::physx::PxU32* mSoftBodyIndicesHost = 0;
    ::physx::PxU32* mSoftBodySizesHost = 0;


    // std::vector<::physx::PxU32> mSoftBodySizes;
    std::vector<DeformableBodyDataDeprecated*> mSoftBodies;

    omni::fabric::Token mSoftBodySchemaToken;
    omni::fabric::Token mPointsToken;

    omni::fabric::Type mTypeFloat3Array;

    ::physx::PxScene* mScene = nullptr;

    bool mGPUInterop = false;
    float3** mFabricPointsCPU;
};


class DeformableBodyManagerDeprecated
{
public:
    DeformableBodyManagerDeprecated();
    ~DeformableBodyManagerDeprecated();

    void registerSoftBody(pxr::UsdGeomXformCache& xfCache,
                          uint64_t stageId,
                          omni::fabric::IStageReaderWriter* iStageReaderWriter,
                          omni::fabric::StageReaderWriterId StageReaderWriter,
                          const pxr::UsdPrim& prim);

    void update(omni::fabric::StageReaderWriter& srw);
    void setInitialTransformation(omni::fabric::StageReaderWriter& srw);
    void saveToUsd(omni::fabric::StageReaderWriter& srw, pxr::UsdStageRefPtr& usdStage);

    void clear();

private:
    bool prepareBuffers(omni::fabric::StageReaderWriter& srw);


    void updateSoftBodies(omni::fabric::StageReaderWriter& srw);
    void fetchSoftBodyData();


    omni::fabric::Type mTypeAppliedSchema;
    omni::fabric::Type mTypeFloat3;
    omni::fabric::Type mTypeDouble3;
    omni::fabric::Type mTypeQuat;
    omni::fabric::Type mTypeFloat3Array;

    omni::fabric::Token mPointsToken;
    omni::fabric::Token mSoftBodySchemaToken;

    omni::fabric::Token mWorldMatrixToken;
    std::unordered_map<omni::fabric::PathC, DeformableBodyDataDeprecated> mSoftBodies;

    std::unordered_map<::physx::PxScene*, DeformableBodySetDeprecated> mSoftBodiesSet;

    omni::physx::IPhysxSimulation* mPhysxSimulationInterface;

    PositionCache mInitialPositions;

    bool mIsDirty = false;
};
} // namespace physx
} // namespace omni

#endif

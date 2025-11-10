// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>

#if !CARB_AARCH64

#    include <omni/fabric/IFabric.h>
#    include <omni/fabric/SimStageWithHistory.h>

#    include <PxPhysicsAPI.h>

#    include <vector_types.h>
#    include "FabricKernels.h"

#    include <private/omni/physx/IPhysxPrivate.h>

namespace omni
{
namespace physx
{
using PositionCache = std::unordered_map<uint64_t, pxr::VtArray<carb::Float3>>;

struct VolumeDeformableBody
{
    InternalVolumeDeformableBodyData data;

    std::unordered_map<omni::fabric::PathC, float3*> stagingPointsDevMap;
    std::unordered_map<omni::fabric::PathC, float3*> fabricPointsCPUMap;

    std::vector<pxr::UsdPrim> skinMeshPrims;

    bool hasCollMesh = false;

    ::physx::PxDeformableVolume* physXPtr = nullptr;
};

// Each scene has a list of deformable bodies
class VolumeDeformableBodySet
{
public:
    VolumeDeformableBodySet();
    ~VolumeDeformableBodySet();

    void releaseBuffers();
    void prepareBuffers(omni::fabric::StageReaderWriter& srw);
    void updateInternalVolumeDeformableBodyData();
    void updateDeformableBodies(omni::fabric::StageReaderWriter& srw);

    void createFabricAttribute(omni::fabric::StageReaderWriter& srw,
                               ::physx::PxCudaContextManager* cudaContextManager,
                               pxr::SdfPath path,
                               uint32_t numVertices,
                               std::unordered_map<omni::fabric::PathC, float3*>& stagingPointsDevMap);
    void prepareInteropData(omni::fabric::StageReaderWriter& srw,
                            pxr::SdfPath path,
                            DeformableBodyGPUData& gpuData,
                            void* src,
                            uint32_t numVertices,
                            pxr::GfMatrix4f worldToDeformableSurface,
                            std::unordered_map<omni::fabric::PathC, float3*>& stagingPointsDevMap,
                            std::unordered_map<omni::fabric::PathC, float3*>& fabricPointsCPUMap, const int srcPointsElemSize);

    uint32_t mNumFabricAttributes = 0;
    uint32_t mMaxPoints = 0;

    // AD - we read the data needed for GPU data transfer directly using pinned memory.
    // We do this because we anyway need to transfer both the PhysX & Fabric pointers every
    // update step, so we might as well read them directly. Theoretically, the transform could
    // even change as well but we don't listen to transform changes from fabric yet.
    DeformableBodyGPUData* mDeformableBodyGPUDataH = nullptr; // host mirror of GPU data

    std::vector<VolumeDeformableBody*> mVolumeDeformableBodies;

    ::physx::PxScene* mScene = nullptr;

    CUstream mDeformableBodyCopyStream = nullptr;
    CUevent mPointsCopyEvent = nullptr;

private:
    omni::fabric::Token mPointsToken;
    omni::fabric::Type mTypeFloat3Array;
    bool mGPUInterop = false;
};

class VolumeDeformableBodyManager
{
public:
    VolumeDeformableBodyManager();
    ~VolumeDeformableBodyManager();

    void registerDeformableBody(pxr::UsdGeomXformCache& xfCache,
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
    void updateVolumeDeformableBodies(omni::fabric::StageReaderWriter& srw);

    omni::fabric::Type mTypeAppliedSchema;
    omni::fabric::Type mTypeFloat3;
    omni::fabric::Type mTypeFloat3Array;

    omni::fabric::Token mPointsToken;

    std::unordered_map<omni::fabric::PathC, VolumeDeformableBody> mVolumeDeformableBodies;
    std::unordered_map<::physx::PxScene*, VolumeDeformableBodySet> mVolumeDeformableBodySets;

    PositionCache mInitialPositions;

    bool mIsDirty = false;
};
} // namespace physx
} // namespace omni

#endif

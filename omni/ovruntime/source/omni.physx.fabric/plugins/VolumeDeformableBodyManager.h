// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>

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
using PositionCache = std::unordered_map<omni::fabric::Path, PXR_NS::VtArray<PXR_NS::GfVec3f>>;

struct VolumeDeformableBody
{
    InternalVolumeDeformableBodyData data;

    std::unordered_map<omni::fabric::Path, float3*> stagingPointsDevMap;
    std::unordered_map<omni::fabric::Path, float3*> fabricPointsCPUMap;

    std::vector<PXR_NS::UsdPrim> skinMeshPrims;

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
                               PXR_NS::SdfPath path,
                               uint32_t numVertices,
                               std::unordered_map<omni::fabric::Path, float3*>& stagingPointsDevMap);
    bool prepareInteropData(omni::fabric::StageReaderWriter& srw,
                            PXR_NS::SdfPath path,
                            DeformableBodyGPUData& gpuData,
                            void* src,
                            uint32_t numVertices,
                            PXR_NS::GfMatrix4f worldToDeformableSurface,
                            std::unordered_map<omni::fabric::Path, float3*>& stagingPointsDevMap,
                            std::unordered_map<omni::fabric::Path, float3*>& fabricPointsCPUMap, const int srcPointsElemSize);

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
    omni::fabric::FabricId mFabricId;
};

class VolumeDeformableBodyManager
{
public:
    VolumeDeformableBodyManager();
    ~VolumeDeformableBodyManager();

    void registerDeformableBody(PXR_NS::UsdGeomXformCache& xfCache,
                                uint64_t stageId,
                                omni::fabric::IStageReaderWriter* iStageReaderWriter,
                                omni::fabric::StageReaderWriterId StageReaderWriter,
                                const PXR_NS::UsdPrim& prim);

    void update(omni::fabric::StageReaderWriter& srw);
    void setInitialTransformation(omni::fabric::StageReaderWriter& srw);
    void saveToUsd(omni::fabric::StageReaderWriter& srw, PXR_NS::UsdStageRefPtr& usdStage);

    void clear();

    // Marks the manager dirty so that the next update() call rebuilds mVolumeDeformableBodySets.
    // Should be invoked when a PhysX deformable volume is destroyed externally so that stale
    // physXPtr values are not dereferenced.
    void setIsDirty()
    {
        mIsDirty = true;
    }

private:
    bool prepareBuffers(omni::fabric::StageReaderWriter& srw);
    void updateVolumeDeformableBodies(omni::fabric::StageReaderWriter& srw);

    omni::fabric::Type mTypeAppliedSchema;
    omni::fabric::Type mTypeFloat3;
    omni::fabric::Type mTypeFloat3Array;

    omni::fabric::Token mPointsToken;

    std::unordered_map<omni::fabric::Path, VolumeDeformableBody> mVolumeDeformableBodies;
    std::unordered_map<::physx::PxScene*, VolumeDeformableBodySet> mVolumeDeformableBodySets;

    PositionCache mInitialPositions;

    bool mIsDirty = false;
};
} // namespace physx
} // namespace omni

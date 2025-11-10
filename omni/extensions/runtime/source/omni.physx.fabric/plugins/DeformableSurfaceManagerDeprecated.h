// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

namespace physx
{
class PxScene;
}

namespace omni
{
namespace physx
{
struct IPhysxSimulation;

using PositionCache = std::unordered_map<uint64_t, pxr::VtArray<carb::Float3>>;

struct DeformableSurfaceDataDeprecated
{
    size_t numVerts = 0;
    pxr::UsdPrim prim;
    pxr::GfMatrix4d initialPrimToParent;
    float3* stagingPointsDev = nullptr; // device memory - used for CPU interop for intermediate results after
                                        // transform.
    float3* fabricPointsCPU = nullptr; // host memory, stores the pointer to CPU fabric bucket for this prim
    ::physx::PxDeformableSurface* physXPtr = nullptr;
    pxr::GfMatrix4d worldToDeformableSurface;
};

// Each scene has a list of deformable surfaces
class DeformableSurfaceSetDeprecated
{
public:
    DeformableSurfaceSetDeprecated();
    ~DeformableSurfaceSetDeprecated();

    void releaseBuffers();
    void prepareBuffers(omni::fabric::StageReaderWriter& srw);
    void updateDeformableSurfaces(omni::fabric::StageReaderWriter& srw) const;

    uint32_t mMaxPoints = 0;

    // AD - we read the data needed for GPU data transfer directly using pinned memory.
    // We do this because we anyway need to transfer both the PhysX & Fabric pointers every
    // update step, so we might as well read them directly. Theoretically, the transform could
    // even change as well but we don't listen to transform changes from fabric yet.
    DeformableSurfaceGPUDataDeprecated* mDeformableSurfaceGPUDataH = nullptr; // host mirror of GPU data

    std::vector<DeformableSurfaceDataDeprecated*> mDeformableSurfaces;

    ::physx::PxScene* mScene = nullptr;

private:
    omni::fabric::Token mPointsToken;
    omni::fabric::Type mTypeFloat3Array;
    bool mGPUInterop = false;
};


class DeformableSurfaceManagerDeprecated
{
public:
    DeformableSurfaceManagerDeprecated();
    ~DeformableSurfaceManagerDeprecated();

    void registerDeformableSurface(pxr::UsdGeomXformCache& xfCache,
                                   uint64_t stageId,
                                   omni::fabric::IStageReaderWriter* iStageReaderWriter,
                                   omni::fabric::StageReaderWriterId stageInProgress,
                                   const pxr::UsdPrim& prim);

    void update(omni::fabric::StageReaderWriter& srw);
    void setInitialTransformation(omni::fabric::StageReaderWriter& srw);
    void saveToUsd(omni::fabric::StageReaderWriter& srw, pxr::UsdStageRefPtr& usdStage);

    void clear();

private:
    bool prepareBuffers(omni::fabric::StageReaderWriter& srw);
    void updateDeformableSurfaces(omni::fabric::StageReaderWriter& srw);

    omni::fabric::Type mTypeAppliedSchema;
    omni::fabric::Type mTypeFloat3;
    omni::fabric::Type mTypeFloat3Array;

    omni::fabric::Token mPointsToken;
    omni::fabric::Token mDeformableSurfaceSchemaToken;

    std::unordered_map<omni::fabric::PathC, DeformableSurfaceDataDeprecated> mDeformableSurfaces;
    std::unordered_map<::physx::PxScene*, DeformableSurfaceSetDeprecated> mDeformableSurfacesSet;

    omni::physx::IPhysxSimulation* mPhysxSimulationInterface = nullptr;

    PositionCache mInitialPositions;

    bool mIsDirty = false;
};
} // namespace physx
} // namespace omni

#endif

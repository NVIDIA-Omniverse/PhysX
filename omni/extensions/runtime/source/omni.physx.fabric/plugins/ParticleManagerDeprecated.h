// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Defines.h>

#if !CARB_AARCH64

#    include <omni/fabric/IFabric.h>
#    include <omni/fabric/SimStageWithHistory.h>

#    include <cudamanager/PxCudaTypes.h>
#    include <PxPhysicsAPI.h>

#    include <unordered_map>
#    include <vector_types.h>

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

struct ParticleClothDataDeprecated
{
    size_t numVerts = 0;
    ::physx::PxVec4* points = nullptr;
    pxr::UsdPrim prim;
    pxr::GfMatrix4d initialPrimToParent;
    CUdeviceptr remapTable = 0; // null - cloth is not welded, otherwise it's welded.
};

// Each scene has a list of particle cloths
class ParticleClothSetDeprecated
{
public:
    ParticleClothSetDeprecated();
    ~ParticleClothSetDeprecated();

    void releaseBuffers();
    void prepareBuffers(omni::fabric::StageReaderWriter& srw);
    void updateParticleCloths(omni::fabric::StageReaderWriter& srw) const;

    uint32_t mMaxPoints = 0;

    CUdeviceptr mParticleClothPointsDev = 0; // list of pointers to particle buffers - input of copy kernel
    CUdeviceptr mParticleClothMtxDevPtrs = 0; // device buffer for transforms
    CUdeviceptr mFabricPointsDev = 0; // list of pointers to fabric buffers - output of copy kernel
    CUdeviceptr mParticleClothSizesDev = 0; // device mirror of size for each cloth.
    CUdeviceptr mParticleClothRemapTablesDev = 0; // device pointer to pointers for remap tables - remap table maps sim
                                                  // points to mesh points.

    // pinned host memory
    CUdeviceptr* mParticleClothPointsDevPtrsHost = 0; // host mirror of mParticleClothPointsDev
    pxr::GfMatrix4f* mWorldToParticleClothHost = 0; // host mirror of mParticleClothMtxDevPtrs
    CUdeviceptr* mFabricHost = 0; // host mirror of mFabricPointsDev
    ::physx::PxU32* mParticleClothSizesHost = 0; // host mirror of mParticleClothSizesDev
    CUdeviceptr* mParticleClothRemapTablesDevPtrsHost = 0; // host mirror of mParticleClothRemapTablesDev
    float3** mFabricPointsCPU; // pointers to CPU fabric buffers

    std::vector<ParticleClothDataDeprecated*> mParticleCloths;

    omni::fabric::Token mParticleClothSchemaToken;
    omni::fabric::Token mPointsToken;

    omni::fabric::Type mTypeFloat3Array;

    ::physx::PxScene* mScene = nullptr;

    bool mGPUInterop = false;
};


class ParticleManagerDeprecated
{
public:
    ParticleManagerDeprecated();
    ~ParticleManagerDeprecated();

    void registerParticleCloth(pxr::UsdGeomXformCache& xfCache,
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
    void updateParticleCloths(omni::fabric::StageReaderWriter& srw);

    omni::fabric::Type mTypeAppliedSchema;
    omni::fabric::Type mTypeFloat3;
    omni::fabric::Type mTypeFloat3Array;

    omni::fabric::Token mPointsToken;
    omni::fabric::Token mParticleClothSchemaToken;

    std::unordered_map<omni::fabric::PathC, ParticleClothDataDeprecated> mParticleCloths;
    std::unordered_map<::physx::PxScene*, ParticleClothSetDeprecated> mParticleClothsSet;

    omni::physx::IPhysxSimulation* mPhysxSimulationInterface;

    PositionCache mInitialPositions;

    bool mIsDirty = false;
};
} // namespace physx
} // namespace omni

#endif

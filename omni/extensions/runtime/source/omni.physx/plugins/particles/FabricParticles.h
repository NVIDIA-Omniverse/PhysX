// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/settings/ISettings.h>

#include <omni/fabric/SimStageWithHistory.h>

#include <omni/fabric/IFabric.h>
#include <omni/fabric/FabricUSD.h>

#include <omni/physx/IPhysx.h>

#include <PxPhysicsAPI.h>

#define ENABLE_BATCH_PROCESSING 0 // Work in progress. Not needed at the moment.

namespace omni
{
namespace kit
{
struct StageUpdateSettings;
}
namespace physx
{
namespace internal
{
class InternalParticleSet;
}

struct IPhysxSimulation;

struct modifiedData
{
    void* fabricPtr;
    void* cudaPtr;
    size_t size;
};

class FabricParticles
{
public:
    FabricParticles();
    ~FabricParticles();

    void attachStage(long stageId);
    void detachStage();
    void update();
    void reset();

    void initializeParticleSet(internal::InternalParticleSet* particleSet);
    void updateParticles(const omni::fabric::PrimBucketList& primBucketList,
                         size_t primBucketListIndex,
                         const omni::fabric::Token& token);
    void updateParticles();
    void updateFabric();
    void saveToUsd();

private:
    std::map<pxr::SdfPath, internal::InternalParticleSet*> mParticleSets;

    // Fabric
    omni::fabric::UsdStageId mStageId;


    omni::fabric::Token mParticleSetApiToken;
    omni::fabric::Token mPositionInvMassesToken;
    omni::fabric::Token mVelocitiesFloat4Token;

    omni::fabric::Type mAppliedSchemaType;
    omni::fabric::Type mFloat4ArrayType;

    omni::fabric::IStageReaderWriter* mIStageReaderWriter;
    omni::fabric::IFabric* mFabric;

    // Gpu interop
    ::physx::PxCudaContextManager* mCudaContextManager = 0;

    std::vector<modifiedData> mPositionsChanged;
    std::vector<modifiedData> mVelocitiesChanged;

#if ENABLE_BATCH_PROCESSING
    CUdeviceptr mInPtrDev = 0;
    CUdeviceptr mOutPtrDev = 0;
    CUdeviceptr mSizePtrDev = 0;
    void** mInPtrHost = 0;
    void** mOutPtrHost = 0;
    int* mSizePtrHost = 0;
#endif
};

} // namespace physx
} // namespace omni

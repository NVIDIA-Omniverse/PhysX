// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Defines.h>

#include <omni/fabric/SimStageWithHistory.h>

#include <foundation/PxTransform.h>
#include <cudamanager/PxCudaTypes.h>

#include <unordered_map>

#include "FabricMapper.h"

namespace physx
{
class PxScene;
}

namespace omni
{
namespace physx
{
struct IPhysxSimulation;

class DirectGpuHelper
{
public:
    DirectGpuHelper();
    ~DirectGpuHelper();

    void attach(unsigned long usdStageId);
    void detach();

    void registerRigidBody(const omni::fabric::Path& primPath, const carb::Float3& scale);

    void update(omni::fabric::StageReaderWriter& srw, bool forceUpdate);

    void clear();

private:
    // indexing data for rigid bodies, which could be articulation links or rigid dynamics
    struct RigidBodyData
    {
        struct LinkData
        {
            uint32_t artiIdx = 0xffffffff; // position in index buffer
            uint32_t linkIdx = 0xffffffff; // link index in articulation
            uint32_t linkOffset = 0xffffffff; // offset in PhysX GPU link buffer
        };

        struct RigidDynamicData
        {
            uint32_t idx = 0xffffffff; // offset in PhysX GPU actor data buffer
        };

        RigidBodyData() : isArticulationLink(false), rd()
        {
        }

        bool isArticulationLink;
        union
        {
            LinkData link;
            RigidDynamicData rd;
        };
        std::unordered_map<omni::fabric::Path, FabricMapper::AttributePtrType>::pointer parentLink;
    };

    bool prepareBuffers(omni::fabric::StageReaderWriter& srw, bool fullFabricGpuInterop);
    bool prepareMappings(omni::fabric::StageReaderWriter& srw);
    void releaseBuffers();

    bool registerScene(::physx::PxScene* scene);

    void updateRigidBodies(omni::fabric::StageReaderWriter& srw, bool updateTransforms, bool updateVelocities, bool fullFabricGpuInterop);
    void fetchRigidBodyData(omni::fabric::StageReaderWriter& srw, bool updateTransforms, bool updateVelocities, bool fullFabricGpuInterop);

    omni::fabric::Token mRigidBodySchemaToken;

    omni::fabric::Token mWorldMatrixToken;
    omni::fabric::Token mLocalMatrixToken;

    omni::fabric::Token mLinVelToken;
    omni::fabric::Token mAngVelToken;

    omni::fabric::Token mRigidBodyWorldPositionToken;
    omni::fabric::Token mRigidBodyWorldOrientationToken;
    omni::fabric::Token mRigidBodyWorldScaleToken;

    omni::fabric::Type mTypeAppliedSchema;
    omni::fabric::Type mTypeFloat3;
    omni::fabric::Type mTypeDouble3;
    omni::fabric::Type mTypeMatrix4d;
    omni::fabric::Type mTypeQuat;

    ::physx::PxScene* mScene = nullptr;

    omni::physx::IPhysxSimulation* mPhysxSimulationInterface;

    std::unordered_map<omni::fabric::Path, RigidBodyData> mRigidBodies;
    // mInitialScales only stores the dynamic rbs
    std::unordered_map<omni::fabric::Path, carb::Float3> mInitialScales;
    std::unordered_map<omni::fabric::Path, FabricMapper::AttributePtrType> mParentsMap;

    uint32_t mNumRds = 0;
    uint32_t mNumArtis = 0;
    uint32_t mMaxLinks = 0;
    uint32_t mLinkBufSize = 0;

    bool mNeedResync = false;

    std::vector<::physx::PxTransform> mLinkTransforms;
    std::vector<::physx::PxVec3> mLinkLinearVelocities;
    std::vector<::physx::PxVec3> mLinkAngularVelocities;

    std::vector<::physx::PxTransform> mRdTransforms;
    std::vector<::physx::PxVec3> mRdLinearVelocities;
    std::vector<::physx::PxVec3> mRdAngularVelocities;

    CUdeviceptr mArtiIndicesDev = 0;
    CUdeviceptr mLinkTransformsDev = 0;
    CUdeviceptr mLinkLinearVelocitiesDev = 0;
    CUdeviceptr mLinkAngularVelocitiesDev = 0;

    CUdeviceptr mRdIndicesDev = 0;
    CUdeviceptr mRdTransformsDev = 0;
    CUdeviceptr mRdLinearVelocitiesDev = 0;
    CUdeviceptr mRdAngularVelocitiesDev = 0;

    CUdeviceptr mInitialScalesDev = 0;

    CUevent mLinkTransformsStartEvent = nullptr;
    CUevent mLinkTransformsCopyEvent = nullptr;

    CUevent mLinkLinearVelocitiesCopyEvent = nullptr;
    CUevent mLinkLinearVelocitiesStartEvent = nullptr;

    CUevent mLinkAngularVelocitiesCopyEvent = nullptr;
    CUevent mLinkAngularVelocitiesStartEvent = nullptr;

    CUevent mRdTransformsCopyEvent = nullptr;
    CUevent mRdTransformsStartEvent = nullptr;

    CUevent mRdLinearVelocityCopyEvent = nullptr;
    CUevent mRdLinearVelocityStartEvent = nullptr;

    CUevent mRdAngularVelocityCopyEvent = nullptr;
    CUevent mRdAngularVelocityStartEvent = nullptr;

    uint64_t mAttachTimestamp = 0;
    uint64_t mRbFetchTimestamp = 0;

    FabricMapper mFabricMapperRb;
    FabricMapper mFabricMapperArticulation;
    uint64_t mFabricTopologyVersion;
    uint64_t mFabricConnectivityVersion;
    bool isTopologyConnectivityInitialized = false;
    fabric::UsdStageId mUsdStageId;

    CUstream mCopyStream = 0;
};

} // namespace physx
} // namespace omni

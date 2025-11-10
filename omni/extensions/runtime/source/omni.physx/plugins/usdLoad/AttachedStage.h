// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <omni/fabric/IFabric.h>
#include <private/omni/physx/IPhysxUsdLoad.h>
#include <usdInterface/UsdInterface.h>
#include "PrimUpdate.h"

namespace omni
{
namespace physx
{
namespace usdparser
{
const uint64_t kInvalidFabricListenerId = 0;

using TimeSampleMap = std::unordered_map<pxr::SdfPath, OnUpdateObjectFn, pxr::SdfPath::Hash>;
using PathPrimMap = std::unordered_map<pxr::SdfPath, pxr::UsdPrim, pxr::SdfPath::Hash>;

enum class ChangeSource
{
    eUsd,
    eFabric,
    eUnknwon,
};

class AttachedStage;
class ChangeSourceBlock
{
public:
    ChangeSourceBlock(AttachedStage& attachedStage, ChangeSource source);
    ~ChangeSourceBlock();

    ChangeSourceBlock(const ChangeSourceBlock&) = delete;
    ChangeSourceBlock& operator=(const ChangeSourceBlock&) = delete;
    ChangeSourceBlock(ChangeSourceBlock&&) = default;
    ChangeSourceBlock& operator=(ChangeSourceBlock&&) = default;

private:
    AttachedStage& mAttachedStage;
    ChangeSource mPrevSource;
};

class AttachedStage
{
public:
    AttachedStage();
    AttachedStage(pxr::UsdStageWeakPtr stage);
    AttachedStage(pxr::UsdStageWeakPtr stage, PhysXUsdPhysicsInterface* iface);
    ~AttachedStage();

    void fabricAttach();

    PhysXUsdPhysicsInterface* getPhysXPhysicsInterface()
    {
        return mPhysicsInterface;
    }

    const PhysXUsdPhysicsInterface* getPhysXPhysicsInterface() const
    {
        return mPhysicsInterface;
    }

    pxr::UsdStageWeakPtr getStage()
    {
        return mStage;
    }

    pxr::UsdStageWeakPtr getStage() const
    {
        return mStage;
    }

    long getStageId() const
    {
        return mStageId;
    }

    void setStage(pxr::UsdStageWeakPtr stage)
    {
        mStage = stage;
        mStageId = stage ? pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt() : 0u;
    }

    bool isPhysXDefaultSimulator() const
    {
        return mPhysXDefaultSim;
    }

    void setIsPhysXDefaultSimulator(bool val)
    {
        mPhysXDefaultSim = val;
    }

    void releasePhysicsObjects();

    ObjectDb* getObjectDatabase()
    {
        return mObjectDatabase;
    }

    const ObjectDb* getObjectDatabase() const
    {
        return mObjectDatabase;
    }

    PathPrimMap& getAnimatedKinematicBodies()
    {
        return mAnimatedKinematicBodies;
    }

    const PathPrimMap& getAnimatedKinematicBodies() const
    {
        return mAnimatedKinematicBodies;
    }

    void removeAnimatedKinematicBody(const pxr::SdfPath& path)
    {
        PathPrimMap::const_iterator fit = mAnimatedKinematicBodies.find(path);
        if (fit != mAnimatedKinematicBodies.end())
        {
            mAnimatedKinematicBodies.erase(fit);
        }
    }

    PathPrimMap& getAnimatedKinematicDeformableBodies()
    {
        return mAnimatedKinematicDeformableBodies;
    }
    const PathPrimMap& getAnimatedKinematicDeformableBodies() const
    {
        return mAnimatedKinematicDeformableBodies;
    }
    void removeAnimatedKinematicDeformableBody(const pxr::SdfPath& path)
    {
        PathPrimMap::const_iterator fit = mAnimatedKinematicDeformableBodies.find(path);
        if (fit != mAnimatedKinematicDeformableBodies.end())
        {
            mAnimatedKinematicDeformableBodies.erase(fit);
        }
    }

    PrimUpdateMap& getPrimUpdateMap()
    {
        return mPrimUpdateMap;
    }

    const PrimUpdateMap& getPrimUpdateMap() const
    {
        return mPrimUpdateMap;
    }

    PrimChangeMap& getPrimChangeMap()
    {
        return mPrimChangeMap;
    }

    const PrimChangeMap& getPrimChangeMap() const
    {
        return mPrimChangeMap;
    }

    TimeSampleMap& getTimeSampleMap()
    {
        return mTimeSampledAttributes;
    }

    const TimeSampleMap& getTimeSampleMap() const
    {
        return mTimeSampledAttributes;
    }

    void bufferRequestRigidBodyMassUpdate(const pxr::UsdPrim& prim)
    {
        mRigidBodyMassUpdateMap[prim.GetPrimPath()] = prim;
    }

    void registerTimeSampledAttribute(const pxr::SdfPath& attributePath, OnUpdateObjectFn onUpdate);

    void unregisterTimeSampledAttribute(const pxr::SdfPath& attributePath);

    void registerStageSpecificAttribute(ChangeParams& changeParam);

    void clearStageSpecificAttributes();

    void registerObjectId(const pxr::SdfPath& path, const ObjectCategory& category, const ObjectId& newEntryId);

    void updateRigidBodyMass();

    const ObjectIdMap* getObjectIds(const pxr::SdfPath& path) const;

    omni::fabric::ListenerId getFabricListenerId() const
    {
        return mListenerId;
    }

    void pauseChangeTracking(bool pause);

    bool isChangeTrackingPaused() const
    {
        return mChangeTrackingPaused;
    }

    CollisionGroupsMap& getCollisionGroupMap()
    {
        return mCollisionGroupsMap;
    }

    const CollisionGroupsMap& getCollisionGroupMap() const
    {
        return mCollisionGroupsMap;
    }

    std::vector<CollisionGroupsMap>& getAdditionalCollisionGroupMaps()
    {
        return mAdditionalCollisionGroupMaps;
    }

    const std::vector<CollisionGroupsMap>& getAdditionalCollisionGroupMaps() const
    {
        return mAdditionalCollisionGroupMaps;
    }

    AttachmentHistoryMapDeprecated& getAttachmentHistoryMapDeprecated()
    {
        return mAttachmentHistoryMapDeprecated;
    }

    DeformableAttachmentHistoryMap& getDeformableAttachmentHistoryMap()
    {
        return mDeformableAttachmentHistoryMap;
    }

    DeformableCollisionFilterHistoryMap& getDeformableCollisionFilterHistoryMap()
    {
        return mDeformableCollisionFilterHistoryMap;
    }

    bool isReplicatorStage() const
    {
        return mReplicatorStage;
    }

    void setReplicatorStage(bool val)
    {
        mReplicatorStage = val;
    }

    void addReplicatorMemoryBlock(void* memory)
    {
        mReplicatorMemory.push_back(memory);
    }

    void freeReplicatorMemory()
    {
        for (void* mem : mReplicatorMemory)
        {
            free(mem);
        }

        mReplicatorMemory.clear();
    }

    void setUseReplicatorEnvIds(bool val)
    {
        mUseReplicatorEnvIds = val;
    }

    bool isUsingReplicatorEnvIds() const
    {
        return mUseReplicatorEnvIds;
    }

    void setChangeSource(ChangeSource source)
    {
        mChangeSource = source;
    }

    ChangeSource getChangeSource() const
    {
        return mChangeSource;
    }

    ChangeSourceBlock getChangeSourceBlock(ChangeSource source)
    {
        return ChangeSourceBlock(*this, source);
    }

private:
    PhysXUsdPhysicsInterface* mPhysicsInterface;
    pxr::UsdStageWeakPtr mStage;
    long mStageId;
    ObjectDb* mObjectDatabase;
    PrimUpdateMap mPrimUpdateMap;
    PrimChangeMap mPrimChangeMap;
    TimeSampleMap mTimeSampledAttributes;
    PathPrimMap mAnimatedKinematicBodies;
    PathPrimMap mAnimatedKinematicDeformableBodies;
    omni::physics::schema::UsdPrimMap mRigidBodyMassUpdateMap;
    CollisionGroupsMap mCollisionGroupsMap;
    std::vector<CollisionGroupsMap> mAdditionalCollisionGroupMaps;
    AttachmentHistoryMapDeprecated mAttachmentHistoryMapDeprecated;
    DeformableAttachmentHistoryMap mDeformableAttachmentHistoryMap;
    DeformableCollisionFilterHistoryMap mDeformableCollisionFilterHistoryMap;
    omni::fabric::ListenerId mListenerId;
    bool mChangeTrackingPaused;
    bool mReplicatorStage;
    bool mPhysXDefaultSim;
    bool mUseReplicatorEnvIds;
    std::vector<void*> mReplicatorMemory;
    ChangeSource mChangeSource{ ChangeSource::eUnknwon };
};

} // namespace usdparser
} // namespace physx
} // namespace omni

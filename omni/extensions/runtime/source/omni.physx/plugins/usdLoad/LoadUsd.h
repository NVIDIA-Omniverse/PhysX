// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <omni/fabric/IToken.h>
#include <private/omni/physx/IPhysxUsdLoad.h>

#include "LoadTools.h"
#include "PrimUpdate.h"
#include "AttachedStage.h"
#include <PhysXReplicator.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
using AttachedStageMap = std::unordered_map<uint64_t, AttachedStage*>;

void releaseDesc(PhysxObjectDesc* objectDesc);
PhysxJointDesc* parseJoint(uint64_t stageId, const pxr::SdfPath& path);
PhysxRigidBodyDesc* parseRigidBody(uint64_t stageId,
                                   const pxr::SdfPath& path,
                                   std::vector<std::pair<pxr::SdfPath, PhysxShapeDesc*>>& collision);
std::vector<PhysxTendonAttachmentHierarchyDesc*> parseSpatialTendons(uint64_t stageId, const pxr::SdfPath& parentPath);
std::vector<PhysxArticulationDesc*> parseArticulations(uint64_t stageId, const pxr::SdfPath& articulationRootPath);
std::vector<PhysxTendonAxisHierarchyDesc*> parseFixedTendons(uint64_t stageId, const pxr::SdfPath& parentPath);
PhysxShapeDesc* parseCollision(uint64_t stageId, const pxr::SdfPath& path, const pxr::SdfPath& gPrimPath);
SoftBodyDesc* parseDeformableBodyDeprecated(uint64_t stageId, const pxr::SdfPath& path);
FEMClothDesc* parseDeformableSurfaceDeprecated(uint64_t stageId, const pxr::SdfPath& path);
ParticleClothDesc* parseParticleClothDeprecated(uint64_t stageId, const pxr::SdfPath& path);
ParticleSetDesc* parseParticleSet(uint64_t stageId, const pxr::SdfPath& path);
ParticleSamplingDesc* parseParticleSampling(uint64_t stageId, const pxr::SdfPath& path);
VehicleComponentTrackerHandle createVehicleComponentTracker();
void releaseVehicleComponentTracker(usdparser::VehicleComponentTrackerHandle);
VehicleDesc* parseVehicle(uint64_t stageId, const pxr::SdfPath&, usdparser::VehicleComponentTrackerHandle);
PhysxAttachmentDesc* parsePhysxAttachmentDeprecated(uint64_t stageId, const pxr::SdfPath& attachmentPath);
ParticleSystemDesc* parseParticleSystem(uint64_t stageId, const pxr::SdfPath& particleSystemPath);

struct FabricTokens
{
    omni::fabric::Token* worldMatrix;
    omni::fabric::Token* localMatrix;
    omni::fabric::Token* worldForce;
    omni::fabric::Token* worldTorque;
    omni::fabric::Token* positionInvMasses;
    omni::fabric::Token* velocitiesFloat4;
};

class UsdLoad
{
public:
    UsdLoad();

    ~UsdLoad();

    static UsdLoad* getUsdLoad();

    void requestRigidBodyMassUpdate(const pxr::UsdPrim& prim);
    void requestDeformableBodyMassUpdateDeprecated(const pxr::UsdPrim& prim);
    void requestDeformableSurfaceMassUpdateDeprecated(const pxr::UsdPrim& prim);
    void requestParticleMassUpdate(const pxr::UsdPrim& prim);

    bool attach(bool loadPhysics, uint64_t stageId, PhysXUsdPhysicsInterface* usdPhysicsInt);
    bool attachReplicator(uint64_t stageId,
                          PhysXUsdPhysicsInterface* usdPhysicsInt,
                          const PathSet& excludePaths,
                          bool attachStage);

    void detach(uint64_t stageId);

    AttachedStage* getAttachedStage(uint64_t stageId) const
    {
        if (stageId == 0 && mAttachedStages.size() == 1)
        {
            return mAttachedStages.begin()->second;
        }

        AttachedStageMap::const_iterator fit = mAttachedStages.find(stageId);
        if (fit != mAttachedStages.end())
        {
            return fit->second;
        }

        return nullptr;
    }

    void fabricAttach(uint64_t stageId);

    void update(uint64_t stageId, float);
    void update(float);
    void flushChanges();

    void releasePhysicsObjects(uint64_t stageId);

    void blockUSDUpdate(bool val);
    bool usdUpdateIsBlocked();

    void setAsyncUSDUpdate(bool val)
    {
        mAsyncUpdate = val;
    }
    bool getAsyncUSDUpdate() const
    {
        return mAsyncUpdate;
    }
    void processChanges();

    MemoryAllocator getMemoryAllocator()
    {
        return mMemoryAllocator;
    }

    const FabricTokens& getFabricTokens() const
    {
        return mFabricTokens;
    }

    void updateRigidBodyMass();

    void pauseChangeTracking(uint64_t stageId, bool pause)
    {
        AttachedStageMap::iterator fit = mAttachedStages.find(stageId);
        if (fit != mAttachedStages.end())
        {
            fit->second->pauseChangeTracking(pause);
        }
    }

    bool isChangeTrackingPaused(uint64_t stageId) const
    {
        AttachedStageMap::const_iterator fit = mAttachedStages.find(stageId);
        if (fit != mAttachedStages.end())
        {
            return fit->second->isChangeTrackingPaused();
        }
        return false;
    }

    void registerTimeSampledAttribute(const pxr::UsdAttribute& attribute, OnUpdateObjectFn onUpdate)
    {
        uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(attribute.GetStage()).ToLongInt();
        AttachedStageMap::iterator fit = mAttachedStages.find(stageId);
        if (fit != mAttachedStages.end())
        {
            fit->second->registerTimeSampledAttribute(attribute.GetPath(), onUpdate);
        }
    }

    void changeDefaultSimulator(const std::string& defaultSim);

private:
    UsdNoticeListener* mUsdNoticeListener;
    pxr::TfNotice::Key mUsdNoticeListenerKey;
    pxr::TfNotice::Key mAttributeValuesChangedListenerKey;
    std::atomic_int32_t mBlockUsdUpdate;
    MemoryAllocator mMemoryAllocator;
    volatile bool mAsyncUpdate;
    FabricTokens mFabricTokens;
    AttachedStageMap mAttachedStages;
};


} // namespace usdparser
} // namespace physx
} // namespace omni

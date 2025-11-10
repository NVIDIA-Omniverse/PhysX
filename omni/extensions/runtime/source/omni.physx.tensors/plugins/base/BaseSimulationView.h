// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include <omni/physx/IPhysx.h>
#include "../CommonTypes.h"
#include "BaseSimulationData.h"

#include <omni/physics/tensors/ISimulationView.h>
#include <omni/physics/tensors/ObjectTypes.h>

#include <omni/fabric/IToken.h>

#include <vector>
#include <carb/events/EventsUtils.h>
#include <omni/timeline/ITimeline.h>

namespace omni
{
namespace physx
{
namespace tensors
{
class BaseArticulationView;
class BaseRigidBodyView;
class BaseSdfShapeView;
class BaseSoftBodyView;
class BaseSoftBodyMaterialView;
class BaseVolumeDeformableBodyView;
class BaseSurfaceDeformableBodyView;
class BaseDeformableMaterialView;
class BaseRigidContactView;
class BaseParticleSystemView;
class BaseParticleClothView;
class BaseParticleMaterialView;

using omni::physics::tensors::ObjectType;

class BaseSimulationView : public omni::physics::tensors::ISimulationView
{
public:
    explicit BaseSimulationView(pxr::UsdStageRefPtr stage);

    virtual ~BaseSimulationView() override;

    //
    // public API
    //

    bool setSubspaceRoots(const char* pattern) override;
    void InitializeKinematicBodies() override;

    void step(float dt) override;

    //
    // utilities
    //

    ObjectType getObjectType(const char* path) override;

    void findMatchingPaths(const std::string& pattern, std::vector<pxr::SdfPath>& pathsRet);

    // process the user-defined path and form the entries
    void processArticulationEntries(const std::vector<std::string>& patterns, std::vector<ArticulationEntry>& entries);
    void findMatchingArticulations(const std::string& pattern, std::vector<ArticulationEntry>& entriesRet);

    // process the user-defined path and form the entries
    void processRigidBodyEntries(const std::vector<std::string>& patterns, std::vector<RigidBodyEntry>& entries);
    void findMatchingRigidBodies(const std::string& pattern, std::vector<RigidBodyEntry>& entriesRet);

    void findMatchingSoftBodies(const std::string& pattern, std::vector<SoftBodyEntry>& entriesRet);

    void findMatchingSoftBodyMaterials(const std::string& pattern, std::vector<SoftBodyMaterialEntry>& entriesRet);

    void findMatchingVolumeDeformableBodies(const std::string& pattern,
                                            std::vector<DeformableBodyEntry>& entriesRet);

    void findMatchingSurfaceDeformableBodies(const std::string& pattern,
                                             std::vector<DeformableBodyEntry>& entriesRet);

    void findMatchingDeformableMaterials(const std::string& pattern,
                                         std::vector<DeformableMaterialEntry>& entriesRet);

    // process the user-defined path and form the entries
    void processRigidContactViewEntries(const std::vector<std::string>& patterns,
                                        const std::vector<std::vector<std::string>>& filterPatterns,
                                        std::vector<RigidContactSensorEntry>& entries,
                                        uint32_t& filterPatternSize);

    void findMatchingRigidContactSensors(const std::string& pattern,
                                         const std::vector<std::string>& filterPatterns,
                                         std::vector<RigidContactSensorEntry>& entriesRet);

    void findMatchingParticleSystems(const std::string& pattern, std::vector<ParticleSystemEntry>& entriesRet);

    void findMatchingParticleCloths(const std::string& pattern, std::vector<ParticleClothEntry>& entriesRet);

    void findMatchingParticleMaterials(const std::string& pattern, std::vector<ParticleMaterialEntry>& entriesRet);

    void findMatchingSDFShapes(const std::string& pattern,
                               std::vector<SdfShapeEntry>& entriesRet,
                               uint32_t numSamplePoints);

    bool getArticulationAtPath(const pxr::SdfPath& path, ArticulationEntry& entryRet);

    bool getRigidBodyAtPath(const pxr::SdfPath& path, RigidBodyEntry& entryRet);

    bool getSoftBodyAtPath(const pxr::SdfPath& path, SoftBodyEntry& entryRet);

    bool getSoftBodyMaterialAtPath(const pxr::SdfPath& path, SoftBodyMaterialEntry& entryRet);

    bool getVolumeDeformableBodyAtPath(const pxr::SdfPath& path, DeformableBodyEntry& entryRet);

    bool getSurfaceDeformableBodyAtPath(const pxr::SdfPath& path, DeformableBodyEntry& entryRet);

    bool getDeformableMaterialAtPath(const pxr::SdfPath& path, DeformableMaterialEntry& entryRet);

    bool getRigidContactSensorAtPath(const pxr::SdfPath& path, RigidContactSensorEntry& entryRet);

    bool getSDFShapeAtPath(const pxr::SdfPath& path, SdfShapeEntry& entryRet);

    bool getParticleSystemAtPath(const pxr::SdfPath& path, ParticleSystemEntry& entryRet);

    bool getParticleClothAtPath(const pxr::SdfPath& path, ParticleClothEntry& entryRet);

    bool getParticleMaterialAtPath(const pxr::SdfPath& path, ParticleMaterialEntry& entryRet);

    const ArticulationMetatype* getUniqueArticulationMetatype(const ArticulationMetatype& metatype);

    Subspace* findSubspaceForPath(const pxr::SdfPath& path) const;

    BaseSimulationDataPtr getBaseSimulationData() const
    {
        return mSimData;
    }

    bool check() const override;

    void release(bool recursive) override;
    static void onPhysXObjectDeletedCallback(const pxr::SdfPath& sdfPath,
                                             usdparser::ObjectId objectId,
                                             PhysXType type,
                                             void* userData);
    static void onAllPhysXObjectDeletedCallback(void* userData);                                             

    // physics scene properties
    bool setGravity(const carb::Float3& gravity) override;
    bool getGravity(carb::Float3& gravity) override;

    void _onChildRelease(const BaseSdfShapeView* sdfView);
    void _onChildRelease(const BaseArticulationView* artiView);
    void _onChildRelease(const BaseRigidBodyView* rbView);
    void _onChildRelease(const BaseSoftBodyView* sbView);
    void _onChildRelease(const BaseSoftBodyMaterialView* sbView);
    void _onChildRelease(const BaseVolumeDeformableBodyView* deformableView);
    void _onChildRelease(const BaseSurfaceDeformableBodyView* deformableView);
    void _onChildRelease(const BaseDeformableMaterialView* deformableView);
    void _onChildRelease(const BaseRigidContactView* rcView);
    void _onChildRelease(const BaseParticleSystemView* psView);
    void _onChildRelease(const BaseParticleClothView* pcView);
    void _onChildRelease(const BaseParticleMaterialView* pmView);

    ::physx::PxMaterial* createSharedMaterial(float staticFriction, float dynamicFriction, float restitution);

    bool getValid() const override
    {
        return isValid;
    }
    void invalidate() override;
    bool hasRigidBody(::physx::PxRigidBody* body) const;
    bool hasArticulation(::physx::PxArticulationReducedCoordinate* arti) const;
    bool hasLink(::physx::PxArticulationLink* link) const;
    bool hasShape(::physx::PxShape* shape) const;
    bool hasfixedTendon(::physx::PxArticulationFixedTendon* ft) const;
    bool hasSpatialTendon(::physx::PxArticulationSpatialTendon* st) const;
    void initializeKinematics();
    std::unordered_map<std::string, ::physx::PxMaterial*> mMaterials;
    std::unordered_set<::physx::PxMaterial*> mUnusedMaterials;
    std::unordered_map<::physx::PxMaterial*, int> mMaterialsRefCount;

    // for fast lookup of all the physics elements of all the views
    std::unordered_set<::physx::PxRigidBody*> rigidBodies;
    std::unordered_set<::physx::PxArticulationReducedCoordinate*> articulations;
    std::unordered_set<::physx::PxArticulationLink*> links;
    std::unordered_set<::physx::PxShape*> shapes;
    std::unordered_set<::physx::PxArticulationFixedTendon*> fixedTendons;
    std::unordered_set<::physx::PxArticulationSpatialTendon*> spatialTendons;

    // fabric for kinematics
    omni::fabric::TokenC mRigidBodySchemaToken;
    omni::fabric::TokenC mWorldMatrixToken;
    omni::fabric::TokenC mLocalMatrixToken;
    omni::fabric::TokenC mDynamicBodyToken;
    omni::fabric::TokenC mRigidBodyWorldPositionToken;
    omni::fabric::TokenC mRigidBodyWorldOrientationToken;
    omni::fabric::TokenC mRigidBodyWorldScaleToken;

protected:
    bool isValid = true;
    pxr::UsdStageWeakPtr mStage;
    BaseSimulationDataPtr mSimData;

    std::vector<BaseSdfShapeView*> mSDFViews;
    std::vector<BaseArticulationView*> mArtiViews;
    std::vector<BaseRigidBodyView*> mRbViews;
    std::vector<BaseSoftBodyView*> mSbViews;
    std::vector<BaseSoftBodyMaterialView*> mSbMaterialViews;
    std::vector<BaseVolumeDeformableBodyView*> mVolumeDeformableBodyViews;
    std::vector<BaseSurfaceDeformableBodyView*> mSurfaceDeformableBodyViews;
    std::vector<BaseDeformableMaterialView*> mDeformableMaterialViews;
    std::vector<BaseRigidContactView*> mRcViews;
    std::vector<BaseParticleSystemView*> mParticleSysViews;
    std::vector<BaseParticleClothView*> mParticleClothViews;
    std::vector<BaseParticleMaterialView*> mParticleMatViews;

private:
    omni::physx::SubscriptionId subscriptionObjId;
    omni::timeline::TimelinePtr mTimeline;
    carb::events::ISubscriptionPtr mTimelineEvtSub;
    std::mutex mMutex;
    bool fabricKinematicsInitialized = false;

    void _hackFixMaterialConnectivity();
};
} // namespace tensors
} // namespace physx
} // namespace omni

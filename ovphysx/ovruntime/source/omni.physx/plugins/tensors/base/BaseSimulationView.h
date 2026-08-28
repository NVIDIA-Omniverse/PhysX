// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include <omni/physx/IPhysx.h>
#include "tensors/CommonTypes.h"
#include "tensors/base/BaseSimulationData.h"

#include <omni/physics/tensors/ISimulationView.h>
#include <omni/physics/tensors/ObjectTypes.h>

#include <vector>
#include <carb/events/EventsUtils.h>

namespace omni
{
namespace physx
{
namespace tensors
{
class BaseArticulationView;
class BaseRigidBodyView;
class BaseSdfShapeView;
class BaseVolumeDeformableBodyView;
class BaseSurfaceDeformableBodyView;
class BaseDeformableMaterialView;
class BaseRigidContactView;

using omni::physics::tensors::ObjectType;

class BaseSimulationView : public omni::physics::tensors::ISimulationView
{
public:
    explicit BaseSimulationView(PXR_NS::UsdStageRefPtr stage);

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

    void findMatchingPaths(const std::string& pattern, std::vector<PXR_NS::SdfPath>& pathsRet);

    void processArticulationEntries(const std::vector<std::string>& patterns, std::vector<ArticulationEntry>& entries);
    void findMatchingArticulations(const std::string& pattern,
                                   std::vector<ArticulationEntry>& entriesRet,
                                   std::unordered_set<const ::physx::PxArticulationReducedCoordinate*>& seenArtis);

    void processRigidBodyEntries(const std::vector<std::string>& patterns, std::vector<RigidBodyEntry>& entries);
    void findMatchingRigidBodies(const std::string& pattern,
                                 std::vector<RigidBodyEntry>& entriesRet,
                                 std::unordered_set<const ::physx::PxRigidBody*>& seenBodies);

    void setNoMatchLoggingQuiet(bool quiet) override;
    bool isNoMatchLoggingQuiet() const override;

    void processVolumeDeformableBodyEntries(const std::vector<std::string>& patterns, std::vector<DeformableBodyEntry>& entries);
    void findMatchingVolumeDeformableBodies(const std::string& pattern,
                                            std::vector<DeformableBodyEntry>& entriesRet,
                                            std::unordered_set<const ::physx::PxDeformableBody*>& seenBodies);

    void processSurfaceDeformableBodyEntries(const std::vector<std::string>& patterns, std::vector<DeformableBodyEntry>& entries);
    void findMatchingSurfaceDeformableBodies(const std::string& pattern,
                                             std::vector<DeformableBodyEntry>& entriesRet,
                                             std::unordered_set<const ::physx::PxDeformableBody*>& seenBodies);

    void processDeformableMaterialEntries(const std::vector<std::string>& patterns, std::vector<DeformableMaterialEntry>& entries);
    void findMatchingDeformableMaterials(const std::string& pattern,
                                         std::vector<DeformableMaterialEntry>& entriesRet,
                                         std::unordered_set<const ::physx::PxDeformableMaterial*>& seenMaterials);

    void processRigidContactViewEntries(const std::vector<std::string>& patterns,
                                        const std::vector<std::vector<std::string>>& filterPatterns,
                                        std::vector<RigidContactSensorEntry>& entries,
                                        uint32_t& filterPatternSize);

    void findMatchingRigidContactSensors(const std::string& pattern,
                                         const std::vector<std::string>& filterPatterns,
                                         std::vector<RigidContactSensorEntry>& entriesRet,
                                         std::unordered_set<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash>& seenSensorPaths);

    void findMatchingSDFShapes(const std::string& pattern,
                               std::vector<SdfShapeEntry>& entriesRet,
                               uint32_t numSamplePoints);

    bool getArticulationAtPath(const PXR_NS::SdfPath& path, ArticulationEntry& entryRet);

    bool getRigidBodyAtPath(const PXR_NS::SdfPath& path, RigidBodyEntry& entryRet);

    bool getVolumeDeformableBodyAtPath(const PXR_NS::SdfPath& path, DeformableBodyEntry& entryRet);

    bool getSurfaceDeformableBodyAtPath(const PXR_NS::SdfPath& path, DeformableBodyEntry& entryRet);

    bool getDeformableMaterialAtPath(const PXR_NS::SdfPath& path, DeformableMaterialEntry& entryRet);

    bool getRigidContactSensorAtPath(const PXR_NS::SdfPath& path, RigidContactSensorEntry& entryRet);

    bool getSDFShapeAtPath(const PXR_NS::SdfPath& path, SdfShapeEntry& entryRet);

    const ArticulationMetatype* getUniqueArticulationMetatype(const ArticulationMetatype& metatype);

    Subspace* findSubspaceForPath(const PXR_NS::SdfPath& path) const;

    BaseSimulationDataPtr getBaseSimulationData() const
    {
        return mSimData;
    }

    bool check() const override;

    void release(bool recursive) override;
    static void onPhysXObjectDeletedCallback(const PXR_NS::SdfPath& sdfPath,
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
    void _onChildRelease(const BaseVolumeDeformableBodyView* deformableView);
    void _onChildRelease(const BaseSurfaceDeformableBodyView* deformableView);
    void _onChildRelease(const BaseDeformableMaterialView* deformableView);
    void _onChildRelease(const BaseRigidContactView* rcView);

    ::physx::PxMaterial* createSharedMaterial(float staticFriction,
                                              float dynamicFriction,
                                              float restitution,
                                              float damping,
                                              ::physx::PxCombineMode::Enum frictionCombineMode,
                                              ::physx::PxCombineMode::Enum restitutionCombineMode,
                                              ::physx::PxCombineMode::Enum dampingCombineMode);

    // Build the shared-material pool key for a set of properties. The float
    // components use a representation that round-trips a float32 exactly, so two
    // distinct requested tuples never collide onto the same key.
    static std::string makeMaterialKey(float staticFriction,
                                       float dynamicFriction,
                                       float restitution,
                                       float damping,
                                       ::physx::PxCombineMode::Enum frictionCombineMode,
                                       ::physx::PxCombineMode::Enum restitutionCombineMode,
                                       ::physx::PxCombineMode::Enum dampingCombineMode);

    // Drop one reference to a pooled material. When the last reference is
    // released, every pool entry that references it is removed (looked up by
    // pointer, not by a reconstructed key) and the material is recycled into
    // mUnusedMaterials. Materials not owned by the pool are ignored.
    void releaseSharedMaterial(::physx::PxMaterial* material);

    bool getValid() const override
    {
        return isValid;
    }
    void invalidate() override;
    bool hasRigidBody(::physx::PxRigidBody* body) const;
    bool hasArticulation(::physx::PxArticulationReducedCoordinate* arti) const;
    bool hasLink(::physx::PxArticulationLink* link) const;
    bool hasShape(::physx::PxShape* shape) const;
    bool hasDeformableBody(::physx::PxDeformableBody* body) const;
    bool hasfixedTendon(::physx::PxArticulationFixedTendon* ft) const;
    bool hasSpatialTendon(::physx::PxArticulationSpatialTendon* st) const;
    std::unordered_map<std::string, ::physx::PxMaterial*> mMaterials;
    std::unordered_set<::physx::PxMaterial*> mUnusedMaterials;
    std::unordered_map<::physx::PxMaterial*, int> mMaterialsRefCount;
    // Reverse index: the exact pool key each live material is stored under, so
    // releaseSharedMaterial can drop its mMaterials entry in O(1) with the same
    // string used at insertion (no per-release map scan, no key reconstruction).
    std::unordered_map<::physx::PxMaterial*, std::string> mMaterialKeys;

    // for fast lookup of all the physics elements of all the views
    std::unordered_set<::physx::PxRigidBody*> rigidBodies;
    std::unordered_set<::physx::PxArticulationReducedCoordinate*> articulations;
    std::unordered_set<::physx::PxArticulationLink*> links;
    std::unordered_set<::physx::PxShape*> shapes;
    std::unordered_set<::physx::PxDeformableBody*> deformableBodies;
    std::unordered_set<::physx::PxArticulationFixedTendon*> fixedTendons;
    std::unordered_set<::physx::PxArticulationSpatialTendon*> spatialTendons;

protected:
    bool isValid = true;
    PXR_NS::UsdStageWeakPtr mStage;
    BaseSimulationDataPtr mSimData;
    bool mNoMatchLoggingQuiet = false;

    std::vector<BaseSdfShapeView*> mSDFViews;
    std::vector<BaseArticulationView*> mArtiViews;
    std::vector<BaseRigidBodyView*> mRbViews;
    std::vector<BaseVolumeDeformableBodyView*> mVolumeDeformableBodyViews;
    std::vector<BaseSurfaceDeformableBodyView*> mSurfaceDeformableBodyViews;
    std::vector<BaseDeformableMaterialView*> mDeformableMaterialViews;
    std::vector<BaseRigidContactView*> mRcViews;

private:
    omni::physx::SubscriptionId subscriptionObjId;
    std::mutex mMutex;
};
} // namespace tensors
} // namespace physx
} // namespace omni

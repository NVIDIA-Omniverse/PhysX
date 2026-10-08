// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @implements REQ-TENSOR-VIEW-001
 * @covers AC-1
 *
 * @implements REQ-TENSOR-ATTACH-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-36 AC-37 AC-38 AC-40
 */

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
namespace usdparser
{
class AttachedStage;
}

namespace tensors
{
class BaseArticulationView;
class BaseRigidBodyView;
class BaseSdfShapeView;
class BaseVolumeDeformableBodyView;
class BaseSurfaceDeformableBodyView;
class BaseDeformableMaterialView;
class BaseRigidContactView;
class BasePointInstancerView;

using omni::physics::tensors::ObjectType;

class BaseSimulationView : public omni::physics::tensors::ISimulationView
{
public:
    // `notifyWhenSimStopped` opts this view's destruction subscription out of the simulation-stopped
    // gate in shouldDeliverObjectChangeNotification(). Default false, the delivery a user-created
    // view gets: its owner rebuilds it around stage edits.
    //
    // A BACKEND-CACHED view must pass true. It outlives the call that built it and is handed to
    // later reads without the consumer knowing it exists, so a destruction it never hears about
    // leaves it holding freed actors -- and actors ARE destroyed while stopped, stage edits being
    // when that happens. Only destructions that raise a notification are covered; stage teardown is
    // handled separately by resetStage() dropping the attach's cache entries.
    explicit BaseSimulationView(usdparser::AttachedStage* attachedStage,
                                ::physx::PxScene* scene,
                                bool notifyWhenSimStopped = false);

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

    void findMatchingPaths(const std::string& pattern, std::vector<omni::physics::parse::ObjectKey>& keysRet);

    // Batched form of findMatchingPaths: resolves every entry of `patterns` in one
    // call, `keysRet[i]` holding `patterns[i]`'s matches. Same two-pass structure
    // (source-routed, then internal-DB-routed) as findMatchingPaths, but the
    // source-routed pass batches its literal-path existence checks across the
    // whole list (PathPatternMatcher::findMatchingObjectKeysBatch) instead of
    // paying one round trip per pattern -- for a caller holding a large per-index
    // pattern list (e.g. contact-sensor filter paths) this is the difference
    // between one round trip and thousands.
    void findMatchingPathsBatch(const std::vector<std::string>& patterns,
                                std::vector<std::vector<omni::physics::parse::ObjectKey>>& keysRet);

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
                                         const std::vector<size_t>& filterPatternIndices,
                                         const std::vector<std::vector<omni::physics::parse::ObjectKey>>& resolvedFilterKeys,
                                         const std::vector<std::vector<std::string>>& resolvedFilterPaths,
                                         const std::vector<std::vector<uint64_t>>& resolvedFilterLegacyIds,
                                         const std::vector<std::vector<uint8_t>>& resolvedFilterLegacyIdValid,
                                         std::vector<RigidContactSensorEntry>& entriesRet,
                                         std::unordered_set<omni::physics::parse::ObjectKey,
                                                            omni::physics::parse::ObjectKey::Hash>& seenSensorKeys);

    void findMatchingSDFShapes(const std::string& pattern,
                               std::vector<SdfShapeEntry>& entriesRet,
                               uint32_t numSamplePoints);

    bool getArticulationAtPath(omni::physics::parse::ObjectKey key, ArticulationEntry& entryRet);

    // Build an ArticulationEntry directly from a PxArticulation (no path/pattern), so the ovstage
    // read path can construct an articulation view with no backing USD stage. Shared with
    // getArticulationAtPath (which resolves the key to the arti then delegates here).
    // `fallbackKey` seeds entryRet.path only when the g_physx canonical path is empty.
    bool buildArticulationEntry(::physx::PxArticulationReducedCoordinate* arti,
                                omni::physics::parse::ObjectKey fallbackKey,
                                ArticulationEntry& entryRet);

    bool getRigidBodyAtPath(omni::physics::parse::ObjectKey key, RigidBodyEntry& entryRet);

    bool getVolumeDeformableBodyAtPath(omni::physics::parse::ObjectKey key, DeformableBodyEntry& entryRet);

    bool getSurfaceDeformableBodyAtPath(omni::physics::parse::ObjectKey key, DeformableBodyEntry& entryRet);

    bool getDeformableMaterialAtPath(omni::physics::parse::ObjectKey key, DeformableMaterialEntry& entryRet);

    bool getRigidContactSensorAtPath(omni::physics::parse::ObjectKey key, RigidContactSensorEntry& entryRet);

    bool getSDFShapeAtPath(omni::physics::parse::ObjectKey key, SdfShapeEntry& entryRet);

    const ArticulationMetatype* getUniqueArticulationMetatype(const ArticulationMetatype& metatype);

    Subspace* findSubspaceForPath(const std::string& path) const;

    BaseSimulationDataPtr getBaseSimulationData() const
    {
        return mSimData;
    }

    // Borrowed pointer, null once the attach goes away (see mAttachedStage). Lets sibling
    // view classes that only hold a BaseSimulationView* (not derive from it) convert a
    // path to an ObjectKey with the same keyFor bijection this class itself uses, rather
    // than the existence-checked public resolveObjectKey boundary function.
    usdparser::AttachedStage* getAttachedStage() const
    {
        return mAttachedStage;
    }

    // Resolves `key` to its live PhysX object pointer / internal ObjectId, scoped to
    // `attachedStage` -- never the process-globally "active" attach IPhysx::getPhysXPtr /
    // getObjectId use. A tensor view is bound to one attach for its lifetime (see
    // mAttachedStage below); resolving a view's own key through whichever attach happens
    // to be globally active instead can silently alias onto an unrelated object under a
    // second concurrent attach, because an ObjectKey is a small per-attach index, not the
    // globally-unique SdfPath string it replaced (ADR-0019). Static so callers that only
    // hold a BaseSimulationView* (siblings) or a bare AttachedStage& (SimulationBackend)
    // can use it without an instance.
    static void* resolvePhysXPtr(const usdparser::AttachedStage* attachedStage,
                                 omni::physics::parse::ObjectKey key, PhysXType type);
    static usdparser::ObjectId resolveObjectId(const usdparser::AttachedStage* attachedStage,
                                               omni::physics::parse::ObjectKey key, PhysXType type);

    bool check() const override;

    void release(bool recursive) override;
    static void onPhysXObjectDeletedCallback(omni::physics::parse::ObjectKey key,
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
    void _onChildRelease(const BasePointInstancerView* instancerView);
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
    bool hasScene(const ::physx::PxScene* scene) const;
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
    // The view's handle on the attach (ADR-0013). This is what says the view can
    // resolve objects: it is answered by the internal DB and the parse source, so it
    // works under a stageless attach. It is a borrowed pointer owned by UsdLoad and
    // dies at detach -- invalidate() clears it, and SimulationBackend::reset() /
    // resetStage() invalidate every view before the attach goes away.
    usdparser::AttachedStage* mAttachedStage = nullptr;
    ::physx::PxScene* mScene = nullptr;
    BaseSimulationDataPtr mSimData;
    bool mNoMatchLoggingQuiet = false;

    std::vector<BaseSdfShapeView*> mSDFViews;
    std::vector<BaseArticulationView*> mArtiViews;
    std::vector<BaseRigidBodyView*> mRbViews;
    std::vector<BaseVolumeDeformableBodyView*> mVolumeDeformableBodyViews;
    std::vector<BaseSurfaceDeformableBodyView*> mSurfaceDeformableBodyViews;
    std::vector<BaseDeformableMaterialView*> mDeformableMaterialViews;
    std::vector<BaseRigidContactView*> mRcViews;
    std::vector<BasePointInstancerView*> mPointInstancerViews;

private:
    omni::physx::SubscriptionId subscriptionObjId;
    std::mutex mMutex;
};
} // namespace tensors
} // namespace physx
} // namespace omni

// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-7 AC-9 AC-10
 *
 * @implements REQ-PARSE-MASS-003
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-7 AC-9 AC-10
 */

#pragma once

#include <PxPhysicsAPI.h>

#include <string>

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/PhysxUsd.h>
#include <omni/physics/parse/Handles.h>

#include <usdLoad/LoadTools.h>
#include <usdLoad/ChangeParams.h>

namespace omni
{
namespace physx
{
class PhysXScene;
namespace internal
{
class InternalPbdParticleSystem;
class InternalDeformableAttachment;
class InternalDeformableCollisionFilter;
class InternalPhysXDatabase;
} // namespace internal


class PhysXUsdPhysicsInterface
{
public:
    struct Transform
    {
        carb::Float3 position;
        carb::Float4 orientation;
        carb::Float3 scale;
    };

    struct MassInformation
    {
        float volume;
        float inertia[9]; // for density 1 so that we can scale it later.
        carb::Float3 centerOfMass;
        carb::Float3 localPos;
        carb::Float4 localRot;
        carb::Float3 aabbLocalMin;
        carb::Float3 aabbLocalMax;
        // Identity means no fitted geometry offset. createShape replaces it with poseOffset for fitted
        // approximations.
        carb::Float3 geometryToSourcePos = { 0.0f, 0.0f, 0.0f };
        carb::Float4 geometryToSourceRot = { 0.0f, 0.0f, 0.0f, 1.0f };
    };

    PhysXUsdPhysicsInterface();

    ~PhysXUsdPhysicsInterface();

    bool isDirty() const
    {
        return mDirty;
    }

    void setDirty(bool val)
    {
        mDirty = val;
    }

    void setExposePrimNames(bool val)
    {
        mExposePrimNames = val;
    }

    // scristiano: this is temporary code to allow physics inspector. we should filter simulation owners at parsing
    // stage. Kit-inspector-only debug filter, opt-in via an explicit Kit setting. Plain string, not
    // SdfPath: pxr-free storage so this setter is callable without pxr in hand; the read sites in
    // UsdInterface.cpp (unconditional, any backend) compare it against the real scene path's string form.
    void setForceParseOnlySingleScene(std::string forceParseOnlySingleScenePath)
    {
        mForceParseOnlySingleScenePath = std::move(forceParseOnlySingleScenePath);
    }

    // eInfiniteVoxelMap is unsupported in the USD-free runtime: the branch warns and returns
    // kInvalidObjectId, so no ePTInfiniteVoxelMap record is ever published.
    usdparser::ObjectId createObject(usdparser::AttachedStage& attachedStage,
                                     omni::physics::parse::ObjectKey key,
                                     const usdparser::PhysxObjectDesc& objectDesc,
                                     const usdparser::ObjectInstance* instance = nullptr);

    // ObjectKey-native entry point: resolves the active attach directly (no
    // UsdStageWeakPtr/stage-cache round trip needed). Both real callers
    // (PointInstancer.cpp, Collision.cpp) already hold the ObjectKey directly; no
    // SdfPath-taking overload is needed.
    usdparser::ObjectId createShape(omni::physics::parse::ObjectKey key,
                                    const usdparser::PhysxObjectDesc& objectDesc,
                                    usdparser::ObjectId bodyId,
                                    const usdparser::ObjectInstance* instance = nullptr);

    // Implements usdLoad/Mass.h's AbstractComputeRigidBodyMass::getShapeMassInfo -- the shape's
    // identity was never actually needed here (it switches purely on objectId's InternalDatabase
    // record type), so the interface dropped the unused SdfPath param and this works
    // unconditionally now. Also implemented by PhysXPropertyQuery.cpp's own override.
    MassInformation getShapeMassInfo(usdparser::ObjectId objectId) const;

    usdparser::ObjectId createJoint(usdparser::AttachedStage& attachedStage,
                                    omni::physics::parse::ObjectKey jointKey,
                                    const usdparser::PhysxJointDesc& desc,
                                    usdparser::ObjectId body0,
                                    usdparser::ObjectId body1);

    void recreateArticulationJoint(usdparser::AttachedStage&,
                                   const usdparser::PhysxJointDesc&,
                                   usdparser::ObjectId link0,
                                   usdparser::ObjectId link1);

    void releaseObject(usdparser::AttachedStage& attachedStage,
                       omni::physics::parse::ObjectKey removedKey,
                       usdparser::ObjectId objectId);

    void fillChangeParams(std::vector<usdparser::ChangeParams>& changeParams);

    bool updateTransform(const usdparser::AttachedStage& attachedStage,
                         omni::physics::parse::ObjectKey key,
                         usdparser::ObjectId objectId,
                         const Transform& transform,
                         bool resetVelocity = true,
                         bool scaleProvided = true);

    // ObjectKey-native entry point (the `path` this replaces was already unused in the body).
    bool updateMass(omni::physics::parse::ObjectKey key,
                    usdparser::ObjectId objectId,
                    float mass,
                    const carb::Float3& diagInertia,
                    const carb::Float3& com,
                    const carb::Float4& principalAxes);

    bool updateDeformableBodyMass(const usdparser::AttachedStage& attachedStage, usdparser::ObjectId objectId);
    bool updateDeformableBodyPositions(usdparser::AttachedStage& attachedStage, const usdparser::ObjectId objectId);
    bool updateDeformableBodyVelocities(usdparser::AttachedStage& attachedStage, const usdparser::ObjectId objectId);
    bool updateDeformableRestOffset(const usdparser::AttachedStage& attachedStage,
                                    usdparser::ObjectId objectId,
                                    float value);
    bool updateDeformableContactOffset(const usdparser::AttachedStage& attachedStage,
                                       usdparser::ObjectId objectId,
                                       float value);
    bool updateDeformableSelfCollisionFilterDistance(const usdparser::AttachedStage& attachedStage,
                                                     usdparser::ObjectId objectId,
                                                     float value);
    // ObjectKey-native entry point (the `path` this replaces was already unused in the body).
    bool updateParticleMass(omni::physics::parse::ObjectKey key,
                            usdparser::ObjectId objectId,
                            const usdparser::ParticleDesc& particleDesc);

    // ObjectKey-native entry point (the `path` this replaces was already unused in the body:
    // updateFn itself carries the TokenId/ReadTime it needs).
    bool updateObject(usdparser::AttachedStage& attachedStage,
                      omni::physics::parse::ObjectKey key,
                      usdparser::ObjectId objectId,
                      usdparser::OnUpdateObjectFn updateFn,
                      omni::physics::parse::TokenId propertyName,
                      omni::physics::parse::ReadTime timeCode);

    // ObjectKey-native entry point (the `path` this replaces was already unused in the body).
    void setupCollisionGroup(omni::physics::parse::ObjectKey key, const usdparser::CollisionGroupDesc& desc);

    bool setVehicleContext(const usdparser::AttachedStage& attachedStage, const usdparser::VehicleContextDesc&);

    void releaseAllObjects();

    // get shapes for mass computation, return true if triggers are present. ObjectIdPathMap is
    // ObjectKey-valued (LoadTools.h), so this works unconditionally.
    bool getRigidBodyShapes(const usdparser::AttachedStage& attachedStage,
                            usdparser::ObjectId rbId,
                            usdparser::ObjectIdPathMap& shapes) const;

    // ObjectKey-native entry point; no UsdPrim/UsdStageWeakPtr needed
    // internally (see the .cpp definition for why).
    static usdparser::ObjectId createShapeOrComputeMass(omni::physics::parse::ObjectKey key,
                                                        const usdparser::PhysxShapeDesc& shapeDesc,
                                                        usdparser::ObjectId bodyId,
                                                        const usdparser::ObjectInstance* instance,
                                                        usdparser::AttachedStage* attachedStage,
                                                        PhysXScene* physxScene,
                                                        bool exposePrimNames,
                                                        PhysXType& physxType,
                                                        internal::InternalPhysXDatabase* db,
                                                        PhysXUsdPhysicsInterface::MassInformation* massInfoOut);

    void finishSetup(const usdparser::AttachedStage& attachedStage);
    void finalizeArticulations(const usdparser::AttachedStage& attachedStage);

    void processDeformableAttachmentShapeEvents();
    void processDeformableCollisionFilterShapeEvents();

    static bool createOBB(const void* inputPoints,
                          const size_t nbPoints,
                          carb::Float3& halfExtent,
                          carb::Float3& offsetPos,
                          carb::Float4& offsetRot);

    static bool createBoundingSphere(const void* inputPoints,
                                     const size_t nbPoints,
                                     carb::Float3& sphereCenter,
                                     float& radius);

    static void reportLoadError(usdparser::ErrorCode::Enum errorCode, const char* msg);

    bool isReady(void);

    omni::physics::parse::ObjectKey getParentJointPathInArticulation(const usdparser::AttachedStage& attachedStage,
                                                  omni::physics::parse::ObjectKey jointKey);

    /**
     * Subscribe to physics object change notifications.
     *
     * @param[in] callback The callback structure with the notification functions.
     * @return Subscription Id to unsubscribe
     */
    SubscriptionId subscribeToObjectChangeNotifications(const IPhysicsObjectChangeCallback& callback);

    /**
     * Unsubscribe to physics object change notifications.
     *
     * @param[in] subscriptionId Subscription ID obtained via @ref subscribeObjectChangeNotifications.
     */
    void unsubscribeToObjectChangeNotifications(SubscriptionId subscriptionId);

    /**
     * Specifying whether object change notifications should be sent.
     *
     * @param[in] enable Defines whether object change notifications should be sent or not
     */
    void enableObjectChangeNotifications(bool enable)
    {
        mObjectChangeNotificationsEnabled = enable;
    }

    bool objectChangeNotificationsEnabled() const
    {
        return mObjectChangeNotificationsEnabled;
    }

    /**
     * Marks the window during which the initial stage population (attach/update
     * traversal) creates its physics objects.
     *
     * While this is set, the initial population is suppressed by default: no
     * object-change notification is delivered unless the subscriber explicitly
     * opts in via IPhysicsObjectChangeCallback::notifyInitialPopulation. This
     * default suppression applies even to subscribers that opt out of the
     * simulation-stopped gate (stopCallbackWhenSimStopped == false, e.g.
     * ovphysx), which the plain @ref enableObjectChangeNotifications gate does
     * not cover -- the documented contract promises the initial population is
     * not notified because the caller already has that state from setup.
     *
     * Prefer @ref InitialStagePopulationScope over calling this directly so the
     * flag is restored on every exit path, including exceptions.
     *
     * @param[in] inProgress Whether the initial stage population is in progress.
     */
    void setInitialStagePopulationInProgress(bool inProgress)
    {
        mInitialStagePopulationInProgress = inProgress;
    }

    bool initialStagePopulationInProgress() const
    {
        return mInitialStagePopulationInProgress;
    }

    void sendObjectCreationNotification(omni::physics::parse::ObjectKey key, usdparser::ObjectId objectId, PhysXType physxType);

    void sendObjectDestructionNotification(omni::physics::parse::ObjectKey key, usdparser::ObjectId objectId, PhysXType physxType);


    /**
     * handle usd API additions and removals that don't trigger a structural change (= prim recreate)
     *
     * @param path the prim path the API was added/removed
     * @param flag a SchemaAPIFlag specifying the API
     * @param removed whether the API was removed (true) or added (false)
     */
    void changeSchemaAPI(usdparser::AttachedStage& attachedStage,
                         omni::physics::parse::ObjectKey key,
                         usdparser::SchemaAPIFlag::Enum flag,
                         bool removed);

private:
    usdparser::ObjectId createVolumeDeformableBody(usdparser::AttachedStage& attachedStage,
                                                   omni::physics::parse::ObjectKey bodyKey,
                                                   usdparser::PhysxVolumeDeformableBodyDesc const& desc);
    usdparser::ObjectId createSurfaceDeformableBody(usdparser::AttachedStage& attachedStage,
                                                    omni::physics::parse::ObjectKey bodyKey,
                                                    usdparser::PhysxSurfaceDeformableBodyDesc const& desc);

    // Defined in UsdInterfaceParticle.cpp. The Hydra-rendering primvar authoring inside
    // createParticleSet stays individually fenced -- see its own comment.
    usdparser::ObjectId createPbdParticleSystem(usdparser::AttachedStage& attachedStage,
                                                omni::physics::parse::ObjectKey systemKey,
                                                const usdparser::ParticleSystemDesc& desc);

    usdparser::ObjectId createParticleSet(usdparser::AttachedStage& attachedStage,
                                          omni::physics::parse::ObjectKey primKey,
                                          const usdparser::ParticleSetDesc& particlesDesc);

    usdparser::ObjectId createDeformableAttachment(usdparser::AttachedStage& attachedStage,
                                                   omni::physics::parse::ObjectKey key,
                                                   const usdparser::PhysxDeformableAttachmentDesc& desc);
    usdparser::ObjectId createDeformableCollisionFilter(usdparser::AttachedStage& attachedStage,
                                                        omni::physics::parse::ObjectKey key,
                                                        const usdparser::PhysxDeformableCollisionFilterDesc& desc);

    // Vehicle creation/registration. The only USD dependency (xform-op authoring of
    // wheel/shape scale) lives in InternalVehicle.cpp's WheelTransformManagementEntry::init.
    usdparser::ObjectId createTireFrictionTable(const usdparser::TireFrictionTableDesc&);
    usdparser::ObjectId createVehicle(usdparser::AttachedStage& attachedStage,
                                      omni::physics::parse::ObjectKey vehicleKey,
                                      const usdparser::VehicleDesc& vehicleDesc);
    usdparser::ObjectId createVehicleController(usdparser::AttachedStage& attachedStage,
                                                omni::physics::parse::ObjectKey vehicleControllerKey,
                                                const usdparser::VehicleControllerDesc&);
    usdparser::ObjectId registerVehicleComponent(omni::physics::parse::ObjectKey key, PhysXType type);
    usdparser::ObjectId registerVehicleWheelComponent(omni::physics::parse::ObjectKey key, PhysXType type);
    usdparser::ObjectId createVehicleWheelController(usdparser::AttachedStage& attachedStage,
                                                     omni::physics::parse::ObjectKey wheelControllerKey,
                                                     const usdparser::WheelControllerDesc&);

    usdparser::ObjectId createMimicJoint(const usdparser::MimicJointDesc&);

    void changeParticlePostProcess(usdparser::AttachedStage& attachedStage,
                                   omni::physics::parse::ObjectKey key,
                                   bool removed,
                                   usdparser::SchemaAPIFlag::Enum flag);

    void changeParticleDiffuseParticles(usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, bool removed);

    void removeArticulationFromSceneAndScheduleForReAdd(const ::physx::PxArticulationReducedCoordinate&);

private:
    /**
     * Whether the given object-change subscription should receive a notification
     * right now. During the initial stage population it is suppressed by default,
     * with explicit per-subscriber opt-in via notifyInitialPopulation; otherwise
     * delivered when simulation notifications are enabled or the subscriber opted
     * out of the simulation-stopped gate.
     */
    bool shouldDeliverObjectChangeNotification(const IPhysicsObjectChangeCallback& callback) const
    {
        // The initial stage population is suppressed by default, even for subscribers that
        // opt out of the simulation-stopped gate (stopCallbackWhenSimStopped == false, e.g.
        // ovphysx) -- the caller already has that state from setup. Only a subscriber that
        // explicitly opts in via notifyInitialPopulation observes the initial population.
        if (mInitialStagePopulationInProgress)
            return callback.notifyInitialPopulation;
        return mObjectChangeNotificationsEnabled || !callback.stopCallbackWhenSimStopped;
    }

    std::vector<usdparser::ObjectId> mArticulations;
    std::vector<internal::InternalPbdParticleSystem*> mParticleSystems;
    PhysicsObjectChangeSubscriptionRegistry mPhysicsObjectChangeSubscriptions;
    // Kit-inspector-only debug filter storage; see setForceParseOnlySingleScene's comment above.
    std::string mForceParseOnlySingleScenePath;
    usdparser::ObjectId mForceParseOnlySingleSceneObjectId;
    bool mDirty;
    bool mObjectChangeNotificationsEnabled;
    bool mInitialStagePopulationInProgress = false;
    bool mExposePrimNames;
};


PhysXUsdPhysicsInterface& getPhysXUsdPhysicsInterface();

/**
 * RAII marker for the initial stage population (the attach/update traversal that
 * creates the physics objects). Enters the "initial population in progress" state
 * and disables object-change notifications on construction, and restores both
 * gates to their prior values on destruction -- including when loadFromStage()
 * throws (e.g. from an opted-in subscriber callback that drives a transactional
 * attach rollback). Saving and restoring the prior values keeps it nesting-safe.
 *
 * After a successful population the caller flips notifications on explicitly, so
 * that the running-simulation state is reached only when the load did not throw.
 */
class InitialStagePopulationScope
{
public:
    explicit InitialStagePopulationScope(PhysXUsdPhysicsInterface& iface)
        : mIface(iface)
        , mPrevInProgress(iface.initialStagePopulationInProgress())
        , mPrevNotificationsEnabled(iface.objectChangeNotificationsEnabled())
    {
        mIface.setInitialStagePopulationInProgress(true);
        mIface.enableObjectChangeNotifications(false);
    }

    ~InitialStagePopulationScope()
    {
        mIface.enableObjectChangeNotifications(mPrevNotificationsEnabled);
        mIface.setInitialStagePopulationInProgress(mPrevInProgress);
    }

    InitialStagePopulationScope(const InitialStagePopulationScope&) = delete;
    InitialStagePopulationScope& operator=(const InitialStagePopulationScope&) = delete;

private:
    PhysXUsdPhysicsInterface& mIface;
    bool mPrevInProgress;
    bool mPrevNotificationsEnabled;
};

void applyRigidDynamicPhysxDesc(PhysXScene* ps,
                                const usdparser::DynamicPhysxRigidBodyDesc& desc,
                                ::physx::PxRigidDynamic& rigidDynamic);

} // namespace physx
} // namespace omni

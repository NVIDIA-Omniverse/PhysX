// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "UsdPCH.h"

#include <PxPhysicsAPI.h>

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
    // stage
    void setForceParseOnlySingleScene(PXR_NS::SdfPath forceParseOnlySingleScenePath)
    {
        mForceParseOnlySingleScenePath = forceParseOnlySingleScenePath;
    }

    usdparser::ObjectId createObject(usdparser::AttachedStage& attachedStage,
                                     const PXR_NS::SdfPath& path,
                                     const usdparser::PhysxObjectDesc& objectDesc,
                                     const usdparser::ObjectInstance* instance = nullptr);

    usdparser::ObjectId createShape(const PXR_NS::SdfPath& path,
                                    const usdparser::PhysxObjectDesc& objectDesc,
                                    usdparser::ObjectId bodyId,
                                    const usdparser::ObjectInstance* instance = nullptr);

    MassInformation getShapeMassInfo(const PXR_NS::SdfPath& path, usdparser::ObjectId objectId) const;

    usdparser::ObjectId createJoint(usdparser::AttachedStage& attachedStage,
                                    const PXR_NS::SdfPath& path,
                                    const usdparser::PhysxJointDesc& desc,
                                    usdparser::ObjectId body0,
                                    usdparser::ObjectId body1);

    void recreateArticulationJoint(usdparser::AttachedStage&,
                                   const usdparser::PhysxJointDesc&,
                                   usdparser::ObjectId link0,
                                   usdparser::ObjectId link1);

    void releaseObject(usdparser::AttachedStage& attachedStage,
                       const PXR_NS::SdfPath& removedPath,
                       usdparser::ObjectId objectId);

    void fillChangeParams(std::vector<usdparser::ChangeParams>& changeParams);

    bool updateTransform(const usdparser::AttachedStage& attachedStage,
                         const PXR_NS::SdfPath& path,
                         usdparser::ObjectId objectId,
                         const Transform& transform,
                         bool resetVelocity = true,
                         bool scaleProvided = true);
    bool updateTransform(const usdparser::AttachedStage& attachedStage,
                         omni::physics::parse::ObjectKey key,
                         usdparser::ObjectId objectId,
                         const Transform& transform,
                         bool resetVelocity = true,
                         bool scaleProvided = true);

    bool updateMass(const PXR_NS::SdfPath& path,
                    usdparser::ObjectId objectId,
                    float mass,
                    const carb::Float3& diagInertia,
                    const carb::Float3& com,
                    const carb::Float4& principalAxes);

    bool updateDeformableBodyMass(const usdparser::AttachedStage& attachedStage, usdparser::ObjectId objectId);
    bool updateDeformableBodyPositions(const usdparser::AttachedStage& attachedStage, const usdparser::ObjectId objectId);
    bool updateDeformableBodyVelocities(const usdparser::AttachedStage& attachedStage, const usdparser::ObjectId objectId);
    bool updateDeformableRestOffset(const usdparser::AttachedStage& attachedStage,
                                    usdparser::ObjectId objectId,
                                    float value);
    bool updateDeformableContactOffset(const usdparser::AttachedStage& attachedStage,
                                       usdparser::ObjectId objectId,
                                       float value);
    bool updateDeformableSelfCollisionFilterDistance(const usdparser::AttachedStage& attachedStage,
                                                     usdparser::ObjectId objectId,
                                                     float value);
    bool updateParticleMass(const PXR_NS::SdfPath& path,
                            usdparser::ObjectId objectId,
                            const usdparser::ParticleDesc& particleDesc);

    bool updateObject(usdparser::AttachedStage& attachedStage,
                      const PXR_NS::SdfPath& path,
                      usdparser::ObjectId objectId,
                      usdparser::OnUpdateObjectFn updateFn,
                      const PXR_NS::TfToken& propertyName,
                      const PXR_NS::UsdTimeCode& timeCode);

    void setupCollisionGroup(const PXR_NS::SdfPath& path, const usdparser::CollisionGroupDesc& desc);

    bool setVehicleContext(const usdparser::AttachedStage& attachedStage, const usdparser::VehicleContextDesc&);

    void releaseAllObjects();

    // get shapes for mass computation, return true if triggers are present
    bool getRigidBodyShapes(const usdparser::AttachedStage& attachedStage,
                            usdparser::ObjectId rbId,
                            usdparser::ObjectIdPathMap& shapes) const;

    static usdparser::ObjectId createShapeOrComputeMass(const PXR_NS::SdfPath& path,
                                                        const usdparser::PhysxShapeDesc& shapeDesc,
                                                        usdparser::ObjectId bodyId,
                                                        const usdparser::ObjectInstance* instance,
                                                        PXR_NS::UsdStageWeakPtr stage,
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

    PXR_NS::SdfPath getParentJointPathInArticulation(const usdparser::AttachedStage& attachedStage,
                                                  const PXR_NS::SdfPath& jointKey);

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

    void sendObjectCreationNotification(const PXR_NS::SdfPath& path, usdparser::ObjectId objectId, PhysXType physxType);

    void sendObjectDestructionNotification(const PXR_NS::SdfPath& path, usdparser::ObjectId objectId, PhysXType physxType);


    /**
     * handle usd API additions and removals that don't trigger a structural change (= prim recreate)
     *
     * @param path the prim path the API was added/removed
     * @param flag a SchemaAPIFlag specifying the API
     * @param removed whether the API was removed (true) or added (false)
     */
    void changeSchemaAPI(usdparser::AttachedStage& attachedStage,
                         const PXR_NS::SdfPath& path,
                         usdparser::SchemaAPIFlag::Enum flag,
                         bool removed);

private:
    usdparser::ObjectId createPbdParticleSystem(usdparser::AttachedStage& attachedStage,
                                                const PXR_NS::SdfPath& path,
                                                const usdparser::ParticleSystemDesc& desc);

    usdparser::ObjectId createParticleSet(usdparser::AttachedStage& attachedStage,
                                          const PXR_NS::SdfPath& path,
                                          const usdparser::ParticleSetDesc& particlesDesc);
    usdparser::ObjectId createVolumeDeformableBody(usdparser::AttachedStage& attachedStage,
                                                   const PXR_NS::SdfPath& path,
                                                   usdparser::PhysxVolumeDeformableBodyDesc const& desc);
    usdparser::ObjectId createSurfaceDeformableBody(usdparser::AttachedStage& attachedStage,
                                                    const PXR_NS::SdfPath& path,
                                                    usdparser::PhysxSurfaceDeformableBodyDesc const& desc);
    usdparser::ObjectId createDeformableAttachment(usdparser::AttachedStage& attachedStage,
                                                   const PXR_NS::SdfPath& path,
                                                   const usdparser::PhysxDeformableAttachmentDesc& desc);
    usdparser::ObjectId createDeformableCollisionFilter(usdparser::AttachedStage& attachedStage,
                                                        const PXR_NS::SdfPath& path,
                                                        const usdparser::PhysxDeformableCollisionFilterDesc& desc);

    usdparser::ObjectId createTireFrictionTable(const usdparser::TireFrictionTableDesc&, const PXR_NS::UsdPrim&);
    usdparser::ObjectId createVehicle(const PXR_NS::SdfPath& vehiclePath,
                                      const usdparser::VehicleDesc& vehicleDesc,
                                      const PXR_NS::UsdPrim& usdPrim,
                                      PXR_NS::UsdStageRefPtr usdStage);
    usdparser::ObjectId createVehicleController(const PXR_NS::SdfPath&,
                                                const PXR_NS::UsdPrim&,
                                                const usdparser::VehicleControllerDesc&);
    usdparser::ObjectId registerVehicleComponent(const PXR_NS::UsdPrim& usdPrim, PhysXType type);
    usdparser::ObjectId registerVehicleWheelComponent(const PXR_NS::UsdPrim&, PhysXType type);
    usdparser::ObjectId createVehicleWheelController(const PXR_NS::SdfPath&,
                                                     const PXR_NS::UsdPrim&,
                                                     const usdparser::WheelControllerDesc&);

    usdparser::ObjectId createMimicJoint(const usdparser::MimicJointDesc&);

    void changeParticlePostProcess(usdparser::AttachedStage& attachedStage,
                                   const PXR_NS::SdfPath& path,
                                   bool removed,
                                   usdparser::SchemaAPIFlag::Enum flag);

    void changeParticleDiffuseParticles(usdparser::AttachedStage& attachedStage, const PXR_NS::SdfPath& path, bool removed);

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
    PXR_NS::SdfPath mForceParseOnlySingleScenePath;
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

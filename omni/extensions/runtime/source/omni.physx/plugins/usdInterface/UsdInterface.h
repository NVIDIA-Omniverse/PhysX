// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include <PxPhysicsAPI.h>

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/PhysxUsd.h>

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
class InternalAttachmentDeprecated;
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
    void setForceParseOnlySingleScene(pxr::SdfPath forceParseOnlySingleScenePath)
    {
        mForceParseOnlySingleScenePath = forceParseOnlySingleScenePath;
    }

    usdparser::ObjectId createObject(usdparser::AttachedStage& attachedStage,
                                     const pxr::SdfPath& path,
                                     const usdparser::PhysxObjectDesc& objectDesc,
                                     const usdparser::ObjectInstance* instance = nullptr);

    usdparser::ObjectId createShape(const pxr::SdfPath& path,
                                    const usdparser::PhysxObjectDesc& objectDesc,
                                    usdparser::ObjectId bodyId,
                                    const usdparser::ObjectInstance* instance = nullptr);

    MassInformation getShapeMassInfo(const pxr::SdfPath& path, usdparser::ObjectId objectId) const;

    usdparser::ObjectId createJoint(usdparser::AttachedStage& attachedStage,
                                    const pxr::SdfPath& path,
                                    const usdparser::PhysxJointDesc& desc,
                                    usdparser::ObjectId body0,
                                    usdparser::ObjectId body1);

    void recreateArticulationJoint(usdparser::AttachedStage&,
                                   const usdparser::PhysxJointDesc&,
                                   usdparser::ObjectId link0,
                                   usdparser::ObjectId link1);

    void releaseObject(usdparser::AttachedStage& attachedStage,
                       const pxr::SdfPath& removedPath,
                       usdparser::ObjectId objectId);

    void fillChangeParams(std::vector<usdparser::ChangeParams>& changeParams);

    bool updateTransform(const usdparser::AttachedStage& attachedStage,
                         const pxr::SdfPath& path,
                         usdparser::ObjectId objectId,
                         const Transform& transform,
                         bool resetVelocity = true,
                         bool scaleProvided = true);

    bool updateKinematicVertexTargetsFromSkinDeprecated(const pxr::SdfPath& path,
                                                        usdparser::ObjectId objectId,
                                                        const carb::Float3* skinPoints,
                                                        const size_t skinPointsSize,
                                                        const pxr::GfMatrix4d& transformTimeSampled);

    bool updateKinematicVertexTargetsFromSimDeprecated(const pxr::SdfPath& path,
                                                       usdparser::ObjectId objectId,
                                                       const carb::Float4* simPoints,
                                                       const size_t simPointsSize);

    bool updateMass(const pxr::SdfPath& path,
                    usdparser::ObjectId objectId,
                    float mass,
                    const carb::Float3& diagInertia,
                    const carb::Float3& com,
                    const carb::Float4& principalAxes);

    bool updateDeformableBodyMassDeprecated(const pxr::SdfPath& path,
                                            usdparser::ObjectId objectId,
                                            const usdparser::SoftBodyDesc& softBodyDesc);
    bool updateDeformableSurfaceMassDeprecated(const pxr::SdfPath& path,
                                               usdparser::ObjectId objectId,
                                               const usdparser::FEMClothDesc& clothDesc);

    bool updateDeformableBodyPositionsDeprecated(const usdparser::AttachedStage& attachedStage,
                                                 const usdparser::ObjectId objectId);
    bool updateDeformableBodyVelocitiesDeprecated(const usdparser::AttachedStage& attachedStage,
                                                  const usdparser::ObjectId objectId);
    bool updateDeformableSurfacePositionsDeprecated(const usdparser::AttachedStage& attachedStage,
                                                    const usdparser::ObjectId objectId);
    bool updateDeformableSurfaceVelocitiesDeprecated(const usdparser::AttachedStage& attachedStage,
                                                     const usdparser::ObjectId objectId);

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
    bool updateParticleMass(const pxr::SdfPath& path,
                            usdparser::ObjectId objectId,
                            const usdparser::ParticleDesc& particleDesc);

    bool updateObject(usdparser::AttachedStage& attachedStage,
                      const pxr::SdfPath& path,
                      usdparser::ObjectId objectId,
                      usdparser::OnUpdateObjectFn updateFn,
                      const pxr::TfToken& propertyName,
                      const pxr::UsdTimeCode& timeCode);

    void setupCollisionGroup(const pxr::SdfPath& path, const usdparser::CollisionGroupDesc& desc);

    bool setVehicleContext(const usdparser::AttachedStage& attachedStage, const usdparser::VehicleContextDesc&);

    void releaseAllObjects();

    // get shapes for mass computation, return true if triggers are present
    bool getRigidBodyShapes(const usdparser::AttachedStage& attachedStage,
                            usdparser::ObjectId rbId,
                            usdparser::ObjectIdUsdPrimMap& shapes) const;

    static usdparser::ObjectId createShapeOrComputeMass(const pxr::SdfPath& path,
                                                        const usdparser::PhysxShapeDesc& shapeDesc,
                                                        usdparser::ObjectId bodyId,
                                                        const usdparser::ObjectInstance* instance,
                                                        pxr::UsdStageWeakPtr stage,
                                                        PhysXScene* physxScene,
                                                        bool exposePrimNames,
                                                        PhysXType& physxType,
                                                        internal::InternalPhysXDatabase* db,
                                                        PhysXUsdPhysicsInterface::MassInformation* massInfoOut,
                                                        pxr::UsdGeomXformCache* xformCache);

    void finishSetup(const usdparser::AttachedStage& attachedStage);
    void finalizeArticulations(const usdparser::AttachedStage& attachedStage);

    void finalizeDeformableRigidAttachmentsDeprecated();
    void updateDeformableAttachmentsDeprecated();
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

    pxr::SdfPath getParentJointPathInArticulation(const usdparser::AttachedStage& attachedStage,
                                                  const pxr::SdfPath& jointPath);

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

    void sendObjectCreationNotification(const pxr::SdfPath& path, usdparser::ObjectId objectId, PhysXType physxType);

    void sendObjectDestructionNotification(const pxr::SdfPath& path, usdparser::ObjectId objectId, PhysXType physxType);


    /**
     * handle usd API additions and removals that don't trigger a structural change (= prim recreate)
     *
     * @param path the prim path the API was added/removed
     * @param flag a SchemaAPIFlag specifying the API
     * @param removed whether the API was removed (true) or added (false)
     */
    void changeSchemaAPI(usdparser::AttachedStage& attachedStage,
                         const pxr::SdfPath& path,
                         usdparser::SchemaAPIFlag::Enum flag,
                         bool removed);

private:
    usdparser::ObjectId createPbdParticleSystem(usdparser::AttachedStage& attachedStage,
                                                const pxr::SdfPath& path,
                                                const usdparser::ParticleSystemDesc& desc);

    usdparser::ObjectId createParticleSet(usdparser::AttachedStage& attachedStage,
                                          const pxr::SdfPath& path,
                                          const usdparser::ParticleSetDesc& particlesDesc);
    usdparser::ObjectId createVolumeDeformableBody(usdparser::AttachedStage& attachedStage,
                                                   const pxr::SdfPath& path,
                                                   usdparser::PhysxVolumeDeformableBodyDesc const& desc);
    usdparser::ObjectId createSurfaceDeformableBody(usdparser::AttachedStage& attachedStage,
                                                    const pxr::SdfPath& path,
                                                    usdparser::PhysxSurfaceDeformableBodyDesc const& desc);
    usdparser::ObjectId createDeformableAttachment(usdparser::AttachedStage& attachedStage,
                                                   const pxr::SdfPath& path,
                                                   const usdparser::PhysxDeformableAttachmentDesc& desc);
    usdparser::ObjectId createDeformableCollisionFilter(usdparser::AttachedStage& attachedStage,
                                                        const pxr::SdfPath& path,
                                                        const usdparser::PhysxDeformableCollisionFilterDesc& desc);

    // DEPRECATED
    usdparser::ObjectId createParticleClothDeprecated(usdparser::AttachedStage& attachedStage,
                                                      const pxr::SdfPath& path,
                                                      usdparser::ParticleClothDesc const& clothDesc);
    usdparser::ObjectId createDeformableBodyDeprecated(const usdparser::AttachedStage& attachedStage,
                                                       const pxr::SdfPath& path,
                                                       usdparser::SoftBodyDesc const& softBodyDesc);
    usdparser::ObjectId createDeformableSurfaceDeprecated(const usdparser::AttachedStage& attachedStage,
                                                          const pxr::SdfPath& path,
                                                          usdparser::FEMClothDesc const& femClothDesc);
    usdparser::ObjectId createPhysicsAttachmentDeprecated(usdparser::AttachedStage& attachedStage,
                                                          const pxr::SdfPath& path,
                                                          usdparser::PhysxAttachmentDesc const& attachmentDesc);
    //~DEPRECATED

    usdparser::ObjectId createTireFrictionTable(const usdparser::TireFrictionTableDesc&, const pxr::UsdPrim&);
    usdparser::ObjectId createVehicle(const pxr::SdfPath& vehiclePath,
                                      const usdparser::VehicleDesc& vehicleDesc,
                                      const pxr::UsdPrim& usdPrim,
                                      pxr::UsdStageRefPtr usdStage);
    usdparser::ObjectId createVehicleController(const pxr::SdfPath&,
                                                const pxr::UsdPrim&,
                                                const usdparser::VehicleControllerDesc&);
    usdparser::ObjectId registerVehicleComponent(const pxr::UsdPrim& usdPrim, PhysXType type);
    usdparser::ObjectId registerVehicleWheelComponent(const pxr::UsdPrim&, PhysXType type);
    usdparser::ObjectId createVehicleWheelController(const pxr::SdfPath&,
                                                     const pxr::UsdPrim&,
                                                     const usdparser::WheelControllerDesc&);

    usdparser::ObjectId createMimicJoint(const usdparser::MimicJointDesc&);

    void changeParticlePostProcess(usdparser::AttachedStage& attachedStage,
                                   const pxr::SdfPath& path,
                                   bool removed,
                                   usdparser::SchemaAPIFlag::Enum flag);

    void changeParticleDiffuseParticles(usdparser::AttachedStage& attachedStage, const pxr::SdfPath& path, bool removed);

    void removeArticulationFromSceneAndScheduleForReAdd(const ::physx::PxArticulationReducedCoordinate&);

private:
    std::vector<usdparser::ObjectId> mArticulations;
    std::vector<internal::InternalAttachmentDeprecated*> mAttachmentsToCreateDeprecated;
    std::vector<internal::InternalPbdParticleSystem*> mParticleSystems;
    PhysicsObjectChangeSubscriptionRegistry mPhysicsObjectChangeSubscriptions;
    pxr::SdfPath mForceParseOnlySingleScenePath;
    usdparser::ObjectId mForceParseOnlySingleSceneObjectId;
    bool mDirty;
    bool mObjectChangeNotificationsEnabled;
    bool mExposePrimNames;
};


PhysXUsdPhysicsInterface& getPhysXUsdPhysicsInterface();

void applyRigidDynamicPhysxDesc(PhysXScene* ps,
                                const usdparser::DynamicPhysxRigidBodyDesc& desc,
                                ::physx::PxRigidDynamic& rigidDynamic);

} // namespace physx
} // namespace omni

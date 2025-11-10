// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/events/IEvents.h>
// #include <private/omni/physx/PhysxUsd.h>
#include "EventSubscriptionRegistry.h"
#include "ObjectId.h"

namespace omni
{

namespace physx
{

enum PhysXType
{
    ePTRemoved,
    ePTScene,
    ePTMaterial,
    ePTShape,
    ePTCompoundShape,
    ePTActor,
    ePTJoint,
    ePTCustomJoint,
    ePTArticulation,
    ePTLink,
    ePTLinkJoint,
    ePTParticleSystem,
    ePTParticleSet,
    ePTParticleClothDeprecated, // DEPRECATED, will be replaced by new deformable implementation in future release.
    ePTSoftBodyMaterialDeprecated, // DEPRECATED, will be replaced by new deformable implementation in future release.
    ePTFEMClothMaterialDeprecated, // DEPRECATED, will be replaced by new deformable implementation in future release.
    ePTSoftBodyDeprecated, // DEPRECATED, will be replaced by new deformable implementation in future release.
    ePTFEMClothDeprecated, // DEPRECATED, will be replaced by new deformable implementation in future release.
    ePTAttachmentDeprecated, // DEPRECATED, will be replaced by new attachment implementation in future release.
    ePTCollisionGroup,
    ePTCct,
    ePTVehicle,
    ePTVehicleController,
    ePTVehicleEngine,
    ePTVehicleTireFrictionTable,
    ePTVehicleSuspension,
    ePTVehicleTire,
    ePTVehicleWheel,
    ePTVehicleWheelAttachment,
    ePTVehicleWheelController,
    ePTVehicleDriveBasic,
    ePTPhysics,
    ePTPointInstancer,
    ePTFixedTendonAxis,
    ePTTendonAttachment,
    ePTInfiniteVoxelMap,
    ePTPBDMaterial,
    ePTForce,
    ePTFilteredPair,
    ePTArticulationFixedBase, // This is dummy ptr to articulation, since we convert the fixed base joint into fixed
                              // base articulation
    ePTMimicJoint,
    ePTDeformableSurfaceMaterial,
    ePTDeformableVolumeMaterial,
    ePTDeformableSurface,
    ePTDeformableVolume,
    ePTDeformableAttachment,
    ePTDeformableCollisionFilter,
    ePTXformActor,
};

enum ForceMode
{
    eForce, //!< parameter has unit of mass * distance/ time^2, i.e. a force
    eImpulse, //!< parameter has unit of mass * distance /time (not used for angular)
    eVelocityChange, //!< parameter has unit of distance / time, i.e. the effect is mass independent: a velocity change.
    eAcceleration //!< parameter has unit of distance/ time^2, i.e. an acceleration. It gets treated just like a force
                  //!< except the mass is not divided out before integration.
};

/// Simulation events received through simulation event stream
enum SimulationEvent
{
    eResumed, //!< When simulation is resumed
    ePaused, //!< When simulation is paused
    eStopped, //!< When simulation is stopped
    eContactFound, //!< Deprecated, please use subscribePhysicsContactReportEvents in IPhysxSimulation
    eContactLost, //!< Deprecated, please use subscribePhysicsContactReportEvents in IPhysxSimulation
    eContactPersists, //!< Deprecated, please use subscribePhysicsContactReportEvents in IPhysxSimulation
    eContactData, //!< Deprecated, please use subscribePhysicsContactReportEvents in IPhysxSimulation
    eJointBreak, //!< When a joint breaks
    ePointGrabbed, //!< When a point is grabbed
    ePointReleased, //!< When a grabbed point is released
    eAttachedToStage, //!< When physx stage attachment (initialization) finished
    eDetachedFromStage, //!< When physx stage detachment (deinitialization) finished
    ePointPushed //!< When a point is force pushed.
};

enum SimulationStatusEvent
{
    eSimulationStarting, //!< Called prior to each substep of the simulate loop.
    eSimulationEnded, //!< Called after each substep of the simulate loop.
    eSimulationComplete //!< Called after all of the simulation substeps have completed.
                        //!< If the simulation is run asynchronoulsy, it is safe to read and write
                        //!< data to/from the simulation during this event.
};

enum ErrorEvent
{
    eUsdLoadError,
    ePhysxError,
    ePhysxCudaError,
    ePhysxTooManyErrors
};

enum struct PhysicsInteractionEvent
{
    eMouseDragBegan,
    eMouseDragChanged,
    eMouseDragEnded,
    eMouseLeftClick,
    eMouseLeftDoubleClick,
    eNone
};

typedef void (*OnPhysicsStepEventFn)(float elapsedTime, void* userData);
typedef void (*OnPhysicsSimulationEventFn)(SimulationStatusEvent eventStatus, void* userData);

/// Notification when a physics object gets created during simulation.
///
/// Whenever a USD operation triggers the creation of a physics object, this notification
/// will fire.
///
/// \note Only covers creations that occur after the simulation has started. No notifications
///       will be sent for the object creation at simulation start.
///
/// \note The notification happens after the object has been created.
///
/// \param[in] sdfPath SdfPath of the prim that triggered the object creation.
/// \param[in] objectId The internal object ID that can be used to directly access PhysX objects.
/// \param[in] type The type of the created object.
/// \param[in] userData The user data provided by the user when registering the IPhysicsObjectChangeCallback
///            structure.
using ObjectCreationNotificationFn =
    std::function<void(const pxr::SdfPath& sdfPath, usdparser::ObjectId objectId, PhysXType type, void* userData)>;

/// Notification when a physics object gets destroyed during simulation.
///
/// Whenever a USD operation triggers the destruction of a physics object, this notification
/// will fire.
///
/// \note Only covers destructions that occur after the simulation has started. No notifications
///       will be sent for the object destruction at simulation end. Furthermore, no notifications
///       will be sent if IPhysx.releasePhysicsObjects() is called (see AllObjectsDestructionNotificationFn
///       to cover that or similar scenarios).
///
/// \note The notification happens before the object gets destroyed.
///
/// \param[in] sdfPath SdfPath of the prim that triggered the object destruction.
/// \param[in] objectId The internal object ID that can be used to directly access PhysX objects.
/// \param[in] type The type of the object that is going to get destroyed.
/// \param[in] userData The user data provided by the user when registering the IPhysicsObjectChangeCallback
///            structure.
using ObjectDestructionNotificationFn =
    std::function<void(const pxr::SdfPath& sdfPath, usdparser::ObjectId objectId, PhysXType type, void* userData)>;

/// Notification when all physics objects get destroyed during simulation.
///
/// \note Only covers destructions that occur after the simulation has started. No notifications
///       will be sent for the object destruction at simulation end.
///
/// \note The notification happens before the objects get destroyed.
///
/// \param[in] userData The user data provided by the user when registering the IPhysicsObjectChangeCallback
///            structure.
using AllObjectsDestructionNotificationFn = std::function<void(void* userData)>;

/// Structure to specify callbacks to send notifications when physics objects get created or
/// destroyed during simulation.
struct IPhysicsObjectChangeCallback
{
    ObjectCreationNotificationFn objectCreationNotifyFn = { nullptr };
    ObjectDestructionNotificationFn objectDestructionNotifyFn = { nullptr };
    AllObjectsDestructionNotificationFn allObjectsDestructionNotifyFn = { nullptr };

    /// Pointer to pass user data into the callback functions
    void* userData = { nullptr };

    /// Whether these callbacks should not be sent when the simulation stops
    bool stopCallbackWhenSimStopped = true;
};

using PhysicsObjectChangeSubscriptionRegistry = EventSubscriptionRegistry<omni::physx::IPhysicsObjectChangeCallback>;

/// Struct holding the drive related state of a vehicle.
///
struct VehicleDriveState
{
    /// State of the accelerator (in range [0, 1])
    ///
    float accelerator;

    /// State of the brake 0 control (in range [0, 1])
    ///
    float brake0;

    /// State of the brake 1 control (in range [0, 1])
    ///
    float brake1;

    /// State of steering (in range [-1, 1], -1 usually being full left and 1 full right)
    ///
    float steer;

    /// State of the clutch (in range [0, 1])
    ///
    /// \note Only valid if the vehicle uses a drive of type PhysxVehicleDriveStandard, else 0.
    float clutch;

    /// Currently active gear
    ///
    /// \note Only valid if the vehicle uses a drive of type PhysxVehicleDriveStandard, else 0.
    int currentGear;

    /// Gear to shift to
    ///
    /// \note This is the internal target gear and does not necessarily match the one specified
    ///       in the vehicle controller (which might, for example, hold the special value for
    ///       automatic gear change mode. Or a gear shift might be in flight while the controller
    ///       already requests a different target gear).
    ///
    /// \note Only valid if the vehicle uses a drive of type PhysxVehicleDriveStandard, else 0.
    int targetGear;

    /// Time that has passed since the current gear change was initiated (in seconds).
    ///
    /// \note Negative value if no gear change is in process.
    ///
    /// \note Only valid if the vehicle uses a drive of type PhysxVehicleDriveStandard, else 0.
    float gearSwitchTime;

    /// Time that has passed since the last autobox gear shift (in seconds).
    ///
    /// \note Only valid if the vehicle uses a drive of type PhysxVehicleDriveStandard, else 0.
    float autoboxTimeSinceLastShift;

    /// Current engine rotation speed (in radians per second)
    ///
    /// \note Only valid if the vehicle uses a drive of type PhysxVehicleDriveStandard, else 0.
    float engineRotationSpeed;

    /// Whether automatic transmission is enabled or not
    ///
    /// \note Only valid if the vehicle uses a drive of type PhysxVehicleDriveStandard, else false.
    bool automaticTransmission;
};

/// Struct holding various vehicle wheel related states.
///
struct VehicleWheelState
{
    /// Position of the wheel in scaled local space (either vehicle center of mass frame
    /// if the custom metadata physxVehicle:referenceFrameIsCenterOfMass is true or else
    /// the vehicle prim frame). The total scale has been applied already.
    ///
    carb::Float3 localPosePosition;

    /// Orientation of the wheel in local space (either vehicle center of mass frame
    /// if the custom metadata physxVehicle:referenceFrameIsCenterOfMass is true or else
    /// the vehicle prim frame).
    ///
    carb::Float4 localPoseQuaternion;

    /// The rotation speed about the rolling axis of the wheel (in radians per second)
    ///
    float rotationSpeed;

    /// The rotation angle about the rolling axis of the wheel (in radians)
    ///
    float rotationAngle;

    /// The steer angle of the wheel (in radians)
    ///
    float steerAngle;

    /// The plane equation of the ground surface the wheel is driving on
    ///
    /// \note Only valid if #isOnGround is true.
    ///
    /// The (x, y, z) entries hold the plane normal n and the w value the
    /// parameter d such that: n.dot(v) + d = 0 for all points v on the plane.
    ///
    carb::Float4 groundPlane;

    /// The object ID of the PhysX actor the wheel is driving on
    ///
    /// \note Only valid if #isOnGround is true.
    ///
    usdparser::ObjectId groundActor;

    /// The object ID of the PhysX shape the wheel is driving on
    ///
    /// \note Only valid if #isOnGround is true.
    ///
    usdparser::ObjectId groundShape;

    /// The object ID of the PhysX material the wheel is driving on
    ///
    /// \note Only valid if #isOnGround is true.
    ///
    usdparser::ObjectId groundMaterial;

    /// The wheel query hit position on the ground (in world space)
    ///
    /// \note Only valid if #isOnGround is true.
    ///
    carb::Float3 groundHitPosition;

    /// The jounce of the suspension
    ///
    /// Zero means the suspension is fully elongated and a value of
    /// travelDistance (see USD schema PhysxVehicleSuspensionAPI) means fully compressed.
    ///
    float suspensionJounce;

    /// The force the suspension created (in world frame)
    ///
    carb::Float3 suspensionForce;

    /// The friction value used for the tire
    ///
    /// Is the product of the road geometry friction and a friction response multiplier.
    ///
    float tireFriction;

    /// The longitudinal direction of the tire (in world frame)
    ///
    carb::Float3 tireLongitudinalDirection;

    /// The lateral direction of the tire (in world frame)
    ///
    carb::Float3 tireLateralDirection;

    /// The longitudinal slip value of the tire
    ///
    /// Approximately (w*r - vz) / PxAbs(vz) where w is the angular speed of the wheel, r is the radius of the wheel,
    /// and vz the component of the vehicle body velocity computed at the wheel base along the longitudinal direction
    /// of the tire.
    ///
    float tireLongitudinalSlip;

    /// The lateral slip value of the tire
    ///
    /// Approximately PxAtan(vx / PxAbs(vz)) where vx and vz are the components of the vehicle body velocity at the
    /// wheel base along the wheel's lateral and longitudinal directions, respectively.
    ///
    float tireLateralSlip;

    /// The force the tire created (in world frame)
    ///
    carb::Float3 tireForce;

    /// Whether the wheel did touch the ground or is in air
    ///
    /// \note If the vehicle is disabled or sleeping then this value will be set
    ///       to false too since related ground collision data might not be valid
    ///       any longer.
    ///
    bool isOnGround;
};


/// General omni.physx plugin interface
struct IPhysx
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysx", 5, 0)

    /// Get the internal objectId, this is index to an array of
    /// internal physics representation. This index can be used for direct access.
    ///
    /// \param path Usd path where the physics object was created
    /// \param type Physics type, note that there can be more than one object per path, so a type is required to return
    /// correct result
    ///
    /// \returns the index of the physics object in the internal array
    usdparser::ObjectId(CARB_ABI* getObjectId)(const pxr::SdfPath& path, PhysXType type);

    /// Get the PhysX pointer
    ///
    /// \param path Usd path where the physics object was created
    /// \param type Physics type, note that there can be more than one object per path, so a type is required to return
    /// correct result
    ///
    /// \returns the pointer to a PhysX representation
    void*(CARB_ABI* getPhysXPtr)(const pxr::SdfPath& path, PhysXType type);

    /// Get the PhysX pointer
    ///
    /// \param objectId ObjectId, index to an array of physics objects in PhysX representation
    ///
    /// \returns the pointer to a PhysX representation (or nullptr if there is no directly
    ///          accessible object)
    void*(CARB_ABI* getPhysXPtrFast)(usdparser::ObjectId objectId);

    /// Get the USD path for given objectId
    ///
    /// \param objectId ObjectId, index to an array of physics objects in PhysX representation
    ///
    /// \returns the USD path for given objectId index
    pxr::SdfPath(CARB_ABI* getPhysXObjectUsdPath)(usdparser::ObjectId objectId);

    /// Forces load physics objects from USD into PhysX
    ///
    void(CARB_ABI* forceLoadPhysicsFromUSD)();

    /// Release physics objects from PhysX
    ///
    void(CARB_ABI* releasePhysicsObjects)();

    /// create D6 joint (temp until we support D6 through schema)
    const void*(CARB_ABI* createD6JointAtPath)(const pxr::SdfPath& path,
                                               const pxr::SdfPath& body0,
                                               const pxr::SdfPath& body1);

    /// release D6 joint
    void(CARB_ABI* releaseD6Joint)(void* jointPtr);


    /// set num threads for simulation
    /// \param threadCount num Threads
    void(CARB_ABI* setThreadCount)(uint32_t threadCount);

    /// -1: use setting from schema, 0: force CPU, 1: force GPU
    void(CARB_ABI* overwriteGPUSetting)(int val);
    int(CARB_ABI* getOverwriteGPUSetting)();

    /// -1: use setting from schema, 0: force PGS, 1: force TGS
    void(CARB_ABI* overwriteSolverType)(int val);

    /// Update the physics simulation. All scenes, except the ones marked as 'Disabled', are updated and stepped.
    ///
    /// \param[in] elapsedStep Simulation time.
    /// \param[in] currentTime Current time, might be used for time sampled transformations to apply.
    void(CARB_ABI* updateSimulation)(float elapsedStep, float currentTime);

    /// Update all the transformations. All scenes, except the ones marked as 'Disabled', are updated and stepped.
    /// Note: the 'outputVelocitiesLocalSpace' parameter has no effect and will be removed in a future version.
    ///
    /// \param[in] updateToFastCache            Update transforms to fast cache. DEPRECATED, fast cache is not used
    /// anymore \param[in] updateToUsd                  Update transforms to USD. \param[in] updateVelocitiesToUsd
    /// Update velocities to USD.
    void(CARB_ABI* updateTransformations)(bool updateToFastCache,
                                          bool updateToUsd,
                                          bool updateVelocitiesToUsd,
                                          bool outputVelocitiesLocalSpace);

    /// Start simulation, store initial USD data
    void(CARB_ABI* startSimulation)();

    /// Reset simulation, set back USD start data
    void(CARB_ABI* resetSimulation)();

    /// Reconnect PVD
    void(CARB_ABI* reconnectPVD)();

    /// Set voxel range in voxelmap if possible
    ///
    /// \param[in] stageId    Stage ID
    /// \param[in] path       VoxelMap USD sdf path.
    /// \param[in] sx         Range start X
    /// \param[in] sy         Range start Y
    /// \param[in] sz         Range start Z
    /// \param[in] ex         Range end X
    /// \param[in] ey         Range end Y
    /// \param[in] ez         Range end Z
    /// \param[in] type       Voxel type
    /// \param[in] subType    Voxel subtype
    /// \param[in] update Update flag. If zero, writing changes to USD is postponed, if non-zero, all accumulated
    /// changes are written to USD \return True if successful
    bool(CARB_ABI* setVoxelRange)(long int stageId,
                                  const pxr::SdfPath& path,
                                  const int sx,
                                  const int sy,
                                  const int sz,
                                  const int ex,
                                  const int ey,
                                  const int ez,
                                  const int type,
                                  const int subType,
                                  const int update);

    /// Get the index of a wheel given the wheel attachment USD path. Should only be called after the
    /// simulation has been started.
    ///
    /// \param[in] wheelPath      Wheel attachment USD path.
    /// \return Index of the wheel or -1 if there is no wheel matching the given path.
    int(CARB_ABI* getWheelIndex)(const pxr::SdfPath& wheelPath);

    /// Error event stream
    /// \return a pointer to the error event stream.
    carb::events::IEventStreamPtr(CARB_ABI* getErrorEventStream)();

    /// Settings a simulation layer, physics updates will be saved into provided layer
    /// \param[in] layerIdentifier Sdf layer string identifier.
    void(CARB_ABI* setSimulationLayer)(const char* layerIdentifier);

    /// Subscribe to physics simulation status events.
    ///
    /// \note Subscription cannot be changed in the onEvent callback
    ///
    /// \param onEvent The callback function to be called on simulation change.
    /// \param userData The userData to be passed back in the callback function.
    /// \return Subscription Id for release, kInvalidSubscriptionId is returned if the opetation failed
    SubscriptionId(CARB_ABI* subscribePhysicsSimulationEvents)(OnPhysicsSimulationEventFn onEvent, void* userData);

    /// Unsubscribes to simulation events.
    ///
    /// \note Subscription cannot be changed in the onEvent callback
    ///
    /// subscriptionId SubscriptionId obtained via @ref subscribePhysicsSimulationEvents.
    void(CARB_ABI* unsubscribePhysicsSimulationEvents)(SubscriptionId subscriptionId);

    /// Get rigid body transformation
    ///
    /// \param[in] path Rigid body USD sdf path.
    /// \param[out] pos Rigid body position.
    /// \param[out] rot Rigid body rotation in quaternion (x,y,z,w).
    /// \return True if rigid body was found.
    bool(CARB_ABI* getRigidBodyTransformation)(const pxr::SdfPath& path, carb::Float3& pos, carb::Float4& rot);

    /// Run backwards compatibility on currently attached USD stage
    /// DEPRECATED, use asset validator instead
    ///
    void(CARB_ABI* runBackwardsCompatibility)(long int stageId);

    /// Check backwards compatibility on currently attached USD stage
    /// DEPRECATED, use asset validator instead
    ///
    /// \return True if backwards compatibility needs to run.
    bool(CARB_ABI* checkBackwardsCompatibility)(long int stageId);

    /// Get backwards compatibility log after it did checked the USD stage.
    /// DEPRECATED, use asset validator instead
    ///
    /// \return List of all required changes.
    const char*(CARB_ABI* getBackwardsCompatibilityCheckLog)();

    /// Get collisionGroup path for a given Collider path
    ///
    /// \param[in] path Collider path.
    /// \return Valid SdfPath if CollisionGroup was found
    pxr::SdfPath(CARB_ABI* getCollisionGroupFromCollider)(const pxr::SdfPath& path);

    /// Check if simulation loop is running
    ///
    /// Note that this function returns true if play was pressed or if
    /// IPhysxSimulation was attached.
    /// \return True if simulation running/attached.
    bool(CARB_ABI* isRunning)();

    /// Reset all physics settings to default
    void(CARB_ABI* resetSettings)();

    /// Check if GPU readback is suppressed for currently running simulation.
    ///
    /// Always returns false when the simulation is not running.
    /// \return True if simulation is running with suppressed readback.
    bool(CARB_ABI* isReadbackSuppressed)();

    /// Set the internal dynamics state of a vehicle back to the rest state. Should only be called after the simulation
    /// has been started.
    ///
    /// \param[in] vehicleId      Vehicle object ID (see getObjectId() to get the ID from a USD path)
    void(CARB_ABI* setVehicleToRestState)(const usdparser::ObjectId vehicleId);

    /// Get the transformations of wheels. Should only be called after the simulation has been
    /// started.
    ///
    /// \param[in] vehicleId            Vehicle object ID (see getObjectId() to get the ID from a USD path)
    /// \param[in] wheelIndices         The indices of the wheels to get transforms from (see getWheelIndex() to get the
    ///                                 index from a USD path)
    /// \param[in] wheelIndexCount      The number of entries in the wheelIndices array
    /// \param[in] addVehicleTransform  By default, the provided transforms are in scaled local space (either vehicle
    ///                                 center of mass frame if the custom metadata physxVehicle:referenceFrameIsCenterOfMass
    ///                                 is true or else the vehicle prim frame). The total scale has been applied to the
    ///                                 position already. Set to true to get world space transforms instead.
    /// \param[out] positions           The positions of the wheels. Needs space for at least wheelIndexCount entries.
    /// \param[out] orientations        The orientations of the wheels as a quaternion (x,y,z,w). Needs space for at
    ///                                 least wheelIndexCount entries.
    /// \return True on success, else false.
    bool(CARB_ABI* getWheelTransformations)(const usdparser::ObjectId vehicleId,
                                            const int* wheelIndices,
                                            const unsigned int wheelIndexCount,
                                            const bool addVehicleTransform,
                                            carb::Float3* positions,
                                            carb::Float4* orientations);

    /// Save current scene state to repX, if scene is empty parse scene save it and release it.
    ///
    /// \param[in] filepath            RepX file path
    /// \return True on success, else false.
    bool(CARB_ABI* saveSceneToRepX)(const char* filepath);

    /// Subscribe to physics object change notifications.
    ///
    /// \param[in] callback The callback structure with the notification functions.
    /// \return Subscription Id to unsubscribe
    SubscriptionId(CARB_ABI* subscribeObjectChangeNotifications)(const IPhysicsObjectChangeCallback& callback);

    /// Unsubscribe to physics object change notifications.
    ///
    /// \param[in] subscriptionId Subscription ID obtained via @ref subscribeObjectChangeNotifications.
    void(CARB_ABI* unsubscribeObjectChangeNotifications)(SubscriptionId subscriptionId);

    /// Returns true if asynchronous simulation and rendering is enabled for one of the scenes in the simulation.
    ///
    bool(CARB_ABI* isAsyncSimRenderEnabled)();

    // Simulation event stream providing simulation SimulationEvent
    carb::events::IEventStreamPtr(CARB_ABI* getSimulationEventStreamV2)();

    /// Reset physics settings in the Preferences window to default
    void(CARB_ABI* resetSettingsInPreferences)();

    /// Reset per-stage physics settings to default
    void(CARB_ABI* resetSettingsInStage)();

    /// Reset a physics setting to default
    void(CARB_ABI* resetSetting)(const char* path);

    /// Get the linear velocity of a vehicle (vehicle prim needs to have vehicle API applied).
    /// Should only be called after the simulation has been started.
    ///
    /// \param[in] vehicleId            Vehicle object ID (see getObjectId() to get the ID from a USD path)
    /// \param[in] direction            Unit length direction vector along which the linear velocity should get
    ///                                 computed. The vector is considered to be relative to the center of
    ///                                 mass frame of the vehicle. If a nullptr is passed in, then the local
    ///                                 forward direction of the vehicle will be used.
    /// \return The velocity along the provided direction vector.
    float(CARB_ABI* computeVehicleVelocity)(const usdparser::ObjectId vehicleId, const carb::Float3* direction);

    /// Check if a raycast hits an interactive actor (i.e. rigid/deformable body, articulation link)
    ///
    /// \param[in] origin            World-space origin of raycast
    /// \param[in] direction         Unit-length direction vector of raycast
    /// \return True if an interactive actor is in the raycast defined by origin and direction
    ///         False if no interactive actor is in the raycast or simulation is not running
    bool(CARB_ABI* isInteractiveActorRaycast)(const carb::Float3* origin, const carb::Float3* direction);

    /// Updates actor interaction based on user input and raycast origin and direction
    /// Call repeatedly from a stage update callback or similar
    ///
    /// \param[in] origin                   World-space origin of interaction
    /// \param[in] direction                Unit-length direction vector of interaction
    /// \param[in] interactionEvent         User input event to update interaction accordingly
    void(CARB_ABI* updateInteraction)(const carb::Float3* origin,
                                      const carb::Float3* direction,
                                      PhysicsInteractionEvent interactionEvent);

    /// Update and step a specific scene in the physics simulation. The specific scene specified in scenePath
    /// is updated and stepped *even if marked as 'Disabled'*.
    /// If scenePath is empty, it behaves like IPhysx::updateSimulation
    ///
    /// \param[in] scenePath              Scene USD path encoded as uint64_t
    /// \param[in] elapsedStep            Simulation time.
    /// \param[in] currentTime            Current time, might be used for time sampled transformations to apply.
    void(CARB_ABI* updateSimulationScene)(uint64_t scenePath, float elapsedStep, float currentTime);

    /// Update the transformations for a specific scene in the physics simulation. The specific scene specified in
    /// scenePath has its transformations updated *even if it is marked as 'Disabled'*. If scenePath is empty, it
    /// behaves like IPhysx::updateTransformations
    ///
    /// \param[in] scenePath                    Scene USD path encoded as uint64_t
    /// \param[in] updateToUsd                  Update transforms to USD.
    /// \param[in] updateVelocitiesToUsd        Update velocities to USD.
    void(CARB_ABI* updateTransformationsScene)(uint64_t scenePath, bool updateToUsd, bool updateVelocitiesToUsd);


    /// Get vehicle drive state (vehicle prim needs to have vehicle controller API applied).
    ///
    /// \note Should only be called after the simulation has been started.
    ///
    /// \param[in] vehicleControllerId  Vehicle controller object ID (see getObjectId() to get the ID from a USD path)
    /// \param[out] driveState          Structure to write drive state to.
    /// \return True on success, else false.
    bool(CARB_ABI* getVehicleDriveState)(const usdparser::ObjectId vehicleControllerId, VehicleDriveState& driveState);

    /// Get the state for the specified wheels. Should only be called after the simulation has been
    /// started.
    ///
    /// \param[in] wheelAttachmentIds      List of wheel attachment object IDs referencing the wheels to get the state
    ///                                    for (see getObjectId() to get the ID from a USD path)
    /// \param[in] wheelAttachmentIdCount  Number of wheel attachment object IDs provided
    /// \param[out] wheelStates            State structures to be filled for each specified wheel. The array needs to
    ///                                    have size wheelAttachmentIdCount.
    /// \return True on success, else false.
    bool(CARB_ABI* getWheelState)(const usdparser::ObjectId* wheelAttachmentIds,
                                  const uint32_t wheelAttachmentIdCount,
                                  VehicleWheelState* states);

    /// Set the rotation speed about the rolling axis of a wheel. Should only be called after the simulation has been
    /// started.
    ///
    /// \param[in] wheelAttachmentIds      List of wheel attachment object IDs to reference the wheels to set the speed
    ///                                    for (see getObjectId() to get the ID from a USD path)
    /// \param[in] wheelAttachmentIdCount  Number of wheel attachment object IDs provided
    /// \param[in] rotationSpeeds          Rotation speed of the wheel in radians per second. The array needs to have
    ///                                    size wheelAttachmentIdCount.
    void(CARB_ABI* setWheelRotationSpeed)(const usdparser::ObjectId* wheelAttachmentIds,
                                          const uint32_t wheelAttachmentIdCount,
                                          const float* rotationSpeeds);

    /// Set the rotation angle about the rolling axis of a wheel. Should only be called after the simulation has been
    /// started.
    ///
    /// \param[in] wheelAttachmentIds      List of wheel attachment object IDs referencing the wheels to set the angle
    ///                                    for (see getObjectId() to get the ID from a USD path)
    /// \param[in] wheelAttachmentIdCount  Number of wheel attachment object IDs provided
    /// \param[in] rotationAngles          Rotation angle of the wheel in radians. The array needs to have size
    ///                                    wheelAttachmentIdCount.
    void(CARB_ABI* setWheelRotationAngle)(const usdparser::ObjectId* wheelAttachmentIds,
                                          const uint32_t wheelAttachmentIdCount,
                                          const float* rotationAngles);

    /// Overrides the behavior of the ResetOnStop setting.
    ///
    /// \param[in] disable      Disable/enable the reset on stop override.
    void(CARB_ABI* disableResetOnStop)(bool disable);

    /// Subscribe to physics pre/post step events.
    ///
    /// \note Subscriptions cannot be changed in the onUpdate callback
    ///
    /// \param onUpdate The callback function to be called on update.
    /// \param userData The userData to be passed back in the callback function.
    /// \param preStep  Whether to execute this callback right *before* the physics step event. If this is false, the
    ///                 callback will be executed right *after* the physics step event.
    /// \param order    An integer value used to order the callbacks: 0 means "highest priority", 1 is "less priority"
    /// and so on. \return Subscription Id for release, returns kInvalidSubscriptionId if failed
    SubscriptionId(CARB_ABI* subscribePhysicsOnStepEvents)(bool preStep,
                                                           int order,
                                                           OnPhysicsStepEventFn onUpdate,
                                                           void* userData);

    /// Unsubscribes to pre/post update events.
    ///
    /// \note Subscription cannot be changed in the onUpdate callback
    ///
    /// subscriptionId SubscriptionId obtained via @ref subscribePhysicsOnStepEvents.
    void(CARB_ABI* unsubscribePhysicsOnStepEvents)(SubscriptionId subscriptionId);
};


} // namespace physx
} // namespace omni

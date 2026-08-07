// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include "ContactEvent.h"
#include "TriggerEvent.h"

namespace omni
{

namespace physx
{

/// Simulation output type
struct SimulationOutputType
{
    enum Enum
    {
        /**
        \brief Transformation updates

        \note Currently supported for RigidBodies and Vehicles.
        */
        eTRANSFORMATION = 0,

        /**
        \brief Velocity updates

        \note Currently supported for RigidBodies.
        */
        eVELOCITY,

        /**
        \brief Points attribute for UsdGeomPoints-derived types

        \note Currently not used
        */
        ePOINTS
    };
};

/// Simulation output flags that can be setup per SdfPath
struct SimulationOutputFlag
{
    enum Enum
    {
        /**
        \brief Disables output for given type on given SdfPath

        <b>Default:</b> False
        */
        eSKIP_WRITE = 1 << 0,

        /**
        \brief Enables output notification for given type on given SdfPath

        <b>Default:</b> False
        */
        eNOTIFY_UPDATE = 1 << 1,

        /**
        \brief Enables output notification in radians rather then degree for given type on given SdfPath

        \note Currently used for velocity update.

        <b>Default:</b> False
        */
        eNOTIFY_IN_RADIANS = 1 << 2,
    };
};

/// Simulation flags that can be setup per SdfPath
/// DEPRECATED, please use the SimulationOutputType and SimulationOutputFlag
struct SimulationFlag
{
    enum Enum
    {
        /**
        \brief Disables transformation update on given SdfPath

        \note Currently supported for RigidBodies and Vehicles. The provided SdfPath is
        expected on a prim with RigidBodyAPI or VehicleAPI

        <b>Default:</b> False
        */
        eSKIP_TRANSFORM_WRITE = 1 << 0,

        /**
        \brief Enables transformation notification through a simulation callback on given SdfPath

        \note Currently supported for RigidBodies and Vehicles. The provided SdfPath is
        expected on a prim with RigidBodyAPI or VehicleAPI.
        \note If set in a VehicleAPI the transform notify is send for each WheelAttachmentAPI prim

        <b>Default:</b> False
        */
        eNOTIFY_TRANSFORM_UPDATE = 1 << 1,
    };
};

/// Force mode
struct ForceModeType
{
    enum Enum
    {
        eFORCE, //!< parameter has unit of mass * distance/ time^2, i.e. a force
        eIMPULSE, //!< parameter has unit of mass * distance /time (not used for angular)
        eVELOCITY_CHANGE, //!< parameter has unit of distance / time, i.e. the effect is mass independent: a velocity
                          //!< change.
        eACCELERATION //!< parameter has unit of distance/ time^2, i.e. an acceleration. It gets treated just like a
                      //!< force except the mass is not divided out before integration.
    };
};

/// Transformation update notification function issued for transformation change by omni.physx
///
/// \note SimulationOutputFlag::eNOTIFY_UPDATE should be set for SimulationOutputType::eTRANSFORMATION
///
/// \param[in] sdfPath SdfPath stored as Uint64_t for the prim that changed transformation
/// \param[in] pos New position
/// \param[in] rot New rotation
/// \param[in] userData User data passed to ISimulationCallback struct
using TransformUpdateNotificationFn =
    std::function<void(uint64_t sdfPath, const carb::Float3& pos, const carb::Float4& rot, void* userData)>;

/// Velocity update notification function issued for active rigid bodies.
///
/// \note SimulationOutputFlag::eNOTIFY_UPDATE should be set for SimulationOutputType::eVELOCITY
///
/// \param[in] sdfPath SdfPath stored as Uint64_t for the prim that changed transformation
/// \param[in] linVelocity New linear velocity
/// \param[in] angVelocity New angular velocity (default in degree see: SimulationOutputFlag::eNOTIFY_IN_RADIANS)
/// \param[in] userData User data passed to ISimulationCallback struct
using VelocityUpdateNotificationFn =
    std::function<void(uint64_t sdfPath, const carb::Float3& linVelocity, const carb::Float3& angVelocity, void* userData)>;

/// Physics transformation update callback. Called after fetch results, when transformations are available for read.
///
/// \param[in] timeStep Time step used for simulation
/// \param[in] currentTime Current global time
/// \param[in] userData User data passed to ISimulationCallback struct
using TransformUpdateFn = std::function<void(float timeStep, float currentTime, void* userData)>;

/// Simulation callback structure holding function pointers for callbacks
struct ISimulationCallback
{
    TransformUpdateNotificationFn transformationWriteFn = { nullptr };
    VelocityUpdateNotificationFn velocityWriteFn = { nullptr };
    TransformUpdateFn transformationUpdateFn = { nullptr };

    void* userData = { nullptr };
};


// NOTE: the ovstage-shaped physics output read (ADR-0007) lives in the
// standalone, non-carb header omni/physx/IOvxPhysicsRead.h — it returns
// ovstage-native types (ovx_primpath_list_t / DLTensor) and so does not belong
// on this carb interface. It is available only under the ovstage backend.

struct IPhysxSimulation
{
    /// Attach USD stage. This will run the physics parser
    /// and will populate the PhysX SDK with the corresponding simulation objects.
    ///
    /// Note: previous stage will be detached.
    ///
    /// \param[in] id USD stageId (can be retrieved from a stagePtr -
    /// PXR_NS::UsdUtilsStageCache::Get().GetId(stagePtr).ToLongInt()) \return True if stage was successfully attached.
    bool(CARB_ABI* attachStage)(long id);

    /// Detach USD stage, this will remove all objects from the PhysX SDK
    ///
    void(CARB_ABI* detachStage)();

    /// Sets simulation callbacks for omni.physx
    ///
    /// /note Simulation callbacks are reset each attach/detachStage call, it is expected
    /// to set a simulation callbacks after attachStage is called, any call before will get ignored
    ///
    /// \param[in] callback ISimulationCallback structure with callback functions
    void(CARB_ABI* setSimulationCallback)(const ISimulationCallback& callback);


    /// Sets simulation output flags
    ///
    /// /note Simulation flags are reset each attach/detachStage call, it is expected
    /// to set a simulation flag after attachStage is called, any call before will get ignored
    ///
    /// /note If a flag is set globally it overrides the local SdfPath flags set
    ///
    /// \param[in] outputType Output type for the simulation flags
    /// \param[in] flags Flags to set
    /// \param[in] paths SdfPaths as uint64_t to set flags. Providing nullptr will enable the flags globally for all
    /// paths. \param[in] numPath Number of paths provided
    void(CARB_ABI* setSimulationOutputFlags)(uint32_t outputType, uint32_t flags, const uint64_t* paths, uint32_t numPaths);


    /// Add simulation output flags
    ///
    /// /note Simulation flags are reset each attach/detachStage call, it is expected
    /// to set a simulation flag after attachStage is called, any call before will get ignored
    ///
    /// \param[in] outputType Output type for the simulation flags
    /// \param[in] flags Flags to add
    /// \param[in] paths SdfPaths as uint64_t to set flags. Providing nullptr will enable the flags globally for all
    /// paths. \param[in] numPath Number of paths provided
    void(CARB_ABI* addSimulationOutputFlags)(uint32_t outputType, uint32_t flags, const uint64_t* paths, uint32_t numPaths);


    /// Remove simulation output flags
    ///
    /// /note Simulation flags are reset each attach/detachStage call, it is expected
    /// to set a simulation flag after attachStage is called, any call before will get ignored
    ///
    /// \param[in] outputType Output type for the simulation flags
    /// \param[in] flags Flags to remove
    /// \param[in] paths SdfPaths as uint64_t to set flags. Providing nullptr will disable (or clear) the flags globally
    /// for all paths. \param[in] numPath Number of paths provided
    void(CARB_ABI* removeSimulationOutputFlags)(uint32_t outputType,
                                                uint32_t flags,
                                                const uint64_t* paths,
                                                uint32_t numPaths);

    /// Execute physics simulation
    ///
    /// The PhysX simulation will simulate the exact elapsedTime passed. No substepping will happen.
    /// It is the caller's responsibility to provide reasonable elapsedTime.
    /// In general it is recommended to use fixed size time steps with a maximum of 1/60 of a second
    ///
    /// \param[in] elapsedTime Simulation time in seconds.
    /// \param[in] currentTime Current time, might be used for time sampled transformations to apply.
    void(CARB_ABI* simulate)(float elapsedTime, float currentTime);

    /// Fetch simulation results.
    /// Writing out simulation results based on physics settings.
    ///
    /// \note This is a blocking call. The function will wait until the simulation is finished.
    void(CARB_ABI* fetchResults)();

    /// Check if simulation finished.
    ///
    /// return True if simulation finished.
    bool(CARB_ABI* checkResults)();

    /// Flush changes will force physics to process buffered changes
    ///
    /// Changes to physics gets buffered, in some cases flushing changes is required if order is required.
    ///
    /// Example - prim A gets added. Existing prim B has a relationship that gets switched to use A. Currently,
    /// the relationship change gets processed immediately and fails because prim A only gets added at the
    /// start of the next sim step.
    ///
    void(CARB_ABI* flushChanges)();

    /// Subscribe to physics simulation contact report events.
    ///
    /// \note The contact buffer data are available for one simulation step.
    ///
    /// \param onEvent The callback function to be called on contact report.
    /// \param userData The userData to be passed back in the callback function.
    /// \return Subscription Id for release
    SubscriptionId(CARB_ABI* subscribePhysicsContactReportEvents)(OnContactReportEventFn onEvent, void* userData);

    /// Unsubscribes to contact report events.
    ///
    /// subscriptionId SubscriptionId obtained via @ref subscribePhysicsContactReportEvents.
    void(CARB_ABI* unsubscribePhysicsContactReportEvents)(SubscriptionId subscriptionId);

    /// Get contact report data for current simulation step directly.
    ///
    /// \note The contact buffer data are available for one simulation step.
    ///
    /// \param contactEventBuffer Contact event header buffer, contains headers for contacts.
    /// \param contactDataBuffer Contact data, contains contact data for individual headers.
    /// \param numContactData Number of contact data.
    /// \return Number of contact headers in the contact event buffer
    uint32_t(CARB_ABI* getContactReport)(const ContactEventHeader** contactEventBuffer,
                                         const ContactData** contactDataBuffer,
                                         uint32_t& numContactData);

    /// Get physics simulation timestamp.
    ///
    /// Timestamp will increase with every simulation step.
    ///
    /// \return Current timestamp
    uint64_t(CARB_ABI* getSimulationTimestamp)();

    /// Get the number of physics steps performed in the active simulation.
    ///
    /// The step count resets to 0 when a new simulation starts.
    ///
    /// \return Number of steps since the currently active simulation started or 0 if there is no active simulation.
    uint64_t(CARB_ABI* getSimulationStepCount)();

    /// Execute the physics simulation on a specific scene.
    ///
    /// The PhysX simulation in the scene will simulate the exact elapsedTime passed. No substepping will happen.
    /// It is the caller's responsibility to provide a reasonable elapsedTime.
    /// In general it is recommended to use fixed size time steps with a maximum of 1/60 of a second.
    /// If scenePath is empty, it behaves like IPhysxSimulation::simulate
    ///
    /// \param[in] scenePath   Scene USD path encoded as uint64_t
    /// \param[in] elapsedTime Simulation time in seconds.
    /// \param[in] currentTime Current time, might be used for time sampled transformations to apply.
    void(CARB_ABI* simulateScene)(uint64_t scenePath, float elapsedTime, float currentTime);

    /// Fetch simulation scene results and writes out simulation results based on physics settings for
    /// a specific scene. Disabling a scene has no effect on this function.
    /// If scenePath is empty, it behaves like IPhysxSimulation::fetchResults
    ///
    /// \note This is a blocking call. The function will wait until the simulation scene is finished.
    ///
    /// \param[in] scenePath   Scene USD path encoded as uint64_t
    void(CARB_ABI* fetchResultsScene)(uint64_t scenePath);

    /// Check if a simulation scene is finished. Disabling a scene has no effect on this function.
    /// If scenePath is empty, it behaves like IPhysxSimulation::checkResults
    ///
    /// return True if the simulation scene is finished.
    ///
    /// \param[in] scenePath   Scene USD path encoded as uint64_t
    bool(CARB_ABI* checkResultsScene)(uint64_t scenePath);

    /// Applies a force (or impulse) defined in the global coordinate frame, acting at a particular
    /// point in global coordinates, to the actor.
    /// \param[in] stageId    USD stageId
    /// \param[in] path		  Body USD path encoded to uint64_t
    /// \param[in] force      Force / impulse to add, defined in the global frame.
    /// \param[in] pos        Position in the global frame to add the force at.
    /// \param[in] mode       The mode to use when applying the force/impulse
    void(CARB_ABI* addForceAtPos)(
        uint64_t stageId, uint64_t path, const carb::Float3& force, const carb::Float3& pos, ForceModeType::Enum mode);

    /// Applies a torque (or impulse) at the center of mass
    /// \param[in] stageId    USD stageId
    /// \param[in] path		  Body USD path encoded to uint64_t
    /// \param[in] force      Torque to add to the body center of mass
    void(CARB_ABI* addTorque)(uint64_t stageId, uint64_t path, const carb::Float3& torque);

    /// Wakes up body on given path
    /// \param[in] stageId    USD stageId
    /// \param[in] path		  Body USD path encoded to uint64_t
    void(CARB_ABI* wakeUp)(uint64_t stageId, uint64_t path);

    /// Puts to sleep body on given path
    /// \param[in] stageId    USD stageId
    /// \param[in] path		  Body USD path encoded to uint64_t
    void(CARB_ABI* putToSleep)(uint64_t stageId, uint64_t path);

    /// Checks whether a body sleeps
    /// \param[in] stageId    USD stageId
    /// \param[in] path		  Body USD path encoded to uint64_t
    /// \return True if body is asleep
    bool(CARB_ABI* isSleeping)(uint64_t stageId, uint64_t path);

    /// Subscribe to physics simulation trigger report events.
    ///
    /// \param stageId The stage containing the prim with trigger API. If it's set to 0 it will report triggers from all
    /// stages. \param path The prim containing the trigger API. IF it's set to 0 it will report triggers from all prims
    /// \param onEvent The callback function to be called on trigger report.
    /// \param userData The userData to be passed back in the callback function.
    /// \return Subscription Id to stop receiving notifications (to be used with @ref
    /// IPhysxSimulation::unsubscribePhysicsTriggerReportEvents)
    SubscriptionId(CARB_ABI* subscribePhysicsTriggerReportEvents)(uint64_t stageId,
                                                                  uint64_t path,
                                                                  OnTriggerEventReportEventFn onEvent,
                                                                  void* userData);

    /// Unsubscribes to trigger report events.
    ///
    /// subscriptionId SubscriptionId obtained via @ref IPhysxSimulation::subscribePhysicsTriggerReportEvents.
    void(CARB_ABI* unsubscribePhysicsTriggerReportEvents)(SubscriptionId subscriptionId);

    /// Applies a force (or impulse) defined in the global coordinate frame, acting at a particular
    /// point in global coordinates, to the point instancer body.
    /// \param[in] stageId          USD stageId
    /// \param[in] pointInstancerPath   Point instancer USD path encoded to uint64_t
    /// \param[in] force                Force / impulse to add, defined in the global frame.
    /// \param[in] pos                  Position in the global frame to add the force at.
    /// \param[in] mode                 The mode to use when applying the force/impulse
    /// \param[in] protoIndex           If protoIndex is 0xffffffff, force will be applied to all instances,
    ///                                 otherwise it will only be applied to the instance at this index.
    void(CARB_ABI* addForceAtPosInstanced)(uint64_t stageId,
                                           uint64_t pointInstancerPath,
                                           const carb::Float3& force,
                                           const carb::Float3& pos,
                                           ForceModeType::Enum mode,
                                           uint32_t protoIndex);

    /// Applies a torque (or impulse) to the point instancer at the center of mass
    /// \param[in] stageId              USD stageId
    /// \param[in] pointInstancerPath   Point instancer USD path encoded to uint64_t
    /// \param[in] force                Torque to add to the body center of mass
    /// \param[in] protoIndex           If protoIndex is 0xffffffff, torque will be applied to all instances,
    ///                                 otherwise it will only be applied to the instance at this index.
    void(CARB_ABI* addTorqueInstanced)(uint64_t stageId,
                                       uint64_t pointInstancerPath,
                                       const carb::Float3& torque,
                                       uint32_t protoIndex);

    /// Wakes up point instancer body on given path
    /// \param[in] stageId              USD stageId
    /// \param[in] pointInstancerPath   Point instancer USD path encoded to uint64_t
    /// \param[in] protoIndex           If protoIndex is 0xffffffff, all instances will be awakened
    ///                                 otherwise it will only be applied to the instance at this index.
    void(CARB_ABI* wakeUpInstanced)(uint64_t stageId, uint64_t pointInstancerPath, uint32_t protoIndex);

    /// Puts to sleep point instancer body on given path
    /// \param[in] stageId              USD stageId
    /// \param[in] pointInstancerPath   Point instancer USD path encoded to uint64_t
    /// \param[in] protoIndex           If is 0xffffffff, all instances will be put to sleep
    ///                                 otherwise it will only be applied to the instance at this index.
    void(CARB_ABI* putToSleepInstanced)(uint64_t stageId, uint64_t pointInstancerPath, uint32_t protoIndex);

    /// Checks whether a point instancer body sleeps
    /// \param[in] stageId              USD stageId
    /// \param[in] pointInstancerPath   Point instancer USD path encoded to uint64_t
    /// \param[in] protoIndex           Checks the instance at this index.
    /// \return True if body is asleep
    bool(CARB_ABI* isSleepingInstanced)(uint64_t stageId, uint64_t pointInstancerPath, uint32_t protoIndex);

    /// Gets the currently attached USD stage.
    ///
    /// \return USD stageId
    long(CARB_ABI* getAttachedStage)();

    /// Subscribe to physics simulation contact report events including friction anchors.
    ///
    /// \note The contact buffer data are available for one simulation step.
    ///
    /// \param onEvent The callback function to be called on contact report.
    /// \param userData The userData to be passed back in the callback function.
    /// \return Subscription Id for release
    SubscriptionId(CARB_ABI* subscribePhysicsFullContactReportEvents)(OnFullContactReportEventFn onEvent, void* userData);

    /// Unsubscribes to contact report events.
    ///
    /// subscriptionId SubscriptionId obtained via @ref subscribePhysicsContactReportEvents.
    void(CARB_ABI* unsubscribePhysicsFullContactReportEvents)(SubscriptionId subscriptionId);

    /// Get contact report data for current simulation step directly including friction anchors.
    ///
    /// \note The contact buffer data are available for one simulation step.
    ///
    /// \param contactEventBuffer Contact event header buffer, contains headers for contacts.
    /// \param contactDataBuffer Contact data, contains contact data for individual headers.
    /// \param numContactData Number of contact data.
    /// \param frictionAnchorDataBuffer Friction anchor data, contains friction anchor data for individual headers.
    /// \param numFrictionAnchorData Number of friction anchor data.
    /// \return Number of contact headers in the contact event buffer
    uint32_t(CARB_ABI* getFullContactReport)(const ContactEventHeader** contactEventBuffer,
                                             const ContactData** contactDataBuffer,
                                             uint32_t& numContactData,
                                             const FrictionAnchor** frictionAnchorDataBuffer,
                                             uint32_t& numFrictionAnchorData);

    /// Attach a consumer-provided ovstage source instead of a USD stage (ADR-0002).
    /// `ovstageAttachPayload` is a backend-opaque pointer (a const
    /// `omni::physics::ovstage::OvstageAttach*` = ovstage instance + path
    /// dictionary), which the caller owns and must keep alive until detach after
    /// a successful attachment.
    /// `readOrdinal` is the sealed ovstage ordinal the initial scene parse reads
    /// at; the caller owns the ordinal and passes it explicitly (it is no longer
    /// carried inside the payload).
    /// The runtime installs and restores the ovstage parse and scan backends as
    /// part of the attachment; callers do not register those process-global
    /// backends themselves. A false return leaves no active attachment and
    /// retains no reference to `ovstageAttachPayload`.
    /// Appended at the end of the struct to preserve ABI of the existing slots.
    /// \return True if the ovstage source was successfully attached.
    bool(CARB_ABI* attachOvstage)(const void* ovstageAttachPayload, uint64_t readOrdinal);

    /// Pull and apply ovstage change deltas for an explicit, producer-supplied
    /// ordinal range [fromOrdinal, toOrdinal] (ADR-0003 M3). The caller is the
    /// agent that advanced the ovstage ordinals, so it owns the range; this is
    /// the ovstage analogue of the USD change-notice drain. A range read returns
    /// only the attributes that changed in the interval, which are applied to the
    /// running simulation. No-op unless an ovstage source is attached.
    /// Appended at the end of the struct to preserve ABI of the existing slots.
    /// \return True if the range was drained (false if no ovstage feed or the
    ///         range could not be served → caller should re-attach).
    bool(CARB_ABI* updateFromOvStage)(uint64_t fromOrdinal, uint64_t toOrdinal);

    /// Clone the environment subtree at \p sourcePath into \p numTargets copies via the PhysX SDK
    /// replicator (binary serialization) -- the path that forms real articulations. Source-agnostic:
    /// \p sourcePath and \p targetPaths are path strings, so no USD type crosses the ABI (the runtime
    /// resolves them against the attached source). Each copy is created at the caller-supplied
    /// \p targetPaths[i] (so callers control naming, e.g. `/World/envs/env*` for the tensor-binding
    /// pattern) -- there is no USD authoring; the clones exist as runtime physics only.
    ///
    /// Placement: \p transforms is a per-clone pose array, `[numTargets * 7]` floats (px,py,pz,
    /// qx,qy,qz,qw -- position + imaginary-first quaternion). `transforms[i]` is the world pose of
    /// copy i's parent: each cloned body keeps its pose relative to the source's parent (copy =
    /// `transforms[i] * inverse(sourceParent) * sourceBody`), a rigid move of the whole env that
    /// keeps the intra-env layout -- for an at-origin source each body lands exactly at
    /// `transforms[i]`. Pass null to co-locate every copy on the source pose.
    ///
    /// env-ids: \p useEnvIds is an optional pass-through to the replicator's env-id
    /// cross-environment collision filtering (engages only under GPU dynamics + GPU broadphase).
    /// \p envIds optionally supplies the LOGICAL environment id per target (`[numTargets]`): copy i
    /// receives runtime env id `envIds[i] + 1` (0 stays the source's), so the same caller id yields
    /// the same runtime id in every call -- required when one logical environment is assembled from
    /// several clone calls (e.g. a ClonePlan cloning one source row at a time), where positional
    /// numbering would put same-environment objects on different ids and silently stop their
    /// contacts. Pass null for positional numbering (each batch gets fresh ids past the previous
    /// batches'). Each id must be `< 0x00FFFFFF` (PhysX supports at most 1<<24 environments and
    /// the runtime id is `envIds[i] + 1`); out-of-range ids fail the call rather than silently
    /// losing isolation (0xFFFFFFFE would land on the collide-with-all id, 0xFFFFFFFF would wrap
    /// to the source's id 0).
    ///
    /// Targets must be disjoint, unpopulated paths: a target already carrying parsed physics, or
    /// one that equals / is an ancestor or descendant of an earlier successful clone target on
    /// this attach (runtime clones author no USD prim, so they are tracked per attach), fails the
    /// call -- cloning there would stack duplicate live actors on one path.
    ///
    /// Appended at the end of the struct to preserve ABI of the existing slots. No-op
    /// unless a stage/ovstage source is attached. \return True on success.
    bool(CARB_ABI* cloneEnvironments)(const char* sourcePath, const char* const* targetPaths,
                                      uint32_t numTargets, const float* transforms,
                                      const uint32_t* envIds, bool useEnvIds);
};


} // namespace physx
} // namespace omni

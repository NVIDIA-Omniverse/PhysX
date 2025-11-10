// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
        ePOINTS,

        /**
        \brief Solver residuals

        \note Solver residuals reported on a physics scene, articulation and joints, reported when class
        "PhysxResidualReportingAPI" is present.
        */
        eRESIDUALS
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

/// Residual reporting notification function issued for all PhysxResidualReportingAPIs.
///
/// \note SimulationOutputFlag::eNOTIFY_UPDATE should be set for SimulationOutputType::eRESIDUALS
///
/// \param[in] sdfPath SdfPath stored as Uint64_t for the prim that changed transformation
/// \param[in] positionIterationResidualMax Maximum position interation residual
/// \param[in] positionIterationResidualRms Rms position interation residual
/// \param[in] velocityIterationResidualMax Maximum velocity interation residual
/// \param[in] velocityIterationResidualRms Rms velocity interation residual
/// \param[in] userData User data passed to ISimulationCallback struct
using ResidualUpdateNotificationFn = std::function<void(uint64_t sdfPath,
                                                        float positionIterationResidualMax,
                                                        float positionIterationResidualRms,
                                                        float velocityIterationResidualMax,
                                                        float velocityIterationResidualRms,
                                                        void* userData)>;

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
    ResidualUpdateNotificationFn residualWriteFn = { nullptr };
    TransformUpdateFn transformationUpdateFn = { nullptr };

    void* userData = { nullptr };
};


struct IPhysxSimulation
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxSimulation", 3, 0)

    /// Attach USD stage. This will run the physics parser
    /// and will populate the PhysX SDK with the corresponding simulation objects.
    ///
    /// Note: previous stage will be detached.
    ///
    /// \param[in] id USD stageId (can be retrieved from a stagePtr -
    /// pxr::UsdUtilsStageCache::Get().GetId(stagePtr).ToLongInt()) \return True if stage was successfully attached.
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

    /// Pause fabric change tracking for physics listener
    ///
    /// \param[in] pause Pause or resume the change tracking
    void(CARB_ABI* pauseChangeTracking)(bool pause);

    /// Check if fabric change tracking for physics listener is paused or not
    ///
    /// return True if change tracking is paused
    bool(CARB_ABI* isChangeTrackingPaused)();

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
};


// Helper class for scoped change tracking usage
class PauseChangeTrackingScope
{
public:
    PauseChangeTrackingScope(IPhysxSimulation* physxSim) : mPhysXSim(physxSim), mChangeTrackingPaused(false)
    {
        if (mPhysXSim)
        {
            mChangeTrackingPaused = mPhysXSim->isChangeTrackingPaused();
            mPhysXSim->pauseChangeTracking(true);
        }
    }

    ~PauseChangeTrackingScope()
    {
        if (mPhysXSim)
        {
            mPhysXSim->pauseChangeTracking(mChangeTrackingPaused);
        }
    }

private:
    IPhysxSimulation* mPhysXSim;
    bool mChangeTrackingPaused;
};

} // namespace physx
} // namespace omni

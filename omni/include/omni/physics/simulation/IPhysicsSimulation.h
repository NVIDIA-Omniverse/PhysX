// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include "simulator/ContactEvent.h"
#include "simulator/Simulation.h"
#include "simulator/Simulator.h"

namespace omni
{

namespace physics
{


struct IPhysicsSimulation
{
    CARB_PLUGIN_INTERFACE("omni::physics::IPhysicsSimulation", 0, 1)

    /// Attach USD stage. This will run the physics parser
    /// and will populate the simulation with the corresponding simulation objects.
    ///
    /// Note: previous stage will be detached.
    ///
    /// \param[in] id USD stageId (can be retrieved from a stagePtr -
    /// pxr::UsdUtilsStageCache::Get().GetId(stagePtr).ToLongInt())
    ///\return True if stage was successfully attached.
    bool(CARB_ABI* attachStage)(long id);

    /// Detach USD stage, this will remove all objects from the simulation
    ///
    void(CARB_ABI* detachStage)();

    /// Gets the currently attached USD stage.
    ///
    /// \return USD stageId, 0 means no stage is attached.
    long(CARB_ABI* getAttachedStage)();

    /// Execute physics simulation
    ///
    /// The simulation will simulate the exact elapsedTime passed. No substepping will happen.
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

    /// Pause change tracking for physics listener
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
    /// \return Subscription Id for release
    SubscriptionId(CARB_ABI* subscribePhysicsContactReportEvents)(OnContactReportEventFn onEvent);

    /// Unsubscribes to contact report events.
    ///
    /// subscriptionId SubscriptionId obtained via @ref subscribePhysicsContactReportEvents.
    void(CARB_ABI* unsubscribePhysicsContactReportEvents)(SubscriptionId subscriptionId);

    /// Get physics simulation time steps per second.
    ///
    /// \param stageId stage id
    /// \param scenePath returns the time steps for given scene if 0 is passed returns the first found scene stepping
    /// \return Current time steps per second
    uint32_t(CARB_ABI* getSimulationTimeStepsPerSecond)(SimulationId simulationId, long stageId, uint64_t scenePath);

    /// Get physics simulation timestamp.
    ///
    /// Timestamp will increase with every simulation step.
    ///
    /// \return Current timestamp
    uint64_t(CARB_ABI* getSimulationTimestamp)(SimulationId simulationId);

    /// Get the number of physics steps performed in the active simulation.
    ///
    /// The step count resets to 0 when a new simulation starts.
    ///
    /// \return Number of steps since the currently active simulation started or 0 if there is no active simulation.
    uint64_t(CARB_ABI* getSimulationStepCount)(SimulationId simulationId);

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


    /// Subscribe to physics pre/post step events.
    ///
    /// \note Subscriptions cannot be changed in the onUpdate callback
    ///
    /// \param onUpdate The callback function to be called on update.
    /// \param preStep  Whether to execute this callback right *before* the physics step event. If this is false, the
    ///                 callback will be executed right *after* the physics step event.
    /// \param order    An integer value used to order the callbacks: 0 means "highest priority", 1 is "less priority"
    /// and so on. \return Subscription Id for release, returns kInvalidSubscriptionId if failed
    SubscriptionId(CARB_ABI* subscribePhysicsOnStepEvents)(bool preStep, int order, OnPhysicsStepEventFn onUpdate);

    /// Unsubscribes to pre/post update events.
    ///
    /// \note Subscription cannot be changed in the onUpdate callback
    ///
    /// subscriptionId SubscriptionId obtained via @ref subscribePhysicsOnStepEvents.
    void(CARB_ABI* unsubscribePhysicsOnStepEvents)(SubscriptionId subscriptionId);
};

} // namespace physics
} // namespace omni

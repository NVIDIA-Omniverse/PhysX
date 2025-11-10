// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <omni/Function.h>

#include "ContactEvent.h"

namespace omni
{
namespace physics
{
struct SimulationId
{
    SimulationId() : id(0xFFffFFff)
    {
    }
    SimulationId(size_t v) : id(v)
    {
    }

    size_t hash() const
    {
        return id;
    }

    bool operator==(const SimulationId& other) const
    {
        return (id == other.id);
    }

    bool operator!=(const SimulationId& other) const
    {
        return (id != other.id);
    }

    size_t id;
};

struct SimulationIdHash
{
    size_t operator()(const SimulationId& pair) const
    {
        return pair.hash();
    }
};

const SimulationId kInvalidSimulationId = SimulationId(0xFFffFFff);

using SubscriptionId = size_t;
const SubscriptionId kInvalidSubscriptionId = SubscriptionId(0xFFffFFffFF);

// Context for physics step events
struct PhysicsStepContext
{
    uint64_t scenePath; // USD scene path encoded to uint64_t
    SimulationId simulationId; // Simulation ID
};

// Callback function for physics step events
//
// \param elapsedTime Elapsed time since the last physics step
// \param context Context for physics step events
using OnPhysicsStepEventFn = std::function<void(float elapsedTime, const PhysicsStepContext& context)>;

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

// Attach the stage
//\param[in] id USD stageId (can be retrieved from a stagePtr -
// pxr::UsdUtilsStageCache::Get().GetId(stagePtr).ToLongInt())
//\return True if stage was successfully attached.
using AttachStageFn = std::function<bool(long id)>;

// Detach the stage
using DetachStageFn = std::function<void()>;

// Get the currently attached stage
//\return USD stageId, 0 means no stage is attached.
using GetAttachedStageFn = std::function<long()>;

// Execute physics simulation
//
// The simulation will simulate the exact elapsedTime passed. No substepping will happen.
// It is the caller's responsibility to provide reasonable elapsedTime.
// In general it is recommended to use fixed size time steps with a maximum of 1/60 of a second
//
//\param[in] elapsedTime Simulation time in seconds.
//\param[in] currentTime Current time, might be used for time sampled transformations to apply.
using SimulateFn = std::function<void(float elapsedTime, float currentTime)>;

// Fetch simulation results.
// Writing out simulation results.
//
// \note This is a blocking call. The function will wait until the simulation is finished.
using FetchResultsFn = std::function<void()>;

// Check if simulation finished.
//
// return True if simulation finished.
using CheckResultsFn = std::function<bool()>;

// Flush changes will force physics to process buffered changes
//
// Changes to physics gets buffered, in some cases flushing changes is required if order is required.
//
// Example - prim A gets added. Existing prim B has a relationship that gets switched to use A. Currently,
// the relationship change gets processed immediately and fails because prim A only gets added at the
// start of the next sim step.
using FlushChangesFn = std::function<void()>;

// Pause the change tracking
//
// \param[in] pause Pause or resume the change tracking
using PauseChangeTrackingFn = std::function<void(bool pause)>;

// Check if fabric change tracking for physics listener is paused or not
//
// return True if change tracking is paused
using IsChangeTrackingPausedFn = std::function<bool()>;

// Subscribe to physics simulation contact report events.
//
// \note The contact buffer data are available for one simulation step.
//
// \param onEvent The callback function to be called on contact report.
// \return Subscription Id for release
using SubscribePhysicsContactReportEventsFn = std::function<SubscriptionId(OnContactReportEventFn onEvent)>;

// Unsubscribe to physics simulation contact report events.
//
// \param subscriptionId SubscriptionId obtained via @ref subscribePhysicsContactReportEvents.
using UnsubscribePhysicsContactReportEventsFn = std::function<void(SubscriptionId subscriptionId)>;

// Get physics simulation time steps per second.
//
// \param scenePath Gets the stepping for given scene, if 0 is provided it returns the first found scene
// \return Current time steps per second
using GetSimulationTimeStepsPerSecondFn = std::function<uint32_t(long stageId, uint64_t scenePath)>;

// Get physics simulation timestamp.
//
// Timestamp will increase with every simulation step.
//
// \return Current timestamp
using GetSimulationTimestampFn = std::function<uint64_t()>;

// Get the number of physics steps performed in the active simulation.
//
// The step count resets to 0 when a new simulation starts.
//
// \return Number of steps since the currently active simulation started or 0 if there is no active simulation.
using GetSimulationStepCountFn = std::function<uint64_t()>;

// Applies a force (or impulse) defined in the global coordinate frame, acting at a particular
// point in global coordinates, to the actor.
//
// \param[in] stageId    USD stageId
// \param[in] path		  Body USD path encoded to uint64_t
// \param[in] force      Force / impulse to add, defined in the global frame.
// \param[in] pos        Position in the global frame to add the force at.
// \param[in] mode       The mode to use when applying the force/impulse
using AddForceAtPosFn = std::function<void(
    uint64_t stageId, uint64_t path, const carb::Float3& force, const carb::Float3& pos, ForceModeType::Enum mode)>;

// Applies a torque (or impulse) at the center of mass
// \param[in] stageId    USD stageId
// \param[in] path		  Body USD path encoded to uint64_t
// \param[in] force      Torque to add to the body center of mass
using AddTorqueFn = std::function<void(uint64_t stageId, uint64_t path, const carb::Float3& torque)>;

// Wakes up body on given path
// \param[in] stageId    USD stageId
// \param[in] path		  Body USD path encoded to uint64_t
using WakeUpFn = std::function<void(uint64_t stageId, uint64_t path)>;

// Puts to sleep body on given path
// \param[in] stageId    USD stageId
// \param[in] path		  Body USD path encoded to uint64_t
using PutToSleepFn = std::function<void(uint64_t stageId, uint64_t path)>;

// Checks whether a body sleeps
// \param[in] stageId    USD stageId
// \param[in] path		  Body USD path encoded to uint64_t
// \return True if body is asleep
using IsSleepingFn = std::function<bool(uint64_t stageId, uint64_t path)>;

// Subscribe to physics pre/post step events.
//
// \note Subscriptions cannot be changed in the onUpdate callback
//
// \param onUpdate The callback function to be called on update.
// \param preStep  Whether to execute this callback right *before* the physics step event. If this is false, the
//                 callback will be executed right *after* the physics step event.
// \param order    An integer value used to order the callbacks: 0 means "highest priority", 1 is "less priority"
// \return Subscription Id for release, returns kInvalidSubscriptionId if failed
using SubscribePhysicsOnStepEventsFn =
    std::function<SubscriptionId(bool preStep, int order, OnPhysicsStepEventFn onUpdate)>;

// Unsubscribes to pre/post update events.
//
// \note Subscription cannot be changed in the onUpdate callback
//
// subscriptionId SubscriptionId obtained via @ref subscribePhysicsOnStepEvents.
using UnsubscribePhysicsOnStepEventsFn = std::function<void(SubscriptionId subscriptionId)>;


struct SimulationFns
{
    SimulationFns()
        : attachStage(nullptr),
          detachStage(nullptr),
          getAttachedStage(nullptr),
          simulate(nullptr),
          fetchResults(nullptr),
          checkResults(nullptr),
          flushChanges(nullptr),
          pauseChangeTracking(nullptr),
          isChangeTrackingPaused(nullptr),
          subscribePhysicsContactReportEvents(nullptr),
          unsubscribePhysicsContactReportEvents(nullptr),
          getSimulationTimeStepsPerSecond(nullptr),
          getSimulationTimestamp(nullptr),
          getSimulationStepCount(nullptr),
          addForceAtPos(nullptr),
          addTorque(nullptr),
          wakeUp(nullptr),
          putToSleep(nullptr),
          isSleeping(nullptr),
          subscribePhysicsOnStepEvents(nullptr),
          unsubscribePhysicsOnStepEvents(nullptr)
    {
    }

    AttachStageFn attachStage;
    DetachStageFn detachStage;
    GetAttachedStageFn getAttachedStage;
    SimulateFn simulate;
    FetchResultsFn fetchResults;
    CheckResultsFn checkResults;
    FlushChangesFn flushChanges;
    PauseChangeTrackingFn pauseChangeTracking;
    IsChangeTrackingPausedFn isChangeTrackingPaused;
    SubscribePhysicsContactReportEventsFn subscribePhysicsContactReportEvents;
    UnsubscribePhysicsContactReportEventsFn unsubscribePhysicsContactReportEvents;
    GetSimulationTimeStepsPerSecondFn getSimulationTimeStepsPerSecond;
    GetSimulationTimestampFn getSimulationTimestamp;
    GetSimulationStepCountFn getSimulationStepCount;
    AddForceAtPosFn addForceAtPos;
    AddTorqueFn addTorque;
    WakeUpFn wakeUp;
    PutToSleepFn putToSleep;
    IsSleepingFn isSleeping;
    SubscribePhysicsOnStepEventsFn subscribePhysicsOnStepEvents;
    UnsubscribePhysicsOnStepEventsFn unsubscribePhysicsOnStepEvents;
};

} // namespace physics
} // namespace omni

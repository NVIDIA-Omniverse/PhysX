// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include <omni/physics/simulation/IPhysicsSimulation.h>
#include "OmniPhysics.h"

using namespace omni;
using namespace physics;

// Initialize physics simulation with a USD stage
bool initialize(long id)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFns.initialize)
        {
            if (!simulation.second.simulation.simulationFns.initialize(id))
            {
                CARB_LOG_ERROR("Failed to initialize for simulation %s", simulation.second.simulationName.c_str());
                return false;
            }
        }
    }
    OmniPhysics::getInstance().sendSimulationEvent(SimulationEvent::eResumed);
    OmniPhysics::getInstance().setSimulationStarted(true);
    return true;
}

// Close the simulation
void close()
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFns.close)
        {
            simulation.second.simulation.simulationFns.close();
        }
    }
    if (OmniPhysics::getInstance().hasSimulationStarted())
    {
        OmniPhysics::getInstance().sendSimulationEvent(SimulationEvent::eStopped);
        OmniPhysics::getInstance().setSimulationStarted(false);
    }
}

// Get currently attached USD stage
long getAttachedStage()
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFns.getAttachedStage)
        {
            return simulation.second.simulation.simulationFns.getAttachedStage();
        }
    }
    return 0;
}

// Simulate physics asynchronously
void simulateAsync(float elapsedTime, float currentTime)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFns.simulateAsync)
        {
            simulation.second.simulation.simulationFns.simulateAsync(elapsedTime, currentTime);
        }
    }
}

// Simulate physics synchronously - runs simulation and waits for results
void simulate(float elapsedTime, float currentTime)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFns.simulate)
        {
            simulation.second.simulation.simulationFns.simulate(elapsedTime, currentTime);
        }
    }
}

// Fetch simulation results
void fetchResults()
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFns.fetchResults)
        {
            simulation.second.simulation.simulationFns.fetchResults();
        }
    }
}

// Check if simulation finished
bool checkResults()
{
    bool workDone = true;
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFns.checkResults)
        {
            if (!simulation.second.simulation.simulationFns.checkResults())
            {
                workDone = false;
            }
        }
    }
    return workDone;
}

// Flush changes
void flushChanges()
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFns.flushChanges)
        {
            simulation.second.simulation.simulationFns.flushChanges();
        }
    }
}

// Pause change tracking
void pauseChangeTracking(bool pause)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.simulationFns.pauseChangeTracking)
        {
            simulation.second.simulation.simulationFns.pauseChangeTracking(pause);
        }
    }
}

// Check if change tracking is paused
bool isChangeTrackingPaused(SimulationId id)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    SimulationMap::const_iterator fit = simulations.find(id);
    if (fit != simulations.end() && fit->second.simulation.simulationFns.isChangeTrackingPaused)
    {
        return fit->second.simulation.simulationFns.isChangeTrackingPaused();
    }
    return false;
}

// Subscribe to physics simulation contact report events
SubscriptionId subscribePhysicsContactReportEvents(OnContactReportEventFn onEvent)
{
    OmniPhysics& omniPhysics = OmniPhysics::getInstance();
    const SimulationMap& simulations = omniPhysics.getSimulationRegistry().getSimulations();
    
    std::vector<SimulationSubscriptionEntry> simulationSubscriptions;
    
    for (const auto& simulation : simulations)
    {
        if (simulation.second.simulation.simulationFns.subscribePhysicsContactReportEvents)
        {
            SubscriptionId simSubId = simulation.second.simulation.simulationFns.subscribePhysicsContactReportEvents(onEvent);
            if (simSubId != kInvalidSubscriptionId)
            {
                simulationSubscriptions.push_back({simulation.first, simSubId});
            }
        }
    }
    
    if (simulationSubscriptions.empty())
    {
        return kInvalidSubscriptionId;
    }
    
    return omniPhysics.addContactReportSubscription(std::move(simulationSubscriptions));
}

// Unsubscribe from physics simulation contact report events
void unsubscribePhysicsContactReportEvents(SubscriptionId subscriptionId)
{
    if (subscriptionId == kInvalidSubscriptionId)
    {
        return;
    }
    
    OmniPhysics& omniPhysics = OmniPhysics::getInstance();
    std::vector<SimulationSubscriptionEntry> simulationSubscriptions = 
        omniPhysics.removeContactReportSubscription(subscriptionId);
    
    if (simulationSubscriptions.empty())
    {
        return;
    }
    
    const SimulationMap& simulations = omniPhysics.getSimulationRegistry().getSimulations();
    
    for (const auto& simSub : simulationSubscriptions)
    {
        auto simIt = simulations.find(simSub.simulationId);
        if (simIt != simulations.end() &&
            simIt->second.simulation.simulationFns.unsubscribePhysicsContactReportEvents)
        {
            simIt->second.simulation.simulationFns.unsubscribePhysicsContactReportEvents(simSub.subscriptionId);
        }
    }
}

// Get physics simulation time steps per second
uint32_t getSimulationTimeStepsPerSecond(SimulationId id, long stageId, uint64_t scenePath)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    SimulationMap::const_iterator fit = simulations.find(id);
    if (fit != simulations.end() && fit->second.simulation.simulationFns.getSimulationTimeStepsPerSecond)
    {
        return fit->second.simulation.simulationFns.getSimulationTimeStepsPerSecond(stageId, scenePath);
    }
    return 0;
}

// Get physics simulation timestamp
uint64_t getSimulationTimestamp(SimulationId id)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    SimulationMap::const_iterator fit = simulations.find(id);
    if (fit != simulations.end() && fit->second.simulation.simulationFns.getSimulationTimestamp)
    {
        return fit->second.simulation.simulationFns.getSimulationTimestamp();
    }
    return 0;
}

// Get the number of physics steps performed in the active simulation
uint64_t getSimulationStepCount(SimulationId id)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    SimulationMap::const_iterator fit = simulations.find(id);
    if (fit != simulations.end() && fit->second.simulation.simulationFns.getSimulationStepCount)
    {
        return fit->second.simulation.simulationFns.getSimulationStepCount();
    }
    return 0;
}

// Subscribe to physics pre/post step events
SubscriptionId subscribePhysicsOnStepEvents(bool preStep, int order, OnPhysicsStepEventFn onUpdate)
{
    OmniPhysics& omniPhysics = OmniPhysics::getInstance();
    const SimulationMap& simulations = omniPhysics.getSimulationRegistry().getSimulations();
    
    std::vector<SimulationSubscriptionEntry> simulationSubscriptions;
    
    for (const auto& simulation : simulations)
    {
        if (simulation.second.simulation.simulationFns.subscribePhysicsOnStepEvents)
        {
            SubscriptionId simSubId = simulation.second.simulation.simulationFns.subscribePhysicsOnStepEvents(
                preStep, order, onUpdate);
            if (simSubId != kInvalidSubscriptionId)
            {
                simulationSubscriptions.push_back({simulation.first, simSubId});
            }
        }
    }
    
    if (simulationSubscriptions.empty())
    {
        return kInvalidSubscriptionId;
    }
    
    return omniPhysics.addOnStepSubscription(std::move(simulationSubscriptions));
}

// Unsubscribe from physics pre/post step events
void unsubscribePhysicsOnStepEvents(SubscriptionId subscriptionId)
{
    if (subscriptionId == kInvalidSubscriptionId)
    {
        return;
    }
    
    OmniPhysics& omniPhysics = OmniPhysics::getInstance();
    std::vector<SimulationSubscriptionEntry> simulationSubscriptions = 
        omniPhysics.removeOnStepSubscription(subscriptionId);
    
    if (simulationSubscriptions.empty())
    {
        return;
    }
    
    const SimulationMap& simulations = omniPhysics.getSimulationRegistry().getSimulations();
    
    for (const auto& simSub : simulationSubscriptions)
    {
        auto simIt = simulations.find(simSub.simulationId);
        if (simIt != simulations.end() &&
            simIt->second.simulation.simulationFns.unsubscribePhysicsOnStepEvents)
        {
            simIt->second.simulation.simulationFns.unsubscribePhysicsOnStepEvents(simSub.subscriptionId);
        }
    }
}

// Check if simulation is capable of simulating given schema names
bool isCapableOfSimulating(SimulationId id, const char** schemaNames, size_t schemaNamesCount, bool* isCapable)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    SimulationMap::const_iterator fit = simulations.find(id);
    if (fit != simulations.end() && fit->second.simulation.simulationFns.isCapableOfSimulating)
    {
        return fit->second.simulation.simulationFns.isCapableOfSimulating(schemaNames, schemaNamesCount, isCapable);
    }
    return false;
}

void fillInterface(IPhysicsSimulation& iface)
{
    iface.initialize = initialize;
    iface.close = close;
    iface.getAttachedStage = getAttachedStage;
    iface.simulateAsync = simulateAsync;
    iface.simulate = simulate;
    iface.fetchResults = fetchResults;
    iface.checkResults = checkResults;
    iface.flushChanges = flushChanges;
    iface.pauseChangeTracking = pauseChangeTracking;
    iface.isChangeTrackingPaused = isChangeTrackingPaused;
    iface.subscribePhysicsContactReportEvents = subscribePhysicsContactReportEvents;
    iface.unsubscribePhysicsContactReportEvents = unsubscribePhysicsContactReportEvents;
    iface.getSimulationTimeStepsPerSecond = getSimulationTimeStepsPerSecond;
    iface.getSimulationTimestamp = getSimulationTimestamp;
    iface.getSimulationStepCount = getSimulationStepCount;
    iface.subscribePhysicsOnStepEvents = subscribePhysicsOnStepEvents;
    iface.unsubscribePhysicsOnStepEvents = unsubscribePhysicsOnStepEvents;
    iface.isCapableOfSimulating = isCapableOfSimulating;
}

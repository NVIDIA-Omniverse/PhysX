// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//


#include <omni/physics/simulation/IPhysicsStageUpdate.h>

#include "OmniPhysics.h"

using namespace omni;
using namespace physics;

// Attach the stage
void onAttach(long int stageId)
{
    SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (SimulationMap::reference simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.stageUpdateFns.onAttach)
        {
            simulation.second.simulation.stageUpdateFns.onAttach(stageId);
        }
    }
}

// Detach the stage
void onDetach()
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.stageUpdateFns.onDetach)
        {
            simulation.second.simulation.stageUpdateFns.onDetach();
        }
    }

    if (OmniPhysics::getInstance().hasSimulationStarted())
    {        
        OmniPhysics::getInstance().sendSimulationEvent(SimulationEvent::eStopped);
        OmniPhysics::getInstance().setSimulationStarted(false);
    }
}

//  On stage update
void onUpdate(float currentTime, float elapsedSecs, bool enableUpdate)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.stageUpdateFns.onUpdate)
        {
            simulation.second.simulation.stageUpdateFns.onUpdate(currentTime, elapsedSecs, enableUpdate);
        }
    }
}

// On resume
void onResume(float currentTime)
{    
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    if (simulations.empty())
    {
        CARB_LOG_WARN("No simulation registered, please make sure to register a simulation.");
    }

    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.stageUpdateFns.onResume)
        {
            simulation.second.simulation.stageUpdateFns.onResume(currentTime);
        }
    }
    OmniPhysics::getInstance().sendSimulationEvent(SimulationEvent::eResumed);
}

// On pause
void onPause()
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.stageUpdateFns.onPause)
        {
            simulation.second.simulation.stageUpdateFns.onPause();
        }
    }
    OmniPhysics::getInstance().sendSimulationEvent(SimulationEvent::ePaused);
}

// On reset
void onReset()
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.stageUpdateFns.onReset)
        {
            simulation.second.simulation.stageUpdateFns.onReset();
        }
    }
    OmniPhysics::getInstance().sendSimulationEvent(SimulationEvent::eStopped);
}

// Handle raycast
static void handleRaycast(const float* orig, const float* dir, bool input)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.stageUpdateFns.handleRaycast)
        {
            simulation.second.simulation.stageUpdateFns.handleRaycast(orig, dir, input);
        }
    }
}

// Force load physics from USD
void forceLoadPhysicsFromUSD()
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.stageUpdateFns.forceLoadPhysicsFromUSD)
        {
            simulation.second.simulation.stageUpdateFns.forceLoadPhysicsFromUSD();
        }
    }
}

// Release physics objects
void releasePhysicsObjects()
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.stageUpdateFns.releasePhysicsObjects)
        {
            simulation.second.simulation.stageUpdateFns.releasePhysicsObjects();
        }
    }
}

// Reset simulation
void resetSimulation()
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.stageUpdateFns.resetSimulation)
        {
            simulation.second.simulation.stageUpdateFns.resetSimulation();
        }
    }
    OmniPhysics::getInstance().setSimulationStarted(false);
}

// Start simulation
void startSimulation()
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.stageUpdateFns.startSimulation)
        {
            simulation.second.simulation.stageUpdateFns.startSimulation();
        }
    }
    OmniPhysics::getInstance().sendSimulationEvent(SimulationEvent::eResumed);
    OmniPhysics::getInstance().setSimulationStarted(true);
}

// Get simulation event stream
carb::events::IEventStreamPtr getSimulationEventStream()
{
    return OmniPhysics::getInstance().getSimulationEventStream();
}

void fillInterface(IPhysicsStageUpdate& iface)
{
    iface.onAttach = onAttach;
    iface.onDetach = onDetach;
    iface.onUpdate = onUpdate;
    iface.onResume = onResume;
    iface.onPause = onPause;
    iface.onReset = onReset;
    iface.handleRaycast = handleRaycast;
    iface.forceLoadPhysicsFromUSD = forceLoadPhysicsFromUSD;
    iface.releasePhysicsObjects = releasePhysicsObjects;
    iface.resetSimulation = resetSimulation;
    iface.startSimulation = startSimulation;
    iface.getSimulationEventStream = getSimulationEventStream;
}

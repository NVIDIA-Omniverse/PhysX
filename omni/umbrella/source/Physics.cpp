// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//


#include <omni/physics/simulation/IPhysics.h>
#include "OmniPhysics.h"


using namespace omni;
using namespace physics;


SimulationId registerSimulation(const Simulation& simulation, const char* simulationName)
{
    return OmniPhysics::getInstance().getSimulationRegistry().registerSimulation(simulation, simulationName);
}

void unregisterSimulation(const SimulationId& id)
{
    OmniPhysics::getInstance().getSimulationRegistry().unregisterSimulation(id);
}

const Simulation* getSimulation(const SimulationId& id)
{
    return OmniPhysics::getInstance().getSimulationRegistry().getSimulation(id);
}

const char* getSimulationName(const SimulationId& id)
{
    return OmniPhysics::getInstance().getSimulationRegistry().getSimulationName(id);
}

size_t getNumSimulations()
{
    return OmniPhysics::getInstance().getSimulationRegistry().getSimulations().size();
}

size_t getSimulationIds(SimulationId* simulationIds, size_t bufferSize)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();

    size_t numSimulations = simulations.size();
    if (numSimulations > bufferSize)
    {
        numSimulations = bufferSize;
    }

    size_t i = 0;
    for (SimulationMap::const_reference ref : simulations)
    {
        simulationIds[i] = ref.first;
        i++;
        if (i >= numSimulations)
            break;
    }
    return numSimulations;
}

void activateSimulation(const SimulationId& id)
{
    OmniPhysics::getInstance().getSimulationRegistry().activateSimulation(id);
}

void deactivateSimulation(const SimulationId& id)
{
    OmniPhysics::getInstance().getSimulationRegistry().deactivateSimulation(id);
}

bool isSimulationActive(const SimulationId& id)
{
    return OmniPhysics::getInstance().getSimulationRegistry().isSimulationActive(id);
}

SubscriptionId subscribeSimulationRegistryEvents(OnSimulationRegistryEventFn onEvent, void* userData=nullptr)
{
    return OmniPhysics::getInstance().getSimulationRegistry().subscribeSimulationRegistryEvents(onEvent, userData);
}

void unsubscribeSimulationRegistryEvents(SubscriptionId subscriptionId)
{
    OmniPhysics::getInstance().getSimulationRegistry().unsubscribeSimulationRegistryEvents(subscriptionId);
}

void fillInterface(IPhysics& iface)
{
    iface.registerSimulation = registerSimulation;
    iface.unregisterSimulation = unregisterSimulation;
    iface.getSimulation = getSimulation;
    iface.getSimulationName = getSimulationName;
    iface.getNumSimulations = getNumSimulations;
    iface.getSimulationIds = getSimulationIds;
    iface.activateSimulation = activateSimulation;
    iface.deactivateSimulation = deactivateSimulation;
    iface.isSimulationActive = isSimulationActive;
    iface.subscribeSimulationRegistryEvents = subscribeSimulationRegistryEvents;
    iface.unsubscribeSimulationRegistryEvents = unsubscribeSimulationRegistryEvents;
}

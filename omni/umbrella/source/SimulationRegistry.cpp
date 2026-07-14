// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//


#include <carb/logging/Log.h>
#include "SimulationRegistry.h"


using namespace omni;
using namespace physics;


SimulationRegistry::SimulationRegistry()
	: mSimulationsCounter(0), mSimulationRegistryEventSubscriptionIdCounter(0)
{	
}

SimulationRegistry::~SimulationRegistry()
{
}

void SimulationRegistry::notifyEvent(const SimulationRegistryEventType::Enum eventType, const SimulationId& id, const char* simulationName)
{
    for (const auto& subscription : mSimulationRegistryEventCallbacks)
    {
        // Call the callback function for each subscriber, passing the stored user data pointer.
        subscription.second.first(eventType, id, simulationName, subscription.second.second);
    }
}

SimulationId SimulationRegistry::registerSimulation(const Simulation& simulation, const char* simulationName)
{
   const SimulationId id = mSimulationsCounter++;
   mSimulations[id] = SimulationRecord{simulation, simulationName, true};
   notifyEvent(SimulationRegistryEventType::eSIMULATION_REGISTERED, id, simulationName);
   return id;
}

void SimulationRegistry::unregisterSimulation(const SimulationId& id)
{
    SimulationMap::const_iterator it = mSimulations.find(id);
    if (it == mSimulations.end())
    {
       CARB_LOG_ERROR("SimulationRegistry::unregisterSimulation: simulation not found");
       return;
    }
    
    // Copy to temporary string to be able to send the notice after the erasure.
    const std::string simulationNameStr(it->second.simulationName);

    mSimulations.erase(it);
    notifyEvent(SimulationRegistryEventType::eSIMULATION_UNREGISTERED, id, simulationNameStr.c_str());
}

const Simulation* SimulationRegistry::getSimulation(const SimulationId& id) const
{
   SimulationMap::const_iterator it = mSimulations.find(id);
   if (it == mSimulations.end())
   {
      CARB_LOG_ERROR("SimulationRegistry::getSimulation: simulation not found");
      return nullptr;
   }
   return &it->second.simulation;
}

const char* SimulationRegistry::getSimulationName(const SimulationId& id) const
{
    SimulationMap::const_iterator it = mSimulations.find(id);
    if (it == mSimulations.end())
    {
        CARB_LOG_ERROR("SimulationRegistry::getSimulation: simulation not found");
        return nullptr;
    }
    return it->second.simulationName.c_str();
}

void SimulationRegistry::activateSimulation(const SimulationId& id)
{
   SimulationMap::iterator it = mSimulations.find(id);
   if (it == mSimulations.end())
   {
      CARB_LOG_ERROR("SimulationRegistry::activateSimulation: simulation not found");
      return;
   }
   it->second.isActive = true;
   notifyEvent(SimulationRegistryEventType::eSIMULATION_ACTIVATED, id, getSimulationName(id));
}

void SimulationRegistry::deactivateSimulation(const SimulationId& id)
{
   SimulationMap::iterator it = mSimulations.find(id);
   if (it == mSimulations.end())
   {
      CARB_LOG_ERROR("SimulationRegistry::deactivateSimulation: simulation not found");
      return;
   }
   it->second.isActive = false;
   notifyEvent(SimulationRegistryEventType::eSIMULATION_DEACTIVATED, id, getSimulationName(id));
}

bool SimulationRegistry::isSimulationActive(const SimulationId& id)
{
   SimulationMap::const_iterator it = mSimulations.find(id);
   if (it == mSimulations.end())
   {
      CARB_LOG_ERROR("SimulationRegistry::isSimulationActive: simulation not found");
      return false;
   }
   return it->second.isActive;
}

SubscriptionId SimulationRegistry::subscribeSimulationRegistryEvents(
    OnSimulationRegistryEventFn onEvent, void* userData)
{
    if (!onEvent)
    {
        CARB_LOG_ERROR("SimulationRegistry::subscribeSimulationRegistryEvents: onEvent is empty");
        return kInvalidSubscriptionId;
    }
    SubscriptionId id = SubscriptionId(mSimulationRegistryEventSubscriptionIdCounter++);
    mSimulationRegistryEventCallbacks.emplace(id, std::make_pair(onEvent, userData));
    return id;
}

void SimulationRegistry::unsubscribeSimulationRegistryEvents(SubscriptionId subscriptionId)
{
    if (subscriptionId == kInvalidSubscriptionId)
    {
        CARB_LOG_ERROR("SimulationRegistry::unsubscribeSimulationRegistryEvents: subscriptionId is invalid");
        return;
    }
    mSimulationRegistryEventCallbacks.erase(subscriptionId);
}

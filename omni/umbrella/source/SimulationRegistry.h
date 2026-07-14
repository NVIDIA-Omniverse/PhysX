// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#pragma once

#include <omni/physics/simulation/IPhysics.h>

#include <unordered_map>
#include <string>
#include <vector>

namespace omni
{
namespace physics
{

struct SimulationRecord
{
    Simulation simulation;
    std::string simulationName;
    bool isActive;
};

// Simulation map
using SimulationMap = std::unordered_map<SimulationId, SimulationRecord, SimulationIdHash>;
using SimulationRegistryEventSubscribersMap = std::unordered_map<SubscriptionId, std::pair<OnSimulationRegistryEventFn, void*>>;

// Simulation registry
// This class is used to register and unregister simulations
class SimulationRegistry
{
public:
    // Constructor
    SimulationRegistry();

    // Destructor
    ~SimulationRegistry();

    // Register a simulation
    //\param[in] simulation The simulation to register
    //\return The id of the simulation
    SimulationId registerSimulation(const Simulation& simulation, const char* simulationName);

    // Unregister a simulation
    //\param[in] id The id of the simulation to unregister
    void unregisterSimulation(const SimulationId& id);

    // Get a simulation
    //\param[in] id The id of the simulation to get
    //\return The simulation
    const Simulation* getSimulation(const SimulationId& id) const;

    // Get a simulation name
    //\param[in] id The id of the simulation to get
    //\return The simulation
    const char* getSimulationName(const SimulationId& id) const;

    // Get the number of simulations
    // \return Return the number of simulation registered
    size_t getNumSimulations() const
    {
        return mSimulations.size();
    }

    // Get all simulations
    //\return All simulations
    const SimulationMap& getSimulations() const
    {
        return mSimulations;
    }
    SimulationMap& getSimulations()
    {
        return mSimulations;
    }

    // Activate a simulation
    //\param[in] id The id of the simulation to activate
    void activateSimulation(const SimulationId& id);

    // Deactivate a simulation
    //\param[in] id The id of the simulation to deactivate
    void deactivateSimulation(const SimulationId& id);

    // Check if a simulation is active
    //\param[in] id The id of the simulation to check
    //\return True if the simulation is active, false otherwise
    bool isSimulationActive(const SimulationId& id);
    
    // Subscribe to simulation registry events
    //\param[in] onEvent The callback function to be called on simulation registry events
    //\param[in] userData Pointer to user data to be passed to the callback function. Default is nullptr.
    //\return The subscription id used for unsubscribing.
    SubscriptionId subscribeSimulationRegistryEvents(OnSimulationRegistryEventFn onEvent, void* userData=nullptr);

    // Unsubscribe from simulation registry events
    //\param[in] subscriptionId The subscription id returned when subscribing.
    void unsubscribeSimulationRegistryEvents(SubscriptionId subscriptionId);
    
private:
    // The number of simulations
    size_t mSimulationsCounter;

    // All simulations
    SimulationMap mSimulations;

    SimulationRegistryEventSubscribersMap mSimulationRegistryEventCallbacks;
    size_t mSimulationRegistryEventSubscriptionIdCounter;

    void notifyEvent(const SimulationRegistryEventType::Enum eventType, const SimulationId& id, const char* simulationName);
};


} // namespace physics
} // namespace omni

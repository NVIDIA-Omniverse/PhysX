// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//


#include <omni/physics/simulation/IPhysicsBenchmark.h>
#include "OmniPhysics.h"


using namespace omni;
using namespace physics;


SubscriptionId subscribeProfileStatsEvents(ProfileStatsNotificationFn onEvent)
{
    OmniPhysics& omniPhysics = OmniPhysics::getInstance();
    const SimulationMap& simulations = omniPhysics.getSimulationRegistry().getSimulations();
    
    std::vector<SimulationSubscriptionEntry> simulationSubscriptions;
    
    for (const auto& simulation : simulations)
    {
        if (simulation.second.simulation.benchmarkFns.subscribeProfileStatsEvents)
        {
            SubscriptionId simSubId = simulation.second.simulation.benchmarkFns.subscribeProfileStatsEvents(onEvent);
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
    
    return omniPhysics.addBenchmarkSubscription(std::move(simulationSubscriptions));
}

// Unsubscribes to simulation events.
void unsubscribeProfileStatsEvents(SubscriptionId subscriptionId)
{
    if (subscriptionId == kInvalidSubscriptionId)
    {
        return;
    }
    
    OmniPhysics& omniPhysics = OmniPhysics::getInstance();
    std::vector<SimulationSubscriptionEntry> simulationSubscriptions = 
        omniPhysics.removeBenchmarkSubscription(subscriptionId);
    
    if (simulationSubscriptions.empty())
    {
        return;
    }
    
    const SimulationMap& simulations = omniPhysics.getSimulationRegistry().getSimulations();
    
    for (const auto& simSub : simulationSubscriptions)
    {
        auto simIt = simulations.find(simSub.simulationId);
        if (simIt != simulations.end() &&
            simIt->second.simulation.benchmarkFns.unsubscribeProfileStatsEvents)
        {
            simIt->second.simulation.benchmarkFns.unsubscribeProfileStatsEvents(simSub.subscriptionId);
        }
    }
}

void fillInterface(IPhysicsBenchmarks& iface)
{
    iface.subscribeProfileStatsEvents = subscribeProfileStatsEvents;
    iface.unsubscribeProfileStatsEvents = unsubscribeProfileStatsEvents;
}

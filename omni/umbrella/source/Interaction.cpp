// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include <omni/physics/simulation/IPhysicsInteraction.h>

#include "OmniPhysics.h"


using namespace omni;
using namespace physics;


// Control the behavior of the ResetOnStop setting.
void disableResetOnStop(bool disable)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.interactionFns.disableResetOnStop)
        {
            return simulation.second.simulation.interactionFns.disableResetOnStop(disable);
        }
    }
}

// Returns the current state of the ResetOnStop setting.
bool isDisabledResetOnStop(SimulationId simId)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    SimulationMap::const_iterator fit = simulations.find(simId);
    if (fit != simulations.end() && fit->second.simulation.interactionFns.isDisabledResetOnStop)
    {
        return fit->second.simulation.interactionFns.isDisabledResetOnStop();
    }
    return false;
}

// Called when a raycast request is executed - used for picking.
static void handleRaycast(const float* orig, const float* dir, bool input)
{
    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.interactionFns.handleRaycast)
        {
            simulation.second.simulation.interactionFns.handleRaycast(orig, dir, input);
        }
    }
}

static carb::dictionary::Item* getPrimDebugData(const char* primPath)
{
    carb::dictionary::IDictionary* iDictionary = carb::getCachedInterface<carb::dictionary::IDictionary>();
    carb::dictionary::Item* debugData = iDictionary->createItem(nullptr, "", carb::dictionary::ItemType::eDictionary);

    const SimulationMap& simulations = OmniPhysics::getInstance().getSimulationRegistry().getSimulations();
    for (const auto& simulation : simulations)
    {
        if (simulation.second.isActive && simulation.second.simulation.interactionFns.getPrimDebugData)
        {
            carb::dictionary::Item* item = simulation.second.simulation.interactionFns.getPrimDebugData(primPath);
            if (item && iDictionary->getItemType(item) == carb::dictionary::ItemType::eDictionary)
            {
                iDictionary->update(debugData, "", item, "", carb::dictionary::overwriteOriginal, nullptr);
            }
        }
    }
    return debugData;
}


void fillInterface(IPhysicsInteraction& iface)
{
    iface.disableResetOnStop = disableResetOnStop;
    iface.isDisabledResetOnStop = isDisabledResetOnStop;
    iface.handleRaycast = handleRaycast;
    iface.getPrimDebugData = getPrimDebugData;
}

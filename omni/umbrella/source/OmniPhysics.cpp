// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#define CARB_EXPORTS

#include <omni/physics/simulation/IPhysics.h>
#include <omni/physics/simulation/IPhysicsBenchmark.h>
#include <omni/physics/simulation/IPhysicsInteraction.h>
#include <omni/physics/simulation/IPhysicsSceneQuery.h>
#include <omni/physics/simulation/IPhysicsSimulation.h>
#include <omni/physics/simulation/IPhysicsStageUpdate.h>

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/events/EventsUtils.h>
#include <carb/events/IEvents.h>

#include "OmniPhysics.h"

using namespace omni;
using namespace physics;



const struct carb::PluginImplDesc kPluginImpl = { "omni.physics.plugin", "Physics", "NVIDIA",
                                                  carb::PluginHotReload::eDisabled, "dev" };
CARB_PLUGIN_IMPL(kPluginImpl,
                 omni::physics::IPhysics,
                 omni::physics::IPhysicsBenchmarks,
                 omni::physics::IPhysicsInteraction,
                 omni::physics::IPhysicsSceneQuery,
                 omni::physics::IPhysicsSimulation,
                 omni::physics::IPhysicsStageUpdate)
CARB_PLUGIN_IMPL_NO_DEPS()

static OmniPhysics* gOmniPhysicsInstance;

CARB_EXPORT void carbOnPluginStartup()
{
    OmniPhysics::createOmniPhysicsInstance();
    OmniPhysics& omniPhysics = OmniPhysics::getInstance();
    gOmniPhysicsInstance = &omniPhysics;
    omniPhysics.onStartup();
}

CARB_EXPORT void carbOnPluginShutdown()
{
    OmniPhysics& omniPhysics = OmniPhysics::getInstance();
    gOmniPhysicsInstance = nullptr;
    omniPhysics.onShutdown();
}

void OmniPhysics::onStartup()
{
    mSimulationEventStream = carb::events::getCachedEventsInterface()->createEventStream();
}

void OmniPhysics::onShutdown()
{
    mSimulationEventStream = nullptr;
}

void OmniPhysics::sendSimulationEvent(SimulationEvent type)
{
    using namespace carb::events;
    if (mSimulationEventStream)
    {
        carb::events::IEventPtr event = carb::stealObject(
            mSimulationEventStream->createEventPtr(static_cast<EventType>(type), kGlobalSenderId));
        mSimulationEventStream->dispatch(event.get());
    }
}

void OmniPhysics::createOmniPhysicsInstance()
{
    gOmniPhysicsInstance = new OmniPhysics();
}

OmniPhysics& OmniPhysics::getInstance()
{
    return *gOmniPhysicsInstance;
}

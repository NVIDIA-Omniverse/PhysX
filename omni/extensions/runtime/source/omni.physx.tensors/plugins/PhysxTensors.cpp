// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#define CARB_EXPORTS

#include <carb/events/IEvents.h>
#include <carb/Framework.h>
#include <carb/PluginUtils.h>


#include <omni/ext/IExt.h>
#include <omni/kit/IStageUpdate.h>
#include <omni/kit/KitUpdateOrder.h>

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxPrivate.h>
#include <omni/physx/IPhysxJoint.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physics/tensors/TensorApi.h>

#include "gpu/CudaCommon.h"
#include "GlobalsAreBad.h"
#include "SimulationBackend.h"

namespace omni
{
namespace physx
{
namespace tensors
{

class ExtensionImpl : public omni::ext::IExt
{
public:
    void onStartup(const char* extId) override;
    void onShutdown() override;
};

}
}
}

const struct carb::PluginImplDesc kPluginImpl = { "omni.physx.tensors.plugin", "PhysX Tensor API implementation",
                                                  "NVIDIA", carb::PluginHotReload::eDisabled, "dev" };

CARB_PLUGIN_IMPL(kPluginImpl, omni::physx::tensors::ExtensionImpl)
CARB_PLUGIN_IMPL_DEPS(omni::physx::IPhysx,
                      omni::physx::IPhysxSimulation,
                      omni::physx::IPhysxPrivate,
                      omni::physx::IPhysxJoint,
                      omni::physics::tensors::BackendRegistry,
                      omni::kit::IStageUpdate)

namespace omni
{
namespace physx
{
namespace tensors
{

// shared globals
omni::physx::IPhysx* g_physx = nullptr;
omni::physx::IPhysxSimulation* g_physxSimulation = nullptr;
omni::physx::IPhysxPrivate* g_physxPrivate = nullptr;
omni::physx::IPhysxJoint* g_physxJoint = nullptr;

// private stuff
namespace
{
omni::kit::StageUpdatePtr g_su;

omni::physics::tensors::BackendRegistry* g_backendRegistry = nullptr;
constexpr const char* g_simBackendName = "physx";
omni::physx::tensors::SimulationBackend g_simBackend;

////////////////////////
class SimulationEventListener : public carb::events::IEventListener
{
    void onEvent(carb::events::IEvent* e) override
    {
        int eventType = int(e->type);

        //printf("~!~!~! Got SimulationEvent %d\n", eventType);

        if (eventType == omni::physx::SimulationEvent::eStopped)
        {
            g_simBackend.reset();
        }
    }

    size_t addRef() override
    {
        return ++mRefCount;
    }

    size_t release() override
    {
        // hmmm
        if (mRefCount)
        {
            --mRefCount;
        }
        return mRefCount;
    }

    size_t mRefCount = 0;
};

SimulationEventListener* g_simEventListener = nullptr;
carb::events::IEventStreamPtr g_simEventStream;
carb::events::ISubscriptionPtr g_simEventSubscription;
/////////////////////////

//
// TODO: get rid of stage update!  Replace with physics events.
//
omni::kit::StageUpdateNode* g_suPrePhysicsNode = nullptr;

} // end of anonymous namespace


SimulationBackend& GetSimulationBackend()
{
    return g_simBackend;
}

void SuPrePhysicsUpdate(float currentTime, float elapsedTime, const omni::kit::StageUpdateSettings* settings, void* data)
{
    //printf("!!! SuPrePhysicsUpdate %d %d\n", int(g_simBackend.getTimestamp()), int(g_simBackend.getStepCount()));
    g_simBackend.prePhysicsUpdate();
}


void onDetach(void* userData)
{
    g_simBackend.reset();
}


void ExtensionImpl::onStartup(const char* extId)
{
    carb::Framework* framework = carb::getFramework();
    if (!framework)
    {
        CARB_LOG_ERROR("Failed to get Carbonite framework");
        return;
    }

    g_physx = carb::getCachedInterface<omni::physx::IPhysx>();
    if (!g_physx)
    {
        CARB_LOG_ERROR("Failed to acquire PhysX interface");
        return;
    }

    g_physxSimulation = carb::getCachedInterface<omni::physx::IPhysxSimulation>();
    if (!g_physxSimulation)
    {
        CARB_LOG_ERROR("Failed to acquire PhysX Simulation interface");
        return;
    }

    g_physxPrivate = carb::getCachedInterface<omni::physx::IPhysxPrivate>();
    if (!g_physxPrivate)
    {
        CARB_LOG_ERROR("Failed to acquire PhysX Private interface");
        return;
    }

    g_physxJoint = carb::getCachedInterface<omni::physx::IPhysxJoint>();
    if (!g_physxJoint)
    {
        CARB_LOG_ERROR("Failed to acquire PhysX Joint interface");
        return;
    }

    g_simEventListener = new SimulationEventListener;
    g_simEventStream = g_physx->getSimulationEventStreamV2();
    g_simEventSubscription = g_simEventStream->createSubscriptionToPop(g_simEventListener, 0);

    g_su = omni::kit::getStageUpdate();
    if (!g_su)
    {
        CARB_LOG_ERROR("Failed to acquire stage update interface");
        return;
    }

    // IStageUpdate node
    omni::kit::StageUpdateNodeDesc suPrePhysicsDesc = { 0 };
    suPrePhysicsDesc.displayName = "omni.physx.tensors pre-physics";
    suPrePhysicsDesc.order = omni::kit::update::eIUsdStageUpdateTensorPrePhysics; // should run before OmniPhysics update
    suPrePhysicsDesc.onUpdate = SuPrePhysicsUpdate;
    suPrePhysicsDesc.onDetach = onDetach;

    g_suPrePhysicsNode = g_su->createStageUpdateNode(suPrePhysicsDesc);
    if (!g_suPrePhysicsNode)
    {
        CARB_LOG_ERROR("Failed to create pre-physics stage update node");
    }

    // backend registry
    // FIXME: changed from getCachedInterface since it returned nullptr on linux when tensor ext would not unload correctly
    g_backendRegistry = framework->tryAcquireInterface<omni::physics::tensors::BackendRegistry>();
    if (!g_backendRegistry)
    {
        CARB_LOG_ERROR("Failed to acquire simulation backend registry interface");
        return;
    }

    // register PhysX backend
    if (!g_backendRegistry->registerBackend(g_simBackendName, &g_simBackend))
    {
        CARB_LOG_ERROR("Failed to register simulation backend '%s'", g_simBackendName);
    }
    else
    {
        CARB_LOG_INFO("Registered simulation backend '%s'\n", g_simBackendName);
    }
}

void ExtensionImpl::onShutdown()
{
    g_simEventSubscription = nullptr;
    g_simEventStream = nullptr;
    delete g_simEventListener;
    g_simEventListener = nullptr;

    if (g_suPrePhysicsNode)
    {
        g_su->destroyStageUpdateNode(g_suPrePhysicsNode);
        g_suPrePhysicsNode = nullptr;
    }

    g_su = nullptr;

    if (g_backendRegistry)
    {
        g_backendRegistry->unregisterBackend(g_simBackendName);
        g_backendRegistry = nullptr;
    }

    g_physx = nullptr;
    g_physxSimulation = nullptr;
    g_physxPrivate = nullptr;
    g_physxJoint = nullptr;
}

}
}
}

CARB_EXPORT void carbOnPluginStartup()
{
    //
    // HACK?!
    //
    {
        /*
        CUcontext ctx = nullptr;
        cuCtxGetCurrent(&ctx);
        printf(":=:=:=: ctx before: %p\n", (void*)ctx);
        */

        omni::physx::tensors::createPrimaryCudaContext();

        /*
        ctx = nullptr;
        cuCtxGetCurrent(&ctx);
        printf(":=:=:=: ctx after: %p\n", (void*)ctx);
        */
    }
}

CARB_EXPORT void carbOnPluginShutdown()
{
}

void fillInterface(omni::physx::tensors::ExtensionImpl& iface)
{
}

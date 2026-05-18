// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxPrivate.h>
#include <omni/physx/IPhysxJoint.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physics/tensors/TensorApi.h>

#include <memory>

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

} // namespace tensors
} // namespace physx
} // namespace omni

const struct carb::PluginImplDesc kPluginImpl = { "omni.physx.tensors.plugin", "PhysX Tensor API implementation",
                                                  "NVIDIA", carb::PluginHotReload::eDisabled, "dev" };

CARB_PLUGIN_IMPL(kPluginImpl, omni::physx::tensors::ExtensionImpl)
CARB_PLUGIN_IMPL_DEPS(omni::physx::IPhysx,
                      omni::physx::IPhysxSimulation,
                      omni::physx::IPhysxPrivate,
                      omni::physx::IPhysxJoint,
                      omni::physics::tensors::BackendRegistry)

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
omni::physics::tensors::BackendRegistry* g_backendRegistry = nullptr;
constexpr const char* g_simBackendName = "physx";

// Simulation backend lifetime notes
// ---------------------------------
// - An explicit init/shutdown path (tensorsInit()/tensorsShutdown())
//   lets us manage all Carbonite-dependent resources (event subscriptions, backend registry, etc.).
// - This guarantees that, when the plugin is shut down (either via omni.ext in omni.kit apps
//   or via carbOnPluginShutdown() in SDK/wheel mode), we release those resources
//   *before* Carbonite tears down its core plugins.
// - With this ordering, the default static destruction of g_simBackend at module
//   unload becomes a no-op with respect to Carbonite and is safe.
// - This design preserves existing Kit behavior while allowing custom packaging (Python wheels)
//   to handle the lifetime of this plugin and the Carbonite framework explicitly, without
//   shutdown crashes, intentional leaks, or skipped destructors.
std::unique_ptr<omni::physx::tensors::SimulationBackend> g_simBackend;
bool g_tensorsStarted = false;

////////////////////////
class SimulationEventListener : public carb::events::IEventListener
{
    void onEvent(carb::events::IEvent* e) override
    {
        int eventType = int(e->type);

        // printf("~!~!~! Got SimulationEvent %d\n", eventType);

        if (eventType == omni::physx::SimulationEvent::eStopped)
        {
            if (g_simBackend)
            {
                g_simBackend->reset();
            }
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
// note(malesiani): IStageUpdate-based callbacks were removed; we now always rely on omni.physx events.
omni::physx::SubscriptionId g_physxPreStepSubscriptionId = omni::physx::kInvalidSubscriptionId;

} // end of anonymous namespace


SimulationBackend* GetSimulationBackend()
{
    return g_simBackend.get();
}

void PhysxPreStepCallback(float elapsedTime, void* userData)
{
    // printf("!!! PhysxPreStepCallback %d %d\n", int(g_simBackend->getTimestamp()), int(g_simBackend->getStepCount()));
    if (g_simBackend)
    {
        g_simBackend->prePhysicsUpdate();
    }
}

void onDetach(void* userData)
{
    if (g_simBackend)
    {
        g_simBackend->reset();
    }
}

// Shared init/shutdown so both IExt::onStartup() and carbOnPluginStartup() can use them.
// This makes omni.physx.tensors work both in Kit (extension manager) and in SDK/wheel mode.
void tensorsInit()
{
    if (g_tensorsStarted)
    {
        CARB_LOG_VERBOSE("omni.physx.tensors tensorsInit() already ran, skipping duplicate");
        return;
    }

    CARB_LOG_INFO("omni.physx.tensors tensorsInit() starting...");

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

    // Create simulation backend on first successful initialization
    if (!g_simBackend)
    {
        g_simBackend = std::make_unique<omni::physx::tensors::SimulationBackend>();
    }

    g_simEventListener = new SimulationEventListener;
    g_simEventStream = g_physx->getSimulationEventStreamV2();
    g_simEventSubscription = g_simEventStream->createSubscriptionToPop(g_simEventListener, 0);

    // Use omni.physx for pre-physics callbacks - this replaces IStageUpdate so we don't have to rely on omni.usd anymore
    g_physxPreStepSubscriptionId = g_physx->subscribePhysicsOnStepEvents(true, 0, PhysxPreStepCallback, nullptr);
    if (g_physxPreStepSubscriptionId == omni::physx::kInvalidSubscriptionId)
    {
        CARB_LOG_ERROR("Failed to subscribe to PhysX pre-step events");
    }

    // backend registry
    // FIXME: changed from getCachedInterface since it returned nullptr on linux when tensor ext would not unload
    // correctly
    g_backendRegistry = framework->tryAcquireInterface<omni::physics::tensors::BackendRegistry>();
    if (!g_backendRegistry)
    {
        CARB_LOG_ERROR("Failed to acquire simulation backend registry interface");
        return;
    }

    // register PhysX backend
    if (!g_backendRegistry->registerBackend(g_simBackendName, g_simBackend.get()))
    {
        CARB_LOG_ERROR("Failed to register simulation backend '%s'", g_simBackendName);
    }
    else
    {
        CARB_LOG_INFO("Registered simulation backend '%s'\n", g_simBackendName);
    }

    // Mark as successfully initialized only after all setup completes
    g_tensorsStarted = true;
}

void tensorsShutdown()
{
    if (!g_tensorsStarted)
        return;
    g_tensorsStarted = false;

    // Release event subscriptions and listeners while Carbonite is still alive.
    // This avoids static-destruction-time callbacks into already torn-down plugins.
    g_simEventSubscription = nullptr;
    g_simEventStream = nullptr;
    delete g_simEventListener;
    g_simEventListener = nullptr;

    if (g_physxPreStepSubscriptionId != omni::physx::kInvalidSubscriptionId)
    {
        g_physx->unsubscribePhysicsOnStepEvents(g_physxPreStepSubscriptionId);
        g_physxPreStepSubscriptionId = omni::physx::kInvalidSubscriptionId;
    }

    if (g_backendRegistry)
    {
        g_backendRegistry->unregisterBackend(g_simBackendName);
        g_backendRegistry = nullptr;
    }

    g_physx = nullptr;
    g_physxSimulation = nullptr;
    g_physxPrivate = nullptr;
    g_physxJoint = nullptr;

    // Finally, destroy the simulation backend once all external references
    // (stage update nodes, PhysX callbacks, backend registry) have been removed.
    if (g_simBackend)
    {
        g_simBackend.reset();
    }
}


void ExtensionImpl::onStartup(const char* extId)
{
    tensorsInit();
}

void ExtensionImpl::onShutdown()
{
    tensorsShutdown();
}

} // namespace tensors
} // namespace physx
} // namespace omni

CARB_EXPORT void carbOnPluginStartup()
{
    //
    // HACK?!
    //
    {
        omni::physx::tensors::createPrimaryCudaContext();
    }

    // Initialize the tensors backend when the plugin loads.
    //
    // In SDK/wheel mode there is no extension manager, so this is the only
    // entrypoint as carbonite plugin. In Kit, initialization is also driven by ExtensionImpl::onStartup()
    // with proper dependencies, but tensorsInit() is idempotent (guarded by
    // g_tensorsStarted), so calling it here as well is safe.
    omni::physx::tensors::tensorsInit();
}

CARB_EXPORT void carbOnPluginShutdown()
{
    // Ensure Carbonite-dependent resources are cleaned up before the framework
    // tears down its core plugins. This prevents static destructors (event subscriptions,
    // backend registry, etc.) from calling back into an already-partially-destroyed
    // Carbonite, which previously caused shutdown-time segfaults in TensorAPI SDK wheels.
    //
    // tensorsShutdown() is idempotent (guarded by g_tensorsStarted), so it is safe to
    // call here for both Kit and SDK/wheel modes even though ExtensionImpl::onShutdown()
    // also calls it in Kit.
    omni::physx::tensors::tensorsShutdown();
}

void fillInterface(omni::physx::tensors::ExtensionImpl& iface)
{
}

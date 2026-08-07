// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include <carb/events/IEvents.h>
#include <carb/Framework.h>
#include <carb/settings/ISettings.h>

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxFoundation.h>
#include <private/omni/physx/IPhysxPrivate.h>
#include <omni/physx/IPhysxJoint.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physics/tensors/TensorApi.h>

#include <cstring>
#include <memory>

#include <omni/physx/PhysXRuntime.h>
#include "gpu/CudaCommon.h"
#include "GlobalsAreBad.h"
#include "SimulationBackend.h"

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

// Simulation backend lifetime notes
// ---------------------------------
// - An explicit init/shutdown path (tensorsInit()/tensorsShutdown())
//   lets us manage all Carbonite-dependent resources (event subscriptions, etc.).
// - This guarantees that, when the static runtime is shut down, we release those resources
//   *before* Carbonite tears down its core plugins.
// - With this ordering, the default static destruction of g_simBackend at module
//   unload becomes a no-op with respect to Carbonite and is safe.
// - This design preserves existing Kit behavior while allowing custom packaging (Python wheels)
//   to handle the lifetime of this plugin and the Carbonite framework explicitly, without
//   shutdown crashes, intentional leaks, or skipped destructors.
std::unique_ptr<omni::physx::tensors::SimulationBackend> g_simBackend;
bool g_tensorsStarted = false;

////////////////////////
// Reset sim data for the currently attached stage, or full reset if none attached.
//
// Note: getAttachedStage() returns 0 when more than one stage is attached
// (see IPhysxSimulation::getAttachedStage / getActiveStageId). In multi-stage
// scenarios (e.g. concurrent ovphysx instances) this path falls back to full
// reset(). For ovphysx usage, per-stage teardown is handled explicitly via
// TensorApi::resetStage(stageId) called from ovphysx_destroy_instance; this
// function serves the Kit single-stage path and is a safety net only.
void resetCurrentStage()
{
    if (g_simBackend && g_physxSimulation)
    {
        long stageId = g_physxSimulation->getAttachedStage();
        if (stageId > 0)
            g_simBackend->resetStage(stageId);
        else
            g_simBackend->reset();
    }
}

class SimulationEventListener : public carb::events::IEventListener
{
    void onEvent(carb::events::IEvent* e) override
    {
        int eventType = int(e->type);

        if (eventType == omni::physx::SimulationEvent::eStopped)
        {
            resetCurrentStage();
        }
    }

    size_t addRef() override
    {
        return ++mRefCount;
    }

    size_t release() override
    {
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
// Pre-step callbacks are driven by omni.physx events rather than IStageUpdate.
omni::physx::SubscriptionId g_physxPreStepSubscriptionId = omni::physx::kInvalidSubscriptionId;

} // end of anonymous namespace


SimulationBackend* GetSimulationBackend()
{
    return g_simBackend.get();
}

void PhysxPreStepCallback(float elapsedTime, void* userData)
{
    if (g_simBackend)
    {
        g_simBackend->prePhysicsUpdate();
    }
}

void onDetach(void* userData)
{
    resetCurrentStage();
}

// Shared init/shutdown driven by the owner of the static PhysX runtime.
void tensorsInit()
{
    if (g_tensorsStarted)
    {
        CARB_LOG_VERBOSE("omni.physx tensors tensorsInit() already ran, skipping duplicate");
        return;
    }

    CARB_LOG_INFO("omni.physx tensors tensorsInit() starting...");

    carb::Framework* framework = carb::getFramework();
    if (!framework)
    {
        CARB_LOG_ERROR("Failed to get Carbonite framework");
        return;
    }

    // Register default values for tensor-API settings. setDefaultBool
    // establishes the fallback; explicit user overrides (via omni.kit or
    // carb.settings at runtime) take precedence.
    carb::settings::ISettings* settings = framework->tryAcquireInterface<carb::settings::ISettings>();
    if (settings)
    {
        settings->setDefaultBool(kSettingRecursiveLeafPatternMatch, kDefaultRecursiveLeafPatternMatch);
    }

    g_physx = omni::physx::runtime::tryGetPhysxInterface();
    if (!g_physx)
    {
        CARB_LOG_ERROR("PhysX runtime interface is not started");
        return;
    }

    g_physxSimulation = omni::physx::runtime::tryGetPhysxSimulationInterface();
    if (!g_physxSimulation)
    {
        CARB_LOG_ERROR("PhysX Simulation runtime interface is not started");
        return;
    }

    g_physxPrivate = omni::physx::runtime::tryGetPhysxPrivateInterface();
    if (!g_physxPrivate)
    {
        CARB_LOG_ERROR("PhysX Private runtime interface is not started");
        return;
    }

    g_physxJoint = omni::physx::runtime::tryGetPhysxJointInterface();
    if (!g_physxJoint)
    {
        CARB_LOG_ERROR("PhysX Joint runtime interface is not started");
        return;
    }

    if (!g_simBackend)
    {
        g_simBackend = std::make_unique<omni::physx::tensors::SimulationBackend>();
    }

    g_simEventListener = new SimulationEventListener;
    g_simEventStream = g_physx->getSimulationEventStreamV2();
    g_simEventSubscription = g_simEventStream->createSubscriptionToPop(g_simEventListener, 0);

    // Pre-physics callbacks go through omni.physx events, avoiding a dependency on omni.usd.
    g_physxPreStepSubscriptionId = g_physx->subscribePhysicsOnStepEvents(true, 0, PhysxPreStepCallback, nullptr);
    if (g_physxPreStepSubscriptionId == omni::physx::kInvalidSubscriptionId)
    {
        CARB_LOG_ERROR("Failed to subscribe to PhysX pre-step events");
    }

    // Mark as successfully initialized only after all setup completes.
    // The backend is reached directly via GetSimulationBackend() from the
    // TensorApi entry points in this plugin (no registry indirection).
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

    g_physx = nullptr;
    g_physxSimulation = nullptr;
    g_physxPrivate = nullptr;
    g_physxJoint = nullptr;

    // Finally, destroy the simulation backend once all external references
    // (stage update nodes, PhysX callbacks) have been removed.
    if (g_simBackend)
    {
        g_simBackend.reset();
    }
}


// Tensor backend startup is driven by the owner of the static PhysX runtime.
// Creates the primary CUDA context when GPU mode is active, then runs
// tensorsInit(). Idempotent via g_tensorsStarted.
void tensorsPluginStartup()
{

    tensorsInit();
}

} // namespace tensors
} // namespace physx
} // namespace omni

// ---------------------------------------------------------------------------
// TensorApi function table, provided directly by the static PhysX runtime.
// ---------------------------------------------------------------------------
namespace
{
omni::physics::tensors::ISimulationView* CARB_ABI physxCreateSimulationView(long stageId)
{
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    return backend ? backend->createSimulationView(stageId) : nullptr;
}

void CARB_ABI physxResetTensors()
{
    if (omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend())
    {
        backend->reset();
    }
}

void CARB_ABI physxResetTensorsStage(long stageId)
{
    if (omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend())
    {
        backend->resetStage(stageId);
    }
}
} // anonymous namespace

void fillInterface(omni::physics::tensors::TensorApi& iface)
{
    memset(&iface, 0, sizeof(iface));

    iface.createSimulationView = physxCreateSimulationView;
    iface.reset = physxResetTensors;
    iface.resetStage = physxResetTensorsStage;
}

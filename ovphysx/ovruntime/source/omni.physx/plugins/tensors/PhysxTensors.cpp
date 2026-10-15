// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-3 AC-4
// @implements REQ-ATTACH-VIEWS-001
// @covers AC-1 AC-2

/**
 * @implements REQ-TENSOR-VIEW-001
 * @covers AC-2 AC-5 AC-8 AC-11
 */

// clang-format off
// clang-format on

#include <carb/events/IEvents.h>
#include <carb/dictionary/IDictionary.h>
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
#include "PublicTensorViews.h"
#include "RuntimeExecution.h"
#include "usdLoad/LoadUsd.h"

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

bool g_tensorsStarted = false;

class SimulationEventListener : public carb::events::IEventListener
{
    void onEvent(carb::events::IEvent* e) override
    {
        int eventType = int(e->type);

        if (eventType == omni::physx::SimulationEvent::eStopped ||
            eventType == omni::physx::SimulationEvent::ePhysicsObjectsReleased)
        {
            carb::dictionary::IDictionary* dictionary = carb::getCachedInterface<carb::dictionary::IDictionary>();
            const carb::dictionary::Item* identity = dictionary ? dictionary->getItem(e->payload, "attachHandle") : nullptr;
            const omni::physics::tensors::AttachHandle attach = identity ?
                static_cast<uint64_t>(dictionary->getAsInt64(identity)) : omni::physics::tensors::kNoAttach;
            if (OmniPhysX::getInstance().tensorBackend && attach != omni::physics::tensors::kNoAttach)
                OmniPhysX::getInstance().tensorBackend->resetStage(attach);
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

} // end of anonymous namespace


SimulationBackend* GetSimulationBackend()
{
    auto* runtime = OmniPhysX::getInstanceCheck();
    return runtime ? runtime->tensorBackend.get() : nullptr;
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

    if (!OmniPhysX::getInstance().tensorBackend)
    {
        OmniPhysX::getInstance().tensorBackend = std::make_shared<omni::physx::tensors::SimulationBackend>();
    }

    g_simEventListener = new SimulationEventListener;
    g_simEventStream = g_physx->getSimulationEventStreamV2();
    g_simEventSubscription = g_simEventStream->createSubscriptionToPop(g_simEventListener, 0);

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

    if (OmniPhysX::getInstance().tensorBackend)
    {
        OmniPhysX::getInstance().tensorBackend->shutdown();
    }

    g_physx = nullptr;
    g_physxSimulation = nullptr;
    g_physxPrivate = nullptr;
    g_physxJoint = nullptr;

    // Finally, destroy the simulation backend once all external references
    // (stage update nodes, PhysX callbacks) have been removed.
    if (OmniPhysX::getInstance().tensorBackend)
    {
        OmniPhysX::getInstance().tensorBackend.reset();
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
omni::physics::tensors::ISimulationView* CARB_ABI physxCreateSimulationView(
    omni::physics::tensors::AttachHandle attachHandle)
{
    omni::physx::RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return nullptr;
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    return backend ? omni::physx::tensors::publicViews::wrap(backend->createSimulationView(attachHandle), attachHandle) : nullptr;
}

bool CARB_ABI physxUpdateArticulationsKinematic(omni::physics::tensors::AttachHandle attachHandle)
{
    omni::physx::RuntimeExecutionScope operation(attachHandle);
    if (!operation)
        return false;
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    return backend ? backend->updateArticulationsKinematic(attachHandle) : false;
}

bool CARB_ABI physxResetTensorsStage(omni::physics::tensors::AttachHandle attachHandle)
{
    omni::physx::RuntimeExecutionScope execution(attachHandle);
    if (!execution || !omni::physx::usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle))
        return false;
    if (omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend())
    {
        backend->resetStage(attachHandle);
        return true;
    }
    return false;
}
} // anonymous namespace

void fillInterface(omni::physics::tensors::TensorApi& iface)
{
    memset(&iface, 0, sizeof(iface));

    iface.createSimulationView = physxCreateSimulationView;
    iface.updateArticulationsKinematic = physxUpdateArticulationsKinematic;
    iface.resetStage = physxResetTensorsStage;
}

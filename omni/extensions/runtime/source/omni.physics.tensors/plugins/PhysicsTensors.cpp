// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#define CARB_EXPORTS

#include <carb/Framework.h>
#include <carb/PluginUtils.h>

#include <omni/physics/tensors/TensorApi.h>
#include <omni/physics/tensors/ISimulationBackend.h>

#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>

#include <string>
#include <map>
#include <cctype>

using namespace pxr;

const struct carb::PluginImplDesc kPluginImpl = { "omni.physics.tensors.plugin",
                                                  "Tensor Interface for Physics Simulations", "NVIDIA",
                                                  carb::PluginHotReload::eDisabled, "dev" };

CARB_PLUGIN_IMPL(kPluginImpl, omni::physics::tensors::TensorApi, omni::physics::tensors::BackendRegistry)
CARB_PLUGIN_IMPL_NO_DEPS()

namespace omni
{
namespace physics
{
namespace tensors
{

class TensorRegistryManager
{
public:
    TensorRegistryManager()
    {

    }
    ~TensorRegistryManager()
    {

    }

    carb::tasking::MutexWrapper& getRegistryMutex()
    {
        return mRegistryMutex;
    }

    std::map<std::string, ISimulationBackend*>& getBackendMap()
    {
        return mBackendMap;
    }

    const std::map<std::string, ISimulationBackend*>& getBackendMap() const
    {
        return mBackendMap;
    }

private:
    carb::tasking::MutexWrapper mRegistryMutex;
    std::map<std::string, ISimulationBackend*> mBackendMap;

};

TensorRegistryManager* gRegistryManager = nullptr;

// private stuff
namespace
{

std::string toLower(const std::string& s)
{
    std::string r = s;
    for (auto& c : r)
    {
        c = std::string::value_type(std::tolower(c));
    }
    return r;
}

} // end of anonymous namespace

bool CARB_ABI RegisterBackend(const char* name, ISimulationBackend* backend)
{
    if (!name || !backend || !gRegistryManager)
    {
        return false;
    }

    std::string id = toLower(name);

    std::lock_guard<carb::tasking::MutexWrapper> lock(gRegistryManager->getRegistryMutex());

    if (gRegistryManager->getBackendMap().find(id) != gRegistryManager->getBackendMap().end())
    {
        CARB_LOG_ERROR("Duplicate name '%s' in simulation backend registry", name);
        return false;
    }

    gRegistryManager->getBackendMap()[id] = backend;

    return true;
}

void CARB_ABI UnregisterBackend(const char* name)
{
    if (!name)
    {
        return;
    }

    std::string id = toLower(name);

    std::lock_guard<carb::tasking::MutexWrapper> lock(gRegistryManager->getRegistryMutex());

    gRegistryManager->getBackendMap().erase(id);
}

ISimulationView* CARB_ABI CreateSimulationView(long stageId)
{
    // TODO: Once we have multiple backends available, we'll need a mechanism to infer
    // which backend should be used.  Possibly with user hint, but ideally this should
    // be automatic.
    const char* backendName = "physx";

    std::string backendId = toLower(backendName);

    std::lock_guard<carb::tasking::MutexWrapper> lock(gRegistryManager->getRegistryMutex());

    auto backendIt = gRegistryManager->getBackendMap().find(backendId);
    if (backendIt == gRegistryManager->getBackendMap().end())
    {
        CARB_LOG_ERROR("Failed to find simulation backend '%s'", backendName);
        return nullptr;
    }

    ISimulationBackend* backend = backendIt->second;

    return backend->createSimulationView(stageId);
}

void CARB_ABI Reset()
{
    // TODO: Once we have multiple backends available, we'll need a mechanism to infer
    // which backend should be used.  Possibly with user hint, but ideally this should
    // be automatic.
    const char* backendName = "physx";

    std::string backendId = toLower(backendName);

    std::lock_guard<carb::tasking::MutexWrapper> lock(gRegistryManager->getRegistryMutex());

    auto backendIt = gRegistryManager->getBackendMap().find(backendId);
    if (backendIt == gRegistryManager->getBackendMap().end())
    {
        CARB_LOG_ERROR("Failed to find simulation backend '%s'", backendName);
        return;
    }

    ISimulationBackend* backend = backendIt->second;

    backend->reset();
}

}
}
}

CARB_EXPORT void carbOnPluginStartup()
{
    omni::physics::tensors::gRegistryManager = new omni::physics::tensors::TensorRegistryManager();
}

CARB_EXPORT void carbOnPluginShutdown()
{
    if (omni::physics::tensors::gRegistryManager)
    {
        delete omni::physics::tensors::gRegistryManager;
        omni::physics::tensors::gRegistryManager = nullptr;
    }
}

void fillInterface(omni::physics::tensors::TensorApi& iface)
{
    using namespace omni::physics::tensors;

    memset(&iface, 0, sizeof(iface));

    iface.createSimulationView = CreateSimulationView;
    iface.reset = Reset;
}

void fillInterface(omni::physics::tensors::BackendRegistry& iface)
{
    using namespace omni::physics::tensors;

    memset(&iface, 0, sizeof(iface));

    iface.registerBackend = RegisterBackend;
    iface.unregisterBackend = UnregisterBackend;
}

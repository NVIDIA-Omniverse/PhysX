// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "UsdPCH.h"

#define CARB_EXPORTS

#include <private/omni/physx/IPhysxGraph.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxCookingService.h>

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/tasking/ITasking.h>

#include <omni/graph/core/NodeTypeRegistrar.h>
#include <omni/graph/core/OgnHelpers.h>
#include <omni/graph/core/iComputeGraph.h>

#include "GraphShared.h"
#include "SceneQueryShared.h"

const struct carb::PluginImplDesc pluginDesc = { "omni.physx.graph.plugin", "NVIDIA PhysX OmniGraph Nodes", "NVIDIA",
                                                 carb::PluginHotReload::eDisabled, "dev" };

CARB_PLUGIN_IMPL(pluginDesc, omni::physx::IPhysxGraph)
CARB_PLUGIN_IMPL_DEPS(  omni::graph::core::IGraphRegistry,
                        carb::tasking::ITasking,
                        omni::physx::IPhysxSceneQuery,
                        omni::physx::IPhysxCookingService,
                        omni::physx::IPhysxSimulation)

DECLARE_OGN_NODES()

// carbonite interface for this plugin (may contain multiple compute nodes)
void fillInterface(omni::physx::IPhysxGraph& iface)
{
    iface = {};
}

CARB_EXPORT void carbOnPluginStartup()
{
    INITIALIZE_OGN_NODES()

    carb::Framework* framework = carb::getFramework();
    omni::physx::graph::setPhysXSceneQuery(carb::getCachedInterface<omni::physx::IPhysxSceneQuery>());
}

CARB_EXPORT void carbOnPluginShutdown()
{
    // During an abnormal process exit (e.g. the renderer calls exit(), which runs carb's
    // OnExitProcess -> unloadAllPlugins) plugin unload order is unspecified, so omni.graph.core
    // (which provides IGraphRegistry) may already be gone by the time we get here.
    // RELEASE_OGN_NODES() deregisters our OmniGraph node types through the cached IGraphRegistry;
    // if that provider is unloaded the cached pointer is stale and the deregistration call faults
    // (SIGSEGV / SEGV_ACCERR, NvBugs 6224712). tryAcquireExistingInterface() only succeeds while
    // the provider plugin is still started (it never tries to reload it) and consults framework
    // state rather than the stale cached pointer, so use it to gate the deregistration. On a normal
    // Kit shutdown the provider is alive and this behaves exactly as before. (This narrows, but
    // cannot fully close, a race where the provider is reported started yet already unmapped.)
    carb::Framework* framework = carb::getFramework();
    if (framework && framework->tryAcquireExistingInterface<omni::graph::core::IGraphRegistry>())
    {
        RELEASE_OGN_NODES()
    }

    omni::physx::graph::setPhysXSceneQuery(nullptr);
    /*

    // note: aquired interfaces are not released here as this causes warning messages "...already been released"

    */
}

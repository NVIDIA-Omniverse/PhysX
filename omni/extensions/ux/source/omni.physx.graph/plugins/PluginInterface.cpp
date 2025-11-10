// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

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
    RELEASE_OGN_NODES()

    omni::physx::graph::setPhysXSceneQuery(nullptr);
    /*

    // note: aquired interfaces are not released here as this causes warning messages "...already been released"

    */
}

// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <private/omni/physx/IPhysxGraph.h>

#include <carb/BindingsPythonUtils.h>

#include <memory>
#include <string>
#include <vector>

CARB_BINDINGS("carb.physx.graph.python")

namespace
{

PYBIND11_MODULE(_physxGraph, m)
{
    using namespace carb;
    using namespace omni::physx;

    m.doc() = "pybind11 carb.physx.graph bindings";

    py::class_<IPhysxGraph> physxGraph = defineInterfaceClass<IPhysxGraph>
    (
        m, "IPhysxGraph", "acquire_physx_graph_interface", "release_physx_graph_interface"
    );
    m.def("release_physx_graph_interface_scripting", [](IPhysxGraph* iface)
    {
        carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface); // OM-60917
    });
}
} // namespace

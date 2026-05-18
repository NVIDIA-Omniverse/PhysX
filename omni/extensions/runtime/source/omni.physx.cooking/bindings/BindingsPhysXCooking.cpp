// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"


#include <common/utilities/Utilities.h>
#include <carb/BindingsPythonUtils.h>

#include <private/omni/physx/IPhysxCookingServicePrivate.h>

CARB_BINDINGS("omni.physx.cooking.python")


namespace
{
PYBIND11_MODULE(_physxCooking, m)
{
    using namespace carb;
    using namespace omni::physx;

    const char* docString;

    m.doc() = "pybind11 omni.physx.cooking bindings";

    auto iface = defineInterfaceClass<omni::physx::IPhysxCookingServicePrivate>(
        m, "PhysxCookingServicePrivate", "acquire_physx_cooking_service_private", "release_physx_cooking_service_private");
    docString = R"(
        Returns true if the process is running on an OVC node, false otherwise.
    )";
    // deprecated
    iface.def("is_OVC_node", wrapInterfaceFunction(&IPhysxCookingServicePrivate::isOVCNodeDeprecated), docString);

    m.def("release_physx_cooking_service_private_scripting", [](omni::physx::IPhysxCookingServicePrivate* iface)
    {
        carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface); // OM-60917
    });

}
} // namespace

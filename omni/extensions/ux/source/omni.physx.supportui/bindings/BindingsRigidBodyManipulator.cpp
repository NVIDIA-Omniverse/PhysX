// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include <private/omni/physx/IPhysxSupportUiRigidBodyManipulator.h>

#include <omni/ui/bind/BindUtils.h>
#include <carb/BindingsPythonUtils.h>

void RigidBodyManipulatorBindings(pybind11::module& m)
{
    using namespace pybind11;
    using namespace carb;
    using namespace omni::ui;
    using namespace omni::physx;

    const char* docString;

    auto rbManipulator = defineInterfaceClass<IPhysxSupportUiRigidBodyManipulator>(
        m, "SupportUiRigidBodyManipulator", "acquire_rigid_body_manipulator", "release_rigid_body_manipulator");

    docString = R"(
        Moves an object using physics force.

        Args:
            'path': int2 - Usd path to the manipulated prim.
            'delta_translation': float3 - translation delta.
            'lock_rotation': bool - allow/disallow rotation while translating.
            'lock_translation': bool - allow/disallow translating on other axes while translating.
        )";
    rbManipulator.def("move", wrapInterfaceFunction(&IPhysxSupportUiRigidBodyManipulator::move), docString,
                      py::arg("path"), py::arg("delta_translation"), py::arg("lock_rotation"), py::arg("lock_translation"));

    docString = R"(
        Rotates an object using physics force.

        Args:
            'path': int2 - Usd path to the manipulated prim.
            'pivot_world_pos': float3 - world pivot position to rotate around.
            'delta_rotation': float4 - target rotation quaternion.
            'lock_rotation': bool - allow/disallow rotation on other axes while rotating.
            'lock_translation': bool - allow/disallow translating while rotating.
        )";
    rbManipulator.def("rotate", wrapInterfaceFunction(&IPhysxSupportUiRigidBodyManipulator::rotate), docString,
                      py::arg("path"), py::arg("pivot_world_pos"), py::arg("delta_rotation"), py::arg("lock_rotation"), py::arg("lock_translation"));

    docString = R"(
        Must be called when manipulation begins, before calling move() or rotate().

        Args:
            'path': int2 - Usd path to the manipulated prim.
        )";
    rbManipulator.def("manipulation_began",
                      wrapInterfaceFunction(&IPhysxSupportUiRigidBodyManipulator::manipulationBegan), docString,
                      py::arg("path"));

    docString = R"(
        Must be called when manipulation ends, always in pairs with manipulation_began().

        Args:
            'path': int2 - Usd path to the manipulated prim.
        )";
    rbManipulator.def("manipulation_ended",
                      wrapInterfaceFunction(&IPhysxSupportUiRigidBodyManipulator::manipulationEnded), docString,
                      py::arg("path"));
}

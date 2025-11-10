// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"
#include <carb/BindingsPythonUtils.h>
#include <pybind11/pybind11.h>
#include <private/omni/physx/IPhysxUIPrivate.h>
#include <carb/Framework.h>
using namespace pybind11;

void wrapPhysXUIOmniUISceneOverlay(module& m)
{
    class_<omni::ui::scene::PhysXUIOmniUISceneOverlay, omni::ui::scene::Manipulator,
           std::shared_ptr<omni::ui::scene::PhysXUIOmniUISceneOverlay>>(m, "PhysXUIOmniUISceneOverlay", "")
        .def(init([](kwargs kwargs) {
                 auto pvt = carb::getFramework()->acquireInterface<omni::physx::ui::IPhysxUIPrivate>();                 
                 std::shared_ptr<omni::ui::scene::PhysXUIOmniUISceneOverlay> ptr = pvt->createPhysXUIOmniUISceneOverlay();
                 carb::getFramework()->releaseInterface<omni::physx::ui::IPhysxUIPrivate>(pvt);
                 return ptr;
             }),
             "")
        .def(
            "set_enable_picking_fn",
            [](omni::ui::scene::PhysXUIOmniUISceneOverlay& self, std::function<void(bool)> fn) {
                self.setEnablePickingFn(carb::wrapPythonCallback(std::move(fn)));
            },
            "");
}

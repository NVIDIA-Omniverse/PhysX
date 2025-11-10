// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

// Physics includes
#include <omni/physics/ui/IUsdPhysicsUI.h>
#include <private/omni/physics/ui/IUsdPhysicsUIPrivate.h>
#include <omni/physics/IUsdPhysicsSettings.h>
#include <omni/ui/scene/Math.h> // Matrix44
#include <omni/ui/scene/SceneView.h> // Matrix44

// Carbonite Includes
#include <carb/BindingsPythonUtils.h>

CARB_BINDINGS("carb.usdphysicsui.python")

namespace
{
    
#define ADD_SETTING(attr_name, path) \
    m.attr(attr_name) = py::str(path); \
    m.attr(attr_name "_DEFAULT") = py::str(path##Default);

void registerUSDPhysicsUI(pybind11::module& m)
{

    using namespace carb;
    using namespace omni::physics::ui;
    using namespace omni::physics;

    m.doc() = "pybind11 carb.usdphysicsui bindings";

    py::class_<IUsdPhysicsUI> usdPhysicsUI = defineInterfaceClass<IUsdPhysicsUI>(
        m, "IUsdPhysicsUI", "acquire_usdphysics_ui_interface", "release_usdphysics_ui_interface");

    m.def("release_usdphysics_ui_interface_scripting", [](IUsdPhysicsUI* iface) {
        carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface); // OM-60917
    });

    ADD_SETTING("SETTING_DISPLAY_JOINTS", kSettingDisplayJoints);

    usdPhysicsUI.def("block_usd_notice_handler", wrapInterfaceFunction(&IUsdPhysicsUI::blockUsdNoticeHandler));
    usdPhysicsUI.def("is_usd_notice_handler_enabled", wrapInterfaceFunction(&IUsdPhysicsUI::isUsdNoticeHandlerEnabled));
}

void registerUSDPhysicsUIPrivate(pybind11::module& m)
{
    using namespace carb;
    using namespace omni::physics::ui;
    using namespace omni::physics;

    m.doc() = "pybind11 carb.usdphysicsuiprivate bindings";

    py::class_<IUsdPhysicsUIPrivate> usdPhysicsUIPrivate = defineInterfaceClass<IUsdPhysicsUIPrivate>(
        m, "IUsdPhysicsUIPrivate", "acquire_usdphysics_ui_private_interface", "release_usdphysics_ui_private_interface");

    m.def("release_usdphysics_ui_private_interface_scripting", [](IUsdPhysicsUIPrivate* iface) {
        carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface); // OM-60917
    });

    usdPhysicsUIPrivate.def(
        "private_draw_immediate_mode_viewport_overlays",
        [](IUsdPhysicsUIPrivate* iface, std::array<double, 16> view, std::array<double, 16> proj,
           float screenPositionX, float screenPositionY, float computedContentWidth, float computedContentHeight, float dpiScale,
           std::function<void(bool)> enablePickingFunction) {
            iface->privateDrawImmediateModeViewportOverlays(
                view.data(), proj.data(), screenPositionX, screenPositionY, computedContentWidth, computedContentHeight,
                dpiScale,
                [](bool enable, void* userData) { (*static_cast<std::function<void(bool)>*>(userData))(enable); },
                &enablePickingFunction);
        });
}

PYBIND11_MODULE(_usdphysicsUI, m)
{
    registerUSDPhysicsUI(m);
    registerUSDPhysicsUIPrivate(m);
}
} // namespace

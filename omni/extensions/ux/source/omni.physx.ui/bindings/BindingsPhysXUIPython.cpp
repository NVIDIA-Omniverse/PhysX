// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <omni/physx/IPhysxUI.h>
#include <private/omni/physx/IPhysxUIPrivate.h>

#include "../plugins/InputManager.h"

#include <carb/BindingsPythonUtils.h>

#include <memory>
#include <string>
#include <vector>

CARB_BINDINGS("carb.physxui.python")

void wrapPhysXUIOmniUISceneOverlay(pybind11::module& m);

namespace
{

#define VEHICLE_DEBUG_ID_SUSPENSION "Suspension"

template <typename T = int, typename S = uint32_t>
struct ResultBuffer
{
    ~ResultBuffer()
    {
        if (ptr)
        {
            free(ptr);
            ptr = nullptr;
        }
        size = 0;
    }

    static void* allocate(size_t numBytes)
    {
        return (void*)malloc(numBytes);
    }

    T* ptr = nullptr;
    S size = 0;
};

PYBIND11_MODULE(_physxUI, m)
{
    using namespace carb;
    using namespace omni::physx::ui;

    const char* docString;

    m.doc() = "pybind11 carb.physxui bindings";

    py::class_<IPhysxUI> physxUI = defineInterfaceClass<IPhysxUI>(m, "IPhysxUI", "acquire_physx_ui_interface", "release_physx_ui_interface");

    m.def("release_physx_ui_interface_scripting", [](IPhysxUI* iface)
    {
        carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface); // OM-60917
    });

    docString = R"(
        DEPRECATED
    )";
    physxUI.def("update_gizmo_transform", []
    (const IPhysxUI* self, const char* path, const carb::Float3& pos, const carb::Float4& rot)
        {
            CARB_LOG_WARN_ONCE("Deprecated: update_gizmo_transform is deprecated and does nothing.");
            return false;
        }
    );
    physxUI.def("update", wrapInterfaceFunction(&IPhysxUI::update));
    physxUI.def("set_visualization_distance", wrapInterfaceFunction(&IPhysxUI::setVisualizationDistance));
    physxUI.def("enable_debug_visualization", wrapInterfaceFunction(&IPhysxUI::enableDebugVisualization));
    physxUI.def("set_tendon_visualization_filter", wrapInterfaceFunction(&IPhysxUI::setTendonVisualizationFilter));
    physxUI.def("set_collision_mesh_type", [](const IPhysxUI* self, const char* type) { self->setCollisionMeshType(type); });
    physxUI.def("enable_collision_mesh_visualization", wrapInterfaceFunction(&IPhysxUI::enableCollisionMeshVisualization));
    physxUI.def("explode_view_distance", wrapInterfaceFunction(&IPhysxUI::explodeViewDistance));
    physxUI.def("block_usd_notice_handler", wrapInterfaceFunction(&IPhysxUI::blockUsdNoticeHandler));
    physxUI.def("is_usd_notice_handler_enabled", wrapInterfaceFunction(&IPhysxUI::isUsdNoticeHandlerEnabled));
    physxUI.def("enable_redraw_optimizations", wrapInterfaceFunction(&IPhysxUI::enableRedrawOptimizations));


    docString = R"(
        Trigger selection of spatial tendon attachment session layer helper geometry.

        The selection is queued in the spatial tendon visualization module, and the selection will fail
        quietly if the provided inputs are invalid, i.e. do not point to a valid attachment in the stage.

        Args:
            link_body_path: Sdf.Path to rigid body articulation link that the attachment API is applied to.
            instance_name: String instance name of attachment API (specific API type is not relevant)
    )";
    physxUI.def("select_spatial_tendon_attachment_helper",
        [](const IPhysxUI* self, const char* linkBodyPath, const char* instanceName)
        {
            self->selectSpatialTendonAttachmentHelper(pxr::SdfPath(linkBodyPath), pxr::TfToken(instanceName));
        },
        docString, py::arg("link_body_path"), py::arg("instance_name")
    );

    docString = R"(
        Toggle individual vehicle debug visualization features.

        Args:
            parameter - Debug visualization feature string identifier, can be one of the following:
                {')" VEHICLE_DEBUG_ID_SUSPENSION R"('}
            enable - Bool to enable/disable the feature.
    )";
    physxUI.def("set_vehicle_visualization",
        [](const IPhysxUI* self, const char* parameter, bool enable)
        {
            if (strstr(parameter, VEHICLE_DEBUG_ID_SUSPENSION))
            {
                self->setVehicleVisualization(PhysXVehicleVisualizationParameter::eSuspension, enable);
            }
            else
            {
                CARB_LOG_ERROR("set_vehicle_visualization: unknown identifier \"%s\".\n", parameter);
            }
        },
        docString, py::arg("parameter"), py::arg("enable")
    );

    docString = R"(
        Get state of vehicle debug visualization features.

        Args:
            parameter - Debug visualization feature string identifier (see set_vehicle_visualization
            for the supported identifiers)

        Returns:
            bool: True if the feature is set for visualization, else false.
    )";
    physxUI.def("get_vehicle_visualization",
        [](const IPhysxUI* self, const char* parameter)
        {
            if (strstr(parameter, VEHICLE_DEBUG_ID_SUSPENSION))
            {
                return self->getVehicleVisualization(PhysXVehicleVisualizationParameter::eSuspension);
            }
            else
            {
                CARB_LOG_ERROR("get_vehicle_visualization: unknown identifier \"%s\".\n", parameter);
                return false;
            }
        },
        docString, py::arg("parameter")
    );

    docString = R"(
        DEPRECATED
    )";
    physxUI.def("set_attachment_visualization",
        [](const IPhysxUI* self, const char* attachmentPath, bool enable)
        {
            CARB_LOG_WARN_ONCE("Deprecated: set_attachment_visualization is deprecated and does nothing.");
        },
        docString, py::arg("attachment_path"), py::arg("enable")
    );

    docString = R"(
        DEPRECATED
    )";
    physxUI.def("hide_attached_actor",
        [](const IPhysxUI* self, const char* attachmentPath, const char* actorPath, bool hide)
        {
            CARB_LOG_WARN_ONCE("Deprecated: hide_attached_actor is deprecated and does nothing.");
        },
        docString, py::arg("attachment_path"), py::arg("actor_path"), py::arg("hide")
    );

    docString = R"(
        DEPRECATED:
        Get all attachments that are associated with a given primitive at path.
        Args:
            prim_path: Sdf.Path to primitive for which associated attachments are queried.
        Returns:
            dict with "attachments" entry containing a list of all attachment paths that are associated with the prim_path.
    )";
    physxUI.def("get_attachments",
        [](const IPhysxUI* self, const char* primPath)
        {
            py::list list;

            ResultBuffer<size_t> resultSizes;
            ResultBuffer<uint8_t, size_t> resultData;
            self->getAttachments(resultSizes.ptr, resultSizes.size, resultData.ptr, resultData.size, pxr::SdfPath(primPath), ResultBuffer<>::allocate);
            if (resultSizes.ptr)
            {
                size_t byteOffset = 0;
                for (uint32_t i = 0; i < resultSizes.size; ++i)
                {
                    size_t strByteSize = resultSizes.ptr[i];
                    py::str string((const char*)(resultData.ptr + byteOffset), strByteSize - 1);
                    list.append(string);
                    byteOffset += strByteSize;
                }
            }
            py::dict dict;
            dict["attachments"] = list;
            return dict;
        },
        docString, py::arg("prim_path")
    );

    docString = R"(
        Refresh deformable attachment.
        Args:
            attachment_path: Sdf.Path to attachment primitive.
    )";
    physxUI.def(
        "refresh_attachment",
        [](const IPhysxUI* self, const char* attachmentPath) { self->refreshAttachment(pxr::SdfPath(attachmentPath)); },
        docString, py::arg("attachment_path"));

    docString = R"(
        Set camera world position. Used by DebugVisualizer of colliders.
        Args:
            pos: Camera world position.
    )";
    physxUI.def(
        "set_camera_pos",
        [](const IPhysxUI* self, const carb::Float3& pos) { self->setCameraPos(pos); },
        docString, py::arg("pos"));

    py::class_<IPhysxUIPrivate> physxUIPrivate = defineInterfaceClass<IPhysxUIPrivate>(m, "IPhysxUIPrivate", "acquire_physx_ui_private_interface", "release_physx_ui_private_interface");

    m.def("release_physx_ui_private_interface_scripting", [](IPhysxUIPrivate* iface) {
        carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface); // OM-60917
    });

    docString = R"(
        Registers a keyboard action mapping. The action will be active only during simulation mode in first person mode and conflicting actions would be automatically disabled.

        Args:
            action_name: Name of the action.
            input: Keyboard input to activate the action with.
            modifiers: Keyboard modifier.
    )";
    physxUIPrivate.def(
        "register_keyboard_action",
        [](const IPhysxUIPrivate* self, const char* actionName, carb::input::KeyboardInput input,
           carb::input::KeyboardModifierFlags modifiers) {
            self->getInputManager().registerAction(actionName, input, modifiers);
        }, docString, py::arg("actionName"), py::arg("input"), py::arg("modifiers"));

    docString = R"(
        Registers a gamepad action mapping. The action will be active only during simulation mode in first person mode and conflicting actions would be automatically disabled.

        Args:
            action_name: Name of the action.
            input: Gamepad input to activate the action with.
            index: Gamepad index.
    )";
    physxUIPrivate.def(
        "register_gamepad_action",
        [](const IPhysxUIPrivate* self, const char* actionName, carb::input::GamepadInput input, size_t gamepad_index) {
            self->getInputManager().registerAction(actionName, input, gamepad_index);
        }, docString, py::arg("actionName"), py::arg("input"), py::arg("gamepad_index") = 0);

    docString = R"(
        Unregisters an action mapping registered with either register_keyboard_action or register_gamepad_action.

        Args:
            action_name: Name of the action.
    )";
    physxUIPrivate.def(
        "unregister_action",
        [](const IPhysxUIPrivate* self, const char* actionName) {
            self->getInputManager().unregisterAction(actionName);
        }, docString, py::arg("actionName"));

    docString = R"(
        Unregister and clear all added actions from the input manager. Enables all disabled conflicting actions.
    )";
    physxUIPrivate.def(
        "clear_input_manager",
        [](const IPhysxUIPrivate* self) {
        self->getInputManager().clear();
        }, docString);

    docString = R"(
        Performs test of Debug Visualization's internal structures.
        Each phase tests specific stuff.
            Phase 0: initial camera position.
            Phase 1: far camera position with completely boxed debug visualization with either AABB or box/bounding box.
            Phase 2: midpoint where some objects are replaced with AABB and some are not.
    )";
    physxUIPrivate.def(
        "test_debug_vis_internal_state",
        [](const IPhysxUIPrivate* self, const int phase) { return self->testDebugVisInternalState(phase); }, docString,
        py::arg("phase"));

    wrapPhysXUIOmniUISceneOverlay(m);
}
} // namespace

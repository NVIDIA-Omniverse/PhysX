// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"
#include <omni/ui/bind/BindUtils.h>

#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <carb/BindingsPythonUtils.h>
#include <private/omni/physx/IPhysxSupportUiPrivate.h>

using namespace pybind11;
DISABLE_PYBIND11_DYNAMIC_CAST(carb::events::IEventStream)

static void wrapSupportUiPrivateInterface(module& m)
{
    using namespace pybind11;
    const char* docString;
    auto supportPrivate = carb::defineInterfaceClass<omni::physx::IPhysxSupportUiPrivate>(
        m, "IPhysxSupportUiPrivate", "acquire_physx_supportui_private_interface",
        "release_physx_supportui_private_interface");

    m.def("release_physx_supportui_private_interface_scripting", [](omni::physx::IPhysxSupportUiPrivate* iface) {
        carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface);
    });
    docString = R"(
        Gets the state of the Inspector (returns PhysXInspectorModelState).
        )";
    supportPrivate.def("get_inspector_state",
                       carb::wrapInterfaceFunction(&omni::physx::IPhysxSupportUiPrivate::getPhysXInspectorState),
                       docString);
    docString = R"(
        Gets the event stream of the Inspector
        )";
    supportPrivate.def("get_inspector_event_stream",
                       carb::wrapInterfaceFunction(&omni::physx::IPhysxSupportUiPrivate::getPhysXInspectorEventStream),
                       docString);
    docString = R"(
        Step inspector simulation
        )";
    supportPrivate.def("step_inspector_simulation",
                       carb::wrapInterfaceFunction(&omni::physx::IPhysxSupportUiPrivate::stepPhysXInspectorSimulation),
                       docString);
    docString = R"()";
    supportPrivate.def(
        "reset_inspector_to_authoring_start",
        carb::wrapInterfaceFunction(&omni::physx::IPhysxSupportUiPrivate::resetPhysXInspectorToAuthoringStart),
        docString);
    docString = R"()";
    supportPrivate.def(
        "enable_inspector_authoring_mode",
        carb::wrapInterfaceFunction(&omni::physx::IPhysxSupportUiPrivate::enablePhysXInspectorAuthoringMode), docString);
    docString = R"(
        Enables USD Notice handler
        )";
    supportPrivate.def("enable_notice_handler",
                       carb::wrapInterfaceFunction(&omni::physx::IPhysxSupportUiPrivate::enableNoticeHandler), docString);
    docString = R"(
        Refresh all inspector models structure (rebuilds joints hierarchy)
        )";
    supportPrivate.def(
        "refresh_all_inspector_models_structure",
        carb::wrapInterfaceFunction(&omni::physx::IPhysxSupportUiPrivate::refreshAllInspectorModelsStructure), docString);
    docString = R"(
        Refresh all inspector models values (does not rebuild joints hierarchy)
        )";
    supportPrivate.def(
        "refresh_all_inspector_models_values",
        carb::wrapInterfaceFunction(&omni::physx::IPhysxSupportUiPrivate::refreshAllInspectorModelsValues), docString);
    docString = R"()";
    supportPrivate.def(
        "commit_authoring_state",
        carb::wrapInterfaceFunction(&omni::physx::IPhysxSupportUiPrivate::commitPhysXInspectorAuthoringState), docString);
}

static void wrapPhysXInspectorOmniUISceneOverlay(module& m)
{
    using namespace pybind11;
    class_<omni::ui::scene::PhysXInspectorOverlay, omni::ui::scene::Manipulator,
           std::shared_ptr<omni::ui::scene::PhysXInspectorOverlay>>(m, "PhysxInspectorOverlay", "")
        .def(init([](kwargs kwargs) {
                 auto pvt = carb::getCachedInterface<omni::physx::IPhysxSupportUiPrivate>();
                 if (pvt == nullptr)
                 {
                     carb::resetCachedInterface<omni::physx::IPhysxSupportUiPrivate>();
                     pvt = carb::getCachedInterface<omni::physx::IPhysxSupportUiPrivate>();
                 }
                 std::shared_ptr<omni::ui::scene::PhysXInspectorOverlay> ptr = pvt->createPhysXInspectorOverlay();
                 carb::getFramework()->releaseInterface<omni::physx::IPhysxSupportUiPrivate>(pvt);
                 return ptr;
             }),
             "");
}

static void wrapPhysXInspectorPanel(module& m)
{
    using namespace pybind11;
    class_<omni::ui::PhysXInspectorWidget, omni::ui::Widget, std::shared_ptr<omni::ui::PhysXInspectorWidget>>(
        m, "PhysXInspectorWidget", "")
        .def(init([](kwargs kwargs) {
                 auto pvt = carb::getCachedInterface<omni::physx::IPhysxSupportUiPrivate>();
                 std::shared_ptr<omni::ui::PhysXInspectorWidget> ptr = pvt->createPhysXInspectorWidget();
                 carb::getFramework()->releaseInterface<omni::physx::IPhysxSupportUiPrivate>(pvt);
                 return ptr;
             }),
             "")
        .def_property("model", &omni::ui::PhysXInspectorWidget::getModel, &omni::ui::PhysXInspectorWidget::setModel, "")
        .def_property("inspector_type", &omni::ui::PhysXInspectorWidget::getInspectorType,
                      &omni::ui::PhysXInspectorWidget::setInspectorType, "");
}

static void wrapPhysXInspectorModel(module& m)
{
    using namespace pybind11;
    using namespace omni::ui;
    using namespace omni::physx;
    enum_<PhysXInspectorModel::State>(m, "PhysXInspectorModelState")
        .value("DISABLED", PhysXInspectorModel::State::eDisabled)
        .value("AUTHORING", PhysXInspectorModel::State::eAuthoring)
        .value("RUNNING_SIMULATION", PhysXInspectorModel::State::eRunningSimulation);

    enum_<PhysXInspectorModel::InspectorType>(m, "PhysXInspectorModelInspectorType", R"(Inspection Type)")
        .value("INSPECTOR_TYPE_JOINTS_LIST", PhysXInspectorModel::InspectorType::eJointsList,
               R"(PhysXInspectorModel::InspectorType::eJointsList)")
        .value("INSPECTOR_TYPE_JOINTS_BODIES_HIERARCHY", PhysXInspectorModel::InspectorType::eJointsBodiesHierarchy,
               R"(PhysXInspectorModel::InspectorType::eJointsBodiesHierarchy)");

    enum_<PhysXInspectorModel::DataShapeType>(m, "PhysXInspectorModelDataShapeType", R"(Data Shape Type)")
        .value("FLAT", PhysXInspectorModel::DataShapeType::eFlat, R"(PhysXInspectorModel::DataShapeType::eFlat)")
        .value("HIERARCHICAL", PhysXInspectorModel::DataShapeType::eHierarchical,
               R"(PhysXInspectorModel::DataShapeType::eHierarchical)");

    enum_<PhysXInspectorModel::ControlType>(m, "PhysXInspectorModelControlType", R"(Control Type)")
        .value("AUTOMATIC", PhysXInspectorModel::ControlType::eAutomatic,
               R"(PhysXInspectorModel::ControlType::eAutomatic)")
        .value("JOINT_STATE", PhysXInspectorModel::ControlType::eJointState,
               R"(PhysXInspectorModel::ControlType::eJointState)")
        .value("JOINT_DRIVE", PhysXInspectorModel::ControlType::eJointDrive,
               R"(PhysXInspectorModel::ControlType::eJointDrive)")
        .value("JOINT_VELOCITY", PhysXInspectorModel::ControlType::eJointVelocity,
               R"(PhysXInspectorModel::ControlType::eJointVelocity)");

    enum_<PhysXInspectorModel::PropertyType>(m, "PhysXInspectorModelPropertyType", R"(Property Type)")
        .value("SHOW_LIMITS", PhysXInspectorModel::PropertyType::eShowLimits,
               R"(PhysXInspectorModel::PropertyType::eShowLimits)")
        .value("SHOW_GAINS", PhysXInspectorModel::PropertyType::eShowGains,
               R"(PhysXInspectorModel::PropertyType::eShowGains)");

    class_<PhysXInspectorModel, AbstractItemModel, std::shared_ptr<PhysXInspectorModel>>(m, "PhysXInspectorModel", "")
        .def(init([](std::vector<std::string> selection) {
                 auto supportUIPvt = carb::getCachedInterface<omni::physx::IPhysxSupportUiPrivate>();
                 pxr::SdfPathVector paths;
                 for (auto& s : selection)
                 {
                     paths.push_back(pxr::SdfPath(s));
                 }
                 std::shared_ptr<PhysXInspectorModel> ptr = supportUIPvt->createPhysXInspectorModel(paths);
                 carb::getFramework()->releaseInterface<omni::physx::IPhysxSupportUiPrivate>(supportUIPvt);
                 return ptr;
             }),
             "")
        .def("get_items_matching_paths", &PhysXInspectorModel::getItemsMatchingPaths, arg("paths"),
             return_value_policy::reference, "")
        .def("set_inspector_type", &PhysXInspectorModel::setInspectorType, "")
        .def("get_inspector_type", &PhysXInspectorModel::getInspectorType, "")
        .def("select_all_connected_body_joints", &PhysXInspectorModel::selectAllConnectedBodyJoints, "")
        .def("select_all_connected_body_shapes", &PhysXInspectorModel::selectAllConnectedBodyShapes, "")
        .def("select_all_connected_joint_shapes", &PhysXInspectorModel::selectAllConnectedJointShapes, "")
        .def("select_all_connected_links", &PhysXInspectorModel::selectAllConnectedLinks, "")
        .def("get_show_masses_and_inertia_model", &PhysXInspectorModel::getShowMassesAndInertiaModel, "")
        .def("get_enable_gravity_model", &PhysXInspectorModel::getEnableGravity, "")
        .def("get_enable_quasi_static_mode_model", &PhysXInspectorModel::getEnableQuasiStaticMode, "")
        .def("get_fix_articulation_base_model", &PhysXInspectorModel::getFixArticulationBaseModel, "")
        .def("get_data_shape_model", &PhysXInspectorModel::getDataShapeModel, "")
        .def("get_control_type_model", &PhysXInspectorModel::getControlTypeModel, "")
        .def("get_property_type_model", &PhysXInspectorModel::getPropertyTypeModel, "")
        .def("set_selected_paths", &PhysXInspectorModel::setSelectedPaths, "")
        .def("set_joint_value", &PhysXInspectorModel::setJointValue, "")
        .def("get_joint_value_attribute_name", &PhysXInspectorModel::getJointValueAttributeName, "")
        .def("refresh_model_values", &PhysXInspectorModel::refreshModelValues, "")
        .def("refresh_model_structure", &PhysXInspectorModel::refreshModelStructure, "")
        .def("get_inspected_prim_string_model", &PhysXInspectorModel::getInspectedPrimStringModel, "");
}

void wrapPhysxInspectorTypes(pybind11::module& m)
{
    wrapPhysXInspectorModel(m);
    wrapPhysXInspectorOmniUISceneOverlay(m);
    wrapPhysXInspectorPanel(m);
    wrapSupportUiPrivateInterface(m);
    m.attr("SETTINGS_PHYSICS_INSPECTOR_ENABLED") = py::str(omni::physx::kSettingsPhysicsInspectorEnabled);
}

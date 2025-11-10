// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include <private/omni/physx/IPhysxSupportUi.h>

#include <omni/ui/bind/BindUtils.h>
#include <carb/BindingsPythonUtils.h>

CARB_BINDINGS("carb.physx.supportui.python")

DISABLE_PYBIND11_DYNAMIC_CAST(carb::events::IEventStream)
void wrapPhysxInspectorTypes(pybind11::module& m);
void RigidBodyManipulatorBindings(pybind11::module& m);

namespace
{

#define ADD_SETTING(attr_name, path) \
    m.attr(attr_name) = py::str(path); \
    m.attr(attr_name "_DEFAULT") = py::str(path##Default);

PYBIND11_MODULE(_physxSupportUi, m)
{
    using namespace carb;
    using namespace omni::physx;

    pybind11::module::import("omni.ui");

    wrapPhysxInspectorTypes(m);

    const char* docString;

    py::enum_<IPhysxSupportUi::EventType>(m, "SupportUiEventType", R"(SupportUi events used by event stream.)")
        .value("COLLIDER_CREATED", IPhysxSupportUi::EventType::eColliderCreated,
               R"(When collider has been created; contains the following in dictionary:
                'primPath': int2 - Usd path to the prim decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                'colliderType': bool - collider type. false = static, true = dynamic
                'simplificationType': int - if collType is static then StaticColliderSimplificationType, otherwise DynamicColliderSimplificationType
                'numRemainingCollTasks': int - number of remaining collision tasks
                'numTotalCollTasks': int - total number of collision tasks
            )")
        .value("RIGID_BODY_CREATED", IPhysxSupportUi::EventType::eRigidBodyCreated,
               R"(When a rigid body API has been applied; contains the following in dictionary:
                'primPath': int2 - Usd path to the prim decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
            )")
        .value("RIGID_BODY_REMOVED", IPhysxSupportUi::EventType::eRigidBodyRemoved,
               R"(When a rigid body API has been removed; contains the following in dictionary:
                'primPath': int2 - Usd path to the prim decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
            )")
        .value("KINEMATICS_TOGGLED", IPhysxSupportUi::EventType::eKinematicsToggled,
               R"(When a kinematic rigid body flag was toggled; contains the following in dictionary:
                'primPath': int2 - Usd path to the prim decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
            )")
        .value("AUTO_COLL_CANCELED", IPhysxSupportUi::EventType::eAutoCollCanceled,
               R"(When an applied physics/physx schema on prim is detected, automatic collider creation is canceled; contains the following in dictionary:
                'primPath': int2 - Usd path to the prim containing the collider decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
            )");

    py::enum_<IPhysxSupportUi::ColliderType>(
        m, "SupportUiColliderType", R"(SupportUi collider type used upon collider creation.)")
        .value("AUTODETECT", IPhysxSupportUi::ColliderType::eAutodetect, "IPhysxSupportUi::ColliderType::eAutodetect")
        .value("STATIC", IPhysxSupportUi::ColliderType::eStatic, "IPhysxSupportUi::ColliderType::eStatic")
        .value("DYNAMIC", IPhysxSupportUi::ColliderType::eDynamic, "IPhysxSupportUi::ColliderType::eDynamic");

    py::enum_<IPhysxSupportUi::StaticColliderSimplificationType>(
        m, "SupportUiStaticColliderSimplificationType",
        R"(SupportUi static collider simplification type used upon collider creation.)")
        .value("NONE", IPhysxSupportUi::StaticColliderSimplificationType::eNone,
               R"(IPhysxSupportUi::StaticColliderSimplificationType::eNone)")
        .value("MESH", IPhysxSupportUi::StaticColliderSimplificationType::eMeshSimplification,
               R"(IPhysxSupportUi::StaticColliderSimplificationType::eMeshSimplification)");

    py::enum_<IPhysxSupportUi::DynamicColliderSimplificationType>(m, "SupportUiDynamicColliderSimplificationType",
        R"(SupportUi dynamic collider simplification type used upon collider creation.)")
        .value("CONVEX_HULL", IPhysxSupportUi::DynamicColliderSimplificationType::eConvexHull,
               R"(IPhysxSupportUi::DynamicColliderSimplificationType::eConvexHull)")
        .value("CONVEX_DECOMPOSITION", IPhysxSupportUi::DynamicColliderSimplificationType::eConvexDecomposition,
               R"(IPhysxSupportUi::DynamicColliderSimplificationType::eConvexDecomposition)")
        .value("SDF", IPhysxSupportUi::DynamicColliderSimplificationType::eSDF,
               R"(IPhysxSupportUi::DynamicColliderSimplificationType::eSDF)");


    m.doc() = "pybind11 carb.physx.supportui bindings";

    ADD_SETTING("SETTINGS_ACTION_BAR_ENABLED", kSettingsActionBarEnabled);
    ADD_SETTING("SETTINGS_LOGGING_ENABLED", kSettingsLoggingEnabled);
    ADD_SETTING("SETTINGS_FLOATING_NOTIFICATIONS_ENABLED", kSettingsFloatingNotificationsEnabled);
    ADD_SETTING("SETTINGS_RIGID_BODY_SELECTION_MODE_ENABLED", kSettingsRigidBodySelectionModeEnabled);
    ADD_SETTING("SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED", kSettingsAutomaticColliderCreationEnabled);
    ADD_SETTING("SETTINGS_ASYNC_COOKING_AT_STAGE_LOAD", kSettingsAsyncCookingAtStageLoad);
    ADD_SETTING("SETTINGS_TOOLBAR_BUTTONS_WITH_TEXT", kSettingsToolbarButtonsWithText);
    ADD_SETTING("SETTINGS_AVOID_CHANGING_EXISTING_COLLIDERS", kSettingsAvoidChangingExistingColliders);
    ADD_SETTING("SETTINGS_CUSTOM_MANIPULATOR_ENABLED", kSettingsCustomManipulatorEnabled);
    ADD_SETTING("SETTINGS_MANIP_MODE_ALLOW_RIGID_BODY_TRAVERSAL", kSettingsManipModeAllowRigidBodyTraversal);
    ADD_SETTING("SETTINGS_MANIP_MODE_ALLOW_ROT_WHILE_TRANSLATING", kSettingsManipModeAllowRotWhileTranslating);
    ADD_SETTING("SETTINGS_MANIP_MODE_ALLOW_TRAN_ON_OTHER_AXES_WHILE_TRANSLATING", kSettingsManipModeAllowTranOnOtherAxesWhileTranslating);
    ADD_SETTING("SETTINGS_MANIP_MODE_ALLOW_TRAN_WHILE_ROTATING", kSettingsManipModeAllowTranWhileRotating);
    ADD_SETTING("SETTINGS_MANIP_MODE_ALLOW_ROT_ON_OTHER_AXES_WHILE_ROTATING", kSettingsManipModeAllowRotOnOtherAxesWhileRotating);
    ADD_SETTING("SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE", kSettingsStaticColliderSimplificationType);
    ADD_SETTING("SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE", kSettingsDynamicColliderSimplificationType);

    py::class_<IPhysxSupportUi> physxSupportUi = defineInterfaceClass<IPhysxSupportUi>
    (
        m, "IPhysxSupportUi", "acquire_physx_supportui_interface", "release_physx_supportui_interface"
    );
    
    m.def("release_physx_supportui_interface_scripting", [](IPhysxSupportUi* iface) { carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface); });

    docString = R"(
        Attempts to create collider(s) on supplied prim and all its children.
        In case of static collider type, existing rigid bodies are removed.
        In case of dynamic collider type, rigid body is created if there
        was none.
        Rules for rigid body creation:
        1) Rigid body should not be created if there is other rigid body up
           in the hierarchy. Warning will be produced.
        2) The most common node, when rigid bodies are created, is a node
           with 'kind' set to 'component'.
        3) In case there is no component kind, the top most node ('primPath')
           gets rigid body created.

        NOTE: This method just queues prims for collider creation, which
              happens later, batched together with other coll. requests.

        Args:
           primPath: prim path (uint64_t -> const pxr::SdfPath&)
           colliderType: collider type specified by SupportUiColliderType enum
    )";
    physxSupportUi.def("create_colliders", wrapInterfaceFunction(&IPhysxSupportUi::createColliders), docString);

    docString = R"(
        Returns:
           return number of queued colliders yet to create.
    )";
    physxSupportUi.def("get_num_of_colliders_to_create", wrapInterfaceFunction(&IPhysxSupportUi::getNumOfCollidersToCreate), docString);

    docString = R"(
        Clears colliders processing queue which effectively stops coll. creation.
    )";
    physxSupportUi.def("clear_colliders_processing_queue", wrapInterfaceFunction(&IPhysxSupportUi::clearCollidersProcessingQueue), docString);

    docString = R"(
        Event stream sending various events defined in SupportUiEvent enum.

        Returns:
            Event stream sending the events.
    )";
    physxSupportUi.def("get_event_stream", wrapInterfaceFunction(&IPhysxSupportUi::getEventStream), docString);

    RigidBodyManipulatorBindings(m);
}

} // namespace

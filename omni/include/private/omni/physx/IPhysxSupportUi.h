// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/events/IEvents.h>

#include <private/omni/physx/PhysxUsd.h>
#include <omni/physx/IPhysxSettings.h>

namespace omni
{
namespace physx
{

struct IPhysxSupportUi
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxSupportUi", 0, 3)

    /// events received through event stream
    enum class EventType : int
    {
        eColliderCreated, //!< When a collider has been has been applied
        eRigidBodyCreated, //!< When a rigid body API has been applied
        eRigidBodyRemoved, //!< When a rigid body API has been removed
        eKinematicsToggled, //!< When a kinematic rigid body flag was toggled
        eAutoCollCanceled, //!< When an applied physics/physx schema on prim is detected, automatic collider creation is
                           //!< canceled
    };

    /// type of collider to create
    enum class ColliderType : int
    {
        eAutodetect,
        eStatic,
        eDynamic,
    };

    /// static collider simplification type
    enum class StaticColliderSimplificationType : int
    {
        eNone, //!< triangle mesh
        eMeshSimplification,
    };

    /// dynamic collider simplification type
    enum class DynamicColliderSimplificationType : int
    {
        eConvexHull,
        eConvexDecomposition,
        eSDF,
    };

    /// Attempts to create collider(s) on supplied prim and all its children.
    /// In case of static collider type, existing rigid bodies are removed.
    /// In case of dynamic collider type, rigid body is created if there
    /// was none.
    /// Rules for rigid body creation:
    /// 1) Rigid body should not be created if there is other rigid body up
    ///    in the hierarchy. Warning will be produced.
    /// 2) The most common node, when rigid bodies are created, is a node
    ///    with 'kind' set to 'component'.
    /// 3) In case there is no component kind, the top most node ('primPath')
    ///    gets rigid body created.
    ///
    /// NOTE: This method just queues prims for collider creation, which
    /// happens later, batched together with other coll. requests.
    //
    /// \param   primPath: prim path (uint64_t -> const pxr::SdfPath&)
    /// \param   colliderType: collider type specified by ColliderType enum

    bool(CARB_ABI* createColliders)(uint64_t primPath, ColliderType colliderType);

    /// \return number of queued colliders yet to create.
    int(CARB_ABI* getNumOfCollidersToCreate)();

    /// Clears colliders processing queue which effectively stops coll. creation.
    void(CARB_ABI* clearCollidersProcessingQueue)();

    /// Event stream sending various events defined in SupportUiEvent enum
    ///
    /// \return Event stream sending the events.
    carb::events::IEventStreamPtr(CARB_ABI* getEventStream)();
};

/** \addtogroup Settings
 *  @{
 */

/// Shows / hides the Physics Authoring Toolbar.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingsActionBarEnabled, "/supportUiActionBarEnabled")
/// \ingroup private
static constexpr bool kSettingsActionBarEnabledDefaultVal{ false };

/// Enables / disables debug logging.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingsLoggingEnabled, "/supportUiLoggingEnabled")
/// \ingroup private
static constexpr bool kSettingsLoggingEnabledDefaultVal{ false };

/// Toggles selection mode, which always tries to select an asset that contains a rigid body component.
DEFINE_PHYSX_SETTING(kSettingsRigidBodySelectionModeEnabled, "/supportUiRigidBodySelectionModeEnabled");
/// \ingroup private
static constexpr bool kSettingsRigidBodySelectionModeEnabledDefaultVal{ false };

/// Toggles informative floating notifications. If set to True they are normally shown each time a collider is created or when physics components are removed.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingsFloatingNotificationsEnabled, "/supportUiFloatingNotificationsEnabled");
/// \ingroup private
static constexpr bool kSettingsFloatingNotificationsEnabledDefaultVal{ true };

/// Toggles custom Rigid Body Manipulator which only works in Simulation Mode and only with physics-ready assets which have a rigid body component in hierarchy.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingsCustomManipulatorEnabled, "/supportUiCustomManipulatorEnabled");
/// \ingroup private
static constexpr bool kSettingsCustomManipulatorEnabledDefaultVal{ true };

/// If set to True and the selected prim lacks a rigid body, the manipulator can traverse up the hierarchy to find a parent with a rigid body, allowing for manipulation through physics forces.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingsManipModeAllowRigidBodyTraversal, "/supportUiManipModeAllowRigidBodyTraversal");
/// \ingroup private
static constexpr bool kSettingsManipModeAllowRigidBodyTraversalDefaultVal{ false };

/// If set to True allows manipulated object to rotate when translating in a selected gizmo direction.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingsManipModeAllowRotWhileTranslating,
                                "/supportUiManipModeAllowRotWhileTranslating");
/// \ingroup private
static constexpr bool kSettingsManipModeAllowRotWhileTranslatingDefaultVal{ true };

/// If set to True allows translated object to be translated also on other axes than ones that were selected via the gizmo.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingsManipModeAllowTranOnOtherAxesWhileTranslating,
                                "/supportUiManipModeAllowTranOnOtherAxesWhileTranslating");
/// \ingroup private
static constexpr bool kSettingsManipModeAllowTranOnOtherAxesWhileTranslatingDefaultVal{ true };

/// If set to True allows manipulated object to translate when rotating in a selected gizmo direction.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingsManipModeAllowTranWhileRotating, "/supportUiManipModeAllowTranWhileRotating");
/// \ingroup private
static constexpr bool kSettingsManipModeAllowTranWhileRotatingDefaultVal{ true };

/// If set to True allows rotated object to be rotated also on other axes than ones that were selected via the gizmo.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingsManipModeAllowRotOnOtherAxesWhileRotating,
                                "/supportUiManipModeAllowRotOnOtherAxesWhileRotating");
/// \ingroup private
static constexpr bool kSettingsManipModeAllowRotOnOtherAxesWhileRotatingDefaultVal{ true };

/// If set to True, colliders are being created automatically, immediately when assets are inserted into a stage.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingsAutomaticColliderCreationEnabled, "/supportUiAutomaticCollisionCreationEnabled");
/// \ingroup private
static constexpr bool kSettingsAutomaticColliderCreationEnabledDefaultVal{ false };

/// if set to True, async cooking is triggered each time an asset is added to the stage, or right after a stage is loaded. Cooking pre-prepares assets for simulation.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingsAsyncCookingAtStageLoad, "/supportUiAsyncCookingAtStageLoad");
/// \ingroup private
static constexpr bool kSettingsAsyncCookingAtStageLoadDefaultVal{ true };

/// if set to True, Physics Authoring Toolbar buttons are shown together with their textual description.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingsToolbarButtonsWithText, "/supportUiToolbarButtonsWithText");
/// \ingroup private
static constexpr bool kSettingsToolbarButtonsWithTextDefaultVal{ false };

/// if set to True, in case an object already has a collider, clicking a button to create new collider will prefer not to change its type.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingsAvoidChangingExistingColliders, "/supportUiAvoidChangingExistingColliders");
/// \ingroup private
static constexpr bool kSettingsAvoidChangingExistingCollidersDefaultVal{ false };

/// Static Collider Simplification Type as defined in enum class StaticColliderSimplificationType.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingsStaticColliderSimplificationType, "/supportUiStaticColliderSimplificationType");
/// \ingroup private
static constexpr IPhysxSupportUi::StaticColliderSimplificationType kSettingsStaticColliderSimplificationTypeDefaultVal{
    IPhysxSupportUi::StaticColliderSimplificationType::eNone
};

/// Dynamic Collider Simplification Type as defined in enum class DynamicColliderSimplificationType.
DEFINE_PERSISTENT_PHYSX_SETTING(kSettingsDynamicColliderSimplificationType,
                                "/supportUiDynamicColliderSimplificationType");
/// \ingroup private
static constexpr IPhysxSupportUi::DynamicColliderSimplificationType kSettingsDynamicColliderSimplificationTypeDefaultVal{
    IPhysxSupportUi::DynamicColliderSimplificationType::eConvexHull
};

/** @}*/

} // namespace physx
} // namespace omni

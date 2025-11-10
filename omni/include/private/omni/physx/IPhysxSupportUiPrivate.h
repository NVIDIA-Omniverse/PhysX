// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/events/IEvents.h>

#include <private/omni/physx/PhysxUsd.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/ui/AbstractItemModel.h>
#include <omni/ui/SimpleStringModel.h>
#include <omni/ui/SimpleNumericModel.h>
#include <omni/ui/Widget.h>
#include <omni/ui/scene/Manipulator.h>

/// A private interface for physics extensions that need to be tightly coupled with omni.physx.
///
/// Every class/interface or enumeration inside this header file is subject to change without notice.
///
/// This interface and support classes should be considered internal to the omni.physx.supportui extension and
/// should not be used by external clients.  Clients should rely on public interfaces IPhysxSupportUi
///

// This class is internal to omni.physx and it's subject to changes without notice
class PhysXInspectorModel : public omni::ui::AbstractItemModel
{
public:
    enum class State : int
    {
        eDisabled,
        eAuthoring,
        eRunningSimulation
    };
    enum class InspectorType : int
    {
        eJointsList,
        eJointsBodiesHierarchy
    };
    enum class DataShapeType : int
    {
        eFlat,
        eHierarchical
    };
    enum class ControlType : int
    {
        eAutomatic,
        eJointState,
        eJointDrive,
        eJointVelocity,
    };
    enum class PropertyType : int
    {
        eShowLimits,
        eShowGains,
    };
    virtual void selectAllConnectedBodyJoints() = 0;
    virtual void selectAllConnectedBodyShapes() = 0;
    virtual void selectAllConnectedJointShapes() = 0;
    virtual void selectAllConnectedLinks() = 0;
    virtual void setInspectorType(InspectorType inspectorType) = 0;
    virtual InspectorType getInspectorType() const = 0;
    virtual std::vector<std::shared_ptr<const omni::ui::AbstractItemModel::AbstractItem>> getItemsMatchingPaths(
        const std::vector<std::string>& paths) = 0;
    virtual std::shared_ptr<omni::ui::SimpleStringModel> getInspectedPrimStringModel() const = 0;
    virtual std::shared_ptr<omni::ui::SimpleIntModel> getDataShapeModel() const = 0;
    virtual std::shared_ptr<omni::ui::SimpleStringModel> getControlTypeModel() const = 0;
    virtual std::shared_ptr<omni::ui::SimpleStringModel> getPropertyTypeModel() const = 0;
    virtual std::shared_ptr<omni::ui::SimpleBoolModel> getShowMassesAndInertiaModel() const = 0;
    virtual std::shared_ptr<omni::ui::SimpleBoolModel> getEnableGravity() const = 0;
    virtual std::shared_ptr<omni::ui::SimpleBoolModel> getEnableQuasiStaticMode() const = 0;
    virtual std::shared_ptr<omni::ui::SimpleBoolModel> getFixArticulationBaseModel() const = 0;
    virtual void setSelectedPaths(std::vector<std::string> paths) = 0;
    virtual bool setJointValue(const char* primPath, float jointValue) = 0;
    virtual const char* getJointValueAttributeName(const char* primPath) = 0;
    virtual void refreshModelValues() = 0;
    virtual void refreshModelStructure() = 0;
    virtual void refreshModelOverrides() = 0;
};

OMNIUI_SCENE_NAMESPACE_OPEN_SCOPE
// This class is internal to omni.physx and it's subject to changes without notice
class PhysXInspectorOverlay : public omni::ui::scene::Manipulator
{
};
OMNIUI_SCENE_NAMESPACE_CLOSE_SCOPE

OMNIUI_NAMESPACE_OPEN_SCOPE
// This class is internal to omni.physx and it's subject to changes without notice
class PhysXInspectorWidget : public omni::ui::Widget
{
public:
    OMNIUI_PROPERTY(PhysXInspectorModel::InspectorType,
                    inspectorType,
                    DEFAULT,
                    PhysXInspectorModel::InspectorType::eJointsList,
                    READ,
                    getInspectorType,
                    WRITE,
                    setInspectorType);
    OMNIUI_PROPERTY(std::shared_ptr<PhysXInspectorModel>, model, READ, getModel, WRITE, setModel);
};
OMNIUI_NAMESPACE_CLOSE_SCOPE

namespace omni
{
namespace physx
{
DEFINE_PHYSX_SETTING(kSettingsPhysicsInspectorEnabled, "/supportUiPhysicsInspector/enabled")
constexpr bool kSettingsPhysicsInspectorEnabledDefaultVal = false;
// This interface is internal to omni.physx and it's subject to changes without notice
struct IPhysxSupportUiPrivate
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxSupportUiPrivate", 0, 1)
    std::shared_ptr<PhysXInspectorModel>(CARB_ABI* createPhysXInspectorModel)(const pxr::SdfPathVector& selection);
    std::shared_ptr<omni::ui::scene::PhysXInspectorOverlay>(CARB_ABI* createPhysXInspectorOverlay)();
    std::shared_ptr<omni::ui::PhysXInspectorWidget>(CARB_ABI* createPhysXInspectorWidget)();
    carb::events::IEventStreamPtr(CARB_ABI* getPhysXInspectorEventStream)();
    PhysXInspectorModel::State(CARB_ABI* getPhysXInspectorState)();
    void(CARB_ABI* stepPhysXInspectorSimulation)(float dt);
    void(CARB_ABI* resetPhysXInspectorToAuthoringStart)();
    void(CARB_ABI* enablePhysXInspectorAuthoringMode)();
    void(CARB_ABI* enableNoticeHandler)(bool enable);
    void(CARB_ABI* refreshAllInspectorModelsStructure)();
    void(CARB_ABI* refreshAllInspectorModelsValues)();
    void(CARB_ABI* commitPhysXInspectorAuthoringState)();
};

} // namespace physx
} // namespace omni

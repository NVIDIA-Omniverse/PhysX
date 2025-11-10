// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once
#include <private/omni/physx/IPhysxSupportUiPrivate.h>
#include <PxPhysicsAPI.h>

namespace omni
{
namespace ui
{
class PhysXInspectorWidgetImpl;
namespace scene
{
class PhysXInspectorOverlayImpl;
}
} // namespace ui
} // namespace omni
using AbstractItemConstPointer = std::shared_ptr<const omni::ui::AbstractItemModel::AbstractItem>;
class PhysXInspectorItem : public omni::ui::AbstractItemModel::AbstractItem
{
public:
    enum class Type
    {
        ArticulationJoint,
        ArticulationLink,
        Articulation,
        MaximalJoint,
        Material,
        Collider,
        Group
    };

    PhysXInspectorItem(const char* name, const char* path, Type type);
    pxr::SdfPath usdPath;
    Type type;
    std::shared_ptr<omni::ui::SimpleStringModel> nameModel; // Attribute 0
    std::shared_ptr<omni::ui::SimpleStringModel> usdPathModel; // Attribute 1
    std::shared_ptr<omni::ui::SimpleStringModel> detailAsTextValueModel; // Attribute 2
    std::shared_ptr<omni::ui::SimpleFloatModel> numericValue; // Attribute 3
    std::shared_ptr<omni::ui::SimpleFloatModel> minValue; // Attribute 4
    std::shared_ptr<omni::ui::SimpleFloatModel> maxValue; // Attribute 5
    std::shared_ptr<omni::ui::SimpleFloatModel> stiffnessModel; // Attribute 6
    std::shared_ptr<omni::ui::SimpleFloatModel> dampingModel; // Attribute 7
    std::vector<AbstractItemConstPointer> children;
};

class PhysXInspector;
class PhysXInspectorDebugVisualization;

class PhysXInspectorModelImpl : public PhysXInspectorModel
{
    friend class PhysXInspector;
    friend class omni::ui::scene::PhysXInspectorOverlayImpl;
    friend class omni::ui::PhysXInspectorWidgetImpl;
    friend class PhysXInspectorDebugVisualization;
    bool isIncludedInCurrentSelection(void* userData) const;
    bool isIncludedInCurrentSelection(const pxr::SdfPath& path) const;

    std::shared_ptr<omni::ui::SimpleStringModel> mSelectedPrimStringModel;
    std::shared_ptr<omni::ui::SimpleIntModel> mDataShapeModel;
    std::shared_ptr<omni::ui::SimpleStringModel> mControlTypeModel; // Automatic / Position / Drive / Velocity
    std::shared_ptr<omni::ui::SimpleStringModel> mPropertyTypeModel; // Limits or gains
    std::shared_ptr<omni::ui::SimpleBoolModel> mShowMassesAndInertia;
    std::shared_ptr<omni::ui::SimpleBoolModel> mFixArticulationBase;

    std::vector<AbstractItemConstPointer> _children;
    PhysXInspector* mInspector;
    InspectorType mInspectorType = InspectorType::eJointsList;
    std::vector<std::string> mSelection;
    pxr::SdfPathVector mSelectionSdf;
    std::vector<omni::physx::usdparser::PhysxArticulationDesc*> mArticulations;
    std::vector<PhysXInspectorItem*> mAllJoints;
    std::vector<pxr::SdfPath> mAllArticulations;
    std::vector<bool> mIsArticulationAllowedToBeFixed;
    std::string mSelectedPrimPath;
    pxr::SdfPath mSelectedPrimPathSdf;
    pxr::SdfPath mScenePrimPathSdf;
    bool mSelectionIsArticulation = false;

    std::shared_ptr<PhysXInspectorItem> makeInspectorItem(const char* name,
                                                          const char* path,
                                                          PhysXInspectorItem::Type type);
    void addArticulationJoints(PhysXInspectorItem& item, ::physx::PxArticulationReducedCoordinate* articulation);
    void addArticulationHierarchy(PhysXInspectorItem& parent);
    void addArticulationHierarchyLink(PhysXInspectorItem& parent, ::physx::PxArticulationLink* link);
    void addArticulationHierarchyJoint(PhysXInspectorItem& parent, ::physx::PxArticulationJointReducedCoordinate* joint);
    void addRigidShapes(PhysXInspectorItem& parent, ::physx::PxRigidActor* rigid);
    void addRigidJoints(PhysXInspectorItem& parent, ::physx::PxRigidActor* rigid);
    void addDynamicBodies(::physx::PxScene* scene);
    void addStaticBodies(::physx::PxScene* scene);
    void addMaterials(::physx::PxScene* scene);
    void addArticulations(::physx::PxScene* scene);
    void fixArticulationsBase(bool fixBases);
    void parseArticulations(uint64_t stageId);
    void releaseArticulationParsing();
    void findCollisionAPIFor(const pxr::SdfPath& path, pxr::SdfPathVector& collisionApis);

    pxr::SdfPath getPathForUserData(void* userData) const;

    std::shared_ptr<PhysXInspectorItem> addJoint(PhysXInspectorItem& item,
                                                 const pxr::SdfPath& path,
                                                 PhysXInspectorItem::Type type);
    DataShapeType refreshJointsList(::physx::PxScene* scene);
    DataShapeType refreshHierarchy(::physx::PxScene* scene);
    void recursivegetItemsMatchingPaths(const std::vector<std::string>& paths,
                                        const std::vector<AbstractItemConstPointer>& lookIntoChildren,
                                        std::vector<AbstractItemConstPointer>& foundItems);

    static void searchSelectionRecursively(const std::vector<AbstractItemConstPointer>& children,
                                           const std::vector<std::string>& paths,
                                           std::vector<AbstractItemConstPointer>& selection);

    static bool tryReadJointDrive(pxr::UsdPrim& prim, float& value);
    static bool tryReadJointVelocity(pxr::UsdPrim& prim, float& value);
    static bool tryReadJointState(pxr::UsdPrim& prim, float& value);
    static bool tryWriteJointVelocity(pxr::UsdPrim& prim, float value);
    static bool tryWriteJointDrive(pxr::UsdPrim& prim, float value);
    static bool tryWriteJointState(pxr::UsdPrim& prim, float value);

    bool isArticulationJoint(const pxr::SdfPath& path) const;
    ::physx::PxArticulationLink* findRootLink(::physx::PxArticulationReducedCoordinate* articulation);

public:
    PhysXInspectorModelImpl(const pxr::SdfPathVector& selection, PhysXInspector* inspector);
    ~PhysXInspectorModelImpl();
    virtual void setInspectorType(InspectorType inspectorType) override;
    virtual InspectorType getInspectorType() const override;
    virtual void selectAllConnectedBodyJoints() override;
    virtual void selectAllConnectedBodyShapes() override;
    virtual void selectAllConnectedJointShapes() override;
    virtual void selectAllConnectedLinks() override;
    virtual std::shared_ptr<omni::ui::SimpleStringModel> getInspectedPrimStringModel() const override
    {
        return mSelectedPrimStringModel;
    }
    virtual std::shared_ptr<omni::ui::SimpleIntModel> getDataShapeModel() const override
    {
        return mDataShapeModel;
    }

    virtual std::shared_ptr<omni::ui::SimpleStringModel> getControlTypeModel() const override
    {
        return mControlTypeModel;
    }

    virtual std::shared_ptr<omni::ui::SimpleStringModel> getPropertyTypeModel() const override
    {
        return mPropertyTypeModel;
    }

    virtual std::shared_ptr<omni::ui::SimpleBoolModel> getShowMassesAndInertiaModel() const override
    {
        return mShowMassesAndInertia;
    }
    virtual std::shared_ptr<omni::ui::SimpleBoolModel> getFixArticulationBaseModel() const override
    {
        return mFixArticulationBase;
    }
    virtual std::shared_ptr<omni::ui::SimpleBoolModel> getEnableGravity() const override;
    virtual std::shared_ptr<omni::ui::SimpleBoolModel> getEnableQuasiStaticMode() const override;

    virtual std::vector<AbstractItemConstPointer> getItemsMatchingPaths(const std::vector<std::string>& paths) override;
    virtual std::vector<AbstractItemConstPointer> getItemChildren(
        const AbstractItemConstPointer& parentItem = nullptr) override;
    virtual size_t getItemValueModelCount(const AbstractItemConstPointer& item = nullptr) override;
    virtual std::shared_ptr<omni::ui::AbstractValueModel> getItemValueModel(const AbstractItemConstPointer& item = nullptr,
                                                                            size_t index = 0) override;
    virtual void setSelectedPaths(std::vector<std::string> paths) override;

    virtual void refreshModelValues() override;
    virtual void refreshModelStructure() override;
    virtual void refreshModelOverrides() override;

    virtual const char* getJointValueAttributeName(const char* primPath) override;
    virtual bool setJointValue(const char* primPath, float jointValue) override;
    bool getJointValue(const char* primPath, float& jointValue);
};

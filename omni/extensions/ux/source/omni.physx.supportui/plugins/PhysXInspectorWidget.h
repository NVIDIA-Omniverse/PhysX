// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once
#include <private/omni/physx/IPhysxSupportUiPrivate.h>
#include <carb/settings/ISettings.h>
#include <omni/physx/IPhysx.h>
#include <omni/ui/IGlyphManager.h>
#include <PxPhysicsAPI.h>

class PhysXInspector;
class PhysXInspectorModelImpl;
OMNIUI_NAMESPACE_OPEN_SCOPE

class PhysXInspectorWidgetImpl : public omni::ui::PhysXInspectorWidget
{
    OMNIUI_OBJECT(PhysXInspectorWidgetImpl)
protected:
    PhysXInspectorWidgetImpl(PhysXInspector* inspector);
    virtual void _drawContent(float elapsedTime) override;

private:
    void drawInspectorSelected(PhysXInspectorModel::InspectorType inspectorType);
    void drawScene(PhysXInspectorModel::InspectorType inspectorType, const std::string& currentPath);
    void drawSceneJoints(::physx::PxScene* scene);
    void drawSceneHierarchy(::physx::PxScene* scene);
    void drawMaterials(::physx::PxPhysics& physics);
    void drawSelectionJointsContextMenu(const pxr::SdfPath& jointPath);
    void drawSelectionBodiesContextMenu(const pxr::SdfPath& bodyPath);
    bool drawSelectableTreenode(const char* fullPath, const char* rigidName);
    void drawSelectableTreenodeLeaf(const char* fullPath, const char* rigidName);
    void drawRigidMassLabel(::physx::PxRigidBody* rigid);
    void drawRigidJoints(::physx::PxRigidBody* rigid);
    void drawRigidShapes(::physx::PxRigidActor* rigid);
    void drawJointSlider(const pxr::SdfPath& jointPath);

    void drawArticulation(PhysXInspectorModel::InspectorType inspectorType, const std::string& currentPath);
    void drawArticulationJoints(::physx::PxArticulationReducedCoordinate* articulation);
    void drawArticulationHierarchy(::physx::PxArticulationReducedCoordinate* articulation);
    void drawArticulationJoint(::physx::PxArticulationJointReducedCoordinate* joint);
    void drawLinkNode(::physx::PxArticulationLink* link);
    void drawLinkChildren(::physx::PxArticulationLink* link);
    void drawShapeLabel(::physx::PxShape* shape);

    PhysXInspector& mInspector;
    carb::settings::ISettings* mISettings;
    omni::physx::IPhysx* mPhysXInterface;
    omni::ui::IGlyphManager* mGlyphManager;
};

OMNIUI_NAMESPACE_CLOSE_SCOPE

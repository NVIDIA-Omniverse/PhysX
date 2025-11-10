// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"
// clang-format off
#include "PhysXInspectorWidget.h"
#include "PhysXInspector.h"
#include "PhysXInspectorModel.h"
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/Selection.h>
#include <imgui.h>
// clang-format on

using namespace physx;
OMNIUI_NAMESPACE_OPEN_SCOPE
PhysXInspectorWidgetImpl::PhysXInspectorWidgetImpl(PhysXInspector* inspector) : mInspector(*inspector)
{
    mISettings = carb::getCachedInterface<carb::settings::ISettings>();
    mPhysXInterface = carb::getCachedInterface<omni::physx::IPhysx>();
    mGlyphManager = carb::getCachedInterface<omni::ui::IGlyphManager>();
}

void PhysXInspectorWidgetImpl::_drawContent(float elapsedTime)
{
    drawInspectorSelected(getInspectorType());
}

void PhysXInspectorWidgetImpl::drawInspectorSelected(PhysXInspectorModel::InspectorType inspectorType)
{
    auto model = std::static_pointer_cast<PhysXInspectorModelImpl>(getModel());
    if (model->mSelectionIsArticulation)
    {
        drawArticulation(inspectorType, model->mSelectedPrimPath);
    }
    else
    {
        drawScene(inspectorType, model->mSelectedPrimPath);
    }
}

void PhysXInspectorWidgetImpl::drawScene(PhysXInspectorModel::InspectorType inspectorType, const std::string& currentPath)
{
    ::physx::PxScene* scene = mInspector.getSceneAt(mInspector.mSelectedPhysicsScenePath.GetText());
    if (scene)
    {
        switch (inspectorType)
        {
        case PhysXInspectorModel::InspectorType::eJointsList: {
            const ::physx::PxU32 nbArticulations = scene->getNbArticulations();
            if (nbArticulations > 0)
            {
                ImGui::Indent(10);
                if (scene->getNbConstraints() > 0)
                {
                    if (ImGui::CollapsingHeader("Maximal Joints", ImGuiTreeNodeFlags_None))
                    {
                        ImGui::Columns(2, "columns", true);
                        drawSceneJoints(scene);
                        ImGui::Columns(1, "columns", false);
                    }
                }
                auto model = std::static_pointer_cast<PhysXInspectorModelImpl>(getModel());
                ::physx::PxArticulationReducedCoordinate* articulation = nullptr;
                for (::physx::PxU32 i = 0; i < nbArticulations; i++)
                {
                    ImGui::PushID((int)i);
                    scene->getArticulations(&articulation, 1, i);
                    if (!model->isIncludedInCurrentSelection(articulation->userData))
                        continue;
                    if (ImGui::CollapsingHeader(articulation->getName(), ImGuiTreeNodeFlags_None))
                    {
                        ImGui::Columns(2, "columns", true);
                        drawArticulationJoints(articulation);
                        ImGui::Columns(1, "columns", false);
                    }
                    ImGui::PopID();
                }
                ImGui::Unindent(10);
            }
            else
            {
                ImGui::Columns(2, "columns", true);
                drawSceneJoints(scene);
                ImGui::Columns(1, "columns", false);
            }
            break;
        }
        case PhysXInspectorModel::InspectorType::eJointsBodiesHierarchy: {
            drawSceneHierarchy(scene);
            break;
        }
        }
    }
}

void PhysXInspectorWidgetImpl::drawSceneJoints(::physx::PxScene* scene)
{
    auto model = std::static_pointer_cast<PhysXInspectorModelImpl>(getModel());
    PxConstraint* cs;
    for (PxU32 constrIdx = 0; constrIdx < scene->getNbConstraints(); constrIdx++)
    {
        scene->getConstraints(&cs, 1, constrIdx);
        PxU32 typeId;
        void* external = cs->getExternalReference(typeId);
        if (typeId == PxConstraintExtIDs::eJOINT)
        {
            PxJoint* joint = (PxJoint*)external;
            if (!model->isIncludedInCurrentSelection(joint->userData))
                continue;
            pxr::SdfPath jointPath =
                mPhysXInterface->getPhysXObjectUsdPath((omni::physx::usdparser::ObjectId)joint->userData);

            ImGui::PushID((int)constrIdx);
            const char* name = strrchr(jointPath.GetText(), '/') + 1;
            drawSelectableTreenodeLeaf(jointPath.GetText(), name);
            drawSelectionJointsContextMenu(jointPath);
            ImGui::NextColumn();
            ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
            drawJointSlider(jointPath);
            ImGui::PopItemWidth();
            ImGui::NextColumn();
            ImGui::PopID();
        }
    }
}

void PhysXInspectorWidgetImpl::drawSelectionJointsContextMenu(const pxr::SdfPath& jointPath)
{
    auto& model = *std::static_pointer_cast<PhysXInspectorModelImpl>(getModel());
    const bool selected =
        std::find(model.mSelection.begin(), model.mSelection.end(), jointPath.GetString()) != model.mSelection.end();
    ImGui::PushItemWidth(ImGui::GetFontSize() * -12.0f);
    if (selected && ImGui::BeginPopupContextItem(nullptr, 1))
    {
        ImGui::MenuItem("Selection helpers", nullptr, false, false);
        char buffer[100];
        auto glyphIcon = mGlyphManager->getGlyphInfo("${glyphs}/menu_physics.svg");
        snprintf(buffer, sizeof(buffer) - 1, "%s Select all connected bodies", glyphIcon.code);
        if (ImGui::MenuItem(buffer, nullptr, false, true))
        {
            model.selectAllConnectedLinks();
        }
        glyphIcon = mGlyphManager->getGlyphInfo("${glyphs}/geometry.svg");
        snprintf(buffer, sizeof(buffer) - 1, "%s Select all connected shapes", glyphIcon.code);
        if (ImGui::MenuItem(buffer, nullptr, false, true))
        {
            model.selectAllConnectedJointShapes();
        }
        ImGui::EndPopup();
    }
    ImGui::PopItemWidth();
}

void PhysXInspectorWidgetImpl::drawSelectionBodiesContextMenu(const pxr::SdfPath& bodyPath)
{
    auto& model = *std::static_pointer_cast<PhysXInspectorModelImpl>(getModel());
    const bool selected =
        std::find(model.mSelection.begin(), model.mSelection.end(), bodyPath.GetString()) != model.mSelection.end();
    ImGui::PushItemWidth(ImGui::GetFontSize() * -12.0f);
    if (selected && ImGui::BeginPopupContextItem(nullptr, 1))
    {
        ImGui::MenuItem("Selection helpers", nullptr, false, false);
        char buffer[100];
        auto glyphIcon = mGlyphManager->getGlyphInfo("${glyphs}/menu_animation.svg");
        snprintf(buffer, sizeof(buffer) - 1, "%s Select all connected joints", glyphIcon.code);
        if (ImGui::MenuItem(buffer, nullptr, false, true))
        {
            model.selectAllConnectedBodyJoints();
        }
        glyphIcon = mGlyphManager->getGlyphInfo("${glyphs}/geometry.svg");
        snprintf(buffer, sizeof(buffer) - 1, "%s Select all connected shapes", glyphIcon.code);
        if (ImGui::MenuItem(buffer, nullptr, false, true))
        {
            model.selectAllConnectedBodyShapes();
        }
        ImGui::EndPopup();
    }
    ImGui::PopItemWidth();
}

bool PhysXInspectorWidgetImpl::drawSelectableTreenode(const char* fullPath, const char* rigidName)
{
    auto& model = *std::static_pointer_cast<PhysXInspectorModelImpl>(getModel());
    const bool selected = std::find(model.mSelection.begin(), model.mSelection.end(), fullPath) != model.mSelection.end();
    ImGuiTreeNodeFlags treeNodeFlags =
        ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick |
        ImGuiTreeNodeFlags_SpanAvailWidth | (selected ? ImGuiTreeNodeFlags_Selected : 0);

    const bool opened = ImGui::TreeNodeEx(rigidName, treeNodeFlags);
    if (ImGui::IsItemHovered(0))
    {
        ImGui::SetTooltip(fullPath);
    }
    if (ImGui::IsItemClicked(0)&& !ImGui::IsItemToggledOpen())
    {
        // When we change selection from inside inspector, we lock selection
        const auto usdContext = omni::usd::UsdContext::getContext();
        if (ImGui::GetIO().KeyMods & ImGuiKeyModFlags_Ctrl)
        {
            usdContext->getSelection()->add(pxr::SdfPath(fullPath), false, false, true);
        }
        else
        {
            usdContext->getSelection()->setPrimPathSelected(fullPath, true, true, true, true);
        }
    }
    return opened;
}

void PhysXInspectorWidgetImpl::drawSelectableTreenodeLeaf(const char* fullPath, const char* rigidName)
{
    auto& model = *std::static_pointer_cast<PhysXInspectorModelImpl>(getModel());
    const bool selected = std::find(model.mSelection.begin(), model.mSelection.end(), fullPath) != model.mSelection.end();
    ImGuiTreeNodeFlags treeNodeFlags =
        ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick |
        ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_Leaf |
        ImGuiTreeNodeFlags_NoTreePushOnOpen | (selected ? ImGuiTreeNodeFlags_Selected : 0);

    ImGui::TreeNodeEx(rigidName, treeNodeFlags);
    if (ImGui::IsItemHovered(0))
    {
        ImGui::SetTooltip(fullPath);
    }
    if (ImGui::IsItemClicked(0) && !ImGui::IsItemToggledOpen())
    {
        // When we change selection from inside inspector, we lock selection
        const auto usdContext = omni::usd::UsdContext::getContext();
        if (ImGui::GetIO().KeyMods & ImGuiKeyModFlags_Ctrl)
        {
            usdContext->getSelection()->add(pxr::SdfPath(fullPath), false, false, true);
        }
        else
        {
            usdContext->getSelection()->setPrimPathSelected(fullPath, true, true, true, true);
        }
    }
}

void PhysXInspectorWidgetImpl::drawRigidMassLabel(::physx::PxRigidBody* rigid)
{
    char buffer[100];
    const ::physx::PxVec3 inertia = rigid->getMassSpaceInertiaTensor();
    snprintf(
        buffer, sizeof(buffer) - 1, "[m=%.2f I=[%.4f,%.4f,%.4f]]", rigid->getMass(), inertia.x, inertia.y, inertia.z);
    ImGui::Text(buffer);
}

void PhysXInspectorWidgetImpl::drawRigidJoints(::physx::PxRigidBody* rigid)
{
    JointBodyMap::iterator bodyItr = mInspector.mJointBodyMap.find(pxr::SdfPath(rigid->getName()));
    if (bodyItr != mInspector.mJointBodyMap.end())
    {
        const pxr::SdfPathSet& jointPaths = bodyItr->second;
        int numJoint = 0;
        for (auto jointPath : jointPaths)
        {
            ImGui::PushID(numJoint++);
            const char* partialName = strrchr(jointPath.GetText(), '/') + 1;
            drawSelectableTreenodeLeaf(jointPath.GetText(), partialName);
            drawSelectionJointsContextMenu(jointPath);
            ImGui::NextColumn();
            ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
            drawJointSlider(jointPath);
            ImGui::PopItemWidth();
            ImGui::NextColumn();
            ImGui::PopID();
        }
    }
}

void PhysXInspectorWidgetImpl::drawSceneHierarchy(::physx::PxScene* scene)
{
    auto model = std::static_pointer_cast<PhysXInspectorModelImpl>(getModel());
    ImGui::Indent(10);
    const PxU32 nbRigidDynamic = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
    if (nbRigidDynamic > 0 && ImGui::CollapsingHeader("Dynamic bodies", ImGuiTreeNodeFlags_None))
    {
        ImGui::Columns(2, "columns", true);
        for (PxU32 i = 0; i < nbRigidDynamic; i++)
        {
            PxActor* pxActor = nullptr;
            scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &pxActor, 1, i);

            if (!pxActor)
                continue;
            ImGui::PushID((int)i);

            PxRigidDynamic* rigid = (PxRigidDynamic*)pxActor;
            if (!model->isIncludedInCurrentSelection(rigid->userData))
                continue;
            pxr::SdfPath rigidPath =
                mPhysXInterface->getPhysXObjectUsdPath((omni::physx::usdparser::ObjectId)rigid->userData);
            const char* rigidName = strrchr(rigidPath.GetText(), '/') + 1;
            bool opened = drawSelectableTreenode(rigidPath.GetText(), rigidName);
            drawSelectionBodiesContextMenu(rigidPath);
            ImGui::NextColumn();
            drawRigidMassLabel(rigid);
            ImGui::NextColumn();
            if (opened)
            {
                drawRigidJoints(rigid);
                drawRigidShapes(rigid);

                ImGui::TreePop();
            }
            ImGui::PopID();
        }
        ImGui::Columns(1, "columns", false);
    }

    const PxU32 nbRigidStatic = scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC);
    if (nbRigidStatic > 0 && ImGui::CollapsingHeader("Static bodies", ImGuiTreeNodeFlags_None))
    {
        for (PxU32 i = 0; i < nbRigidStatic; i++)
        {
            PxActor* pxActor = nullptr;
            scene->getActors(PxActorTypeFlag::eRIGID_STATIC, &pxActor, 1, i);
            if (!pxActor)
                continue;
            PxRigidStatic* rigid = (PxRigidStatic*)pxActor;
            if (!model->isIncludedInCurrentSelection(rigid->userData))
                continue;
            ImGui::PushID((int)i);
            pxr::SdfPath rigidPath =
                mPhysXInterface->getPhysXObjectUsdPath((omni::physx::usdparser::ObjectId)rigid->userData);
            const char* name = strrchr(rigidPath.GetText(), '/') + 1;
            drawSelectableTreenodeLeaf(rigidPath.GetText(), name);
            ImGui::PopID();
        }
    }
    const PxU32 nbArticulations = scene->getNbArticulations();
    if (nbArticulations > 0 && ImGui::CollapsingHeader("Articulations", ImGuiTreeNodeFlags_None))
    {
        PxArticulationReducedCoordinate* articulation = nullptr;
        for (PxU32 i = 0; i < nbArticulations; i++)
        {
            scene->getArticulations(&articulation, 1, i);
            if (!model->isIncludedInCurrentSelection(articulation->userData))
                continue;
            ImGui::PushID((int)i);
            drawArticulationHierarchy(articulation);
            ImGui::PopID();
        }
    }
    drawMaterials(scene->getPhysics());
    ImGui::Unindent(10);
    ImGui::Dummy(ImVec2(0, 40));
}

void PhysXInspectorWidgetImpl::drawMaterials(::physx::PxPhysics& physics)
{
    if (!mInspector.mMaterialsToShapes.empty() && ImGui::CollapsingHeader("Materials", ImGuiTreeNodeFlags_None))
    {
        ImGui::Columns(2, "columns", true);
        int id = 0;
        for (auto& material : mInspector.mMaterialsToShapes)
        {
            ImGui::PushID(id++);
            bool opened;
            if (material.first.IsEmpty())
            {
                opened = ImGui::TreeNodeEx("Default Material", ImGuiTreeNodeFlags_OpenOnArrow |
                                                                    ImGuiTreeNodeFlags_OpenOnDoubleClick |
                                                                    ImGuiTreeNodeFlags_SpanAvailWidth);
            }
            else
            {
                const char* materialName = strrchr(material.first.GetText(), '/') + 1;
                opened = drawSelectableTreenode(material.first.GetText(), materialName);
            }
            ImGui::NextColumn();
            ImGui::Text("(%d shapes)", (int)material.second.size());
            ImGui::NextColumn();
            if (opened)
            {
                for (auto& shape : material.second)
                {
                    pxr::SdfPath shapePath =
                        mPhysXInterface->getPhysXObjectUsdPath((omni::physx::usdparser::ObjectId)shape->userData);
                    const char* shapeName = strrchr(shapePath.GetText(), '/') + 1;
                    drawSelectableTreenodeLeaf(shapePath.GetText(), shapeName);
                    ImGui::NextColumn();
                    drawShapeLabel(shape);
                    ImGui::NextColumn();
                }
                ImGui::TreePop();
            }
            ImGui::PopID();
        }
        const PxU32 nbAllMAterials = physics.getNbMaterials();
        for (PxU32 materialIdx = 0; materialIdx < nbAllMAterials; ++materialIdx)
        {
            PxMaterial* material;
            physics.getMaterials(&material, 1, materialIdx);
            pxr::SdfPath materialPath =
                mPhysXInterface->getPhysXObjectUsdPath((omni::physx::usdparser::ObjectId)material->userData);

            if (mInspector.mMaterialsToShapes.find(materialPath) == mInspector.mMaterialsToShapes.end())
            {
                const char* materialName = strrchr(materialPath.GetText(), '/') + 1;
                drawSelectableTreenodeLeaf(materialPath.GetText(), materialName);
                ImGui::NextColumn();
                ImGui::Text("[unused by current selection]");
                ImGui::NextColumn();
            }
        }
        ImGui::Columns(1, "columns", false);
    }
}

void PhysXInspectorWidgetImpl::drawJointSlider(const pxr::SdfPath& jointPath)
{
    pxr::UsdPrim prim = mInspector.mStage->GetPrimAtPath(jointPath);
    float lowLimit, highLimit, jointValue;
    auto model = std::static_pointer_cast<PhysXInspectorModelImpl>(getModel());
    if (mInspector.USDGetJointLimitsClamped(prim, lowLimit, highLimit))
    {
        if (model->getJointValue(jointPath.GetText(), jointValue))
        {
            if (ImGui::SliderFloat("##joint", &jointValue, lowLimit, highLimit, nullptr, 1.0f))
            {
                model->setJointValue(jointPath.GetText(), jointValue);
            }
            if (ImGui::IsItemActive())
            {
                mInspector.stepIfNecessary();
            }
        }
        else
        {
            ImGui::Text("[cannot modify joint]");
        }
    }
    else if (prim.HasAPI<pxr::PhysxSchemaPhysxMimicJointAPI>())
    {
        ImGui::Text("Mimic Joint");
    }
    else if (prim.IsA<pxr::UsdPhysicsSphericalJoint>())
    {
        ImGui::Text("Spherical Joint");
    }
    else if (prim.IsA<pxr::UsdPhysicsFixedJoint>())
    {
        ImGui::Text("Fixed Joint");
    }
    else
    {
        ImGui::Text("Unsupported joint type");
    }
}

void PhysXInspectorWidgetImpl::drawArticulation(PhysXInspectorModel::InspectorType inspectorType,
                                                const std::string& currentPath)
{
    ::physx::PxArticulationReducedCoordinate* articulation = mInspector.getArticulationAt(currentPath);
    if (articulation)
    {
        switch (inspectorType)
        {
        case PhysXInspectorModel::InspectorType::eJointsList: {
            ImGui::Columns(2, "columns", true);
            drawArticulationJoints(articulation);
            ImGui::Columns(1, "columns", false);
            break;
        }
        case PhysXInspectorModel::InspectorType::eJointsBodiesHierarchy: {
            if (ImGui::CollapsingHeader("Articulations", ImGuiTreeNodeFlags_None))
            {
                drawArticulationHierarchy(articulation);
            }
            mInspector.mMaterialsToShapes.clear();
            drawMaterials(articulation->getScene()->getPhysics());
            break;
        }
        }
    }
}

void PhysXInspectorWidgetImpl::drawArticulationJoints(::physx::PxArticulationReducedCoordinate* articulation)
{
    ::physx::PxU32 numLinks = articulation->getNbLinks();
    for (::physx::PxU32 idx = 0; idx < numLinks; ++idx)
    {
        ::physx::PxArticulationLink* link = nullptr;
        articulation->getLinks(&link, 1, idx);
        ::physx::PxArticulationJointReducedCoordinate* joint = link->getInboundJoint();
        if (joint != nullptr)
        {
            pxr::SdfPath jointPath =
                mPhysXInterface->getPhysXObjectUsdPath((omni::physx::usdparser::ObjectId)joint->userData);
            ImGui::PushID((int)idx);
            const char* name = strrchr(jointPath.GetText(), '/') + 1;
            drawSelectableTreenodeLeaf(jointPath.GetText(), name);
            drawSelectionJointsContextMenu(jointPath);
            ImGui::NextColumn();
            ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
            drawJointSlider(jointPath);
            ImGui::PopItemWidth();
            ImGui::NextColumn();
            ImGui::PopID();
        }
    }
}

void PhysXInspectorWidgetImpl::drawArticulationHierarchy(::physx::PxArticulationReducedCoordinate* articulation)
{
    if (drawSelectableTreenode(articulation->getName(), articulation->getName()))
    {
        ImGui::Columns(2, "columns", true);
        ::physx::PxU32 numLinks = articulation->getNbLinks();
        for (::physx::PxU32 idx = 0; idx < numLinks; ++idx)
        {
            ::physx::PxArticulationLink* link = nullptr;
            articulation->getLinks(&link, 1, idx);
            if (link->getInboundJoint() == nullptr)
            {
                // root link
                drawLinkNode(link);
                break;
            }
        }
        ImGui::Columns(1, "columns", false);
        ImGui::TreePop();
    }
}

void PhysXInspectorWidgetImpl::drawArticulationJoint(::physx::PxArticulationJointReducedCoordinate* joint)
{
    pxr::SdfPath jointPath = mPhysXInterface->getPhysXObjectUsdPath((omni::physx::usdparser::ObjectId)joint->userData);
    const char* jointName = strrchr(jointPath.GetText(), '/') + 1;
    bool opened = drawSelectableTreenode(jointPath.GetText(), jointName);
    drawSelectionJointsContextMenu(jointPath);
    ImGui::NextColumn();
    ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
    drawJointSlider(jointPath);
    ImGui::PopItemWidth();
    ImGui::NextColumn();

    if (opened)
    {
        drawLinkNode(&joint->getChildArticulationLink());
        ImGui::TreePop();
    }
}

void PhysXInspectorWidgetImpl::drawLinkNode(::physx::PxArticulationLink* link)
{
    pxr::SdfPath path = mPhysXInterface->getPhysXObjectUsdPath((omni::physx::usdparser::ObjectId)link->userData);
    const char* name = strrchr(path.GetText(), '/') + 1;
    bool opened = drawSelectableTreenode(path.GetText(), name);
    drawSelectionBodiesContextMenu(path);
    ImGui::NextColumn();
    drawRigidMassLabel(link);
    ImGui::NextColumn();
    if (opened)
    {
        drawLinkChildren(link);
        drawRigidShapes(link);
        ImGui::TreePop();
    }
}

void PhysXInspectorWidgetImpl::drawLinkChildren(::physx::PxArticulationLink* link)
{
    ::physx::PxU32 numChildren = link->getNbChildren();
    for (::physx::PxU32 childIdx = 0; childIdx < numChildren; ++childIdx)
    {
        ::physx::PxArticulationLink* childLink = nullptr;
        link->getChildren(&childLink, 1, childIdx);
        ImGui::PushID((int)childIdx);
        drawArticulationJoint(childLink->getInboundJoint());
        ImGui::PopID();
    }
}

void PhysXInspectorWidgetImpl::drawShapeLabel(::physx::PxShape* shape)
{
    ImGui::Text(PhysXInspector::getPhysXShapeDescription(shape));
}

void PhysXInspectorWidgetImpl::drawRigidShapes(::physx::PxRigidActor* rigid)
{
    ::physx::PxU32 numShapes = rigid->getNbShapes();
    char buffer[100];
    snprintf(buffer, sizeof(buffer) - 1, "%d (items)", numShapes);
    if (numShapes == 0)
    {
        // No Shapes
    }
    else if (ImGui::TreeNode("Shapes"))
    {
        ImGui::NextColumn();
        ImGui::Text(buffer);
        ImGui::NextColumn();

        for (::physx::PxU32 shapeIdx = 0; shapeIdx < numShapes; ++shapeIdx)
        {
            ::physx::PxShape* shape = nullptr;
            rigid->getShapes(&shape, 1, shapeIdx);
            ImGui::PushID((int)shapeIdx);
            const char* fullPath = shape->getName();
            const char* name = strrchr(fullPath, '/') + 1;
            drawSelectableTreenodeLeaf(fullPath, name);
            ImGui::NextColumn();
            drawShapeLabel(shape);
            ImGui::NextColumn();
            ImGui::PopID();
        }
        ImGui::TreePop();
    }
    else
    {
        ImGui::NextColumn();
        ImGui::Text(buffer);
        ImGui::NextColumn();
    }
}
OMNIUI_NAMESPACE_CLOSE_SCOPE

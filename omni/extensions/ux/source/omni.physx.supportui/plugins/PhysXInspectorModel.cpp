// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"
// clang-format off
#include <private/omni/physx/IPhysxUsdLoad.h>
#include <common/foundation/Allocator.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/Selection.h>
#include <carb/logging/Log.h>
#include <usdrt/scenegraph/usd/usd/stage.h>
#include <omni/fabric/usd/PathConversion.h>
#include <carb/Format.h>
#include "PhysXInspectorModel.h"
#include "PhysXInspector.h"
// clang-format on

using namespace physx;

PhysXInspectorItem::PhysXInspectorItem(const char* name, const char* path, Type type)
    : type(type),
      nameModel(omni::ui::SimpleStringModel::create(name)),
      usdPathModel(omni::ui::SimpleStringModel::create(path)),
      numericValue(omni::ui::SimpleFloatModel::create(0.0)),
      minValue(omni::ui::SimpleFloatModel::create(std::numeric_limits<double>::infinity())),
      maxValue(omni::ui::SimpleFloatModel::create(std::numeric_limits<double>::infinity())),
      stiffnessModel(omni::ui::SimpleFloatModel::create(std::numeric_limits<double>::infinity())),
      dampingModel(omni::ui::SimpleFloatModel::create(std::numeric_limits<double>::infinity()))
{
}

PhysXInspectorModelImpl::PhysXInspectorModelImpl(const pxr::SdfPathVector& selection, PhysXInspector* inspector)
{
    mInspector = inspector;
    mSelectedPrimStringModel = omni::ui::SimpleStringModel::create();
    mShowMassesAndInertia = omni::ui::SimpleBoolModel::create(false);
    mFixArticulationBase = omni::ui::SimpleBoolModel::create(true);
    mDataShapeModel = omni::ui::SimpleIntModel::create();
    mDataShapeModel->SimpleNumericModel<int64_t>::setValue(static_cast<int64_t>(DataShapeType::eHierarchical));
    mControlTypeModel =
        omni::ui::SimpleStringModel::create(carb::fmt::format("{}", static_cast<int>(ControlType::eAutomatic)));
    mPropertyTypeModel =
        omni::ui::SimpleStringModel::create(carb::fmt::format("{}", static_cast<int>(PropertyType::eShowLimits)));
    mFixArticulationBase->addValueChangedFn(
        [this](auto) { fixArticulationsBase(mFixArticulationBase->getValueAsBool()); });
}

PhysXInspectorModelImpl::~PhysXInspectorModelImpl()
{
    mInspector->onModelDestroyed(this);
    releaseArticulationParsing();
}

std::shared_ptr<omni::ui::SimpleBoolModel> PhysXInspectorModelImpl::getEnableGravity() const
{
    return mInspector->mEnableGravity;
}

std::shared_ptr<omni::ui::SimpleBoolModel> PhysXInspectorModelImpl::getEnableQuasiStaticMode() const
{
    return mInspector->mEnableQuasiStaticMode;
}

void PhysXInspectorModelImpl::fixArticulationsBase(bool fixBases)
{
    int idx = 0;
    for (pxr::SdfPath path : mAllArticulations)
    {
        PxArticulationReducedCoordinate* articulation;
        articulation = static_cast<PxArticulationReducedCoordinate*>(
            mInspector->mPhysXInterface->getPhysXPtr(path, omni::physx::ePTArticulation));
        if (articulation)
        {
            if (!mIsArticulationAllowedToBeFixed.empty() && mIsArticulationAllowedToBeFixed[idx])
            {
                articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, fixBases);
                if (fixBases)
                {
                    articulation->wakeUp();
                }
            }
        }
        idx++;
    }
}

void PhysXInspectorModelImpl::parseArticulations(uint64_t stageId)
{
    releaseArticulationParsing();
    auto usdLoad = carb::getCachedInterface<omni::physx::usdparser::IPhysxUsdLoad>();
    if (pxr::SdfPath::IsValidPathString(mSelectedPrimPath))
    {
        mArticulations = usdLoad->parseArticulations(stageId, pxr::SdfPath(mSelectedPrimPath));
    }
}

void PhysXInspectorModelImpl::releaseArticulationParsing()
{
    for (auto& it : mArticulations)
    {
        // releaseDesc will only call ICE_FREE that doesn't call destructor correctly so we use ICE_PLACEMENT_DELETE
        ICE_PLACEMENT_DELETE(it, PhysxArticulationDesc);
    }
    mArticulations.clear();
}

std::shared_ptr<PhysXInspectorItem> PhysXInspectorModelImpl::makeInspectorItem(const char* name,
                                                                               const char* path,
                                                                               PhysXInspectorItem::Type type)
{
    auto node = std::make_shared<PhysXInspectorItem>(name, path, type);
    return node;
}

std::shared_ptr<PhysXInspectorItem> PhysXInspectorModelImpl::addJoint(PhysXInspectorItem& item,
                                                                      const pxr::SdfPath& usdPath,
                                                                      PhysXInspectorItem::Type type)
{
    const char* name = strrchr(usdPath.GetText(), '/') + 1;
    auto child = makeInspectorItem(name, usdPath.GetText(), type);
    child->usdPath = usdPath;
    item.children.push_back(child);
    auto downcast = std::dynamic_pointer_cast<PhysXInspectorItem>(child);
    PhysXInspectorItem* typedChild = &(*downcast);
    mAllJoints.push_back(typedChild);
    return child;
}

void PhysXInspectorModelImpl::addArticulationJoints(PhysXInspectorItem& item,
                                                    ::physx::PxArticulationReducedCoordinate* articulation)
{
    PxU32 numLinks = articulation->getNbLinks();
    for (PxU32 idx = 0; idx < numLinks; ++idx)
    {
        PxArticulationLink* link = nullptr;
        articulation->getLinks(&link, 1, idx);
        PxArticulationJointReducedCoordinate* joint = link->getInboundJoint();
        if (joint != nullptr)
        {
            pxr::SdfPath usdPath =
                mInspector->mPhysXInterface->getPhysXObjectUsdPath((omni::physx::usdparser::ObjectId)joint->userData);
            addJoint(item, usdPath, PhysXInspectorItem::Type::ArticulationJoint);
        }
    }
}

void PhysXInspectorModelImpl::addArticulationHierarchy(PhysXInspectorItem& parent)
{
    PxArticulationReducedCoordinate* articulation = static_cast<PxArticulationReducedCoordinate*>(
        mInspector->mPhysXInterface->getPhysXPtr(parent.usdPath, omni::physx::ePTArticulation));

    ::physx::PxArticulationLink* rootLink = findRootLink(articulation);
    if (rootLink)
    {
        addArticulationHierarchyLink(parent, rootLink);
    }
}

::physx::PxArticulationLink* PhysXInspectorModelImpl::findRootLink(::physx::PxArticulationReducedCoordinate* articulation)
{
    if (articulation->getNbLinks() > 0)
    {
        ::physx::PxArticulationLink* link = nullptr;
        articulation->getLinks(&link, 1, 0);
        return link;
    }
    return nullptr;
}

void PhysXInspectorModelImpl::addArticulationHierarchyLink(PhysXInspectorItem& parent, ::physx::PxArticulationLink* link)
{
    const char* name = strrchr(link->getName(), '/') + 1;
    auto linkNode = makeInspectorItem(name, link->getName(), PhysXInspectorItem::Type::ArticulationLink);
    linkNode->usdPath = pxr::SdfPath(link->getName());
    parent.children.push_back(linkNode);
    ::physx::PxU32 numChildren = link->getNbChildren();
    for (::physx::PxU32 childIdx = 0; childIdx < numChildren; ++childIdx)
    {
        ::physx::PxArticulationLink* childLink = nullptr;
        link->getChildren(&childLink, 1, childIdx);
        addArticulationHierarchyJoint(*linkNode, childLink->getInboundJoint());
        addRigidShapes(*linkNode, childLink);
        std::string massVisualization;
        char massBuffer[100];
        const physx::PxVec3 inertia = childLink->getMassSpaceInertiaTensor();
        snprintf(massBuffer, sizeof(massBuffer) - 1, "[m=%.2f I=[%.4f,%.4f,%.4f]]", childLink->getMass(), inertia.x,
                 inertia.y, inertia.z);
        linkNode->detailAsTextValueModel = omni::ui::SimpleStringModel::create(massBuffer);
    }
}

void PhysXInspectorModelImpl::addArticulationHierarchyJoint(PhysXInspectorItem& parent,
                                                            ::physx::PxArticulationJointReducedCoordinate* joint)
{
    pxr::SdfPath jointPath =
        mInspector->mPhysXInterface->getPhysXObjectUsdPath((omni::physx::usdparser::ObjectId)joint->userData);
    auto jointNode = addJoint(parent, jointPath, PhysXInspectorItem::Type::ArticulationJoint);
    addArticulationHierarchyLink(*jointNode, &joint->getChildArticulationLink());
}

void PhysXInspectorModelImpl::addRigidShapes(PhysXInspectorItem& parent, ::physx::PxRigidActor* rigid)
{
    ::physx::PxU32 numShapes = rigid->getNbShapes();
    if (numShapes > 0)
    {
        auto shapesNode = makeInspectorItem("Colliders", "", PhysXInspectorItem::Type::Group);
        parent.children.push_back(shapesNode);
        for (::physx::PxU32 shapeIdx = 0; shapeIdx < numShapes; ++shapeIdx)
        {
            ::physx::PxShape* shape = nullptr;
            rigid->getShapes(&shape, 1, shapeIdx);

            const char* name = strrchr(shape->getName(), '/') + 1;
            auto shapeNode = makeInspectorItem(name, shape->getName(), PhysXInspectorItem::Type::Collider);
            shapeNode->usdPath = pxr::SdfPath(shape->getName());
            shapesNode->children.push_back(shapeNode);
            const char* shapeType = PhysXInspector::getPhysXShapeDescription(shape);
            shapeNode->detailAsTextValueModel = omni::ui::SimpleStringModel::create(shapeType);
        }
    }
}

void PhysXInspectorModelImpl::addRigidJoints(PhysXInspectorItem& parent, ::physx::PxRigidActor* rigid)
{
    JointBodyMap::iterator bodyItr = mInspector->mJointBodyMap.find(pxr::SdfPath(rigid->getName()));
    if (bodyItr != mInspector->mJointBodyMap.end())
    {
        auto jointsNode = makeInspectorItem("Joints", "", PhysXInspectorItem::Type::Group);
        parent.children.push_back(jointsNode);
        const pxr::SdfPathSet& jointPaths = bodyItr->second;
        int numJoint = 0;
        for (auto jointPath : jointPaths)
        {
            addJoint(*jointsNode, jointPath, PhysXInspectorItem::Type::MaximalJoint);
        }
    }
}

void PhysXInspectorModelImpl::addArticulations(::physx::PxScene* scene)
{
    const PxU32 nbArticulations = scene->getNbArticulations();
    std::shared_ptr<PhysXInspectorItem> articulationsNode;
    mAllArticulations.clear();
    mIsArticulationAllowedToBeFixed.clear();
    for (PxU32 i = 0; i < nbArticulations; i++)
    {
        PxArticulationReducedCoordinate* articulation = nullptr;
        scene->getArticulations(&articulation, 1, i);
        if (!isIncludedInCurrentSelection(articulation->userData))
            continue;
        mAllArticulations.push_back(getPathForUserData(articulation->userData));
        const char* name = strrchr(articulation->getName(), '/') + 1;
        auto articulationNode = makeInspectorItem(name, articulation->getName(), PhysXInspectorItem::Type::Articulation);
        if (!articulationsNode)
        {
            articulationsNode = makeInspectorItem("Articulations", "", PhysXInspectorItem::Type::Group);
            _children.push_back(articulationsNode);
        }
        articulationNode->usdPath = pxr::SdfPath(articulation->getName());
        addArticulationHierarchy(*articulationNode);
        articulationsNode->children.push_back(articulationNode);
    }
}

void PhysXInspectorModelImpl::addDynamicBodies(::physx::PxScene* scene)
{
    const PxU32 nbRigidDynamic = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
    std::shared_ptr<PhysXInspectorItem> dynamicBodiesNode;
    for (PxU32 i = 0; i < nbRigidDynamic; i++)
    {
        PxActor* pxActor = nullptr;
        scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &pxActor, 1, i);

        if (!pxActor)
            continue;

        PxRigidDynamic* rigid = (PxRigidDynamic*)pxActor;
        if (!isIncludedInCurrentSelection(rigid->userData))
            continue;

        const char* name = strrchr(rigid->getName(), '/') + 1;
        auto rigidDynamic = makeInspectorItem(name, rigid->getName(), PhysXInspectorItem::Type::Group);
        rigidDynamic->usdPath = pxr::SdfPath(rigid->getName());
        char massBuffer[100];
        const physx::PxVec3 inertia = rigid->getMassSpaceInertiaTensor();
        snprintf(massBuffer, sizeof(massBuffer) - 1, "[m=%.2f I=[%.4f,%.4f,%.4f]]", rigid->getMass(), inertia.x,
                 inertia.y, inertia.z);
        rigidDynamic->detailAsTextValueModel = omni::ui::SimpleStringModel::create(massBuffer);
        if (!dynamicBodiesNode)
        {
            dynamicBodiesNode = makeInspectorItem("Dynamic Rigids", "", PhysXInspectorItem::Type::Group);
            _children.push_back(dynamicBodiesNode);
        }
        dynamicBodiesNode->children.push_back(rigidDynamic);
        addRigidJoints(*rigidDynamic, rigid);
        addRigidShapes(*rigidDynamic, rigid);
    }
}

void PhysXInspectorModelImpl::addStaticBodies(::physx::PxScene* scene)
{
    const PxU32 nbRigidStatic = scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC);
    std::shared_ptr<PhysXInspectorItem> staticBodiesNode;
    for (PxU32 i = 0; i < nbRigidStatic; i++)
    {
        PxActor* pxActor = nullptr;
        scene->getActors(PxActorTypeFlag::eRIGID_STATIC, &pxActor, 1, i);
        if (!pxActor)
            continue;
        PxRigidStatic* rigid = (PxRigidStatic*)pxActor;
        if (!isIncludedInCurrentSelection(rigid->userData))
            continue;
        const char* name = strrchr(rigid->getName(), '/') + 1;
        auto rigidStatic = makeInspectorItem(name, rigid->getName(), PhysXInspectorItem::Type::Collider);
        rigidStatic->usdPath = pxr::SdfPath(rigid->getName());
        if (!staticBodiesNode)
        {
            staticBodiesNode = makeInspectorItem("Static Colliders", "", PhysXInspectorItem::Type::Group);
            _children.push_back(staticBodiesNode);
        }
        staticBodiesNode->children.push_back(rigidStatic);
    }
}

void PhysXInspectorModelImpl::addMaterials(::physx::PxScene* scene)
{
    std::shared_ptr<PhysXInspectorItem> materialsHeader;
    for (auto& material : mInspector->mMaterialsToShapes)
    {
        std::shared_ptr<PhysXInspectorItem> materialNode;
        for (auto& shape : material.second)
        {
            if (!isIncludedInCurrentSelection(shape->userData))
                continue;
            pxr::SdfPath shapePath =
                mInspector->mPhysXInterface->getPhysXObjectUsdPath((omni::physx::usdparser::ObjectId)shape->userData);
            if (!materialNode)
            {
                if (!materialsHeader)
                {
                    materialsHeader = makeInspectorItem("Materials", "", PhysXInspectorItem::Type::Group);
                    _children.push_back(materialsHeader);
                }
                if (material.first.IsEmpty())
                {
                    materialNode = makeInspectorItem("Default Material", "", PhysXInspectorItem::Type::Material);
                    materialsHeader->children.push_back(materialNode);
                }
                else
                {
                    const char* materialName = strrchr(material.first.GetText(), '/') + 1;
                    materialNode =
                        makeInspectorItem(materialName, material.first.GetText(), PhysXInspectorItem::Type::Material);
                    materialNode->usdPath = material.first;
                    materialsHeader->children.push_back(materialNode);
                }
            }
            const char* shapeName = strrchr(shapePath.GetText(), '/') + 1;
            auto shapeNode = makeInspectorItem(shapeName, shapePath.GetText(), PhysXInspectorItem::Type::Collider);
            shapeNode->usdPath = shapePath;
            materialNode->children.push_back(shapeNode);
        }
    }
}
void PhysXInspectorModelImpl::findCollisionAPIFor(const pxr::SdfPath& path, pxr::SdfPathVector& collisionApis)
{
    const pxr::UsdPhysicsCollisionAPI collisionApi = pxr::UsdPhysicsCollisionAPI::Get(mInspector->mStage, path);
    if (collisionApi)
    {
        collisionApis.push_back(path);
    }
    else
    {
        // We have to find all colliders down
        pxr::UsdPrimRange primRange(mInspector->mStage->GetPrimAtPath(path));
        for (pxr::UsdPrimRange::const_iterator iter = primRange.begin(); iter != primRange.end(); ++iter)
        {
            const pxr::UsdPrim& subPrim = *iter;
            const pxr::UsdPhysicsCollisionAPI subCollisionApi =
                pxr::UsdPhysicsCollisionAPI::Get(mInspector->mStage, subPrim.GetPath());
            if (subCollisionApi)
            {
                collisionApis.push_back(subPrim.GetPath());
            }
        }
    }
}


void PhysXInspectorModelImpl::selectAllConnectedLinks()
{
    const auto usdContext = omni::usd::UsdContext::getContext();
    auto selectionVector = usdContext->getSelection()->getSelectedPrimPathsV2(omni::usd::Selection::SourceType::eUsd);
    pxr::SdfPathVector addToSelection;
    for (pxr::SdfPath& path : selectionVector)
    {
        if (!isIncludedInCurrentSelection(path))
            continue;
        pxr::UsdPrim prim = mInspector->mStage->GetPrimAtPath(path);
        const bool existsInInspector = mInspector->mPhysXInterface->getObjectId(path, omni::physx::ePTJoint) !=
                                           omni::physx::usdparser::kInvalidObjectId ||
                                       mInspector->mPhysXInterface->getObjectId(path, omni::physx::ePTLinkJoint) !=
                                           omni::physx::usdparser::kInvalidObjectId;
        if (existsInInspector && prim.IsA<pxr::UsdPhysicsJoint>())
        {
            pxr::UsdPhysicsJoint physicsJoint(prim);
            pxr::SdfPathVector target;
            physicsJoint.GetBody0Rel().GetTargets(&target);
            addToSelection.insert(addToSelection.end(), target.begin(), target.end());
            target.clear();
            physicsJoint.GetBody1Rel().GetTargets(&target);
            addToSelection.insert(addToSelection.end(), target.begin(), target.end());
        }
    }
    usdContext->getSelection()->setSelectedPrimPathsV2(addToSelection);
}

void PhysXInspectorModelImpl::selectAllConnectedJointShapes()
{
    const auto usdContext = omni::usd::UsdContext::getContext();
    auto selectionVector = usdContext->getSelection()->getSelectedPrimPathsV2(omni::usd::Selection::SourceType::eUsd);
    pxr::SdfPathVector collisionApis;
    for (pxr::SdfPath& path : selectionVector)
    {
        if (!isIncludedInCurrentSelection(path))
            continue;
        pxr::UsdPrim prim = mInspector->mStage->GetPrimAtPath(path);
        const bool existsInInspector = mInspector->mPhysXInterface->getObjectId(path, omni::physx::ePTJoint) !=
                                           omni::physx::usdparser::kInvalidObjectId ||
                                       mInspector->mPhysXInterface->getObjectId(path, omni::physx::ePTLinkJoint) !=
                                           omni::physx::usdparser::kInvalidObjectId;
        if (existsInInspector && prim.IsA<pxr::UsdPhysicsJoint>())
        {
            pxr::UsdPhysicsJoint physicsJoint(prim);
            pxr::SdfPathVector target;
            physicsJoint.GetBody0Rel().GetTargets(&target);
            for (auto& path : target)
            {
                findCollisionAPIFor(path, collisionApis);
            }
            target.clear();
            physicsJoint.GetBody1Rel().GetTargets(&target);
            for (auto& path : target)
            {
                findCollisionAPIFor(path, collisionApis);
            }
        }
    }
    usdContext->getSelection()->setSelectedPrimPathsV2(collisionApis);
}

void PhysXInspectorModelImpl::selectAllConnectedBodyShapes()
{
    const auto usdContext = omni::usd::UsdContext::getContext();
    auto selectionVector = usdContext->getSelection()->getSelectedPrimPathsV2(omni::usd::Selection::SourceType::eUsd);
    pxr::SdfPathVector collisionApis;
    for (pxr::SdfPath& path : selectionVector)
    {
        if (!isIncludedInCurrentSelection(path))
            continue;
        pxr::UsdPrim prim = mInspector->mStage->GetPrimAtPath(path);
        const bool existsInInspector = mInspector->mPhysXInterface->getObjectId(path, omni::physx::ePTActor) !=
                                           omni::physx::usdparser::kInvalidObjectId ||
                                       mInspector->mPhysXInterface->getObjectId(path, omni::physx::ePTLink) !=
                                           omni::physx::usdparser::kInvalidObjectId;
        if (existsInInspector && prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>())
        {
            findCollisionAPIFor(path, collisionApis);
        }
    }
    usdContext->getSelection()->setSelectedPrimPathsV2(collisionApis);
}

void PhysXInspectorModelImpl::selectAllConnectedBodyJoints()
{
    const auto usdContext = omni::usd::UsdContext::getContext();
    auto selectionVector = usdContext->getSelection()->getSelectedPrimPathsV2(omni::usd::Selection::SourceType::eUsd);
    pxr::SdfPathVector allJoints;
    for (pxr::SdfPath& path : selectionVector)
    {
        if (!isIncludedInCurrentSelection(path))
            continue;
        pxr::UsdPrim prim = mInspector->mStage->GetPrimAtPath(path);
        const bool existsInInspector = mInspector->mPhysXInterface->getObjectId(path, omni::physx::ePTActor) !=
                                           omni::physx::usdparser::kInvalidObjectId ||
                                       mInspector->mPhysXInterface->getObjectId(path, omni::physx::ePTLink) !=
                                           omni::physx::usdparser::kInvalidObjectId;
        if (existsInInspector && prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>())
        {
            JointBodyMap::iterator bodyItr = mInspector->mJointBodyMap.find(path);
            if (bodyItr != mInspector->mJointBodyMap.end())
            {
                const pxr::SdfPathSet& jointPaths = bodyItr->second;
                allJoints.insert(allJoints.end(), jointPaths.begin(), jointPaths.end());
            }
        }
    }
    usdContext->getSelection()->setSelectedPrimPathsV2(allJoints);
}

void PhysXInspectorModelImpl::recursivegetItemsMatchingPaths(const std::vector<std::string>& paths,
                                                             const std::vector<AbstractItemConstPointer>& lookIntoChildren,
                                                             std::vector<AbstractItemConstPointer>& foundItems)
{
    for (auto& child : lookIntoChildren)
    {
        auto inspectorItem = std::dynamic_pointer_cast<const PhysXInspectorItem>(child);
        if (std::find(paths.begin(), paths.end(), inspectorItem->usdPathModel->getValueAsString()) != paths.end())
        {
            foundItems.push_back(child);
        }
        recursivegetItemsMatchingPaths(paths, inspectorItem->children, foundItems);
    }
}

std::vector<AbstractItemConstPointer> PhysXInspectorModelImpl::getItemsMatchingPaths(const std::vector<std::string>& paths)
{
    std::vector<AbstractItemConstPointer> items;
    recursivegetItemsMatchingPaths(paths, _children, items);
    return items;
}

void PhysXInspectorModelImpl::setInspectorType(InspectorType inspectorType)
{
    if (inspectorType != mInspectorType)
    {
        mInspectorType = inspectorType;
        refreshModelStructure();
    }
}

PhysXInspectorModel::InspectorType PhysXInspectorModelImpl::getInspectorType() const
{
    return mInspectorType;
}

void PhysXInspectorModelImpl::setSelectedPaths(std::vector<std::string> paths)
{
    CARB_PROFILE_ZONE(0, "PhysXInspectorModelImpl::setSelectedPaths");
    mSelection = move(paths);
    mSelectionSdf.clear();
    for (auto& str : mSelection)
    {
        mSelectionSdf.push_back(pxr::SdfPath(str));
    }
    if (mSelection.size() == 1)
    {
        mSelectionIsArticulation = mInspector->findParentArticulationRoot(mSelection.back(), mSelectedPrimPath);
    }
    else
    {
        mSelectedPrimPath.clear();
        mSelectionIsArticulation = false;
    }

    if (mSelectedPrimPath.empty())
    {
        mSelectedPrimPathSdf = pxr::SdfPath();
    }
    else
    {
        mSelectedPrimPathSdf = pxr::SdfPath(mSelectedPrimPath);
    }

    getInspectedPrimStringModel()->setValue(mSelectedPrimPath);
    mInspector->applyChangeSelection();
}

void PhysXInspectorModelImpl::refreshModelStructure()
{
    CARB_PROFILE_ZONE(0, "PhysXInspectorModelImpl::refreshModelStructure");
    mAllJoints.clear();
    _children.clear();
    ::physx::PxScene* scene = mInspector->getSceneAt(mScenePrimPathSdf.GetText());
    if (scene == nullptr)
    {
        if (mSelectionIsArticulation && mInspector->getArticulationAt(mSelectedPrimPath))
        {
            scene = mInspector->getArticulationAt(mSelectedPrimPath)->getScene();
        }
        else
        {
            omni::physx::IPhysx* iPhysXInterface = carb::getCachedInterface<omni::physx::IPhysx>();

            // Find the default created scene
            const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(mInspector->mStage).ToLongInt();
            omni::fabric::IStageReaderWriter* iStageReaderWriter =
                carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
            omni::fabric::ISimStageWithHistory* iSimStageWithHistory =
                carb::getCachedInterface<omni::fabric::ISimStageWithHistory>();
            omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(stageId);

            usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(stageId, stageInProgress);
            if (usdrtStage)
            {
                for (auto& usdrtPath : usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("UsdPhysicsScene")))
                {
                    const omni::fabric::PathC pathC(usdrtPath);
                    const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
                    if (!usdPath.IsEmpty())
                    {
                        scene = static_cast<PxScene*>(iPhysXInterface->getPhysXPtr(usdPath, omni::physx::ePTScene));
                        if (scene != nullptr)
                        {
                            break;
                        }
                    }
                }
            }
        }
    }

    if (scene == nullptr)
    {
        CARB_LOG_ERROR("PhysXInspectorModel - Cannot find a valid PxScene");
        _itemChanged(nullptr); // nullptr signals to refresh the whole model
        return;
    }
    DataShapeType newType;
    if (mInspectorType == InspectorType::eJointsList)
    {
        newType = refreshJointsList(scene);
    }
    else
    {
        newType = refreshHierarchy(scene);
    }
    mDataShapeModel->SimpleNumericModel<int64_t>::setValue(static_cast<int64_t>(newType));
    _itemChanged(nullptr); // nullptr signals to refresh the whole model
    refreshModelValues();
    refreshModelOverrides();
}

void PhysXInspectorModelImpl::refreshModelOverrides()
{
    if (mInspector->getState() != State::eRunningSimulation)
    {
        fixArticulationsBase(mFixArticulationBase->getValueAsBool());
    }
}

void PhysXInspectorModelImpl::refreshModelValues()
{
    CARB_PROFILE_ZONE(0, "PhysXInspectorModelImpl::refreshModelValues");
    for (PhysXInspectorItem* it : mAllJoints)
    {
        pxr::UsdPrim prim = mInspector->mStage->GetPrimAtPath(it->usdPath);
        if (!prim)
        {
            continue;
        }
        float jointValue;
        if (getJointValue(it->usdPath.GetText(), jointValue))
        {
            it->numericValue->SimpleNumericModel::setValue(jointValue);
        }
        {
            float lowerLimit, upperLimit;
            if (mInspector->USDGetJointLimits(prim, lowerLimit, upperLimit))
            {
                it->minValue->SimpleNumericModel::setValue(lowerLimit);
                it->maxValue->SimpleNumericModel::setValue(upperLimit);
            }
        }
        {
            float stiffness, damping;
            if (mInspector->USDGetJointStiffnessDamping(prim, stiffness, damping))
            {
                it->stiffnessModel->SimpleNumericModel::setValue(stiffness);
                it->dampingModel->SimpleNumericModel::setValue(damping);
            }
        }
    }
}

bool PhysXInspectorModelImpl::tryReadJointDrive(pxr::UsdPrim& prim, float& jointValue)
{
    if (prim.IsA<pxr::UsdPhysicsRevoluteJoint>())
    {
        if (prim.HasAPI<pxr::UsdPhysicsDriveAPI>())
        {
            pxr::UsdPhysicsDriveAPI driveAPI = pxr::UsdPhysicsDriveAPI::Get(prim, pxr::UsdPhysicsTokens->angular);
            jointValue = 0.0f;
            driveAPI.GetTargetPositionAttr().Get(&jointValue);
            return true;
        }
    }
    else if (prim.IsA<pxr::UsdPhysicsPrismaticJoint>())
    {
        if (prim.HasAPI<pxr::UsdPhysicsDriveAPI>())
        {
            pxr::UsdPhysicsDriveAPI driveAPI = pxr::UsdPhysicsDriveAPI::Get(prim, pxr::UsdPhysicsTokens->linear);
            jointValue = 0.0f;
            driveAPI.GetTargetPositionAttr().Get(&jointValue);
            return true;
        }
    }
    return false;
}

bool PhysXInspectorModelImpl::tryReadJointVelocity(pxr::UsdPrim& prim, float& jointValue)
{
    if (prim.IsA<pxr::UsdPhysicsRevoluteJoint>())
    {
        if (prim.HasAPI<pxr::UsdPhysicsDriveAPI>())
        {
            pxr::UsdPhysicsDriveAPI driveAPI = pxr::UsdPhysicsDriveAPI::Get(prim, pxr::UsdPhysicsTokens->angular);
            jointValue = 0.0f;
            driveAPI.GetTargetVelocityAttr().Get(&jointValue);
            return true;
        }
    }
    else if (prim.IsA<pxr::UsdPhysicsPrismaticJoint>())
    {
        if (prim.HasAPI<pxr::UsdPhysicsDriveAPI>())
        {
            pxr::UsdPhysicsDriveAPI driveAPI = pxr::UsdPhysicsDriveAPI::Get(prim, pxr::UsdPhysicsTokens->linear);
            jointValue = 0.0f;
            driveAPI.GetTargetVelocityAttr().Get(&jointValue);
            return true;
        }
    }
    return false;
}

bool PhysXInspectorModelImpl::tryReadJointState(pxr::UsdPrim& prim, float& jointValue)
{
    if (prim.IsA<pxr::UsdPhysicsRevoluteJoint>())
    {
        if (prim.HasAPI<pxr::PhysxSchemaJointStateAPI>())
        {
            pxr::PhysxSchemaJointStateAPI jointStateAPI =
                pxr::PhysxSchemaJointStateAPI::Get(prim, pxr::UsdPhysicsTokens->angular);
            jointValue = 0.0f;
            jointStateAPI.GetPositionAttr().Get(&jointValue);
            return true;
        }
    }
    else if (prim.IsA<pxr::UsdPhysicsPrismaticJoint>())
    {
        if (prim.HasAPI<pxr::PhysxSchemaJointStateAPI>())
        {
            pxr::PhysxSchemaJointStateAPI jointStateAPI =
                pxr::PhysxSchemaJointStateAPI::Get(prim, pxr::UsdPhysicsTokens->linear);
            jointValue = 0.0f;
            jointStateAPI.GetPositionAttr().Get(&jointValue);
            return true;
        }
    }
    return false;
}

bool PhysXInspectorModelImpl::tryWriteJointDrive(pxr::UsdPrim& prim, float jointValue)
{
    if (prim.IsA<pxr::UsdPhysicsRevoluteJoint>())
    {
        if (prim.HasAPI<pxr::UsdPhysicsDriveAPI>())
        {
            pxr::UsdPhysicsDriveAPI driveAPI = pxr::UsdPhysicsDriveAPI::Get(prim, pxr::UsdPhysicsTokens->angular);
            driveAPI.CreateTargetPositionAttr().Set(jointValue);
            return true;
        }
    }
    else if (prim.IsA<pxr::UsdPhysicsPrismaticJoint>())
    {
        if (prim.HasAPI<pxr::UsdPhysicsDriveAPI>())
        {
            pxr::UsdPhysicsDriveAPI driveAPI = pxr::UsdPhysicsDriveAPI::Get(prim, pxr::UsdPhysicsTokens->linear);
            driveAPI.CreateTargetPositionAttr().Set(jointValue);
            return true;
        }
    }
    return false;
}


bool PhysXInspectorModelImpl::tryWriteJointVelocity(pxr::UsdPrim& prim, float jointValue)
{
    if (prim.IsA<pxr::UsdPhysicsRevoluteJoint>())
    {
        if (prim.HasAPI<pxr::UsdPhysicsDriveAPI>())
        {
            pxr::UsdPhysicsDriveAPI driveAPI = pxr::UsdPhysicsDriveAPI::Get(prim, pxr::UsdPhysicsTokens->angular);
            driveAPI.CreateTargetVelocityAttr().Set(jointValue);
            return true;
        }
    }
    else if (prim.IsA<pxr::UsdPhysicsPrismaticJoint>())
    {
        if (prim.HasAPI<pxr::UsdPhysicsDriveAPI>())
        {
            pxr::UsdPhysicsDriveAPI driveAPI = pxr::UsdPhysicsDriveAPI::Get(prim, pxr::UsdPhysicsTokens->linear);
            driveAPI.CreateTargetVelocityAttr().Set(jointValue);
            return true;
        }
    }
    return false;
}

bool PhysXInspectorModelImpl::tryWriteJointState(pxr::UsdPrim& prim, float jointValue)
{
    if (prim.IsA<pxr::UsdPhysicsRevoluteJoint>())
    {
        if (prim.HasAPI<pxr::PhysxSchemaJointStateAPI>())
        {
            pxr::PhysxSchemaJointStateAPI jointStateAPI =
                pxr::PhysxSchemaJointStateAPI::Get(prim, pxr::UsdPhysicsTokens->angular);
            jointStateAPI.CreatePositionAttr().Set(jointValue);
            jointStateAPI.CreateVelocityAttr().Set(0.0f);
            return true;
        }
    }
    else if (prim.IsA<pxr::UsdPhysicsPrismaticJoint>())
    {
        if (prim.HasAPI<pxr::PhysxSchemaJointStateAPI>())
        {
            pxr::PhysxSchemaJointStateAPI jointStateAPI =
                pxr::PhysxSchemaJointStateAPI::Get(prim, pxr::UsdPhysicsTokens->linear);
            jointStateAPI.CreatePositionAttr().Set(jointValue);
            jointStateAPI.CreateVelocityAttr().Set(0.0f);
            return true;
        }
    }
    return false;
}

bool PhysXInspectorModelImpl::isArticulationJoint(const pxr::SdfPath& path) const
{
    for (PhysXInspectorItem* joint : mAllJoints)
    {
        if (joint->usdPath == path)
        {
            if (joint->type == PhysXInspectorItem::Type::ArticulationJoint)
            {
                return true;
            }
        }
    }
    return false;
}

const char* PhysXInspectorModelImpl::getJointValueAttributeName(const char* primPath)
{
    pxr::UsdPrim prim = mInspector->mStage->GetPrimAtPath(pxr::SdfPath(primPath));
    if (!prim.IsValid())
    {
        return "Invalid prim path";
    }

    const ControlType controlType = static_cast<ControlType>(mControlTypeModel->getValueAsInt());
    switch (controlType)
    {
    case ControlType::eAutomatic: {
        const bool isArticulation = isArticulationJoint(prim.GetPath());
        if (prim.IsA<pxr::UsdPhysicsRevoluteJoint>())
        {
            if (prim.HasAPI<pxr::UsdPhysicsDriveAPI>())
            {
                return "drive:angular:physics:targetPosition";
            }
            else if (isArticulation && prim.HasAPI<pxr::PhysxSchemaJointStateAPI>())
            {
                return "state:angular:physics:position";
            }
        }
        else if (prim.IsA<pxr::UsdPhysicsPrismaticJoint>())
        {
            if (prim.HasAPI<pxr::UsdPhysicsDriveAPI>())
            {
                return "drive:angular:physics:targetPosition";
            }
            else if (isArticulation && prim.HasAPI<pxr::PhysxSchemaJointStateAPI>())
            {
                return "state:linear:physics:position";
            }
        }
        return "No DriveAPI";
    }
    case ControlType::eJointState: {
        if (!isArticulationJoint(prim.GetPath()))
        {
            return "No JointStateAPI (Maximal Joint)";
        }
        if (prim.IsA<pxr::UsdPhysicsRevoluteJoint>() && prim.HasAPI<pxr::PhysxSchemaJointStateAPI>())
        {
            return "state:angular:physics:position";
        }
        else if (prim.IsA<pxr::UsdPhysicsPrismaticJoint>() && prim.HasAPI<pxr::PhysxSchemaJointStateAPI>())
        {
            return "state:linear:physics:position";
        }
        return "No JointStateAPI";
    }
    case ControlType::eJointDrive: {
        if (prim.IsA<pxr::UsdPhysicsRevoluteJoint>() && prim.HasAPI<pxr::UsdPhysicsDriveAPI>())
        {
            return "drive:angular:physics:targetPosition";
        }
        else if (prim.IsA<pxr::UsdPhysicsPrismaticJoint>() && prim.HasAPI<pxr::UsdPhysicsDriveAPI>())
        {
            return "drive:linear:physics:targetPosition";
        }
        return "No DriveAPI";
    }
    case ControlType::eJointVelocity:
        if (prim.IsA<pxr::UsdPhysicsRevoluteJoint>() && prim.HasAPI<pxr::UsdPhysicsDriveAPI>())
        {
            return "drive:angular:physics:targetVelocity";
        }
        else if (prim.IsA<pxr::UsdPhysicsPrismaticJoint>() && prim.HasAPI<pxr::UsdPhysicsDriveAPI>())
        {
            return "drive:linear:physics:targetVelocity";
        }
        return "No DriveAPI";
    }
    return "Invalid ControlType";
}

bool PhysXInspectorModelImpl::getJointValue(const char* primPath, float& jointValue)
{
    CARB_PROFILE_ZONE(0, "PhysXInspectorModelImpl::getJointValue");
    pxr::UsdPrim prim = mInspector->mStage->GetPrimAtPath(pxr::SdfPath(primPath));
    if (!prim.IsValid())
    {
        return false;
    }
    const ControlType controlType = static_cast<ControlType>(mControlTypeModel->getValueAsInt());
    switch (controlType)
    {
    case ControlType::eAutomatic: {
        if (!tryReadJointDrive(prim, jointValue))
        {
            if (!tryReadJointState(prim, jointValue))
            {
                return false;
            }
        }
        return true;
    }
    break;
    case ControlType::eJointState:
        return tryReadJointState(prim, jointValue);
    case ControlType::eJointDrive:
        return tryReadJointDrive(prim, jointValue);
    case ControlType::eJointVelocity:
        return tryReadJointVelocity(prim, jointValue);
    }
    return false;
}

bool PhysXInspectorModelImpl::setJointValue(const char* primPath, float jointValue)
{
    CARB_PROFILE_ZONE(0, "PhysXInspectorModelImpl::setJointValue");
    pxr::UsdPrim prim = mInspector->mStage->GetPrimAtPath(pxr::SdfPath(primPath));
    if (!prim.IsValid())
    {
        return false;
    }
    mInspector->enableNoticeHandler(false);
    bool result = false;

    const ControlType controlType = static_cast<ControlType>(mControlTypeModel->getValueAsInt());
    switch (controlType)
    {
    case ControlType::eAutomatic: {
        result = tryWriteJointDrive(prim, jointValue);
        if (!result)
        {
            result = tryWriteJointState(prim, jointValue);
        }
    }
    break;
    case ControlType::eJointState:
        result = tryWriteJointState(prim, jointValue);
        if (result)
        {
            tryWriteJointDrive(prim, jointValue);
        }
        break;
    case ControlType::eJointDrive:
        result = tryWriteJointDrive(prim, jointValue);
        break;
    case ControlType::eJointVelocity:
        result = tryWriteJointVelocity(prim, jointValue);
        break;
    }

    mInspector->enableNoticeHandler(true);
    return result;
}


PhysXInspectorModel::DataShapeType PhysXInspectorModelImpl::refreshHierarchy(::physx::PxScene* scene)
{
    addArticulations(scene);
    addDynamicBodies(scene);
    addStaticBodies(scene);
    addMaterials(scene);
    return DataShapeType::eHierarchical;
}

PhysXInspectorModel::DataShapeType PhysXInspectorModelImpl::refreshJointsList(::physx::PxScene* scene)
{
    CARB_PROFILE_ZONE(0, "PhysXInspectorModelImpl::refreshJointsList");
    // Maximal Joints
    PxConstraint* cs;
    std::shared_ptr<PhysXInspectorItem> maximalJointsNode;
    std::vector<PxJoint*> maximalJoints;
    for (PxU32 constrIdx = 0; constrIdx < scene->getNbConstraints(); constrIdx++)
    {
        scene->getConstraints(&cs, 1, constrIdx);
        PxU32 typeId;
        void* external = cs->getExternalReference(typeId);
        if (typeId == PxConstraintExtIDs::eJOINT)
        {
            PxJoint* joint = (PxJoint*)external;
            maximalJoints.push_back(joint);
            pxr::SdfPath jointPath =
                mInspector->mPhysXInterface->getPhysXObjectUsdPath((omni::physx::usdparser::ObjectId)joint->userData);
            if (!isIncludedInCurrentSelection(joint->userData))
                continue;
            if (!maximalJointsNode)
            {
                maximalJointsNode = makeInspectorItem("Joints", "", PhysXInspectorItem::Type::Group);
                _children.push_back(maximalJointsNode);
            }
            addJoint(*maximalJointsNode, jointPath, PhysXInspectorItem::Type::MaximalJoint);
        }
    }

    // Articulations
    PxArticulationReducedCoordinate* articulation = nullptr;
    const PxU32 nbArticulations = scene->getNbArticulations();
    int numCurrentArticulations = 0;
    std::unordered_map<std::string, int> hierarchiesPerArticulation, hierarchiesPerArticulationIndex;
    std::string name;
    std::string path;
    mAllArticulations.clear();
    mIsArticulationAllowedToBeFixed.clear();
    for (PxU32 i = 0; i < nbArticulations; i++)
    {
        scene->getArticulations(&articulation, 1, i);
        if (!isIncludedInCurrentSelection(articulation->userData))
            continue;

        // Check if this articulation is free, in order to know if we can "fix it" when the "Fix base" option is enabled
        // Trying to be a little bit smarter here and we declare as "non free" (so non-fixable) articulations that
        // are being referenced by some maximal joint in the scene that is used to "stich together" multiple
        // articulations. This is not perfect but should cover a reasonable number of common use cases.
        bool isArticulationFree = !(articulation->getArticulationFlags() & PxArticulationFlag::eFIX_BASE);
        if (isArticulationFree)
        {
            for (PxJoint* joint : maximalJoints)
            {
                PxRigidActor* actors[2] = { nullptr, nullptr };
                joint->getActors(actors[0], actors[1]);
                PxArticulationLink* link[2];
                link[0] = actors[0] ? actors[0]->is<PxArticulationLink>() : nullptr;
                link[1] = actors[1] ? actors[1]->is<PxArticulationLink>() : nullptr;
                if (link[0] && link[1])
                {
                    if (&link[0]->getArticulation() == articulation || &link[1]->getArticulation() == articulation)
                    {
                        isArticulationFree = false;
                        break;
                    }
                }
            }
        }
        mIsArticulationAllowedToBeFixed.push_back(isArticulationFree);
        mAllArticulations.push_back(getPathForUserData(articulation->userData));
        numCurrentArticulations++;
        path = articulation->getName();
        hierarchiesPerArticulation[path]++;
        if (numCurrentArticulations > 1)
            break;
    }
    for (PxU32 i = 0; i < nbArticulations; i++)
    {
        scene->getArticulations(&articulation, 1, i);
        if (!isIncludedInCurrentSelection(articulation->userData))
            continue;
        path = articulation->getName();
        name = strrchr(path.c_str(), '/') + 1;
        const int numHierarchies = hierarchiesPerArticulation[path];
        if (numHierarchies > 1)
        {
            int index = hierarchiesPerArticulationIndex[path] + 1;
            name = carb::fmt::format("{} ({} of {})", name, index, numHierarchies);
            hierarchiesPerArticulationIndex[path]++;
        }
        auto articulationNode = makeInspectorItem(name.c_str(), path.c_str(), PhysXInspectorItem::Type::Articulation);
        articulationNode->usdPath = pxr::SdfPath(path.c_str());
        addArticulationJoints(*articulationNode, articulation);

        if (numCurrentArticulations == 1 && _children.empty())
        {
            // If we only have 1 articulation, we don't group it under
            // an "articulation" node, but we directly show list of joints
            _children = articulationNode->children;
            return DataShapeType::eFlat;
        }
        else
        {
            _children.push_back(articulationNode);
        }
    }
    return DataShapeType::eHierarchical;
}

pxr::SdfPath PhysXInspectorModelImpl::getPathForUserData(void* userData) const
{
    return mInspector->mPhysXInterface->getPhysXObjectUsdPath((omni::physx::usdparser::ObjectId)userData);
}

bool PhysXInspectorModelImpl::isIncludedInCurrentSelection(void* userData) const
{
    return isIncludedInCurrentSelection(getPathForUserData(userData));
}

bool PhysXInspectorModelImpl::isIncludedInCurrentSelection(const pxr::SdfPath& path) const
{
    if (mSelectionIsArticulation)
    {
        if (path.HasPrefix(mSelectedPrimPathSdf))
            return true;
    }
    for (const auto& p : mSelectionSdf)
    {
        if (path.HasPrefix(p))
            return true;
    }
    return false;
}


std::vector<AbstractItemConstPointer> PhysXInspectorModelImpl::getItemChildren(const AbstractItemConstPointer& parentItem)
{
    if (parentItem)
    {
        auto downcast = std::dynamic_pointer_cast<const PhysXInspectorItem>(parentItem);
        return downcast->children;
    }
    return _children;
}

size_t PhysXInspectorModelImpl::getItemValueModelCount(const AbstractItemConstPointer& item)
{
    return 2;
}

std::shared_ptr<omni::ui::AbstractValueModel> PhysXInspectorModelImpl::getItemValueModel(
    const AbstractItemConstPointer& item, size_t index)
{
    auto downcast = std::dynamic_pointer_cast<const PhysXInspectorItem>(item);
    switch (index)
    {
    case 0:
        return downcast->nameModel;
    case 1:
        return downcast->usdPathModel;
    case 2:
        return downcast->detailAsTextValueModel;
    case 3:
        return downcast->numericValue;
    case 4:
        return downcast->minValue;
    case 5:
        return downcast->maxValue;
    case 6:
        return downcast->stiffnessModel;
    case 7:
        return downcast->dampingModel;
    }
    return nullptr;
}

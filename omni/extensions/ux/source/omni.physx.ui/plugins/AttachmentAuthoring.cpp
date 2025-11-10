// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "AttachmentAuthoring.h"

#include <algorithm>

#include <carb/profiler/Profile.h>
#include <omni/kit/EditorUsd.h>
#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/Selection.h>
#include <private/omni/physx/PhysxUsd.h>
#include <omni/physx/IPhysxSettings.h>
#include <private/omni/physx/IPhysxAttachmentPrivate.h>

#include <usdrt/scenegraph/usd/usd/stage.h>


using namespace omni::physx::ui;
using namespace pxr;

static const TfToken autoDeformableAttachmentInputCrcToken{ "physxAutoDeformableAttachment:inputCrc" };
static const pxr::TfToken deformableBodyDataCrcToken("physxDeformableBody:deformableBodyDataCrc");

extern UsdStageRefPtr gStage;

extern omni::physx::IPhysxAttachmentPrivate* gPhysXAttachmentPrivate;

namespace
{
    void parseTargets(SdfPath targetPaths[2], const UsdPrim autoAttachmentPrim)
    {
        if (autoAttachmentPrim)
        {
            SdfPathVector pathVector;
            SdfPath targetPath;

            {
                UsdRelationship rel0 = autoAttachmentPrim.GetRelationship(PhysxAdditionAttrTokens->attachable0);
                rel0.GetTargets(&pathVector);
                targetPath = pathVector.size() == 1 ? pathVector[0] : SdfPath();
                targetPaths[0] = targetPath;
            }

            {
                UsdRelationship rel1 = autoAttachmentPrim.GetRelationship(PhysxAdditionAttrTokens->attachable1);
                rel1.GetTargets(&pathVector);
                targetPath = pathVector.size() == 1 ? pathVector[0] : SdfPath();
                targetPaths[1] = targetPath;
            }
        }
    }

    void parseMaskShapes(SdfPathSet& shapePaths, const UsdPrim autoAttachmentPrim)
    {
        if (autoAttachmentPrim)
        {
            UsdRelationship maskShapesRel = autoAttachmentPrim.GetRelationship(PhysxAdditionAttrTokens->maskShapes);
            if (maskShapesRel)
            {
                SdfPathVector pathVector;
                maskShapesRel.GetTargets(&pathVector);
                for (uint32_t i = 0; i < pathVector.size(); ++i)
                {
                    UsdPrim shapePrim = gStage->GetPrimAtPath(pathVector[i]);
                    if (shapePrim && (shapePrim.IsA<UsdGeomSphere>() || shapePrim.IsA<UsdGeomCapsule>() || shapePrim.IsA<UsdGeomCube>()))
                    {
                        shapePaths.insert(pathVector[i]);
                    }
                }
            }
        }
    }

}

AttachmentAuthoring::AttachmentAuthoring(bool enable) : mIsEnabled(enable)
{
}

AttachmentAuthoring::~AttachmentAuthoring()
{
    release();
}

void AttachmentAuthoring::parseStage()
{
    CARB_PROFILE_ZONE(0, "AttachmentAuthoring::parseStage");

    if (!mIsEnabled)
    {
        return;
    }

    PXR_NS::UsdStageCache& cache = PXR_NS::UsdUtilsStageCache::Get();
    omni::fabric::UsdStageId stageId = { static_cast<uint64_t>(cache.GetId(gStage).ToLongInt()) };
    omni::fabric::IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(stageId);
    usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(stageId, stageInProgress);

    bool needToParse = false;
    {
        const std::vector<usdrt::SdfPath> paths = usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("PhysxPhysicsAttachment"));
        if (!paths.empty())
            needToParse = true;
    }

    if (needToParse)
        handlePrimResync(gStage->GetPseudoRoot().GetPath());
}

void AttachmentAuthoring::update()
{
    CARB_PROFILE_ZONE(0, "AttachmentAuthoring::Update");
    if (!mIsEnabled)
    {
        return;
    }

    {
        for (SdfPath attachmentPath : mBufferAttachmentPathsToRemove)
        {
            removeAttachment(attachmentPath);
            mBufferAttachmentPathsToUpdateTargets.erase(attachmentPath);
            mBufferAttachmentPathsToUpdateShapes.erase(attachmentPath);
            mBufferAttachmentPathsToUpdate.erase(attachmentPath);
        }

        for (SdfPath attachmentPath : mBufferAttachmentPathsToAdd)
        {
            addAttachment(attachmentPath);
            mBufferAttachmentPathsToUpdate.insert(attachmentPath);
        }

        for (SdfPath attachmentPath : mBufferAttachmentPathsToUpdateTargets)
        {
            updateAttachmentTargets(attachmentPath);
            mBufferAttachmentPathsToUpdate.insert(attachmentPath);
        }

        for (SdfPath attachmentPath : mBufferAttachmentPathsToUpdateShapes)
        {
            updateAttachmentMaskShapes(attachmentPath);
            mBufferAttachmentPathsToUpdate.insert(attachmentPath);
        }

        for (SdfPath attachmentPath : mBufferAttachmentPathsToUpdate)
        {
            gPhysXAttachmentPrivate->updateAutoDeformableAttachment(attachmentPath);
        }
    }

    clearBuffers();
}

void AttachmentAuthoring::release()
{
    mAttachments.clear();
    mTargets.clear();
    mTargetTable.clear();
    mShapes.clear();
    mShapeTable.clear();

    clearBuffers();
}

void AttachmentAuthoring::refreshAttachment(SdfPath attachmentPath)
{
    //force recomputation of the attachment
    UsdPrim attachmentPrim = gStage->GetPrimAtPath(attachmentPath);
    if (attachmentPrim)
    {
        attachmentPrim.RemoveProperty(autoDeformableAttachmentInputCrcToken);
    }
    mBufferAttachmentPathsToUpdate.insert(attachmentPath);
}

void AttachmentAuthoring::handlePrimResync(const SdfPath path)
{
    TfType bodyType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
    UsdPrimRange range(gStage->GetPrimAtPath(path));
    for(pxr::UsdPrimRange::const_iterator cit = range.begin(); cit != range.end(); ++cit)
    {
        const UsdPrim& prim = *cit;
        if (!prim)
            continue;

        SdfPath primPath = prim.GetPath();
        TfType autoAttachmentType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->AutoDeformableAttachmentAPI);
        
        if (prim.HasAPI(autoAttachmentType))
        {
            if (mAttachments.count(primPath) == 0)
            {
                mBufferAttachmentPathsToAdd.insert(primPath);
            }
        }
        else if (prim.IsA<UsdGeomXformable>() || prim.HasAPI(bodyType))
        {
            //handling readding of removed targets
            SdfPathToRefInfoMap::iterator tit = mTargets.find(primPath);
            if (tit != mTargets.end() && tit->second.isStale)
            {
                for (SdfPath attachmentPath : tit->second.attachments)
                {
                    mBufferAttachmentPathsToUpdate.insert(attachmentPath);
                }
                tit->second.isStale = false;
            }
        }

        //for now we allow collision or rigid body APIs on attachment shapes
        if (prim.IsA<UsdGeomSphere>() || prim.IsA<UsdGeomCapsule>() || prim.IsA<UsdGeomCube>())
        {
            SdfPathToRefInfoMap::iterator sit = mShapes.find(primPath);
            if (sit != mShapes.end() && sit->second.isStale)
            {
                for (SdfPath attachmentPath : sit->second.attachments)
                {
                    mBufferAttachmentPathsToUpdate.insert(attachmentPath);
                }
                sit->second.isStale = false;
            }
        }
    }
}

void AttachmentAuthoring::handlePrimRemove(const SdfPath path)
{
    {
        const auto iteratorPair = mAttachmentTable.FindSubtreeRange(path);
        for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
        {
            // it->first is a path in the subtree starting at path.
            if (mAttachments.count(it->first))
                mBufferAttachmentPathsToRemove.insert(it->first);
        }
    }

    {
        const auto iteratorPair = mTargetTable.FindSubtreeRange(path);
        for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
        {
            // it->first is a path in the subtree starting at path.
            SdfPathToRefInfoMap::iterator tit = mTargets.find(it->first);
            if (tit != mTargets.end() && !tit->second.isStale)
            {
                for (SdfPath attachmentPath : tit->second.attachments)
                {
                    mBufferAttachmentPathsToUpdate.insert(attachmentPath);
                }
                tit->second.isStale = true;
            }
        }
    }

    {
        const auto iteratorPair = mShapeTable.FindSubtreeRange(path);
        for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
        {
            // it->first is a path in the subtree starting at path.
            SdfPathToRefInfoMap::iterator sit = mShapes.find(it->first);
            if (sit != mShapes.end() && !sit->second.isStale)
            {
                for (SdfPath attachmentPath : sit->second.attachments)
                {
                    mBufferAttachmentPathsToUpdate.insert(attachmentPath);
                }
                sit->second.isStale = true;
            }
        }
    }
}

void AttachmentAuthoring::handleAttributeChange(const SdfPath path, const TfToken attributeName, const bool isXform)
{
    if (isXform)
    {
        //we don't actively update attachments based on target transform changes nor on attachment shape transform changes
        return;
    }

    UsdPrim prim = gStage->GetPrimAtPath(path);
    if (!prim)
        return;

    if (attributeName == deformableBodyDataCrcToken ||
        attributeName == UsdGeomTokens->surfaceFaceVertexIndices ||
        attributeName == UsdGeomTokens->tetVertexIndices ||
        attributeName == UsdGeomTokens->faceVertexCounts ||
        attributeName == UsdGeomTokens->faceVertexIndices)
    {
        SdfPathToRefInfoMap::iterator it = mTargets.find(path);
        if (it != mTargets.end())
        {
            for (SdfPath attachmentPath : it->second.attachments)
            {
                mBufferAttachmentPathsToUpdate.insert(attachmentPath);
            }
        }
    }
    else if (prim.IsA<UsdGeomSphere>() || prim.IsA<UsdGeomCapsule>() || prim.IsA<UsdGeomCube>())
    {
        if (attributeName == UsdGeomTokens.Get()->radius ||
            attributeName == UsdGeomTokens.Get()->height ||
            attributeName == UsdGeomTokens.Get()->axis ||
            attributeName == UsdGeomTokens.Get()->size)
        {
            SdfPathToRefInfoMap::iterator it = mShapes.find(path);
            if (it != mShapes.end())
            {
                for (SdfPath attachmentPath : it->second.attachments)
                {
                    mBufferAttachmentPathsToUpdate.insert(attachmentPath);
                }
            }
        }
    }

    // otherwise only handle changes that refer to an attachment:
    else if (!mAttachments.count(path))
        return;

    // check point updates:
    if (attributeName == PhysxAdditionAttrTokens->enableDeformableVertexAttachments ||
        attributeName == PhysxAdditionAttrTokens->deformableVertexOverlapOffset ||
        attributeName == PhysxAdditionAttrTokens->enableRigidSurfaceAttachments ||
        attributeName == PhysxAdditionAttrTokens->rigidSurfaceSamplingDistance ||
        attributeName == PhysxAdditionAttrTokens->enableCollisionFiltering ||
        attributeName == PhysxAdditionAttrTokens->collisionFilteringOffset ||
        attributeName == PhysxAdditionAttrTokens->enableDeformableFilteringPairs)
    {
        mBufferAttachmentPathsToUpdate.insert(path);
    }
    else if (attributeName == PhysxAdditionAttrTokens->attachable0 ||
             attributeName == PhysxAdditionAttrTokens->attachable1)
    {
        mBufferAttachmentPathsToUpdateTargets.insert(path);
    }
    else if (attributeName == PhysxAdditionAttrTokens->maskShapes)
    {
        mBufferAttachmentPathsToUpdateShapes.insert(path);
    }
}

void AttachmentAuthoring::addAttachment(const SdfPath attachmentPath)
{
    UsdPrim attachmentPrim = gStage->GetPrimAtPath(attachmentPath);
    TfType autoAttachmentType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->AutoDeformableAttachmentAPI);
    if (!attachmentPrim.HasAPI(autoAttachmentType))
        return;

    SdfPath targetPaths[2];
    parseTargets(targetPaths, attachmentPrim);

    SdfPathSet shapePaths;
    parseMaskShapes(shapePaths, attachmentPrim);

    //add targets
    for (uint32_t s = 0; s < 2; ++s)
    {
        if (!targetPaths[s].IsEmpty())
        {
            addRefPath(mTargets, mTargetTable, targetPaths[s], attachmentPath);
        }
    }

    //add shapes
    for (SdfPath shapePath : shapePaths)
    {
        addRefPath(mShapes, mShapeTable, shapePath, attachmentPath);
    }

    //add attachment, first remove if already exist (could also check whether all the data is up to date instead).
    SdfPathToAttachmentInfoMap::iterator ait = mAttachments.find(attachmentPath);
    if (ait != mAttachments.end())
    {
        mAttachments.erase(attachmentPath);
        mAttachmentTable.erase(attachmentPath);
    }

    AttachmentInfo info = {{targetPaths[0], targetPaths[1]}, shapePaths};
    mAttachments.insert({attachmentPath, info});
    mAttachmentTable.insert({attachmentPath, Empty()});
}

void AttachmentAuthoring::removeAttachment(const SdfPath attachmentPath)
{
    SdfPathToAttachmentInfoMap::iterator ait = mAttachments.find(attachmentPath);
    if (ait == mAttachments.end())
        return;

    AttachmentInfo& info = ait->second;

    //remove targets
    for (uint32_t s = 0; s < 2; ++s)
    {
        removeRefPath(mTargets, mTargetTable, info.targets[s], attachmentPath);
    }

    //remove shapes
    for (SdfPath shapePath : info.shapes)
    {
        removeRefPath(mShapes, mShapeTable, shapePath, attachmentPath);
    }

    //remove attachment
    mAttachments.erase(attachmentPath);
    mAttachmentTable.erase(attachmentPath);
}

void AttachmentAuthoring::updateAttachmentTargets(const SdfPath attachmentPath)
{
    UsdPrim attachmentPrim = gStage->GetPrimAtPath(attachmentPath);
    TfType autoAttachmentType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->AutoDeformableAttachmentAPI);
    if (!attachmentPrim.HasAPI(autoAttachmentType))
        return;

    SdfPathToAttachmentInfoMap::iterator ait = mAttachments.find(attachmentPath);
    if (ait == mAttachments.end())
        return;

    AttachmentInfo& info = ait->second;

    SdfPath newTargetPaths[2];
    parseTargets(newTargetPaths, attachmentPrim);

    for (uint32_t s = 0; s < 2; ++s)
    {
        SdfPath oldTargetPath = info.targets[s];
        SdfPath newTargetPath = newTargetPaths[s];
        if (oldTargetPath != newTargetPath)
        {
            if (!oldTargetPath.IsEmpty())
            {
                //remove target
                removeRefPath(mTargets, mTargetTable, oldTargetPath, attachmentPath);
            }

            if (!newTargetPath.IsEmpty())
            {
                //add target
                addRefPath(mTargets, mTargetTable, newTargetPath, attachmentPath);
            }
            else
            {
                attachmentPrim.RemoveProperty(autoDeformableAttachmentInputCrcToken);
            }

            info.targets[s] = newTargetPath;
        }
    }
}

void AttachmentAuthoring::updateAttachmentMaskShapes(const SdfPath attachmentPath)
{
    UsdPrim attachmentPrim = gStage->GetPrimAtPath(attachmentPath);
    TfType autoAttachmentType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->AutoDeformableAttachmentAPI);
    if (!attachmentPrim.HasAPI(autoAttachmentType))
        return;

    SdfPathToAttachmentInfoMap::iterator ait = mAttachments.find(attachmentPath);
    if (ait == mAttachments.end())
        return;

    AttachmentInfo& info = ait->second;

    SdfPathSet newShapePaths;
    parseMaskShapes(newShapePaths, attachmentPrim);

    //copy, old shapes, so difference test won't fail when inserting or removing discovered shapes
    SdfPathSet oldShapePaths = info.shapes;

    for (SdfPath oldShapePath : oldShapePaths)
    {
        if (newShapePaths.count(oldShapePath) == 0)
        {
            //remove shape
            removeRefPath(mShapes, mShapeTable, oldShapePath, attachmentPath);
            info.shapes.erase(oldShapePath);
        }
    }

    for (SdfPath newShapePath : newShapePaths)
    {
        if (oldShapePaths.count(newShapePath) == 0)
        {
            //add shape
            addRefPath(mShapes, mShapeTable, newShapePath, attachmentPath);
            info.shapes.insert(newShapePath);
        }
    }
}

void AttachmentAuthoring::addRefPath(SdfPathToRefInfoMap& pathToRefInfoMap, SdfPathTable& pathTable, const SdfPath refPath, const SdfPath attachmentPath)
{
    SdfPathToRefInfoMap::iterator it = pathToRefInfoMap.find(refPath);
    if (it != pathToRefInfoMap.end())
    {
        it->second.attachments.insert(attachmentPath);
    }
    else
    {
        bool isStale = !gStage->GetPrimAtPath(refPath).IsValid();
        pathToRefInfoMap.insert({refPath, {{attachmentPath}, isStale}});
        pathTable.insert({refPath, Empty()});
    }
}

void AttachmentAuthoring::removeRefPath(SdfPathToRefInfoMap& pathToRefInfoMap, SdfPathTable& pathTable, const SdfPath refPath, const SdfPath attachmentPath)
{
    SdfPathToRefInfoMap::iterator it = pathToRefInfoMap.find(refPath);
    if (it != pathToRefInfoMap.end())
    {
        it->second.attachments.erase(attachmentPath);
        if (it->second.attachments.empty())
        {
            pathToRefInfoMap.erase(it);
            pathTable.erase(refPath);
        }
    }
}

void AttachmentAuthoring::clearBuffers(void)
{
    mBufferAttachmentPathsToAdd.clear();
    mBufferAttachmentPathsToRemove.clear();
    mBufferAttachmentPathsToUpdate.clear();
    mBufferAttachmentPathsToUpdateTargets.clear();
    mBufferAttachmentPathsToUpdateShapes.clear();
}


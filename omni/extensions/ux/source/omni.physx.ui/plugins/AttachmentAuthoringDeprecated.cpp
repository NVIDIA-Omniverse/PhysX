// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "AttachmentAuthoringDeprecated.h"

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

static const TfToken physxDeformableBodyTetMeshCrcToken{ "physxDeformable:tetMeshCrc" };
static const TfToken physxDeformableSurfaceInputCrcToken{ "physxDeformableSurface:clothDataInputCrc" };
static const TfToken physxParticleClothInputCrcToken{ "physxParticle:clothDataInputCrc" };
static const TfToken physxAttachmentInputCrcToken{ "physxAttachment:inputCrc" };
static const TfToken physxAutoAttachmentMaskShapesToken{ "physxAutoAttachment:maskShapes" };

extern UsdStageRefPtr gStage;

extern omni::physx::IPhysxAttachmentPrivate* gPhysXAttachmentPrivate;

namespace
{
    void parseTargets(SdfPath targetPaths[2], const PhysxSchemaPhysxPhysicsAttachment attachment)
    {
        if (attachment)
        {
            SdfPathVector pathVector;
            for (uint32_t s = 0; s < 2; ++s)
            {
                attachment.GetActorRel(s).GetTargets(&pathVector);
                SdfPath targetPath = pathVector.size() == 1 ? pathVector[0] : SdfPath();
                targetPaths[s] = targetPath;
            }
        }
    }

    void parseMaskShapes(SdfPathSet& shapePaths, const PhysxSchemaPhysxAutoAttachmentAPI autoAttachment)
    {
        if (autoAttachment)
        {
            UsdRelationship maskShapesRel = autoAttachment.GetPrim().GetRelationship(physxAutoAttachmentMaskShapesToken);
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

AttachmentAuthoringDeprecated::AttachmentAuthoringDeprecated(bool enable) : mIsEnabled(enable)
{
}

AttachmentAuthoringDeprecated::~AttachmentAuthoringDeprecated()
{
    release();
}

void AttachmentAuthoringDeprecated::parseStage()
{
    CARB_PROFILE_ZONE(0, "AttachmentAuthoringDeprecated::parseStage");

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

void AttachmentAuthoringDeprecated::update()
{
    CARB_PROFILE_ZONE(0, "AttachmentAuthoringDeprecated::Update");
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
            gPhysXAttachmentPrivate->computeAttachmentPoints(attachmentPath);
        }
    }

    clearBuffers();
}

void AttachmentAuthoringDeprecated::release()
{
    mAttachments.clear();
    mTargets.clear();
    mTargetTable.clear();
    mShapes.clear();
    mShapeTable.clear();

    clearBuffers();
}

void AttachmentAuthoringDeprecated::getAttachments(pxr::SdfPathSet& attachmentPaths, const pxr::SdfPath& primPath)
{
    SdfPathToRefInfoMap::iterator it = mTargets.find(primPath);
    if (it != mTargets.end())
    {
        for (SdfPath attachmentPath : it->second.attachments)
        {
            attachmentPaths.insert(attachmentPath);
        }
    }
}

void AttachmentAuthoringDeprecated::refreshAttachment(SdfPath attachmentPath)
{
    //force recomputation of the attachment
    UsdPrim attachmentPrim = gStage->GetPrimAtPath(attachmentPath);
    if (attachmentPrim)
    {
        attachmentPrim.RemoveProperty(physxAttachmentInputCrcToken);
    }
    mBufferAttachmentPathsToUpdate.insert(attachmentPath);
}

void AttachmentAuthoringDeprecated::handlePrimResync(const SdfPath path)
{
    UsdPrimRange range(gStage->GetPrimAtPath(path));
    for(pxr::UsdPrimRange::const_iterator cit = range.begin(); cit != range.end(); ++cit)
    {
        const UsdPrim& prim = *cit;
        if (!prim)
            continue;

        SdfPath primPath = prim.GetPath();

        if(prim.IsA<PhysxSchemaPhysxPhysicsAttachment>())
        {
            if (mAttachments.count(primPath) == 0)
            {
                mBufferAttachmentPathsToAdd.insert(primPath);
            }
        }
        else if (prim.HasAPI<UsdPhysicsRigidBodyAPI>() || prim.HasAPI<UsdPhysicsCollisionAPI>() ||
            prim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>() || prim.HasAPI<PhysxSchemaPhysxDeformableSurfaceAPI>() || prim.HasAPI<PhysxSchemaPhysxParticleClothAPI>())
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

void AttachmentAuthoringDeprecated::handlePrimRemove(const SdfPath path)
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

void AttachmentAuthoringDeprecated::handleAttributeChange(const SdfPath path, const TfToken attributeName, const bool isXform)
{
    if (isXform)
    {
        //we don't actively update attachments based on target transform changes nor on attachment shape transform changes
        return;
    }

    UsdPrim prim = gStage->GetPrimAtPath(path);
    if (!prim)
        return;

    if (attributeName == physxDeformableBodyTetMeshCrcToken ||
        attributeName == physxDeformableSurfaceInputCrcToken ||
        attributeName == physxParticleClothInputCrcToken)
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
    if (attributeName == PhysxSchemaTokens.Get()->physxAutoAttachmentEnableDeformableVertexAttachments ||
        attributeName == PhysxSchemaTokens.Get()->physxAutoAttachmentDeformableVertexOverlapOffset ||
        attributeName == PhysxSchemaTokens.Get()->physxAutoAttachmentEnableRigidSurfaceAttachments ||
        attributeName == PhysxSchemaTokens.Get()->physxAutoAttachmentRigidSurfaceSamplingDistance ||
        attributeName == PhysxSchemaTokens.Get()->physxAutoAttachmentEnableCollisionFiltering ||
        attributeName == PhysxSchemaTokens.Get()->physxAutoAttachmentCollisionFilteringOffset ||
        attributeName == PhysxSchemaTokens.Get()->physxAutoAttachmentEnableDeformableFilteringPairs)
    {
        mBufferAttachmentPathsToUpdate.insert(path);
    }
    else if (attributeName == PhysxSchemaTokens.Get()->actor0 || attributeName == PhysxSchemaTokens.Get()->actor1)
    {
        mBufferAttachmentPathsToUpdateTargets.insert(path);
    }
    else if (attributeName == physxAutoAttachmentMaskShapesToken)
    {
        mBufferAttachmentPathsToUpdateShapes.insert(path);
    }
}

void AttachmentAuthoringDeprecated::addAttachment(const SdfPath attachmentPath)
{
    UsdPrim attachmentPrim = gStage->GetPrimAtPath(attachmentPath);
    PhysxSchemaPhysxPhysicsAttachment attachment(attachmentPrim);
    if (!attachment)
        return;

    SdfPath targetPaths[2];
    parseTargets(targetPaths, attachment);

    SdfPathSet shapePaths;
    parseMaskShapes(shapePaths, PhysxSchemaPhysxAutoAttachmentAPI(attachmentPrim));

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

void AttachmentAuthoringDeprecated::removeAttachment(const SdfPath attachmentPath)
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

void AttachmentAuthoringDeprecated::removeTargetFromAttachmentPrim(UsdPrim attachmentPrim, uint32_t slot)
{
    if (attachmentPrim)
    {
        attachmentPrim.RemoveProperty(physxAttachmentInputCrcToken);
        if (slot == 0)
        {
            attachmentPrim.RemoveProperty(pxr::PhysxSchemaTokens.Get()->points0);
            attachmentPrim.RemoveProperty(pxr::PhysxSchemaTokens.Get()->collisionFilterIndices0);
            attachmentPrim.RemoveProperty(pxr::PhysxSchemaTokens.Get()->filterType0);
        }
        if (slot == 1)
        {
            attachmentPrim.RemoveProperty(pxr::PhysxSchemaTokens.Get()->points1);
            attachmentPrim.RemoveProperty(pxr::PhysxSchemaTokens.Get()->collisionFilterIndices1);
            attachmentPrim.RemoveProperty(pxr::PhysxSchemaTokens.Get()->filterType1);
        }
    }
}

void AttachmentAuthoringDeprecated::updateAttachmentTargets(const SdfPath attachmentPath)
{
    UsdPrim attachmentPrim = gStage->GetPrimAtPath(attachmentPath);
    PhysxSchemaPhysxPhysicsAttachment attachment(attachmentPrim);
    if (!attachment)
        return;

    SdfPathToAttachmentInfoMap::iterator ait = mAttachments.find(attachmentPath);
    if (ait == mAttachments.end())
        return;

    AttachmentInfo& info = ait->second;

    SdfPath newTargetPaths[2];
    parseTargets(newTargetPaths, attachment);

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
                removeTargetFromAttachmentPrim(attachmentPrim, s);
            }

            info.targets[s] = newTargetPath;
        }
    }
}

void AttachmentAuthoringDeprecated::updateAttachmentMaskShapes(const SdfPath attachmentPath)
{
    UsdPrim attachmentPrim = gStage->GetPrimAtPath(attachmentPath);
    PhysxSchemaPhysxPhysicsAttachment attachment(attachmentPrim);
    if (!attachment)
        return;

    SdfPathToAttachmentInfoMap::iterator ait = mAttachments.find(attachmentPath);
    if (ait == mAttachments.end())
        return;

    AttachmentInfo& info = ait->second;

    SdfPathSet newShapePaths;
    parseMaskShapes(newShapePaths, PhysxSchemaPhysxAutoAttachmentAPI(attachmentPrim));

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

void AttachmentAuthoringDeprecated::addRefPath(SdfPathToRefInfoMap& pathToRefInfoMap, SdfPathTable& pathTable, const SdfPath refPath, const SdfPath attachmentPath)
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

void AttachmentAuthoringDeprecated::removeRefPath(SdfPathToRefInfoMap& pathToRefInfoMap, SdfPathTable& pathTable, const SdfPath refPath, const SdfPath attachmentPath)
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

void AttachmentAuthoringDeprecated::clearBuffers(void)
{
    mBufferAttachmentPathsToAdd.clear();
    mBufferAttachmentPathsToRemove.clear();
    mBufferAttachmentPathsToUpdate.clear();
    mBufferAttachmentPathsToUpdateTargets.clear();
    mBufferAttachmentPathsToUpdateShapes.clear();
}

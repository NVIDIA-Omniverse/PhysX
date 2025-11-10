// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"
#include "SpatialTendonAuthoring.h"

#include <algorithm>

#include <carb/profiler/Profile.h>
#include <carb/Framework.h>
#include <omni/core/ITypeFactory.h>
#include <omni/fabric/usd/PathConversion.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSettings.h>
#include <private/omni/physx/IPhysxUsdLoad.h>
#include <omni/kit/EditorUsd.h>
#include <omni/kit/PythonInterOpHelper.h>
#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/Selection.h>
#include <omni/renderer/IDebugDraw.h>

#include <common/ui/ImguiDrawingUtils.h>
#include "DebugVisualizationTendonCommon.h"

using namespace omni::physx::ui;
using namespace pxr;

extern UsdStageRefPtr gStage;
extern omni::physx::IPhysx* gPhysX;
extern carb::settings::ISettings* gSettings;
extern bool gBlockNoticeHandle;
static omni::renderer::IDebugDraw* gDebugDraw;

// viz parameters:
static const double kAttachmentDiameterRatio = 0.1; // attachment diameter = ratio *
                                                    // average(max(extentsOfAllBodiesWithAttachments))
static const uint32_t kLineColorARGB = 0xDB101080;
static const float kLineWidth = 1.0f;
static const GfVec3f kColorRoot(0.0f, 1.0f, 1.0f); // cyan
static const GfVec3f kColorAttachment(1.0f, 0.0f, 1.0f); // magenta
static const GfVec3f kColorLeaf(1.0f, 1.0f, 0.0f); // yellow

// attribute and api name tokens/strings
static const TfToken kTransformMatrixAttrName{ "xformOp:transform" };
static const TfToken kTranslateAttrName{ "xformOp:translate" };
static const TfToken kScaleAttrName{ "xformOp:scale" };
static const std::string kAttachmentAPICommonName("PhysxTendonAttachment");
static const std::string kAttachmentAPIName("PhysxTendonAttachmentAPI");
static const std::string kAttachmentRootAPIName("PhysxTendonAttachmentRootAPI");
static const std::string kAttachmentLeafAPIName("PhysxTendonAttachmentLeafAPI");
static const TfToken kAttachmentAPIToken("PhysxTendonAttachmentAPI");
static const TfToken kAttachmentRootAPIToken("PhysxTendonAttachmentRootAPI");
static const TfToken kAttachmentLeafAPIToken("PhysxTendonAttachmentLeafAPI");
static const TfToken kPhysicsRigidBodyAPIToken("PhysicsRigidBodyAPI");
static const std::string kPhysxTendonNamespace("physxTendon");

static bool isRBLinkWithAttachment(const UsdPrim& prim)
{
    return prim.IsA<UsdGeomXformable>() && prim.HasAPI<UsdPhysicsRigidBodyAPI>() &&
           prim.HasAPI<PhysxSchemaPhysxTendonAttachmentAPI>();
}

SpatialTendonManager::SpatialTendonManager(const VisualizerMode visualizationMode)
    : mMode{ visualizationMode },
      mLastSelectedAttachment{ SdfPath() },
      mBufferAttachmentToSelect{ SdfPath() },
      mBBCache(UsdTimeCode::Default(), { UsdGeomTokens->default_ })
{
    gDebugDraw = carb::getCachedInterface<omni::renderer::IDebugDraw>();
}

SpatialTendonManager::~SpatialTendonManager()
{
    release();
    gDebugDraw = nullptr;
}

void SpatialTendonManager::parseStage()
{
    CARB_PROFILE_ZONE(0, "SpatialTendonManager::parseStage");

    if (!isActive())
        return;

    // handleResync will traverse
    handlePrimResync(gStage->GetPseudoRoot().GetPath());
    updateGeomScaleFromSettings(); // read out new viewport settings

    if (gStage)
    {
        const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(gStage).ToLongInt();
        mFabricSync = omni::core::createType<usdrt::xformcache::IXformCache>();
        if (mFabricSync)
        {
            CARB_PROFILE_ZONE(0, "SpatialTendonManager::parseStage attachToStage");
            mFabricSync->attachToStage(stageId);
        }
    }
}

void SpatialTendonManager::update()
{
    CARB_PROFILE_ZONE(0, "SpatialTendonManager::Update");

    if (!gStage || !isActive())
        return;

    // clear xform cache
    mXformCache.Clear();
    // block all notices from changes starting here:
    gBlockNoticeHandle = true;

    mFabricEnabled = gSettings->getAsBool(kSettingFabricEnabled);
    if (mFabricEnabled && mFabricSync)
    {
        mFabricSync->syncXforms();
        for (const auto& bodyPath : mVisibleBodies)
        {
            const usdrt::GfMatrix4d matrix = mFabricSync->computeWorldXform(omni::fabric::asInt(bodyPath));
            pxr::GfMatrix4d gfMatrix;
            memcpy(&gfMatrix, &matrix, sizeof(pxr::GfMatrix4d));
            auto iterMat = mBodyMatrices.find(bodyPath);
            if (iterMat == mBodyMatrices.end())
            {
                mBufferPathsToUpdateTransform.insert(bodyPath);
                mBodyMatrices[bodyPath] = gfMatrix;
            }
            else if (iterMat->second != gfMatrix)
            {
                mBufferPathsToUpdateTransform.insert(bodyPath);
                iterMat->second = gfMatrix;
            }
        }
    }

    if (!mBodiesWithAttachments.empty())
    {
        SdfChangeBlock block;
        // remove deleted attachment APIs / must be outside session layer scope
        for (SdfPathSet::const_iterator cit = mBufferPathsToDeletedAttachments.cbegin();
             cit != mBufferPathsToDeletedAttachments.cend(); ++cit)
        {
            // check if sim running:
            if (gPhysX->isRunning())
            {
                CARB_LOG_WARN(
                    "PhysX Spatial Tendon: Cannot remove attachment %s while simulation is running", cit->GetText());
                // the session layer helper geom was deleted, so remove from session to body map:
                const SdfPath sessionHelperPath =
                    getSessionLayerXformPath(cit->GetParentPath(), false).AppendChild(cit->GetNameToken());
                mSessionToBodies.erase(sessionHelperPath);
                mBufferVisibilityDirty = true; // trigger recreation if needed
            }
            else
            {
                runRemoveAPICommand(*cit);
                mBufferPathsToUpdate.insert(cit->GetParentPath());
            }
        }
        // check for new connections if the selection changed:
        if (mBufferSelectionChanged)
        {
            // update selected paths to be processed in the following steps
            omni::usd::UsdContext* usdContext = omni::usd::UsdContext::getContext();
            mBufferSelectedPaths = usdContext->getSelection()->getSelectedPrimPathsV2();
            if (mBufferSelectedPaths.size() == 2u)
            {
                const SdfPath attachmentWithNewParent = checkConnection(); // may adjust the selected paths buffer if a
                                                                           // new connection is made
                if (!attachmentWithNewParent.IsEmpty())
                {
                    // if an attachment got a new parent, adapt selection buffer AND selection to point to session geom
                    // so visibility is updated properly and the user gets auto-scroll to edited attachment
                    mBufferSelectedPaths[0] = { getSessionLayerXformPath(attachmentWithNewParent.GetParentPath(), false)
                                                    .AppendChild(attachmentWithNewParent.GetNameToken()) };
                    mBufferSelectedPaths.resize(1u);
                    usdContext->getSelection()->setSelectedPrimPathsV2(mBufferSelectedPaths);
                    // and queue for a parent update
                    mBufferPathsToUpdateParent.insert(attachmentWithNewParent);
                }
            }
            mBufferVisibilityDirty = true;
        }
    }
    {
        // all updates following are to session layer
        omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());

        // register new bodies that have attachments
        for (SdfPathSet::const_iterator cit = mBufferPathsToAdd.cbegin(); cit != mBufferPathsToAdd.cend(); ++cit)
        {
            // printf("SpatialTendonManager::update: Adding body %s\n", cit->GetText());
            addBody(*cit);
        }
        // sync body changes:
        for (SdfPathSet::const_iterator cit = mBufferPathsToUpdate.cbegin(); cit != mBufferPathsToUpdate.cend(); ++cit)
        {
            // printf("SpatialTendonManager::update: Updating body %s\n", cit->GetText());
            updateBody(*cit);
        }

        // can early exit if there are no bodies with attachments:
        if (mBodiesWithAttachments.empty())
        {
            clearBuffers();
            gBlockNoticeHandle = false;
            return;
        }

        // sync parent updates:
        for (SdfPathSet::const_iterator cit = mBufferPathsToUpdateParent.cbegin();
             cit != mBufferPathsToUpdateParent.cend(); ++cit)
        {
            // printf("SpatialTendonManager::update: Updating parent of %s\n", cit->GetText());
            // issue warning about runtime edits
            if (gPhysX->isRunning())
            {
                CARB_LOG_WARN(
                    "PhysX Spatial Tendon: Cannot commit parent changes of %s to simulation while simulation is running. "
                    "New relationship is displayed but does not affect the simulation.",
                    cit->GetText());
            }
            removeFromParentChildMaps(*cit);
            insertInParentChildMaps(*cit);
            mBufferVisibilityDirty = true;
        }

        // determine visible-bodies set and update visible bodies
        if (mBufferVisibilityDirty)
        {
            updateVisibilityAndCreateNewAttachmentGeoms();
            mBufferSetNewAttachmentRadius = true; // make sure newly visible attachments have the newest radius
        }
    }

    // Do all attribute updates and prim deletions in Sdf changeblock:
    {
        SdfChangeBlock block;
        // First update TO local pos which is NOT to session layer
        for (SdfPathSet::const_iterator cit = mBufferPathsToUpdateToLocalPos.cbegin();
             cit != mBufferPathsToUpdateToLocalPos.cend(); ++cit)
        {
            // printf("SpatialTendonManager::update: Updating to localpos of attachment %s\n", cit->GetText());
            updateToLocalPos(*cit);
        }
        {
            // all updates following are to session layer
            omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());
            // process FROM localPos updates
            for (SdfPathSet::const_iterator cit = mBufferPathsToUpdateFromLocalPos.cbegin();
                 cit != mBufferPathsToUpdateFromLocalPos.cend(); ++cit)
            {
                // printf("SpatialTendonManager::update: Updating from localpos of attachment %s\n", cit->GetText());
                updateFromLocalPos(*cit);
            }
            // process transform updates
            for (SdfPathSet::const_iterator cit = mBufferPathsToUpdateTransform.cbegin();
                 cit != mBufferPathsToUpdateTransform.cend(); ++cit)
            {
                // printf("SpatialTendonManager::update: Updating transform of body %s\n", cit->GetText());
                updateXformTransform(*cit);
            }
            // recompute attachment radius from current set of bodies with attachments
            // recompute if requested via buffer bool from add/remove body.
            // also update if xforms changed and not running (i.e. user scaling a body in the stage)
            const bool updateBecauseOfXformChange = !mBufferPathsToUpdateTransform.empty() && !gPhysX->isRunning();
            if (mBufferRecomputeAttachmentRadius || updateBecauseOfXformChange)
            {
                computeAttachmentRadiusFromBodies();
            }
            // delete removed bodies
            for (SdfPathSet::const_iterator cit = mBufferPathsToRemove.cbegin(); cit != mBufferPathsToRemove.cend(); ++cit)
            {
                // printf("SpatialTendonManager::update: Removing body %s\n", cit->GetText());
                removeBody(*cit);
            }
            if (mBufferSetNewAttachmentRadius)
            {
                updateVisibleAttachmentRadii();
            }
            if (mBufferSelectionChanged)
                processSelectionChanged(); // run selection update logic
        }
    }

    // printMaps(updated, mBufferVisibilityDirty);
    clearBuffers();

    gBlockNoticeHandle = false;

    // update draw:
    draw();
}

void SpatialTendonManager::selectionChanged()
{
    mBufferSelectionChanged = true;
}

void SpatialTendonManager::setMode(const VisualizerMode mode)
{
    if (mMode != mode)
    {
        if (mMode == VisualizerMode::eNone) // reparse the stage if we are activating the visualizer
        {
            if (gStage)
            {
                //not calling parseStage here, since it needs mMode updated
                handlePrimResync(gStage->GetPseudoRoot().GetPath());
            }
        }
        else if (mode == VisualizerMode::eNone)
        {
            // release all the buffers if we turn off
            release();
        }
    }
    mMode = mode;
    mBufferVisibilityDirty = true; // trigger visibility update
    mBufferSelectionChanged = true; // and make sure selection is checked
}

void SpatialTendonManager::release()
{
    if (gStage)
    {
        omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());
        for (SdfPathToSdfPath::const_iterator cit = mSessionToBodies.cbegin(); cit != mSessionToBodies.cend(); ++cit)
        {
            SdfPath sessionPath = cit->first;
            gStage->RemovePrim(sessionPath);
        }
    }

    mFabricSync = nullptr;
    mBodyMatrices.clear();

    mBodiesWithAttachments.clear();
    mVisibleBodies.clear();
    mChildToParent.clear();
    mSessionToBodies.clear();
    mParentToChildren.clear();
    mBodiesToAttachmentName.clear();
    mBodiesToSessionXforms.clear();
    mSessionLayerScopePath = SdfPath();
    mBBCache.Clear();
    clearBuffers();
}

void SpatialTendonManager::updateGeomScaleFromSettings(void)
{
    if (gSettings)
    {
        if (gSettings->getAsBool(kViewportGizmoConstantScaleEnabledPath))
        {
            setAttachmentRadiusScale(static_cast<double>(gSettings->getAsFloat(kViewportGizmoConstantScalePath) * 0.1f));
        }
        else
        {
            setAttachmentRadiusScale(static_cast<double>(gSettings->getAsFloat(kViewportGizmoScalePath)));
        }
    }
}

void SpatialTendonManager::draw(void)
{
    if (mVisibleBodies.empty() || !gDebugDraw)
        return;

    updateGeomScaleFromSettings();

    for (const SdfPath bodyPath : mVisibleBodies)
    {
        const SdfPathToAttachmentName::const_iterator cit = mBodiesToAttachmentName.find(bodyPath);
        if (cit == mBodiesToAttachmentName.end() || cit->second.empty())
            continue;

        const SdfPath bodyXformPath = getSessionLayerXformPath(bodyPath, false);
        const UsdPrim bodyPrim = gStage->GetPrimAtPath(bodyXformPath);
        if (!bodyPrim.IsValid())
            continue;

        const GfMatrix4d bodyXformToWorld = getBodyTransform(bodyPath, bodyPrim);

        // now go through all attachments and draw a line from the attachment to its parent:
        for (const UsdPrim child : bodyPrim.GetChildren())
        {
            GfVec3f localPos;
            const UsdAttribute translateAttr = child.GetAttribute(kTranslateAttrName);
            if (!translateAttr)
                continue;
            translateAttr.Get(&localPos);
            const GfVec3d center = bodyXformToWorld.Transform(GfVec3d(localPos));
            const SdfPath attachmentPath = bodyPath.AppendElementToken(child.GetPath().GetNameToken());
            const SdfPathToSdfPath::const_iterator it = mChildToParent.find(attachmentPath);
            if (it != mChildToParent.end())
            {
                const SdfPath parentBodyPath = it->second.GetParentPath();
                const SdfPath parentXformPath = getSessionLayerXformPath(parentBodyPath, false);
                const UsdPrim parentXformPrim = gStage->GetPrimAtPath(parentXformPath);
                if (!parentXformPrim.IsValid())
                    continue;
                // get parent attachment geom prim:
                const UsdPrim parentAttachment =
                    gStage->GetPrimAtPath(parentXformPath.AppendElementToken(it->second.GetNameToken()));
                if (!parentAttachment)
                    continue;

                const GfMatrix4d parentXformToWorld = getBodyTransform(parentBodyPath, parentXformPrim);

                GfVec3f parentAttachmentLocalPos;
                const UsdAttribute parentTranslateAttr = parentAttachment.GetAttribute(kTranslateAttrName);
                if (!parentTranslateAttr)
                    continue;
                parentTranslateAttr.Get(&parentAttachmentLocalPos);
                const GfVec3d parentCenter = parentXformToWorld.Transform(GfVec3d(parentAttachmentLocalPos));

                carb::Float3 start, end;
                start.x = float(parentCenter[0]);
                start.y = float(parentCenter[1]);
                start.z = float(parentCenter[2]);
                end.x = float(center[0]);
                end.y = float(center[1]);
                end.z = float(center[2]);
                gDebugDraw->drawLine(start, kLineColorARGB, kLineWidth, end, kLineColorARGB, kLineWidth);
            }
        }
    }
}

void SpatialTendonManager::handlePrimResync(const SdfPath path)
{
    UsdPrimRange range(gStage->GetPrimAtPath(path));
    // printf("handlePrimResync with %s\n", path.GetText());
    for (UsdPrimRange::const_iterator cit = range.begin(); cit != range.end(); ++cit)
    {
        const UsdPrim& prim = *cit;
        // if the body was already parsed and has attachments, we need to check if attachments were added/removed
        if (mBodiesWithAttachments.count(prim.GetPath()))
        {
            mBufferPathsToUpdate.insert(prim.GetPath());
            mBufferSelectionChanged = true; // schedule visibility update based on selection
            cit.PruneChildren(); // cannot have a link below another link in the USD hierarchy
            // printf("Added %s to bodies to update\n", prim.GetPath().GetText());
        }
        else if (isRBLinkWithAttachment(prim))
        {
            // new body with attachments
            mBufferPathsToAdd.insert(prim.GetPath());
            mBufferSelectionChanged = true; // schedule visibility update based on selection
            cit.PruneChildren(); // cannot have a link below another link in the USD hierarchy
            // printf("Added %s to bodies to add\n", prim.GetPath().GetText());
        }
    }
}

void SpatialTendonManager::handlePrimRemove(const SdfPath path)
{
    // check if it is an attachment:
    const SdfPathToSdfPath::const_iterator cit = mSessionToBodies.find(path);
    if (cit != mSessionToBodies.cend())
    {
        mBufferPathsToDeletedAttachments.insert(cit->second.AppendElementToken(cit->first.GetNameToken()));
        mBufferSelectionChanged = true; // schedule visibility update based on selection
        return;
    }
    const auto iteratorPair = mBodiesToAttachmentName.FindSubtreeRange(path);
    for (SdfPathToAttachmentName::const_iterator cit = iteratorPair.first; cit != iteratorPair.second; cit++)
    {
        if (!cit->second.empty())
        {
            // printf("HandlePrimRemove REMOVE with %s\n", path.GetText());
            mBufferPathsToRemove.insert(path);
            mBufferSelectionChanged = true; // schedule visibility update based on selection
        }
    }
}

void SpatialTendonManager::clearBuffers(void)
{
    mBufferPathsToAdd.clear();
    mBufferPathsToUpdate.clear();
    mBufferPathsToRemove.clear();
    mBufferPathsToSetVisible.clear();
    mBufferPathsToUpdateTransform.clear();
    mBufferPathsToUpdateFromLocalPos.clear();
    mBufferPathsToUpdateToLocalPos.clear();
    mBufferPathsToUpdateParent.clear();
    mBufferPathsToDeletedAttachments.clear();
    mBufferAttachmentToSelect = SdfPath();
    mBufferSelectedPaths.clear();
    mBufferVisibilityDirty = false;
    mBufferRecomputeAttachmentRadius = false;
    mBufferSetNewAttachmentRadius = false;
    mBufferSelectionChanged = false;
}

void SpatialTendonManager::handleAttributeChange(const SdfPath path, const TfToken attributeName, const bool isXform)
{
    // static string members (do here and not globally to not have issues with static globals init order)
    static const std::string kLocalPosStr("localPos");// PhysxSchemaTokens.Get()->localPos.GetString());
    static const std::string kParentAttachmentStr("parentAttachment");// PhysxSchemaTokens.Get()->parentAttachment.GetString());
    static const std::string kParentLinkStr("parentLink");//PhysxSchemaTokens.Get()->parentLink.GetString());

    // xform must always be handled because the change may have happened in the xform hierarchy above the deformable:
    if (isXform)
    {
        // use awesome path table to quickly find all candidate deformable paths to update:
        bool wasBodyOnStage = false;
        const auto iteratorPair = mBodiesToAttachmentName.FindSubtreeRange(path);
        for (SdfPathToAttachmentName::const_iterator cit = iteratorPair.first; cit != iteratorPair.second; cit++)
        {
            // it->first is a path in the subtree starting at path.
            // so if it is a RB with attachments and there is a corresponding xform, add it to the xform update buffer
            if (!cit->second.empty())
            {
                mBufferPathsToUpdateTransform.insert(cit->first);
                wasBodyOnStage = true;
            }
        }
        if (!wasBodyOnStage)
        {
            const SdfPathToSdfPath::const_iterator cit = mSessionToBodies.find(path);
            if (cit != mSessionToBodies.cend())
            {
                const SdfPath attachmentPath = cit->second.AppendElementToken(path.GetNameToken());
                mBufferPathsToUpdateToLocalPos.insert(attachmentPath);
            }
        }
        return;
    }

    // otherwise only handle changes that refer to a RB with attachments:
    if (!mBodiesWithAttachments.count(path))
        return;

    // need to manipulate strings to figure out instance and attribute names :(
    const std::string attributeStr = attributeName.GetString();
    // only handle physxTendon attributes:
    if (attributeStr.substr(0u, kPhysxTendonNamespace.size()) == kPhysxTendonNamespace)
    {
        mBufferSelectionChanged = true; // schedule visibility update based on selection
        // check parent attachment property:
        if (attributeStr.substr(attributeStr.size() - kParentAttachmentStr.size()) == kParentAttachmentStr)
        {
            const size_t instanceNameStrLen =
                attributeStr.size() - 2u - kParentAttachmentStr.size() - kPhysxTendonNamespace.size();
            const std::string instanceName = attributeStr.substr(kPhysxTendonNamespace.size() + 1, instanceNameStrLen);
            const SdfPath attachmentPath = path.AppendElementString(instanceName);
            mBufferPathsToUpdateParent.insert(attachmentPath);
            // printf("Got parent attachment token change for path %s\n", attachmentPath.GetText());
        }
        else if (attributeStr.substr(attributeStr.size() - kParentLinkStr.size()) == kParentLinkStr)
        {
            const size_t instanceNameStrLen =
                attributeStr.size() - 2u - kParentLinkStr.size() - kPhysxTendonNamespace.size();
            const std::string instanceName = attributeStr.substr(kPhysxTendonNamespace.size() + 1, instanceNameStrLen);
            const SdfPath attachmentPath = path.AppendElementString(instanceName);
            mBufferPathsToUpdateParent.insert(attachmentPath);
            // printf("Got parent link rel change for path %s\n", attachmentPath.GetText());
        }
        else if (attributeStr.substr(attributeStr.size() - kLocalPosStr.size()) == kLocalPosStr)
        {
            const size_t instanceNameStrLen =
                attributeStr.size() - 2u - kLocalPosStr.size() - kPhysxTendonNamespace.size();
            const std::string instanceName = attributeStr.substr(kPhysxTendonNamespace.size() + 1, instanceNameStrLen);
            const SdfPath attachmentPath = path.AppendElementString(instanceName);
            mBufferPathsToUpdateFromLocalPos.insert(attachmentPath);
            // printf("Got localPos change for path %s\n", attachmentPath.GetText());
        }
    }
}

bool SpatialTendonManager::isAttachmentAPI(const TfToken apiName, TfToken& apiNameOut, TfToken& instanceNameOut) const
{
    static const size_t kMinApiNameLength = kAttachmentAPIName.size() + 1; // +1 for colon
    static const size_t kRootLeafOffset = kAttachmentRootAPIName.size() + 1; // +1 for colon
    static const size_t kApiCommonNameOffset = kAttachmentAPICommonName.size();
    const std::string apiNameStr = apiName.GetString();
    if (apiNameStr.size() < kMinApiNameLength)
        return false;

    // otherwise we can check:
    if (TfStringStartsWith(apiNameStr, kAttachmentAPICommonName.c_str()))
    {
        // it is an attachment API
        if (apiNameStr[kApiCommonNameOffset] == 'A')
        {
            instanceNameOut = TfToken(apiNameStr.substr(kMinApiNameLength));
            apiNameOut = kAttachmentAPIToken;
        }
        else if (apiNameStr[kApiCommonNameOffset] == 'L')
        {
            instanceNameOut = TfToken(apiNameStr.substr(kRootLeafOffset));
            apiNameOut = kAttachmentLeafAPIToken;
        }
        else
        {
            CARB_ASSERT(apiNameStr[kApiCommonNameOffset] == 'R');
            instanceNameOut = TfToken(apiNameStr.substr(kRootLeafOffset));
            apiNameOut = kAttachmentRootAPIToken;
        }
        return true;
    }
    return false;
}

void SpatialTendonManager::addAttachment(const SdfPath bodyPath, const TfToken apiName, const TfToken instanceName)
{
    // issue warning about runtime additions
    if (gPhysX->isRunning())
    {
        CARB_LOG_WARN("PhysX Spatial Tendon: Cannot add attachment %s to simulation while simulation is running. "
            "It is displayed but does not affect the simulation.",
            bodyPath.AppendElementToken(instanceName).GetText());
    }
    // update parent/child maps
    insertInParentChildMaps(bodyPath.AppendElementToken(instanceName));
    SdfPathToAttachmentName::iterator it = mBodiesToAttachmentName.find(bodyPath);
    // if not in table, insert a new set:
    if (it == mBodiesToAttachmentName.end())
    {
        const std::pair<SdfPathToAttachmentName::iterator, bool> insertResult =
            mBodiesToAttachmentName.insert({ bodyPath, TokenToTokenMap() });
        CARB_ASSERT(insertResult.second);
        insertResult.first->second.insert({ instanceName, apiName });
    }
    else
    {
        it->second.insert({ instanceName, apiName });
    }
}

void SpatialTendonManager::addBody(const SdfPath bodyPath)
{
    const UsdPrim prim = gStage->GetPrimAtPath(bodyPath);
    if (!prim.IsValid())
    {
        return;
    }
    // insert into bodies with attachments
    mBodiesWithAttachments.insert(bodyPath);
    // process attachments
    const TfTokenVector appliedSchemas = prim.GetPrimTypeInfo().GetAppliedAPISchemas();
    for (const TfToken api : appliedSchemas)
    {
        TfToken instanceName;
        TfToken apiName;
        if (isAttachmentAPI(api, apiName, instanceName))
        {
            addAttachment(bodyPath, apiName, instanceName);
        }
    }
    mBufferRecomputeAttachmentRadius = true;
    mBufferVisibilityDirty = true;
}

void SpatialTendonManager::updateBody(const SdfPath bodyPath)
{
    // Only update if body still exists:
    const UsdPrim prim = gStage->GetPrimAtPath(bodyPath);
    if (!prim)
        return;
    // get set of currently applied APIs/attachments
    const SdfPathToAttachmentName::const_iterator cit = mBodiesToAttachmentName.find(bodyPath);
    if (cit == mBodiesToAttachmentName.end())
        return;
    // copy map to remove found ones so we can delete any api left:
    TokenToTokenMap existingAttachments = cit->second;
    const size_t initialNumAttachments = existingAttachments.size();
    const TfTokenVector appliedSchemas = prim.GetPrimTypeInfo().GetAppliedAPISchemas();
    bool addedAttachment = false;
    bool hasRBAPI = false;
    for (const TfToken api : appliedSchemas)
    {
        TfToken instanceName;
        TfToken apiName;
        if (isAttachmentAPI(api, apiName, instanceName))
        {
            const size_t numErased = existingAttachments.erase(instanceName);
            if (numErased == 0u)
            {
                // new api:
                addedAttachment = true;
                addAttachment(bodyPath, apiName, instanceName);
                // printf("Adding instance %s to body %s\n", instanceName.GetText(), bodyPath.GetText());
            }
        }
        else if (api == kPhysicsRigidBodyAPIToken)
        {
            hasRBAPI = true;
        }
    }
    // if RB api was removed; delete body:
    if (!hasRBAPI)
    {
        // printf("Removing body %s after update because no RB API applied anymore\n", bodyPath.GetText());
        removeBody(bodyPath);
        return;
    }

    // if no attachment added and all removed, can delete body:
    if (!addedAttachment && initialNumAttachments == existingAttachments.size())
    {
        // printf("Removing body %s after update because no attachments left\n", bodyPath.GetText());
        removeBody(bodyPath);
        return;
    }
    // now go through items left over in existingAttachments and delete them:
    for (const std::pair<TfToken, TfToken>& entryToDelete : existingAttachments)
    {
        // printf("Removing deleted attachment %s from body %s\n", entryToDelete.first.GetText(), bodyPath.GetText());
        removeAttachment(bodyPath, entryToDelete.first);
    }
}

void SpatialTendonManager::printMaps(const bool bodyUpdate, const bool selectionUpdate)
{
    if (bodyUpdate)
    {
        printf("== mBodiesToAttachments ==\n");
        for (const auto element : mBodiesToAttachmentName)
        {
            if (!element.second.empty())
            {
                printf("Body %s:\n", element.first.GetText());
                for (auto keyValuePair : element.second)
                {
                    printf("-- %s, %s\n", keyValuePair.first.GetText(), keyValuePair.second.GetText());
                }
            }
        }

        printf("\n== mParentToChildren ==\n");
        for (const auto element : mParentToChildren)
        {
            printf("%s: ", element.first.GetText());
            for (const auto child : element.second)
            {
                printf("%s, ", child.GetText());
            }
            printf("\n");
        }
        printf("\n== mChildToParent ==\n");
        for (const auto element : mChildToParent)
        {
            printf("%s: %s\n", element.first.GetText(), element.second.GetText());
        }
    }
    if (selectionUpdate)
    {
        printf("\n== mBufferPathToSetVisible ==\n");
        for (const auto element : mBufferPathsToSetVisible)
        {
            printf("%s: %s\n", element.GetText(), element.GetText());
        }
    }
}

void SpatialTendonManager::removeBody(const SdfPath bodyPath)
{
    // remove all attachments
    const SdfPathToAttachmentName::const_iterator tableCit = mBodiesToAttachmentName.find(bodyPath);
    // if there is no body, there is no work:
    if (tableCit == mBodiesToAttachmentName.end())
        return;

    // otherwise, process:
    std::vector<TfToken> instanceNames;
    instanceNames.reserve(tableCit->second.size());
    // remove attachment will remove the attachment from the bodyToAttachmentName table,
    // so need to buffer instance names to avoid hitting invalid map iterators
    for (const std::pair<TfToken, TfToken>& keyValuePair : tableCit->second)
    {
        instanceNames.push_back(keyValuePair.first);
    }
    for (const TfToken instanceName : instanceNames)
    {
        removeAttachment(bodyPath, instanceName);
    }
    // remove session layer xform if there:
    const SdfPath xformPath = getSessionLayerXformPath(bodyPath, false);
    if (!xformPath.IsEmpty())
        gStage->RemovePrim(xformPath);
    mBodiesToSessionXforms.erase(bodyPath);
    mBodiesToAttachmentName.erase(bodyPath);
    mBodiesWithAttachments.erase(bodyPath);
    mBufferRecomputeAttachmentRadius = true;
    mBufferVisibilityDirty = true;
}

SdfPath SpatialTendonManager::checkConnection(void)
{
    // check if the user selected an attachment and update visibility based on that
    const SdfPathToSdfPath::const_iterator itA = mSessionToBodies.find(mBufferSelectedPaths[0]);
    if (itA == mSessionToBodies.cend())
        return SdfPath();
    const SdfPathToSdfPath::const_iterator itB = mSessionToBodies.find(mBufferSelectedPaths[1]);
    if (itB == mSessionToBodies.cend())
        return SdfPath();

    // have two session bodies - get their parent paths:
    const TfToken instanceA = mBufferSelectedPaths[0].GetNameToken();
    const SdfPath attachmentA = itA->second.AppendElementToken(instanceA);
    const SdfPath parentA = getParentAttachmentPath(attachmentA);
    const TfToken instanceB = mBufferSelectedPaths[1].GetNameToken();
    const SdfPath attachmentB = itB->second.AppendElementToken(instanceB);
    const SdfPath parentB = getParentAttachmentPath(itB->second.AppendElementToken(instanceB));

    const bool hasParentA = !parentA.IsEmpty();
    const bool isRootA = isRootAttachment(attachmentA);
    const bool isLeafA = isLeafAttachment(attachmentA);
    const bool hasParentB = !parentB.IsEmpty();
    const bool isRootB = isRootAttachment(attachmentB);
    const bool isLeafB = isLeafAttachment(attachmentB);

    if (hasParentA && hasParentB)
    {
        CARB_LOG_WARN(
            "PhysX Spatial Tendon: Cannot create connection between %s and %s as both already have a parent set.",
            attachmentA.GetText(), attachmentB.GetText());
        return SdfPath();
    }

    if (isRootA && isRootB)
    {
        CARB_LOG_WARN("PhysX Spatial Tendon: Cannot create a connection between two root attachments.");
        return SdfPath();
    }

    if (isLeafA && isLeafB)
    {
        CARB_LOG_WARN("PhysX Spatial Tendon: Cannot create a connection between two leaf attachments.");
        return SdfPath();
    }

    // if both attachment do NOT have parents, we define the connection direction from first to second selected, so from
    // A (child) to B (parent)
    SdfPath parentPath = attachmentB;
    SdfPath childPath = attachmentA;

    // if there is exactly one root, the root must be the parent:
    if (isRootA != isRootB)
    {
        if (isRootA)
        {
            childPath = attachmentB;
            parentPath = attachmentA;
            if (hasParentB)
            {
                CARB_LOG_WARN(
                    "PhysX Spatial Tendon: Cannot set root %s as parent of %s as there is already parent information set.",
                    attachmentA.GetText(), attachmentB.GetText());
                return SdfPath();
            }
        }
        else if (hasParentA)
        {
            CARB_LOG_WARN(
                "PhysX Spatial Tendon: Cannot set root %s as parent of %s as there is already parent information set.",
                attachmentB.GetText(), attachmentA.GetText());
            return SdfPath();
        }
    }
    // if there is exactly one leaf, the leaf must be the child
    else if (isLeafA != isLeafB)
    {
        if (isLeafB)
        {
            childPath = attachmentB;
            parentPath = attachmentA;
            if (hasParentB)
            {
                CARB_LOG_WARN(
                    "PhysX Spatial Tendon: Cannot set leaf %s as child of %s as there is already parent information set.",
                    attachmentB.GetText(), attachmentA.GetText());
                return SdfPath();
            }
        }
        else if (hasParentA)
        {
            CARB_LOG_WARN(
                "PhysX Spatial Tendon: Cannot set leaf %s as child of %s as there is already parent information set.",
                attachmentA.GetText(), attachmentB.GetText());
            return SdfPath();
        }
    }
    // if only one of the attachments has no parent, the direction is from no parent (child) to attachment with a parent
    else if (hasParentA != hasParentB)
    {
        if (!hasParentB)
        {
            // swap child/parent
            parentPath = attachmentA;
            childPath = attachmentB;
        }
    }

    // set parent in target:
    runSetParentCommand(childPath, parentPath);
    // return attachment path that got a new parent
    return childPath;
}

void SpatialTendonManager::updateVisibilityAndCreateNewAttachmentGeoms(void)
{
    SdfPath newRequestedAttachmentSelection = SdfPath();
    mVisibleBodies.clear();
    if (mBufferAttachmentToSelect.IsEmpty())
    {
        SdfPathVector selectedPaths;
        switch (mMode)
        {
        case VisualizerMode::eAll:
            selectedPaths.push_back(gStage->GetPseudoRoot().GetPath());
            break;
        case VisualizerMode::eNone:
            break;
        case VisualizerMode::eSelected:
            selectedPaths = mBufferSelectedPaths;
        default:
            break;
        }

        for (const SdfPath path : selectedPaths)
        {
            // check if the user selected an attachment and update visibility based on that
            const SdfPathToSdfPath::const_iterator cit = mSessionToBodies.find(path);
            if (cit != mSessionToBodies.cend())
            {
                const SdfPathToAttachmentName::const_iterator tableCit = mBodiesToAttachmentName.find(cit->second);
                if (tableCit == mBodiesToAttachmentName.end() || tableCit->second.empty())
                    continue;
                traverseTendonVisibility(tableCit->first, tableCit->second);
            }
            else // find any RBs with attachments in subtree (selection can be ancestor of RBs with attachments):
            {
                const auto iteratorPair = mBodiesToAttachmentName.FindSubtreeRange(path);
                for (SdfPathToAttachmentName::const_iterator cit = iteratorPair.first; cit != iteratorPair.second; cit++)
                {
                    // cit->first is body path, cit->second is TokenToTokenMap (instanceName, apiName)
                    // if it is a body with attachments
                    if (!cit->second.empty())
                    {
                        // process
                        traverseTendonVisibility(cit->first, cit->second);
                    }
                }
            }
        }
    }
    else
    {
        // see if attachment exists:
        const SdfPath bodyPath = mBufferAttachmentToSelect.GetParentPath();
        const SdfPathToAttachmentName::const_iterator cit = mBodiesToAttachmentName.find(bodyPath);
        if (cit == mBodiesToAttachmentName.end())
            return;
        // check if the parent instance exists and it is NOT a leaf
        const TfToken instanceName = mBufferAttachmentToSelect.GetNameToken();
        const TokenToTokenMap::const_iterator tokenIt = cit->second.find(instanceName);
        if (tokenIt == cit->second.cend())
            return;
        // if it does not exist, it was just created by the GUI. Buffer it for selection after creation.
        newRequestedAttachmentSelection = getSessionLayerXformPath(bodyPath, true).AppendElementToken(instanceName);
        traverseTendonVisibility(cit->first, cit->second);
    }

    // now update visibility for all bodies with attachments based on the determined visibility set:
    for (const SdfPath path : mBodiesWithAttachments)
    {
        if (mBufferPathsToSetVisible.count(path))
        {
            setVisible(path);
            mVisibleBodies.push_back(path);
            // queue for xform update
            mBufferPathsToUpdateTransform.insert(path);
        }
        else
        {
            setInvisible(path);
        }
    }

    // after the attachment helper was possibly created, can select it
    if (!newRequestedAttachmentSelection.IsEmpty())
    {
        // update user-facing selection to newly created session prim:
        omni::usd::UsdContext::getContext()->getSelection()->setSelectedPrimPathsV2({ newRequestedAttachmentSelection });
        mBufferSelectedPaths = { newRequestedAttachmentSelection };
        mBufferSelectionChanged = true; // trigger gizmo creation
    }
}

void SpatialTendonManager::traverseTendonVisibility(const SdfPath bodyPath, const TokenToTokenMap& attachments)
{
    if (mBufferPathsToSetVisible.count(bodyPath))
    {
        // not a body with attachments or already processed, no need to process further
        return;
    }

    // add this path to the visible set:
    mBufferPathsToSetVisible.insert(bodyPath);

    // process attachments:
    for (const std::pair<TfToken, TfToken>& attachment : attachments)
    {
        const SdfPath attachmentPath = bodyPath.AppendElementToken(attachment.first);
        // 1. Parent
        {
            const SdfPathToSdfPath::const_iterator childToParentIt = mChildToParent.find(attachmentPath);
            if (childToParentIt != mChildToParent.end())
            {
                // Get candidate parent path:
                const SdfPath parentBodyPath = childToParentIt->second.GetParentPath();
                const SdfPathToAttachmentName::const_iterator bodyToAttachmentNamesIt =
                    mBodiesToAttachmentName.find(parentBodyPath);
                if (bodyToAttachmentNamesIt != mBodiesToAttachmentName.end())
                {
                    // check if the parent instance exists and it is NOT a leaf
                    const TokenToTokenMap::const_iterator tokenIt =
                        bodyToAttachmentNamesIt->second.find(childToParentIt->second.GetNameToken());
                    if (tokenIt != bodyToAttachmentNamesIt->second.cend() && tokenIt->second != kAttachmentLeafAPIToken)
                        traverseTendonVisibility(bodyToAttachmentNamesIt->first, bodyToAttachmentNamesIt->second);
                }
            }
        }
        // 2. children
        {
            const SdfPathToSdfPathSet::const_iterator parentToChildrenIt = mParentToChildren.find(attachmentPath);
            if (parentToChildrenIt != mParentToChildren.end())
            {
                // there are children:
                const pxr::SdfPathSet& children = parentToChildrenIt->second;
                for (const SdfPath child : children)
                {
                    const SdfPath childBodyPath = child.GetParentPath();
                    const SdfPathToAttachmentName::const_iterator bodyToAttachmentNamesIt =
                        mBodiesToAttachmentName.find(childBodyPath);
                    if (bodyToAttachmentNamesIt != mBodiesToAttachmentName.end())
                    {
                        // if it is a body with attachments
                        if (!bodyToAttachmentNamesIt->second.empty())
                        {
                            traverseTendonVisibility(bodyToAttachmentNamesIt->first, bodyToAttachmentNamesIt->second);
                        }
                    }
                }
            }
        }
    }
}

void SpatialTendonManager::removeAttachment(const SdfPath bodyPath, const TfToken instanceName)
{
    const SdfPath attachmentPath = bodyPath.AppendElementToken(instanceName);
    removeFromParentChildMaps(attachmentPath);
    // remove from body to attachments map:
    SdfPathToAttachmentName::iterator it = mBodiesToAttachmentName.find(bodyPath);
    if (it != mBodiesToAttachmentName.end())
    {
        it->second.erase(instanceName);
    }
    // delete session layer geom:
    const SdfPath xformPath = getSessionLayerXformPath(bodyPath, false);
    if (xformPath.IsEmpty())
        return;
    const SdfPath geomPrimPath = xformPath.AppendElementToken(instanceName);
    const UsdPrim geomPrim = gStage->GetPrimAtPath(geomPrimPath);
    if (geomPrim)
    {
        gStage->RemovePrim(geomPrimPath);
    }
    // try to erase from session map in any case (may be deleted by user)
    mSessionToBodies.erase(geomPrimPath);
    mBufferVisibilityDirty = true;
}

void SpatialTendonManager::insertInParentChildMaps(const SdfPath attachmentPath)
{
    const SdfPath parentAttachmentPath = getParentAttachmentPath(attachmentPath);
    if (parentAttachmentPath.IsEmpty())
        return;

    mChildToParent.insert({ attachmentPath, parentAttachmentPath });
    SdfPathToSdfPathSet::iterator parentToChildrenIt = mParentToChildren.find(parentAttachmentPath);
    if (parentToChildrenIt != mParentToChildren.end())
    {
        parentToChildrenIt->second.insert(attachmentPath);
    }
    else
    {
        mParentToChildren.insert({ parentAttachmentPath, SdfPathSet({ attachmentPath }) });
    }
}

void SpatialTendonManager::removeFromParentChildMaps(const SdfPath attachmentPath)
{
    // 1) remove the attachment's parent-child relationship from the parent-child map
    SdfPath toErase = SdfPath();
    for (SdfPathToSdfPathSet::iterator it = mParentToChildren.begin(); it != mParentToChildren.end(); ++it)
    {
        size_t erased = it->second.erase(attachmentPath);
        if (erased)
        {
            if (it->second.empty())
                toErase = it->first;
            break;
        }
    }

    // purge empty child sets:
    if (!toErase.IsEmpty())
        mParentToChildren.erase(toErase);

    // 2) remove the attachment's parent-child relationship from the child to parent map
    mChildToParent.erase(attachmentPath);
}

void SpatialTendonManager::setVisible(const pxr::SdfPath bodyPath)
{
    const SdfPath rootXformPath = getSessionLayerXformPath(bodyPath, true);
    UsdGeomXform xform = UsdGeomXform::Get(gStage, rootXformPath);
    if (!xform)
    {
        xform = UsdGeomXform::Define(gStage, rootXformPath);
        xform.AddTransformOp(UsdGeomXformOp::PrecisionDouble).Set(GfMatrix4d(0.0));
    }

    xform.MakeVisible();

    // check/update the attachment geoms:
    const SdfPathToAttachmentName::const_iterator entry = mBodiesToAttachmentName.find(bodyPath);
    CARB_ASSERT(!entry->second.empty());
    // attachment = {instanceName, apiName}
    for (const std::pair<TfToken, TfToken>& attachment : entry->second)
    {
        const SdfPath geomPath = rootXformPath.AppendElementToken(attachment.first);
        const std::pair<SdfPathToSdfPath::const_iterator, bool> inserted =
            mSessionToBodies.insert({ geomPath, bodyPath });
        // if it was inserted, need to create the geom
        if (inserted.second)
        {
            const SdfPath attachmentPath = bodyPath.AppendElementToken(attachment.first);
            mBufferPathsToUpdateFromLocalPos.insert(attachmentPath);
            createAttachmentGeom(geomPath, attachment.second);
            // update here to get the init sequence right (there will be xform updates triggered, too)
            updateFromLocalPos(attachmentPath);
        }
    }
}

void SpatialTendonManager::setInvisible(const pxr::SdfPath bodyPath)
{
    const SdfPath rootXformPath = getSessionLayerXformPath(bodyPath, false);
    if (rootXformPath.IsEmpty())
        return;
    UsdGeomXform xform = UsdGeomXform::Define(gStage, rootXformPath);
    if (xform)
    {
        xform.MakeInvisible();
    }
}

SdfPath SpatialTendonManager::getSessionLayerXformPath(const SdfPath bodyPath, const bool createIfNotExist)
{
    // check if it already exists:
    SdfPathToSdfPath::const_iterator element = mBodiesToSessionXforms.find(bodyPath);
    if (element != mBodiesToSessionXforms.end())
    {
        return element->second;
    }

    if (!createIfNotExist)
    {
        // return if it shall not be created
        return SdfPath();
    }

    // otherwise create it:
    // session layer root is only created if necessary
    if (mSessionLayerScopePath.IsEmpty())
    {
        // TODO preist: adapt root based on already existing root?
        mSessionLayerScopePath = SdfPath("/PhysxTendonSpatialAttachmentGeoms");
        CARB_ASSERT(!gStage->GetPrimAtPath(mSessionLayerScopePath));
        UsdGeomScope rootScope = UsdGeomScope::Define(gStage, mSessionLayerScopePath);
        CARB_ASSERT(rootScope);
        omni::kit::EditorUsd::setNoDelete(rootScope.GetPrim(), true);
        omni::kit::EditorUsd::setHideInStageWindow(rootScope.GetPrim(), true);
    }

    std::string nameString = bodyPath.GetString();
    std::replace(nameString.begin(), nameString.end(), '/', '_');
    const SdfPath newXformPath = mSessionLayerScopePath.AppendElementString(nameString);
    mBodiesToSessionXforms.insert({ bodyPath, newXformPath });
    return newXformPath;
}

void SpatialTendonManager::createAttachmentGeom(const SdfPath geomPath, const TfToken apiNameToken)
{
    UsdGeomSphere attSphere = UsdGeomSphere::Define(gStage, geomPath);
    attSphere.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(GfVec3f(0.0f, 0.0f, 0.0f));
    attSphere.AddScaleOp(UsdGeomXformOp::PrecisionDouble).Set(GfVec3d(0.0, 0.0, 0.0));
    GfVec3f color = kColorAttachment;
    if (apiNameToken == kAttachmentLeafAPIToken)
        color = kColorLeaf;
    else if (apiNameToken == kAttachmentRootAPIToken)
        color = kColorRoot;
    VtArray<GfVec3f> colorArray(1, color);
    attSphere.CreateDisplayColorAttr().Set(colorArray);
    attSphere.CreateRadiusAttr().Set(1.0);
    omni::kit::EditorUsd::setHideInStageWindow(attSphere.GetPrim(), true);
}

void SpatialTendonManager::updateFromLocalPos(const SdfPath attachmentPath)
{
    // first get localPos:
    const TfToken instanceName = attachmentPath.GetNameToken();
    const SdfPath bodyPath = attachmentPath.GetParentPath();
    const UsdPrim bodyPrim = gStage->GetPrimAtPath(bodyPath);
    PhysxSchemaPhysxTendonAttachmentAPI api = PhysxSchemaPhysxTendonAttachmentAPI::Get(bodyPrim, instanceName);
    if (!api)
        return;
    GfVec3f localPos;
    const UsdAttribute localPosAttr = api.GetLocalPosAttr();
    if (!localPosAttr.IsValid())
        return;
    localPosAttr.Get(&localPos);
    const SdfPath sessionGeomPath = getSessionLayerXformPath(bodyPath, false).AppendElementToken(instanceName);
    UsdPrim geomPrim = gStage->GetPrimAtPath(sessionGeomPath);
    if (!geomPrim.IsValid())
        return;
    const UsdAttribute translateAttr = geomPrim.GetAttribute(kTranslateAttrName);
    if (!translateAttr.IsValid())
        return;
    translateAttr.Set(localPos);
}

void SpatialTendonManager::updateXformTransform(const SdfPath bodyPath)
{
    const SdfPath sessionXformPath = getSessionLayerXformPath(bodyPath, false);
    const UsdPrim xformPrim = gStage->GetPrimAtPath(sessionXformPath);
    if (!xformPrim.IsValid())
        return;
    const UsdPrim bodyPrim = gStage->GetPrimAtPath(bodyPath);
    if (!bodyPrim.IsValid())
        return;
    const GfMatrix4d bodyLocalToWorld = getBodyTransform(bodyPath, bodyPrim);
    const UsdAttribute transformMatrixAttr = xformPrim.GetAttribute(kTransformMatrixAttrName);
    if (!transformMatrixAttr.IsValid())
        return;
    transformMatrixAttr.Set(bodyLocalToWorld);

    GfVec3d scale = GfTransform(bodyLocalToWorld).GetScale();
    for (int i = 0; i < 3; ++i)
    {
        if (std::abs(scale[i]) <= 1e-6)
            // sanitize in case of near zero:
            scale[i] = std::signbit(scale[i]) ? -1e6 : 1e6;
        else
            scale[i] = 1.0 / scale[i];
    }
    // set new scale
    for (const UsdPrim spherePrim : xformPrim.GetChildren())
    {
        const UsdAttribute scaleAttr = spherePrim.GetAttribute(kScaleAttrName);
        if (!scaleAttr.IsValid())
            continue;
        scaleAttr.Set(scale);
    }
}

void SpatialTendonManager::updateToLocalPos(const SdfPath attachmentPath)
{
    // get new translation:
    const TfToken instanceName = attachmentPath.GetNameToken();
    const SdfPath bodyPath = attachmentPath.GetParentPath();
    const SdfPath sessionGeomPath = getSessionLayerXformPath(bodyPath, false).AppendElementToken(instanceName);
    UsdPrim geomPrim = gStage->GetPrimAtPath(sessionGeomPath);
    if (!geomPrim.IsValid())
        return;
    GfVec3f localPos;
    const UsdAttribute translateAttr = geomPrim.GetAttribute(kTranslateAttrName);
    if (!translateAttr.IsValid())
        return;
    translateAttr.Get(&localPos);
    // printf("Updating %s TO local pos %f, %f, %f\n", attachmentPath.GetText(), localPos[0], localPos[1], localPos[2]);
    // and write to local pos:
    const UsdPrim bodyPrim = gStage->GetPrimAtPath(bodyPath);
    PhysxSchemaPhysxTendonAttachmentAPI api = PhysxSchemaPhysxTendonAttachmentAPI::Get(bodyPrim, instanceName);
    if (!api)
        return;
    api.CreateLocalPosAttr().Set(localPos);
}

void SpatialTendonManager::selectSpatialTendonAttachmentHelper(const SdfPath linkBodyPath, TfToken instanceName)
{
    if (!linkBodyPath.IsPrimPath() || linkBodyPath.IsEmpty() || instanceName.IsEmpty())
        return;

    mBufferAttachmentToSelect = linkBodyPath.AppendElementToken(instanceName);
    mBufferVisibilityDirty = true;
}

bool SpatialTendonManager::isRootAttachment(const pxr::SdfPath attachmentPath)
{
    const TfToken apiToken = getAttachmentAPI(attachmentPath);
    if (apiToken.IsEmpty())
        return false;
    return apiToken == kAttachmentRootAPIToken;
}

bool SpatialTendonManager::isLeafAttachment(const pxr::SdfPath attachmentPath)
{
    const TfToken apiToken = getAttachmentAPI(attachmentPath);
    if (apiToken.IsEmpty())
        return false;
    return apiToken == kAttachmentLeafAPIToken;
}

SdfPath SpatialTendonManager::getParentAttachmentPath(const SdfPath attachmentPath)
{
    const SdfPath bodyPath = attachmentPath.GetParentPath();
    const TfToken instanceName = attachmentPath.GetNameToken();
    const UsdPrim prim = gStage->GetPrimAtPath(bodyPath);
    PhysxSchemaPhysxTendonAttachmentAPI api = PhysxSchemaPhysxTendonAttachmentAPI::Get(prim, instanceName);
    if (!api)
        return SdfPath();

    SdfPathVector targets;
    api.GetParentLinkRel().GetTargets(&targets);
    if (targets.size() > 0)
    {
        const SdfPath parentBodyPath = targets[0];
        TfToken parentInstanceName;
        api.GetParentAttachmentAttr().Get(&parentInstanceName);
        if (parentBodyPath.IsEmpty() || parentInstanceName.IsEmpty())
            // only add if info is complete
            return SdfPath();
        return parentBodyPath.AppendElementToken(parentInstanceName);
    }
    return SdfPath();
}

void SpatialTendonManager::computeAttachmentRadiusFromBodies(void)
{
    // this function computes a heuristic to use as attachment helper sphere radius
    // it sets the radius to a ratio * the average max scaled local BB of all bodies with attachments
    if (mBodiesWithAttachments.empty())
        return;
    double average = 0.0f;
    size_t numBodies = mBodiesWithAttachments.size();
    for (const SdfPath path : mBodiesWithAttachments)
    {
        UsdPrim bodyPrim = gStage->GetPrimAtPath(path);
        if (!bodyPrim)
        {
            --numBodies;
            continue;
        }
        const GfVec3d localSize = mBBCache.ComputeLocalBound(bodyPrim).GetRange().GetSize();
        const GfVec3d scale = GfTransform(getBodyTransform(path, bodyPrim)).GetScale();
        const GfVec3d worldSize = GfCompMult(localSize, scale);
        double max = worldSize[0];
        if (max < worldSize[1])
            max = worldSize[1];
        if (max < worldSize[2])
            max = worldSize[2];
        average += max;
    }
    double newRadius = kAttachmentDiameterRatio * 0.5 * average / double(numBodies);
    if (newRadius != mAttachmentRadiusFromBodyGeometry)
    {
        mBufferSetNewAttachmentRadius = true;
        mAttachmentRadiusFromBodyGeometry = newRadius;
    }
}

void SpatialTendonManager::setAttachmentRadiusScale(const double scale)
{
    // only trigger update if new scale
    if (scale != mAttachmentRadiusScale)
    {
        mAttachmentRadiusScale = scale;
        mBufferSetNewAttachmentRadius = true;
    }
}

void SpatialTendonManager::updateVisibleAttachmentRadii(void)
{
    if (mSessionLayerScopePath.IsEmpty())
        return; // nothing to update
    const double newRadius = mAttachmentRadiusFromBodyGeometry * mAttachmentRadiusScale;
    const UsdPrim scopePrim = gStage->GetPrimAtPath(mSessionLayerScopePath);
    for (const UsdPrim xformPrim : scopePrim.GetChildren())
    {
        const UsdGeomXform xform(xformPrim);
        if (!xform)
            continue;
        TfToken visibleAttr;
        xform.GetVisibilityAttr().Get(&visibleAttr);
        if (visibleAttr == UsdGeomTokens->inherited)
        {
            // set new radii:
            for (const UsdPrim spherePrim : xformPrim.GetChildren())
            {
                const UsdGeomSphere sphere = UsdGeomSphere(spherePrim);
                if (!sphere)
                    continue;
                sphere.CreateRadiusAttr().Set(newRadius);
            }
        }
    }
}

void SpatialTendonManager::processSelectionChanged(void)
{
    if (mBufferSelectedPaths.size() == 1u)
    {
        const auto it = mSessionToBodies.find(mBufferSelectedPaths[0]);
        if (it != mSessionToBodies.end())
        {
            mLastSelectedAttachment = it->second.AppendElementToken(mBufferSelectedPaths[0].GetNameToken());
        }
        else if (!mLastSelectedAttachment.IsEmpty())
        {
            // see if the user clicked on the last selected attachment's parent:
            const SdfPathSet::const_iterator cit = mBodiesWithAttachments.find(mBufferSelectedPaths[0]);
            if (cit != mBodiesWithAttachments.end() && *cit == mLastSelectedAttachment.GetParentPath())
            {
                // scroll to that attachment:
                scrollPropertyWindowToAPI(getAttachmentAPI(mLastSelectedAttachment));
            }
            mLastSelectedAttachment = SdfPath();
        }
    }
    else if (mBufferSelectedPaths.size() == 0u && !mLastSelectedAttachment.IsEmpty())
    {
        omni::usd::UsdContext::getContext()->getSelection()->setSelectedPrimPathsV2(
            { mLastSelectedAttachment.GetParentPath() });
        // don't scroll here will be taken care of in next update:
    }
}

void SpatialTendonManager::runSetParentCommand(const SdfPath childAttachment, const SdfPath parentAttachment)
{
    std::ostringstream stringStream;
    stringStream << "import omni.kit.commands\n"
                 << "from pxr import Gf, Usd\n"
                 << "omni.kit.commands.execute(\n"
                 << "  'SetSpatialTendonAttachmentParentCommand',\n"
                 << "  child_attachment_path='" << childAttachment.GetString() << "',\n"
                 << "  parent_attachment_path='" << parentAttachment.GetString() << "')\n";
    const std::string commandStr = stringStream.str();
    omni::kit::PythonInterOpHelper::executeCommand(commandStr.data());
}

void SpatialTendonManager::runRemoveAPICommand(const SdfPath attachmentPath)
{
    const SdfPath bodyPath = attachmentPath.GetParentPath();
    const TfToken instanceName = attachmentPath.GetNameToken();
    const SdfPathToAttachmentName::iterator nameIt = mBodiesToAttachmentName.find(bodyPath);
    if (nameIt == mBodiesToAttachmentName.end())
        return;

    const TokenToTokenMap::const_iterator apiNameIt = nameIt->second.find(instanceName);
    if (apiNameIt == nameIt->second.end())
        return;
    const TfToken apiName = apiNameIt->second;

    std::ostringstream stringStream;
    stringStream << "import omni.kit.commands\n"
                 << "from pxr import Gf, Usd\n"
                 << "omni.kit.commands.execute(\n"
                 << "  'RemoveSpatialTendonAttachmentAPICommand',\n"
                 << "  attachment_path='" << attachmentPath.GetString() << "',\n"
                 << "  attachment_api='" << apiName.GetString() << "')\n";
    const std::string commandStr = stringStream.str();
    omni::kit::PythonInterOpHelper::executeCommand(commandStr.data());
}

void SpatialTendonManager::scrollPropertyWindowToAPI(const TfToken api)
{
    std::ostringstream stringStream;
    stringStream << "omni.kit.property.physx.utils.scroll_property_window_to_physx_component('" << api.GetString()
                 << "')\n";
    const std::string commandStr = stringStream.str();
    omni::kit::PythonInterOpHelper::executeCommand(commandStr.data());
}

TfToken SpatialTendonManager::getAttachmentAPI(const SdfPath attachmentPath)
{
    const auto it = mBodiesToAttachmentName.find(attachmentPath.GetParentPath());
    if (it == mBodiesToAttachmentName.end())
        return TfToken();

    const auto instIt = it->second.find(attachmentPath.GetNameToken());
    if (instIt == it->second.end())
        return TfToken();
    return instIt->second;
}


pxr::GfMatrix4d SpatialTendonManager::getBodyTransform(const pxr::SdfPath& bodyPath,
                                                       const pxr::UsdPrim& bodyPrim)
{
    if (mFabricEnabled && mFabricSync)
    {
        const usdrt::GfMatrix4d matrix = mFabricSync->computeWorldXform(omni::fabric::asInt(bodyPath));
        pxr::GfMatrix4d convertedMat;
        memcpy(&convertedMat, &matrix, sizeof(pxr::GfMatrix4d));
        return convertedMat;
    }

    return mXformCache.GetLocalToWorldTransform(bodyPrim);
}

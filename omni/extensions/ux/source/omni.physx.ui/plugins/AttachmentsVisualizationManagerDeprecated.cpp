// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "AttachmentsVisualizationManagerDeprecated.h"
#include "DeformableBodyVisualizationManagerDeprecated.h"

#include <algorithm>

#include <carb/profiler/Profile.h>
#include <omni/kit/EditorUsd.h>
#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/Selection.h>
#include <omni/timeline/ITimeline.h>
#include <private/omni/physx/PhysxUsd.h>
#include <omni/physx/IPhysxSettings.h>
#include <private/omni/physx/IPhysxAttachmentPrivate.h>
#include <private/omni/physx/IPhysxUsdLoad.h>

using namespace omni::physx::ui;
using namespace omni::physx::usdparser;
using namespace pxr;

extern UsdStageRefPtr gStage;
extern omni::timeline::TimelinePtr gTimeline;
extern carb::settings::ISettings* gSettings;
extern omni::physx::IPhysxAttachmentPrivate* gPhysXAttachmentPrivate;
extern uint8_t gProxySelectionGroup;
extern bool gBlockNoticeHandle;
extern omni::physx::usdparser::IPhysxUsdLoad* gUsdLoad;

static const TfToken transformMatrixAttrName{ "xformOp:transform" };
static const TfToken scaleAttrName{ "xformOp:scale" };
static const TfToken deformableBodyTetMeshCrcToken{ "physxDeformable:tetMeshCrc" };
static const TfToken physxDeformableSurfaceInputCrcToken{ "physxDeformableSurface:clothDataInputCrc" };
static const TfToken physxParticleClothInputCrcToken{ "physxParticle:clothDataInputCrc" };

static constexpr char kViewportGizmoScalePath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/scale";
static constexpr char kViewportGizmoConstantScaleEnabledPath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/constantScaleEnabled";
static constexpr char kViewportGizmoConstantScalePath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/constantScale";

static const float kEdgeToPointScale = 0.1f;

static const GfVec3f kColorSample[2] = { GfVec3f(0.6f, 0.0f, 0.6f), GfVec3f(0.3f, 0.0f, 0.3f) }; // violet/dark violet
static const GfVec3f kColorFilter[2] = { GfVec3f(0.0f, 0.7f, 0.5f), GfVec3f(0.0f, 0.4f, 0.2f) }; // cyan/dark cyan

static const float kRadiusSample[2] = { 1.2f, 0.8f };
static const float kRadiusFilter[2] = { 0.6f, 0.6f };

static const double kPrototypeRadiusZero = 0.0;

namespace
{
    SdfPath getSessionTargetPath(const SdfPath sessionAttachmentPath, int slot)
    {
        std::string nameString("target_");
        return sessionAttachmentPath.AppendElementString(nameString + std::to_string(slot));
    }

    SdfPath getSessionPointInstancerPrototypePath(const SdfPath sessionPointInstancerPath)
    {
        return sessionPointInstancerPath.AppendElementString(std::string("prototype"));
    }

    SdfPath getSessionSamplePointInstancerPath(const SdfPath sessionAttachmentPath, int slot)
    {
        SdfPath targetPath = getSessionTargetPath(sessionAttachmentPath, slot);
        return targetPath.AppendElementString("samplePoints");
    }

    SdfPath getSessionFilterPointInstancerPath(const SdfPath sessionAttachmentPath, int slot)
    {
        SdfPath targetPath = getSessionTargetPath(sessionAttachmentPath, slot);
        return targetPath.AppendElementString("filterPoints");
    }

    using TargetPathPair = std::array<SdfPath, 2>;

    TargetPathPair getAttachmentTargets(PhysxSchemaPhysxPhysicsAttachment attachment)
    {
        CARB_ASSERT(attachment);
        TargetPathPair dstTargets;
        SdfPathVector srcTargets;
        for (uint32_t slot = 0; slot < 2; ++slot)
        {
            attachment.GetActorRel(slot).GetTargets(&srcTargets);
            SdfPath targetPath = srcTargets.size() == 1 ? srcTargets[0] : SdfPath();
            dstTargets[slot] = targetPath;
        }
        return dstTargets;
    }

    void readSettings(float& visualizationScale, float& visualizationGap)
    {
        visualizationGap = 0.0f;
        visualizationScale = 1.0f;

        if (gSettings)
        {
            visualizationGap = gSettings->getAsFloat(omni::physx::kSettingVisualizationGap);

            if (gSettings->getAsBool(kViewportGizmoConstantScaleEnabledPath))
            {
                //kViewportGizmoConstantScalePath: default = 10.0f
                visualizationScale = 0.1f * gSettings->getAsFloat(kViewportGizmoConstantScalePath);
            }
            else
            {
                //kViewportGizmoScalePath: default = 1.0f
                //meterPerUnit: default = 0.01f
                visualizationScale = 0.01f * gSettings->getAsFloat(kViewportGizmoScalePath)/float(UsdGeomGetStageMetersPerUnit(gStage));
            }
        }
    }

    void fillinDummyPoint(UsdGeomPointInstancer& pointInstancer)
    {
        VtArray<GfVec3f> points(1, GfVec3f(0.0f));
        VtArray<int> prototypeIndices(1, 0);
        pointInstancer.GetPositionsAttr().Set(points);
        pointInstancer.GetProtoIndicesAttr().Set(prototypeIndices);
    }
}

AttachmentsVisualizationManagerDeprecated::AttachmentsVisualizationManagerDeprecated(const VisualizerMode mode, bool hideActor0, bool hideActor1) :
    mSessionAttachmentRoot{ SdfPath() },
    mMode(mode),
    mHideActor{hideActor0, hideActor1},
    mVisualizationGap(-1.0f),
    mVisualizationScale(-1.0f),
    mBufferVizDirty(false)
{
}

AttachmentsVisualizationManagerDeprecated::~AttachmentsVisualizationManagerDeprecated()
{
    release();
}

void AttachmentsVisualizationManagerDeprecated::setDeformableBodyVisualizationManager(DeformableBodyVisualizationManagerDeprecated& deformableBodyVisualizationManager)
{
    mDeformableBodyVisualizationManager = &deformableBodyVisualizationManager;
}

void AttachmentsVisualizationManagerDeprecated::setMode(const VisualizerMode mode)
{
    if (mMode != mode)
    {
        bool deformableResync = false;
        bool deformableRelease = false;
        if (mMode == VisualizerMode::eNone) // reparse the stage if we are activating the visualizer
        {
            if (gStage)
            {
                //not calling parseStage here, since it needs mMode updated
                handlePrimResync(gStage->GetPseudoRoot().GetPath());
                deformableResync = true;
            }
        }
        else if (mode == VisualizerMode::eNone && mUserVizAttachments.size() == 0)
        {
            // release all the buffers if we turn off
            clear(true);
            deformableRelease = true;
        }
        if (mDeformableBodyVisualizationManager)
        {
            mDeformableBodyVisualizationManager->notifyAttachmentVisualization(deformableResync, deformableRelease);
        }
    }

    mMode = mode;
    mBufferVizDirty = true;
}

bool AttachmentsVisualizationManagerDeprecated::isActive() const
{
    return mMode != VisualizerMode::eNone || mUserVizAttachments.size() > 0;
}

void AttachmentsVisualizationManagerDeprecated::setHideActor0(const bool hideActor0)
{
    mHideActor[0] = hideActor0;
    mBufferVizDirty = true;
}

void AttachmentsVisualizationManagerDeprecated::setHideActor1(const bool hideActor1)
{
    mHideActor[1] = hideActor1;
    mBufferVizDirty = true;
}

bool AttachmentsVisualizationManagerDeprecated::isTargetHidden(SdfPath attachmentPath, int32_t slot)
{
    //first check global setting
    if (mHideActor[slot])
    {
        return true;
    }

    //then check user settings
    SdfPathToUserVisibilityMap::const_iterator uit = mUserVizAttachments.find(attachmentPath);
    if (uit != mUserVizAttachments.cend())
    {
        if (uit->second.hideTarget[slot])
        {
            return true;
        }
    }

    return false;
}

/**
Returns true if ANY user attachment is configured to hide the target
*/
bool AttachmentsVisualizationManagerDeprecated::isTargetHidden(SdfPath targetPath)
{
    SdfPathToSdfPathSetMap::const_iterator tit = mTargets.find(targetPath);
    if (tit != mTargets.cend())
    {
        const SdfPathSet& targetAttachments = tit->second;
        for (SdfPath attachmentPath : targetAttachments)
        {
            auto ait = mAttachmentToInfo.find(attachmentPath);
            if (ait != mAttachmentToInfo.end())
            {
                int32_t slot = ait->second->targets[0] == targetPath ? 0 : 1;
                if (isTargetHidden(attachmentPath, slot))
                {
                    return true;
                }
            }
        }
    }
    return false;
}

void AttachmentsVisualizationManagerDeprecated::setUserAttachment(SdfPath attachmentPath, bool enable)
{
    PhysxSchemaPhysxPhysicsAttachment attachment = PhysxSchemaPhysxPhysicsAttachment::Get(gStage, attachmentPath);
    if (!attachment)
        return;

    bool deformableResync = false;
    bool deformableRelease = false;
    if (enable)
    {
        bool hadUser = mUserVizAttachments.size() > 0;
        UserVisibility userViz = {false, false};
        auto it = mUserVizAttachments.find(attachmentPath);
        if (it != mUserVizAttachments.end())
        {
            it->second = userViz;
        }
        else
        {
            mUserVizAttachments.insert({attachmentPath, userViz});
        }
        if (mMode == VisualizerMode::eNone && !hadUser)
        {
            if (gStage)
            {
                //not calling parseStage here, since it needs mMode updated
                handlePrimResync(gStage->GetPseudoRoot().GetPath());
                deformableResync = true;
            }
        }
    }
    else
    {
        mUserVizAttachments.erase(attachmentPath);
        if (mMode == VisualizerMode::eNone && mUserVizAttachments.size() == 0)
        {
            clear(true);
            deformableRelease = true;
        }
    }
    if (mDeformableBodyVisualizationManager)
    {
        mDeformableBodyVisualizationManager->notifyAttachmentVisualization(deformableResync, deformableRelease);
    }

    mBufferVizDirty = true;
}

void AttachmentsVisualizationManagerDeprecated::hideUserAttachmentActor(SdfPath attachmentPath, SdfPath actorPath, bool hide)
{
    auto it = mUserVizAttachments.find(attachmentPath);
    if (it != mUserVizAttachments.end())
    {
        AttachmentInfo* attachmentInfo = getAttachmentInfo(attachmentPath);
        if (attachmentInfo)
        {
            bool isActor0 = attachmentInfo->targets[0] == actorPath;
            bool isActor1 = attachmentInfo->targets[1] == actorPath;
            if (isActor0 || isActor1)
            {
                int32_t slot = isActor0 ? 0 : 1;
                it->second.hideTarget[slot] = hide;
            }
        }
    }
    mBufferVizDirty = true;
}

void AttachmentsVisualizationManagerDeprecated::parseStage()
{
    CARB_PROFILE_ZONE(0, "AttachmentsVisualizationManagerDeprecated::parseStage");

    if (!isActive())
        return;

    handlePrimResync(gStage->GetPseudoRoot().GetPath());
}

void AttachmentsVisualizationManagerDeprecated::update()
{
    if (!gStage || !isActive())
        return;

    gBlockNoticeHandle = true;

    CARB_PROFILE_ZONE(0, "AttachmentsVisualizationManagerDeprecated::Update");
    {
        // all updates are to session layer
        omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());

        bool notifyDeformableViz = false;
        std::vector<SdfPath> attachmentPathsToRelease;
        attachmentPathsToRelease.reserve(mBufferAttachmentPathsToRemove.size());
        std::vector<SdfPath> attachmentPathsToCreate;
        attachmentPathsToCreate.reserve(mBufferAttachmentPathsToAdd.size());

        for (SdfPath attachmentPath : mBufferAttachmentPathsToRemove)
        {
            if (removeAttachment(attachmentPath))
            {
                if (mAttachmentToInfo.count(attachmentPath))
                {
                    attachmentPathsToRelease.push_back(attachmentPath);
                }
            }
        }

        for (SdfPath attachmentPath : mBufferAttachmentPathsToAdd)
        {
            if (addAttachment(attachmentPath))
            {
                if (isComplete(attachmentPath))
                {
                    attachmentPathsToCreate.push_back(attachmentPath);
                }
            }
        }

        for (SdfPath attachmentPath : attachmentPathsToRelease)
        {
            releaseAttachment(attachmentPath);
            notifyDeformableViz = true;
            mBufferVizDirty = true;
        }

        for (SdfPath attachmentPath : attachmentPathsToCreate)
        {
            createAttachment(attachmentPath);
            notifyDeformableViz = true;
            mBufferVizDirty = true;
        }

        //always update transforms for kinematics
        for (auto targetIt : mTargetToInfo)
        {
            TargetInfo* targetInfo = targetIt.second;
            if (targetInfo && targetInfo->isKinematic)
            {
                mBufferTargetPathsToUpdateTransform.insert(targetIt.first);
            }
        }

        if (!mAttachmentToInfo.empty())
        {
            //update gap and scale
            float visualizationGap;
            float visualizationScale;
            readSettings(visualizationScale, visualizationGap);
            bool vizAttributeDirty = (mVisualizationGap != visualizationGap) || (mVisualizationScale != visualizationScale);
            mVisualizationGap = visualizationGap;
            mVisualizationScale = visualizationScale;

            if (mBufferVizDirty)
            {
                mVizAttachments.clear();
                mVizSelected.clear();

                // update selection if viz dirty
                const auto usdContext = omni::usd::UsdContext::getContext();
                const std::vector<SdfPath> selectedPaths = usdContext->getSelection()->getSelectedPrimPathsV2();

                for (SdfPath selectedPath : selectedPaths)
                {
                    // use awesome path table to quickly find all candidate attachment paths to update:
                    const auto iteratorPair = mAttachmentTable.FindSubtreeRange(selectedPath);
                    for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
                    {
                        // it->first is a path in the subtree starting at path s.
                        // so if it is an active deformable, add it to  update buffer
                        if (mAttachmentToInfo.count(it->first))
                            mVizSelected.insert(it->first);
                    }
                }

                if (mMode == VisualizerMode::eSelected)
                {
                    mVizAttachments = mVizSelected;
                }
                else if (mMode == VisualizerMode::eAll)
                {
                    for (SdfPathToAttachmentInfoMap::const_iterator it = mAttachmentToInfo.cbegin(); it != mAttachmentToInfo.cend(); ++it)
                    {
                        mVizAttachments.insert(it->first);
                    }
                }

                // add user requested attachments
                for (SdfPathToUserVisibilityMap::const_iterator it = mUserVizAttachments.cbegin(); it != mUserVizAttachments.cend(); ++it)
                {
                    if (mAttachmentToInfo.count(it->first))
                    {
                        mVizAttachments.insert(it->first);
                    }
                }
            }

            // create new session attachment prim if needed:
            for (SdfPath attachmentPath : attachmentPathsToCreate)
            {
                createSessionAttachment(attachmentPath);
                createSessionAttachmentGeometries(attachmentPath);
                mBufferVizDirty = true;
            }

            if (vizAttributeDirty)
            {
                mBufferAttachmentPathsToUpdateVizAttribute = mVizAttachments;
            }

            // Do all attribute updates and prim deletions in Sdf changeblock:
            {
                SdfChangeBlock block;
                // post creation rendering initialization:
                for (SdfPath attachmentPath : attachmentPathsToCreate)
                {
                    setupSessionAttachmentGeometries(attachmentPath);
                }

                if (mBufferVizDirty)
                {
                    // update visibilities of session attachments
                    for (SdfPathToAttachmentInfoMap::const_iterator it = mAttachmentToInfo.cbegin(); it != mAttachmentToInfo.cend(); ++it)
                    {
                        updateAttachmentVisibility(it->first, isVisible(it->first));
                        notifyDeformableViz = true;
                    }
                }

                // update xforms and verts:
                mXformCache.Clear();
                UsdTimeCode time = UsdTimeCode(gTimeline->getCurrentTime() * gStage->GetTimeCodesPerSecond());
                mXformCache.SetTime(time);

                for (SdfPath targetPath : mBufferTargetPathsToUpdateTransform)
                {
                    updateTargetTransform(targetPath);
                }

                for (SdfPath targetPath : mBufferTargetPathsToUpdateGeometry)
                {
                    updateTargetGeometry(targetPath);
                }

                for (SdfPath attachmentPath : mBufferAttachmentPathsToUpdateGeometry)
                {
                    updateAttachmentGeometry(attachmentPath);
                    notifyDeformableViz = true;
                }

                for (SdfPath attachmentPath : mBufferAttachmentPathsToUpdateVizAttribute)
                {
                    updateVisualizationScale(attachmentPath);
                }
            }
        }

        if (mBufferVizDirty)
        {
            for (auto it = mTargetToInfo.begin(); it != mTargetToInfo.end(); ++it)
            {
                SdfPath targetPath = it->first;
                TargetInfo* targetInfo = it->second;
                if (targetInfo && targetInfo->type == TargetInfo::Type::eRigid)
                {
                    bool hide = isTargetHidden(it->first);
                    updateRigidSkinVisibility(targetPath, hide);
                }
            }
        }

        if (mDeformableBodyVisualizationManager && notifyDeformableViz)
        {
            mDeformableBodyVisualizationManager->notifyAttachmentVisualization(false, false);
        }
    }

    clearBuffers();

    gBlockNoticeHandle = false;
}

void AttachmentsVisualizationManagerDeprecated::selectionChanged()
{
    const auto usdContext = omni::usd::UsdContext::getContext();
    const std::vector<SdfPath> selectedPaths = usdContext->getSelection()->getSelectedPrimPathsV2();

    bool needToRedirect = false;
    std::vector<SdfPath> primPaths;
    primPaths.reserve(selectedPaths.size());
    for(const SdfPath path : selectedPaths)
    {
        SdfPathToSdfPathMap::const_iterator it = mSessionGeomToAttachment.find(path);
        if(it != mSessionGeomToAttachment.cend())
        {
            primPaths.push_back(it->second);
            needToRedirect = true;
        }
        else
        {
            primPaths.push_back(path);
        }
    }

    if(needToRedirect)
    {
        usdContext->getSelection()->setSelectedPrimPathsV2(primPaths);
    }

    mBufferVizDirty = true;
}

void AttachmentsVisualizationManagerDeprecated::clear(bool updateSkin)
{
    if (gStage)
    {
        omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());

        for (auto it = mTargetToInfo.cbegin(); it != mTargetToInfo.cend(); ++it)
        {
            if (updateSkin)
            {
                // updateRigidSkinVisibility causes flatcache crash on stage close
                updateRigidSkinVisibility(it->first, false);
            }
        }

        for (auto it = mAttachmentToInfo.cbegin(); it != mAttachmentToInfo.cend(); ++it)
        {
            SdfPath sessionAttachmentPath = mAttachmentToSession[it->first];
            gStage->RemovePrim(sessionAttachmentPath);
        }
    }
    mAttachments.clear();
    mAttachmentTable.clear();
    for (SdfPathToAttachmentInfoMap::const_iterator it = mAttachmentToInfo.cbegin(); it != mAttachmentToInfo.cend(); ++it)
    {
        if (it->second)
        {
            delete it->second;
        }
    }
    mAttachmentToInfo.clear();
    mAttachmentToSession.clear();
    mSessionGeomToAttachment.clear();
    mTargets.clear();
    mTargetTable.clear();
    for (SdfPathToTargetInfoMap::const_iterator it = mTargetToInfo.cbegin(); it != mTargetToInfo.cend(); ++it)
    {
        releaseTargetInfo(it->second);
    }
    mTargetToInfo.clear();
    mSessionAttachmentRoot = SdfPath();
    mVizAttachments.clear();
    mVizSelected.clear();

    clearBuffers();
}

void AttachmentsVisualizationManagerDeprecated::release()
{
    clear(false);
}

SdfPath AttachmentsVisualizationManagerDeprecated::createSessionAttachment(const SdfPath attachmentPath)
{
    const SdfPath sessionAttachmentPath = mAttachmentToSession[attachmentPath];
    UsdGeomScope sessionAttachmentScope = UsdGeomScope::Get(gStage, sessionAttachmentPath);
    if (!sessionAttachmentScope)
    {
        sessionAttachmentScope = UsdGeomScope::Define(gStage, sessionAttachmentPath);
        omni::kit::EditorUsd::setNoDelete(sessionAttachmentScope.GetPrim(), true);
        omni::kit::EditorUsd::setHideInStageWindow(sessionAttachmentScope.GetPrim(), true);
    }

    for (int slot = 0; slot < 2; ++slot)
    {
        const SdfPath targetAttachmentPath = getSessionTargetPath(sessionAttachmentPath, slot);
        UsdGeomXform targetAttachmentXform = UsdGeomXform::Get(gStage, targetAttachmentPath);
        if (!targetAttachmentXform)
        {
            targetAttachmentXform = UsdGeomXform::Define(gStage, targetAttachmentPath);
            omni::kit::EditorUsd::setNoDelete(targetAttachmentXform.GetPrim(), true);
            omni::kit::EditorUsd::setHideInStageWindow(targetAttachmentXform.GetPrim(), true);

            UsdGeomXformOp opTransform;
            opTransform = targetAttachmentXform.AddTransformOp();
            opTransform.Set(GfMatrix4d(1.0)); // workaround for hydra update not working if op value is set in later update.
        }
    }

    return sessionAttachmentPath;
}

void AttachmentsVisualizationManagerDeprecated::createSessionPointInstancer(const SdfPath pointInstancerPath)
{
    SdfPath pointPrototypePath = getSessionPointInstancerPrototypePath(pointInstancerPath);

    UsdGeomSphere samplePointPrototype = UsdGeomSphere::Get(gStage, pointPrototypePath);
    if (!samplePointPrototype)
    {
        samplePointPrototype = UsdGeomSphere::Define(gStage, pointPrototypePath);
        omni::kit::EditorUsd::setNoDelete(samplePointPrototype.GetPrim(), true);
        omni::kit::EditorUsd::setHideInStageWindow(samplePointPrototype.GetPrim(), true);
        omni::kit::EditorUsd::setNoSelectionOutline(samplePointPrototype.GetPrim(), true);
        samplePointPrototype.AddScaleOp().Set(GfVec3f(1.0f));
    }

    UsdGeomPointInstancer pointInstancer = UsdGeomPointInstancer::Get(gStage, pointInstancerPath);
    if (!pointInstancer)
    {
        pointInstancer = UsdGeomPointInstancer::Define(gStage, pointInstancerPath);
        omni::kit::EditorUsd::setNoDelete(pointInstancer.GetPrim(), true);
        omni::kit::EditorUsd::setHideInStageWindow(pointInstancer.GetPrim(), true);
        omni::kit::EditorUsd::setNoSelectionOutline(pointInstancer.GetPrim(), true);
    }
}

void AttachmentsVisualizationManagerDeprecated::createSessionAttachmentGeometries(const SdfPath attachmentPath)
{
    const AttachmentInfo& attachmentInfo = *mAttachmentToInfo[attachmentPath];

    SdfPath sessionAttachmentPath = mAttachmentToSession[attachmentPath];

    for (uint32_t slot = 0; slot < 2; ++slot)
    {
        SdfPath samplePointInstancerPath = getSessionSamplePointInstancerPath(sessionAttachmentPath, slot);
        SdfPath filterPointInstancerPath = getSessionFilterPointInstancerPath(sessionAttachmentPath, slot);

        createSessionPointInstancer(samplePointInstancerPath);
        mSessionGeomToAttachment.insert({ samplePointInstancerPath, attachmentPath });
        mSessionGeomToAttachment.insert({ getSessionPointInstancerPrototypePath(samplePointInstancerPath), attachmentPath });

        SdfPath targetPath = attachmentInfo.targets[slot];
        SdfPathToTargetInfoMap::const_iterator it = mTargetToInfo.find(targetPath);
        if (it != mTargetToInfo.end())
        {
            const TargetInfo& targetInfo = *it->second;

            if (targetInfo.type == TargetInfo::eDeformableBody || targetInfo.type == TargetInfo::eDeformableSurface || targetInfo.type == TargetInfo::eParticleCloth)
            {
                createSessionPointInstancer(filterPointInstancerPath);
                mSessionGeomToAttachment.insert({ filterPointInstancerPath, attachmentPath });
                mSessionGeomToAttachment.insert({ getSessionPointInstancerPrototypePath(filterPointInstancerPath), attachmentPath });
            }
        }
    }
}

void AttachmentsVisualizationManagerDeprecated::setupSessionPointInstancer(const SdfPath pointInstancerPath, const float radius, const GfVec3f& color)
{
    SdfPath pointPrototypePath = getSessionPointInstancerPrototypePath(pointInstancerPath);
    UsdGeomSphere pointPrototype = UsdGeomSphere::Get(gStage, pointPrototypePath);
    pointPrototype.GetRadiusAttr().Set(kPrototypeRadiusZero);
    pointPrototype.GetDisplayColorAttr().Set(VtArray<GfVec3f>(1, color));

    UsdGeomPointInstancer pointInstancer = UsdGeomPointInstancer::Get(gStage, pointInstancerPath);
    pointInstancer.GetPrototypesRel().AddTarget(pointPrototypePath);

    fillinDummyPoint(pointInstancer);  
}

void AttachmentsVisualizationManagerDeprecated::resetSessionPointInstancer(const SdfPath pointInstancerPath)
{
    UsdGeomPointInstancer pointInstancer = UsdGeomPointInstancer::Get(gStage, pointInstancerPath);
    if (!pointInstancer)
        return;

    SdfPath pointPrototypePath = getSessionPointInstancerPrototypePath(pointInstancerPath);
    UsdGeomSphere pointPrototype = UsdGeomSphere::Get(gStage, pointPrototypePath);
    if (!pointPrototype)
        return;

    pointPrototype.GetRadiusAttr().Set(kPrototypeRadiusZero);
    fillinDummyPoint(pointInstancer);
}

void AttachmentsVisualizationManagerDeprecated::updateSessionPointInstancerRadius(const SdfPath pointInstancerPath, const float radius)
{
    SdfPath pointPrototypePath = getSessionPointInstancerPrototypePath(pointInstancerPath);
    UsdGeomSphere pointPrototype = UsdGeomSphere::Get(gStage, pointPrototypePath);
    if (pointPrototype)
    {
        pointPrototype.GetRadiusAttr().Set(double(radius));
    }

    UsdGeomPointInstancer pointInstancer = UsdGeomPointInstancer::Get(gStage, pointInstancerPath);
    if (pointInstancer && radius == kPrototypeRadiusZero)
    {
        fillinDummyPoint(pointInstancer);
    }
}

void AttachmentsVisualizationManagerDeprecated::setupSessionAttachmentGeometries(const SdfPath attachmentPath)
{
    SdfPath sessionAttachmentPath = mAttachmentToSession[attachmentPath];
    PhysxSchemaPhysxPhysicsAttachment attachment = PhysxSchemaPhysxPhysicsAttachment::Get(gStage, attachmentPath);
    AttachmentInfo& attachmentInfo = *mAttachmentToInfo[attachmentPath];

    float pointScale = getPointScale(attachmentPath);

    for (uint32_t slot = 0; slot < 2; ++slot)
    {
        SdfPath targetPath = attachmentInfo.targets[slot];

        SdfPath samplePointInstancerPath = getSessionSamplePointInstancerPath(sessionAttachmentPath, slot);
        SdfPath filterPointInstancerPath = getSessionFilterPointInstancerPath(sessionAttachmentPath, slot);

        setupSessionPointInstancer(samplePointInstancerPath, kRadiusSample[slot]*pointScale, kColorSample[slot]);
        attachmentInfo.numPointSamples[slot] = 0;

        SdfPathToTargetInfoMap::const_iterator it = mTargetToInfo.find(targetPath);
        if (it != mTargetToInfo.end())
        {
            const TargetInfo& targetInfo = *it->second;
            if (targetInfo.type == TargetInfo::eDeformableBody || targetInfo.type == TargetInfo::eDeformableSurface || targetInfo.type == TargetInfo::eParticleCloth)
            {
                setupSessionPointInstancer(filterPointInstancerPath, kRadiusFilter[slot]*pointScale, kColorFilter[slot]);
                attachmentInfo.numPointFilters[slot] = 0;
            }
        }
    }
}

SdfPath AttachmentsVisualizationManagerDeprecated::getSessionAttachmentPath(const SdfPath attachmentPath)
{
    // session layer root is only created if necessary
    if(mSessionAttachmentRoot.IsEmpty())
    {
        // TODO preist: adapt root based on prim existence?
        mSessionAttachmentRoot = SdfPath("/PhysxAttachmentVisualization");
        CARB_ASSERT(!gStage->GetPrimAtPath(mSessionAttachmentRoot));
        UsdGeomScope rootScope = UsdGeomScope::Define(gStage, mSessionAttachmentRoot);
        CARB_ASSERT(rootScope);
        omni::kit::EditorUsd::setNoDelete(rootScope.GetPrim(), true);
        omni::kit::EditorUsd::setHideInStageWindow(rootScope.GetPrim(), true);
    }

    // create the path:
    std::string nameString = attachmentPath.GetString();
    std::replace(nameString.begin(), nameString.end(), '/', '_');
    SdfPath sessionPath = mSessionAttachmentRoot.AppendElementString(nameString);
    return sessionPath;
}

void AttachmentsVisualizationManagerDeprecated::handlePrimResync(const SdfPath path)
{
    UsdPrimRange range(gStage->GetPrimAtPath(path));
    for(UsdPrimRange::const_iterator it = range.cbegin(); it != range.cend(); ++it)
    {
        const UsdPrim& prim = *it;
        if (!prim)
            continue;

        if(prim.IsA<PhysxSchemaPhysxPhysicsAttachment>())
        {
            mBufferAttachmentPathsToAdd.insert(prim.GetPath());
        }
        else if (prim.HasAPI<UsdPhysicsRigidBodyAPI>() || prim.HasAPI<UsdPhysicsCollisionAPI>() ||
                 prim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>() || prim.HasAPI<PhysxSchemaPhysxDeformableSurfaceAPI>() || prim.HasAPI<PhysxSchemaPhysxParticleClothAPI>())
        {
            //stale targets might have become valid again
            SdfPathToSdfPathSetMap::const_iterator tit = mTargets.find(path);
            if (tit != mTargets.cend())
            {
                const SdfPathSet& targetAttachments = tit->second;
                for (SdfPath targetAttachment : targetAttachments)
                {
                    if (!mAttachmentToInfo.count(targetAttachment))
                    {
                        mBufferAttachmentPathsToRemove.insert(targetAttachment);
                        mBufferAttachmentPathsToAdd.insert(targetAttachment);
                    }
                }
                // could be a tranform related attribute removal
                mBufferTargetPathsToUpdateTransform.insert(path);
            }
        }
        else if (mTargetToInfo.count(path))
        {
            //target api got removed
            const SdfPathSet& targetAttachments = mTargets[path];
            for (SdfPath targetAttachment : targetAttachments)
            {
                if (mAttachmentToInfo.count(targetAttachment))
                {
                    mBufferAttachmentPathsToRemove.insert(targetAttachment);
                    mBufferAttachmentPathsToAdd.insert(targetAttachment);
                }
            }
        }
    }
}

void AttachmentsVisualizationManagerDeprecated::handlePrimRemove(const SdfPath path)
{
    {
        const auto iteratorPair = mAttachmentTable.FindSubtreeRange(path);
        for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
        {
            // it->first is a path in the subtree starting at path.
            if (mAttachments.count(it->first))
            {
                mBufferAttachmentPathsToRemove.insert(it->first);
            }
        }
    }

    {
        const auto iteratorPair = mTargetTable.FindSubtreeRange(path);
        for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
        {
            // it->first is a path in the subtree starting at path.
            SdfPathToSdfPathSetMap::const_iterator tit = mTargets.find(it->first);
            if (tit != mTargets.cend())
            {
                const SdfPathSet& targetAttachments = tit->second;
                for (SdfPath targetAttachment : targetAttachments)
                {
                    mBufferAttachmentPathsToRemove.insert(targetAttachment);
                    mBufferAttachmentPathsToAdd.insert(targetAttachment);
                }
            }
        }
    }
}

void AttachmentsVisualizationManagerDeprecated::handleAttributeChange(const SdfPath path, const TfToken attributeName, const bool isXform)
{
    // xform must always be handled because the change may have happened in the xform hierarchy above the target:
    if(isXform || attributeName == UsdPhysicsTokens->physicsKinematicEnabled)
    {
        const auto iteratorPair = mTargetTable.FindSubtreeRange(path);
        for(auto it = iteratorPair.first; it != iteratorPair.second; it++)
        {
            const SdfPath targetPath = it->first;
            // it->first is a path in the subtree starting at path.
            // so if it is a target, add the it to the xform update buffer
            SdfPathToSdfPathSetMap::const_iterator tit = mTargets.find(targetPath);
            if (tit != mTargets.cend())
            {
                if (isXform)
                {
                    mBufferTargetPathsToUpdateTransform.insert(targetPath);
                    const SdfPathSet& targetAttachments = tit->second;
                    for (SdfPath attachmentPath : targetAttachments)
                    {
                        mBufferAttachmentPathsToUpdateVizAttribute.insert(attachmentPath);
                    }
                }
                else
                {
                    auto itMap = mTargetToInfo.find(targetPath);
                    if (itMap != mTargetToInfo.end() && itMap->second)
                    {
                        UsdPhysicsRigidBodyAPI rigidBody = UsdPhysicsRigidBodyAPI::Get(gStage, path);
                        if (rigidBody)
                        {
                            rigidBody.GetKinematicEnabledAttr().Get(&itMap->second->isKinematic);
                        }
                    }
                }
            }
        }
        return;
    }

    // same for visibility
    if (attributeName == UsdGeomTokens.Get()->visibility)
    {
        // use awesome path table to quickly find all candidate particles paths to update:
        const auto iteratorPair = mAttachmentTable.FindSubtreeRange(path);
        for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
        {
            // it->first is a path in the subtree starting at path.
            if (mAttachments.count(it->first))
            {
                mBufferVizDirty = true;
            }
        }
        return;
    }

    if (mTargetToInfo.count(path))
    {
        bool targetDeformed = false;
        UsdPrim prim = gStage->GetPrimAtPath(path);

        if (attributeName == deformableBodyTetMeshCrcToken || attributeName == physxDeformableSurfaceInputCrcToken || attributeName == physxParticleClothInputCrcToken)
        {
            const SdfPathSet& targetAttachments = mTargets[path];
            for (SdfPath targetAttachment : targetAttachments)
            {
                if (mAttachmentToInfo.count(targetAttachment))
                {
                    mBufferAttachmentPathsToRemove.insert(targetAttachment);
                    mBufferAttachmentPathsToAdd.insert(targetAttachment);
                }
            }
        }
        else if (attributeName == PhysxSchemaTokens->physxDeformableCollisionPoints)
        {
            targetDeformed = prim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>();
        }
        else if (attributeName == UsdGeomTokens.Get()->points)
        {
            targetDeformed = prim.HasAPI<PhysxSchemaPhysxParticleClothAPI>() || prim.HasAPI<PhysxSchemaPhysxDeformableSurfaceAPI>();
        }

        if (targetDeformed)
        {
            mBufferTargetPathsToUpdateGeometry.insert(path);
        }
    }
    else if (mAttachments.count(path))
    {
        // check attachment data changes:
        if (attributeName == PhysxSchemaTokens->points0 || attributeName == PhysxSchemaTokens->points1 ||
            attributeName == PhysxSchemaTokens->collisionFilterIndices0 || attributeName == PhysxSchemaTokens->collisionFilterIndices1)
        {
            if (mAttachmentToInfo.count(path))
            {
                mBufferAttachmentPathsToUpdateGeometry.insert(path);
            }
        }
        else if (attributeName == PhysxSchemaTokens->actor0 || attributeName == PhysxSchemaTokens->actor1)
        {
            mBufferAttachmentPathsToRemove.insert(path);
            mBufferAttachmentPathsToAdd.insert(path);
        }
    }
}

bool AttachmentsVisualizationManagerDeprecated::addAttachment(const SdfPath attachmentPath)
{
    PhysxSchemaPhysxPhysicsAttachment attachment = PhysxSchemaPhysxPhysicsAttachment::Get(gStage, attachmentPath);
    bool adding = attachment && !mAttachments.count(attachmentPath);
    if (adding)
    {
        TargetPathPair targetPaths = getAttachmentTargets(attachment);
        mAttachments.insert({attachmentPath, targetPaths});
        mAttachmentTable.insert({attachmentPath, 42}); // we only care about the keys, so set 42 as dummy.
        for (uint32_t slot = 0; slot < 2; ++slot)
        {
            SdfPath targetPath = targetPaths[slot];
            if (!targetPath.IsEmpty())
            {
                SdfPathToSdfPathSetMap::iterator tit = mTargets.find(targetPath);
                if (tit != mTargets.end())
                {
                    SdfPathSet& targetAttachments = tit->second;
                    targetAttachments.insert(attachmentPath);
                }
                else
                {
                    SdfPathSet targetAttachments;
                    targetAttachments.insert(attachmentPath);
                    mTargets.insert({targetPath, targetAttachments});
                    mTargetTable.insert({targetPath, 42});
                }
            }
        }
    }
    return adding;
}

bool AttachmentsVisualizationManagerDeprecated::removeAttachment(const SdfPath attachmentPath)
{
    auto it = mAttachments.find(attachmentPath);
    bool removing = it != mAttachments.end();
    if (removing)
    {
        TargetPathPair targetPathPair = it->second;
        for (uint32_t slot = 0; slot < 2; ++slot)
        {
            SdfPath targetPath = targetPathPair[slot];
            SdfPathToSdfPathSetMap::iterator tit = mTargets.find(targetPath);
            if (tit != mTargets.end())
            {
                SdfPathSet& targetAttachments = tit->second;
                if (targetAttachments.size() == 1)
                {
                    mTargets.erase(targetPath);
                    mTargetTable.erase(targetPath);
                }
                else
                {
                    targetAttachments.erase(attachmentPath);
                }
            }
        }

        mAttachments.erase(attachmentPath);
        mAttachmentTable.erase(attachmentPath);
    }
    return removing;
}

void AttachmentsVisualizationManagerDeprecated::createAttachment(const SdfPath attachmentPath)
{
    UsdPrim attachmentPrim = gStage->GetPrimAtPath(attachmentPath);
    PhysxSchemaPhysxPhysicsAttachment attachment(attachmentPrim);
    if (!attachment)
        return;

    mAttachmentToSession[attachmentPath] = getSessionAttachmentPath(attachmentPath);

    // attachment info and targets
    AttachmentInfo* attachmentInfo = new AttachmentInfo();
    TargetPathPair targetPathPair = getAttachmentTargets(attachment);

    for (uint32_t slot = 0; slot < 2; ++slot)
    {
        SdfPath targetPath = targetPathPair[slot];
        if (!targetPath.IsEmpty())
        {
            SdfPathToTargetInfoMap::iterator tit = mTargetToInfo.find(targetPath);
            if (tit == mTargetToInfo.end())
            {
                //new target
                TargetInfo* info = createTargetInfo(targetPath);
                if (info)
                {
                    info->numAttachments = 1;
                    mTargetToInfo.insert({targetPath, info});
                }
            }
            else
            {
                tit->second->numAttachments++;
            }
        }

        attachmentInfo->targets[slot] = targetPath;
        attachmentInfo->numPointSamples[slot] = 0;
        attachmentInfo->numPointFilters[slot] = 0;
    }

    mAttachmentToInfo.insert({attachmentPath, attachmentInfo});
}

void AttachmentsVisualizationManagerDeprecated::releaseAttachment(const SdfPath attachmentPath)
{
    SdfPathToAttachmentInfoMap::const_iterator ait = mAttachmentToInfo.find(attachmentPath);
    if (ait == mAttachmentToInfo.cend())
    {
        return;
    }
    AttachmentInfo* attachmentInfo = ait->second;

    for (uint32_t slot = 0; slot < 2; ++slot)
    {
        SdfPath targetPath = attachmentInfo->targets[slot];
        SdfPathToTargetInfoMap::const_iterator tit = mTargetToInfo.find(targetPath);
        if (tit != mTargetToInfo.cend())
        {
            TargetInfo* targetInfo = tit->second;
            if (targetInfo->numAttachments == 1)
            {
                releaseTargetInfo(tit->second);
                mTargetToInfo.erase(tit);
            }
            else
            {
                targetInfo->numAttachments--;
            }
        }
    }

    mAttachmentToInfo.erase(ait);
    delete attachmentInfo;

    // remove session layer representation:
    const SdfPathToSdfPathMap::const_iterator it = mAttachmentToSession.find(attachmentPath);
    if(it != mAttachmentToSession.cend())
    {
        SdfPath sessionAttachmentPath = it->second;
        for (uint32_t slot = 0; slot < 2; ++slot)
        {
            SdfPath samplePointInstancerPath = getSessionSamplePointInstancerPath(sessionAttachmentPath, slot);
            SdfPath filterPointInstancerPath = getSessionFilterPointInstancerPath(sessionAttachmentPath, slot);
            mSessionGeomToAttachment.erase(samplePointInstancerPath);
            mSessionGeomToAttachment.erase(getSessionPointInstancerPrototypePath(samplePointInstancerPath));
            mSessionGeomToAttachment.erase(filterPointInstancerPath);
            mSessionGeomToAttachment.erase(getSessionPointInstancerPrototypePath(filterPointInstancerPath));
        }

        if(!sessionAttachmentPath.IsEmpty())
        {
            gStage->RemovePrim(sessionAttachmentPath);
        }
        mAttachmentToSession.erase(it);
    }

    // clear viz state, as it won't get update if no attachments left. TODO, refactor update() flow.
    mVizAttachments.erase(attachmentPath);
    mVizSelected.erase(attachmentPath);
}

AttachmentsVisualizationManagerDeprecated::AttachmentInfo* AttachmentsVisualizationManagerDeprecated::getAttachmentInfo(SdfPath attachmentPath)
{
    auto it = mAttachmentToInfo.find(attachmentPath);
    if (it != mAttachmentToInfo.end())
    {
        return it->second;
    }
    return nullptr;
}

float AttachmentsVisualizationManagerDeprecated::getPointScale(const UsdPrim& targetMeshPrim, const VtArray<GfVec3f>& restPoints) const
{
    //compute area for scale heuristic, use original mesh and rest positions. TODO, change the desc data
    float scale = 0.0f;
    UsdGeomMesh mesh(targetMeshPrim);
    if (mesh && restPoints.size() > 0)
    {
        float scaleArea = 0.0f;
        VtArray<int32_t> faceVertexCounts;
        VtArray<int32_t> faceVertexIndices;
        mesh.GetFaceVertexCountsAttr().Get(&faceVertexCounts);
        mesh.GetFaceVertexIndicesAttr().Get(&faceVertexIndices);
        int32_t offset = 0;
        int32_t numPoints = int32_t(restPoints.size());
        int32_t numIndices = int32_t(faceVertexIndices.size());
        for (size_t f = 0; f < faceVertexCounts.size(); ++f)
        {
            GfVec3f triAreaVec(0.0f);
            const int32_t count = faceVertexCounts[f];
            if (count > 2 && offset + count - 1 < numIndices)
            {
                int32_t i0 = faceVertexIndices[offset];
                if (i0 < numPoints)
                {
                    const GfVec3f& p0 = restPoints[i0];
                    for (int32_t v = 1; v < count - 1; ++v)
                    {
                        int32_t i1 = faceVertexIndices[offset + v];
                        int32_t i2 = faceVertexIndices[offset + v + 1];
                        if (i1 < numPoints && i2 < numPoints)
                        {
                            const GfVec3f& p1 = restPoints[i1];
                            const GfVec3f& p2 = restPoints[i2];
                            triAreaVec += GfCross(p1 - p0, p2 - p0);
                        }
                    }
                }

                offset += count;
                scaleArea += triAreaVec.Normalize() / 2.0f;
            }
        }

        scale = std::sqrt(scaleArea / restPoints.size()) * kEdgeToPointScale;
    }
    return scale;
}

AttachmentsVisualizationManagerDeprecated::TargetInfo* AttachmentsVisualizationManagerDeprecated::createTargetInfo(const SdfPath targetPath)
{
    TargetInfo* targetInfo = new TargetInfo();
    UsdPrim targetPrim = gStage->GetPrimAtPath(targetPath);
    if (targetPrim)
    {
        const uint64_t stageId = UsdUtilsStageCache::Get().GetId(gStage).ToLongInt();
        if (targetPrim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>())
        {
            PhysxSchemaPhysxDeformableBodyAPI(targetPrim).GetCollisionRestPointsAttr().Get(&targetInfo->restPoints);
            PhysxSchemaPhysxDeformableBodyAPI(targetPrim).GetCollisionIndicesAttr().Get(&targetInfo->indices);

            //compute volume for scale heuristic
            float scaleVolume = 0.0f;
            bool valid = targetInfo->restPoints.size() > 0 && targetInfo->indices.size() > 0;
            int32_t numPoints = int32_t(targetInfo->indices.size());
            for (size_t t = 0; t < targetInfo->indices.size()/4; ++t)
            {
                int32_t i0 = targetInfo->indices[t * 4 + 0];
                int32_t i1 = targetInfo->indices[t * 4 + 1];
                int32_t i2 = targetInfo->indices[t * 4 + 2];
                int32_t i3 = targetInfo->indices[t * 4 + 3];
                valid &= (i0 < numPoints && i1 < numPoints && i2 < numPoints && i3 < numPoints);
                if (valid)
                {
                    const GfVec3f& p0 = targetInfo->restPoints[i0];
                    const GfVec3f& p1 = targetInfo->restPoints[i1];
                    const GfVec3f& p2 = targetInfo->restPoints[i2];
                    const GfVec3f& p3 = targetInfo->restPoints[i3];
                    float tetVolume = std::abs((p1 - p0) * GfCross(p2 - p0, p3 - p0)) / 6.0f;
                    scaleVolume += tetVolume;
                }
            }

            if (valid)
            {
                targetInfo->pointScale = std::pow(scaleVolume / targetInfo->restPoints.size(), 1.0f/3.0f) * kEdgeToPointScale;

                targetInfo->tetFinder = gPhysXAttachmentPrivate->createTetFinder(
                    (carb::Float3*)targetInfo->restPoints.data(), uint32_t(targetInfo->restPoints.size()),
                    (uint32_t*)targetInfo->indices.data(), uint32_t(targetInfo->indices.size()));

                if (targetInfo->tetFinder)
                {
                    targetInfo->type = TargetInfo::eDeformableBody;
                }
            }
        }
        else if (targetPrim.HasAPI<PhysxSchemaPhysxDeformableSurfaceAPI>())
        {
            const PhysxSchemaPhysxDeformableAPI deformableSurfaceTarget(targetPrim);
            deformableSurfaceTarget.GetRestPointsAttr().Get(&targetInfo->restPoints);
            deformableSurfaceTarget.GetSimulationIndicesAttr().Get(&targetInfo->indices);
            bool valid = targetInfo->restPoints.size() > 0 && targetInfo->indices.size() > 0;
            if (valid)
            {
                targetInfo->pointScale = getPointScale(targetPrim, targetInfo->restPoints);

                targetInfo->pointFinder = gPhysXAttachmentPrivate->createPointFinder(
                    (carb::Float3*)targetInfo->restPoints.data(), uint32_t(targetInfo->restPoints.size()));

                if (targetInfo->pointFinder)
                {
                    targetInfo->type = TargetInfo::eDeformableSurface;
                }
            }
        }
        else if (targetPrim.HasAPI<PhysxSchemaPhysxParticleClothAPI>())
        {
            //make sure to parse cloth in order to have (welded) restPoints available
            ParticleClothDesc* clothDesc = gUsdLoad->parseParticleCloth(stageId, targetPath);
            if (clothDesc)
            {
                if (clothDesc->restPoints.size() > 0)
                {
                    targetInfo->restPoints.resize(clothDesc->restPoints.size());
                    std::memcpy(targetInfo->restPoints.data(), clothDesc->restPoints.data(),
                                sizeof(GfVec3f) * targetInfo->restPoints.size());

                    if (clothDesc->isWelded)
                    {
                        // Get welded points through remap table
                        targetInfo->clothPointRemap.assign(
                            clothDesc->verticesRemapToOrig.begin(), clothDesc->verticesRemapToOrig.end());
                    }

                    const PhysxSchemaPhysxParticleClothAPI particleClothTarget(targetPrim);
                    VtArray<GfVec3f> restPoints; // original rest points, not welded rest points!
                    particleClothTarget.GetRestPointsAttr().Get(&restPoints);
                    targetInfo->pointScale = getPointScale(targetPrim, restPoints);

                    targetInfo->pointFinder = gPhysXAttachmentPrivate->createPointFinder(
                        (carb::Float3*)targetInfo->restPoints.data(), uint32_t(targetInfo->restPoints.size()));

                    if (targetInfo->pointFinder)
                    {
                        targetInfo->type = TargetInfo::eParticleCloth;
                    }
                }
                gUsdLoad->releaseDesc(clothDesc);
            }
        }
        else if (targetPrim.HasAPI<UsdPhysicsCollisionAPI>())
        {
            targetInfo->isKinematic = false;
            UsdPrim prim = targetPrim;
            bool found = false;
            while (prim && !found)
            {
                if (prim.HasAPI<UsdPhysicsRigidBodyAPI>())
                {
                    UsdPhysicsRigidBodyAPI(prim).GetKinematicEnabledAttr().Get(&targetInfo->isKinematic);
                    found = true;
                }
                prim = prim.GetParent();
            }
            if (found)
            {
                targetInfo->type = TargetInfo::eRigid;
            }
        }
        else if (targetPrim.HasAPI<UsdPhysicsRigidBodyAPI>())
        {
            targetInfo->type = TargetInfo::eRigid;
            UsdPhysicsRigidBodyAPI(targetPrim).GetKinematicEnabledAttr().Get(&targetInfo->isKinematic);
        }
    }
    if (targetInfo->type == TargetInfo::eNone)
    {
        delete targetInfo;
        return nullptr;
    }
    return targetInfo;
}

void AttachmentsVisualizationManagerDeprecated::releaseTargetInfo(TargetInfo* targetInfo)
{
    if (targetInfo)
    {
        if (targetInfo->tetFinder)
        {
            gPhysXAttachmentPrivate->releaseTetFinder(targetInfo->tetFinder);
        }
        if (targetInfo->pointFinder)
        {
            gPhysXAttachmentPrivate->releasePointFinder(targetInfo->pointFinder);
        }
        delete targetInfo;
    }
}

AttachmentsVisualizationManagerDeprecated::TargetInfo* AttachmentsVisualizationManagerDeprecated::getTargetInfo(SdfPath targetPath)
{
    auto it = mTargetToInfo.find(targetPath);
    if (it != mTargetToInfo.end())
    {
        return it->second;
    }
    return nullptr;
}

void AttachmentsVisualizationManagerDeprecated::updateAttachmentVisibility(const SdfPath attachmentPath, const bool isVisible)
{
    // if it does not exist, no need to update visibility
    SdfPath sessionAttachmentPath = mAttachmentToSession[attachmentPath];
    if(sessionAttachmentPath.IsEmpty())
        return;

    UsdGeomScope attachmentScope = UsdGeomScope::Get(gStage, sessionAttachmentPath);

    if(!attachmentScope)
        return;

    AttachmentInfo& attachmentInfo = *mAttachmentToInfo[attachmentPath];

    //consider user visibility
    bool userVisible = true;
    {
        UsdGeomImageable imgParent;
        UsdPrim parent = gStage->GetPrimAtPath(attachmentPath);
        while (parent.IsValid() && !bool(imgParent))
        {
            imgParent = UsdGeomImageable(parent);
            parent = parent.GetParent();
        }

        if (imgParent)
        {
            userVisible = imgParent.ComputeVisibility() == UsdGeomTokens.Get()->inherited;
        }
    }

    if (isVisible && userVisible)
    {
        attachmentScope.MakeVisible();
        mBufferAttachmentPathsToUpdateGeometry.insert(attachmentPath);
        mBufferAttachmentPathsToUpdateVizAttribute.insert(attachmentPath);

        for (uint32_t slot = 0; slot < 2; ++slot)
        {
            SdfPath targetPath = attachmentInfo.targets[slot];
            if (!targetPath.IsEmpty())
            {
                mBufferTargetPathsToUpdateTransform.insert(targetPath);
            }
        }

        SdfPath sessionAttachmentPath = mAttachmentToSession[attachmentPath];

        // update selection outlines
        const auto usdContext = omni::usd::UsdContext::getContext();
        uint8_t group = isSelected(attachmentPath) ? gProxySelectionGroup : 0;

        for (uint32_t slot = 0; slot < 2; ++slot)
        {
            SdfPath samplePointInstancerPath = getSessionSamplePointInstancerPath(sessionAttachmentPath, slot);
            SdfPath samplePointInstancerProtoPath = getSessionPointInstancerPrototypePath(samplePointInstancerPath);
            SdfPath filterPointInstancerPath = getSessionFilterPointInstancerPath(sessionAttachmentPath, slot);
            SdfPath filterPointInstancerProtoPath = getSessionPointInstancerPrototypePath(filterPointInstancerPath);
            if (gStage->GetPrimAtPath(samplePointInstancerPath))
            {
                usdContext->setSelectionGroup(group, samplePointInstancerPath.GetString());
                usdContext->setSelectionGroup(group, samplePointInstancerProtoPath.GetString());
            }
            if (gStage->GetPrimAtPath(filterPointInstancerPath))
            {
                usdContext->setSelectionGroup(group, filterPointInstancerPath.GetString());
                usdContext->setSelectionGroup(group, filterPointInstancerProtoPath.GetString());
            }
        }
    }
    else
    {
        attachmentScope.MakeInvisible();
    }
}

void AttachmentsVisualizationManagerDeprecated::updateRigidSkinVisibility(SdfPath targetPath, bool hide)
{
    TargetInfo* targetInfo = getTargetInfo(targetPath);
    if (targetInfo && targetInfo->type == TargetInfo::Type::eRigid)
    {
        UsdGeomImageable imageable = UsdGeomImageable::Get(gStage, targetPath);
        if (imageable)
        {
            TfToken newPurpose = hide ? UsdGeomTokens->proxy : UsdGeomTokens->default_;
            TfToken currentPurpose;
            imageable.GetPurposeAttr().Get(&currentPurpose);
            if (currentPurpose != newPurpose)
            {
                imageable.GetPurposeAttr().Set(newPurpose);
            }
        }
    }
}

bool AttachmentsVisualizationManagerDeprecated::isVisible(const SdfPath attachmentPath)
{
    return mVizAttachments.find(attachmentPath) != mVizAttachments.end();
}

bool AttachmentsVisualizationManagerDeprecated::isSelected(const SdfPath deformablePath)
{
    return mVizSelected.find(deformablePath) != mVizSelected.end();
}

bool AttachmentsVisualizationManagerDeprecated::isComplete(const SdfPath attachmentPath)
{
    SdfPathToSdfPathPairMap::const_iterator ait = mAttachments.find(attachmentPath);
    if (ait == mAttachments.end())
    {
        return false;
    }

    const TargetPathPair& targetPaths = ait->second;

    uint32_t numRigid = 0;
    uint32_t numDeformable = 0;
    uint32_t numCloth = 0;
    uint32_t numWorld = 0;
    for (uint32_t slot = 0; slot < 2; ++slot)
    {
        SdfPath targetPath = targetPaths[slot];
        if (targetPath.IsEmpty())
        {
            numWorld++;
        }
        else
        {
            UsdPrim prim = gStage->GetPrimAtPath(targetPath);
            if (prim)
            {
                if (prim.HasAPI<UsdPhysicsRigidBodyAPI>() || prim.HasAPI<UsdPhysicsCollisionAPI>())
                {
                    numRigid++;
                }
                else if (prim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>())
                {
                    numDeformable++;
                }
                else if (prim.HasAPI<PhysxSchemaPhysxParticleClothAPI>() || prim.HasAPI<PhysxSchemaPhysxDeformableSurfaceAPI>())
                {
                    numCloth++;
                }
            }
        }
    }

    bool isWorld = (numWorld == 1) && (numCloth + numDeformable == 1);
    bool isRigid = (numRigid == 1) && (numCloth + numDeformable == 1);
    bool isDeformable = (numDeformable + numCloth == 2) && (numCloth < 2);

    return (isWorld || isRigid || isDeformable);
}

namespace
{
    void updateDeformableBodyAttachmentCache(
        VtArray<int32_t>& deformableAttachmentTetIds,
        VtArray<GfVec4f>& deformableAttachmentBarycentrics,
        VtArray<uint32_t>& deformableFilterTetIds,
        const int slot,
        const PhysxSchemaPhysxPhysicsAttachment& attachment,
        const VtArray<int32_t>& deformableIndices,
        const uint64_t tetFinder)
    {
        VtArray<GfVec3f> attachmentPoints;
        attachment.GetPointsAttr(slot).Get(&attachmentPoints);

        deformableAttachmentTetIds.resize(attachmentPoints.size());
        deformableAttachmentBarycentrics.resize(attachmentPoints.size());
        gPhysXAttachmentPrivate->pointsToTetMeshLocal(&deformableAttachmentTetIds[0], (carb::Float4*)&deformableAttachmentBarycentrics[0],
            tetFinder, (const carb::Float3*)&attachmentPoints[0], uint32_t(attachmentPoints.size()));

        TfToken filterType;
        attachment.GetFilterTypeAttr(slot).Get(&filterType);

        if (filterType == PhysxSchemaTokens->Geometry)
        {
            VtArray<uint32_t> filterTetIds;
            attachment.GetCollisionFilterIndicesAttr(slot).Get(&filterTetIds);

            //we have no way of displaying filter pairs in case of deformable-deformable
            //or deformable-cloth filters. therefore we can uniquify the tet filters
            std::unordered_set<uint32_t> filterTetIdSet;
            for (size_t i = 0; i < filterTetIds.size(); ++i)
            {
                // skip wildcard filtering
                if (filterTetIds[i] != static_cast<uint32_t>(-1))
                    filterTetIdSet.insert(filterTetIds[i]);
            }
            deformableFilterTetIds.clear();
            deformableFilterTetIds.resize(filterTetIdSet.size());
            uint32_t tetIndex = 0;
            for (std::unordered_set<uint32_t>::const_iterator cit = filterTetIdSet.cbegin(); cit != filterTetIdSet.cend(); ++cit)
            {
                deformableFilterTetIds[tetIndex++] = *cit;
            }
        }
    }

    void updateParticleClothAttachmentCache(
        VtArray<int32_t>& clothAttachmentIndices,
        VtArray<uint32_t>& clothFilterIndices,
        const int slot,
        const PhysxSchemaPhysxPhysicsAttachment& attachment,
        const uint64_t pointFinder,
        const VtArray<uint32_t>& clothPointRemap)
    {
        VtArray<GfVec3f> attachmentPoints;
        attachment.GetPointsAttr(slot).Get(&attachmentPoints);

        if (clothPointRemap.size())
        {
            VtArray<int32_t> weldedSampleIndices;
            weldedSampleIndices.resize(attachmentPoints.size());
            gPhysXAttachmentPrivate->pointsToIndices(&weldedSampleIndices[0], pointFinder, (carb::Float3*)&attachmentPoints[0], uint32_t(attachmentPoints.size()));

            clothAttachmentIndices.resize(attachmentPoints.size());
            for (size_t i = 0; i < clothAttachmentIndices.size(); ++i)
            {
                clothAttachmentIndices[i] = clothPointRemap[weldedSampleIndices[i]];
            }
        }
        else
        {
            clothAttachmentIndices.resize(attachmentPoints.size());
            gPhysXAttachmentPrivate->pointsToIndices(&clothAttachmentIndices[0], pointFinder, (carb::Float3*)&attachmentPoints[0], uint32_t(attachmentPoints.size()));
        }

        VtArray<uint32_t> filterPointIds;
        attachment.GetCollisionFilterIndicesAttr(slot).Get(&filterPointIds);

        //we have no way of displaying filter pairs in case of deformable-cloth filters.
        //therefore we can uniquify the filter indices
        std::unordered_set<uint32_t> filterPointIdSet;
        for (size_t i = 0; i < filterPointIds.size(); ++i)
        {
            filterPointIdSet.insert(filterPointIds[i]);
        }

        clothFilterIndices.clear();
        clothFilterIndices.reserve(filterPointIdSet.size());
        if (clothPointRemap.size())
        {
            for (std::unordered_set<uint32_t>::const_iterator cit = filterPointIdSet.cbegin(); cit != filterPointIdSet.cend(); ++cit)
            {
                uint32_t pointId = *cit;
                clothFilterIndices.push_back(clothPointRemap[pointId]);
            }
        }
        else
        {
            for (std::unordered_set<uint32_t>::const_iterator cit = filterPointIdSet.cbegin(); cit != filterPointIdSet.cend(); ++cit)
            {
                uint32_t pointId = *cit;
                clothFilterIndices.push_back(pointId);
            }
        }
    }

    uint32_t updateDeformableBodyTargetAttachmentSamplePoints(
        UsdGeomPointInstancer& samplePointInstancer,
        const PhysxSchemaPhysxDeformableBodyAPI& deformableTarget,
        const int slot,
        const PhysxSchemaPhysxPhysicsAttachment& attachment,
        const VtArray<int32_t>& deformableIndices,
        const VtArray<int32_t>& deformableAttachmentTetIds,
        const VtArray<GfVec4f>& deformableAttachmentBarycentrics)
    {
        VtArray<GfVec3f> attachmentPoints;
        attachment.GetPointsAttr(slot).Get(&attachmentPoints);

        // Sanity check: Make sure that deformableAttachmentTetIds.size() is equal or bigger than attachmentPoints.size()
        if (deformableAttachmentTetIds.size() < attachmentPoints.size())
        {
            return 0;
        }

        VtArray<GfVec3f> deformablePoints;
        deformableTarget.GetCollisionPointsAttr().Get(&deformablePoints);

        bool valid = true;
        VtArray<GfVec3f> samplePoints;
        samplePoints.reserve(attachmentPoints.size());
        uint32_t indicesSize = uint32_t(deformableIndices.size());
        uint32_t pointsSize = uint32_t(deformablePoints.size());
        for (size_t p = 0; p < attachmentPoints.size(); ++p)
        {
            int32_t tetId = deformableAttachmentTetIds[p];
            if (tetId >= 0) // could be invalid due to precision issues
            {
                uint32_t i0 = deformableIndices[4*tetId + 0];
                uint32_t i1 = deformableIndices[4*tetId + 1];
                uint32_t i2 = deformableIndices[4*tetId + 2];
                uint32_t i3 = deformableIndices[4*tetId + 3];

                valid &= (i0 < pointsSize) && (i1 < pointsSize) && (i2 < pointsSize) && (i3 < pointsSize);
                if (valid)
                {
                    const GfVec3f& p0 = deformablePoints[i0];
                    const GfVec3f& p1 = deformablePoints[i1];
                    const GfVec3f& p2 = deformablePoints[i2];
                    const GfVec3f& p3 = deformablePoints[i3];
                    const GfVec4f b = deformableAttachmentBarycentrics[p];
                    samplePoints.push_back(p0 * b[0] + p1 * b[1] + p2 * b[2] + p3 * b[3]);
                }
            }
        }

        if (valid)
        {
            samplePointInstancer.GetPositionsAttr().Set(samplePoints);
            return uint32_t(samplePoints.size());
        }
        else
        {
            return 0;
        }
    }

    uint32_t updateParticleClothTargetAttachmentSamplePoints(
        UsdGeomPointInstancer& samplePointInstancer,
        const UsdGeomMesh& mesh,
        const VtArray<int32_t>& clothAttachmentIndices)
    {
        CARB_ASSERT(mesh);

        VtArray<GfVec3f> clothPoints;
        mesh.GetPointsAttr().Get(&clothPoints);

        bool valid = true;
        int32_t pointsSize = int32_t(clothPoints.size());
        VtArray<GfVec3f> samplePoints(clothAttachmentIndices.size());
        for (size_t p = 0; p < clothAttachmentIndices.size(); ++p)
        {
            int32_t index = clothAttachmentIndices[p];
            valid &= (index >= 0 && index < pointsSize);
            if (valid)
            {
                samplePoints[p] = clothPoints[index];
            }
        }

        if (valid)
        {
            samplePointInstancer.GetPositionsAttr().Set(samplePoints);
            return uint32_t(clothAttachmentIndices.size());
        }
        else
        {
            return 0;
        }
    }

    uint32_t updateDeformableBodyTargetAttachmentFilterPoints(
        UsdGeomPointInstancer& filterPointInstancer,
        const PhysxSchemaPhysxDeformableBodyAPI& deformableBodyTarget,
        const int slot,
        const PhysxSchemaPhysxPhysicsAttachment& attachment)
    {
        TfToken filterType;
        attachment.GetFilterTypeAttr(slot).Get(&filterType);

        VtArray<GfVec3f> points;
        deformableBodyTarget.GetCollisionPointsAttr().Get(&points);

        bool valid = true;
        uint32_t pointsSize = uint32_t(points.size());
        if (filterType == PhysxSchemaTokens->Vertices)
        {
            VtArray<uint32_t> filterIndices;
            attachment.GetCollisionFilterIndicesAttr(slot).Get(&filterIndices);

            VtArray<GfVec3f> filterPoints(filterIndices.size());
            for (size_t i = 0; i < filterIndices.size(); ++i)
            {
                uint32_t index = filterIndices[i];
                valid &= (index < pointsSize);
                if (valid)
                {
                    filterPoints[i] = points[index];
                }
            }

            if (valid)
            {
                filterPointInstancer.GetPositionsAttr().Set(filterPoints);
                return uint32_t(filterPoints.size());
            }
            else
            {
                return 0;
            }
        }
        else if (filterType == PhysxSchemaTokens->Geometry)
        {
            //filter tets are now visualized as part of the deformable visualization
            return uint32_t(0);
        }
        return 0;
    }

    uint32_t updateParticleClothTargetAttachmentFilterPoints(
        UsdGeomPointInstancer& filterPointInstancer,
        const UsdGeomMesh& mesh,
        const int slot,
        const VtArray<uint32_t>& clothFilterIndices)
    {
        VtArray<GfVec3f> points;
        mesh.GetPointsAttr().Get(&points);

        bool valid = true;
        uint32_t pointsSize = uint32_t(points.size());
        VtArray<GfVec3f> filterPoints(clothFilterIndices.size());
        for (size_t i = 0; i < clothFilterIndices.size(); ++i)
        {
            uint32_t index = clothFilterIndices[i];
            valid &= (index < pointsSize);
            if (valid)
            {
                filterPoints[i] = points[index];
            }
        }

        if (valid)
        {
            filterPointInstancer.GetPositionsAttr().Set(filterPoints);
            return uint32_t(filterPoints.size());
        }
        else
        {
            return 0;
        }
    }

    void updatePrototypeScale(const SdfPath& pointInstancerPath, const GfMatrix4d& targetToWorld)
    {
        SdfPath pointPrototypePath = pointInstancerPath.AppendElementString(std::string("prototype"));
        UsdGeomSphere pointPrototype = UsdGeomSphere::Get(gStage, pointPrototypePath);

        GfVec3d scale = GfTransform(targetToWorld).GetScale();
        for (int i = 0; i < 3; ++i)
        {
            if (std::abs(scale[i]) <= 1e-6)
                // sanitize in case of near zero:
                scale[i] = std::signbit(scale[i]) ? -1e6 : 1e6;
            else
                scale[i] = 1.0 / scale[i];
        }
        // set new scale
        const UsdAttribute scaleAttr = pointPrototype.GetPrim().GetAttribute(scaleAttrName);
        if (!scaleAttr.IsValid())
            return;
        scaleAttr.Set(GfVec3f(scale));
    }

}

void AttachmentsVisualizationManagerDeprecated::updateTargetTransform(const SdfPath targetPath)
{
    // get target prim
    UsdPrim targetPrim = gStage->GetPrimAtPath(targetPath);
    if(!targetPrim)
        return;  // no need to update if there is no target prim

    if (mTargetToInfo.count(targetPath))
    {
        const SdfPathSet& targetAttachments = mTargets[targetPath];

        for (SdfPath attachmentPath : targetAttachments)
        {
            if (!isVisible(attachmentPath))
                continue;

            PhysxSchemaPhysxPhysicsAttachment attachment = PhysxSchemaPhysxPhysicsAttachment(gStage->GetPrimAtPath(attachmentPath));
            if (!attachment || !isVisible(attachmentPath))
                continue;

            const AttachmentInfo& attachmentInfo = *mAttachmentToInfo[attachmentPath];
            CARB_ASSERT(attachmentInfo.targets[0] == targetPath || attachmentInfo.targets[1] == targetPath);
            int32_t slot = attachmentInfo.targets[0] == targetPath ? 0 : 1;

            SdfPath sessionAttachmentPath = mAttachmentToSession[attachmentPath];
            SdfPath targetXformPath = getSessionTargetPath(sessionAttachmentPath, slot);
            
            // get transformation with cache:
            const GfMatrix4d targetToWorld = mXformCache.GetLocalToWorldTransform(targetPrim);

            UsdGeomXform targetXform = UsdGeomXform::Get(gStage, targetXformPath);
            UsdAttribute localToWorldAttr = targetXform.GetPrim().GetAttribute(transformMatrixAttrName);
            CARB_ASSERT(localToWorldAttr);
            localToWorldAttr.Set(targetToWorld);

            SdfPath samplePointInstancerPath = getSessionSamplePointInstancerPath(sessionAttachmentPath, slot);
            updatePrototypeScale(samplePointInstancerPath, targetToWorld);

            if (targetPrim.HasAPI<PhysxSchemaPhysxDeformableAPI>() || targetPrim.HasAPI<PhysxSchemaPhysxDeformableSurfaceAPI>() || targetPrim.HasAPI<PhysxSchemaPhysxParticleClothAPI>())
            {
                SdfPath filterPointInstancerPath = getSessionFilterPointInstancerPath(sessionAttachmentPath, slot);
                updatePrototypeScale(filterPointInstancerPath, targetToWorld);
            }
        }
    }
}

void AttachmentsVisualizationManagerDeprecated::updateTargetGeometry(const SdfPath targetPath)
{
    // get target prim
    UsdPrim targetPrim = gStage->GetPrimAtPath(targetPath);
    if (!targetPrim)
        return;  // no need to update if there is no target prim

    CARB_ASSERT(targetPrim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>() || targetPrim.HasAPI<PhysxSchemaPhysxDeformableSurfaceAPI>() || targetPrim.HasAPI<PhysxSchemaPhysxParticleClothAPI>());

    SdfPathToTargetInfoMap::const_iterator tit = mTargetToInfo.find(targetPath);
    if (tit != mTargetToInfo.cend())
    {
        const TargetInfo& targetInfo = *tit->second;
        const SdfPathSet& targetAttachments = mTargets[targetPath];
        for (SdfPath attachmentPath : targetAttachments)
        {
            if (!isVisible(attachmentPath))
                continue;

            if (mBufferAttachmentPathsToUpdateGeometry.count(attachmentPath) > 0)
            {
                // skip attachments which are going to be updated later on
                continue;
            }

            PhysxSchemaPhysxPhysicsAttachment attachment = PhysxSchemaPhysxPhysicsAttachment::Get(gStage, attachmentPath);
            if (!attachment)
                continue;

            const AttachmentInfo& attachmentInfo = *mAttachmentToInfo[attachmentPath];
            CARB_ASSERT(attachmentInfo.targets[0] == targetPath || attachmentInfo.targets[1] == targetPath);
            int slot = (attachmentInfo.targets[0] == targetPath) ? 0 : 1;

            SdfPath sessionAttachmentPath = mAttachmentToSession[attachmentPath];
            SdfPath samplePointInstancerPath = getSessionSamplePointInstancerPath(sessionAttachmentPath, slot);
            SdfPath filterPointInstancerPath = getSessionFilterPointInstancerPath(sessionAttachmentPath, slot);

            UsdGeomPointInstancer samplePointInstancer = UsdGeomPointInstancer::Get(gStage, samplePointInstancerPath);
            UsdGeomPointInstancer filterPointInstancer = UsdGeomPointInstancer::Get(gStage, filterPointInstancerPath);

            if (targetInfo.type == TargetInfo::eDeformableBody)
            {
                updateDeformableBodyTargetAttachmentSamplePoints(samplePointInstancer,
                    PhysxSchemaPhysxDeformableBodyAPI(targetPrim), slot, attachment,
                    targetInfo.indices, attachmentInfo.deformableSampleTetIds[slot],
                    attachmentInfo.deformableSampleBarycentrics[slot]);

                updateDeformableBodyTargetAttachmentFilterPoints(filterPointInstancer,
                    PhysxSchemaPhysxDeformableBodyAPI(targetPrim), slot, attachment);
            }
            else if (targetInfo.type == TargetInfo::eDeformableSurface)
            {
                const PhysxSchemaPhysxDeformableSurfaceAPI deformableSurfaceTarget(targetPrim);
                UsdGeomMesh mesh(deformableSurfaceTarget);
                updateParticleClothTargetAttachmentSamplePoints(samplePointInstancer,
                    mesh, attachmentInfo.clothSampleIndices[slot]);

                updateParticleClothTargetAttachmentFilterPoints(filterPointInstancer,
                    mesh, slot, attachmentInfo.clothFilterIndices[slot]);
            }
            else if (targetInfo.type == TargetInfo::eParticleCloth)
            {
                const PhysxSchemaPhysxParticleClothAPI particleClothTarget(targetPrim);
                UsdGeomMesh mesh(particleClothTarget);
                updateParticleClothTargetAttachmentSamplePoints(samplePointInstancer,
                    mesh, attachmentInfo.clothSampleIndices[slot]);

                updateParticleClothTargetAttachmentFilterPoints(filterPointInstancer,
                    mesh, slot, attachmentInfo.clothFilterIndices[slot]);
            }
        }
    }
}

void AttachmentsVisualizationManagerDeprecated::updateAttachmentGeometry(const SdfPath attachmentPath)
{
    if (!isVisible(attachmentPath))
        return;

    // get attachment
    PhysxSchemaPhysxPhysicsAttachment attachment = PhysxSchemaPhysxPhysicsAttachment(gStage->GetPrimAtPath(attachmentPath));
    if(!attachment)
        return;

    SdfPath sessionAttachmentPath = mAttachmentToSession[attachmentPath];
    AttachmentInfo& attachmentInfo = *mAttachmentToInfo[attachmentPath];

    for (int32_t s = 0; s < 2; ++s)
    {
        SdfPath targetPath = attachmentInfo.targets[s];

        uint32_t numPointSamples = 0;
        uint32_t numPointFilters = 0;

        SdfPath samplePointInstancerPath = getSessionSamplePointInstancerPath(sessionAttachmentPath, s);
        SdfPath filterPointInstancerPath = getSessionFilterPointInstancerPath(sessionAttachmentPath, s);

        UsdGeomPointInstancer samplePointInstancer = UsdGeomPointInstancer::Get(gStage, samplePointInstancerPath);

        if (targetPath.IsEmpty())
        {
            // world target case
            if (samplePointInstancer)
            {
                VtArray<GfVec3f> samplePoints;
                attachment.GetPointsAttr(s).Get(&samplePoints);
                samplePointInstancer.GetPositionsAttr().Set(samplePoints);
                numPointSamples = uint32_t(samplePoints.size());
            }
        }
        else
        {
            UsdPrim targetPrim = gStage->GetPrimAtPath(targetPath);
            auto it = mTargetToInfo.find(targetPrim.GetPath());
            if (targetPrim && it != mTargetToInfo.end())
            {
                const TargetInfo& targetInfo = *it->second;
                if (targetInfo.type == TargetInfo::eRigid)
                {
                    VtArray<GfVec3f> rigidSamplePoints;
                    attachment.GetPointsAttr(s).Get(&rigidSamplePoints);

                    samplePointInstancer.GetPositionsAttr().Set(rigidSamplePoints);
                    numPointSamples = uint32_t(rigidSamplePoints.size());
                }
                else
                {
                    UsdGeomPointInstancer filterPointInstancer = UsdGeomPointInstancer::Get(gStage, filterPointInstancerPath);

                    if (targetInfo.type == TargetInfo::eDeformableBody)
                    {
                        updateDeformableBodyAttachmentCache(attachmentInfo.deformableSampleTetIds[s],
                            attachmentInfo.deformableSampleBarycentrics[s],
                            attachmentInfo.deformableFilterTetIds[s],
                            s, attachment, targetInfo.indices, targetInfo.tetFinder);

                        numPointSamples = updateDeformableBodyTargetAttachmentSamplePoints(samplePointInstancer,
                            PhysxSchemaPhysxDeformableBodyAPI(targetPrim), s, attachment,
                            targetInfo.indices, attachmentInfo.deformableSampleTetIds[s],
                            attachmentInfo.deformableSampleBarycentrics[s]);

                        numPointFilters = updateDeformableBodyTargetAttachmentFilterPoints(filterPointInstancer,
                            PhysxSchemaPhysxDeformableBodyAPI(targetPrim), s, attachment);
                    }
                    else if (targetInfo.type == TargetInfo::eDeformableSurface)
                    {
                        updateParticleClothAttachmentCache(attachmentInfo.clothSampleIndices[s],
                            attachmentInfo.clothFilterIndices[s],
                            s, attachment, targetInfo.pointFinder, targetInfo.clothPointRemap);

                        const PhysxSchemaPhysxDeformableSurfaceAPI deformableSurfaceTarget(targetPrim);
                        UsdGeomMesh mesh(deformableSurfaceTarget);
                        numPointSamples = updateParticleClothTargetAttachmentSamplePoints(samplePointInstancer,
                            mesh, attachmentInfo.clothSampleIndices[s]);

                        numPointFilters = updateParticleClothTargetAttachmentFilterPoints(filterPointInstancer,
                            mesh, s, attachmentInfo.clothFilterIndices[s]);
                    }
                    else if (targetInfo.type == TargetInfo::eParticleCloth)
                    {
                        updateParticleClothAttachmentCache(attachmentInfo.clothSampleIndices[s],
                            attachmentInfo.clothFilterIndices[s],
                            s, attachment, targetInfo.pointFinder, targetInfo.clothPointRemap);

                        const PhysxSchemaPhysxParticleClothAPI particleClothTarget(targetPrim);
                        UsdGeomMesh mesh(particleClothTarget);
                        numPointSamples = updateParticleClothTargetAttachmentSamplePoints(samplePointInstancer,
                            mesh, attachmentInfo.clothSampleIndices[s]);

                        numPointFilters = updateParticleClothTargetAttachmentFilterPoints(filterPointInstancer,
                            mesh, s, attachmentInfo.clothFilterIndices[s]);
                    }
                }
            }
            else
            {
                //removed target
                numPointSamples = 0;
                numPointFilters = 0;
            }
        }

        //Update selection group in case of changing number of points (workaround)
        const auto usdContext = omni::usd::UsdContext::getContext();
        uint8_t group = isSelected(attachmentPath) ? gProxySelectionGroup : 0;

        if (gStage->GetPrimAtPath(samplePointInstancerPath))
        {
            usdContext->setSelectionGroup(group, samplePointInstancerPath.GetString());

            if (attachmentInfo.numPointSamples[s] != numPointSamples)
            {
                VtArray<int> protoIndices(numPointSamples, 0);
                UsdGeomPointInstancer::Get(gStage, samplePointInstancerPath).GetProtoIndicesAttr().Set(protoIndices);

                if (numPointSamples > 0)
                {
                    if (isVisible(attachmentPath))
                    {
                        SdfPath sessionAttachmentPath = mAttachmentToSession[attachmentPath];
                        float pointScale = getPointScale(attachmentPath);
                        SdfPath samplePointInstancerPath = getSessionSamplePointInstancerPath(sessionAttachmentPath, s);
                        updateSessionPointInstancerRadius(samplePointInstancerPath, kRadiusSample[s] * pointScale);
                    }
                }
                else
                {
                    resetSessionPointInstancer(samplePointInstancerPath);
                }
            }
        }
        if (gStage->GetPrimAtPath(filterPointInstancerPath))
        {
            usdContext->setSelectionGroup(group, filterPointInstancerPath.GetString());

            if (attachmentInfo.numPointFilters[s] != numPointFilters)
            {
                VtArray<int> protoIndices(numPointFilters, 0);
                UsdGeomPointInstancer::Get(gStage, filterPointInstancerPath).GetProtoIndicesAttr().Set(protoIndices);

                if (numPointFilters > 0)
                {
                    if (isVisible(attachmentPath))
                    {
                        SdfPath sessionAttachmentPath = mAttachmentToSession[attachmentPath];
                        float pointScale = getPointScale(attachmentPath);
                        SdfPath filterPointInstancerPath = getSessionFilterPointInstancerPath(sessionAttachmentPath, s);
                        updateSessionPointInstancerRadius(filterPointInstancerPath, kRadiusFilter[s] * pointScale);
                    }
                }
                else
                {
                    resetSessionPointInstancer(filterPointInstancerPath);
                }
            }
        }

        attachmentInfo.numPointSamples[s] = numPointSamples;
        attachmentInfo.numPointFilters[s] = numPointFilters;
    }
}

void AttachmentsVisualizationManagerDeprecated::updateVisualizationScale(const SdfPath attachmentPath)
{
    if (!isVisible(attachmentPath))
        return;

    AttachmentInfo* attachmentInfo = getAttachmentInfo(attachmentPath);
    if (!attachmentInfo)
        return;

    SdfPath sessionAttachmentPath = mAttachmentToSession[attachmentPath];
    float pointScale = getPointScale(attachmentPath);

    for (int32_t s = 0; s < 2; ++s)
    {
        SdfPath samplePointInstancerPath = getSessionSamplePointInstancerPath(sessionAttachmentPath, s);
        updateSessionPointInstancerRadius(samplePointInstancerPath, kRadiusSample[s] * pointScale * (attachmentInfo->numPointSamples[s] > 0 ? 1.0f : float(kPrototypeRadiusZero)));

        SdfPath filterPointInstancerPath = getSessionFilterPointInstancerPath(sessionAttachmentPath, s);
        updateSessionPointInstancerRadius(filterPointInstancerPath, kRadiusFilter[s] * pointScale * (attachmentInfo->numPointFilters[s] > 0 ? 1.0f : float(kPrototypeRadiusZero)));
    }
}

bool AttachmentsVisualizationManagerDeprecated::needsAttachmentVisualization(const SdfPath deformablePath, bool* forceHide)
{
    if (forceHide)
    {
        *forceHide = isTargetHidden(deformablePath);
    }
    SdfPathToSdfPathSetMap::const_iterator tit = mTargets.find(deformablePath);
    if (tit != mTargets.cend())
    {
        const SdfPathSet& attachmentPaths = tit->second;
        for (SdfPath attachmentPath : attachmentPaths)
        {
            if (isVisible(attachmentPath))
            {
                return true;
            }
        }
    }
    return false;
}

void AttachmentsVisualizationManagerDeprecated::getFilteredTetIndices(VtArray<int32_t>& indices, const SdfPath deformablePath)
{
    indices.clear();

    std::unordered_set<uint32_t> filterTets;
    SdfPathToSdfPathSetMap::const_iterator tit = mTargets.find(deformablePath);
    if (tit != mTargets.end())
    {
        const SdfPathSet& attachmentPaths = tit->second;
        for (SdfPath attachmentPath : attachmentPaths)
        {
            if (isVisible(attachmentPath))
            {
                SdfPathToAttachmentInfoMap::const_iterator ait = mAttachmentToInfo.find(attachmentPath);
                if (ait != mAttachmentToInfo.cend())
                {
                    const AttachmentInfo& attachmentInfo = *ait->second;
                    CARB_ASSERT(attachmentInfo.targets[0] == deformablePath || attachmentInfo.targets[1] == deformablePath);
                    int32_t slot = attachmentInfo.targets[0] == deformablePath ? 0 : 1;

                    for (size_t t = 0; t < attachmentInfo.deformableFilterTetIds[slot].size(); ++t)
                    {
                        // skip wildcard filtering
                        if (attachmentInfo.deformableFilterTetIds[slot][t] != static_cast<uint32_t>(-1))
                            filterTets.insert(attachmentInfo.deformableFilterTetIds[slot][t]);
                    }
                }
            }
        }
    }

    size_t newIndicesSize = filterTets.size() * 4;
    indices.resize(newIndicesSize);
    uint32_t dstIndex = 0;
    for (const uint32_t tetId : filterTets)
    {
        indices[dstIndex++] = tetId * 4 + 0;
        indices[dstIndex++] = tetId * 4 + 1;
        indices[dstIndex++] = tetId * 4 + 2;
        indices[dstIndex++] = tetId * 4 + 3;
    }
}

void AttachmentsVisualizationManagerDeprecated::clearBuffers(void)
{
    mBufferAttachmentPathsToAdd.clear();
    mBufferAttachmentPathsToRemove.clear();
    mBufferAttachmentPathsToUpdateGeometry.clear();
    mBufferAttachmentPathsToUpdateVizAttribute.clear();
    mBufferTargetPathsToUpdateTransform.clear();
    mBufferTargetPathsToUpdateGeometry.clear();
    mBufferVizDirty = false;
}

float AttachmentsVisualizationManagerDeprecated::getPointScale(const SdfPath attachmentPath)
{
    const AttachmentInfo& attachmentInfo = *mAttachmentToInfo[attachmentPath];
    float pointScale = FLT_MAX;
    for (uint32_t slot = 0; slot < 2; ++slot)
    {
        const SdfPath targetPath = attachmentInfo.targets[slot];
        auto it = mTargetToInfo.find(attachmentInfo.targets[slot]);
        if (it != mTargetToInfo.end())
        {
            const TargetInfo& targetInfo = *it->second;
            UsdPrim targetPrim = gStage->GetPrimAtPath(targetPath);
            if (targetPrim && targetInfo.pointScale > 0.0f)
            {
                const GfVec3d scale = GfTransform(mXformCache.GetLocalToWorldTransform(targetPrim)).GetScale();
                float maxScaleDim = float(scale[0]);
                maxScaleDim = std::max(maxScaleDim, float(scale[1]));
                maxScaleDim = std::max(maxScaleDim, float(scale[2]));

                pointScale = std::min(maxScaleDim*targetInfo.pointScale, pointScale);
            }
        }
    }
    pointScale = (pointScale == FLT_MAX) ? 0.0f : pointScale;
    return pointScale*mVisualizationScale;
}

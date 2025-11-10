// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "DeformableBodyVisualizationManagerDeprecated.h"
#include "AttachmentsVisualizationManagerDeprecated.h"
#include "TetrahedralMeshVisualizerDeprecated.h"

#include <algorithm>

#include <carb/profiler/Profile.h>
#include <omni/kit/EditorUsd.h>
#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/Selection.h>
#include <omni/physx/IPhysxSettings.h>

#include <physxSchema/tetrahedralMesh.h>

using namespace omni::physx::ui;
using namespace pxr;

extern UsdStageRefPtr gStage;
extern carb::settings::ISettings* gSettings;
extern bool gBlockNoticeHandle;
extern uint8_t gProxySelectionGroup;

static const TfToken transformMatrixAttrName{ "xformOp:transform" };
static const TfToken visualizationGapAttributeName{ "visualizationGap" };
static const TfToken scaleAttrName{ "xformOp:scale" };
static const TfToken invMassScaleAttributeName("physxDeformable:invMassScale");

static constexpr char kViewportGizmoScalePath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/scale";
static constexpr char kViewportGizmoConstantScaleEnabledPath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/constantScaleEnabled";
static constexpr char kViewportGizmoConstantScalePath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/constantScale";

static const GfVec3f kColorCollision(0.0f, 0.3f, 0.05f); // green
static const GfVec3f kColorSimulation(0.0f, 0.05f, 0.3f); // blue
static const GfVec3f kColorFilter = GfVec3f(0.0f, 0.7f, 0.5f); // cyan

static const float kRadiusSample = 1.2f;
static const GfVec3f kColorSample = GfVec3f(0.0f, 0.0f, 0.4f); // blue

static const float kEdgeToPointScale = 0.1f;

static const double kPrototypeRadiusZero = 0.0;

namespace
{
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

    void readVisualizationScale(float& visualizationScale)
    {
        visualizationScale = 1.0f;

        if (gSettings)
        {
            if (gSettings->getAsBool(kViewportGizmoConstantScaleEnabledPath))
            {
                //kViewportGizmoConstantScalePath: default = 10.0f
                visualizationScale = 0.1f * gSettings->getAsFloat(kViewportGizmoConstantScalePath);
            }
            else
            {
                //kViewportGizmoScalePath: default = 1.0f
                //meterPerUnit: default = 0.01f
                visualizationScale = 0.01f * gSettings->getAsFloat(kViewportGizmoScalePath) / float(pxr::UsdGeomGetStageMetersPerUnit(gStage));
            }
        }
    }

    template <typename T>
    bool SafeGetAttributeP(T* out, UsdAttribute const& attribute)
    {
        if (attribute.HasValue())
        {
            attribute.Get(out);

            return true;
        }

        return false;
    }
}

DeformableBodyVisualizationManagerDeprecated::DeformableBodyVisualizationManagerDeprecated(const VisualizerMode mode, const DeformableBodyVisualizerTypeDeprecated type) :
    mSessionLayerTetsRoot{ SdfPath() },
    mMode(mode),
    mType(type),
    mVisualizationGap(-1.0f),
    mVisualizationScale(-1.0f),
    mBufferVizDirty(false)
{
}

DeformableBodyVisualizationManagerDeprecated::~DeformableBodyVisualizationManagerDeprecated()
{
    release();
}

SdfPath DeformableBodyVisualizationManagerDeprecated::getSessionInvMassPointInstancerPath(const pxr::SdfPath deformablePath)
{
    const auto it = mDeformableToSessionTable.find(deformablePath);
    if (it == mDeformableToSessionTable.end())
        return SdfPath();

    SdfPath collisionPath = it->second.first;
    // Get collision mesh session path for inverse mass points visualization
    return collisionPath.IsEmpty() ? SdfPath() : collisionPath.AppendElementString("samplePoints");
}

TetrahedralMeshVisualizerDeprecated* DeformableBodyVisualizationManagerDeprecated::getTetMeshVisualizer(const SdfPath sessionTetPath)
{
    TetrahedralMeshVisualizerDeprecated* visualizer = nullptr;

    const auto iter = mSessionToTetVisualizerMap.find(sessionTetPath);
    if (iter != mSessionToTetVisualizerMap.end())
    {
        visualizer = iter->second;
    }  

    return visualizer;
}

void DeformableBodyVisualizationManagerDeprecated::setAttachmentsVisualizationManager(AttachmentsVisualizationManagerDeprecated& attachmentsVisualizationManager)
{
    mAttachmentsVisualizationManager = &attachmentsVisualizationManager;
}

void DeformableBodyVisualizationManagerDeprecated::notifyAttachmentVisualization(bool deformableResync, bool deformableRelease)
{
    CARB_ASSERT(!(deformableResync && deformableRelease));
    if (mMode == VisualizerMode::eNone)
    {
        // because the deformable viz has attachment viz functionality
        // we need to make sure it's functional even if mMode is eNone
        if (deformableResync)
        {
            if (gStage)
            {
                handlePrimResync(gStage->GetPseudoRoot().GetPath());
            }
        }
        else if (deformableRelease)
        {
            clear(true);
        }
    }
    mBufferVizDirty = true;
}

bool DeformableBodyVisualizationManagerDeprecated::isActive() const
{
    bool deformableViz = mMode != VisualizerMode::eNone;
    bool attachmentViz = mAttachmentsVisualizationManager ? mAttachmentsVisualizationManager->isActive() : false;
    return deformableViz || attachmentViz;
};

void DeformableBodyVisualizationManagerDeprecated::setMode(const VisualizerMode mode)
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
        else if (mode == VisualizerMode::eNone && !(mAttachmentsVisualizationManager && mAttachmentsVisualizationManager->isActive()))
        {
            clear(true);
        }
    }

    mMode = mode;
    mBufferVizDirty = true;
}

void DeformableBodyVisualizationManagerDeprecated::setType(const DeformableBodyVisualizerTypeDeprecated type)
{
    mType = type;
    mBufferVizDirty = true;
}

DeformableBodyVisualizerTypeDeprecated DeformableBodyVisualizationManagerDeprecated::getDisplayType(const pxr::SdfPath deformablePath)
{
    return mAttachmentsVisualizationManager->needsAttachmentVisualization(deformablePath) ? Type::eCollision : mType;
}

void DeformableBodyVisualizationManagerDeprecated::parseStage()
{
    CARB_PROFILE_ZONE(0, "DeformableBodyVisualizationManagerDeprecated::parseStage");

    if (!isActive())
        return;

    handlePrimResync(gStage->GetPseudoRoot().GetPath());
}

void DeformableBodyVisualizationManagerDeprecated::update()
{
    CARB_PROFILE_ZONE(0, "DeformableBodyVisualizationManagerDeprecated::Update");

    if (!gStage || !isActive())
        return;

    {
        // all updates are to session layer
        omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());

        // register new deformable paths
        for (SdfPathSet::const_iterator cit = mBufferPathsToAdd.cbegin(); cit != mBufferPathsToAdd.cend(); ++cit)
        {
            mDeformables.insert(*cit);
            mDeformableToSessionTable.insert({*cit, SdfPathPair()});
            // make sure they are processed in the viz updates:
            mBufferVizDirty = true;
        }

        for (SdfPathSet::const_iterator cit = mBufferPathsToUpdate.cbegin(); cit != mBufferPathsToUpdate.cend(); ++cit)
        {
            const UsdPrim prim = gStage->GetPrimAtPath(*cit);
            if (prim && !prim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>())
            {
                removeDeformable(*cit);
            }
            else
            {
                //could be a tranform related attribute removal
                mBufferPathsToUpdateTransform.insert(*cit);
            }
        }

        if (!mDeformables.empty())
        {
            // set new tet gap
            const float visualizationGap = gSettings ? gSettings->getAsFloat(omni::physx::kSettingVisualizationGap) : 0.0f;

            // set new vis scale
            float visualizationScale;
            readVisualizationScale(visualizationScale);

            const bool visualizationDirty = (mVisualizationGap != visualizationGap) || (mVisualizationScale != visualizationScale);
            mVisualizationGap = visualizationGap;
            mVisualizationScale = visualizationScale;

            if (mBufferVizDirty)
            {
                mVizDeformables.clear();
                mVizSelected.clear();

                // update selection if viz dirty
                const auto usdContext = omni::usd::UsdContext::getContext();
                const std::vector<SdfPath> selectedPaths = usdContext->getSelection()->getSelectedPrimPathsV2();

                for (uint32_t s = 0; s < selectedPaths.size(); ++s)
                {
                    // use awesome path table to quickly find all candidate deformable paths to update:
                    const auto iteratorPair = mDeformableToSessionTable.FindSubtreeRange(selectedPaths[s]);
                    for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
                    {
                        // it->first is a path in the subtree starting at path s.
                        // so if it is a deformable, add it to  update buffer
                        if (mDeformables.count(it->first))
                            mVizSelected.insert(it->first);
                    }
                }

                if (mMode == VisualizerMode::eSelected)
                {
                    mVizDeformables = mVizSelected;
                }
                else if (mMode == VisualizerMode::eAll)
                {
                    mVizDeformables = mDeformables;
                }

                for (SdfPathSet::const_iterator cit = mDeformables.cbegin(); cit != mDeformables.cend(); ++cit)
                {
                    bool needsViz = mAttachmentsVisualizationManager->needsAttachmentVisualization(*cit);
                    if (needsViz)
                    {
                        mVizDeformables.insert(*cit);
                    }
                }
                
            }

            SdfPathSet setupPaths; // buffer for newly created tetmeshes that need xform ops added, and other attributes
            for (SdfPathSet::const_iterator cit = mDeformables.cbegin(); cit != mDeformables.cend(); ++cit)
            {
                // create if necessary:
                if (getSessionLayerTetmeshPath(*cit, getDisplayType(*cit)).IsEmpty() && mBufferPathsToUpdateTopology.count(*cit) > 0)
                {
                    createSessionTetmesh(*cit, Type::eCollision);
                    createSessionTetmesh(*cit, Type::eSimulation);

                    SdfPath samplePointInstancerPath = getSessionInvMassPointInstancerPath(*cit);
                    createSessionPointInstancer(samplePointInstancerPath);

                    setupPaths.insert(*cit);

                    mBufferVizDirty = true;

                    const float pointScale = calculateDeformablePointScale(*cit);
                    mDeformableToScaleMap.insert({*cit, pointScale });
                }
            }

            for (SdfPath deformablePath : mBufferPathsToUpdatePointInstancers)
            {
                const float pointScale = getPointScale(deformablePath);
                updatePointInstancer(deformablePath, pointScale);
            }

            if (visualizationDirty)
            {
                mBufferPathsToUpdateGap = mVizDeformables;
            }

            // Do all attribute updates and prim deletions in Sdf changeblock:
            {
                SdfChangeBlock block;
                // setup xform ops for newly created tets:
                for (SdfPathSet::const_iterator cit = setupPaths.cbegin(); cit != setupPaths.cend(); ++cit)
                {
                    setupSessionTetmesh(*cit, Type::eCollision);
                    setupSessionTetmesh(*cit, Type::eSimulation);

                    const float pointScale = getPointScale(*cit);
                    setupSessionPointInstancers(*cit, pointScale);
                }

                if (mBufferVizDirty)
                {
                    // update visibilities of helper tetmeshes
                    for (SdfPathSet::const_iterator cit = mDeformables.cbegin(); cit != mDeformables.cend(); ++cit)
                    {
                        Type vizType = getDisplayType(*cit);
                        Type otherType = vizType == Type::eCollision ? Type::eSimulation : Type::eCollision;
                        updateTetVisibility(*cit, getSessionLayerTetmeshPath(*cit, vizType), isVisible(*cit));
                        updateTetVisibility(*cit, getSessionLayerTetmeshPath(*cit, otherType), false);
                    }
                }

                // update xforms and verts:
                mXformCache.Clear();

                for (SdfPathSet::const_iterator cit = mBufferPathsToUpdateTransform.cbegin(); cit != mBufferPathsToUpdateTransform.cend(); ++cit)
                {
                    updateTetTransform(*cit, getSessionLayerTetmeshPath(*cit, getDisplayType(*cit)));
                }

                for (SdfPathSet::const_iterator cit = mBufferPathsToUpdatePoints.cbegin(); cit != mBufferPathsToUpdatePoints.cend(); ++cit)
                {
                    if (mBufferPathsToUpdateTopology.count(*cit) == 0)
                    {
                        updateTetPoints(*cit, getSessionLayerTetmeshPath(*cit, getDisplayType(*cit)), getDisplayType(*cit));

                        const float pointScale = calculateDeformablePointScale(*cit);
                        mDeformableToScaleMap[*cit] = pointScale;
                    }
                }

                for (SdfPathSet::const_iterator cit = mBufferPathsToUpdateTopology.cbegin(); cit != mBufferPathsToUpdateTopology.cend(); ++cit)
                {
                    updateTetTopology(*cit, getSessionLayerTetmeshPath(*cit, getDisplayType(*cit)), getDisplayType(*cit));
                }

                for (SdfPathSet::const_iterator cit = mBufferPathsToUpdateGap.cbegin(); cit != mBufferPathsToUpdateGap.cend(); ++cit)
                {
                    updateTetGap(*cit, getSessionLayerTetmeshPath(*cit, getDisplayType(*cit)));
                    updateVisualizationScale(*cit);
                }

                // delete removed deformables:
                for (SdfPathSet::const_iterator cit = mBufferPathsToRemove.cbegin(); cit != mBufferPathsToRemove.cend(); ++cit)
                {
                    removeDeformable(*cit);
                }
            }
        }

        // block all notices from changes starting here:
        gBlockNoticeHandle = true;

        if (mBufferVizDirty)
        {
            for (SdfPathSet::const_iterator cit = mDeformables.cbegin(); cit != mDeformables.cend(); ++cit)
            {
                mBufferPathsToUpdateSkinViz.insert(*cit);
            }
        }

        for (SdfPathSet::const_iterator cit = mBufferPathsToUpdateSkinViz.cbegin(); cit != mBufferPathsToUpdateSkinViz.cend(); ++cit)
        {
            bool skinVisible = !isVisible(*cit);
            updateSkinVisibility(*cit, skinVisible);
        }

        gBlockNoticeHandle = false;

    }

    clearBuffers();
}

void DeformableBodyVisualizationManagerDeprecated::selectionChanged()
{
    const auto usdContext = omni::usd::UsdContext::getContext();
    const std::vector<SdfPath> selectedPaths = usdContext->getSelection()->getSelectedPrimPathsV2();

    bool needToRedirect = false;
    std::vector<SdfPath> primPaths;
    primPaths.reserve(selectedPaths.size());
    for(const SdfPath path : selectedPaths)
    {
        const auto cit = mSessionToDeformableMap.find(path);
        if(cit != mSessionToDeformableMap.end())
        {
            primPaths.push_back(cit->second);
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


void DeformableBodyVisualizationManagerDeprecated::clear(bool updateSkin)
{
    if (gStage)
    {
        omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());
        for (SdfPathSet::const_iterator cit = mDeformables.cbegin(); cit != mDeformables.cend(); ++cit)
        {
            if (updateSkin)
            {
                // updateSkinVisibility causes flatcache crash on stage close
                updateSkinVisibility(*cit, true);
            }

            SdfPath collisionTetPath = getSessionLayerTetmeshPath(*cit, DeformableBodyVisualizerTypeDeprecated::eCollision);
            SdfPath simulationTetPath = getSessionLayerTetmeshPath(*cit, DeformableBodyVisualizerTypeDeprecated::eSimulation);
            gStage->RemovePrim(collisionTetPath);
            gStage->RemovePrim(simulationTetPath);

            auto iter = mSessionToTetVisualizerMap.find(collisionTetPath);
            if (iter != mSessionToTetVisualizerMap.end())
            {
                if (iter->second)
                    delete iter->second;
                mSessionToTetVisualizerMap.erase(iter->first);
            }

            iter = mSessionToTetVisualizerMap.find(simulationTetPath);
            if (iter != mSessionToTetVisualizerMap.end())
            {
                if (iter->second)
                    delete iter->second;
                mSessionToTetVisualizerMap.erase(iter->first);
            }
        }
    }
    mDeformables.clear();
    mDeformableToSessionTable.clear();
    mSessionToDeformableMap.clear();
    mSessionLayerTetsRoot = SdfPath();
    mVizDeformables.clear();
    mVizSelected.clear();
    mDeformableToScaleMap.clear();
    mDeformableToNumInvMassPointsMap.clear();

    clearBuffers();
}

void DeformableBodyVisualizationManagerDeprecated::release()
{
    clear(false);
}

SdfPath DeformableBodyVisualizationManagerDeprecated::createSessionTetmesh(const SdfPath deformablePath, const Type type)
{
    const SdfPath helperPath = createSessionLayerTetmeshPath(deformablePath, type);
    UsdGeomMesh tetMesh = UsdGeomMesh::Define(gStage, helperPath);
    CARB_ASSERT(tetMesh);
    omni::kit::EditorUsd::setNoDelete(tetMesh.GetPrim(), true);
    omni::kit::EditorUsd::setHideInStageWindow(tetMesh.GetPrim(), true);
    omni::kit::EditorUsd::setNoSelectionOutline(tetMesh.GetPrim(), true);

    TetrahedralMeshVisualizerDeprecated* visualizer = new TetrahedralMeshVisualizerDeprecated(helperPath);
    mSessionToTetVisualizerMap.insert({ helperPath, visualizer });

    // add to map from helper path to deformable path for selection redirection
    mSessionToDeformableMap.insert({ helperPath, deformablePath });

    return helperPath;
}

void DeformableBodyVisualizationManagerDeprecated::updateInvMassPoints(
    UsdGeomPointInstancer& samplePointInstancer,
    const SdfPath deformablePath,
    const PhysxSchemaPhysxDeformableBodyAPI& deformableAPI)
{
    // Get invMassPoints array
    VtArray<GfVec3f> invMassPoints;
    static const TfToken invMassScaleToken("physxDeformable:invMassScale");
    VtArray<float> invMassScale;
    UsdPrim prim = gStage->GetPrimAtPath(deformablePath);
    SafeGetAttributeP(&invMassScale, prim.GetAttribute(invMassScaleToken));

    VtArray<GfVec3f> deformablePoints;
    deformableAPI.GetCollisionPointsAttr().Get(&deformablePoints);

    if (invMassScale.size() != deformablePoints.size())
    {
        invMassScale.assign(deformablePoints.size(), 1.0f);
    }

    for (size_t i = 0; i < deformablePoints.size(); ++i)
    {
        if (invMassScale[i] == 0.0)
        {
            invMassPoints.push_back(deformablePoints[i]);
        }
    }

    size_t numPointSamples = invMassPoints.size();
    if (numPointSamples > 0)
    {
        samplePointInstancer.GetPositionsAttr().Set(invMassPoints);
        VtArray<int> protoIndices(numPointSamples, 0);
        samplePointInstancer.GetProtoIndicesAttr().Set(protoIndices);
    }

    mDeformableToNumInvMassPointsMap[deformablePath] = numPointSamples;
}

void DeformableBodyVisualizationManagerDeprecated::setupSessionPointInstancers(const pxr::SdfPath deformablePath, const float radius)
{
    SdfPath samplePointInstancerPath = getSessionInvMassPointInstancerPath(deformablePath);
    setupSessionPointInstancer(samplePointInstancerPath, kRadiusSample*radius, kColorSample);
}

void DeformableBodyVisualizationManagerDeprecated::updatePointInstancer(const SdfPath deformablePath, const float radius)
{
    if (!isVisible(deformablePath))
        return;

    SdfPath samplePointInstancerPath = getSessionInvMassPointInstancerPath(deformablePath);
    UsdGeomPointInstancer samplePointInstancer = UsdGeomPointInstancer::Get(gStage, samplePointInstancerPath);

    UsdPrim prim = gStage->GetPrimAtPath(deformablePath);
    if (prim)
    {
        updateInvMassPoints(samplePointInstancer, deformablePath,
            PhysxSchemaPhysxDeformableBodyAPI(prim));
    }

    updateSessionPointInstancerRadius(deformablePath, samplePointInstancerPath, kRadiusSample * radius);
}

void DeformableBodyVisualizationManagerDeprecated::createSessionPointInstancer(const pxr::SdfPath pointInstancerPath)
{
    if (pointInstancerPath.IsEmpty())
        return;

    SdfPath pointPrototypePath = pointInstancerPath.AppendElementString(std::string("prototype"));
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

void DeformableBodyVisualizationManagerDeprecated::setupSessionPointInstancer(const pxr::SdfPath pointInstancerPath, const float radius, const pxr::GfVec3f& color)
{
    SdfPath pointPrototypePath = pointInstancerPath.AppendElementString(std::string("prototype"));
    UsdGeomSphere pointPrototype = UsdGeomSphere::Get(gStage, pointPrototypePath);
    pointPrototype.GetRadiusAttr().Set(kPrototypeRadiusZero);
    pointPrototype.GetDisplayColorAttr().Set(VtArray<GfVec3f>(1, color));

    UsdGeomPointInstancer pointInstancer = UsdGeomPointInstancer::Get(gStage, pointInstancerPath);
    pointInstancer.GetPrototypesRel().AddTarget(pointPrototypePath);

    VtArray<GfVec3f> points(1, GfVec3f(0.0f));
    VtArray<int> prototypeIndices(1, 0);
    pointInstancer.GetPositionsAttr().Set(points);
    pointInstancer.GetProtoIndicesAttr().Set(prototypeIndices);
}

void DeformableBodyVisualizationManagerDeprecated::updateSessionPointInstancerRadius(const pxr::SdfPath deformablePath, const pxr::SdfPath pointInstancerPath, const float radius)
{
    SdfPath pointPrototypePath = pointInstancerPath.AppendElementString(std::string("prototype"));
    UsdGeomSphere pointPrototype = UsdGeomSphere::Get(gStage, pointPrototypePath);
    if (pointPrototype)
    {
        const auto cit = mDeformableToNumInvMassPointsMap.find(deformablePath);
        const double prototypeRadiusScale = (cit != mDeformableToNumInvMassPointsMap.end() && cit->second > 0) ? 1.0 : kPrototypeRadiusZero;
        pointPrototype.GetRadiusAttr().Set(double(radius) * prototypeRadiusScale);
    }
}

void DeformableBodyVisualizationManagerDeprecated::updateVisualizationScale(const SdfPath deformablePath)
{
    if (!isVisible(deformablePath))
        return;

    float pointScale = getPointScale(deformablePath);

    SdfPath samplePointInstancerPath = getSessionInvMassPointInstancerPath(deformablePath);
    updateSessionPointInstancerRadius(deformablePath, samplePointInstancerPath, kRadiusSample * pointScale);
}

float DeformableBodyVisualizationManagerDeprecated::getPointScale(const SdfPath deformablePath)
{
    float pointScale = FLT_MAX;
    UsdPrim targetPrim = gStage->GetPrimAtPath(deformablePath);
    const auto cit = mDeformableToScaleMap.find(deformablePath);
    if (cit != mDeformableToScaleMap.end())
    {
        const float deformablePointScale = mDeformableToScaleMap[deformablePath];
        if (targetPrim && deformablePointScale > 0.0f)
        {
            const GfVec3d scale = GfTransform(mXformCache.GetLocalToWorldTransform(targetPrim)).GetScale();
            float maxScaleDim = float(scale[0]);
            maxScaleDim = std::max(maxScaleDim, float(scale[1]));
            maxScaleDim = std::max(maxScaleDim, float(scale[2]));

            pointScale = std::min(maxScaleDim*deformablePointScale, pointScale);
        }
    }

    pointScale = (pointScale == FLT_MAX) ? 0.0f : pointScale;
    return pointScale*mVisualizationScale;
}

float DeformableBodyVisualizationManagerDeprecated::calculateDeformablePointScale(const SdfPath deformablePath)
{
    float pointScale = 0.0;
    //compute volume for scale heuristic
    pxr::VtArray<pxr::GfVec3f> restPoints;
    pxr::VtArray<int32_t> indices;
    UsdPrim deformablePrim = gStage->GetPrimAtPath(deformablePath);
    const PhysxSchemaPhysxDeformableBodyAPI deformableBody(deformablePrim);
    deformableBody.GetCollisionRestPointsAttr().Get(&restPoints);
    deformableBody.GetCollisionIndicesAttr().Get(&indices);
    float scaleVolume = 0.0f;
    bool valid = restPoints.size() > 0 && indices.size() > 0;

    int32_t numPoints = int32_t(restPoints.size());
    for (size_t t = 0; t < indices.size() / 4; ++t)
    {
        int32_t i0 = indices[t * 4 + 0];
        int32_t i1 = indices[t * 4 + 1];
        int32_t i2 = indices[t * 4 + 2];
        int32_t i3 = indices[t * 4 + 3];
        valid &= i0 < numPoints && i1 < numPoints && i2 < numPoints && i3 < numPoints;
        if (valid)
        {
            const GfVec3f& p0 = restPoints[i0];
            const GfVec3f& p1 = restPoints[i1];
            const GfVec3f& p2 = restPoints[i2];
            const GfVec3f& p3 = restPoints[i3];
            float tetVolume = std::abs((p1 - p0) * GfCross(p2 - p0, p3 - p0)) / 6.0f;
            scaleVolume += tetVolume;
        }
    }
    if (valid)
    {
        pointScale = std::pow(scaleVolume / restPoints.size(), 1.0f / 3.0f) * kEdgeToPointScale;
    }
    return pointScale;
}

void DeformableBodyVisualizationManagerDeprecated::setupSessionTetmesh(const SdfPath deformablePath, const Type type)
{
    const SdfPath tetPath = getSessionLayerTetmeshPath(deformablePath, type);
    UsdGeomMesh tetMesh = UsdGeomMesh::Get(gStage, tetPath);

    UsdGeomXformOp op = tetMesh.AddTransformOp();  // just do matrix
                                                   // workaround for hydra update not working if op value is set in later update.
    op.Set(GfMatrix4d(1.0));

    // same workaround for colors and other attributes
    tetMesh.GetPrim().CreateAttribute(visualizationGapAttributeName, SdfValueTypeNames->Float, true).Set(mVisualizationGap);
}

SdfPath DeformableBodyVisualizationManagerDeprecated::getSessionLayerTetmeshPath(const SdfPath deformablePath, const Type type)
{
    const auto it = mDeformableToSessionTable.find(deformablePath);
    CARB_ASSERT(it != mDeformableToSessionTable.end());
    if(type == Type::eCollision)
    {
        return it->second.first;
    }else
    {
        return it->second.second;
    }
}

SdfPath DeformableBodyVisualizationManagerDeprecated::createSessionLayerTetmeshPath(const SdfPath deformablePath, const Type type)
{
    // session layer root is only created if necessary
    if(mSessionLayerTetsRoot.IsEmpty())
    {
        // TODO preist: adapt root based on prim existence?
        mSessionLayerTetsRoot = SdfPath("/PhysxDeformablesVisualizationMeshes");
        CARB_ASSERT(!gStage->GetPrimAtPath(mSessionLayerTetsRoot));
        UsdGeomScope rootScope = UsdGeomScope::Define(gStage, mSessionLayerTetsRoot);
        CARB_ASSERT(rootScope);
        omni::kit::EditorUsd::setNoDelete(rootScope.GetPrim(), true);
        omni::kit::EditorUsd::setHideInStageWindow(rootScope.GetPrim(), true);
    }

    // create the path:
    std::string nameString = deformablePath.GetString();
    std::replace(nameString.begin(), nameString.end(), '/', '_');
    if(type == Type::eCollision)
    {
        nameString.append("_collision");
    }
    else
    {
        nameString.append("_simulation");
    }
    SdfPath path = mSessionLayerTetsRoot.AppendElementString(nameString);

    auto it = mDeformableToSessionTable.find(deformablePath);
    CARB_ASSERT(it != mDeformableToSessionTable.end());
    if(type == Type::eCollision)
    {
        it->second.first = path;
    }
    else
    {
        it->second.second = path;
    }
    return path;
}

void DeformableBodyVisualizationManagerDeprecated::handlePrimResync(const SdfPath path)
{
    UsdPrimRange range(gStage->GetPrimAtPath(path));
    for(pxr::UsdPrimRange::const_iterator cit = range.begin(); cit != range.end(); ++cit)
    {
        const UsdPrim& prim = *cit;
        if (!prim)
            continue;

        if(mDeformables.count(prim.GetPath()))
        {
            mBufferPathsToUpdate.insert(prim.GetPath());
        }
        else if(prim.IsA<UsdGeomMesh>() && prim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>())
        {
            mBufferPathsToAdd.insert(prim.GetPath());
            cit.PruneChildren(); // cannot have a deformable below another deformable in the USD hierarchy
            if (checkDeformableBodyMeshes(prim.GetPath()))
            {
                mBufferPathsToUpdateTopology.insert(prim.GetPath());
            }
        }
    }
}

void DeformableBodyVisualizationManagerDeprecated::handlePrimRemove(const SdfPath path)
{
    const auto iteratorPair = mDeformableToSessionTable.FindSubtreeRange(path);
    for(auto it = iteratorPair.first; it != iteratorPair.second; it++)
    {
        // it->first is a path in the subtree starting at path.
        if(mDeformables.count(it->first))
            mBufferPathsToRemove.insert(it->first);
    }
}

void DeformableBodyVisualizationManagerDeprecated::removeDeformable(const SdfPath deformablePath)
{
    CARB_ASSERT(mDeformables.count(deformablePath));
    mDeformables.erase(deformablePath);
    mVizDeformables.erase(deformablePath);
    mVizSelected.erase(deformablePath);
    mDeformableToScaleMap.erase(deformablePath);
    mDeformableToNumInvMassPointsMap.erase(deformablePath);

    // see if there are any viz helpers for this deformable:
    const auto cit = mDeformableToSessionTable.find(deformablePath);
    if(cit != mDeformableToSessionTable.end())
    {
        if(!cit->second.first.IsEmpty())
        {
            gStage->RemovePrim(cit->second.first);
            mSessionToDeformableMap.erase(cit->second.first);

            TetrahedralMeshVisualizerDeprecated* visualizer = getTetMeshVisualizer(cit->second.first);
            if (visualizer)
                delete visualizer;
            mSessionToTetVisualizerMap.erase(cit->second.first);
        }

        if(!cit->second.second.IsEmpty())
        {
            gStage->RemovePrim(cit->second.second);
            mSessionToDeformableMap.erase(cit->second.second);

            TetrahedralMeshVisualizerDeprecated* visualizer = getTetMeshVisualizer(cit->second.second);
            if (visualizer)
                delete visualizer;
            mSessionToTetVisualizerMap.erase(cit->second.second);
        }

        mDeformableToSessionTable.erase(deformablePath);
    }

    mBufferPathsToUpdateSkinViz.insert(deformablePath);
}

void DeformableBodyVisualizationManagerDeprecated::updateTetVisibility(const SdfPath deformablePath, const SdfPath tetPath, const bool isVisible)
{
    // if it does not exist, no need to update visibility
    if(tetPath.IsEmpty())
        return;

    UsdGeomMesh tetMesh = UsdGeomMesh::Get(gStage, tetPath);

    // handle case where there is no tetmesh
    if(!tetMesh)
        return;

    UsdGeomMesh skinMesh = UsdGeomMesh::Get(gStage, deformablePath);
    if (skinMesh)
    {
        if (skinMesh.ComputeVisibility() == UsdGeomTokens->inherited && isVisible)
        {
            tetMesh.MakeVisible();
            mBufferPathsToUpdateTransform.insert(deformablePath);
            mBufferPathsToUpdateTopology.insert(deformablePath);
            mBufferPathsToUpdateGap.insert(deformablePath);

            const auto usdContext = omni::usd::UsdContext::getContext();
            uint8_t group = isSelected(deformablePath) ? gProxySelectionGroup : 0;
            usdContext->setSelectionGroup(group, tetPath.GetString());
            return;
        }
    }

    tetMesh.MakeInvisible();
}

void DeformableBodyVisualizationManagerDeprecated::updateSkinVisibility(const SdfPath deformablePath, const bool isVisible)
{
    if (gStage)
    {
        UsdGeomMesh skinMesh = UsdGeomMesh::Get(gStage, deformablePath);
        if (skinMesh)
        {
            const TfToken purpose = isVisible ? UsdGeomTokens->default_ : UsdGeomTokens->proxy;
            skinMesh.GetPurposeAttr().Set(purpose);
        }
    }
}

bool DeformableBodyVisualizationManagerDeprecated::isVisible(const SdfPath deformablePath)
{
    return mVizDeformables.find(deformablePath) != mVizDeformables.end();
}

bool DeformableBodyVisualizationManagerDeprecated::isSelected(const SdfPath deformablePath)
{
    return mVizSelected.find(deformablePath) != mVizSelected.end();
}

void DeformableBodyVisualizationManagerDeprecated::handleAttributeChange(const SdfPath path, const TfToken attributeName, const bool isXform)
{
    // xform must always be handled because the change may have happened in the xform hierarchy above the deformable:
    if(isXform)
    {
        // use awesome path table to quickly find all candidate deformable paths to update:
        const auto iteratorPair = mDeformableToSessionTable.FindSubtreeRange(path);
        for(auto it = iteratorPair.first; it != iteratorPair.second; it++)
        {
            // it->first is a path in the subtree starting at path.
            // so if it is a deformable, add it to the xform update buffer
            if(mDeformables.count(it->first))
                mBufferPathsToUpdateTransform.insert(it->first);
        }
        return;
    }

    // same for visibility
    if (attributeName == UsdGeomTokens.Get()->visibility)
    {
        // use awesome path table to quickly find all candidate particles paths to update:
        const auto iteratorPair = mDeformableToSessionTable.FindSubtreeRange(path);
        for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
        {
            // it->first is a path in the subtree starting at path.
            if (mDeformables.count(it->first))
            {
                mBufferVizDirty = true;
            }
        }
        return;
    }

    // otherwise only handle changes that refer to a deformable volume:
    else if(!mDeformables.count(path))
        return;

    // check vert updates:
    else if(attributeName == PhysxSchemaTokens.Get()->physxDeformableCollisionIndices
            || attributeName == PhysxSchemaTokens.Get()->physxDeformableSimulationIndices)
    {
        mBufferPathsToUpdateTopology.insert(path);
    }
    else if (attributeName == PhysxSchemaTokens.Get()->physxDeformableCollisionPoints
             || attributeName == PhysxSchemaTokens.Get()->physxDeformableSimulationPoints)
    {
        mBufferPathsToUpdatePoints.insert(path);
        mBufferPathsToUpdatePointInstancers.insert(path);
    }
    else if (attributeName == invMassScaleAttributeName)
    {
        mBufferPathsToUpdatePointInstancers.insert(path);
    }
}

void DeformableBodyVisualizationManagerDeprecated::updateTetTransform(const SdfPath deformablePath, const SdfPath tetPath)
{
    // get tet prim
    UsdPrim tetPrim = gStage->GetPrimAtPath(tetPath);
    if(!tetPrim)
        return;  // no need to update if there is no tetmesh

    const UsdPrim deformablePrim = gStage->GetPrimAtPath(deformablePath);
    if(!deformablePrim)
        return;  // no need to update if there is no source

    if (!isVisible(deformablePath))
        return;

    // get transformation with cache:
    const GfMatrix4d deformableToWorld = mXformCache.GetLocalToWorldTransform(deformablePrim);
    GfVec3d translation = deformableToWorld.ExtractTranslation();

    UsdAttribute localToWorldAttr = tetPrim.GetAttribute(transformMatrixAttrName);
    CARB_ASSERT(localToWorldAttr);
    localToWorldAttr.Set(deformableToWorld);

    SdfPath samplePointInstancerPath = getSessionInvMassPointInstancerPath(deformablePath);
    updatePrototypeScale(samplePointInstancerPath, deformableToWorld);
}

bool DeformableBodyVisualizationManagerDeprecated::checkDeformableBodyMeshes(const SdfPath deformablePath)
{
    //just check for the presence of simulation indices as the presence of
    //simulation points, collision indices, collision points,
    //follows by design
    PhysxSchemaPhysxDeformableBodyAPI deformableBody = PhysxSchemaPhysxDeformableBodyAPI::Get(gStage, deformablePath);
    if (deformableBody)
    {
        UsdAttribute simIndicesAttr = PhysxSchemaPhysxDeformableAPI(deformableBody).GetSimulationIndicesAttr();
        if (simIndicesAttr.HasAuthoredValue())
        {
            VtArray<int> inds;
            simIndicesAttr.Get(&inds);
            return inds.size() >= 4;
        }
    }
    return false;
}

void DeformableBodyVisualizationManagerDeprecated::updateTetPoints(const SdfPath deformablePath, const SdfPath tetPath, const Type type)
{
    TetrahedralMeshVisualizerDeprecated* visualizer = getTetMeshVisualizer(tetPath);
    if (!visualizer)
        return;

    const PhysxSchemaPhysxDeformableBodyAPI deformableBody = PhysxSchemaPhysxDeformableBodyAPI::Get(gStage, deformablePath);
    if (!deformableBody)
        return;

    if (!isVisible(deformablePath))
        return;

    // get attributes:
    UsdAttribute pointsAttr;
    if(type == Type::eCollision)
    {
        pointsAttr = deformableBody.GetCollisionPointsAttr();
        if (!pointsAttr.HasAuthoredValue())
        {
            pointsAttr = deformableBody.GetCollisionRestPointsAttr();
        }
    }
    else
    {
        pointsAttr = deformableBody.GetSimulationPointsAttr();
        if (!pointsAttr.HasAuthoredValue())
        {
            pointsAttr = deformableBody.GetSimulationRestPointsAttr();
        }
    }

    VtArray<GfVec3f> points;
    pointsAttr.Get(&points);
    if (points.size() >= 4)
    {
        visualizer->setTetPoints(points);
        visualizer->updateTetPoints();
    }
}

void DeformableBodyVisualizationManagerDeprecated::updateTetTopology(const SdfPath deformablePath, const SdfPath tetPath, const Type type)
{
    TetrahedralMeshVisualizerDeprecated* visualizer = getTetMeshVisualizer(tetPath);
    if (!visualizer)
        return;

    const UsdPrim deformablePrim = gStage->GetPrimAtPath(deformablePath);
    if(!deformablePrim)
        return;

    const PhysxSchemaPhysxDeformableAPI deformable(deformablePrim);
    if (!deformable)
        return;

    const PhysxSchemaPhysxDeformableBodyAPI deformableBody(deformablePrim);
    if (!deformableBody)
        return;

    if (!isVisible(deformablePath))
        return;

    bool forceHideProxy;
    bool attachmentsVisOn = mAttachmentsVisualizationManager->needsAttachmentVisualization(deformablePath, &forceHideProxy);
    if (attachmentsVisOn && forceHideProxy)
    {
        UsdGeomMesh tetMesh = UsdGeomMesh::Get(gStage, tetPath);
        tetMesh.MakeInvisible();
        return;
    }

    // get attributes:
    UsdAttribute pointsAttr;
    UsdAttribute indsAttr;
    if (type == Type::eCollision)
    {
        pointsAttr = deformableBody.GetCollisionPointsAttr();
        if (!pointsAttr.HasAuthoredValue())
        {
            pointsAttr = deformableBody.GetCollisionRestPointsAttr();
        }
        indsAttr = deformableBody.GetCollisionIndicesAttr();
    }
    else
    {
        pointsAttr = deformableBody.GetSimulationPointsAttr();
        if (!pointsAttr.HasAuthoredValue())
        {
            pointsAttr = deformableBody.GetSimulationRestPointsAttr();
        }
        indsAttr = deformable.GetSimulationIndicesAttr();
    }

    VtArray<GfVec3f> points;
    VtArray<int32_t> allIndices;
    pointsAttr.Get(&points);
    indsAttr.Get(&allIndices);
    bool valid = points.size() >= 4 && allIndices.size() > 0 && allIndices.size() % 4 == 0;
    int32_t numPoints = int32_t(points.size());
    for (int32_t index : allIndices)
    {
        if (index >= numPoints)
        {
            valid = false;
        }
    }

    if (valid)
    {
        VtArray<int32_t> filterVertIndices;
        if (attachmentsVisOn)
        {
            mAttachmentsVisualizationManager->getFilteredTetIndices(filterVertIndices, deformablePath);
        }

        visualizer->setTetIndices(allIndices);

        GfVec3f color = type == Type::eCollision ? kColorCollision : kColorSimulation;
        VtArray<GfVec3f> colors(allIndices.size(), color);
        for (uint32_t index : filterVertIndices)
        {
            if (index < colors.size())
                colors[index] = kColorFilter;
        }

        visualizer->setTetColors(colors);
        visualizer->setTetPoints(points);
        visualizer->updateTetTopology();
    }
    else
    {
        visualizer->setTetIndices(VtArray<int32_t>());
        visualizer->setTetColors(VtArray<GfVec3f>());
        visualizer->setTetPoints(VtArray<GfVec3f>());
        visualizer->updateTetTopology();
    }
}

void DeformableBodyVisualizationManagerDeprecated::updateTetGap(const SdfPath deformablePath, const SdfPath tetPath)
{
    TetrahedralMeshVisualizerDeprecated* visualizer = getTetMeshVisualizer(tetPath);
    if (!visualizer)
        return;

    // get tet prim
    UsdPrim tetPrim = gStage->GetPrimAtPath(tetPath);
    if(!tetPrim)
        return;  // no need to update if there is no tetmesh

    if (!isVisible(deformablePath))
        return;

    UsdAttribute tetGapAttr = tetPrim.GetAttribute(visualizationGapAttributeName);
    if (tetGapAttr)
    {
        tetGapAttr.Set(mVisualizationGap);
        if (mSessionToTetVisualizerMap.find(tetPath) == mSessionToTetVisualizerMap.end())
            return;

        visualizer->setGapValue(mVisualizationGap);
    }
}

void DeformableBodyVisualizationManagerDeprecated::clearBuffers(void)
{
    mBufferPathsToAdd.clear();
    mBufferPathsToUpdate.clear();
    mBufferPathsToRemove.clear();
    mBufferPathsToUpdateSkinViz.clear();
    mBufferPathsToUpdateTransform.clear();
    mBufferPathsToUpdatePoints.clear();
    mBufferPathsToUpdateTopology.clear();
    mBufferPathsToUpdateGap.clear();
    mBufferPathsToUpdatePointInstancers.clear();
    mBufferVizDirty = false;
}

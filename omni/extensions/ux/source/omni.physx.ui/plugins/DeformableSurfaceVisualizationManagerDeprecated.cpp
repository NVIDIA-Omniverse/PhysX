// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"
#include "DeformableSurfaceVisualizationManagerDeprecated.h"

#include <carb/profiler/Profile.h>

#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/Selection.h>
#include <omni/physx/IPhysxSettings.h>
#include <private/omni/physx/IPhysxUsdLoad.h>
#include <private/omni/physx/IPhysxVisualizationPrivate.h>
#include <omni/renderer/IDebugDraw.h>

#include <algorithm>

using namespace omni::physx::ui;
using namespace pxr;

extern UsdStageRefPtr gStage;
extern carb::settings::ISettings* gSettings;
extern bool gBlockNoticeHandle;
extern uint8_t gProxySelectionGroup;
extern omni::physx::usdparser::IPhysxUsdLoad* gUsdLoad;

static omni::renderer::IDebugDraw* gDebugDraw;
static omni::physx::IPhysxVisualizationPrivate* gPhysXVisualizationPrivate = nullptr;

static const TfToken transformMatrixAttrName{ "xformOp:transform" };
static constexpr float kLineWidth = 2.0f;

DeformableSurfaceVisualizationManagerDeprecated::DeformableSurfaceVisualizationManagerDeprecated(const VisualizerMode mode)
    :mMode(mode), mBufferVizDirty(false)
{
    gPhysXVisualizationPrivate = carb::getCachedInterface<omni::physx::IPhysxVisualizationPrivate>();
    gDebugDraw = carb::getCachedInterface<omni::renderer::IDebugDraw>();
}

DeformableSurfaceVisualizationManagerDeprecated::~DeformableSurfaceVisualizationManagerDeprecated()
{
    release();

    gPhysXVisualizationPrivate = nullptr;
    gDebugDraw = nullptr;
}

void DeformableSurfaceVisualizationManagerDeprecated::setMode(const VisualizerMode mode)
{
    if (mMode != mode)
    {
        if (mMode == VisualizerMode::eNone) // reparse the stage if we are activating the visualizer
            parseStage();
        else if (mode == VisualizerMode::eNone) // release all the buffers if we turn off
            release();
    }

    mMode = mode;
    mBufferVizDirty = true;
}

void DeformableSurfaceVisualizationManagerDeprecated::parseStage()
{
    CARB_PROFILE_ZONE(0, "DeformableSurfaceVisualizationManagerDeprecated::parseStage");

    if (!isActive())
        return;

    handlePrimResync(gStage->GetPseudoRoot().GetPath());
}

void DeformableSurfaceVisualizationManagerDeprecated::update()
{
    CARB_PROFILE_ZONE(0, "DeformableSurfaceVisualizationManagerDeprecated::Update");

    if (!gStage || !isActive())
        return;

    {
        // all updates are to session layer
        omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(gStage, gStage->GetSessionLayer());

        // register new deformable surface paths
        for (SdfPathSet::const_iterator cit = mBufferPathsToAdd.cbegin(); cit != mBufferPathsToAdd.cend(); ++cit)
        {
            mDeformableSurfaces.insert(*cit);
            mDeformableSurfaceToSessionTable.insert({ *cit, pxr::SdfPath() });

            // make sure they are processed in the viz updates:
            mBufferVizDirty = true;
        }

        if (!mDeformableSurfaces.empty())
        {
            if (mBufferVizDirty)
            {
                mVizDeformableSurfaces.clear();
                mVizSelected.clear();

                // update selection if viz dirty
                const auto usdContext = omni::usd::UsdContext::getContext();
                const std::vector<SdfPath> selectedPaths = usdContext->getSelection()->getSelectedPrimPathsV2();

                for (uint32_t s = 0; s < selectedPaths.size(); ++s)
                {
                    // use awesome path table to quickly find all candidate deformable paths to update:
                    const auto iteratorPair = mDeformableSurfaceToSessionTable.FindSubtreeRange(selectedPaths[s]);
                    for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
                    {
                        // it->first is a path in the subtree starting at path s.
                        // so if it is a deformable surface, add it to  update buffer
                        if (mDeformableSurfaces.count(it->first))
                            mVizSelected.insert(it->first);
                    }
                }

                if (mMode == VisualizerMode::eSelected)
                {
                    mVizDeformableSurfaces = mVizSelected;
                }
                else if (mMode == VisualizerMode::eAll)
                {
                    mVizDeformableSurfaces = mDeformableSurfaces;
                }
            }
     
            // Do all attribute updates and prim deletions in Sdf changeblock:
            {
                SdfChangeBlock block;

                // delete removed deformables:
                for (SdfPathSet::const_iterator cit = mBufferPathsToRemove.cbegin(); cit != mBufferPathsToRemove.cend();
                    ++cit)
                {
                    removeDeformable(*cit);
                }
            }
        }
    }

    clearBuffers();
}

uint32_t convertColor(uint32_t inColor);

void DeformableSurfaceVisualizationManagerDeprecated::draw()
{
    CARB_PROFILE_ZONE(0, "DeformableSurfaceVisualizationManagerDeprecated::Update");

    if (!gDebugDraw || !isActive())
        return;

    for (SdfPathSet::const_iterator cit = mVizDeformableSurfaces.cbegin(); cit != mVizDeformableSurfaces.cend(); ++cit)
    {
        SdfPath clothPath = *cit;
        UsdGeomMesh mesh = UsdGeomMesh::Get(gStage, clothPath);
        if (!mesh)
            return;

        VtVec3fArray points;
        mesh.GetPointsAttr().Get(&points);

        UsdGeomXform xform = UsdGeomXform::Get(gStage, clothPath);
        pxr::GfMatrix4f localToWorld = pxr::GfMatrix4f(xform.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default()));

        pxr::VtArray<uint32_t> vertices;
        gPhysXVisualizationPrivate->getFEMClothDebugDraw(vertices, clothPath);

        if (vertices.empty())
            return;

        for (uint32_t i = 0; i < vertices.size(); i += 2)
        {
            GfVec3f p1 = localToWorld.Transform(points[vertices[i]]);
            GfVec3f p2 = localToWorld.Transform(points[vertices[i + 1]]);

            // same color as particle cloth
            gDebugDraw->drawLine(
                carb::Float3{ p1[0], p1[1], p1[2] }, convertColor(0xBB4444FF), kLineWidth,
                carb::Float3{ p2[0], p2[1], p2[2] }, convertColor(0xBB4444FF), kLineWidth);
        }
    }
}

void DeformableSurfaceVisualizationManagerDeprecated::selectionChanged()
{
    mBufferVizDirty = true;
}

void DeformableSurfaceVisualizationManagerDeprecated::release()
{
    mDeformableSurfaces.clear();
    mDeformableSurfaceToSessionTable.clear();
    mVizDeformableSurfaces.clear();
    mVizSelected.clear();

    clearBuffers();
}

void DeformableSurfaceVisualizationManagerDeprecated::handlePrimResync(const SdfPath path)
{
    UsdPrimRange range(gStage->GetPrimAtPath(path));
    for (pxr::UsdPrimRange::const_iterator cit = range.begin(); cit != range.end(); ++cit)
    {
        const UsdPrim& prim = *cit;
        if (!prim)
            continue;

        if (!mDeformableSurfaces.count(prim.GetPath()) && prim.IsA<UsdGeomMesh>() && prim.HasAPI<PhysxSchemaPhysxDeformableSurfaceAPI>())
        {
            mBufferPathsToAdd.insert(prim.GetPath());
            cit.PruneChildren(); // cannot have a deformable below another deformable in the USD hierarchy
        }
    }
}

void DeformableSurfaceVisualizationManagerDeprecated::handlePrimRemove(const SdfPath path)
{
    const auto iteratorPair = mDeformableSurfaceToSessionTable.FindSubtreeRange(path);
    for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
    {
        // it->first is a path in the subtree starting at path.
        if (mDeformableSurfaces.count(it->first))
            mBufferPathsToRemove.insert(it->first);
    }
}

void DeformableSurfaceVisualizationManagerDeprecated::removeDeformable(const SdfPath deformablePath)
{
    mDeformableSurfaces.erase(deformablePath);
    mVizDeformableSurfaces.erase(deformablePath);
    mVizSelected.erase(deformablePath);

    const auto cit = mDeformableSurfaceToSessionTable.find(deformablePath);
    if (cit != mDeformableSurfaceToSessionTable.end())
    {
        if (!cit->second.IsEmpty())
        {
            gStage->RemovePrim(cit->second);
        }

        mDeformableSurfaceToSessionTable.erase(deformablePath);
    }
}

void DeformableSurfaceVisualizationManagerDeprecated::handleAttributeChange(const SdfPath path,
    const TfToken attributeName, const bool isXform)
{
    return;
}

void DeformableSurfaceVisualizationManagerDeprecated::clearBuffers(void)
{
    mBufferPathsToAdd.clear();
    mBufferPathsToRemove.clear();
    mBufferVizDirty = false;
}

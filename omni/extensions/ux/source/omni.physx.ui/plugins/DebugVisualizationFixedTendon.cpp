// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/Types.h>
#include <carb/profiler/Profile.h>
#include <private/omni/physx/IPhysxUsdLoad.h>
#include <omni/physx/IPhysx.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/Selection.h>
#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>

#include <common/ui/ImguiDrawingUtils.h>
#include "DebugVisualizationFixedTendon.h"
#include "DebugVisualizationTendonCommon.h"

extern omni::physx::usdparser::IPhysxUsdLoad* gUsdLoad;
extern carb::settings::ISettings* gSettings;
extern pxr::UsdStageRefPtr gStage;
extern omni::physx::IPhysx* gPhysX;

static const uint32_t gCyanColor = 0xDB808010;
static const uint32_t gYellowColor = 0xDB108080;
static const uint32_t gRedColor = 0xDB101080;
static const uint32_t gTesselation = 12;
static const float gLineThickness = 2.0f;
static const float gCircleThickness = 2.0f;
static const float gCircleRadius = 1.0f;

using namespace pxr;
using namespace omni::physics::ui;
using omni::physx::usdparser::PhysxTendonAxisHierarchyDesc;


namespace omni
{
namespace physx
{
namespace ui
{

    FixedTendonVisualizer::FixedTendonVisualizer(VisualizerMode mode) : mMode(mode), mDirty(false)
    {
    }

    FixedTendonVisualizer::~FixedTendonVisualizer()
    {
        release();
    }

    void FixedTendonVisualizer::update()
    {
        CARB_PROFILE_ZONE(0, "FixedTendonVisualizer::Update");

        // do not process updates if not visible
        if (gPhysX->isRunning() || !isActive())
        {
            return;
        }

        if (mDirty)
        {
            parseStage(); // parse whole stage for now
            mDirty = false;
        }
    }

    void FixedTendonVisualizer::parseStage()
    {
        CARB_PROFILE_ZONE(0, "FixedTendonVisualizer::parseStage");

        // early exit if not active / being used
        if (!isActive())
        {
            return;
        }
        release();
        if (gStage)
        {
            const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(gStage).ToLongInt();
            mTendons = gUsdLoad->parseFixedTendons(stageId, gStage->GetPseudoRoot().GetPath());
            recomputeTendonsPathSet();
            recomputeDrawList();
        }
    }

    void FixedTendonVisualizer::recomputeTendonsPathSet()
    {
        mTendonsPathSet.clear();

        std::function<void(const PhysxTendonAxisHierarchyDesc*)> recomputeRecurse;
        recomputeRecurse = [&](const PhysxTendonAxisHierarchyDesc* axis)
        {
            mTendonsPathSet.insert(axis->jointPath);

            for (const PhysxTendonAxisHierarchyDesc* child : axis->children)
            {
                recomputeRecurse(child);
            }
        };

        for (const PhysxTendonAxisHierarchyDesc* axis : mTendons)
        {
            recomputeRecurse(axis);
        }
    }

    void FixedTendonVisualizer::selectionChanged()
    {
        // early exit if not active / being used
        if (!isActive())
        {
            return;
        }
        mInstanceFilter = TfToken();
        mSelectedPaths.clear();
        const std::vector<std::string> selectedPaths =
            omni::usd::UsdContext::getContext()->getSelection()->getSelectedPrimPaths();

        mSelectedPaths.reserve(selectedPaths.size());
        for (const std::string& path : selectedPaths)
        {
            mSelectedPaths.push_back(pxr::SdfPath(path));
        }

        recomputeDrawList();
    }

    bool FixedTendonVisualizer::isSelected(const pxr::SdfPath pathToCheck) const
    {
        for (const pxr::SdfPath path : mSelectedPaths)
        {
            if (pathToCheck.HasPrefix(path))
                return true;
        }

        return false;
    }

    void FixedTendonVisualizer::recomputeDrawList()
    {
        mSelectedTendons.clear();

        for (PhysxTendonAxisHierarchyDesc* axis : mTendons)
        {
            if (shouldDrawTendon(axis))
            {
                mSelectedTendons.push_back(axis);
                continue;
            }

            for (PhysxTendonAxisHierarchyDesc* child : axis->children)
            {
                if (recomputeDrawListRecursive(child))
                {
                    mSelectedTendons.push_back(axis);
                    break; // don't enter tendon into draw list more than once
                }
            }
        }
    }

    void FixedTendonVisualizer::draw(const pxr::GfMatrix4d& viewMatrix,
        const pxr::GfMatrix4d& projMatrix,
        const carb::Float4& viewPortRect) const
    {
        if (mMode == VisualizerMode::eNone)
        {
            return;
        }

        bool wantDrawAll = (mMode == VisualizerMode::eAll) && mInstanceFilter.IsEmpty();
        const TDescVec& drawList = wantDrawAll ? mTendons : mSelectedTendons;

        if (drawList.size() == 0)
        {
            return;
        }

        const GfMatrix4d viewProjection = viewMatrix * projMatrix;

        float radius;
        if (gSettings->getAsBool(kViewportGizmoConstantScaleEnabledPath))
        {
            radius = gCircleRadius * gSettings->getAsFloat(kViewportGizmoConstantScalePath);
        }
        else
        {
            radius = gCircleRadius * gSettings->getAsFloat(kViewportGizmoScalePath) /
                static_cast<float>(pxr::UsdGeomGetStageMetersPerUnit(gStage));
        }

        for (const PhysxTendonAxisHierarchyDesc* axis : drawList)
        {
            // mark axis as red if unused, yellow if root
            const uint32_t color = axis->type == omni::physx::usdparser::eTendonAxisUI ? gRedColor : gYellowColor;

            drawRecursive(axis, viewProjection, viewPortRect, radius, color, false, GfVec3d(0.0));
        }
    }

    void FixedTendonVisualizer::drawRecursive(const PhysxTendonAxisHierarchyDesc* axis,
        const pxr::GfMatrix4d& viewProjection,
        const carb::Float4& viewPortRect,
        const float radius,
        const uint32_t color,
        const bool drawLineToParent,
        const pxr::GfVec3d& parentCenter) const
    {
        const GfMatrix4d source = GetJointPosition(gStage, axis->jointPath);
        const GfVec3d center = source.ExtractTranslation();

        if (drawLineToParent)
        {
            addLine(parentCenter, center, viewProjection, viewPortRect, color, gLineThickness);
        }

        if (isSelected(axis->jointPath))
        {
            if (isRotationAxis(axis->axes[0]))
                addFilledCircle(center, viewProjection, viewPortRect, color, gTesselation, radius);
            else
                addFilledRect(center, viewProjection, viewPortRect, color, radius);
        }
        else
        {
            if (isRotationAxis(axis->axes[0]))
                addCircle(center, viewProjection, viewPortRect, color, gTesselation, radius, gCircleThickness);
            else
                addRect(center, viewProjection, viewPortRect, color, radius, gLineThickness);
        }

        for (PhysxTendonAxisHierarchyDesc* child : axis->children)
        {
            drawRecursive(child, viewProjection, viewPortRect, radius, gCyanColor, true, center);
        }
    }

    // returns true if the axis subtree has at least one axis that is parented by the selection path
    bool FixedTendonVisualizer::recomputeDrawListRecursive(const PhysxTendonAxisHierarchyDesc* axis)
    {
        if (shouldDrawTendon(axis))
        {
            return true;
        }

        for (PhysxTendonAxisHierarchyDesc* child : axis->children)
        {
            if (recomputeDrawListRecursive(child))
            {
                return true;
            }
        }

        return false;
    }

    bool FixedTendonVisualizer::shouldDrawTendon(const PhysxTendonAxisHierarchyDesc* axis)
    {
        if (!mInstanceFilter.IsEmpty())
        {
            return axis->instanceToken == mInstanceFilter;
        }
        else
        {
            return isSelected(axis->jointPath) || isSelected(axis->link0) || isSelected(axis->link1);
        }    
    }

    bool FixedTendonVisualizer::isRotationAxis(const omni::physx::usdparser::JointAxis axis) const
    {
        switch (axis)
        {
        case omni::physx::usdparser::JointAxis::eRotX:
        case omni::physx::usdparser::JointAxis::eRotY:
        case omni::physx::usdparser::JointAxis::eRotZ:
        {
            return true;
        }
        default:
        {
            return false;
        }
        }
    }

    void FixedTendonVisualizer::releaseRecursive(PhysxTendonAxisHierarchyDesc* desc)
    {
        for (PhysxTendonAxisHierarchyDesc* child : desc->children)
        {
            releaseRecursive(child);
        }

        gUsdLoad->releaseDesc(desc);
    }

    void FixedTendonVisualizer::release()
    {
        mSelectedTendons.clear();

        //enough to clear everything once
        for (PhysxTendonAxisHierarchyDesc* desc : mTendons)
        {
            releaseRecursive(desc);
        }
        mTendons.clear();
        mDirty = false;
    }

    void FixedTendonVisualizer::setInstanceFilter(const char* instanceName)
    {
        mInstanceFilter = TfToken(instanceName);
        recomputeDrawList();
    }

    void FixedTendonVisualizer::setMode(VisualizerMode mode)
    {
        mMode = mode;
        if (isActive())
        {
            // trigger parse if now visible
            mDirty = true;
            // and make sure selection is up-to-date
            selectionChanged();
        }
    }

    bool FixedTendonVisualizer::isEmpty()
    {
        return mTendons.empty();
    }

    bool FixedTendonVisualizer::isDirty()
    {
        return mDirty;
    }

    void FixedTendonVisualizer::setDirty()
    {
        mDirty = true;
    }

    bool FixedTendonVisualizer::hasTendon(const pxr::SdfPath path)
    {
        return mTendonsPathSet.find(path) != mTendonsPathSet.end();
    }

    } // namespace ui
    } // namespace physx
} // namespace omni

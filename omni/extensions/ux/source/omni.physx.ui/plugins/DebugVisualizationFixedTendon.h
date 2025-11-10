// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "UsdPCH.h"

#include <private/omni/physx/PhysxUsd.h>
#include <private/omni/physx/ui/VisualizerMode.h>

namespace omni
{
namespace physx
{
namespace ui
{
class FixedTendonVisualizer
{
    using TDescVec = std::vector<omni::physx::usdparser::PhysxTendonAxisHierarchyDesc*>;
    using TPathSet = std::unordered_set<pxr::SdfPath, pxr::SdfPath::Hash>;

public:
    FixedTendonVisualizer(VisualizerMode mode);
    ~FixedTendonVisualizer();

    // draws the visualization overlay. Called on every frame.
    void draw(const pxr::GfMatrix4d& viewMatrix,
              const pxr::GfMatrix4d& projMatrix,
              const carb::Float4& viewPortRect) const;

    // notifies the visualization that the USD has been updated
    void update();

    // forces a reparse of the whole USD stage
    void parseStage();

    // notifies the visualization that the user prim selection changed
    void selectionChanged();

    // clears all data structures and frees the associated memory
    void release();

    // visualize only tendons of this instanceName, empty string shows all (default)
    void setInstanceFilter(const char* instanceName);

    // visualization mode
    void setMode(VisualizerMode mode);

    // helpers for usd notice handling
    bool isEmpty();
    bool isDirty();
    void setDirty();
    bool hasTendon(const pxr::SdfPath path);
    bool isActive() const
    {
        return mMode != VisualizerMode::eNone;
    };

private:
    // these methods compute which tendons need to be drawn based on user selection
    void recomputeDrawList();
    bool recomputeDrawListRecursive(const omni::physx::usdparser::PhysxTendonAxisHierarchyDesc* axis);
    bool shouldDrawTendon(const omni::physx::usdparser::PhysxTendonAxisHierarchyDesc* axis);

    // helper functions
    bool isSelected(const pxr::SdfPath pathToCheck) const;
    bool isRotationAxis(const omni::physx::usdparser::JointAxis axis) const;

    // see draw()
    void drawRecursive(const omni::physx::usdparser::PhysxTendonAxisHierarchyDesc* axis,
                       const pxr::GfMatrix4d& viewProjection,
                       const carb::Float4& viewPortRect,
                       const float radius,
                       const uint32_t color,
                       const bool drawLineToParent,
                       const pxr::GfVec3d& parentCenter) const;

    // see release()
    void releaseRecursive(omni::physx::usdparser::PhysxTendonAxisHierarchyDesc* desc);

    void recomputeTendonsPathSet();

    // data structure to save all fixed tendon hierarchies in the stage
    TDescVec mTendons;
    TPathSet mTendonsPathSet;

    // data structures to keep track of which tendons are selected/should be displayed
    std::vector<pxr::SdfPath> mSelectedPaths;
    TDescVec mSelectedTendons;

    VisualizerMode mMode;
    bool mDirty;
    pxr::TfToken mInstanceFilter;
};

} // namespace ui
} // namespace physx
} // namespace omni

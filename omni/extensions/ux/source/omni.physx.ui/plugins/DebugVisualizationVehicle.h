// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once
#include "UsdPCH.h"
#include "DescCache.h"
#include <omni/renderer/IDebugDraw.h>
#include <private/omni/physx/IPhysxUsdLoad.h>
#include <omni/physx/IPhysxUI.h>

namespace omni
{
namespace physx
{
namespace ui
{


typedef pxr::TfHashSet<pxr::SdfPath, pxr::SdfPath::Hash> PathSet;


class DebugVisualizationVehicle
{
public:
    DebugVisualizationVehicle(pxr::UsdGeomXformCache&);
    ~DebugVisualizationVehicle();

    uint32_t getVisualizationFlags() const
    {
        return mVisualizationFlags;
    }

    void setVisualization(PhysXVehicleVisualizationParameter::Enum, bool enable);
    bool getVisualization(PhysXVehicleVisualizationParameter::Enum) const;

    void updateVehicle(const pxr::UsdPrim&, pxr::UsdStageWeakPtr);

    void updateTrackedVehicles(pxr::UsdStageWeakPtr);

    bool isXformPathTracked(const pxr::SdfPath&);

    // Clear caches and releases buffers
    void clear(bool resetDrawBuffersInsteadOfRelease);

    void releaseDrawBuffersIfEmpty();

private:
    void createLineBuffer();
    void releaseLineBuffer();


private:
    pxr::UsdGeomXformCache& mXformCache;
    PathSet mVehiclePathCache; // Cache for vehicle paths
    PathSet mVehicleWheelPathCache; // Cache for vehicle wheel paths
    DescCache mDescCache; // cache for vehicle and component descriptors
    omni::renderer::IDebugDraw* mDebugDraw;

    usdparser::VehicleComponentTrackerHandle mVehicleComponentTrackerHandle;

    omni::renderer::SimplexBuffer mLineBuffer;
    omni::renderer::RenderInstanceBuffer mLineRenderInstanceBuffer;
    size_t mLineIndex;
    size_t mMaxLineBufferSize;

    uint32_t mVisualizationFlags;
};

} // namespace ui
} // namespace physx
} // namespace omni

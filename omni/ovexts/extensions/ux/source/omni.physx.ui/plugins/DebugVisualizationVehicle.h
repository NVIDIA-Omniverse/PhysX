// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


typedef PXR_NS::TfHashSet<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash> PathSet;


class DebugVisualizationVehicle
{
public:
    DebugVisualizationVehicle(PXR_NS::UsdGeomXformCache&);
    ~DebugVisualizationVehicle();

    uint32_t getVisualizationFlags() const
    {
        return mVisualizationFlags;
    }

    void setVisualization(PhysXVehicleVisualizationParameter::Enum, bool enable);
    bool getVisualization(PhysXVehicleVisualizationParameter::Enum) const;

    void updateVehicle(const PXR_NS::UsdPrim&, PXR_NS::UsdStageWeakPtr);

    void updateTrackedVehicles(PXR_NS::UsdStageWeakPtr);

    bool isXformPathTracked(const PXR_NS::SdfPath&);

    // Clear caches and releases buffers
    void clear(bool resetDrawBuffersInsteadOfRelease);

    void releaseDrawBuffersIfEmpty();

private:
    void createLineBuffer();
    void releaseLineBuffer();


private:
    PXR_NS::UsdGeomXformCache& mXformCache;
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

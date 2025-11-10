// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once
#include "UsdPCH.h"
#include "DescCache.h"
#include <omni/renderer/IDebugDraw.h>
#include <omni/physx/IPhysxVisualization.h>

#include <vector>
#include <string>

// Helper class to manage solid shaded debug visualization
// of collision meshes by creating renderable meshes in
// the session layer and synchronizing their transforms

namespace omni
{
namespace physx
{
namespace ui
{

enum class CollisionMeshDisplayMode
{
    BOTH,
    GRAPHICS,
    COLLISION
};

class DebugVisualizationCollisionMesh
{
public:
    static DebugVisualizationCollisionMesh* create(pxr::UsdGeomXformCache& xformcache);

    virtual void updateDebugVisualization(void) = 0;

    virtual bool refreshSelectionSet(void) = 0;

    virtual bool releaseDescCache(const pxr::SdfPath& path) = 0;

    virtual void setCollisionMeshType(const char* type) = 0;

    virtual void enableCollisionMeshVisualization(bool enable) = 0;

    virtual void explodeViewDistance(float distance) = 0;

    virtual bool isEnabled(void) const = 0;

    virtual bool isXformPathTracked(const pxr::SdfPath& path) const = 0;

    virtual void release(void) = 0;

    virtual bool wantsForcedUpdate(void) const = 0;

protected:
    virtual ~DebugVisualizationCollisionMesh(void){};
};

} // namespace ui
} // namespace physx
} // namespace omni

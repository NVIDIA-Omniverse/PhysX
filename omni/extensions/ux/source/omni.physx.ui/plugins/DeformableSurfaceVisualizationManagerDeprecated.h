// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "UsdPCH.h"
#include "DescCache.h"

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/PhysxUsd.h>
#include <private/omni/physx/ui/VisualizerMode.h>
#include <private/omni/physics/schema/IUsdPhysics.h>

namespace omni
{

namespace physx
{

namespace ui
{

class DeformableSurfaceVisualizationManagerDeprecated
{
public:
    explicit DeformableSurfaceVisualizationManagerDeprecated(const VisualizerMode mode);
    ~DeformableSurfaceVisualizationManagerDeprecated();

    void handlePrimResync(const pxr::SdfPath path);
    void handlePrimRemove(const pxr::SdfPath path);
    void handleAttributeChange(const pxr::SdfPath path, const pxr::TfToken attributeName, const bool isXform);

    void parseStage();
    void update();
    void release();
    void selectionChanged();

    void setMode(const VisualizerMode mode);

    bool isEmpty() const
    {
        return mDeformableSurfaces.empty();
    }
    bool isActive() const
    {
        return mMode != VisualizerMode::eNone;
    }

    // draw triangle edge visualization
    void draw();

private:
    void removeDeformable(const pxr::SdfPath deformablePath);
    void clearBuffers(void);

    // class-scope using:
    using SdfPathToSdfPathMap = pxr::TfHashMap<pxr::SdfPath, pxr::SdfPath, pxr::SdfPath::Hash>;
    using SdfPathToSdfPathTable = pxr::SdfPathTable<pxr::SdfPath>;

    // internal members:
    pxr::SdfPathSet mDeformableSurfaces; // path set of deformables on stage (for quick checks if a path is a deformable
                                         // in callbacks)
    SdfPathToSdfPathTable mDeformableSurfaceToSessionTable; // path table that maps from stage deformable to session
                                                            // triangles

    VisualizerMode mMode; // mode for visualization [none, all, selected]
    pxr::SdfPathSet mVizDeformableSurfaces; // all deformables that need to be visualized
    pxr::SdfPathSet mVizSelected; // all deformables that need to be visualized and are selected

    // buffers to store updates until next stage update that calls the manager's update function
    pxr::SdfPathSet mBufferPathsToAdd;
    pxr::SdfPathSet mBufferPathsToRemove;
    bool mBufferVizDirty; // set to true whenever the set of triangle meshes to visualize changes
};

} // namespace ui
} // namespace physx
} // namespace omni

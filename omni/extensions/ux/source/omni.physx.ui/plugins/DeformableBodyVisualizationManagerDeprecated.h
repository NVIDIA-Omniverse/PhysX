// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "UsdPCH.h"

#include <private/omni/physx/ui/VisualizerMode.h>

namespace omni
{

namespace physx
{

namespace ui
{

enum struct DeformableBodyVisualizerTypeDeprecated : char
{
    eSimulation = 0,
    eCollision = 1
};

class DeformableBodyVisualizationManagerDeprecated
{
public:
    explicit DeformableBodyVisualizationManagerDeprecated(const VisualizerMode mode,
                                                          const DeformableBodyVisualizerTypeDeprecated type);
    ~DeformableBodyVisualizationManagerDeprecated();

    void setAttachmentsVisualizationManager(class AttachmentsVisualizationManagerDeprecated& attachmentVisualizationManager);

    void handlePrimResync(const pxr::SdfPath path);
    void handlePrimRemove(const pxr::SdfPath path);
    void handleAttributeChange(const pxr::SdfPath path, const pxr::TfToken attributeName, const bool isXform);

    void parseStage();
    void update();
    void release();
    void selectionChanged();

    void setMode(const VisualizerMode mode);
    void setType(const DeformableBodyVisualizerTypeDeprecated type);

    bool isEmpty() const
    {
        return mDeformables.empty();
    }
    bool isActive() const;

    void notifyAttachmentVisualization(bool deformableResync, bool deformableRelease);
    DeformableBodyVisualizerTypeDeprecated getDisplayType(const pxr::SdfPath deformablePath);

private:
    typedef DeformableBodyVisualizerTypeDeprecated Type;

    // functions
    pxr::SdfPath getSessionLayerTetmeshPath(const pxr::SdfPath deformablePath, const Type type);
    pxr::SdfPath createSessionLayerTetmeshPath(const pxr::SdfPath deformablePath, const Type type);
    pxr::SdfPath createSessionTetmesh(const pxr::SdfPath deformablePath, const Type type);
    void setupSessionTetmesh(const pxr::SdfPath deformablePath, const Type type);
    void removeDeformable(const pxr::SdfPath deformablePath);
    void updateTetTransform(const pxr::SdfPath deformablePath, const pxr::SdfPath tetPath);
    void updateTetPoints(const pxr::SdfPath deformablePath, const pxr::SdfPath tetPath, const Type type);
    void updateTetTopology(const pxr::SdfPath deformablePath, const pxr::SdfPath tetPath, const Type type);
    void updateTetGap(const pxr::SdfPath deformablePath, const pxr::SdfPath tetPath);
    void updateTetVisibility(const pxr::SdfPath deformablePath, const pxr::SdfPath tetPath, const bool isVisible);
    void updateSkinVisibility(const pxr::SdfPath deformablePath, const bool isVisible);
    bool isVisible(const pxr::SdfPath deformablePath);
    bool isSelected(const pxr::SdfPath deformablePath);
    bool checkDeformableBodyMeshes(const pxr::SdfPath deformablePath);
    void clear(bool updateSkin);
    void clearBuffers(void);
    void setupSessionPointInstancers(const pxr::SdfPath deformablePath, const float radius);
    void updatePointInstancer(const pxr::SdfPath deformablePath, const float radius);
    void updateInvMassPoints(pxr::UsdGeomPointInstancer& samplePointInstancer,
                             const pxr::SdfPath deformablePath,
                             const pxr::PhysxSchemaPhysxDeformableBodyAPI& deformableAPI);
    void createSessionPointInstancer(const pxr::SdfPath sessionPointInstancerPath);
    void setupSessionPointInstancer(const pxr::SdfPath pointInstancerPath, const float radius, const pxr::GfVec3f& color);
    void updateSessionPointInstancerRadius(const pxr::SdfPath deformablePath,
                                           const pxr::SdfPath pointInstancerPath,
                                           const float radius);
    void updateVisualizationScale(const pxr::SdfPath deformablePath);
    float getPointScale(const pxr::SdfPath deformablePath);
    float calculateDeformablePointScale(const pxr::SdfPath deformablePath);
    pxr::SdfPath getSessionInvMassPointInstancerPath(const pxr::SdfPath deformablePath);
    class TetrahedralMeshVisualizerDeprecated* getTetMeshVisualizer(const pxr::SdfPath sessionTetPath);

    // class-scope using:
    using SdfPathToSdfPathMap = pxr::TfHashMap<pxr::SdfPath, pxr::SdfPath, pxr::SdfPath::Hash>;
    using SdfPathPair = std::pair<pxr::SdfPath, pxr::SdfPath>;
    using SdfPathToPairOfSdfPathsTable = pxr::SdfPathTable<SdfPathPair>;
    using SdfPathToPointScaleMap = pxr::TfHashMap<pxr::SdfPath, float, pxr::SdfPath::Hash>;
    using SdfPathToTetMeshVisualizerMap =
        pxr::TfHashMap<pxr::SdfPath, TetrahedralMeshVisualizerDeprecated*, pxr::SdfPath::Hash>;
    using SdfPathToNumInvMassPointsMap = pxr::TfHashMap<pxr::SdfPath, size_t, pxr::SdfPath::Hash>;

    // internal members:
    pxr::SdfPathSet mDeformables; // path set of deformables on stage (for quick checks if a path is a deformable in
                                  // callbacks)
    SdfPathToPairOfSdfPathsTable mDeformableToSessionTable; // path table that maps from stage deformable to session
                                                            // tets
    SdfPathToSdfPathMap mSessionToDeformableMap; // path hashmap from session tets to deformable path (for selection
                                                 // swapping)
    pxr::SdfPath mSessionLayerTetsRoot; // path to session layer scope "folder" for session tets
    VisualizerMode mMode; // mode for visualization [none, all, selected]
    Type mType; // type of mesh to visualize [simulation, collision]
    pxr::SdfPathSet mVizDeformables; // all deformables that need to be visualized
    pxr::SdfPathSet mVizSelected; // all deformables that need to be visualized and are selected
    float mVisualizationGap;
    float mVisualizationScale;
    SdfPathToPointScaleMap mDeformableToScaleMap; // path hashmap from deformable path to point scales
    SdfPathToTetMeshVisualizerMap mSessionToTetVisualizerMap; // path hashmap from session tets to tet mesh visualizers
    SdfPathToNumInvMassPointsMap mDeformableToNumInvMassPointsMap; // path hashmap from deformable path to the size of
                                                                   // inverse mass points array

    // buffers to store updates until next stage update that calls the manager's update function
    pxr::SdfPathSet mBufferPathsToAdd;
    pxr::SdfPathSet mBufferPathsToUpdate;
    pxr::SdfPathSet mBufferPathsToRemove;
    pxr::SdfPathSet mBufferPathsToUpdateSkinViz;
    pxr::SdfPathSet mBufferPathsToUpdateTransform;
    pxr::SdfPathSet mBufferPathsToUpdatePoints;
    pxr::SdfPathSet mBufferPathsToUpdateTopology;
    pxr::SdfPathSet mBufferPathsToUpdateGap;
    pxr::SdfPathSet mBufferPathsToUpdatePointInstancers;
    bool mBufferVizDirty; // set to true whenever the set of tet meshes to visualize changes

    // xform cache:
    pxr::UsdGeomXformCache mXformCache;

    class AttachmentsVisualizationManagerDeprecated* mAttachmentsVisualizationManager;
};

} // namespace ui
} // namespace physx
} // namespace omni

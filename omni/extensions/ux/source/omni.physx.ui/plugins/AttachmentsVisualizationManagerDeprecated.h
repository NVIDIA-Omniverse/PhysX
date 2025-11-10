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

class AttachmentsVisualizationManagerDeprecated
{
public:
    explicit AttachmentsVisualizationManagerDeprecated(const VisualizerMode mode,
                                                       const bool hideActor0,
                                                       const bool hideActor1);
    ~AttachmentsVisualizationManagerDeprecated();

    void setDeformableBodyVisualizationManager(
        class DeformableBodyVisualizationManagerDeprecated& deformableBodyVisualizationManager);
    bool needsAttachmentVisualization(pxr::SdfPath deformablePath, bool* forceHide = nullptr);
    void getFilteredTetIndices(pxr::VtArray<int32_t>& indices, const pxr::SdfPath deformablePath);

    void handlePrimResync(const pxr::SdfPath path);
    void handlePrimRemove(const pxr::SdfPath path);
    void handleAttributeChange(const pxr::SdfPath path, const pxr::TfToken attributeName, const bool isXform);

    void parseStage();
    void update();
    void release();
    void selectionChanged();

    void setMode(const VisualizerMode mode);
    bool hasUserAttachments() const
    {
        return mUserVizAttachments.size() > 0;
    }
    void setHideActor0(const bool hideActor0);
    void setHideActor1(const bool hideActor1);

    void setUserAttachment(pxr::SdfPath attachmentPath, bool enable);
    void hideUserAttachmentActor(pxr::SdfPath attachmentPath, pxr::SdfPath actorPath, bool hide);

    bool isEmpty() const
    {
        return mAttachments.empty();
    }
    bool isActive() const;

private:
    struct AttachmentInfo
    {
        pxr::SdfPath targets[2] = { pxr::SdfPath(), pxr::SdfPath() };
        uint32_t numPointSamples[2] = { 0, 0 };
        uint32_t numPointFilters[2] = { 0, 0 };
        pxr::VtArray<int32_t> deformableSampleTetIds[2];
        pxr::VtArray<pxr::GfVec4f> deformableSampleBarycentrics[2];
        pxr::VtArray<uint32_t> deformableFilterTetIds[2];
        pxr::VtArray<int32_t> clothSampleIndices[2];
        pxr::VtArray<uint32_t> clothFilterIndices[2];
    };

    struct TargetInfo
    {
        enum Type
        {
            eParticleCloth,
            eDeformableBody,
            eDeformableSurface,
            eRigid,
            eNone
        };

        Type type = eNone;
        bool isKinematic = false;
        uint64_t tetFinder = 0;
        uint64_t pointFinder = 0;
        pxr::VtArray<int32_t> indices;
        pxr::VtArray<pxr::GfVec3f> restPoints;
        pxr::VtArray<uint32_t> clothPointRemap;
        float pointScale = 0.0f;
        uint32_t numAttachments = 0;
    };

    struct UserVisibility
    {
        bool hideTarget[2];
    };

    // functions
    pxr::SdfPath getSessionAttachmentPath(const pxr::SdfPath attachmentPath);
    pxr::SdfPath createSessionAttachment(const pxr::SdfPath attachmentPath);
    void createSessionPointInstancer(const pxr::SdfPath sessionPointInstancerPath);
    void createSessionAttachmentGeometries(const pxr::SdfPath attachmentPath);
    void setupSessionPointInstancer(const pxr::SdfPath pointInstancerPath, const float radius, const pxr::GfVec3f& color);
    void resetSessionPointInstancer(const pxr::SdfPath pointInstancerPath);
    void updateSessionPointInstancerRadius(const pxr::SdfPath pointInstancerPath, const float radius);
    void setupSessionAttachmentGeometries(const pxr::SdfPath attachmentPath);

    bool addAttachment(const pxr::SdfPath attachmentPath);
    bool removeAttachment(const pxr::SdfPath attachmentPath);
    void createAttachment(const pxr::SdfPath attachmentPath);
    void releaseAttachment(const pxr::SdfPath attachmentPath);
    AttachmentInfo* getAttachmentInfo(pxr::SdfPath attachmentPath);

    TargetInfo* createTargetInfo(const pxr::SdfPath targetPath);
    void releaseTargetInfo(TargetInfo* targetInfo);
    TargetInfo* getTargetInfo(pxr::SdfPath targetPath);
    bool isTargetHidden(pxr::SdfPath attachmentPath, int32_t slot);
    bool isTargetHidden(pxr::SdfPath targetPath);
    float getPointScale(const pxr::SdfPath attachmentPath);
    float getPointScale(const pxr::UsdPrim& targetMeshPrim, const pxr::VtArray<pxr::GfVec3f>& restPoints) const;

    void updateAttachmentGeometry(const pxr::SdfPath attachmentPath);
    void updateAttachmentVisibility(const pxr::SdfPath attachmentPath, const bool isVisible);
    void updateRigidSkinVisibility(pxr::SdfPath targetPath, bool hide);
    void updateTargetTransform(const pxr::SdfPath targetPath);
    void updateTargetGeometry(const pxr::SdfPath targetPath);
    void updateVisualizationScale(const pxr::SdfPath attachmentPath);
    bool isVisible(const pxr::SdfPath attachmentPath);
    bool isSelected(const pxr::SdfPath attachmentPath);
    bool isComplete(const pxr::SdfPath attachmentPath);
    void clear(bool updateSkin);
    void clearBuffers(void);

    // class-scope using:
    using TargetPathPair = std::array<pxr::SdfPath, 2>;
    using SdfPathToAttachmentInfoMap = pxr::TfHashMap<pxr::SdfPath, AttachmentInfo*, pxr::SdfPath::Hash>;
    using SdfPathToTargetInfoMap = pxr::TfHashMap<pxr::SdfPath, TargetInfo*, pxr::SdfPath::Hash>;
    using SdfPathToSdfPathSetMap = pxr::TfHashMap<pxr::SdfPath, pxr::SdfPathSet, pxr::SdfPath::Hash>;
    using SdfPathToSdfPathMap = pxr::TfHashMap<pxr::SdfPath, pxr::SdfPath, pxr::SdfPath::Hash>;
    using SdfPathToSdfPathPairMap = pxr::TfHashMap<pxr::SdfPath, TargetPathPair, pxr::SdfPath::Hash>;

    using SdfPathToUserVisibilityMap = pxr::TfHashMap<pxr::SdfPath, UserVisibility, pxr::SdfPath::Hash>;
    using SdfPathTable = pxr::SdfPathTable<uint32_t>;

    // internal members:
    SdfPathToSdfPathPairMap mAttachments; // path map from all attachments on the stage to their two targets
    SdfPathToSdfPathSetMap mTargets; // path map from targets in mAttachments to their attachments

    SdfPathTable mAttachmentTable; // path table tracking all attachments on the stage
    SdfPathTable mTargetTable; // path table tracking all targets on the stage

    SdfPathToAttachmentInfoMap mAttachmentToInfo; // path map from valid attachments to their infos
    SdfPathToTargetInfoMap mTargetToInfo; // path map from targets of valid attachments to their info

    SdfPathToSdfPathMap mAttachmentToSession; // path map from attachments to session prim
    SdfPathToSdfPathMap mSessionGeomToAttachment; // path map from session geometry to attachment path (for selection
                                                  // swapping)
    pxr::SdfPath mSessionAttachmentRoot; // path to session layer scope "folder" for session tets
    VisualizerMode mMode; // mode for visualization [none, all, selected]
    bool mHideActor[2]; // whether actor0 or actor1 of each attachment should be hidden
    SdfPathToUserVisibilityMap mUserVizAttachments; // all attachments for which an external user (brush) requests
                                                    // visualization, mapping to bool indicating whether deformable
                                                    // should be hidden.
    pxr::SdfPathSet mVizAttachments; // all attachments that need to be visualized
    pxr::SdfPathSet mVizSelected; // all attachments that need to be visualized and are selected
    float mVisualizationGap;
    float mVisualizationScale;

    // buffers to store updates until next stage update that calls the manager's update function
    pxr::SdfPathSet mBufferAttachmentPathsToAdd;
    pxr::SdfPathSet mBufferAttachmentPathsToRemove;
    pxr::SdfPathSet mBufferAttachmentPathsToUpdateGeometry;
    pxr::SdfPathSet mBufferAttachmentPathsToUpdateVizAttribute;

    pxr::SdfPathSet mBufferTargetPathsToUpdateTransform;
    pxr::SdfPathSet mBufferTargetPathsToUpdateGeometry;
    bool mBufferVizDirty; // set to true whenever the set of tet meshes to visualize changes

    // xform cache:
    pxr::UsdGeomXformCache mXformCache;

    class DeformableBodyVisualizationManagerDeprecated* mDeformableBodyVisualizationManager;
};

} // namespace ui
} // namespace physx
} // namespace omni

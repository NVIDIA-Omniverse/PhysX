// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "UsdPCH.h"

#include <private/omni/physx/ui/VisualizerMode.h>

#include <usdrt/xformcache/IXformCache.h>


namespace omni
{

namespace physx
{

namespace ui
{

class SpatialTendonManager
{
public:
    explicit SpatialTendonManager(const VisualizerMode visualizationMode);
    ~SpatialTendonManager();

    void handlePrimResync(const PXR_NS::SdfPath path);
    void handlePrimRemove(const PXR_NS::SdfPath path);
    void handleAttributeChange(const PXR_NS::SdfPath path, const PXR_NS::TfToken attributeName, const bool isXform);

    void selectSpatialTendonAttachmentHelper(const PXR_NS::SdfPath linkBodyPath, const PXR_NS::TfToken instanceName);

    void parseStage();
    void update();
    void release();
    void selectionChanged();

    // visualization mode
    void setMode(const VisualizerMode mode);

    bool isEmpty() const
    {
        return mBodiesToAttachmentName.empty();
    }
    bool isActive() const
    {
        return mMode != VisualizerMode::eNone;
    };

private:
    // class-scope using:
    using SdfPathToSdfPathSet = PXR_NS::TfHashMap<PXR_NS::SdfPath, PXR_NS::SdfPathSet, PXR_NS::SdfPath::Hash>;
    using SdfPathToSdfPath = PXR_NS::TfHashMap<PXR_NS::SdfPath, PXR_NS::SdfPath, PXR_NS::SdfPath::Hash>;
    using TokenToTokenMap = PXR_NS::TfHashMap<PXR_NS::TfToken, PXR_NS::TfToken, PXR_NS::TfToken::HashFunctor>;
    using SdfPathToAttachmentName = PXR_NS::SdfPathTable<TokenToTokenMap>;
    using SdfPathToMatrixMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, PXR_NS::GfMatrix4d, PXR_NS::SdfPath::Hash>;

    // functions
    void clearBuffers(void);
    bool isAttachmentAPI(const PXR_NS::TfToken apiName, PXR_NS::TfToken& apiNameOut, PXR_NS::TfToken& instanceNameOut) const;
    void addAttachment(const PXR_NS::SdfPath bodyPath, const PXR_NS::TfToken apiName, const PXR_NS::TfToken instanceName);
    void removeAttachment(const PXR_NS::SdfPath bodyPath, const PXR_NS::TfToken instanceName);
    void addBody(const PXR_NS::SdfPath bodyPath);
    void updateBody(const PXR_NS::SdfPath bodyPath);
    void insertInParentChildMaps(const PXR_NS::SdfPath attachmentPath);
    void removeFromParentChildMaps(const PXR_NS::SdfPath attachmentPath);
    void removeBody(const PXR_NS::SdfPath bodyPath);
    void updateVisibilityAndCreateNewAttachmentGeoms(void);
    PXR_NS::SdfPath checkConnection(void);
    void setVisible(const PXR_NS::SdfPath bodyPath);
    void setInvisible(const PXR_NS::SdfPath bodyPath);
    void traverseTendonVisibility(const PXR_NS::SdfPath bodyPath, const TokenToTokenMap& attachments);
    void createAttachmentGeom(const PXR_NS::SdfPath geomPath, const PXR_NS::TfToken apiNameToken);
    void updateFromLocalPos(const PXR_NS::SdfPath attachmentPath);
    void updateToLocalPos(const PXR_NS::SdfPath attachmentPath);
    void updateXformTransform(const PXR_NS::SdfPath bodyPath);
    PXR_NS::SdfPath getSessionLayerXformPath(const PXR_NS::SdfPath bodyPath, const bool createIfNotExist);
    PXR_NS::SdfPath getParentAttachmentPath(const PXR_NS::SdfPath attachmentPath);
    bool isRootAttachment(const PXR_NS::SdfPath attachmentPath);
    bool isLeafAttachment(const PXR_NS::SdfPath attachmentPath);
    void computeAttachmentRadiusFromBodies(void);
    void setAttachmentRadiusScale(const double scale);
    void updateVisibleAttachmentRadii(void);
    void updateGeomScaleFromSettings(void);
    void processSelectionChanged(void);
    void runSetParentCommand(const PXR_NS::SdfPath childAttachment, const PXR_NS::SdfPath parentAttachment);
    void runRemoveAPICommand(const PXR_NS::SdfPath attachmentPath);
    // draws the tendon line overlay. Called on every frame from update:
    void draw(void);
    void scrollPropertyWindowToAPI(const PXR_NS::TfToken api);
    inline PXR_NS::TfToken getAttachmentAPI(const PXR_NS::SdfPath attachmentPath);
    PXR_NS::GfMatrix4d getBodyTransform(const PXR_NS::SdfPath& bodyPath, const PXR_NS::UsdPrim& bodyPrim);


    // internal members:
    PXR_NS::SdfPathSet mBodiesWithAttachments; // convenience set that keeps track of all paths to bodies with attachments
    PXR_NS::SdfPathVector mVisibleBodies; // set that keeps track of visible bodies
    SdfPathToSdfPath mChildToParent; // map to keep track of an attachment's parent
    SdfPathToSdfPath mSessionToBodies; // map from session layer attachment geoms to the bodies
    SdfPathToSdfPathSet mParentToChildren; // sets to keep track of an attachment's children
    SdfPathToAttachmentName mBodiesToAttachmentName; // table that keeps tabs on all attachments' names and APIs
    SdfPathToSdfPath mBodiesToSessionXforms; // map from bodies to their session layer xforms
    PXR_NS::SdfPath mSessionLayerScopePath; // path to session layer scope "folder" for session attachments
    VisualizerMode mMode;
    double mAttachmentRadiusFromBodyGeometry = 0.1;
    double mAttachmentRadiusScale = 1.0f;
    PXR_NS::SdfPath mLastSelectedAttachment;

    // buffers to store updates until next stage update that calls the manager's update function
    PXR_NS::SdfPathSet mBufferPathsToAdd; // paths to new RBs that have attachments
    PXR_NS::SdfPathSet mBufferPathsToUpdate; // paths to RBs that may have updated attachments
    PXR_NS::SdfPathSet mBufferPathsToRemove; // paths to Rbs that were removed
    PXR_NS::SdfPathSet mBufferPathsToSetVisible; // paths to Rbs that should be set to visible
    PXR_NS::SdfPathSet mBufferPathsToUpdateTransform; // paths to session layer attachments that need an xform update
    PXR_NS::SdfPathSet mBufferPathsToUpdateFromLocalPos;
    PXR_NS::SdfPathSet mBufferPathsToUpdateToLocalPos;
    PXR_NS::SdfPathSet mBufferPathsToUpdateParent;
    PXR_NS::SdfPathSet mBufferPathsToDeletedAttachments;
    PXR_NS::SdfPath mBufferAttachmentToSelect;
    PXR_NS::SdfPathVector mBufferSelectedPaths;
    bool mBufferVisibilityDirty = false;
    bool mBufferRecomputeAttachmentRadius = false;
    bool mBufferSetNewAttachmentRadius = false;
    bool mBufferSelectionChanged = false;

    // xform cache:
    PXR_NS::UsdGeomXformCache mXformCache;
    PXR_NS::UsdGeomBBoxCache mBBCache;

    omni::core::ObjectPtr<usdrt::xformcache::IXformCache> mFabricSync{ nullptr };
    bool mFabricEnabled = false;
    SdfPathToMatrixMap mBodyMatrices;

    // debug:
    void printMaps(const bool bodyUpdate, const bool selectionUpdate); // debug
};

} // namespace ui
} // namespace physx
} // namespace omni

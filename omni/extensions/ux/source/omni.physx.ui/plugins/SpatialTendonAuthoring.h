// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

    void handlePrimResync(const pxr::SdfPath path);
    void handlePrimRemove(const pxr::SdfPath path);
    void handleAttributeChange(const pxr::SdfPath path, const pxr::TfToken attributeName, const bool isXform);

    void selectSpatialTendonAttachmentHelper(const pxr::SdfPath linkBodyPath, const pxr::TfToken instanceName);

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
    using SdfPathToSdfPathSet = pxr::TfHashMap<pxr::SdfPath, pxr::SdfPathSet, pxr::SdfPath::Hash>;
    using SdfPathToSdfPath = pxr::TfHashMap<pxr::SdfPath, pxr::SdfPath, pxr::SdfPath::Hash>;
    using TokenToTokenMap = pxr::TfHashMap<pxr::TfToken, pxr::TfToken, pxr::TfToken::HashFunctor>;
    using SdfPathToAttachmentName = pxr::SdfPathTable<TokenToTokenMap>;
    using SdfPathToMatrixMap = pxr::TfHashMap<pxr::SdfPath, pxr::GfMatrix4d, pxr::SdfPath::Hash>;

    // functions
    void clearBuffers(void);
    bool isAttachmentAPI(const pxr::TfToken apiName, pxr::TfToken& apiNameOut, pxr::TfToken& instanceNameOut) const;
    void addAttachment(const pxr::SdfPath bodyPath, const pxr::TfToken apiName, const pxr::TfToken instanceName);
    void removeAttachment(const pxr::SdfPath bodyPath, const pxr::TfToken instanceName);
    void addBody(const pxr::SdfPath bodyPath);
    void updateBody(const pxr::SdfPath bodyPath);
    void insertInParentChildMaps(const pxr::SdfPath attachmentPath);
    void removeFromParentChildMaps(const pxr::SdfPath attachmentPath);
    void removeBody(const pxr::SdfPath bodyPath);
    void updateVisibilityAndCreateNewAttachmentGeoms(void);
    pxr::SdfPath checkConnection(void);
    void setVisible(const pxr::SdfPath bodyPath);
    void setInvisible(const pxr::SdfPath bodyPath);
    void traverseTendonVisibility(const pxr::SdfPath bodyPath, const TokenToTokenMap& attachments);
    void createAttachmentGeom(const pxr::SdfPath geomPath, const pxr::TfToken apiNameToken);
    void updateFromLocalPos(const pxr::SdfPath attachmentPath);
    void updateToLocalPos(const pxr::SdfPath attachmentPath);
    void updateXformTransform(const pxr::SdfPath bodyPath);
    pxr::SdfPath getSessionLayerXformPath(const pxr::SdfPath bodyPath, const bool createIfNotExist);
    pxr::SdfPath getParentAttachmentPath(const pxr::SdfPath attachmentPath);
    bool isRootAttachment(const pxr::SdfPath attachmentPath);
    bool isLeafAttachment(const pxr::SdfPath attachmentPath);
    void computeAttachmentRadiusFromBodies(void);
    void setAttachmentRadiusScale(const double scale);
    void updateVisibleAttachmentRadii(void);
    void updateGeomScaleFromSettings(void);
    void processSelectionChanged(void);
    void runSetParentCommand(const pxr::SdfPath childAttachment, const pxr::SdfPath parentAttachment);
    void runRemoveAPICommand(const pxr::SdfPath attachmentPath);
    // draws the tendon line overlay. Called on every frame from update:
    void draw(void);
    void scrollPropertyWindowToAPI(const pxr::TfToken api);
    inline pxr::TfToken getAttachmentAPI(const pxr::SdfPath attachmentPath);
    pxr::GfMatrix4d getBodyTransform(const pxr::SdfPath& bodyPath, const pxr::UsdPrim& bodyPrim);


    // internal members:
    pxr::SdfPathSet mBodiesWithAttachments; // convenience set that keeps track of all paths to bodies with attachments
    pxr::SdfPathVector mVisibleBodies; // set that keeps track of visible bodies
    SdfPathToSdfPath mChildToParent; // map to keep track of an attachment's parent
    SdfPathToSdfPath mSessionToBodies; // map from session layer attachment geoms to the bodies
    SdfPathToSdfPathSet mParentToChildren; // sets to keep track of an attachment's children
    SdfPathToAttachmentName mBodiesToAttachmentName; // table that keeps tabs on all attachments' names and APIs
    SdfPathToSdfPath mBodiesToSessionXforms; // map from bodies to their session layer xforms
    pxr::SdfPath mSessionLayerScopePath; // path to session layer scope "folder" for session attachments
    VisualizerMode mMode;
    double mAttachmentRadiusFromBodyGeometry = 0.1;
    double mAttachmentRadiusScale = 1.0f;
    pxr::SdfPath mLastSelectedAttachment;

    // buffers to store updates until next stage update that calls the manager's update function
    pxr::SdfPathSet mBufferPathsToAdd; // paths to new RBs that have attachments
    pxr::SdfPathSet mBufferPathsToUpdate; // paths to RBs that may have updated attachments
    pxr::SdfPathSet mBufferPathsToRemove; // paths to Rbs that were removed
    pxr::SdfPathSet mBufferPathsToSetVisible; // paths to Rbs that should be set to visible
    pxr::SdfPathSet mBufferPathsToUpdateTransform; // paths to session layer attachments that need an xform update
    pxr::SdfPathSet mBufferPathsToUpdateFromLocalPos;
    pxr::SdfPathSet mBufferPathsToUpdateToLocalPos;
    pxr::SdfPathSet mBufferPathsToUpdateParent;
    pxr::SdfPathSet mBufferPathsToDeletedAttachments;
    pxr::SdfPath mBufferAttachmentToSelect;
    pxr::SdfPathVector mBufferSelectedPaths;
    bool mBufferVisibilityDirty = false;
    bool mBufferRecomputeAttachmentRadius = false;
    bool mBufferSetNewAttachmentRadius = false;
    bool mBufferSelectionChanged = false;

    // xform cache:
    pxr::UsdGeomXformCache mXformCache;
    pxr::UsdGeomBBoxCache mBBCache;

    omni::core::ObjectPtr<usdrt::xformcache::IXformCache> mFabricSync{ nullptr };
    bool mFabricEnabled = false;
    SdfPathToMatrixMap mBodyMatrices;

    // debug:
    void printMaps(const bool bodyUpdate, const bool selectionUpdate); // debug
};

} // namespace ui
} // namespace physx
} // namespace omni

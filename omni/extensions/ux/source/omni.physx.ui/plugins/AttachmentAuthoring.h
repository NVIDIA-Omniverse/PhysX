// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

namespace omni
{

namespace physx
{

namespace ui
{
class AttachmentAuthoring
{
public:
    explicit AttachmentAuthoring(bool enabled);
    ~AttachmentAuthoring();

    void refreshAttachment(pxr::SdfPath attachmentPath);

    void handlePrimResync(const pxr::SdfPath path);
    void handlePrimRemove(const pxr::SdfPath path);
    void handleAttributeChange(const pxr::SdfPath path, const pxr::TfToken attributeName, const bool isXform);

    void parseStage();
    void update();
    void release();

    void setEnabled(bool enable)
    {
        mIsEnabled = enable;
    }

    bool isActive() const
    {
        return mIsEnabled;
    };

    // returns true if there are no attachments in the stage
    bool isEmpty() const
    {
        return mAttachments.empty();
    }

private:
    struct AttachmentInfo
    {
        pxr::SdfPath targets[2] = { pxr::SdfPath(), pxr::SdfPath() }; // targets can be empty, or stale (point to
                                                                      // removed prims)
        pxr::SdfPathSet shapes; // shapes can be stale (point to removed prims)
    };

    struct RefInfo
    {
        pxr::SdfPathSet attachments;
        bool isStale;
    };

    // class-scope using:
    using SdfPathToAttachmentInfoMap = pxr::TfHashMap<pxr::SdfPath, AttachmentInfo, pxr::SdfPath::Hash>;
    using SdfPathToRefInfoMap = pxr::TfHashMap<pxr::SdfPath, RefInfo, pxr::SdfPath::Hash>;

    struct Empty
    {
    };
    using SdfPathTable = pxr::SdfPathTable<Empty>;

    // functions
    void addAttachment(const pxr::SdfPath attachmentPath);
    void removeAttachment(const pxr::SdfPath attachmentPath);
    void updateAttachmentTargets(const pxr::SdfPath attachmentPath);
    void updateAttachmentMaskShapes(const pxr::SdfPath attachmentPath);
    void addRefPath(SdfPathToRefInfoMap& pathToRefInfoMap,
                    SdfPathTable& pathTable,
                    const pxr::SdfPath refPath,
                    const pxr::SdfPath attachmentPath);
    void removeRefPath(SdfPathToRefInfoMap& pathToRefInfoMap,
                       SdfPathTable& pathTable,
                       const pxr::SdfPath refPath,
                       const pxr::SdfPath attachmentPath);
    void clearBuffers(void);

    // internal members:
    SdfPathToAttachmentInfoMap mAttachments; // path map from attachments on stage to AttachmentInfo
    SdfPathTable mAttachmentTable; // path table that tracks attachments
    SdfPathToRefInfoMap mTargets; // path map from targets to RefInfo, targets can be stale
    SdfPathTable mTargetTable; // path table that tracks targets
    SdfPathToRefInfoMap mShapes; // path map from shapes to RefInfo, shapes can be stale
    SdfPathTable mShapeTable; // path table that tracks attachment shapes

    // buffers to store updates until next stage update that calls the manager's update function
    pxr::SdfPathSet mBufferAttachmentPathsToAdd;
    pxr::SdfPathSet mBufferAttachmentPathsToRemove;
    pxr::SdfPathSet mBufferAttachmentPathsToUpdate;
    pxr::SdfPathSet mBufferAttachmentPathsToUpdateTargets;
    pxr::SdfPathSet mBufferAttachmentPathsToUpdateShapes;

    bool mIsEnabled;
};
} // namespace ui
} // namespace physx
} // namespace omni

// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

    void refreshAttachment(PXR_NS::SdfPath attachmentPath);

    void handlePrimResync(const PXR_NS::SdfPath path);
    void handlePrimRemove(const PXR_NS::SdfPath path);
    void handleAttributeChange(const PXR_NS::SdfPath path, const PXR_NS::TfToken attributeName, const bool isXform);

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
        PXR_NS::SdfPath targets[2] = { PXR_NS::SdfPath(), PXR_NS::SdfPath() }; // targets can be empty, or stale (point to
                                                                      // removed prims)
        PXR_NS::SdfPathSet shapes; // shapes can be stale (point to removed prims)
    };

    struct RefInfo
    {
        PXR_NS::SdfPathSet attachments;
        bool isStale;
    };

    // class-scope using:
    using SdfPathToAttachmentInfoMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, AttachmentInfo, PXR_NS::SdfPath::Hash>;
    using SdfPathToRefInfoMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, RefInfo, PXR_NS::SdfPath::Hash>;

    struct Empty
    {
    };
    using SdfPathTable = PXR_NS::SdfPathTable<Empty>;

    // functions
    void addAttachment(const PXR_NS::SdfPath attachmentPath);
    void removeAttachment(const PXR_NS::SdfPath attachmentPath);
    void updateAttachmentTargets(const PXR_NS::SdfPath attachmentPath);
    void updateAttachmentMaskShapes(const PXR_NS::SdfPath attachmentPath);
    void addRefPath(SdfPathToRefInfoMap& pathToRefInfoMap,
                    SdfPathTable& pathTable,
                    const PXR_NS::SdfPath refPath,
                    const PXR_NS::SdfPath attachmentPath);
    void removeRefPath(SdfPathToRefInfoMap& pathToRefInfoMap,
                       SdfPathTable& pathTable,
                       const PXR_NS::SdfPath refPath,
                       const PXR_NS::SdfPath attachmentPath);
    void clearBuffers(void);

    // internal members:
    SdfPathToAttachmentInfoMap mAttachments; // path map from attachments on stage to AttachmentInfo
    SdfPathTable mAttachmentTable; // path table that tracks attachments
    SdfPathToRefInfoMap mTargets; // path map from targets to RefInfo, targets can be stale
    SdfPathTable mTargetTable; // path table that tracks targets
    SdfPathToRefInfoMap mShapes; // path map from shapes to RefInfo, shapes can be stale
    SdfPathTable mShapeTable; // path table that tracks attachment shapes

    // buffers to store updates until next stage update that calls the manager's update function
    PXR_NS::SdfPathSet mBufferAttachmentPathsToAdd;
    PXR_NS::SdfPathSet mBufferAttachmentPathsToRemove;
    PXR_NS::SdfPathSet mBufferAttachmentPathsToUpdate;
    PXR_NS::SdfPathSet mBufferAttachmentPathsToUpdateTargets;
    PXR_NS::SdfPathSet mBufferAttachmentPathsToUpdateShapes;

    bool mIsEnabled;
};
} // namespace ui
} // namespace physx
} // namespace omni

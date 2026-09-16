// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-BUILD-UNIBUILD-001
 * @covers AC-3
 */

// pxr-free facade for the deformable auto-attachment sub-prim authoring (ADR-0027).
//
// The capability is USD-library-only by design (see AttachmentAuthoringBridge.h). This facade
// keeps ObjectKey->path-string resolution on its own pxr-free side -- AttachedStage::textFor()
// resolves each key through IPhysicsSource::sourceKeyToString, which yields exactly the SdfPath
// string a USD source would interned-look-up -- and hands the parse::IUsdAttachmentAuthor seam
// only the resolved string paths plus the opaque UsdStage handle storage, so the seam rebuilds
// each SdfPath and never needs the live source. When no seam is installed (production),
// canAuthor() returns false and every author call is a no-op; canAuthor() gates the sole
// caller, setupAutoDeformableAttachment.

#include "usdBridge/AttachmentAuthoringBridge.h"
#include "usdLoad/AttachedStage.h"

#include <omni/physics/parse/UsdAttachmentAuthor.h>

namespace omni
{
namespace physx
{
namespace attachmentauthoring
{

namespace
{
omni::physics::parse::UsdAttachmentPrimKind seamKind(AttachmentPrimKind kind)
{
    switch (kind)
    {
    case AttachmentPrimKind::eVtxVtx:
        return omni::physics::parse::UsdAttachmentPrimKind::eVtxVtx;
    case AttachmentPrimKind::eVtxTri:
        return omni::physics::parse::UsdAttachmentPrimKind::eVtxTri;
    case AttachmentPrimKind::eVtxTet:
        return omni::physics::parse::UsdAttachmentPrimKind::eVtxTet;
    case AttachmentPrimKind::eVtxXform:
    default:
        return omni::physics::parse::UsdAttachmentPrimKind::eVtxXform;
    }
}
} // namespace

bool canAuthor(const usdparser::AttachedStage& stage)
{
    omni::physics::parse::IUsdAttachmentAuthor* author = omni::physics::parse::usdAttachmentAuthor();
    if (!author)
        return false;
    const usdparser::AttachedStageUsdHandle handle = stage.getStage();
    return author->canAuthor(handle.storage());
}

void removeAttachmentsAndFilters(const usdparser::AttachedStage& stage,
                                 omni::physics::parse::ObjectKey autoAttachmentKey)
{
    omni::physics::parse::IUsdAttachmentAuthor* author = omni::physics::parse::usdAttachmentAuthor();
    if (!author)
        return;
    const usdparser::AttachedStageUsdHandle handle = stage.getStage();
    author->removeAttachmentsAndFilters(handle.storage(), stage.textFor(autoAttachmentKey));
}

void disableAttachmentsAndFilters(const usdparser::AttachedStage& stage,
                                  omni::physics::parse::ObjectKey autoAttachmentKey)
{
    omni::physics::parse::IUsdAttachmentAuthor* author = omni::physics::parse::usdAttachmentAuthor();
    if (!author)
        return;
    const usdparser::AttachedStageUsdHandle handle = stage.getStage();
    author->disableAttachmentsAndFilters(handle.storage(), stage.textFor(autoAttachmentKey));
}

void defineAttachmentPrim(const usdparser::AttachedStage& stage,
                          omni::physics::parse::ObjectKey autoAttachmentKey,
                          const char* childName,
                          AttachmentPrimKind kind,
                          omni::physics::parse::ObjectKey body0,
                          omni::physics::parse::ObjectKey body1)
{
    omni::physics::parse::IUsdAttachmentAuthor* author = omni::physics::parse::usdAttachmentAuthor();
    if (!author)
        return;
    const usdparser::AttachedStageUsdHandle handle = stage.getStage();
    author->defineAttachmentPrim(handle.storage(), stage.textFor(autoAttachmentKey), childName, seamKind(kind),
                                 stage.textFor(body0), stage.textFor(body1));
}

void defineElementCollisionFilterPrim(const usdparser::AttachedStage& stage,
                                      omni::physics::parse::ObjectKey autoAttachmentKey,
                                      const char* childName,
                                      omni::physics::parse::ObjectKey body0,
                                      omni::physics::parse::ObjectKey body1)
{
    omni::physics::parse::IUsdAttachmentAuthor* author = omni::physics::parse::usdAttachmentAuthor();
    if (!author)
        return;
    const usdparser::AttachedStageUsdHandle handle = stage.getStage();
    author->defineElementCollisionFilterPrim(handle.storage(), stage.textFor(autoAttachmentKey), childName,
                                             stage.textFor(body0), stage.textFor(body1));
}

} // namespace attachmentauthoring
} // namespace physx
} // namespace omni

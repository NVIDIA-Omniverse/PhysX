// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

#pragma once

#include <cstddef>
#include <memory>

namespace omni::physics::parse
{

// Which OmniPhysics*Attachment prim type to define. Mirrors the pxr-free
// omni::physx::attachmentauthoring::AttachmentPrimKind on the omni.physx side,
// which the facade maps onto this one before crossing the seam (the two enums
// are kept in step but not shared, so this pxr-free parse header takes no
// dependency on an omni.physx plugin header).
enum class UsdAttachmentPrimKind
{
    eVtxXform,
    eVtxVtx,
    eVtxTri,
    eVtxTet
};

// pxr-free interface the loadable USD library installs so the USD-free omni.physx side can
// create/destroy the Attachment / ElementCollisionFilter child prims of a deformable
// auto-attachment prim without linking or naming any pxr type (ADR-0027 bridge collapse).
//
// The whole capability is USD-library-only by design: no IPhysicsDataWrite verb covers
// whole-prim create/destroy; an attach without this seam keeps the sub prims in memory
// instead (ADR-0028, see AttachmentAuthoringBridge.h). The omni.physx facade keeps ObjectKey->path-string resolution
// on its own pxr-free side (IPhysicsSource::sourceKeyToString, via AttachedStage::textFor) and
// hands this seam only resolved string paths, so the seam never needs the live source -- it
// rebuilds each SdfPath from the string. `stageHandleStorage` points at the opaque
// UsdStageWeakPtr bytes an AttachedStageUsdHandle owns.
//
// Installed by omniPhysicsUsdInstallBackends(); null in production, where the omni.physx
// facade reports "cannot author" and no-ops.
class IUsdAttachmentAuthor
{
public:
    virtual ~IUsdAttachmentAuthor() = default;

    // Can this attach create/destroy attachment sub prims at all? Asks about the backing USD
    // stage at `stageHandleStorage` (true only when it is live), mirroring the USD arm's
    // canAuthor(bool(stage.getStage())).
    virtual bool canAuthor(const void* stageHandleStorage) const = 0;

    // Destroys every Attachment / ElementCollisionFilter prim under `autoAttachmentPath`.
    virtual void removeAttachmentsAndFilters(const void* stageHandleStorage,
                                             const char* autoAttachmentPath) = 0;

    // Clears attachmentEnabled / filterEnabled on those prims without destroying them.
    virtual void disableAttachmentsAndFilters(const void* stageHandleStorage,
                                              const char* autoAttachmentPath) = 0;

    // Defines `<autoAttachmentPath>/<childName>` as an attachment prim of `kind` between the
    // two bodies (resolved path strings, empty for an unresolvable body).
    virtual void defineAttachmentPrim(const void* stageHandleStorage,
                                      const char* autoAttachmentPath,
                                      const char* childName,
                                      UsdAttachmentPrimKind kind,
                                      const char* body0Path,
                                      const char* body1Path) = 0;

    // Defines `<autoAttachmentPath>/<childName>` as an element-collision-filter prim between
    // the two bodies.
    virtual void defineElementCollisionFilterPrim(const void* stageHandleStorage,
                                                  const char* autoAttachmentPath,
                                                  const char* childName,
                                                  const char* body0Path,
                                                  const char* body1Path) = 0;
};

// Install `author` as the single active attachment author, replacing any previous one.
// null clears it. Must be called detached (ADR-0005/0027).
void setUsdAttachmentAuthor(std::unique_ptr<IUsdAttachmentAuthor> author);

// The active attachment author, or null when none is installed.
IUsdAttachmentAuthor* usdAttachmentAuthor();

} // namespace omni::physics::parse

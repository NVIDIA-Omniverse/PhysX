// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-BUILD-UNIBUILD-001
 * @covers AC-3
 *
 * @implements REQ-SIM-AUTOATTACH-001
 * @covers AC-7
 */

#pragma once

// Deformable auto-attachment sub-prim authoring (ADR-0027). Creating and destroying the
// Attachment / ElementCollisionFilter child prims of a PhysxAutoDeformableAttachmentAPI prim
// goes through omni::physics::usd::define*Prim / removeAttachmentsAndFilters
// (UsdDeformableAttachmentWrite.h), which is USD-library-only by design: no IPhysicsDataWrite
// verb covers whole-prim create/destroy. An attach that cannot author keeps the sub prims in
// memory instead (GeneratedAutoAttachmentLayout, ADR-0028); canAuthor() is the switch.
//
// AttachmentAuthoringBridge.cpp resolves each ObjectKey to a path string and authors through
// the parse::usdAttachmentAuthor() seam; with no seam installed it reports "cannot author"
// and no-ops.
//
// Deliberately NOT gated on AttachedStage::getDataWrite(): that is null for an ovstage attach
// while getStage() is live, so a write-sink gate would silently disable this authoring in the
// default build. canAuthor() asks about the stage, as the fenced code did.

#include <omni/physics/parse/Handles.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;
}

namespace attachmentauthoring
{

// Which OmniPhysics*Attachment prim type to define. Mirrors the
// OmniUsdPhysicsDeformableSchemaTokens the USD arm maps these onto.
enum class AttachmentPrimKind
{
    eVtxXform,
    eVtxVtx,
    eVtxTri,
    eVtxTet
};

// Can this attach create/destroy attachment sub prims at all? False when no attachment-author
// seam is installed, and whenever the attach has no live USD stage.
bool canAuthor(const usdparser::AttachedStage& stage);

// Destroys every Attachment / ElementCollisionFilter prim under the auto-attachment prim.
void removeAttachmentsAndFilters(const usdparser::AttachedStage& stage,
                                 omni::physics::parse::ObjectKey autoAttachmentKey);

// Clears attachmentEnabled / filterEnabled on those prims without destroying them.
void disableAttachmentsAndFilters(const usdparser::AttachedStage& stage,
                                  omni::physics::parse::ObjectKey autoAttachmentKey);

// Defines `<autoAttachment>/<childName>` as an attachment prim of `kind` between the two bodies.
void defineAttachmentPrim(const usdparser::AttachedStage& stage,
                          omni::physics::parse::ObjectKey autoAttachmentKey,
                          const char* childName,
                          AttachmentPrimKind kind,
                          omni::physics::parse::ObjectKey body0,
                          omni::physics::parse::ObjectKey body1);

// Defines `<autoAttachment>/<childName>` as an element-collision-filter prim between the two
// bodies.
void defineElementCollisionFilterPrim(const usdparser::AttachedStage& stage,
                                      omni::physics::parse::ObjectKey autoAttachmentKey,
                                      const char* childName,
                                      omni::physics::parse::ObjectKey body0,
                                      omni::physics::parse::ObjectKey body1);

} // namespace attachmentauthoring
} // namespace physx
} // namespace omni

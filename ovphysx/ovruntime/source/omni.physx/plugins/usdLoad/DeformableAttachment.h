// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <private/omni/physx/PhysxUsd.h>

namespace omni::physics::parse
{
struct PhysxDeformableAttachmentDesc;
struct PhysxDeformableCollisionFilterDesc;
} // namespace omni::physics::parse

namespace omni::physics::usd
{
class ScannedStage;
} // namespace omni::physics::usd

namespace omni
{
namespace physx
{
namespace usdparser
{

// Translate a parse-lib `PhysxDeformableAttachmentDesc` (from `scanStage`)
// into the consumer-side ICE-allocated `usdparser::PhysxDeformableAttachmentDesc`.
// The `ScannedStage` parameter resolves the parse-lib `ObjectKey`s on
// `src0` / `src1` back to `SdfPath`s for the consumer descriptor's
// path-typed cross-references.  Returns nullptr for attachment subtypes
// the consumer runtime does not handle (eAttachmentVtxCrv /
// eAttachmentTriTri).
PhysxDeformableAttachmentDesc* parseDeformableAttachment(
    const omni::physics::usd::ScannedStage& scanned,
    const omni::physics::parse::PhysxDeformableAttachmentDesc& inDesc);

// Translate a parse-lib `PhysxDeformableCollisionFilterDesc` into the
// consumer-side desc.  Same `ScannedStage` rationale as above.
PhysxDeformableCollisionFilterDesc* parseDeformableCollisionFilter(
    const omni::physics::usd::ScannedStage& scanned,
    const omni::physics::parse::PhysxDeformableCollisionFilterDesc& inDesc);
} // namespace usdparser
} // namespace physx
} // namespace omni

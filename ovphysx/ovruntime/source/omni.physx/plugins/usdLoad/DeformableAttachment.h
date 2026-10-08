// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27 AC-29
 */

#pragma once

#include <private/omni/physx/PhysxUsd.h>

namespace omni::physics::parse
{
struct PhysxDeformableAttachmentDesc;
struct PhysxDeformableCollisionFilterDesc;
class ScannedStage;
} // namespace omni::physics::parse

namespace omni
{
namespace physx
{
namespace usdparser
{

class AttachedStage;

// Translate a parse-lib `PhysxDeformableAttachmentDesc` (from `scanStage`)
// into the consumer-side ICE-allocated `usdparser::PhysxDeformableAttachmentDesc`
// -- now an alias of the same parse-lib type (ADR-0019 increment 7), so this
// is mostly a field copy plus the `ObjectType` remap, EXCEPT `src0`/`src1`:
// `scanned`'s ObjectKeys are minted by its own throwaway, parse-time source
// (ADR-0004's key-space invariant), but every consumer of the returned desc
// (InternalDeformableAttachment.cpp) resolves them through `attachedStage`,
// so they are re-keyed into `attachedStage`'s persistent namespace via a
// source-key-string round trip here. Returns nullptr for attachment subtypes
// the consumer runtime does not handle (eAttachmentVtxCrv / eAttachmentTriTri).
// `scanned` takes the source-agnostic base type (pxr-free) -- callers may still
// pass a derived `omni::physics::usd::ScannedStage`, which upcasts implicitly.
PhysxDeformableAttachmentDesc* parseDeformableAttachment(
    const omni::physics::parse::ScannedStage& scanned,
    const omni::physics::parse::PhysxDeformableAttachmentDesc& inDesc,
    const AttachedStage& attachedStage);

// Translate a parse-lib `PhysxDeformableCollisionFilterDesc` into the
// consumer-side desc.  Same rationale as above.
PhysxDeformableCollisionFilterDesc* parseDeformableCollisionFilter(
    const omni::physics::parse::ScannedStage& scanned,
    const omni::physics::parse::PhysxDeformableCollisionFilterDesc& inDesc,
    const AttachedStage& attachedStage);
} // namespace usdparser
} // namespace physx
} // namespace omni

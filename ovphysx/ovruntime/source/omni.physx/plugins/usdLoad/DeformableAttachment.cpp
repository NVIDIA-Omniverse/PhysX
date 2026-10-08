// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27 AC-29
 */

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>

#include "LoadTools.h"
#include "AttachedStage.h"
#include "DeformableAttachment.h"

#include <omni/physics/parse/Descriptors.h>
#include <omni/physics/parse/ScannedStage.h>

using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{
    // `scanned` takes the source-agnostic base type (omni::physics::parse::ScannedStage,
    // pxr-free -- omni::physics::usd::ScannedStage publicly derives from it and callers
    // still pass that derived object; the reference upcasts implicitly), so this file
    // needs no pxr and both functions are unconditional. Uses
    // scanned.source().sourceKeyToString() instead of the USD-only pathFor()/keyFor(SdfPath)
    // (same rekey idiom as LoadStage.cpp's invertCollisionGroupMembers).
    PhysxDeformableAttachmentDesc* parseDeformableAttachment(
        const omni::physics::parse::ScannedStage& scanned,
        const omni::physics::parse::PhysxDeformableAttachmentDesc& inDesc,
        const AttachedStage& attachedStage)
    {
        // inDesc's ObjectKeys are minted by `scanned`'s own throwaway,
        // parse-time source (ADR-0004 key-space invariant); re-key into
        // `attachedStage`'s persistent namespace via a source-key-string
        // round trip, since every consumer of the returned desc resolves
        // through `attachedStage` (mirrors the `rekey` pattern in LoadStage.cpp).
        auto rekey = [&](omni::physics::parse::ObjectKey k) -> omni::physics::parse::ObjectKey
        {
            return k.valid() ? attachedStage.keyFor(scanned.source().sourceKeyToString(k)) :
                                omni::physics::parse::ObjectKey{};
        };

        ObjectType outType = ObjectType::eUndefined;
        switch (inDesc.type)
        {
        case omni::physics::parse::ObjectType::eAttachmentVtxVtx:
            outType = ObjectType::eAttachmentVtxVtx;
            break;
        case omni::physics::parse::ObjectType::eAttachmentVtxTri:
            outType = ObjectType::eAttachmentVtxTri;
            break;
        case omni::physics::parse::ObjectType::eAttachmentVtxTet:
            outType = ObjectType::eAttachmentVtxTet;
            break;
        case omni::physics::parse::ObjectType::eAttachmentVtxXform:
            outType = ObjectType::eAttachmentVtxXform;
            break;
        case omni::physics::parse::ObjectType::eAttachmentTetXform:
            outType = ObjectType::eAttachmentTetXform;
            break;
        default:
            // eAttachmentVtxCrv / eAttachmentTriTri are not supported
            // by the consumer runtime today — mirrors the legacy
            // schema-desc switch which silently dropped them.
            return nullptr;
        }

        PhysxDeformableAttachmentDesc* outDesc = ICE_PLACEMENT_NEW(PhysxDeformableAttachmentDesc)();
        if (outDesc)
        {
            outDesc->type = outType;
            outDesc->primKey = rekey(inDesc.primKey);
            outDesc->enabled = inDesc.enabled;
            outDesc->src0 = rekey(inDesc.src0);
            outDesc->src1 = rekey(inDesc.src1);
            outDesc->stiffness = inDesc.stiffness;
            outDesc->damping = inDesc.damping;
        }
        return outDesc;
    }

    PhysxDeformableCollisionFilterDesc* parseDeformableCollisionFilter(
        const omni::physics::parse::ScannedStage& scanned,
        const omni::physics::parse::PhysxDeformableCollisionFilterDesc& inDesc,
        const AttachedStage& attachedStage)
    {
        auto rekey = [&](omni::physics::parse::ObjectKey k) -> omni::physics::parse::ObjectKey
        {
            return k.valid() ? attachedStage.keyFor(scanned.source().sourceKeyToString(k)) :
                                omni::physics::parse::ObjectKey{};
        };

        PhysxDeformableCollisionFilterDesc* outDesc = ICE_PLACEMENT_NEW(PhysxDeformableCollisionFilterDesc)();

        outDesc->primKey = rekey(inDesc.primKey);
        outDesc->enabled = inDesc.enabled;
        outDesc->src0 = rekey(inDesc.src0);
        outDesc->src1 = rekey(inDesc.src1);

        return outDesc;
    }

} // namespace usdparser
} // namespace physx
} // namespace omni

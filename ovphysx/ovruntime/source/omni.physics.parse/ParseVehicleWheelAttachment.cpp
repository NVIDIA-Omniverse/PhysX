// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-VEH-WHEELATTACH-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleWheelAttachmentAPI reader. Per-wheel-attachment parser
// for the per-vehicle wheel attachment descriptor. Walker pre-reads
// scalar / Vec3 / Quat attrs + pre-resolves the three rel-or-API
// cross-refs (wheel / tire / suspension) + pre-walks descendants for
// the collision-shape detection.
//
// SuspensionComplianceAPI may be applied alongside on the same prim;
// it is emitted separately by the walker (the consumer adapter wires
// the suspensionCompliancePath ObjectKey on the WheelAttachmentDesc
// to the matching SuspensionComplianceDesc).

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

namespace
{

constexpr int kMaxNumberOfWheels = 20;  // mirrors VehicleDesc::maxNumberOfWheels

} // namespace

// @implements REQ-PARSE-VEH-WHEELATTACH-001
// @covers AC-1 AC-2 AC-3
DescPtr<WheelAttachmentDesc> parseWheelAttachment(
    ParseContext& ctx, ObjectKey key, const WheelAttachmentInfo& info)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(key));

    DescPtr<WheelAttachmentDesc> desc = allocateDesc<WheelAttachmentDesc>(ctx.descriptorAllocator());
    desc->key = key;
    desc->id = kInvalidObjectId;
    desc->state = info.state;  // walker-resolved (eMANAGE_TRANSFORMS / eHAS_SHAPE / eHAS_WHEEL_COM_OFFSET etc.)
    desc->wheel = nullptr;
    desc->wheelId = kInvalidObjectId;
    desc->tire = nullptr;
    desc->tireId = kInvalidObjectId;
    desc->suspension = nullptr;
    desc->suspensionId = kInvalidObjectId;
    desc->suspensionCompliance = nullptr;

    desc->shapeKey = info.shapeKey;
    desc->shapeId = kInvalidObjectId;
    desc->collisionGroupKey = info.collisionGroupKey;
    desc->collisionGroupId = kInvalidObjectId;

    desc->suspensionTravelDirection      = info.suspensionTravelDirection;
    desc->suspensionForceAppPointOffset  = info.suspensionForceAppPointOffset;
    desc->wheelCenterOfMassOffset        = info.wheelCenterOfMassOffset;
    desc->tireForceAppPointOffset        = info.tireForceAppPointOffset;
    desc->suspensionFramePosition        = info.suspensionFramePosition;
    desc->suspensionFrameOrientation     = info.suspensionFrameOrientation;
    desc->wheelFramePosition             = info.wheelFramePosition;
    desc->wheelFrameOrientation          = info.wheelFrameOrientation;
    desc->driven                         = info.driven;

    // index must be in [-1, maxNumberOfWheels)
    if (info.index < -1 || info.index >= kMaxNumberOfWheels)
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"index\" needs to be in [-1, %d).",
                       ownerName.c_str(), kMaxNumberOfWheels);
        return {};
    }
    desc->index = info.index;

    return desc;
}

} // namespace omni::physics::parse

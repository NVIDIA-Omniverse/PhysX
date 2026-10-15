// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

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
#include <omni/physics/parse/PreparedVehicle.h>
#include <stdexcept>
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
PreparedWheelAttachment prepareWheelAttachment(ParseContext& ctx, ObjectKey key, const WheelAttachmentInfo& info)
{
    PreparedWheelAttachment input;
    input.key = key;
    input.info = info;
    input.reads = prepareVehicleReads(ctx.source(), key, {

    });
    return input;
}

void prepareWheelAttachmentStorage(const PreparedWheelAttachment& input, WheelAttachmentDesc& output)
{
    (void)input; (void)output;
}

bool fillWheelAttachment(const PreparedWheelAttachment& input, WheelAttachmentDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;
    [[maybe_unused]] const WheelAttachmentInfo& info = input.info;


    const std::string& ownerName = src.sourceKeyToString(key);

    WheelAttachmentDesc* desc = &output;
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
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"index\" needs to be in [-1, %d).",
                       ownerName.c_str(), kMaxNumberOfWheels);
        return false;
    }
    desc->index = info.index;

    return true;
}

void finishWheelAttachmentStorage(WheelAttachmentDesc& output, const VehicleFillStatus& status)
{
    (void)output; (void)status;
}

DescPtr<WheelAttachmentDesc> parseWheelAttachment(ParseContext& ctx, ObjectKey key, const WheelAttachmentInfo& info)
{
    const PreparedWheelAttachment input = prepareWheelAttachment(ctx, key, info);
    DescPtr<WheelAttachmentDesc> output = allocateDesc<WheelAttachmentDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareWheelAttachmentStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillWheelAttachment(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishWheelAttachmentStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse

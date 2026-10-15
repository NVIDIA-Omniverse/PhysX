// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-VEH-CONTEXT-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleContextAPI reader. Parses the scene-level vehicle update
// mode + vertical / longitudinal axis tokens. When verticalAxis or
// longitudinalAxis resolves to "undefined", falls back to the
// deprecated upAxis / forwardAxis Vec3 attributes. Caller-side
// validates that the prim is a UsdPhysicsScene before invoking.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedVehicle.h>
#include <stdexcept>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

namespace omni::physics::parse
{

namespace
{

bool tokenToAxisDir(std::string_view tok, VehicleContextDesc::AxisDir& out)
{
    if      (tok == "posX") { out = VehicleContextDesc::ePosX; return true; }
    else if (tok == "negX") { out = VehicleContextDesc::eNegX; return true; }
    else if (tok == "posY") { out = VehicleContextDesc::ePosY; return true; }
    else if (tok == "negY") { out = VehicleContextDesc::eNegY; return true; }
    else if (tok == "posZ") { out = VehicleContextDesc::ePosZ; return true; }
    else if (tok == "negZ") { out = VehicleContextDesc::eNegZ; return true; }
    return false;
}

bool readAxisOrDeprecatedVec3(PreparedVehicleReader& src, ObjectKey key,
                              std::string_view tokenAttrName,
                              std::string_view deprecatedVec3AttrName,
                              VehicleContextDesc::AxisDir& outAxis,
                              carb::Float3& outVec3,
                              std::string_view ownerName,
                              const char* axisLabel, VehicleFillStatus& status)
{
    TokenId tok;
    if (!src.getAttribute(key, src.attribute(tokenAttrName), tok))
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: \"%s\" needs to have token \"%s\" defined.",
                       ownerName.data(), axisLabel);
        return false;
    }
    const std::string_view tokStr = src.tokenToString(tok);
    if (tokStr != "undefined")
    {
        if (!tokenToAxisDir(tokStr, outAxis))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: \"%s\" has unsupported \"%s\" token \"%s\".",
                           ownerName.data(), axisLabel, tokStr.data());
            return false;
        }
        return true;
    }
    outAxis = VehicleContextDesc::eUndefined;
    if (!src.getAttribute(key, src.attribute(deprecatedVec3AttrName), outVec3))
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: \"%s\" needs to have attribute \"%s\" defined when \"%s\" is undefined.",
                       ownerName.data(), deprecatedVec3AttrName.data(), axisLabel);
        return false;
    }
    VEHICLE_LOG_WARN(status, "Usd Physics: vehicle context \"%s\": attribute \"%s\" is deprecated. Please use \"%s\" instead.",
                  ownerName.data(), deprecatedVec3AttrName.data(), axisLabel);
    return true;
}

} // namespace

// @implements REQ-PARSE-VEH-CONTEXT-001
// @covers AC-1 AC-2 AC-3
PreparedVehicleContext prepareVehicleContext(ParseContext& ctx, ObjectKey key)
{
    PreparedVehicleContext input;
    input.key = key;
    input.reads = prepareVehicleReads(ctx.source(), key, {
        { "physxVehicleContext:updateMode", VehicleAttributeType::token },
        { "physxVehicleContext:verticalAxis", VehicleAttributeType::token },
        { "physxVehicleContext:longitudinalAxis", VehicleAttributeType::token },
        { "physxVehicleContext:upAxis", VehicleAttributeType::float3 },
        { "physxVehicleContext:forwardAxis", VehicleAttributeType::float3 }
    });
    return input;
}

void prepareVehicleContextStorage(const PreparedVehicleContext& input, VehicleContextDesc& output)
{
    (void)input; (void)output;
}

bool fillVehicleContext(const PreparedVehicleContext& input, VehicleContextDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;


    const std::string& ownerName = src.sourceKeyToString(key);

    VehicleContextDesc* desc = &output;
    desc->setDefaultValues();
    desc->sceneKey = key;

    // updateMode → eVelocityChange when token == "velocityChange", else eAcceleration.
    TokenId mode;
    if (!src.getAttribute(key, src.attribute("physxVehicleContext:updateMode"), mode))
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: \"%s\" needs to have token \"updateMode\" defined.", ownerName.c_str());
        return false;
    }
    const std::string_view modeStr = src.tokenToString(mode);
    desc->vehicleUpdateMode = (modeStr == "velocityChange") ? eVelocityChange : eAcceleration;

    if (!readAxisOrDeprecatedVec3(src, key,
                                  "physxVehicleContext:verticalAxis",
                                  "physxVehicleContext:upAxis",
                                  desc->verticalAxis, desc->upAxis,
                                  ownerName, "verticalAxis", status))
        return false;

    if (!readAxisOrDeprecatedVec3(src, key,
                                  "physxVehicleContext:longitudinalAxis",
                                  "physxVehicleContext:forwardAxis",
                                  desc->longitudinalAxis, desc->forwardAxis,
                                  ownerName, "longitudinalAxis", status))
        return false;

    // Same-axis check (deprecated path only): warn when vertical and
    // longitudinal collapse onto the same axis pair.
    static_assert(VehicleContextDesc::ePosX == 0, "");
    static_assert(VehicleContextDesc::eNegX == 1, "");
    static_assert(VehicleContextDesc::ePosY == 2, "");
    static_assert(VehicleContextDesc::eNegY == 3, "");
    static_assert(VehicleContextDesc::ePosZ == 4, "");
    static_assert(VehicleContextDesc::eNegZ == 5, "");
    if (desc->verticalAxis != VehicleContextDesc::eUndefined &&
        desc->longitudinalAxis != VehicleContextDesc::eUndefined &&
        (desc->verticalAxis >> 1) == (desc->longitudinalAxis >> 1))
    {
        VEHICLE_LOG_WARN(status, "Usd Physics: vehicle context \"%s\": tokens \"verticalAxis\" and \"longitudinalAxis\" "
                      "can not use the same axis.", ownerName.c_str());
    }

    return true;
}

void finishVehicleContextStorage(VehicleContextDesc& output, const VehicleFillStatus& status)
{
    (void)output; (void)status;
}

DescPtr<VehicleContextDesc> parseVehicleContext(ParseContext& ctx, ObjectKey key)
{
    const PreparedVehicleContext input = prepareVehicleContext(ctx, key);
    DescPtr<VehicleContextDesc> output = allocateDesc<VehicleContextDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareVehicleContextStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillVehicleContext(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishVehicleContextStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse

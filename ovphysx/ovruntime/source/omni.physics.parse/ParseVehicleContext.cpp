// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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

bool readAxisOrDeprecatedVec3(IPhysicsSource& src, ObjectKey key,
                              const std::string& tokenAttrName,
                              const std::string& deprecatedVec3AttrName,
                              VehicleContextDesc::AxisDir& outAxis,
                              carb::Float3& outVec3,
                              std::string_view ownerName,
                              const char* axisLabel)
{
    TokenId tok;
    if (!src.getAttribute(key, src.internToken(tokenAttrName), tok))
    {
        CARB_LOG_ERROR("Usd Physics: \"%s\" needs to have token \"%s\" defined.",
                       std::string(ownerName).c_str(), axisLabel);
        return false;
    }
    const std::string_view tokStr = src.tokenToString(tok);
    if (tokStr != "undefined")
    {
        if (!tokenToAxisDir(tokStr, outAxis))
        {
            CARB_LOG_ERROR("Usd Physics: \"%s\" has unsupported \"%s\" token \"%s\".",
                           std::string(ownerName).c_str(), axisLabel, std::string(tokStr).c_str());
            return false;
        }
        return true;
    }
    outAxis = VehicleContextDesc::eUndefined;
    if (!src.getAttribute(key, src.internToken(deprecatedVec3AttrName), outVec3))
    {
        CARB_LOG_ERROR("Usd Physics: \"%s\" needs to have attribute \"%s\" defined when \"%s\" is undefined.",
                       std::string(ownerName).c_str(), deprecatedVec3AttrName.c_str(), axisLabel);
        return false;
    }
    CARB_LOG_WARN("Usd Physics: vehicle context \"%s\": attribute \"%s\" is deprecated. Please use \"%s\" instead.",
                  std::string(ownerName).c_str(), deprecatedVec3AttrName.c_str(), axisLabel);
    return true;
}

} // namespace

// @implements REQ-PARSE-VEH-CONTEXT-001
// @covers AC-1 AC-2 AC-3
DescPtr<VehicleContextDesc> parseVehicleContext(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(key));

    DescPtr<VehicleContextDesc> desc = allocateDesc<VehicleContextDesc>(ctx.descriptorAllocator());
    desc->setDefaultValues();
    desc->sceneKey = key;

    // updateMode → eVelocityChange when token == "velocityChange", else eAcceleration.
    TokenId mode;
    if (!src.getAttribute(key, src.internToken("physxVehicleContext:updateMode"), mode))
    {
        CARB_LOG_ERROR("Usd Physics: \"%s\" needs to have token \"updateMode\" defined.", ownerName.c_str());
        return {};
    }
    const std::string_view modeStr = src.tokenToString(mode);
    desc->vehicleUpdateMode = (modeStr == "velocityChange") ? eVelocityChange : eAcceleration;

    if (!readAxisOrDeprecatedVec3(src, key,
                                  "physxVehicleContext:verticalAxis",
                                  "physxVehicleContext:upAxis",
                                  desc->verticalAxis, desc->upAxis,
                                  ownerName, "verticalAxis"))
        return {};

    if (!readAxisOrDeprecatedVec3(src, key,
                                  "physxVehicleContext:longitudinalAxis",
                                  "physxVehicleContext:forwardAxis",
                                  desc->longitudinalAxis, desc->forwardAxis,
                                  ownerName, "longitudinalAxis"))
        return {};

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
        CARB_LOG_WARN("Usd Physics: vehicle context \"%s\": tokens \"verticalAxis\" and \"longitudinalAxis\" "
                      "can not use the same axis.", ownerName.c_str());
    }

    return desc;
}

} // namespace omni::physics::parse

// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-VEH-WHEELATTACH-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleSteeringAPI (single-apply) and
// PhysxVehicleAckermannSteeringAPI (single-apply) readers.
//
// The two APIs are mutually exclusive: if PhysxVehicleSteeringAPI is
// present, basic-steering parser fires; otherwise if
// AckermannSteeringAPI is applied, Ackermann parser fires.
//
// Walker pre-reads wheels (VtArray<int>) + angleMultipliers
// (VtArray<float>) for the basic variant; the Ackermann variant is
// all-scalar.
//
// NonlinearCmdResponse pointer is set to nullptr in both variants;
// the consumer adapter wires it from the matching multi-apply emit.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>
#include <cmath>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

namespace omni::physics::parse
{

// @implements REQ-PARSE-VEH-WHEELATTACH-001
// @covers AC-1 AC-2 AC-3
DescPtr<SteeringBasicDesc> parseSteeringBasic(ParseContext& ctx, ObjectKey key,
                                              const SteeringBasicInfo& info)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(key));

    DescPtr<SteeringBasicDesc> desc = allocateDesc<SteeringBasicDesc>(ctx.descriptorAllocator());
    desc->nonlinearCmdResponse = nullptr;  // wired by consumer adapter from multi-apply emit

    if (!src.getAttribute(key, src.internToken("physxVehicleSteering:maxSteerAngle"), desc->maxSteerAngle))
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"maxSteerAngle\" defined.", ownerName.c_str());
        return {};
    }

    desc->wheels.reserve(info.wheels.size());
    for (int w : info.wheels)
    {
        if (w < 0)
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"wheels\" of steering system \"%s\" can not hold negative values.",
                           ownerName.c_str());
            return {};
        }
        desc->wheels.push_back(w);
    }

    if (info.hasAngleMultipliers)
    {
        if (info.angleMultipliers.size() != desc->wheels.size())
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"angleMultipliers\" of steering system \"%s\" needs to have the same "
                           "number of entries as the \"wheels\" attribute or else should not be defined at all.",
                           ownerName.c_str());
            return {};
        }
        desc->angleMultipliers.reserve(info.angleMultipliers.size());
        for (float m : info.angleMultipliers)
        {
            const float maxAngle = desc->maxSteerAngle * m;
            if (std::fabs(maxAngle) > static_cast<float>(M_PI))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"angleMultipliers\": maxSteerAngle * angleMultiplier "
                               "has to in range [-pi, pi] for all steered wheels.", ownerName.c_str());
                return {};
            }
            desc->angleMultipliers.push_back(m);
        }
    }

    return desc;
}

// @implements REQ-PARSE-VEH-WHEELATTACH-001
// @covers AC-1 AC-2 AC-3
DescPtr<SteeringAckermannDesc> parseSteeringAckermann(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(key));

    DescPtr<SteeringAckermannDesc> desc = allocateDesc<SteeringAckermannDesc>(ctx.descriptorAllocator());
    desc->nonlinearCmdResponse = nullptr;  // wired by consumer adapter from multi-apply emit

    auto readRequiredFloat = [&](const char* attrName, float& out) -> bool {
        if (!src.getAttribute(key, src.internToken(attrName), out))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                           ownerName.c_str(), attrName);
            return false;
        }
        return true;
    };

    auto readRequiredInt = [&](const char* attrName, int& out) -> bool {
        int64_t iv;
        if (!src.getAttribute(key, src.internToken(attrName), iv))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                           ownerName.c_str(), attrName);
            return false;
        }
        out = static_cast<int>(iv);
        return true;
    };

    if (!readRequiredFloat("physxVehicleAckermannSteering:maxSteerAngle", desc->maxSteerAngle)) return {};

    if (!readRequiredInt("physxVehicleAckermannSteering:wheel0", desc->wheel0)) return {};
    if (desc->wheel0 < 0)
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"wheel0\" needs to be non-negative.", ownerName.c_str());
        return {};
    }

    if (!readRequiredInt("physxVehicleAckermannSteering:wheel1", desc->wheel1)) return {};
    if (desc->wheel1 < 0)
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"wheel1\" needs to be non-negative.", ownerName.c_str());
        return {};
    }

    if (desc->wheel0 == desc->wheel1)
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attributes \"wheel0\" and \"wheel1\" can not hold the "
                       "same value.", ownerName.c_str());
        return {};
    }

    if (!readRequiredFloat("physxVehicleAckermannSteering:wheelBase", desc->wheelBase)) return {};
    if (!(desc->wheelBase > 0.0f))
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"wheelBase\" needs to be positive.", ownerName.c_str());
        return {};
    }

    if (!readRequiredFloat("physxVehicleAckermannSteering:trackWidth", desc->trackWidth)) return {};
    if (!(desc->trackWidth > 0.0f))
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"trackWidth\" needs to be positive.", ownerName.c_str());
        return {};
    }

    {
        float v;
        if (src.getAttribute(key, src.internToken("physxVehicleAckermannSteering:strength"), v))
        {
            if (!(v >= 0.0f) || !(v < 1.0f + FLT_EPSILON))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"strength\" out of range.", ownerName.c_str());
                return {};
            }
            desc->strength = v;
        }
        else
        {
            desc->strength = 1.0f;
        }
    }

    return desc;
}

} // namespace omni::physics::parse

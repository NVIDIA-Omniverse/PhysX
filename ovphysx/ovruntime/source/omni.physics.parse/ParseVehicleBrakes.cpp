// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-VEH-WHEELATTACH-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleBrakesAPI (multi-apply per brake instance) reader.
// Walker drives multi-apply iteration via `forEachMultiApplyInstance`
// and calls this once per applied instance (only the `brakes0` /
// `brakes1` instance tokens are honored; walker filters).
//
// Walker pre-reads wheels (VtArray<int>) + torqueMultipliers
// (VtArray<float>) since IPhysicsSource has no array accessor.
// NonlinearCmdResponse pointer is set to nullptr; the consumer
// adapter wires it from the matching multi-apply emit.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>
#include <string>

namespace omni::physics::parse
{

// @implements REQ-PARSE-VEH-WHEELATTACH-001
// @covers AC-1 AC-2 AC-3
DescPtr<BrakesDesc> parseBrakes(ParseContext& ctx, ObjectKey key,
                                std::string_view instanceName, uint8_t brakesIndex,
                                const BrakesInfo& info)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(key));

    DescPtr<BrakesDesc> desc = allocateDesc<BrakesDesc>(ctx.descriptorAllocator());
    desc->brakesIndex = brakesIndex;
    desc->nonlinearCmdResponse = nullptr;  // wired by consumer adapter from multi-apply emit

    const std::string attrBase = std::string("physxVehicleBrakes:") + std::string(instanceName) + ":";

    float mbt;
    if (!src.getAttribute(key, src.internToken(attrBase + "maxBrakeTorque"), mbt))
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"maxBrakeTorque\" defined.", ownerName.c_str());
        return {};
    }
    if (!(mbt >= 0.0f) || !(mbt < FLT_MAX))
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"maxBrakeTorque\" out of range.", ownerName.c_str());
        return {};
    }
    desc->maxBrakeTorque = mbt;

    desc->wheels.reserve(info.wheels.size());
    for (int w : info.wheels)
    {
        if (w < 0)
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"wheels\" of braking system \"%s\" can not hold negative values.",
                           std::string(instanceName).c_str());
            return {};
        }
        desc->wheels.push_back(w);
    }

    if (info.hasTorqueMultipliers)
    {
        if (info.torqueMultipliers.size() != desc->wheels.size())
        {
            CARB_LOG_ERROR("Usd Physics: attribute \"torqueMultipliers\" of braking system \"%s\" needs to have the same "
                           "number of entries as the \"wheels\" attribute or else should not be defined at all.",
                           std::string(instanceName).c_str());
            return {};
        }
        desc->torqueMultipliers.reserve(info.torqueMultipliers.size());
        for (float m : info.torqueMultipliers)
        {
            if (m < 0.0f)
            {
                CARB_LOG_ERROR("Usd Physics: attribute \"torqueMultipliers\" of braking system \"%s\" can not hold "
                               "negative values.", std::string(instanceName).c_str());
                return {};
            }
            desc->torqueMultipliers.push_back(m);
        }
    }

    return desc;
}

} // namespace omni::physics::parse

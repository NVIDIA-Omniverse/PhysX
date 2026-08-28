// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-VEH-ROOT-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleAPI chassis-level reader. Reads all ~25 scalar / bool /
// uint8 attrs from the vehicle prim. Cross-references (drive /
// differential / steering / brakes) are pre-resolved by the walker via
// rel-or-API and passed via VehicleInfo. Wheel attachments + wheel
// controllers + controller initial state stay walker-side / consumer-
// side respectively — parseVehicle here only fills the scalar chassis
// fields.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

namespace
{

constexpr int kMaxSubstepCount = 255;

bool readFloatRequired(IPhysicsSource& src, ObjectKey key, const char* attrName,
                       const std::string& owner, float& out)
{
    if (!src.getAttribute(key, src.internToken(attrName), out))
    {
        CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                       owner.c_str(), attrName);
        return false;
    }
    return true;
}

} // namespace

// @implements REQ-PARSE-VEH-ROOT-001
// @covers AC-1 AC-2 AC-3
DescPtr<VehicleDesc> parseVehicle(ParseContext& ctx, ObjectKey key, const VehicleInfo& info)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(key));

    DescPtr<VehicleDesc> desc = allocateDesc<VehicleDesc>(ctx.descriptorAllocator());
    desc->bodyId = kInvalidObjectId;
    desc->drive = nullptr;
    desc->differential = nullptr;
    desc->steering = nullptr;
    desc->scale = info.scale;
    desc->referenceFrameIsCenterOfMass = info.referenceFrameIsCenterOfMass;
    desc->queryType = info.queryType;
    desc->hasUserDefinedSprungMassValues = false;
    desc->hasUserDefinedMaxDroopValues = false;
    desc->hasUserDefinedRestLoadValues = false;
    desc->isUsingDeprecatedLatStiffY = false;
    // Deprecated field used by the deprecated-attr fallback path.
    desc->minLongitudinalSlipDenominator = 0.0f;

    // vehicleEnabled (bool, required by schema fallback)
    desc->enabled = true;
    src.getAttribute(key, src.internToken("physxVehicle:vehicleEnabled"), desc->enabled);

    // limitSuspensionExpansionVelocity (bool)
    desc->limitSuspensionExpansionVelocity = false;
    src.getAttribute(key, src.internToken("physxVehicle:limitSuspensionExpansionVelocity"),
                     desc->limitSuspensionExpansionVelocity);

    // subStepThresholdLongitudinalSpeed: default 5.0 * lengthScale
    {
        float v;
        if (src.getAttribute(key, src.internToken("physxVehicle:subStepThresholdLongitudinalSpeed"), v))
        {
            if (!(v >= 0.0f) || !(v < FLT_MAX))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"subStepThresholdLongitudinalSpeed\" out of range.",
                               ownerName.c_str());
                return {};
            }
            desc->subStepThresholdLongitudinalSpeed = v;
        }
        else
        {
            desc->subStepThresholdLongitudinalSpeed = 5.0f * info.lengthScale;
        }
    }

    auto readSubStepCount = [&](const char* attrName, int defaultVal, int& out) -> bool {
        int64_t iv;
        if (src.getAttribute(key, src.internToken(attrName), iv))
        {
            int n = static_cast<int>(iv);
            if (!(n > 0))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\" needs to be positive.",
                               ownerName.c_str(), attrName);
                return false;
            }
            if (n > kMaxSubstepCount)
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\" needs to be smaller or equal %d. "
                               "Value will be clamped.", ownerName.c_str(), attrName, kMaxSubstepCount);
                n = kMaxSubstepCount;
            }
            out = n;
        }
        else
        {
            out = defaultVal;
        }
        return true;
    };

    if (!readSubStepCount("physxVehicle:lowForwardSpeedSubStepCount",  3, desc->lowForwardSpeedSubStepCount))  return {};
    if (!readSubStepCount("physxVehicle:highForwardSpeedSubStepCount", 1, desc->highForwardSpeedSubStepCount)) return {};

    // minPassiveLongitudinalSlipDenominator: required; sentinel 0 →
    // fall back to deprecated minLongitudinalSlipDenominator (default
    // 4.0 * lengthScale).
    {
        float v;
        if (!src.getAttribute(key, src.internToken("physxVehicle:minPassiveLongitudinalSlipDenominator"), v))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"minPassiveLongitudinalSlipDenominator\" defined.",
                           ownerName.c_str());
            return {};
        }
        desc->minPassiveLongitudinalSlipDenominator = v;
        if (v != 0.0f)
        {
            if (!(v > 0.0f))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"minPassiveLongitudinalSlipDenominator\" needs to be positive.",
                               ownerName.c_str());
                return {};
            }
        }
        else
        {
            CARB_LOG_WARN("Usd Physics: prim \"%s\": attribute \"minLongitudinalSlipDenominator\" "
                          "is deprecated. Please use minPassiveLongitudinalSlipDenominator instead.", ownerName.c_str());
            float dv;
            if (src.getAttribute(key, src.internToken("physxVehicle:minLongitudinalSlipDenominator"), dv))
            {
                if (!(dv >= 0.0f) || !(dv < FLT_MAX))
                {
                    CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"minLongitudinalSlipDenominator\" out of range.",
                                   ownerName.c_str());
                    return {};
                }
                desc->minLongitudinalSlipDenominator = dv;
            }
            else
            {
                desc->minLongitudinalSlipDenominator = 4.0f * info.lengthScale;
            }
        }
    }

    auto readActiveSlipOrDefault = [&](const char* attrName, float scaledDefault, float& outField) -> bool {
        float v;
        if (!src.getAttribute(key, src.internToken(attrName), v))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                           ownerName.c_str(), attrName);
            return false;
        }
        if (v != 0.0f)
        {
            if (!(v > 0.0f))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\" needs to be positive.",
                               ownerName.c_str(), attrName);
                return false;
            }
            outField = v;
        }
        else
        {
            outField = scaledDefault;
        }
        return true;
    };

    if (!readActiveSlipOrDefault("physxVehicle:minActiveLongitudinalSlipDenominator",
                                 0.1f * info.lengthScale, desc->minActiveLongitudinalSlipDenominator))
        return {};
    if (!readActiveSlipOrDefault("physxVehicle:minLateralSlipDenominator",
                                 1.0f * info.lengthScale, desc->minLateralSlipDenominator))
        return {};

    auto readStickyThresholdSpeed = [&](const char* attrName, float& outField) -> bool {
        float v;
        if (!src.getAttribute(key, src.internToken(attrName), v))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                           ownerName.c_str(), attrName);
            return false;
        }
        if (v != -1.0f)
        {
            if (!(v >= 0.0f))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\" needs to be non-negative.",
                               ownerName.c_str(), attrName);
                return false;
            }
            outField = v;
        }
        else
        {
            outField = 0.2f * info.lengthScale;
        }
        return true;
    };

    if (!readStickyThresholdSpeed("physxVehicle:longitudinalStickyTireThresholdSpeed",
                                  desc->longitudinalStickyTireThresholdSpeed))
        return {};

    auto readNonNegFloat = [&](const char* attrName, float& out) -> bool {
        float v;
        if (!src.getAttribute(key, src.internToken(attrName), v))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                           ownerName.c_str(), attrName);
            return false;
        }
        if (!(v >= 0.0f) || !(v < FLT_MAX))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"%s\" out of range.",
                           ownerName.c_str(), attrName);
            return false;
        }
        out = v;
        return true;
    };

    if (!readNonNegFloat("physxVehicle:longitudinalStickyTireThresholdTime",  desc->longitudinalStickyTireThresholdTime))  return {};
    if (!readNonNegFloat("physxVehicle:longitudinalStickyTireDamping",        desc->longitudinalStickyTireDamping))        return {};

    if (!readStickyThresholdSpeed("physxVehicle:lateralStickyTireThresholdSpeed",
                                  desc->lateralStickyTireThresholdSpeed))
        return {};

    // lateralStickyTireThresholdTime + lateralStickyTireDamping —
    // note ThresholdTime reads the longitudinal attribute (the
    // schema-side name has a documented historical mismatch).
    if (!readNonNegFloat("physxVehicle:longitudinalStickyTireThresholdTime",  desc->lateralStickyTireThresholdTime))  return {};
    if (!readNonNegFloat("physxVehicle:lateralStickyTireDamping",             desc->lateralStickyTireDamping))        return {};

    return desc;
}

} // namespace omni::physics::parse

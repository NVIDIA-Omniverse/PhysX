// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

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
#include <omni/physics/parse/PreparedVehicle.h>
#include <stdexcept>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

namespace
{

constexpr int kMaxSubstepCount = 255;

bool readFloatRequired(PreparedVehicleReader& src, ObjectKey key, const char* attrName,
                       const std::string& owner, float& out, VehicleFillStatus& status)
{
    if (!src.getAttribute(key, src.attribute(attrName), out))
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                       owner.c_str(), attrName);
        return false;
    }
    return true;
}

} // namespace

// @implements REQ-PARSE-VEH-ROOT-001
// @covers AC-1 AC-2 AC-3
PreparedVehicle prepareVehicle(ParseContext& ctx, ObjectKey key, const VehicleInfo& info)
{
    PreparedVehicle input;
    input.key = key;
    input.info = info;
    input.reads = prepareVehicleReads(ctx.source(), key, {
        { "physxVehicle:highForwardSpeedSubStepCount", VehicleAttributeType::integer },
        { "physxVehicle:lateralStickyTireDamping", VehicleAttributeType::floating },
        { "physxVehicle:lateralStickyTireThresholdSpeed", VehicleAttributeType::floating },
        { "physxVehicle:limitSuspensionExpansionVelocity", VehicleAttributeType::boolean },
        { "physxVehicle:longitudinalStickyTireDamping", VehicleAttributeType::floating },
        { "physxVehicle:longitudinalStickyTireThresholdSpeed", VehicleAttributeType::floating },
        { "physxVehicle:longitudinalStickyTireThresholdTime", VehicleAttributeType::floating },
        { "physxVehicle:lowForwardSpeedSubStepCount", VehicleAttributeType::integer },
        { "physxVehicle:minActiveLongitudinalSlipDenominator", VehicleAttributeType::floating },
        { "physxVehicle:minLateralSlipDenominator", VehicleAttributeType::floating },
        { "physxVehicle:minLongitudinalSlipDenominator", VehicleAttributeType::floating },
        { "physxVehicle:minPassiveLongitudinalSlipDenominator", VehicleAttributeType::floating },
        { "physxVehicle:subStepThresholdLongitudinalSpeed", VehicleAttributeType::floating },
        { "physxVehicle:vehicleEnabled", VehicleAttributeType::boolean }
    });
    return input;
}

void prepareVehicleStorage(const PreparedVehicle& input, VehicleDesc& output)
{
    (void)input; (void)output;
}

bool fillVehicle(const PreparedVehicle& input, VehicleDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;
    [[maybe_unused]] const VehicleInfo& info = input.info;


    const std::string& ownerName = src.sourceKeyToString(key);

    VehicleDesc* desc = &output;
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
    src.getAttribute(key, src.attribute("physxVehicle:vehicleEnabled"), desc->enabled);

    // limitSuspensionExpansionVelocity (bool)
    desc->limitSuspensionExpansionVelocity = false;
    src.getAttribute(key, src.attribute("physxVehicle:limitSuspensionExpansionVelocity"),
                     desc->limitSuspensionExpansionVelocity);

    // subStepThresholdLongitudinalSpeed: default 5.0 * lengthScale
    {
        float v;
        if (src.getAttribute(key, src.attribute("physxVehicle:subStepThresholdLongitudinalSpeed"), v))
        {
            if (!(v >= 0.0f) || !(v < FLT_MAX))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"subStepThresholdLongitudinalSpeed\" out of range.",
                               ownerName.c_str());
                return false;
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
        if (src.getAttribute(key, src.attribute(attrName), iv))
        {
            int n = static_cast<int>(iv);
            if (!(n > 0))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s\" needs to be positive.",
                               ownerName.c_str(), attrName);
                return false;
            }
            if (n > kMaxSubstepCount)
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s\" needs to be smaller or equal %d. "
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

    if (!readSubStepCount("physxVehicle:lowForwardSpeedSubStepCount",  3, desc->lowForwardSpeedSubStepCount))  return false;
    if (!readSubStepCount("physxVehicle:highForwardSpeedSubStepCount", 1, desc->highForwardSpeedSubStepCount)) return false;

    // minPassiveLongitudinalSlipDenominator: required; sentinel 0 →
    // fall back to deprecated minLongitudinalSlipDenominator (default
    // 4.0 * lengthScale).
    {
        float v;
        if (!src.getAttribute(key, src.attribute("physxVehicle:minPassiveLongitudinalSlipDenominator"), v))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"minPassiveLongitudinalSlipDenominator\" defined.",
                           ownerName.c_str());
            return false;
        }
        desc->minPassiveLongitudinalSlipDenominator = v;
        if (v != 0.0f)
        {
            if (!(v > 0.0f))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"minPassiveLongitudinalSlipDenominator\" needs to be positive.",
                               ownerName.c_str());
                return false;
            }
        }
        else
        {
            VEHICLE_LOG_WARN(status, "Usd Physics: prim \"%s\": attribute \"minLongitudinalSlipDenominator\" "
                          "is deprecated. Please use minPassiveLongitudinalSlipDenominator instead.", ownerName.c_str());
            float dv;
            if (src.getAttribute(key, src.attribute("physxVehicle:minLongitudinalSlipDenominator"), dv))
            {
                if (!(dv >= 0.0f) || !(dv < FLT_MAX))
                {
                    VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"minLongitudinalSlipDenominator\" out of range.",
                                   ownerName.c_str());
                    return false;
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
        if (!src.getAttribute(key, src.attribute(attrName), v))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                           ownerName.c_str(), attrName);
            return false;
        }
        if (v != 0.0f)
        {
            if (!(v > 0.0f))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s\" needs to be positive.",
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
        return false;
    if (!readActiveSlipOrDefault("physxVehicle:minLateralSlipDenominator",
                                 1.0f * info.lengthScale, desc->minLateralSlipDenominator))
        return false;

    auto readStickyThresholdSpeed = [&](const char* attrName, float& outField) -> bool {
        float v;
        if (!src.getAttribute(key, src.attribute(attrName), v))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                           ownerName.c_str(), attrName);
            return false;
        }
        if (v != -1.0f)
        {
            if (!(v >= 0.0f))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s\" needs to be non-negative.",
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
        return false;

    auto readNonNegFloat = [&](const char* attrName, float& out) -> bool {
        float v;
        if (!src.getAttribute(key, src.attribute(attrName), v))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"%s\" defined.",
                           ownerName.c_str(), attrName);
            return false;
        }
        if (!(v >= 0.0f) || !(v < FLT_MAX))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"%s\" out of range.",
                           ownerName.c_str(), attrName);
            return false;
        }
        out = v;
        return true;
    };

    if (!readNonNegFloat("physxVehicle:longitudinalStickyTireThresholdTime",  desc->longitudinalStickyTireThresholdTime))  return false;
    if (!readNonNegFloat("physxVehicle:longitudinalStickyTireDamping",        desc->longitudinalStickyTireDamping))        return false;

    if (!readStickyThresholdSpeed("physxVehicle:lateralStickyTireThresholdSpeed",
                                  desc->lateralStickyTireThresholdSpeed))
        return false;

    // lateralStickyTireThresholdTime + lateralStickyTireDamping —
    // note ThresholdTime reads the longitudinal attribute (the
    // schema-side name has a documented historical mismatch).
    if (!readNonNegFloat("physxVehicle:longitudinalStickyTireThresholdTime",  desc->lateralStickyTireThresholdTime))  return false;
    if (!readNonNegFloat("physxVehicle:lateralStickyTireDamping",             desc->lateralStickyTireDamping))        return false;

    return true;
}

void finishVehicleStorage(VehicleDesc& output, const VehicleFillStatus& status)
{
    (void)output; (void)status;
}

DescPtr<VehicleDesc> parseVehicle(ParseContext& ctx, ObjectKey key, const VehicleInfo& info)
{
    const PreparedVehicle input = prepareVehicle(ctx, key, info);
    DescPtr<VehicleDesc> output = allocateDesc<VehicleDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareVehicleStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillVehicle(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishVehicleStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse

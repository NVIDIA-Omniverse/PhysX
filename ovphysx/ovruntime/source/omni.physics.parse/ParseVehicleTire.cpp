// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-VEH-COMPONENTS-001
 * @covers AC-1 AC-2 AC-3
 */

// PhysxVehicleTireAPI reader. Per-component parser for the shareable
// tire descriptor (stiffness parameters + friction-vs-slip graph +
// friction-table relationship + restLoad). Includes the deprecated
// lateral / longitudinal / camber stiffness fallback chains.
//
// Walker pre-reads `frictionVsSlipGraph` (VtArray<GfVec2f>, exactly
// three entries when authored) and the `frictionTable` relationship
// target (single ObjectKey or empty), passes them via TireInfo since
// IPhysicsSource has no float[2] array accessor.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedVehicle.h>
#include <stdexcept>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

// @implements REQ-PARSE-VEH-COMPONENTS-001
// @covers AC-1 AC-2 AC-3
PreparedTire prepareTire(ParseContext& ctx, ObjectKey key, const TireInfo& info)
{
    PreparedTire input;
    input.key = key;
    input.info = info;
    input.reads = prepareVehicleReads(ctx.source(), key, {
        { "physxVehicleTire:camberStiffness", VehicleAttributeType::floating },
        { "physxVehicleTire:camberStiffnessPerUnitGravity", VehicleAttributeType::floating },
        { "physxVehicleTire:latStiffX", VehicleAttributeType::floating },
        { "physxVehicleTire:latStiffY", VehicleAttributeType::floating },
        { "physxVehicleTire:lateralStiffnessGraph", VehicleAttributeType::float2 },
        { "physxVehicleTire:longitudinalStiffness", VehicleAttributeType::floating },
        { "physxVehicleTire:longitudinalStiffnessPerUnitGravity", VehicleAttributeType::floating },
        { "physxVehicleTire:restLoad", VehicleAttributeType::floating }
    });
    return input;
}

void prepareTireStorage(const PreparedTire& input, TireDesc& output)
{
    (void)input; (void)output;
}

bool fillTire(const PreparedTire& input, TireDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    [[maybe_unused]] ObjectKey key = input.key;
    [[maybe_unused]] const TireInfo& info = input.info;


    const std::string& ownerName = src.sourceKeyToString(key);

    TireDesc* desc = &output;
    desc->key = key;
    desc->latStiffX = 0.0f;
    desc->latStiffY = 0.0f;
    desc->lateralStiffnessGraph = { 0.0f, 0.0f };
    desc->longitudinalStiffnessPerUnitGravity = 0.0f;
    desc->longitudinalStiffness = 0.0f;
    desc->camberStiffnessPerUnitGravity = 0.0f;
    desc->camberStiffness = 0.0f;
    desc->frictionTableId = kInvalidObjectId;
    desc->restLoad = 0.0f;

    // lateralStiffnessGraph: required (Vec2). If (0,0), fall back to
    // deprecated latStiffX/latStiffY.
    {
        carb::Float2 graph;
        if (!src.getAttribute(key, src.attribute("physxVehicleTire:lateralStiffnessGraph"), graph))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"lateralStiffnessGraph\" defined.",
                           ownerName.c_str());
            return false;
        }
        desc->lateralStiffnessGraph = graph;
        if (graph.x != 0.0f || graph.y != 0.0f)
        {
            if (graph.x < 0.0f)
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"lateralStiffnessGraph.x\" needs to be non-negative.",
                               ownerName.c_str());
                return false;
            }
            if (!(graph.y > 0.0f))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"lateralStiffnessGraph.y\" needs to be positive.",
                               ownerName.c_str());
                return false;
            }
        }
        else
        {
            VEHICLE_LOG_WARN(status, "Usd Physics: tire \"%s\": attributes \"latStiffX\" and \"latStiffY\" "
                          "are deprecated. Please use lateralStiffnessGraph instead.", ownerName.c_str());
            float lx;
            if (!src.getAttribute(key, src.attribute("physxVehicleTire:latStiffX"), lx))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"latStiffX\" defined.",
                               ownerName.c_str());
                return false;
            }
            if (!(lx >= 0.0f) || !(lx < FLT_MAX))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"latStiffX\" out of range.", ownerName.c_str());
                return false;
            }
            desc->latStiffX = lx;
            float ly;
            if (src.getAttribute(key, src.attribute("physxVehicleTire:latStiffY"), ly))
            {
                if (!(ly >= 0.0f) || !(ly < FLT_MAX))
                {
                    VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"latStiffY\" out of range.", ownerName.c_str());
                    return false;
                }
                desc->latStiffY = ly;
            }
            else
            {
                desc->latStiffY = 17.095f;
            }
        }
    }

    // longitudinalStiffness: required; 0 means fall back to deprecated
    // longitudinalStiffnessPerUnitGravity (default 500 * massScale).
    {
        float v;
        if (!src.getAttribute(key, src.attribute("physxVehicleTire:longitudinalStiffness"), v))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"longitudinalStiffness\" defined.",
                           ownerName.c_str());
            return false;
        }
        desc->longitudinalStiffness = v;
        if (v != 0.0f)
        {
            if (!(v > 0.0f))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"longitudinalStiffness\" needs to be positive.",
                               ownerName.c_str());
                return false;
            }
        }
        else
        {
            VEHICLE_LOG_WARN(status, "Usd Physics: tire \"%s\": attribute \"longitudinalStiffnessPerUnitGravity\" "
                          "is deprecated. Please use lateralStiffnessGraph instead.", ownerName.c_str());
            float dv;
            if (src.getAttribute(key, src.attribute("physxVehicleTire:longitudinalStiffnessPerUnitGravity"), dv))
            {
                if (!(dv >= 0.0f) || !(dv < FLT_MAX))
                {
                    VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"longitudinalStiffnessPerUnitGravity\" out of range.",
                                   ownerName.c_str());
                    return false;
                }
                desc->longitudinalStiffnessPerUnitGravity = dv;
            }
            else
            {
                desc->longitudinalStiffnessPerUnitGravity = 500.0f * info.massScale;
            }
        }
    }

    // camberStiffness: required; sentinel -1 means fall back to
    // deprecated camberStiffnessPerUnitGravity.
    {
        float v;
        if (!src.getAttribute(key, src.attribute("physxVehicleTire:camberStiffness"), v))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"camberStiffness\" defined.",
                           ownerName.c_str());
            return false;
        }
        desc->camberStiffness = v;
        if (v != -1.0f)
        {
            if (v < 0.0f)
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"camberStiffness\" needs to be non-negative.",
                               ownerName.c_str());
                return false;
            }
        }
        else
        {
            VEHICLE_LOG_WARN(status, "Usd Physics: tire \"%s\": attribute \"camberStiffnessPerUnitGravity\" "
                          "is deprecated. Please use camberStiffness instead.", ownerName.c_str());
            float dv;
            if (!src.getAttribute(key, src.attribute("physxVehicleTire:camberStiffnessPerUnitGravity"), dv))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"camberStiffnessPerUnitGravity\" defined.",
                               ownerName.c_str());
                return false;
            }
            if (!(dv >= 0.0f) || !(dv < FLT_MAX))
            {
                VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"camberStiffnessPerUnitGravity\" out of range.",
                               ownerName.c_str());
                return false;
            }
            desc->camberStiffnessPerUnitGravity = dv;
        }
    }

    // frictionVsSlipGraph: optional, walker pre-read. Schema default is
    // the (0,1)(0.1,1)(1,1) triple. Walker passes the authored values
    // when present (3 entries); empty means default.
    if (info.hasFrictionVsSlipGraph)
    {
        for (uint32_t i = 0; i < 3; ++i)
            desc->frictionVsSlipGraph[i] = info.frictionVsSlipGraph[i];
        if (desc->frictionVsSlipGraph[0].x >= desc->frictionVsSlipGraph[1].x ||
            desc->frictionVsSlipGraph[1].x >= desc->frictionVsSlipGraph[2].x)
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: attribute \"frictionVsSlipGraph\" of tire \"%s\" has invalid values: "
                           "frictionVsSlipGraph[2].x > frictionVsSlipGraph[1].x > frictionVsSlipGraph[0].x has to hold.",
                           ownerName.c_str());
            return false;
        }
    }
    else
    {
        desc->frictionVsSlipGraph[0] = { 0.0f, 1.0f };
        desc->frictionVsSlipGraph[1] = { 0.1f, 1.0f };
        desc->frictionVsSlipGraph[2] = { 1.0f, 1.0f };
    }

    // frictionTable: walker pre-resolved single rel target with
    // PhysxVehicleTireFrictionTable type. info.frictionTableTooMany is
    // set when multiple targets were authored — reject the whole tire.
    if (info.frictionTableTooMany)
        return false;
    desc->frictionTableKey = info.frictionTableKey;

    // restLoad: required, non-negative.
    {
        float v;
        if (!src.getAttribute(key, src.attribute("physxVehicleTire:restLoad"), v))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\" needs to have attribute \"restLoad\" defined.", ownerName.c_str());
            return false;
        }
        if (!(v >= 0.0f) || !(v < FLT_MAX))
        {
            VEHICLE_LOG_ERROR(status, "Usd Physics: prim \"%s\": attribute \"restLoad\" out of range.", ownerName.c_str());
            return false;
        }
        desc->restLoad = v;
    }

    return true;
}

void finishTireStorage(TireDesc& output, const VehicleFillStatus& status)
{
    (void)output; (void)status;
}

DescPtr<TireDesc> parseTire(ParseContext& ctx, ObjectKey key, const TireInfo& info)
{
    const PreparedTire input = prepareTire(ctx, key, info);
    DescPtr<TireDesc> output = allocateDesc<TireDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareTireStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillTire(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishTireStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse

// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cfloat>

namespace omni::physics::parse
{

// @implements REQ-PARSE-VEH-COMPONENTS-001
// @covers AC-1 AC-2 AC-3
DescPtr<TireDesc> parseTire(ParseContext& ctx, ObjectKey key, const TireInfo& info)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerName(src.sourceKeyToString(key));

    DescPtr<TireDesc> desc = allocateDesc<TireDesc>(ctx.descriptorAllocator());
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
        if (!src.getAttribute(key, src.internToken("physxVehicleTire:lateralStiffnessGraph"), graph))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"lateralStiffnessGraph\" defined.",
                           ownerName.c_str());
            return {};
        }
        desc->lateralStiffnessGraph = graph;
        if (graph.x != 0.0f || graph.y != 0.0f)
        {
            if (graph.x < 0.0f)
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"lateralStiffnessGraph.x\" needs to be non-negative.",
                               ownerName.c_str());
                return {};
            }
            if (!(graph.y > 0.0f))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"lateralStiffnessGraph.y\" needs to be positive.",
                               ownerName.c_str());
                return {};
            }
        }
        else
        {
            CARB_LOG_WARN("Usd Physics: tire \"%s\": attributes \"latStiffX\" and \"latStiffY\" "
                          "are deprecated. Please use lateralStiffnessGraph instead.", ownerName.c_str());
            float lx;
            if (!src.getAttribute(key, src.internToken("physxVehicleTire:latStiffX"), lx))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"latStiffX\" defined.",
                               ownerName.c_str());
                return {};
            }
            if (!(lx >= 0.0f) || !(lx < FLT_MAX))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"latStiffX\" out of range.", ownerName.c_str());
                return {};
            }
            desc->latStiffX = lx;
            float ly;
            if (src.getAttribute(key, src.internToken("physxVehicleTire:latStiffY"), ly))
            {
                if (!(ly >= 0.0f) || !(ly < FLT_MAX))
                {
                    CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"latStiffY\" out of range.", ownerName.c_str());
                    return {};
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
        if (!src.getAttribute(key, src.internToken("physxVehicleTire:longitudinalStiffness"), v))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"longitudinalStiffness\" defined.",
                           ownerName.c_str());
            return {};
        }
        desc->longitudinalStiffness = v;
        if (v != 0.0f)
        {
            if (!(v > 0.0f))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"longitudinalStiffness\" needs to be positive.",
                               ownerName.c_str());
                return {};
            }
        }
        else
        {
            CARB_LOG_WARN("Usd Physics: tire \"%s\": attribute \"longitudinalStiffnessPerUnitGravity\" "
                          "is deprecated. Please use lateralStiffnessGraph instead.", ownerName.c_str());
            float dv;
            if (src.getAttribute(key, src.internToken("physxVehicleTire:longitudinalStiffnessPerUnitGravity"), dv))
            {
                if (!(dv >= 0.0f) || !(dv < FLT_MAX))
                {
                    CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"longitudinalStiffnessPerUnitGravity\" out of range.",
                                   ownerName.c_str());
                    return {};
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
        if (!src.getAttribute(key, src.internToken("physxVehicleTire:camberStiffness"), v))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"camberStiffness\" defined.",
                           ownerName.c_str());
            return {};
        }
        desc->camberStiffness = v;
        if (v != -1.0f)
        {
            if (v < 0.0f)
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"camberStiffness\" needs to be non-negative.",
                               ownerName.c_str());
                return {};
            }
        }
        else
        {
            CARB_LOG_WARN("Usd Physics: tire \"%s\": attribute \"camberStiffnessPerUnitGravity\" "
                          "is deprecated. Please use camberStiffness instead.", ownerName.c_str());
            float dv;
            if (!src.getAttribute(key, src.internToken("physxVehicleTire:camberStiffnessPerUnitGravity"), dv))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"camberStiffnessPerUnitGravity\" defined.",
                               ownerName.c_str());
                return {};
            }
            if (!(dv >= 0.0f) || !(dv < FLT_MAX))
            {
                CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"camberStiffnessPerUnitGravity\" out of range.",
                               ownerName.c_str());
                return {};
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
            CARB_LOG_ERROR("Usd Physics: attribute \"frictionVsSlipGraph\" of tire \"%s\" has invalid values: "
                           "frictionVsSlipGraph[2].x > frictionVsSlipGraph[1].x > frictionVsSlipGraph[0].x has to hold.",
                           ownerName.c_str());
            return {};
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
        return {};
    desc->frictionTableKey = info.frictionTableKey;

    // restLoad: required, non-negative.
    {
        float v;
        if (!src.getAttribute(key, src.internToken("physxVehicleTire:restLoad"), v))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\" needs to have attribute \"restLoad\" defined.", ownerName.c_str());
            return {};
        }
        if (!(v >= 0.0f) || !(v < FLT_MAX))
        {
            CARB_LOG_ERROR("Usd Physics: prim \"%s\": attribute \"restLoad\" out of range.", ownerName.c_str());
            return {};
        }
        desc->restLoad = v;
    }

    return desc;
}

} // namespace omni::physics::parse

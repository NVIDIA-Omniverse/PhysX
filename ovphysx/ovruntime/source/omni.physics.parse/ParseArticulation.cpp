// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-ART-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1 AC-2
 */

// Articulation extension parser. Mirrors the PhysxArticulationAPI override
// block in omni.physx Articulation.cpp::parseArticulation.

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/KnownTokens.h>

#include <cfloat>

namespace omni::physics::parse
{

namespace
{

template <typename T>
T readClampedScalar(const IPhysicsSource& src, ObjectKey key, TokenId attr,
                    T defaultVal, T minVal, T maxVal)
{
    T result;
    if constexpr (std::is_same_v<T, int>)
    {
        int64_t tmp;
        if (!src.getAttribute(key, attr, tmp))
            return defaultVal;
        result = static_cast<int>(tmp);
    }
    else
    {
        if (!src.getAttribute(key, attr, result))
            return defaultVal;
    }
    if (result < minVal) result = minVal;
    if (result > maxVal) result = maxVal;
    return result;
}

} // namespace

void setToDefault(ArticulationFields& fields, const SourceUnits& units)
{
    // Units-aware overlay only. solver*IterationCount, selfCollision and
    // articulationEnabled are default-initialised in ArticulationFields
    // (ParseApi.h).
    const float tolerancesSpeed = 10.0f / units.metersPerUnit;
    fields.sleepThreshold         = 5e-5f * tolerancesSpeed * tolerancesSpeed;
    fields.stabilizationThreshold = 1e-2f * tolerancesSpeed * tolerancesSpeed;
}

void parseArticulation(ParseContext& ctx, ObjectKey key, ArticulationFields& fields)
{
    IPhysicsSource& src = ctx.source();
    KnownTokens tok;
    tok.intern(src);

    // Whether the API is applied dictates whether overrides happen at all.
    const bool hasPhysxApi = src.hasSchema(key, tok.physxArticulationAPI);

    if (hasPhysxApi)
    {
        src.getAttribute(key, tok.physxArticulationEnabled, fields.articulationEnabled);

        // Clamp each authored value to PhysX's documented valid range.
        fields.sleepThreshold = readClampedScalar<float>(
            src, key, tok.physxArticulationSleepThreshold,
            fields.sleepThreshold, 0.0f, FLT_MAX);
        fields.stabilizationThreshold = readClampedScalar<float>(
            src, key, tok.physxArticulationStabilizationThreshold,
            fields.stabilizationThreshold, 0.0f, FLT_MAX);
        fields.solverPositionIterationCount = readClampedScalar<int>(
            src, key, tok.physxArticulationSolverPositionIterationCount,
            fields.solverPositionIterationCount, 1, 255);
        fields.solverVelocityIterationCount = readClampedScalar<int>(
            src, key, tok.physxArticulationSolverVelocityIterationCount,
            fields.solverVelocityIterationCount, 0, 255);

        src.getAttribute(key, tok.physxArticulationEnabledSelfCollisions, fields.selfCollision);
    }

    // Newton fallback: when the PhysxArticulationAPI selfCollisions isn't
    // authored, use newton:selfCollisionEnabled. Legacy checks
    // HasAuthoredValue(); we proxy that via getAttribute() returning invalid
    // when nothing is authored.
    if (!hasPhysxApi || !src.getAttribute(key, tok.physxArticulationEnabledSelfCollisions).valid())
    {
        src.getAttribute(key, tok.newtonSelfCollisionEnabled, fields.selfCollision);
    }
}

} // namespace omni::physics::parse

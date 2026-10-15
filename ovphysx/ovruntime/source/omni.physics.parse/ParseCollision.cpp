// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-PARSE-COL-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-PARSE-COL-002
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-PARSE-COL-003
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-PARSE-SHAPE-002
 * @covers AC-1
 *
 * @implements REQ-PARSE-CORE-003
 * @covers AC-2
 */

// Collision extension parser — covers the PhysxCollisionAPI fields that
// have no time-sample-registration dependency. Full collision parsing
// (shape geometry, mesh data via BufferHandle, cooking integration,
// contactOffset/restOffset inf-sentinel logic) lives in the consumer-
// side walker.

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedCollision.h>
#include <omni/physics/parse/KnownTokens.h>

#include <cfloat>
#include <cmath>

namespace omni::physics::parse
{

namespace
{

float readPreparedClampedFloat(const PreparedAttribute<float>& input, float defaultVal,
                               float minVal, float maxVal) noexcept
{
    float result = input.valueOr(defaultVal);
    if (result < minVal) result = minVal;
    if (result > maxVal) result = maxVal;
    return result;
}

bool readPreparedUint32(const PreparedAttribute<int64_t>& input, uint32_t& out) noexcept
{
    if (!input.valid || input.value < 0 || uint64_t(input.value) > UINT32_MAX) return false;
    out = static_cast<uint32_t>(input.value);
    return true;
}

void readPreparedWeld(const PreparedAttribute<float>& input, float& out) noexcept
{
    input.read(out);
    if (std::isnan(out)) out = -FLT_MAX;
}

} // namespace

void parseCollisionExt(ParseContext& ctx, ObjectKey key, CollisionExtFields& fields)
{
    fillCollisionExt(prepareCollisionExt(ctx, key), fields);
}

PreparedCollisionExt prepareCollisionExt(ParseContext& ctx, ObjectKey key)
{
    const IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();
    PreparedCollisionExt input;
    input.physxCollisionAPI = src.hasSchema(key, tok.physxCollisionAPI);
    input.physxTriggerAPI = src.hasSchema(key, tok.physxTriggerAPI);
    input.physxTriggerStateAPI = src.hasSchema(key, tok.physxTriggerStateAPI);
    readCollisionExtAttributes(src, key, tok, input);
    return input;
}

bool fillCollisionExt(const PreparedCollisionExt& input, CollisionExtFields& fields) noexcept
{
    if (!input.prepared)
        return false;


    // PhysxCollisionAPI extension fields. Skip the per-attribute reads when
    // the API isn't applied, but DON'T early-return — trigger flags below are
    // independent of PhysxCollisionAPI.
    if (input.physxCollisionAPI)
    {
        fields.torsionalPatchRadius = readPreparedClampedFloat(input.physxCollisionTorsionalPatchRadius,
            fields.torsionalPatchRadius, 0.0f, FLT_MAX);
        fields.minTorsionalPatchRadius = readPreparedClampedFloat(input.physxCollisionMinTorsionalPatchRadius,
            fields.minTorsionalPatchRadius, 0.0f, FLT_MAX);
    }

    // PhysxTriggerAPI flips isTrigger; PhysxTriggerStateAPI additionally
    // flips isTriggerUsdOutput. Both flags are only ever set to true
    // (never to false) — the caller passes the current outDesc value in.
    if (input.physxTriggerAPI)
    {
        fields.isTrigger = true;
        if (input.physxTriggerStateAPI)
            fields.isTriggerUsdOutput = true;
    }

    // contactOffset / restOffset (PhysxCollisionAPI). The schema default for
    // both is NEGATIVE inf, an "unset" sentinel, and that sentinel is what
    // separates an authored value from the schema fallback.
    //
    // This used to ask `hasAuthoredAttribute`. A resolved-value backend answers
    // that `true` for everything it publishes (ADR-0020), so on ovstage BOTH
    // authored bits were always set and the Newton contactMargin/contactGap
    // fallbacks below — which gate on them — silently never fired, at load and
    // at runtime alike. Reading the sentinel out of the resolved value instead
    // is answerable identically on every backend: ovstage publishes the raw
    // -inf here (measured), and USD resolves the same schema fallback.
    //
    // Sign matters, and is the reason this does not lose the sentinel mapping:
    //   v < -0.5e38f (-inf or -FLT_MAX) -> UNSET. Leave the caller's value
    //            alone, authored bit stays false.
    //   +inf  -> an explicitly authored sentinel; keep the historical mapping
    //            (contactOffset -> -1.0f, restOffset -> 0.0f) and set authored.
    //   finite -> authored; clamp to range.
    // Mirrors readSceneGravity's `< -0.5e38f` test for physics:gravityMagnitude.
    // The one behaviour this cannot preserve is an explicitly authored value
    // at or below -0.5e38f, which is by construction indistinguishable from
    // the schema fallback.
    //
    // Per-axis range:
    //   contactOffset: clamp to [0, FLT_MAX].
    //   restOffset:    clamp to [-FLT_MAX, FLT_MAX].
    //
    // Cross-validate concrete contact offsets. A negative contact offset
    // means automatic; the runtime resolves it from geometry and restOffset.
    if (input.physxCollisionAPI)
    {
        float contactOffset = fields.contactOffset;
        float restOffset = fields.restOffset;

        // The schema's "unset" fallback. Threshold, not exact -inf: the
        // resolved-value sentinel this guards against (like the joint-limit
        // sentinels -- kJointSentinelLimit, ParseMimicJoint's
        // kFiniteLimitSentinel, PhysXScenePropertiesUpdate's gravityMagnitude
        // check) shows up in the wild as both -inf and -FLT_MAX; ovstage's
        // population already publishes the FLT_MAX spelling for the positive
        // maxJointVelocity sentinel (see hasAxisVelocityLimitOpinion). An
        // exact isinf() test would miss the -FLT_MAX spelling and mark the
        // attribute authored, silently disabling the Newton
        // contactMargin/contactGap fallbacks the authored bit gates.
        // static: MSVC 14.29 (VS2019, the OSS-build floor) rejects a non-static
        // constexpr local read inside a captureless lambda with C3493.
        static constexpr float kFiniteLimitSentinel = 0.5e38f;
        const auto isUnset = [](float v) { return v < -kFiniteLimitSentinel; };

        float attrVal;
        if (input.physxCollisionContactOffset.read(attrVal) && !isUnset(attrVal))
        {
            fields.contactOffsetAuthored = true;
            if (std::isinf(attrVal))
                contactOffset = -1.0f;
            else if (attrVal >= 0.0f && attrVal <= FLT_MAX)
                contactOffset = attrVal;
        }

        if (input.physxCollisionRestOffset.read(attrVal) && !isUnset(attrVal))
        {
            fields.restOffsetAuthored = true;
            if (std::isinf(attrVal))
                restOffset = 0.0f;
            else if (attrVal >= -FLT_MAX && attrVal <= FLT_MAX)
                restOffset = attrVal;
        }

        // Preserve the automatic contact sentinel and authored rest offset
        // together. Comparing restOffset to the unresolved -1 sentinel would
        // discard valid values before the runtime can compute contactOffset.
        if (contactOffset < 0.0f || contactOffset >= restOffset)
            fields.contactOffset = contactOffset;
        if (contactOffset < 0.0f || restOffset < contactOffset)
            fields.restOffset = restOffset;
    }

    // Newton fallback: NewtonCollisionAPI -> PhysxCollisionAPI mapping.
    // Mirrors usdLoad/Collision.cpp:446-484.
    //   newton:contactMargin -> physxCollision:restOffset    (direct)
    //   newton:contactGap    -> physxCollision:contactOffset = margin + gap
    // Applies only when the PhysX side is NOT authored. The epsilon-bump
    // at the end re-validates the contactOffset > restOffset constraint and
    // patches up edge cases where the Newton-derived values violate it.
    {
        float newtonMargin = 0.0f;
        bool hasNewtonMargin = false;
        if (!fields.restOffsetAuthored)
        {
            float m;
            if (input.newtonContactMargin.read(m) && m >= 0.0f)
            {
                fields.restOffset = m;
                newtonMargin = m;
                hasNewtonMargin = true;
            }
        }

        if (!fields.contactOffsetAuthored)
        {
            float gap;
            // Newton uses -inf as "use default" sentinel; skip it.
            if (input.newtonContactGap.read(gap) && !std::isinf(gap) && gap >= 0.0f)
            {
                // Newton gap is on top of margin; PhysX contactOffset is
                // measured from the surface, so add margin.
                float margin = hasNewtonMargin ? newtonMargin : fields.restOffset;
                fields.contactOffset = margin + gap;
            }
        }

        // Re-validate the contactOffset > restOffset constraint. If
        // contactOffset is non-negative AND not strictly greater, bump
        // it by an epsilon so PhysX accepts the pair.
        if (fields.contactOffset >= 0.0f && fields.contactOffset <= fields.restOffset)
        {
            constexpr float epsilon = 1e-4f;
            fields.contactOffset = fields.restOffset + epsilon;
        }
    }
    return true;
}




// Reads `PhysxSchemaPhysxConvexHullCollisionAPI` (minThickness +
// hullVertexLimit) plus the `newton:maxHullVertices` fallback. No
// clamping, no explicit default override: when the attribute isn't
// authored, the existing `params` field is preserved.
void parseConvexHullCookingExt(ParseContext& ctx, ObjectKey key, ConvexMeshCookingParams& params)
{
    fillConvexHullCookingExt(prepareConvexHullCookingExt(ctx, key), params);
}

PreparedConvexHullCookingExt prepareConvexHullCookingExt(ParseContext& ctx, ObjectKey key)
{
    const IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();
    PreparedConvexHullCookingExt input;
    input.physxConvexHullCollisionAPI = src.hasSchema(key, tok.physxConvexHullCollisionAPI);
    readConvexHullCookingExtAttributes(src, key, tok, input);
    return input;
}

bool fillConvexHullCookingExt(const PreparedConvexHullCookingExt& input, ConvexMeshCookingParams& params) noexcept
{
    if (!input.prepared)
        return false;


    bool hullVertexLimitAuthored = false;
    if (input.physxConvexHullCollisionAPI)
    {
        input.physxConvexHullCollisionMinThickness.read(params.minThickness);
        hullVertexLimitAuthored = readPreparedUint32(input.physxConvexHullCollisionHullVertexLimit, params.maxHullVertices);
    }

    // Newton fallback: newton:maxHullVertices (int) → maxHullVertices when
    // PhysX hullVertexLimit is not authored AND the Newton value is > 0.
    if (!hullVertexLimitAuthored)
    {
        int64_t nv;
        if (input.newtonMaxHullVertices.read(nv) && nv > 0)
            params.maxHullVertices = static_cast<uint32_t>(nv);
    }
    return true;
}


// Reads PhysxSchemaPhysxConvexDecompositionCollisionAPI. Six fields
// plus the same Newton `maxHullVertices` fallback as ConvexHull.
//
// Note: no `if (physxColMeshAPI)` schema-presence gate before reading
// (unlike ConvexHull) — each helper no-ops on missing attributes. This
// is parity-equivalent in practice (when the API isn't applied the
// attribute isn't authored either) but also picks up "free" attributes
// authored without applying the API.
void parseConvexDecompositionCookingExt(ParseContext& ctx, ObjectKey key, ConvexDecompositionCookingParams& params)
{
    fillConvexDecompositionCookingExt(prepareConvexDecompositionCookingExt(ctx, key), params);
}

PreparedConvexDecompositionCookingExt prepareConvexDecompositionCookingExt(ParseContext& ctx, ObjectKey key)
{
    const IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();
    PreparedConvexDecompositionCookingExt input;
    readConvexDecompositionCookingExtAttributes(src, key, tok, input);
    return input;
}

bool fillConvexDecompositionCookingExt(const PreparedConvexDecompositionCookingExt& input, ConvexDecompositionCookingParams& params) noexcept
{
    if (!input.prepared)
        return false;


    input.physxConvexDecompositionCollisionMinThickness.read(params.minThickness);
    readPreparedUint32(input.physxConvexDecompositionCollisionMaxConvexHulls, params.maxConvexHulls);
    const bool hullVertexLimitAuthored = readPreparedUint32(input.physxConvexDecompositionCollisionHullVertexLimit, params.maxHullVertices);
    readPreparedUint32(input.physxConvexDecompositionCollisionVoxelResolution, params.voxelResolution);
    input.physxConvexDecompositionCollisionErrorPercentage.read(params.errorPercentage);
    input.physxConvexDecompositionCollisionShrinkWrap.read(params.shrinkWrap);

    // Newton fallback for hullVertexLimit — same gating as ConvexHull.
    if (!hullVertexLimitAuthored)
    {
        int64_t nv;
        if (input.newtonMaxHullVertices.read(nv) && nv > 0)
            params.maxHullVertices = static_cast<uint32_t>(nv);
    }
    return true;
}


// Reads PhysxSchemaPhysxSphereFillCollisionAPI. Four fields including
// a token enum for the fill mode. Like ConvexDecomp, no
// `if (physxColMeshAPI)` schema-presence gate.
void parseSphereFillCookingExt(ParseContext& ctx, ObjectKey key, SphereFillCookingParams& params)
{
    fillSphereFillCookingExt(prepareSphereFillCookingExt(ctx, key), params);
}

PreparedSphereFillCookingExt prepareSphereFillCookingExt(ParseContext& ctx, ObjectKey key)
{
    const IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();
    PreparedSphereFillCookingExt input;
    input.fillModeFlood = tok.fillModeFlood;
    input.fillModeRaycast = tok.fillModeRaycast;
    input.fillModeSurface = tok.fillModeSurface;
    readSphereFillCookingExtAttributes(src, key, tok, input);
    return input;
}

bool fillSphereFillCookingExt(const PreparedSphereFillCookingExt& input, SphereFillCookingParams& params) noexcept
{
    if (!input.prepared)
        return false;


    readPreparedUint32(input.physxSphereFillCollisionMaxSpheres, params.maxSpheres);
    readPreparedUint32(input.physxSphereFillCollisionSeedCount, params.seedCount);
    readPreparedUint32(input.physxSphereFillCollisionVoxelResolution, params.voxelResolution);

    // fillMode is a token enum: "flood" / "raycast" / "surface". Any
    // other value (or missing attribute) keeps the existing default.
    TokenId fm;
    if (input.physxSphereFillCollisionFillMode.read(fm))
    {
        if      (fm == input.fillModeFlood)   params.fillMode = ::omni::physx::SphereFillMode::eFLOOD;
        else if (fm == input.fillModeRaycast) params.fillMode = ::omni::physx::SphereFillMode::eRAYCAST;
        else if (fm == input.fillModeSurface) params.fillMode = ::omni::physx::SphereFillMode::eSURFACE;
    }
    return true;
}




// Reads `PhysxTriangleMeshCollisionAPI:weldTolerance`. No
// schema-applied gate (the API may be applied or not; the attribute
// is read regardless).
void parseTriangleMeshCookingExt(ParseContext& ctx, ObjectKey key, TriangleMeshCookingParams& params)
{
    fillTriangleMeshCookingExt(prepareTriangleMeshCookingExt(ctx, key), params);
}

PreparedTriangleMeshCookingExt prepareTriangleMeshCookingExt(ParseContext& ctx, ObjectKey key)
{
    const IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();
    PreparedTriangleMeshCookingExt input;
    readTriangleMeshCookingExtAttributes(src, key, tok, input);
    return input;
}

bool fillTriangleMeshCookingExt(const PreparedTriangleMeshCookingExt& input, TriangleMeshCookingParams& params) noexcept
{
    if (!input.prepared)
        return false;

    readPreparedWeld(input.physxTriangleMeshCollisionWeldTolerance, params.meshWeldTolerance);
    return true;
}


// Reads `simplificationMetric` and `weldTolerance` from the
// PhysxTriangleMeshSimplificationCollisionAPI schema. No
// schema-applied gate.
void parseTriangleMeshSimplificationCookingExt(ParseContext& ctx, ObjectKey key, TriangleMeshCookingParams& params)
{
    fillTriangleMeshSimplificationCookingExt(prepareTriangleMeshSimplificationCookingExt(ctx, key), params);
}

PreparedTriangleMeshSimplificationCookingExt prepareTriangleMeshSimplificationCookingExt(ParseContext& ctx, ObjectKey key)
{
    const IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();
    PreparedTriangleMeshSimplificationCookingExt input;
    readTriangleMeshSimplificationCookingExtAttributes(src, key, tok, input);
    return input;
}

bool fillTriangleMeshSimplificationCookingExt(const PreparedTriangleMeshSimplificationCookingExt& input, TriangleMeshCookingParams& params) noexcept
{
    if (!input.prepared)
        return false;

    input.physxTriangleMeshSimplificationCollisionMetric.read(params.simplificationMetric);
    readPreparedWeld(input.physxTriangleMeshSimplificationCollisionWeldTolerance, params.meshWeldTolerance);
    return true;
}


// SDF cooking-knob reader. Two-stage gate: (1) PhysxSDFMeshCollisionAPI
// must be applied; (2) sdfResolution must be > 0. Only when both hold
// does the parser populate the remaining six fields. Returns the
// validity flag so the caller can switch between SDF and triangle-mesh
// cooking.
bool parseSdfMeshCookingExt(ParseContext& ctx, ObjectKey key, SdfMeshCookingParams& params)
{
    return fillSdfMeshCookingExt(prepareSdfMeshCookingExt(ctx, key), params);
}

PreparedSdfMeshCookingExt prepareSdfMeshCookingExt(ParseContext& ctx, ObjectKey key)
{
    const IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();
    PreparedSdfMeshCookingExt input;
    input.physxSDFMeshCollisionAPI = src.hasSchema(key, tok.physxSDFMeshCollisionAPI);
    input.bitsPerPixel8 = tok.bitsPerPixel8;
    input.bitsPerPixel16 = tok.bitsPerPixel16;
    input.bitsPerPixel32 = tok.bitsPerPixel32;
    readSdfMeshCookingExtAttributes(src, key, tok, input);
    return input;
}

bool fillSdfMeshCookingExt(const PreparedSdfMeshCookingExt& input, SdfMeshCookingParams& params) noexcept
{
    if (!input.prepared)
        return false;


    if (!input.physxSDFMeshCollisionAPI)
        return false;

    // sdfResolution is the validity gate.
    uint32_t resolution = 0;
    readPreparedUint32(input.physxSDFMeshCollisionSdfResolution, resolution);
    if (resolution == 0)
        return false;

    params.sdfResolution = resolution;
    readPreparedUint32(input.physxSDFMeshCollisionSdfSubgridResolution, params.sdfSubgridResolution);

    // sdfBitsPerSubgridPixel is a token enum. Default is 16; only
    // recognised tokens override. On any other (or missing) value, the
    // param keeps the seeded 16.
    uint32_t sdfBitsPerSubgridPixel = 16;
    TokenId bp;
    if (input.physxSDFMeshCollisionSdfBitsPerSubgridPixel.read(bp))
    {
        if      (bp == input.bitsPerPixel8)  sdfBitsPerSubgridPixel = 8;
        else if (bp == input.bitsPerPixel16) sdfBitsPerSubgridPixel = 16;
        else if (bp == input.bitsPerPixel32) sdfBitsPerSubgridPixel = 32;
    }
    params.sdfBitsPerSubgridPixel = sdfBitsPerSubgridPixel;

    // Declare a local seeded to a safe default, try to read, then
    // unconditionally assign back to params so an unauthored attribute
    // doesn't leave the field with whatever the caller seeded.
    float narrowBandThickness = 0.0f;
    input.physxSDFMeshCollisionSdfNarrowBandThickness.read(narrowBandThickness);
    params.sdfNarrowBandThickness = narrowBandThickness;

    float sdfMargin = 0.0f;  // seeded to avoid UB on unauthored attribute
    input.physxSDFMeshCollisionSdfMargin.read(sdfMargin);
    params.sdfMargin = sdfMargin;

    bool sdfEnableRemeshing = false;  // seeded to avoid UB on unauthored attribute
    input.physxSDFMeshCollisionSdfEnableRemeshing.read(sdfEnableRemeshing);
    params.sdfEnableRemeshing = sdfEnableRemeshing;

    float sdfTriangleCountReductionFactor = 0.0f;  // same.
    input.physxSDFMeshCollisionSdfTriangleCountReductionFactor.read(sdfTriangleCountReductionFactor);
    params.sdfTriangleCountReductionFactor = sdfTriangleCountReductionFactor;

    return true;
}


} // namespace omni::physics::parse

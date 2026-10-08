// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
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
 *
 * @implements REQ-PARSE-COL-005
 * @covers AC-1
 */

// Collision extension parser -- covers PhysxCollisionAPI scalar fields,
// contactOffset/restOffset sentinel logic, and Newton offset fallbacks.
// Shape geometry, mesh data via BufferHandle, and cooking integration stay
// in the consumer-side walker.

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/KnownTokens.h>

#include <cfloat>
#include <cmath>

namespace omni::physics::parse
{

namespace
{

bool readFloatWithFallback(const IPhysicsSource& src,
                           ObjectKey key,
                           ObjectKey fallback,
                           TokenId attr,
                           float& out)
{
    if (src.getAttribute(key, attr, out))
        return true;
    return fallback.valid() && fallback != key && src.getAttribute(fallback, attr, out);
}

float readClampedFloat(const IPhysicsSource& src,
                       ObjectKey key,
                       ObjectKey fallback,
                       TokenId attr,
                       float defaultVal,
                       float minVal,
                       float maxVal)
{
    float result;
    if (!readFloatWithFallback(src, key, fallback, attr, result))
        return defaultVal;
    if (result < minVal) result = minVal;
    if (result > maxVal) result = maxVal;
    return result;
}

} // namespace

void parseCollisionExt(ParseContext& ctx, ObjectKey key, CollisionExtFields& fields)
{
    IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();

    // PhysxCollisionAPI extension fields. Skip the per-attribute reads when
    // the API isn't applied, but DON'T early-return — trigger flags below are
    // independent of PhysxCollisionAPI.
    if (src.hasSchema(key, tok.physxCollisionAPI))
    {
        fields.torsionalPatchRadius = readClampedFloat(
            src, key, fields.attributeFallback, tok.physxCollisionTorsionalPatchRadius,
            fields.torsionalPatchRadius, 0.0f, FLT_MAX);
        fields.minTorsionalPatchRadius = readClampedFloat(
            src, key, fields.attributeFallback, tok.physxCollisionMinTorsionalPatchRadius,
            fields.minTorsionalPatchRadius, 0.0f, FLT_MAX);
    }

    // PhysxTriggerAPI flips isTrigger; PhysxTriggerStateAPI additionally
    // flips isTriggerUsdOutput. Both flags are only ever set to true
    // (never to false) — the caller passes the current outDesc value in.
    if (src.hasSchema(key, tok.physxTriggerAPI))
    {
        fields.isTrigger = true;
        if (src.hasSchema(key, tok.physxTriggerStateAPI))
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
    // Cross-validation: only writes the local back to outDesc when
    // `contactOffset >= restOffset` (and the inverse for restOffset), so
    // a one-sided author can't violate the constraint.
    if (src.hasSchema(key, tok.physxCollisionAPI))
    {
        float contactOffset = fields.contactOffset;
        float restOffset = fields.restOffset;

        // The schema's "unset" fallback. Threshold, not exact -inf: the
        // resolved-value sentinel this guards against (like the joint-limit
        // sentinels -- kJointSentinelLimit, ParseMimicJoint's
        // kFiniteLimitSentinel, PhysXScenePropertiesUpdate's gravityMagnitude
        // check) shows up in the wild as both -inf and -FLT_MAX; ovstage's
        // population already publishes the FLT_MAX spelling for the positive
        // maxJointVelocity sentinel (see isPhysxMaxJointVelocityAuthored). An
        // exact isinf() test would miss the -FLT_MAX spelling and mark the
        // attribute authored, silently disabling the Newton
        // contactMargin/contactGap fallbacks the authored bit gates.
        // static: MSVC 14.29 (VS2019, the OSS-build floor) rejects a non-static
        // constexpr local read inside a captureless lambda with C3493.
        static constexpr float kFiniteLimitSentinel = 0.5e38f;
        const auto isUnset = [](float v) { return v < -kFiniteLimitSentinel; };

        float attrVal;
        if (readFloatWithFallback(src, key, fields.attributeFallback, tok.physxCollisionContactOffset, attrVal) &&
            !isUnset(attrVal))
        {
            fields.contactOffsetAuthored = true;
            if (std::isinf(attrVal))
                contactOffset = -1.0f;
            else if (attrVal >= 0.0f && attrVal <= FLT_MAX)
                contactOffset = attrVal;
        }

        if (readFloatWithFallback(src, key, fields.attributeFallback, tok.physxCollisionRestOffset, attrVal) &&
            !isUnset(attrVal))
        {
            fields.restOffsetAuthored = true;
            if (std::isinf(attrVal))
                restOffset = 0.0f;
            else if (attrVal >= -FLT_MAX && attrVal <= FLT_MAX)
                restOffset = attrVal;
        }

        // Cross-validation gates the writeback: each field is only
        // committed if it leaves contactOffset >= restOffset. When
        // neither is authored, both locals equal the input fields and
        // the writes are no-ops, so this is safe.
        if (contactOffset >= restOffset)
            fields.contactOffset = contactOffset;
        if (restOffset < contactOffset)
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
            if (readFloatWithFallback(src, key, fields.attributeFallback, tok.newtonContactMargin, m) && m >= 0.0f)
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
            if (readFloatWithFallback(src, key, fields.attributeFallback, tok.newtonContactGap, gap) &&
                !std::isinf(gap) && gap >= 0.0f)
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
}

namespace
{
// Tiny helpers — `attr.Get(&v)` with no clamp, no override on missing.

bool readFloatIfAuthored(IPhysicsSource& src, ObjectKey key, TokenId attr, float& out)
{
    return src.getAttribute(key, attr, out);
}

bool readIntAsUint32IfAuthored(IPhysicsSource& src, ObjectKey key, TokenId attr, uint32_t& out)
{
    int64_t tmp;
    if (!src.getAttribute(key, attr, tmp)) return false;
    out = static_cast<uint32_t>(tmp);
    return true;
}

bool readBoolIfAuthored(IPhysicsSource& src, ObjectKey key, TokenId attr, bool& out)
{
    return src.getAttribute(key, attr, out);
}

} // namespace

// Reads `PhysxSchemaPhysxConvexHullCollisionAPI` (minThickness +
// hullVertexLimit) plus the `newton:maxHullVertices` fallback. No
// clamping, no explicit default override: when the attribute isn't
// authored, the existing `params` field is preserved.
void parseConvexHullCookingExt(ParseContext& ctx, ObjectKey key, ConvexMeshCookingParams& params)
{
    IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();

    bool hullVertexLimitAuthored = false;
    if (src.hasSchema(key, tok.physxConvexHullCollisionAPI))
    {
        readFloatIfAuthored(src, key, tok.physxConvexHullCollisionMinThickness, params.minThickness);
        hullVertexLimitAuthored = readIntAsUint32IfAuthored(
            src, key, tok.physxConvexHullCollisionHullVertexLimit, params.maxHullVertices);
    }

    // Newton fallback: newton:maxHullVertices (int) → maxHullVertices when
    // PhysX hullVertexLimit is not authored AND the Newton value is > 0.
    if (!hullVertexLimitAuthored)
    {
        int64_t nv;
        if (src.getAttribute(key, tok.newtonMaxHullVertices, nv) && nv > 0)
            params.maxHullVertices = static_cast<uint32_t>(nv);
    }
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
    IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();

    readFloatIfAuthored(src, key, tok.physxConvexDecompositionCollisionMinThickness, params.minThickness);
    readIntAsUint32IfAuthored(src, key, tok.physxConvexDecompositionCollisionMaxConvexHulls, params.maxConvexHulls);
    const bool hullVertexLimitAuthored = readIntAsUint32IfAuthored(
        src, key, tok.physxConvexDecompositionCollisionHullVertexLimit, params.maxHullVertices);
    readIntAsUint32IfAuthored(src, key, tok.physxConvexDecompositionCollisionVoxelResolution, params.voxelResolution);
    readFloatIfAuthored(src, key, tok.physxConvexDecompositionCollisionErrorPercentage, params.errorPercentage);
    readBoolIfAuthored(src, key, tok.physxConvexDecompositionCollisionShrinkWrap, params.shrinkWrap);

    // Newton fallback for hullVertexLimit — same gating as ConvexHull.
    if (!hullVertexLimitAuthored)
    {
        int64_t nv;
        if (src.getAttribute(key, tok.newtonMaxHullVertices, nv) && nv > 0)
            params.maxHullVertices = static_cast<uint32_t>(nv);
    }
}

// Reads PhysxSchemaPhysxSphereFillCollisionAPI. Four fields including
// a token enum for the fill mode. Like ConvexDecomp, no
// `if (physxColMeshAPI)` schema-presence gate.
void parseSphereFillCookingExt(ParseContext& ctx, ObjectKey key, SphereFillCookingParams& params)
{
    IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();

    readIntAsUint32IfAuthored(src, key, tok.physxSphereFillCollisionMaxSpheres, params.maxSpheres);
    readIntAsUint32IfAuthored(src, key, tok.physxSphereFillCollisionSeedCount, params.seedCount);
    readIntAsUint32IfAuthored(src, key, tok.physxSphereFillCollisionVoxelResolution, params.voxelResolution);

    // fillMode is a token enum: "flood" / "raycast" / "surface". Any
    // other value (or missing attribute) keeps the existing default.
    TokenId fm;
    if (src.getAttribute(key, tok.physxSphereFillCollisionFillMode, fm))
    {
        if      (fm == tok.fillModeFlood)   params.fillMode = ::omni::physx::SphereFillMode::eFLOOD;
        else if (fm == tok.fillModeRaycast) params.fillMode = ::omni::physx::SphereFillMode::eRAYCAST;
        else if (fm == tok.fillModeSurface) params.fillMode = ::omni::physx::SphereFillMode::eSURFACE;
    }
}

namespace
{
// Read attribute (if authored / schema-fallback present), then
// unconditionally NaN-guard down to -FLT_MAX. The NaN check applies
// even when the attribute is missing — defensive against an existing
// NaN in the params struct.
void readWeldToleranceWithNanGuard(IPhysicsSource& src, ObjectKey key, TokenId attr, float& out)
{
    readFloatIfAuthored(src, key, attr, out);
    if (std::isnan(out))
        out = -FLT_MAX;
}
} // namespace

// Reads `PhysxTriangleMeshCollisionAPI:weldTolerance`. No
// schema-applied gate (the API may be applied or not; the attribute
// is read regardless).
void parseTriangleMeshCookingExt(ParseContext& ctx, ObjectKey key, TriangleMeshCookingParams& params)
{
    IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();
    readWeldToleranceWithNanGuard(src, key, tok.physxTriangleMeshCollisionWeldTolerance, params.meshWeldTolerance);
}

// Reads `simplificationMetric` and `weldTolerance` from the
// PhysxTriangleMeshSimplificationCollisionAPI schema. No
// schema-applied gate.
void parseTriangleMeshSimplificationCookingExt(ParseContext& ctx, ObjectKey key, TriangleMeshCookingParams& params)
{
    IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();
    readFloatIfAuthored(src, key, tok.physxTriangleMeshSimplificationCollisionMetric, params.simplificationMetric);
    readWeldToleranceWithNanGuard(src, key, tok.physxTriangleMeshSimplificationCollisionWeldTolerance, params.meshWeldTolerance);
}

// SDF cooking-knob reader. Two-stage gate: (1) PhysxSDFMeshCollisionAPI
// must be applied; (2) sdfResolution must be > 0. Only when both hold
// does the parser populate the remaining six fields. Returns the
// validity flag so the caller can switch between SDF and triangle-mesh
// cooking.
bool parseSdfMeshCookingExt(ParseContext& ctx, ObjectKey key, SdfMeshCookingParams& params)
{
    IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();

    if (!src.hasSchema(key, tok.physxSDFMeshCollisionAPI))
        return false;

    // sdfResolution is the validity gate.
    uint32_t resolution = 0;
    readIntAsUint32IfAuthored(src, key, tok.physxSDFMeshCollisionSdfResolution, resolution);
    if (resolution == 0)
        return false;

    params.sdfResolution = resolution;
    readIntAsUint32IfAuthored(src, key, tok.physxSDFMeshCollisionSdfSubgridResolution, params.sdfSubgridResolution);

    // sdfBitsPerSubgridPixel is a token enum. Default is 16; only
    // recognised tokens override. On any other (or missing) value, the
    // param keeps the seeded 16.
    uint32_t sdfBitsPerSubgridPixel = 16;
    TokenId bp;
    if (src.getAttribute(key, tok.physxSDFMeshCollisionSdfBitsPerSubgridPixel, bp))
    {
        if      (bp == tok.bitsPerPixel8)  sdfBitsPerSubgridPixel = 8;
        else if (bp == tok.bitsPerPixel16) sdfBitsPerSubgridPixel = 16;
        else if (bp == tok.bitsPerPixel32) sdfBitsPerSubgridPixel = 32;
    }
    params.sdfBitsPerSubgridPixel = sdfBitsPerSubgridPixel;

    // Declare a local seeded to a safe default, try to read, then
    // unconditionally assign back to params so an unauthored attribute
    // doesn't leave the field with whatever the caller seeded.
    float narrowBandThickness = 0.0f;
    readFloatIfAuthored(src, key, tok.physxSDFMeshCollisionSdfNarrowBandThickness, narrowBandThickness);
    params.sdfNarrowBandThickness = narrowBandThickness;

    float sdfMargin = 0.0f;  // seeded to avoid UB on unauthored attribute
    readFloatIfAuthored(src, key, tok.physxSDFMeshCollisionSdfMargin, sdfMargin);
    params.sdfMargin = sdfMargin;

    bool sdfEnableRemeshing = false;  // seeded to avoid UB on unauthored attribute
    readBoolIfAuthored(src, key, tok.physxSDFMeshCollisionSdfEnableRemeshing, sdfEnableRemeshing);
    params.sdfEnableRemeshing = sdfEnableRemeshing;

    float sdfTriangleCountReductionFactor = 0.0f;  // same.
    readFloatIfAuthored(src, key, tok.physxSDFMeshCollisionSdfTriangleCountReductionFactor, sdfTriangleCountReductionFactor);
    params.sdfTriangleCountReductionFactor = sdfTriangleCountReductionFactor;

    return true;
}

} // namespace omni::physics::parse

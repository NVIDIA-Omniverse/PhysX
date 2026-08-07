// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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
 */

// Collision extension parser — covers the PhysxCollisionAPI fields that
// have no time-sample-registration dependency. Full collision parsing
// (shape geometry, mesh data via BufferHandle, cooking integration,
// contactOffset/restOffset inf-sentinel logic) lives in the consumer-
// side walker.

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/KnownTokens.h>

#include <cfloat>
#include <cmath>

namespace omni::physics::parse
{

namespace
{

float readClampedFloat(const IPhysicsSource& src, ObjectKey key, TokenId attr,
                      float defaultVal, float minVal, float maxVal)
{
    float result;
    if (!src.getAttribute(key, attr, result))
        return defaultVal;
    if (result < minVal) result = minVal;
    if (result > maxVal) result = maxVal;
    return result;
}

} // namespace

void parseCollisionExt(ParseContext& ctx, ObjectKey key, CollisionExtFields& fields)
{
    IPhysicsSource& src = ctx.source();
    KnownTokens tok;
    tok.intern(src);

    // PhysxCollisionAPI extension fields. Skip the per-attribute reads when
    // the API isn't applied, but DON'T early-return — trigger flags below are
    // independent of PhysxCollisionAPI.
    if (src.hasSchema(key, tok.physxCollisionAPI))
    {
        fields.torsionalPatchRadius = readClampedFloat(
            src, key, tok.physxCollisionTorsionalPatchRadius,
            fields.torsionalPatchRadius, 0.0f, FLT_MAX);
        fields.minTorsionalPatchRadius = readClampedFloat(
            src, key, tok.physxCollisionMinTorsionalPatchRadius,
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

    // contactOffset / restOffset (PhysxCollisionAPI). Schema default for
    // both is `-inf` (an "unset" sentinel), so reads MUST gate on
    // `hasAuthoredAttribute`, not on `getAttribute(...).valid()` (which
    // resolves schema fallbacks).
    //
    // Per-axis sentinel and range:
    //   contactOffset: inf -> -1.0f; otherwise clamp to [0, FLT_MAX].
    //   restOffset:    inf ->  0.0f; otherwise clamp to [-FLT_MAX, FLT_MAX].
    //
    // Cross-validation: only writes the local back to outDesc when
    // `contactOffset >= restOffset` (and the inverse for restOffset), so
    // a one-sided author can't violate the constraint.
    if (src.hasSchema(key, tok.physxCollisionAPI))
    {
        float contactOffset = fields.contactOffset;
        float restOffset = fields.restOffset;

        if (src.hasAuthoredAttribute(key, tok.physxCollisionContactOffset))
        {
            fields.contactOffsetAuthored = true;
            float attrVal;
            if (src.getAttribute(key, tok.physxCollisionContactOffset, attrVal))
            {
                if (std::isinf(attrVal))
                    contactOffset = -1.0f;
                else if (attrVal >= 0.0f && attrVal <= FLT_MAX)
                    contactOffset = attrVal;
            }
        }

        if (src.hasAuthoredAttribute(key, tok.physxCollisionRestOffset))
        {
            fields.restOffsetAuthored = true;
            float attrVal;
            if (src.getAttribute(key, tok.physxCollisionRestOffset, attrVal))
            {
                if (std::isinf(attrVal))
                    restOffset = 0.0f;
                else if (attrVal >= -FLT_MAX && attrVal <= FLT_MAX)
                    restOffset = attrVal;
            }
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
            if (src.getAttribute(key, tok.newtonContactMargin, m) && m >= 0.0f)
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
            if (src.getAttribute(key, tok.newtonContactGap, gap) && !std::isinf(gap) && gap >= 0.0f)
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
    KnownTokens tok;
    tok.intern(src);

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
    KnownTokens tok;
    tok.intern(src);

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
    KnownTokens tok;
    tok.intern(src);

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
    KnownTokens tok;
    tok.intern(src);
    readWeldToleranceWithNanGuard(src, key, tok.physxTriangleMeshCollisionWeldTolerance, params.meshWeldTolerance);
}

// Reads `simplificationMetric` and `weldTolerance` from the
// PhysxTriangleMeshSimplificationCollisionAPI schema. No
// schema-applied gate.
void parseTriangleMeshSimplificationCookingExt(ParseContext& ctx, ObjectKey key, TriangleMeshCookingParams& params)
{
    IPhysicsSource& src = ctx.source();
    KnownTokens tok;
    tok.intern(src);
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
    KnownTokens tok;
    tok.intern(src);

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

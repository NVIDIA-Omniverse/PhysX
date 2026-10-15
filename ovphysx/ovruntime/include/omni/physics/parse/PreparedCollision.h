// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2 AC-5
 */
#pragma once
#include "ParseApi.h"
#include "PreparedAttribute.h"
#include "KnownTokens.h"

namespace omni::physics::parse
{

struct PreparedCollisionExt
{
    bool prepared = false;
    bool physxCollisionAPI = false;
    bool physxTriggerAPI = false;
    bool physxTriggerStateAPI = false;
    PreparedAttribute<float> physxCollisionTorsionalPatchRadius;
    PreparedAttribute<float> physxCollisionMinTorsionalPatchRadius;
    PreparedAttribute<float> physxCollisionContactOffset;
    PreparedAttribute<float> physxCollisionRestOffset;
    PreparedAttribute<float> newtonContactMargin;
    PreparedAttribute<float> newtonContactGap;
};
PreparedCollisionExt prepareCollisionExt(ParseContext& context, ObjectKey key);
bool fillCollisionExt(const PreparedCollisionExt& input, CollisionExtFields& output) noexcept;

struct PreparedConvexHullCookingExt
{
    bool prepared = false;
    bool physxConvexHullCollisionAPI = false;
    PreparedAttribute<float> physxConvexHullCollisionMinThickness;
    PreparedAttribute<int64_t> physxConvexHullCollisionHullVertexLimit;
    PreparedAttribute<int64_t> newtonMaxHullVertices;
};
PreparedConvexHullCookingExt prepareConvexHullCookingExt(ParseContext& context, ObjectKey key);
bool fillConvexHullCookingExt(const PreparedConvexHullCookingExt& input, ConvexMeshCookingParams& output) noexcept;

struct PreparedConvexDecompositionCookingExt
{
    bool prepared = false;
    PreparedAttribute<float> physxConvexDecompositionCollisionMinThickness;
    PreparedAttribute<int64_t> physxConvexDecompositionCollisionMaxConvexHulls;
    PreparedAttribute<int64_t> physxConvexDecompositionCollisionHullVertexLimit;
    PreparedAttribute<int64_t> physxConvexDecompositionCollisionVoxelResolution;
    PreparedAttribute<float> physxConvexDecompositionCollisionErrorPercentage;
    PreparedAttribute<bool> physxConvexDecompositionCollisionShrinkWrap;
    PreparedAttribute<int64_t> newtonMaxHullVertices;
};
PreparedConvexDecompositionCookingExt prepareConvexDecompositionCookingExt(ParseContext& context, ObjectKey key);
bool fillConvexDecompositionCookingExt(const PreparedConvexDecompositionCookingExt& input, ConvexDecompositionCookingParams& output) noexcept;

struct PreparedSphereFillCookingExt
{
    bool prepared = false;
    TokenId fillModeFlood;
    TokenId fillModeRaycast;
    TokenId fillModeSurface;
    PreparedAttribute<int64_t> physxSphereFillCollisionMaxSpheres;
    PreparedAttribute<int64_t> physxSphereFillCollisionSeedCount;
    PreparedAttribute<int64_t> physxSphereFillCollisionVoxelResolution;
    PreparedAttribute<TokenId> physxSphereFillCollisionFillMode;
};
PreparedSphereFillCookingExt prepareSphereFillCookingExt(ParseContext& context, ObjectKey key);
bool fillSphereFillCookingExt(const PreparedSphereFillCookingExt& input, SphereFillCookingParams& output) noexcept;

struct PreparedTriangleMeshCookingExt
{
    bool prepared = false;
    PreparedAttribute<float> physxTriangleMeshCollisionWeldTolerance;
};
PreparedTriangleMeshCookingExt prepareTriangleMeshCookingExt(ParseContext& context, ObjectKey key);
bool fillTriangleMeshCookingExt(const PreparedTriangleMeshCookingExt& input, TriangleMeshCookingParams& output) noexcept;

struct PreparedTriangleMeshSimplificationCookingExt
{
    bool prepared = false;
    PreparedAttribute<float> physxTriangleMeshSimplificationCollisionMetric;
    PreparedAttribute<float> physxTriangleMeshSimplificationCollisionWeldTolerance;
};
PreparedTriangleMeshSimplificationCookingExt prepareTriangleMeshSimplificationCookingExt(ParseContext& context, ObjectKey key);
bool fillTriangleMeshSimplificationCookingExt(const PreparedTriangleMeshSimplificationCookingExt& input, TriangleMeshCookingParams& output) noexcept;

struct PreparedSdfMeshCookingExt
{
    bool prepared = false;
    bool physxSDFMeshCollisionAPI = false;
    TokenId bitsPerPixel8;
    TokenId bitsPerPixel16;
    TokenId bitsPerPixel32;
    PreparedAttribute<int64_t> physxSDFMeshCollisionSdfResolution;
    PreparedAttribute<int64_t> physxSDFMeshCollisionSdfSubgridResolution;
    PreparedAttribute<float> physxSDFMeshCollisionSdfNarrowBandThickness;
    PreparedAttribute<float> physxSDFMeshCollisionSdfMargin;
    PreparedAttribute<bool> physxSDFMeshCollisionSdfEnableRemeshing;
    PreparedAttribute<float> physxSDFMeshCollisionSdfTriangleCountReductionFactor;
    PreparedAttribute<TokenId> physxSDFMeshCollisionSdfBitsPerSubgridPixel;
};
PreparedSdfMeshCookingExt prepareSdfMeshCookingExt(ParseContext& context, ObjectKey key);
bool fillSdfMeshCookingExt(const PreparedSdfMeshCookingExt& input, SdfMeshCookingParams& output) noexcept;

template <typename Reader>
void readCollisionExtAttributes(const Reader& src, ObjectKey key, const KnownTokens& tok, PreparedCollisionExt& input)
{
    if (input.physxCollisionAPI)
        input.physxCollisionTorsionalPatchRadius = prepareAttribute<float>(src, key, tok.physxCollisionTorsionalPatchRadius);
    if (input.physxCollisionAPI)
        input.physxCollisionMinTorsionalPatchRadius = prepareAttribute<float>(src, key, tok.physxCollisionMinTorsionalPatchRadius);
    if (input.physxCollisionAPI)
        input.physxCollisionContactOffset = prepareAttribute<float>(src, key, tok.physxCollisionContactOffset);
    if (input.physxCollisionAPI)
        input.physxCollisionRestOffset = prepareAttribute<float>(src, key, tok.physxCollisionRestOffset);
    input.newtonContactMargin = prepareAttribute<float>(src, key, tok.newtonContactMargin);
    input.newtonContactGap = prepareAttribute<float>(src, key, tok.newtonContactGap);
    input.prepared = true;
}

template <typename Reader>
void readConvexHullCookingExtAttributes(const Reader& src, ObjectKey key, const KnownTokens& tok, PreparedConvexHullCookingExt& input)
{
    if (input.physxConvexHullCollisionAPI)
    {
        input.physxConvexHullCollisionMinThickness = prepareAttribute<float>(src, key, tok.physxConvexHullCollisionMinThickness);
        input.physxConvexHullCollisionHullVertexLimit = prepareAttribute<int64_t>(src, key, tok.physxConvexHullCollisionHullVertexLimit);
    }
    if (!input.physxConvexHullCollisionHullVertexLimit.valid)
        input.newtonMaxHullVertices = prepareAttribute<int64_t>(src, key, tok.newtonMaxHullVertices);
    input.prepared = true;
}

template <typename Reader>
void readConvexDecompositionCookingExtAttributes(const Reader& src, ObjectKey key, const KnownTokens& tok, PreparedConvexDecompositionCookingExt& input)
{
    input.physxConvexDecompositionCollisionMinThickness = prepareAttribute<float>(src, key, tok.physxConvexDecompositionCollisionMinThickness);
    input.physxConvexDecompositionCollisionMaxConvexHulls = prepareAttribute<int64_t>(src, key, tok.physxConvexDecompositionCollisionMaxConvexHulls);
    input.physxConvexDecompositionCollisionHullVertexLimit = prepareAttribute<int64_t>(src, key, tok.physxConvexDecompositionCollisionHullVertexLimit);
    input.physxConvexDecompositionCollisionVoxelResolution = prepareAttribute<int64_t>(src, key, tok.physxConvexDecompositionCollisionVoxelResolution);
    input.physxConvexDecompositionCollisionErrorPercentage = prepareAttribute<float>(src, key, tok.physxConvexDecompositionCollisionErrorPercentage);
    input.physxConvexDecompositionCollisionShrinkWrap = prepareAttribute<bool>(src, key, tok.physxConvexDecompositionCollisionShrinkWrap);
    if (!input.physxConvexDecompositionCollisionHullVertexLimit.valid)
        input.newtonMaxHullVertices = prepareAttribute<int64_t>(src, key, tok.newtonMaxHullVertices);
    input.prepared = true;
}

template <typename Reader>
void readSphereFillCookingExtAttributes(const Reader& src, ObjectKey key, const KnownTokens& tok, PreparedSphereFillCookingExt& input)
{
    input.physxSphereFillCollisionMaxSpheres = prepareAttribute<int64_t>(src, key, tok.physxSphereFillCollisionMaxSpheres);
    input.physxSphereFillCollisionSeedCount = prepareAttribute<int64_t>(src, key, tok.physxSphereFillCollisionSeedCount);
    input.physxSphereFillCollisionVoxelResolution = prepareAttribute<int64_t>(src, key, tok.physxSphereFillCollisionVoxelResolution);
    input.physxSphereFillCollisionFillMode = prepareAttribute<TokenId>(src, key, tok.physxSphereFillCollisionFillMode);
    input.prepared = true;
}

template <typename Reader>
void readTriangleMeshCookingExtAttributes(const Reader& src, ObjectKey key, const KnownTokens& tok, PreparedTriangleMeshCookingExt& input)
{
    input.physxTriangleMeshCollisionWeldTolerance = prepareAttribute<float>(src, key, tok.physxTriangleMeshCollisionWeldTolerance);
    input.prepared = true;
}

template <typename Reader>
void readTriangleMeshSimplificationCookingExtAttributes(const Reader& src, ObjectKey key, const KnownTokens& tok, PreparedTriangleMeshSimplificationCookingExt& input)
{
    input.physxTriangleMeshSimplificationCollisionMetric = prepareAttribute<float>(src, key, tok.physxTriangleMeshSimplificationCollisionMetric);
    input.physxTriangleMeshSimplificationCollisionWeldTolerance = prepareAttribute<float>(src, key, tok.physxTriangleMeshSimplificationCollisionWeldTolerance);
    input.prepared = true;
}

template <typename Reader>
void readSdfMeshCookingExtAttributes(const Reader& src, ObjectKey key, const KnownTokens& tok, PreparedSdfMeshCookingExt& input)
{
    if (input.physxSDFMeshCollisionAPI)
    {
        input.physxSDFMeshCollisionSdfResolution = prepareAttribute<int64_t>(src, key, tok.physxSDFMeshCollisionSdfResolution);
        input.physxSDFMeshCollisionSdfSubgridResolution = prepareAttribute<int64_t>(src, key, tok.physxSDFMeshCollisionSdfSubgridResolution);
        input.physxSDFMeshCollisionSdfNarrowBandThickness = prepareAttribute<float>(src, key, tok.physxSDFMeshCollisionSdfNarrowBandThickness);
        input.physxSDFMeshCollisionSdfMargin = prepareAttribute<float>(src, key, tok.physxSDFMeshCollisionSdfMargin);
        input.physxSDFMeshCollisionSdfEnableRemeshing = prepareAttribute<bool>(src, key, tok.physxSDFMeshCollisionSdfEnableRemeshing);
        input.physxSDFMeshCollisionSdfTriangleCountReductionFactor = prepareAttribute<float>(src, key, tok.physxSDFMeshCollisionSdfTriangleCountReductionFactor);
        input.physxSDFMeshCollisionSdfBitsPerSubgridPixel = prepareAttribute<TokenId>(src, key, tok.physxSDFMeshCollisionSdfBitsPerSubgridPixel);
    }
    input.prepared = true;
}

inline void appendCollisionAttributeTokens(const PreparedCollisionExt&, const KnownTokens& tok, std::vector<TokenId>& result)
{
    result.insert(result.end(), {
        tok.physxCollisionTorsionalPatchRadius,
        tok.physxCollisionMinTorsionalPatchRadius,
        tok.physxCollisionContactOffset,
        tok.physxCollisionRestOffset,
        tok.newtonContactMargin,
        tok.newtonContactGap,
    });
}

inline void appendCollisionAttributeTokens(const PreparedConvexHullCookingExt&, const KnownTokens& tok, std::vector<TokenId>& result)
{
    result.insert(result.end(), {
        tok.physxConvexHullCollisionMinThickness,
        tok.physxConvexHullCollisionHullVertexLimit,
        tok.newtonMaxHullVertices,
    });
}

inline void appendCollisionAttributeTokens(const PreparedConvexDecompositionCookingExt&, const KnownTokens& tok, std::vector<TokenId>& result)
{
    result.insert(result.end(), {
        tok.physxConvexDecompositionCollisionMinThickness,
        tok.physxConvexDecompositionCollisionMaxConvexHulls,
        tok.physxConvexDecompositionCollisionHullVertexLimit,
        tok.physxConvexDecompositionCollisionVoxelResolution,
        tok.physxConvexDecompositionCollisionErrorPercentage,
        tok.physxConvexDecompositionCollisionShrinkWrap,
        tok.newtonMaxHullVertices,
    });
}

inline void appendCollisionAttributeTokens(const PreparedSphereFillCookingExt&, const KnownTokens& tok, std::vector<TokenId>& result)
{
    result.insert(result.end(), {
        tok.physxSphereFillCollisionMaxSpheres,
        tok.physxSphereFillCollisionSeedCount,
        tok.physxSphereFillCollisionVoxelResolution,
        tok.physxSphereFillCollisionFillMode,
    });
}

inline void appendCollisionAttributeTokens(const PreparedTriangleMeshCookingExt&, const KnownTokens& tok, std::vector<TokenId>& result)
{
    result.insert(result.end(), {
        tok.physxTriangleMeshCollisionWeldTolerance,
    });
}

inline void appendCollisionAttributeTokens(const PreparedTriangleMeshSimplificationCookingExt&, const KnownTokens& tok, std::vector<TokenId>& result)
{
    result.insert(result.end(), {
        tok.physxTriangleMeshSimplificationCollisionMetric,
        tok.physxTriangleMeshSimplificationCollisionWeldTolerance,
    });
}

inline void appendCollisionAttributeTokens(const PreparedSdfMeshCookingExt&, const KnownTokens& tok, std::vector<TokenId>& result)
{
    result.insert(result.end(), {
        tok.physxSDFMeshCollisionSdfResolution,
        tok.physxSDFMeshCollisionSdfSubgridResolution,
        tok.physxSDFMeshCollisionSdfNarrowBandThickness,
        tok.physxSDFMeshCollisionSdfMargin,
        tok.physxSDFMeshCollisionSdfEnableRemeshing,
        tok.physxSDFMeshCollisionSdfTriangleCountReductionFactor,
        tok.physxSDFMeshCollisionSdfBitsPerSubgridPixel,
    });
}

inline std::vector<TokenId> collisionAttributeTokens(const KnownTokens& tok)
{
    return {
        tok.physxCollisionTorsionalPatchRadius,
        tok.physxCollisionMinTorsionalPatchRadius,
        tok.physxCollisionContactOffset,
        tok.physxCollisionRestOffset,
        tok.newtonContactMargin,
        tok.newtonContactGap,
        tok.physxConvexHullCollisionMinThickness,
        tok.physxConvexHullCollisionHullVertexLimit,
        tok.newtonMaxHullVertices,
        tok.physxConvexDecompositionCollisionMinThickness,
        tok.physxConvexDecompositionCollisionMaxConvexHulls,
        tok.physxConvexDecompositionCollisionHullVertexLimit,
        tok.physxConvexDecompositionCollisionVoxelResolution,
        tok.physxConvexDecompositionCollisionErrorPercentage,
        tok.physxConvexDecompositionCollisionShrinkWrap,
        tok.physxSphereFillCollisionMaxSpheres,
        tok.physxSphereFillCollisionSeedCount,
        tok.physxSphereFillCollisionVoxelResolution,
        tok.physxSphereFillCollisionFillMode,
        tok.physxTriangleMeshCollisionWeldTolerance,
        tok.physxTriangleMeshSimplificationCollisionMetric,
        tok.physxTriangleMeshSimplificationCollisionWeldTolerance,
        tok.physxSDFMeshCollisionSdfResolution,
        tok.physxSDFMeshCollisionSdfSubgridResolution,
        tok.physxSDFMeshCollisionSdfNarrowBandThickness,
        tok.physxSDFMeshCollisionSdfMargin,
        tok.physxSDFMeshCollisionSdfEnableRemeshing,
        tok.physxSDFMeshCollisionSdfTriangleCountReductionFactor,
        tok.physxSDFMeshCollisionSdfBitsPerSubgridPixel,
    };
}

} // namespace omni::physics::parse

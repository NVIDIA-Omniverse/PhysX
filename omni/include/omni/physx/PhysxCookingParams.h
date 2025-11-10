// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{

namespace physx
{

///  Default values
const uint32_t defaultCookedMaxHullCount = 16;
const uint32_t defaultCookedVoxelResolution = 100000;
const uint32_t defaultCookedMaxHullVertices = 64;
const float defaultCookedErrorPercentage = 10.0f;
const float defaultCookedMinThickness = 0.001f;
const uint32_t defaultSdfResolution = 256u;
const uint32_t defaultSdfSubgridResolution = 6u;
const uint32_t defaultSdfBitsPerSubgridPixel = 16u;
const float defaultSdfNarrowBandThickness = 0.01f;
const float defaultSdfMargin = 0.01f;
const bool defaultSdfEnableRemeshing = false;
const float defaultSdfTriangleCountReductionFactor = 1.0f;
const uint32_t defaultMaxSpheres = 128;
const uint32_t defaultSeedCount = 1000;

struct CookingParamsType
{
    enum Enum : uint32_t
    {
        eUNDEFINED = 0,
        eCONVEX_MESH = 1,
        eTRIANGLE_MESH = 2,
        eCONVEX_DECOMPOSITION = 3,
        eSDF_TRIANGLE_MESH = 4,
        eSPHERE_FILL = 5,
    };
};

/// Triangle mesh mode
struct TriangleMeshMode
{
    enum Enum
    {
        eORIGINAL_TRIANGLES, //!< original input triangles are used
        eQUADRIC_SIMPLIFICATION //!< mesh decimation is used
    };
};

/// Sphere fill mode
struct SphereFillMode
{
    enum Enum
    {
        eFLOOD,
        eRAYCAST,
        eSURFACE,
    };
};

/// Cooking params base
struct CookingParams
{
    CookingParams() : type(CookingParamsType::eUNDEFINED)
    {
    }

    CookingParamsType::Enum type;
};

/// Convex mesh cooking params
struct ConvexMeshCookingParams : CookingParams
{
    ConvexMeshCookingParams()
    {
        type = CookingParamsType::eCONVEX_MESH;
    }

    uint32_t maxHullVertices{ defaultCookedMaxHullVertices };
    float minThickness{ defaultCookedMinThickness };
    carb::Float3 signScale = { 1.0f, 1.0f, 1.0f };
};

/// Triangle mesh cooking params
struct TriangleMeshCookingParams : CookingParams
{
    TriangleMeshCookingParams()
    {
        type = CookingParamsType::eTRIANGLE_MESH;
    }

    TriangleMeshMode::Enum mode{ TriangleMeshMode::eORIGINAL_TRIANGLES };
    float simplificationMetric{ 0.55f };
    float meshWeldTolerance{ -FLT_MAX };
};

struct ConvexDecompositionCookingParams : CookingParams
{
    ConvexDecompositionCookingParams()
    {
        type = CookingParamsType::eCONVEX_DECOMPOSITION;
    }

    uint32_t maxHullVertices{ defaultCookedMaxHullVertices }; // The maximum number of vertices for each convex hull
    uint32_t maxConvexHulls{ defaultCookedMaxHullCount }; // The maximum number of convex hulls to generate
    uint32_t voxelResolution{ defaultCookedVoxelResolution }; // The voxel resolution
    float errorPercentage{ defaultCookedErrorPercentage }; // The volume error percentage threshold
    float minThickness{ defaultCookedMinThickness };
    bool shrinkWrap{ false }; // whether or not to shrinkwrap the convex decomposition hulls
    carb::Float3 signScale = { 1.0f, 1.0f, 1.0f };
};

struct SdfMeshCookingParams : CookingParams
{
    SdfMeshCookingParams()
    {
        type = CookingParamsType::eSDF_TRIANGLE_MESH;
    }

    uint32_t sdfResolution{ defaultSdfResolution };
    uint32_t sdfSubgridResolution{ defaultSdfSubgridResolution };
    uint32_t sdfBitsPerSubgridPixel{ defaultSdfBitsPerSubgridPixel };
    float sdfNarrowBandThickness{ defaultSdfNarrowBandThickness };
    float sdfMargin{ defaultSdfMargin };
    bool sdfEnableRemeshing{ defaultSdfEnableRemeshing };
    float sdfTriangleCountReductionFactor{ defaultSdfTriangleCountReductionFactor };
};

struct SphereFillCookingParams : CookingParams
{
    SphereFillCookingParams()
    {
        type = CookingParamsType::eSPHERE_FILL;
    }

    uint32_t maxSpheres{ defaultMaxSpheres };
    uint32_t seedCount{ defaultSeedCount };
    SphereFillMode::Enum fillMode{ SphereFillMode::eFLOOD };
    uint32_t voxelResolution{ defaultCookedVoxelResolution };
    carb::Float3 signScale = { 1.0f, 1.0f, 1.0f };
};
} // namespace physx
} // namespace omni

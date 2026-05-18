// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/ujitso/UjitsoUtils.inl>
#include <private/omni/physx/IPhysxCookingServicePrivate.h>
#include <carb/extras/Hash.h>
#include <carb/profiler/Profile.h>

#include "UjitsoServiceUtils.h"

#include "../service/CookingTask.h"


// string literals used to store/retrieve data from UJITSO requests
#define PHYSX_COOK_MESH_STR "PhysXCookMesh"
#define DATA_TYPE_STR "dataType"
#define COOKING_VERSION_STR "cookingVersion"
#define METERS_PER_UNIT_STR "metersPerUnit"
#define BUILD_GPU_DATA_STR "buildGpuData"
#define BUILD_TRIANGLE_ADJACENCIES_STR "buildTriangleAdjacencies"
#define PRIM_MESH_TEXT "primMeshText"

#define PHYSX_TRIANGULATE_MESH_STR "PhysXTriangulateMesh"
#define TRIANGULATION_VERSION_STR "triangulationVersion"

#define PHYSX_COOKING_REQUEST_DEVELOPER_KEY "PhysXCookingRequestDeveloperKey"

#define PHYSX_UJITSO_CACHE_BEHAVIOR "PhysXUjitsoCacheBehavior"

#define PHYSX_UJITSO_ENABLE_GPU_COOKING "PhysXUjitsoEnableGpuCooking"

#define CONTEXT_ID "contextId"

using namespace carb::ujitso;

namespace omni
{
namespace physx
{

/**
 * Utilities to build a triangulation view from data and vice versa
 */

static inline bool buildTriangulationViewFromData(
    PhysxCookingMeshTriangulationView& view, uint16_t& maxMaterialIndex, const std::vector<uint8_t>& data)
{
    CARB_PROFILE_ZONE(0, "omni::physx::buildTriangulationViewFromData");

    BufferReader reader(data);

    // Check version
    uint32_t version;
    CHECK_RETURN_FALSE_ON_FAIL(reader.readValue(version));
    CHECK_RETURN_FALSE_ON_FAIL(version == PhysxCookingDataVersion_MeshTriangulation);

    // Vertices
    CHECK_RETURN_FALSE_ON_FAIL((reader.readConstSpan<uint32_t, carb::Float3>(view.points)));

    // Triangles
    using triangle_t = uint32_t[3];
    CHECK_RETURN_FALSE_ON_FAIL((reader.readConstSpan<uint32_t, triangle_t>(view.triangles)));

    // Face mapping
    CHECK_RETURN_FALSE_ON_FAIL((reader.readConstSpan<uint32_t, uint32_t>(view.trianglesFaceMap)));

    // Materials
    CHECK_RETURN_FALSE_ON_FAIL((reader.readConstSpan<uint32_t, uint16_t>(view.faceMaterials)));

    // Num used materials
    CHECK_RETURN_FALSE_ON_FAIL((reader.readValue(maxMaterialIndex)));

    return true;
}

static inline void buildTriangulationMeshFromView(
    TriangulationMesh& mesh, const PhysxCookingMeshTriangulationView& view, uint16_t maxMaterialIndex)
{
    CARB_PROFILE_ZONE(0, "omni::physx::buildTriangulationMeshFromView");

    mesh.points.assign(view.points.begin(), view.points.end());
    const uint32_t* traiangleData  = reinterpret_cast<const uint32_t*>(view.triangles.data());
    mesh.indices.assign(traiangleData, traiangleData + 3*view.triangles.size());
    mesh.trianglesToFacesMapping.assign(view.trianglesFaceMap.begin(), view.trianglesFaceMap.end());
    mesh.faceMaterials.assign(view.faceMaterials.begin(), view.faceMaterials.end());
    mesh.maxMaterialIndex = maxMaterialIndex;
}


/**
 * Utilities to read and write request values from various parameter structs.
 * Struct members are read/written individually, rather than as one struct memory block.
 */

template <typename T, typename DynamicRequestT>
inline bool getRequestValue(T& value, DynamicRequestT& request, const char* key)
{
    const T* valuePtr;
    if (!request.template getAs<T>(key, valuePtr) || !valuePtr) return false;
    value = *valuePtr;
    return true;
}

#define ADD_REQUEST_PARAM(_name) builder.add(#_name, params._name)
#define GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(_name) CHECK_RETURN_FALSE_ON_FAIL(getRequestValue(params._name, request, #_name))
#define REMOVE_REQUEST_PARAM(_name) builder.removeKey(#_name)


// TriangleMeshCookingParams load/unload

template<typename RequestBuilderT>
void addParamsToRequest(RequestBuilderT& builder, const TriangleMeshCookingParams& params)
{
    builder.add("mode", (uint32_t)params.mode);
    ADD_REQUEST_PARAM(simplificationMetric);
    ADD_REQUEST_PARAM(meshWeldTolerance);
}

template<typename DynamicRequestT>
bool getParamsFromRequest(TriangleMeshCookingParams& params, DynamicRequestT& request)
{
    uint32_t mode;
    CHECK_RETURN_FALSE_ON_FAIL(getRequestValue(mode, request, "mode"));
    params.mode = (TriangleMeshMode::Enum)mode;
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(simplificationMetric);
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(meshWeldTolerance);
    return true;
}


// SdfMeshCookingParams load/unload

template<typename RequestBuilderT>
void addParamsToRequest(RequestBuilderT& builder, const SdfMeshCookingParams& params)
{
    ADD_REQUEST_PARAM(sdfResolution);
    ADD_REQUEST_PARAM(sdfSubgridResolution);
    ADD_REQUEST_PARAM(sdfBitsPerSubgridPixel);
    ADD_REQUEST_PARAM(sdfNarrowBandThickness);
    ADD_REQUEST_PARAM(sdfMargin);
    ADD_REQUEST_PARAM(sdfEnableRemeshing);
    ADD_REQUEST_PARAM(sdfTriangleCountReductionFactor);
}

template<typename DynamicRequestT>
bool getParamsFromRequest(SdfMeshCookingParams& params, DynamicRequestT& request)
{
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(sdfResolution);
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(sdfSubgridResolution);
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(sdfBitsPerSubgridPixel);
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(sdfNarrowBandThickness);
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(sdfMargin);
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(sdfEnableRemeshing);
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(sdfTriangleCountReductionFactor);
    return true;
}


// ConvexMeshCookingParams load/unload

template<typename RequestBuilderT>
void addParamsToRequest(RequestBuilderT& builder, const ConvexMeshCookingParams& params)
{
    ADD_REQUEST_PARAM(maxHullVertices);
    ADD_REQUEST_PARAM(minThickness);
    ADD_REQUEST_PARAM(signScale);
}

template<typename DynamicRequestT>
bool getParamsFromRequest(ConvexMeshCookingParams& params, DynamicRequestT& request)
{
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(maxHullVertices);
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(minThickness);
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(signScale);
    return true;
}


// ConvexDecompositionCookingParams load/unload

template<typename RequestBuilderT>
void addParamsToRequest(RequestBuilderT& builder, const ConvexDecompositionCookingParams& params)
{
    ADD_REQUEST_PARAM(maxHullVertices);
    ADD_REQUEST_PARAM(maxConvexHulls);
    ADD_REQUEST_PARAM(voxelResolution);
    ADD_REQUEST_PARAM(errorPercentage);
    ADD_REQUEST_PARAM(minThickness);
    ADD_REQUEST_PARAM(shrinkWrap);
    ADD_REQUEST_PARAM(signScale);
}

template<typename DynamicRequestT>
bool getParamsFromRequest(ConvexDecompositionCookingParams& params, DynamicRequestT& request)
{
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(maxHullVertices);
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(maxConvexHulls);
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(voxelResolution);
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(errorPercentage);
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(minThickness);
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(shrinkWrap);
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(signScale);
    return true;
}


// SphereFillCookingParams load/unload

template<typename RequestBuilderT>
void addParamsToRequest(RequestBuilderT& builder, const SphereFillCookingParams& params)
{
    ADD_REQUEST_PARAM(maxSpheres);
    ADD_REQUEST_PARAM(seedCount);
    builder.add("fillMode", (uint32_t)params.fillMode);
    ADD_REQUEST_PARAM(voxelResolution);
    ADD_REQUEST_PARAM(signScale);
}

template<typename DynamicRequestT>
bool getParamsFromRequest(SphereFillCookingParams& params, DynamicRequestT& request)
{
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(maxSpheres);
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(seedCount);
    uint32_t fillMode;
    CHECK_RETURN_FALSE_ON_FAIL(getRequestValue(fillMode, request, "fillMode"));
    params.fillMode = (SphereFillMode::Enum)fillMode;
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(voxelResolution);
    GET_REQUEST_PARAM_RETURN_FALSE_ON_FAIL(signScale);
    return true;
}

} // namespace physx
} // namespace omni


// Register token types

REGISTER_REQUEST_TOKEN_TYPE(ContainerHandle);
REGISTER_REQUEST_TOKEN_TYPE(ContainerContentHash);
REGISTER_REQUEST_TOKEN_TYPE(carb::extras::hash128_t);
REGISTER_REQUEST_TOKEN_TYPE(carb::Float3);

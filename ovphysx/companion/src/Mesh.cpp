// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-ATTACH-COMPUTE-001
// @covers AC-1 AC-2 AC-3 AC-4
// @implements REQ-COMP-DEFORM-COOK-001
// @covers AC-2 AC-3 AC-4
// @implements REQ-CAPI-COMPANION-001
// @covers AC-1
#include "ovphysx_companion/mesh.h"
#include <omni/physx/AuthoringElements.h>
#include <algorithm>

namespace {
ovphysx_companion_status_t velocities(const float*, const float*, uint32_t, const uint32_t*,
    uint32_t, uint32_t, const int32_t*, const float*, const float*, uint32_t, float*) noexcept;
std::vector<::physx::PxVec3> vectors(const float* data, uint32_t count) {
    std::vector<::physx::PxVec3> result;
    result.reserve(count);
    for (uint32_t i=0;i<count;++i) result.emplace_back(data[size_t(i)*3],data[size_t(i)*3+1],data[size_t(i)*3+2]);
    return result;
}
ovphysx_companion_status_t embed(const float* vertices, uint32_t vertexCount,
    const uint32_t* indices, uint32_t elementCount, uint32_t width,
    const float* points, uint32_t pointCount, float distance,
    int32_t* elements, float* weights, float* distances, float* offsets, uint32_t* invalid) noexcept {
    if (!invalid) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    *invalid=UINT32_MAX;
    if ((!vertices && vertexCount) || (!points && pointCount) ||
        (pointCount && (!elements || !weights || !distances || !offsets))) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try {
        const std::vector< ::physx::PxVec3> verts=vectors(vertices,vertexCount), queries=vectors(points,pointCount);
        omni::physx::authoring::ElementMapping mapping;
        if (!omni::physx::authoring::embedPoints(verts.data(),vertexCount,indices,elementCount,width,
            queries.data(),pointCount,distance,mapping,*invalid)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        if (pointCount) {
            std::copy(mapping.elements.begin(),mapping.elements.end(),elements);
            std::copy(mapping.weights.begin(),mapping.weights.end(),weights);
            std::copy(mapping.distances.begin(),mapping.distances.end(),distances);
            std::copy(mapping.normalOffsets.begin(),mapping.normalOffsets.end(),offsets);
        }
        return OVPHYSX_COMPANION_OK;
    } catch (...) { return OVPHYSX_COMPANION_ERROR; }
}
ovphysx_companion_status_t interpolate(const float* vertices, uint32_t vertexCount,
    const uint32_t* indices, uint32_t elementCount, uint32_t width,
    const int32_t* elements, const float* weights, const float* offsets,
    uint32_t pointCount, float* output) noexcept {
    if ((!vertices && vertexCount) || (!output && pointCount)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try {
        const std::vector< ::physx::PxVec3> verts=vectors(vertices,vertexCount);
        std::vector<::physx::PxVec3> result;
        if (!omni::physx::authoring::interpolateElements(verts.data(),vertexCount,indices,elementCount,width,
            elements,weights,offsets,pointCount,result)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        for (uint32_t i=0;i<pointCount;++i) for (uint32_t k=0;k<3;++k) output[size_t(i)*3+k]=result[i][k];
        return OVPHYSX_COMPANION_OK;
    } catch (...) { return OVPHYSX_COMPANION_ERROR; }
}
}
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_mesh_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_mesh_api_t* output) {
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if (version!=1 || size!=sizeof(*output)) return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output={sizeof(*output),1,embed,interpolate};
    return OVPHYSX_COMPANION_OK;
}

namespace {
ovphysx_companion_status_t velocities(const float* positions, const float* inputVelocities,
    uint32_t vertexCount, const uint32_t* indices, uint32_t elementCount, uint32_t width,
    const int32_t* elements, const float* weights, const float* offsets,
    uint32_t pointCount, float* output) noexcept {
    if ((vertexCount && (!positions || !inputVelocities)) || (pointCount && !output))
        return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try {
        const std::vector< ::physx::PxVec3> verts=vectors(positions,vertexCount), vels=vectors(inputVelocities,vertexCount);
        std::vector<::physx::PxVec3> result;
        if (!omni::physx::authoring::interpolateElementVelocities(verts.data(),vels.data(),vertexCount,
            indices,elementCount,width,elements,weights,offsets,pointCount,result))
            return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        for (uint32_t i=0;i<pointCount;++i) {
            output[size_t(i)*3]=result[i].x;
            output[size_t(i)*3+1]=result[i].y;
            output[size_t(i)*3+2]=result[i].z;
        }
        return OVPHYSX_COMPANION_OK;
    } catch (...) { return OVPHYSX_COMPANION_ERROR; }
}
}
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_mesh_motion_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_mesh_motion_api_t* output) {
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if (version!=1 || size!=sizeof(*output)) return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output={sizeof(*output),1,velocities};
    return OVPHYSX_COMPANION_OK;
}

namespace {
ovphysx_companion_status_t filterPairs(const float* vertices0, uint32_t count0,
    const uint32_t* triangles0, uint32_t triangleCount0,
    const float* vertices1, uint32_t count1, const uint32_t* triangles1, uint32_t triangleCount1,
    float distance, uint32_t capacity, uint32_t* pairs, uint32_t* count) noexcept {
    if ((!vertices0 && count0) || (!vertices1 && count1) || !count || (capacity && !pairs) ||
        count0>10000000 || count1>10000000 || capacity>1000000 ||
        uint64_t(triangleCount0)*triangleCount1>10000000) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try {
        const std::vector< ::physx::PxVec3> a=vectors(vertices0,count0), b=vectors(vertices1,count1);
        std::vector<uint32_t> result;
        if (!omni::physx::authoring::triangleFilterPairs(a.data(),count0,triangles0,triangleCount0,
            b.data(),count1,triangles1,triangleCount1,distance,capacity,result))
            return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        if (!result.empty()) std::copy(result.begin(),result.end(),pairs);
        *count=uint32_t(result.size()/2);
        return OVPHYSX_COMPANION_OK;
    } catch (...) { return OVPHYSX_COMPANION_ERROR; }
}
}
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_mesh_filter_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_mesh_filter_api_t* output) {
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if (version!=1 || size!=sizeof(*output)) return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output={sizeof(*output),1,filterPairs};
    return OVPHYSX_COMPANION_OK;
}

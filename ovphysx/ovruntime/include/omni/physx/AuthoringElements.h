// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-AUTHORING-ELEMENTS-001
// @covers AC-1 AC-2 AC-3 AC-4 AC-5
#pragma once
#include <foundation/PxVec3.h>
#include <cstdint>
#include <vector>

namespace omni::physx::authoring
{
struct ElementMapping
{
    // One entry per query point. -1 denotes no match within inclusive distance.
    std::vector<int32_t> elements;
    // Four weights per point; triangle fourth weight is zero. Unmatched weights
    // and offsets are zero; unmatched distances are +infinity.
    std::vector<float> weights;
    std::vector<float> distances;
    std::vector<float> normalOffsets;
};

// CPU, no foundation/SDK or stage required. Coordinates share one Cartesian frame
// and unit. Width is 3 (triangles) or 4 (tetrahedra). Outside tets project to the
// closest face; distance reports that projection explicitly. Equal-distance ties
// choose the lowest element index. Nearly degenerate elements reject the whole
// operation; invalidElement identifies topology failures, UINT32_MAX otherwise.
// Inputs remain caller-owned through return. Failures leave output unchanged.
bool embedPoints(const ::physx::PxVec3* vertices, uint32_t vertexCount,
    const uint32_t* indices, uint32_t elementCount, uint32_t width,
    const ::physx::PxVec3* points, uint32_t pointCount, float maxDistance,
    ElementMapping& output, uint32_t& invalidElement);

// Evaluate complete barycentric mappings. Missing elements (-1), invalid weights
// or degenerate current triangles with nonzero normal offsets fail without partial output. Triangle offsets
// are measured along the oriented current unit normal; tetrahedra require zero
// offsets. Output owns its memory and can outlive the input/mapping.
bool interpolateElements(const ::physx::PxVec3* vertices, uint32_t vertexCount,
    const uint32_t* indices, uint32_t elementCount, uint32_t width,
    const int32_t* elements, const float* weights, const float* normalOffsets,
    uint32_t pointCount, std::vector<::physx::PxVec3>& output);

// Time derivative of interpolateElements for fixed weights/normal offsets.
// Positions and velocities use the same Cartesian frame and length units;
// velocities are per second. Includes the oriented unit-normal derivative for
// triangle offsets. Invalid/collapsed offset geometry leaves output unchanged.
bool interpolateElementVelocities(const ::physx::PxVec3* vertices,
    const ::physx::PxVec3* velocities, uint32_t vertexCount,
    const uint32_t* indices, uint32_t elementCount, uint32_t width,
    const int32_t* elements, const float* weights, const float* normalOffsets,
    uint32_t pointCount, std::vector<::physx::PxVec3>& output);
// Inclusive triangle distance in shared coordinates; copied interleaved pairs
// sorted by first then second index. Empty is no match, not a wildcard. At most
// 10M candidate pairs and 1M output pairs; capacity/validation failure preserves
// output. Degenerate triangles reject the request. No foundation/SDK needed.
bool triangleFilterPairs(const ::physx::PxVec3* vertices0, uint32_t vertexCount0,
    const uint32_t* triangles0, uint32_t triangleCount0,
    const ::physx::PxVec3* vertices1, uint32_t vertexCount1,
    const uint32_t* triangles1, uint32_t triangleCount1,
    float maxDistance, uint32_t maxPairs, std::vector<uint32_t>& output);

}

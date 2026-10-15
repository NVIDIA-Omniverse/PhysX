// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-NATIVE-COOKING-001
// @covers AC-1 AC-2 AC-3 AC-4
#pragma once
#include <omni/physx/NativeInspection.h>
#include <cstdint>
#include <vector>
namespace omni::physx::native
{
enum class RawCookKind:uint32_t {Convex=1,Triangle=2,Decomposition=3,Sdf=4,Spheres=5,Conforming=6,Voxel=7};
struct RawCookMesh {
    uint32_t point_count, index_count, face_count, right_handed;
    const float* points;
    const int32_t* indices;
    const int32_t* faces;
    double meters_per_unit;
};
struct RawCookOptions {
    uint32_t kind, triangle_mode;
    uint32_t max_hull_vertices, max_hulls, voxel_resolution, shrink_wrap;
    uint32_t sdf_resolution, sdf_subgrid_resolution, sdf_bits, sdf_remesh;
    uint32_t max_spheres, seed_count, sphere_mode, reserved;
    float simplification, weld_tolerance, min_thickness, error_percentage;
    float sdf_band, sdf_margin, sdf_reduction, scale[3];
};
struct RawCookResult
{
    uint64_t recipe_hash[2]{};
    uint32_t warnings=0,element_width=0;
    std::vector<float> points;
    std::vector<uint32_t> indices,faces;
    std::vector<int32_t> embedding;
    std::vector<std::vector<unsigned char>> blobs;
};
// Synchronous copied-data computation. Own authoring admission in the runtime:
// callback reentry and exclusive recording/configuration report Busy, while
// unrelated pending attachments remain usable. Failure preserves output.
InspectionResult cookRawMesh(const RawCookMesh&,const RawCookOptions&,RawCookResult&) noexcept;
}

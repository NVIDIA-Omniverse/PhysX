// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-NATIVE-SAMPLING-001
// @covers AC-1 AC-2 AC-3
#pragma once
#include "NativeInspection.h"
#include <vector>
namespace omni::physx::native
{
struct SamplingOptions
{
    float radius=.1f,volumeRadius=0;
    uint32_t maxSamples=100000,attempts=30;
};
struct SampledPoints
{
    std::vector<float> points,weights; // packed XYZ / barycentric triples
    std::vector<int32_t> triangles; // -1 for volume samples; zero weights
};
// CPU Poisson surface/optional volume samples in the caller's common Cartesian
// frame. Surface radius is minimum separation; volume radius is average spacing.
// Nondegenerate triangles; volume mode additionally requires a closed oriented
// two-manifold. Intersecting shells are not supported. maxSamples bounds output.
// Requires a live runtime; copied authoring excludes recording transitions only.
// Playback and attachment configuration can proceed while sampling runs.
// No source cache, USD, random-seed promise, or simulation mutation.
InspectionResult samplePoints(const float* points,uint32_t pointCount,const uint32_t* triangles,
    uint32_t triangleCount,const SamplingOptions&,SampledPoints&) noexcept;
}

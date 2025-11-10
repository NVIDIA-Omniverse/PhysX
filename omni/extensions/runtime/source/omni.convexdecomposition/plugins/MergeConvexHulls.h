// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <stdint.h>

namespace vcd
{

class SimpleMesh;
class RaycastMesh;
class JobSystem;

class MergeConvexHulls
{
public:
    static MergeConvexHulls* create(JobSystem* js);

    virtual void addConvexHull(const SimpleMesh& sm) = 0;

    virtual uint32_t mergeConvexHulls(uint32_t maxHullCount, double overallMeshVolume) = 0;

    virtual uint32_t getOriginalConvexHullCount(void) const = 0;
    virtual uint32_t getMergeConvexHullCount(void) const = 0;

    virtual const SimpleMesh* getOriginalConvexHull(uint32_t index) const = 0;
    virtual const SimpleMesh* getMergedConvexHull(uint32_t index) const = 0;

    virtual uint32_t finalizeResults(uint32_t maxHullVertices,
                                     RaycastMesh* raycastMesh,
                                     double voxelScale,
                                     bool shrinkWrap,
                                     double scaleVertices) = 0;

    // Just use the input hulls as the output without no other corrections
    virtual void finalizeHulls(double scaleVertices) = 0;

    virtual void release(void) = 0;
};

} // namespace vcd

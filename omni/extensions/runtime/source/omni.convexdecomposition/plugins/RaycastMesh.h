// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <stdint.h>

namespace vcd
{

// Very simple brute force raycast against a triangle mesh.  Tests every triangle; no hierachy.
// Does a deep copy, always does calculations with full double float precision
class RaycastMesh
{
public:
    static RaycastMesh* createRaycastMesh(uint32_t vcount, // The number of vertices in the source triangle mesh
                                          const double* vertices, // The array of vertex positions in the format
                                                                  // x1,y1,z1..x2,y2,z2.. etc.
                                          uint32_t tcount, // The number of triangles in the source triangle mesh
                                          const uint32_t* indices); // The triangle indices in the format of i1,i2,i3
                                                                    // ... i4,i5,i6, ...

    static RaycastMesh* createRaycastMesh(uint32_t vcount, // The number of vertices in the source triangle mesh
                                          const float* vertices, // The array of vertex positions in the format
                                                                 // x1,y1,z1..x2,y2,z2.. etc.
                                          uint32_t tcount, // The number of triangles in the source triangle mesh
                                          const uint32_t* indices); // The triangle indices in the format of i1,i2,i3
                                                                    // ... i4,i5,i6, ...


    // Uses high speed AABB raycasting
    virtual bool raycast(const double* start,
                         const double* dir,
                         double& outT,
                         double& u,
                         double& v,
                         double& w,
                         double& faceSign,
                         uint32_t& faceIndex) const = 0;

    virtual bool raycast(
        const double* start, const double* to, double& outT, double& faceSign, double* hitLocation) const = 0;

    virtual bool getClosestPointWithinDistance(const double* point, double maxDistance, double* closestPoint) = 0;

    virtual void release(void) = 0;

protected:
    virtual ~RaycastMesh(void){};
};

} // namespace vcd

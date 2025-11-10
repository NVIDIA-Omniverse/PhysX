// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <stdint.h>

namespace quickhull
{

class HullPoints
{
public:
    uint32_t mVertexCount{ 0 }; // Number of input vertices
    const double* mVertices{ nullptr }; // array of input vertices
    uint32_t mMaxHullVertices{ 60 }; // Maximum number of vertices in the output convex hull
    uint32_t mMaxQuantizeVertexCount{ 256 }; // Maximum number of input vertices to consider (will be quantized not to
                                             // exceed this amount)
};

class QuickHull
{
public:
    static QuickHull* create(void);

    virtual uint32_t computeConvexHull(const HullPoints& hp) = 0;
    virtual const double* getVertices(uint32_t& vcount) const = 0;
    virtual const uint32_t* getIndices(uint32_t& tcount) const = 0;
    virtual uint32_t getPolygonCount(void) const = 0; // report number of polygons
    virtual const uint32_t* getPolygon(uint32_t index, uint32_t& numPoints, uint32_t& indexBase, double plane[4]) const = 0;

    // Inflate the convex hull by this distance
    virtual void inflate(double distance) = 0;

    virtual void release(void) = 0;

protected:
    virtual ~QuickHull(void){};
};

} // namespace quickhull

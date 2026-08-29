// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <stdint.h>
#include <carb/Defines.h>
// Triangulates a USD mesh prim in two stages: first copy the buffers into
// scratch memory on the main thread (USD is not thread-safe), then run the
// actual triangulation, which is thread-safe, from a background thread.

#include "UsdPCH.h"

namespace omni
{
namespace convexdecomposition
{
namespace triangulateusd
{

class TriangulateUSDPrim
{
public:
    static TriangulateUSDPrim* create(const PXR_NS::UsdPrim& prim);

    virtual const uint32_t* getFaceIndices(uint32_t& indexCount) const = 0;
    virtual const uint32_t* getFaceBuffer(uint32_t& faceCount) const = 0;


    // Perform the triangulation of the source data
    // returns the number of triangles produced
    virtual uint32_t triangulate(void) = 0;

    virtual float* getVertices(uint32_t& vertexCount) const = 0;
    virtual uint32_t* getIndices(uint32_t& triangleCount) const = 0;

    virtual uint32_t* getTriangleFaceMap(uint32_t& triangleCount) const = 0;
    virtual uint16_t* getFaceMaterials(uint32_t& faceCount) const = 0;

    virtual void release(void) = 0;

protected:
    TriangulateUSDPrim() = default;
    virtual ~TriangulateUSDPrim() = default;
};

} // namespace triangulateusd
} // namespace convexdecomposition
} // namespace omni

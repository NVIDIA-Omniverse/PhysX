// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <stdint.h>
#include <carb/Defines.h>
#include <private/omni/physx/IPhysxCookingServicePrivate.h> // PhysxCookingMeshView

// Triangulates a caller-supplied mesh view (read by the caller through IPhysicsSource, for
// example from USD) in two stages: buffers are first copied to scratch memory on the main
// thread, then triangulation runs on the copied data via a separate, thread-safe call usable
// from a background thread.

namespace triangulateusd
{

class TriangulateUSDPrim
{
public:
    static TriangulateUSDPrim* create(const omni::physx::PhysxCookingMeshView& meshView);

    virtual const uint32_t* getFaceIndices(uint32_t& indexCount) const = 0;
    virtual const uint32_t* getFaceBuffer(uint32_t& faceCount) const = 0;


    // Returns the number of triangles produced
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

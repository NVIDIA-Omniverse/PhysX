// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <stdint.h>
#include <carb/Defines.h>
#include <private/omni/physx/IPhysxCookingServicePrivate.h> // PhysxCookingMeshView

// This is a helper function which will
// triangulate a USD mesh prim in two
// stages.
// The first stage is it copies the buffers
// into scratch memory. This has to be done
// in the main thread because USD is not
// thread safe.
//
// The actual triangulation happens in a
// separate call which *is* thread safe.
//
// So the usage is to first copy the buffers
// then you can generate the triangulation
// using a background thread

namespace triangulateusd
{

class TriangulateUSDPrim
{
public:
    static TriangulateUSDPrim* create(const pxr::UsdPrim& prim, uint16_t& numMaterials);
    static TriangulateUSDPrim* create(const omni::physx::PhysxCookingMeshView& meshView);

    static bool fillFaceMaterials(const pxr::UsdPrim& usdPrim,
                                  omni::span<uint16_t> faceMaterials,
                                  pxr::UsdTimeCode time,
                                  uint16_t& maxMaterialIndex);

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

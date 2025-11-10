// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once


// This is a utility that will convert a
// simple triangle mesh (no normals, UVs, just vertices)
// into a USD prim that can be rendered as a solid
// shaded visualization with face normals
#include <stdint.h>

namespace debugmeshprim
{

class DebugMesh
{
public:
    uint32_t mVertexCount{ 0 };
    uint32_t mTriangleCount{ 0 };
    const float* mVertices{ nullptr };
    const uint32_t* mIndices{ nullptr };
};

class DebugMeshPrim
{
public:
    static DebugMeshPrim* create(void);

    // Create a mesh primitive relative to the provided parent
    // prim.
    // Returns true if it was successful, false if it failed
    virtual bool createDebugMeshPrim(const char* primName, const DebugMesh& mesh, uint32_t meshColor) = 0;

    virtual void release(void) = 0;

protected:
    virtual ~DebugMeshPrim(void)
    {
    }
};


} // namespace debugmeshprim

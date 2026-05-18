// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

// Utility class to make sure that a source mesh matches
// the provided minimum mesh thickness
// Looks for co-planar meshes or meshes that are ultra tiny and extrudes
// them to at least match the minimum thickness provided

#include <stdint.h>

namespace meshthickness
{

struct Mesh
{
    uint32_t vertexCount{ 0 };
    uint32_t triangleCount{ 0 };
    const float* vertices{ nullptr };
    const uint32_t* indices{ nullptr };
};

// Check to see if the input mesh is within the provided minimum thickness value.
// Returns false if it was not and no output mesh is generated
// If it returns true, then the outputMesh contains the new extruded triangle mesh
bool checkMeshThickness(const Mesh& inputMesh, Mesh& outputMesh, float minThickness);

// Release any memory associated with the output mesh previously generated
void releaseMeshOutput(Mesh& outputMesh);

} // namespace meshthickness

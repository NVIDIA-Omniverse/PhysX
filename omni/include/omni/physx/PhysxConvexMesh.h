// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{
namespace physx
{

/**
\brief Polygon data for convex mesh

Plane format: (mPlane[0],mPlane[1],mPlane[2]).dot(x) + mPlane[3] = 0
With the normal outward-facing from the hull.
*/
struct ConvexMeshPolygon
{
    float plane[4]; //!< Plane equation for this polygon
    uint16_t numVerts; //!< Number of vertices/edges in the polygon
    uint16_t indexBase; //!< Offset in index buffer
};

/**
\brief convex mesh data
\note memory is owned by omni.physx and releases when simulation is stopped
*/
struct ConvexMeshData
{
    ConvexMeshData()
        : numVertices(0), vertices(nullptr), indices(nullptr), numPolygons(0), numIndices(0), polygons(nullptr)
    {
    }

    ConvexMeshData(
        uint32_t nbVerts, const carb::Float3* verts, const uint8_t* ind, uint32_t nbPoly, const ConvexMeshPolygon* poly)
        : numVertices(nbVerts), vertices(verts), indices(ind), numPolygons(nbPoly), numIndices(0), polygons(poly)
    {
        computeNumIndices();
    }

    void computeNumIndices()
    {
        numIndices = 0;
        for (uint32_t i = 0; i < numPolygons; i++)
        {
            const ConvexMeshPolygon& meshPolygon = polygons[i];
            if (static_cast<uint32_t>(meshPolygon.indexBase + meshPolygon.numVerts) > numIndices)
            {
                numIndices = meshPolygon.indexBase + meshPolygon.numVerts;
            }
        }
    }
    uint32_t numVertices;
    const carb::Float3* vertices;
    const uint8_t* indices;
    uint32_t numPolygons;
    uint32_t numIndices;
    const ConvexMeshPolygon* polygons;
};


} // namespace physx
} // namespace omni

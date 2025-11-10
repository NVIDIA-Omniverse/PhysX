// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include "ImmediateNarrowPhase.h"
#include "ImmediateShared.h"
#include "../MultiThreader.h"

#include <PxPhysicsAPI.h>
using namespace ::physx;
using namespace omni::physx::graph;

static const float gPoseEpsilon = 1e-6f;
bool ImmediateNarrowPhase::sameMatrices(const pxr::GfMatrix4d& mat0, const pxr::GfMatrix4d& mat1)
{
    const double epsilon = double(gPoseEpsilon);
    const double* data0 = mat0.data();
    const double* data1 = mat1.data();
    for (uint32_t i = 0; i < 16; i++)
    {
        if (fabs(data0[i] - data1[i]) > epsilon)
            return false;
    }
    return true;
}

bool ImmediateNarrowPhase::sameGeometryInSamePosition(const MeshGeometryData& g1, const MeshGeometryData& g2)
{
    const bool bothTriangles = (g1.triangleMeshGeometry.triangleMesh == g2.triangleMeshGeometry.triangleMesh) &&
                               g1.triangleMeshGeometry.triangleMesh != nullptr;
    const bool bothConvexes = (g1.convexMeshGeometry.convexMesh == g2.convexMeshGeometry.convexMesh) &&
                              g1.convexMeshGeometry.convexMesh != nullptr;
    if (bothTriangles || bothConvexes)
    {
        if (sameMatrices(g1.worldMatrix, g2.worldMatrix))
        {
            return true;
        }
    }
    return false;
}

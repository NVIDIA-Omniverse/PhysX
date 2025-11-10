// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include "ImmediateCreateGeometries.h"
#include "ImmediateMeshCache.h"
#include "ImmediateNode.h"
#include "ImmediateShared.h"
#include "../MultiThreader.h"

#include <PxPhysicsAPI.h>
#include <common/foundation/TypeCast.h>
using namespace ::physx;

using namespace omni::physx::graph;

void ImmediateCreateGeometries::createGeometries(const std::vector<MeshCookedData>& meshCookedData,
                                                 std::vector<MeshInputView>& inputMeshView,
                                                 bool computeBoundingBoxes)
{
    const uint32_t nbMeshes = (uint32_t)inputMeshView.size();
    meshGeometryData.clear();
    meshGeometryData.resize(nbMeshes);
    meshWorldBounds.resize(nbMeshes);

    struct Local
    {
        static void threadFunction(ImmediateCreateGeometries& self,
                                   uint32_t nbMeshes,
                                   const MeshCookedData* PX_RESTRICT meshCookedData,
                                   const MeshInputView* PX_RESTRICT meshInputViews,
                                   MeshGeometryData* PX_RESTRICT meshGeometryData,
                                   PxBounds3* PX_RESTRICT worldBounds,
                                   bool computeBoundingBoxes)
        {
            PX_SIMD_GUARD

            for (uint32_t i = 0; i < nbMeshes; i++)
            {
                meshGeometryData[i].isValid = meshCookedData[i].isValid;
                if (meshCookedData[i].isValid)
                {
                    meshGeometryData[i].meshFacesToTrianglesMapping = meshCookedData[i].meshFacesToTrianglesMapping;
                    meshGeometryData[i].worldMatrix = meshInputViews[i].worldMatrix;

                    if (meshCookedData[i].pxTriangleMesh)
                    {
                        omni::physx::toPhysX(meshGeometryData[i].worldTransform,
                                             meshGeometryData[i].triangleMeshGeometry.scale.scale,
                                             meshInputViews[i].worldMatrix);
                        meshGeometryData[i].triangleMeshGeometry.triangleMesh = meshCookedData[i].pxTriangleMesh;
                        if (computeBoundingBoxes)
                        {

                            if (self.enableTightBounds)
                                meshGeometryData[i].triangleMeshGeometry.meshFlags = PxMeshGeometryFlag::eTIGHT_BOUNDS;
                            const float epsilon = self.boundsEpsilon;
                            PxGeometryQuery::computeGeomBounds(worldBounds[i], meshGeometryData[i].triangleMeshGeometry,
                                                               meshGeometryData[i].worldTransform, epsilon,
                                                               1.0f + epsilon, PxGeometryQueryFlags(0));
                        }
                    }
                    else
                    {
                        omni::physx::toPhysX(meshGeometryData[i].worldTransform,
                                             meshGeometryData[i].convexMeshGeometry.scale.scale,
                                             meshInputViews[i].worldMatrix);
                        meshGeometryData[i].convexMeshGeometry.convexMesh = meshCookedData[i].pxConvexMesh;
                        if (computeBoundingBoxes)
                        {
                            if (self.enableTightBounds)
                                meshGeometryData[i].convexMeshGeometry.meshFlags =
                                    PxConvexMeshGeometryFlag::eTIGHT_BOUNDS;
                            const float epsilon = self.boundsEpsilon;
                            PxGeometryQuery::computeGeomBounds(worldBounds[i], meshGeometryData[i].convexMeshGeometry,
                                                               meshGeometryData[i].worldTransform, epsilon,
                                                               1.0f + epsilon, PxGeometryQueryFlags(0));
                        }
                    }
                }
                else
                {
                    worldBounds[i].setEmpty();
                }
            }
        }
    };

    if (shared->singleThreaded && shared->carbTasking)
    {
        MultiThreader mt(shared->numberOfTasks, nbMeshes);

        for (uint32_t i = 0; i < mt.mNbTasks; i++)
        {
            const uint32_t start = mt.mStarts[i];
            const uint32_t nbToGo = mt.mLoads[i];
            const MeshInputView* meshView = inputMeshView.data() + start;
            const MeshCookedData* cookedData = meshCookedData.data() + start;
            MeshGeometryData* geometryData = meshGeometryData.data() + start;
            PxBounds3* worldBounds = meshWorldBounds.data() + start;

            mt.mFutures[i] = shared->carbTasking->addTask(
                carb::tasking::Priority::eHigh, {},
                [this, nbToGo, meshView, cookedData, geometryData, worldBounds, computeBoundingBoxes] {
                    Local::threadFunction(
                        *this, nbToGo, cookedData, meshView, geometryData, worldBounds, computeBoundingBoxes);
                });
        }
    }
    else
    {
        Local::threadFunction(*this, nbMeshes, meshCookedData.data(), inputMeshView.data(), meshGeometryData.data(),
                              meshWorldBounds.data(), computeBoundingBoxes);
    }
}

// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include "ImmediateCookMeshes.h"
#include "ImmediateMeshCache.h"
#include "ImmediateNode.h"
#include "ImmediateShared.h"
#include "../MultiThreader.h"

#include <PxPhysicsAPI.h>
using namespace ::physx;
using namespace omni::physx::graph;
void ImmediateCookMeshes::cookMeshes(const std::vector<MeshInputView>& inputMeshView,
                                     const std::vector<::physx::PxU32>& uniqueMeshIndices,
                                     std::vector<MeshCookedData>& meshCookedData)
{
    const uint32_t nbToCook = (uint32_t)uniqueMeshIndices.size();
    if (nbToCook)
    {
        struct Local
        {
            static void threadFunction(ImmediateMeshCache& sharedMeshCache,
                                       uint32_t nbToCook,
                                       const uint32_t* PX_RESTRICT sources,
                                       const MeshInputView* PX_RESTRICT meshInputViews,
                                       MeshCookedData* PX_RESTRICT meshCookedData)
            {
                for (uint32_t i = 0; i < nbToCook; i++)
                {
                    const uint32_t sourceID = sources[i];

                    const MeshInputView& inputMesh = (const MeshInputView&)(meshInputViews[sourceID]);
                    MeshDefinitionView meshViewWithHashes;
                    meshViewWithHashes.meshView = inputMesh.meshView;
                    meshViewWithHashes.meshHashes = meshCookedData[sourceID].meshHashes;
                    meshViewWithHashes.meshCollision = inputMesh.meshCollision;
                    if (inputMesh.meshCollision.collisionApproximation ==
                        ImmediateNode::kCollisionApproximationConvexHull.token)
                    {
                        meshCookedData[sourceID].isValid = sharedMeshCache.queryConvexApproximationFor(
                            meshViewWithHashes, meshCookedData[sourceID].pxConvexMesh,
                            meshCookedData[sourceID].meshFacesToTrianglesMapping);
                    }
                    else
                    {
                        meshCookedData[sourceID].isValid = sharedMeshCache.queryTriangleApproximationFor(
                            meshViewWithHashes, meshCookedData[sourceID].pxTriangleMesh,
                            meshCookedData[sourceID].meshFacesToTrianglesMapping);
                    }
                }
            }
        };

        if (!shared->singleThreaded && shared->carbTasking)
        {
            MultiThreader mt(shared->numberOfTasks, nbToCook);

            for (uint32_t i = 0; i < mt.mNbTasks; i++)
            {
                const uint32_t start = mt.mStarts[i];
                const uint32_t nbToCook = mt.mLoads[i];
                const uint32_t* toCook = uniqueMeshIndices.data() + start;
                const MeshInputView* prims = inputMeshView.data();
                MeshCookedData* cookedData = meshCookedData.data();

                ImmediateMeshCache& immediateMeshCache = shared->immediateMeshCache;
                mt.mFutures[i] = shared->carbTasking->addTask(
                    carb::tasking::Priority::eHigh, {}, [&immediateMeshCache, nbToCook, toCook, prims, cookedData] {
                        Local::threadFunction(immediateMeshCache, nbToCook, toCook, prims, cookedData);
                    });
            }
        }
        else
        {
            Local::threadFunction(shared->immediateMeshCache, nbToCook, uniqueMeshIndices.data(), inputMeshView.data(),
                                  meshCookedData.data());
        }
    }

    const uint32_t nbMeshes = (uint32_t)inputMeshView.size();
    for (uint32_t meshIdx = 0; meshIdx < nbMeshes; meshIdx++)
    {
        const uint32_t sourceIdx = meshCookedData[meshIdx].uniqueMeshIndex;
        if (meshIdx != sourceIdx)
        {
            meshCookedData[meshIdx].isValid = sourceIdx != 0xffffffff ? meshCookedData[sourceIdx].isValid : false;
        }
        if (meshCookedData[meshIdx].isValid)
        {
            if (meshCookedData[meshIdx].pxTriangleMesh || meshCookedData[meshIdx].pxConvexMesh)
            {
                if (sourceIdx != meshIdx)
                    CARB_LOG_WARN(
                        "ImmediateCookMeshes: mesh at index %d has different cooked data from uniqueMeshIndex=%d",
                        meshIdx, sourceIdx);
            }
            else
            {
                meshCookedData[meshIdx].pxTriangleMesh = meshCookedData[sourceIdx].pxTriangleMesh;
                meshCookedData[meshIdx].pxConvexMesh = meshCookedData[sourceIdx].pxConvexMesh;
                meshCookedData[meshIdx].meshFacesToTrianglesMapping =
                    meshCookedData[sourceIdx].meshFacesToTrianglesMapping;
            }
        }
    }
}

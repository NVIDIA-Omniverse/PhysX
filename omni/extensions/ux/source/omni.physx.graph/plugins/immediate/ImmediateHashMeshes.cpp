// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include "ImmediateHashMeshes.h"
#include "ImmediateMeshCache.h"
#include "ImmediateShared.h"
#include "../MultiThreader.h"

#include <PxPhysicsAPI.h>
using namespace ::physx;
using namespace omni::physx::graph;

void ImmediateHashMeshes::computeInitialMeshData()
{
    struct Local
    {
        static void threadFunction(uint32_t nbPrims,
                                   const MeshInputView* PX_RESTRICT meshInputViews,
                                   MeshCookedData* PX_RESTRICT meshCookedData,
                                   ImmediateMeshCache& immediateMeshCache)
        {
            for (uint32_t i = 0; i < nbPrims; i++)
            {
                const MeshInputView& inputMesh = (const MeshInputView&)(meshInputViews[i]);
                MeshCookedData& cookedData = meshCookedData[i];
                if(inputMesh.meshView.usePrimID)
                {
                    cookedData.meshHashes.meshKey = inputMesh.meshKey;
                    cookedData.isValid = immediateMeshCache.computeCookedDataCRCForMeshInputView(
                        inputMesh, cookedData.meshHashes.meshKey, cookedData.meshHashes.cookedDataCRC);
                }
                else
                {
                    const uint32_t pointCount = uint32_t(inputMesh.meshView.cookingMeshView.points.size());
                    const uint32_t indicesCount = uint32_t(inputMesh.meshView.cookingMeshView.indices.size());
                    const uint32_t facesCount = uint32_t(inputMesh.meshView.cookingMeshView.faces.size());
                    if (pointCount && indicesCount && facesCount)
                    {
                        cookedData.meshHashes.meshKey = inputMesh.meshKey;
                        cookedData.isValid = immediateMeshCache.computeCookedDataCRCForMeshInputView(
                            inputMesh, cookedData.meshHashes.meshKey, cookedData.meshHashes.cookedDataCRC);
                    }
                    else
                    {
                        cookedData.isValid = false;
                    }
                }
            }
        }
    };

    const uint32_t nbMeshes = (uint32_t)inputMeshView.size();

    meshCookedData.clear();
    meshCookedData.resize(nbMeshes);

    ImmediateMeshCache& immediateMeshCache = shared->immediateMeshCache;

    if (!shared->singleThreaded && shared->carbTasking)
    {
        MultiThreader mt(shared->numberOfTasks, nbMeshes);

        for (uint32_t i = 0; i < mt.mNbTasks; i++)
        {
            const uint32_t start = mt.mStarts[i];
            const uint32_t nbPrims = mt.mLoads[i];

            const MeshInputView* prims = inputMeshView.data() + start;
            MeshCookedData* cookedData = meshCookedData.data() + start;
            mt.mFutures[i] =
                shared->carbTasking->addTask(carb::tasking::Priority::eHigh, {}, [nbPrims, prims, cookedData, &immediateMeshCache] {
                    Local::threadFunction(nbPrims, prims, cookedData, immediateMeshCache);
                });
        }
    }
    else
    {
        Local::threadFunction(nbMeshes, inputMeshView.data(), meshCookedData.data(), immediateMeshCache);
    }
}

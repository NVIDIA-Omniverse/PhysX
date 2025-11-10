// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once
#include "../MeshTypes.h"
namespace omni
{
namespace physx
{
namespace graph
{
struct ImmediateShared;

struct ImmediateFindDuplicates
{
    struct MeshIndex
    {
        ::physx::PxU32 meshIndex = 0xffffffff;
    };
    // Temporaries
    std::unordered_map<MeshKey, MeshIndex, MeshKeyHash> uniqueMeshHashMap;

    // Outputs
    std::vector<::physx::PxU32> uniqueMeshIndices;

    void init(ImmediateShared& immediateShared)
    {
    }

    void release()
    {
        uniqueMeshHashMap.clear();
        uniqueMeshIndices.clear();
    }

    void findSharedMeshes(std::vector<MeshCookedData>& meshCookedData)
    {
        uniqueMeshHashMap.clear();
        uniqueMeshIndices.clear();
        uniqueMeshHashMap.reserve(meshCookedData.size());
        uniqueMeshIndices.reserve(meshCookedData.size());
        const uint32_t nbMeshes = (uint32_t)meshCookedData.size();
        uint32_t nbSharedMeshes = 0;
        for (uint32_t i = 0; i < nbMeshes; i++)
        {
            if (meshCookedData[i].isValid)
            {
                MeshIndex& index = uniqueMeshHashMap[meshCookedData[i].meshHashes.cookedDataCRC];
                if (index.meshIndex != 0xffffffff)
                {
                    nbSharedMeshes++;
                    meshCookedData[i].uniqueMeshIndex = index.meshIndex;
                }
                else
                {
                    uniqueMeshIndices.push_back(i);
                    index.meshIndex = i;
                    meshCookedData[i].uniqueMeshIndex = i;
                }
            }
        }
    }
};
} // namespace graph
} // namespace physx
} // namespace omni

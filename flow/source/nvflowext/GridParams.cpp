// SPDX-FileCopyrightText: Copyright (c) 2014-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "NvFlowExt.h"

#include "NvFlowMath.h"
#include "NvFlowArray.h"

#include "LuidTable.h"

#include <string.h>
#include <mutex>
#include <atomic>
#include <chrono>
#include <thread>

namespace NvFlowGridParamsSimple
{
    struct ParamType
    {
        const NvFlowReflectDataType* dataType;
        const char* displayTypename;
    };

    struct Snapshot
    {
        NvFlowGridParamsDescSnapshot snapshot;
        NvFlowArray<NvFlowUint8> userdata;

        // TEMPORARY override to add ray march parameters with levelCount > 0u until Kit is updated
        NvFlowArray<NvFlowDatabaseTypeSnapshot> typeSnapshots_override;
        NvFlowArray<NvFlowGridRenderLayerParams*> instanceDatas_override;
        NvFlowArray<int> instanceDatas_rootLevel;
        NvFlowArrayPointer<NvFlowGridRenderLayerParams*> renderLayerParams_override;
    };

    struct GridParamsSnapshot
    {
        NvFlowGridParamsDesc paramsDesc = {};
        std::atomic_int32_t refCount;
    };

    NV_FLOW_CAST_PAIR(NvFlowGridParamsSnapshot, GridParamsSnapshot)

    struct GridParams
    {
        // constant after initialization
        NvFlowArray<ParamType> paramTypes;

        // joint
        std::mutex mutex;
        NvFlowArrayPointer<Snapshot*> committedSnapshots;
        NvFlowArrayPointer<Snapshot*> inUseSnapshots;
        NvFlowArrayPointer<Snapshot*> freeSnapshots;
        NvFlowArray<NvFlowGridParamsDescSnapshot> packedSnapshots;
        std::atomic_uint64_t stagingVersion;
        std::atomic_uint64_t minActiveVersion;

        // consumer
        NvFlowUint64 pullId = 0llu;
        GridParamsSnapshot snapshot = {};
    };

    NV_FLOW_CAST_PAIR(NvFlowGridParams, GridParams)

    NvFlowGridParams* createGridParams()
    {
        auto grid = new GridParams();

        grid->stagingVersion.store(1llu);
        grid->minActiveVersion.store(1llu);

        grid->paramTypes.pushBack(ParamType{ &NvFlowGridSimulateLayerParams_NvFlowReflectDataType,        "FlowSimulate"});
        grid->paramTypes.pushBack(ParamType{ &NvFlowGridOffscreenLayerParams_NvFlowReflectDataType,        "FlowOffscreen"});
        grid->paramTypes.pushBack(ParamType{ &NvFlowGridRenderLayerParams_NvFlowReflectDataType,        "FlowRender"});
        grid->paramTypes.pushBack(ParamType{ &NvFlowGridIsosurfaceLayerParams_NvFlowReflectDataType,    "FlowIsosurface"});
        grid->paramTypes.pushBack(ParamType{ &NvFlowGridEmitterSphereParams_NvFlowReflectDataType,        "FlowEmitterSphere"});
        grid->paramTypes.pushBack(ParamType{ &NvFlowGridEmitterBoxParams_NvFlowReflectDataType,            "FlowEmitterBox"});
        grid->paramTypes.pushBack(ParamType{ &NvFlowGridEmitterPointParams_NvFlowReflectDataType,        "FlowEmitterPoint"});
        grid->paramTypes.pushBack(ParamType{ &NvFlowGridEmitterMeshParams_NvFlowReflectDataType,        "FlowEmitterMesh" });
        grid->paramTypes.pushBack(ParamType{ &NvFlowGridEmitterTextureParams_NvFlowReflectDataType,        "FlowEmitterTexture"});
        grid->paramTypes.pushBack(ParamType{ &NvFlowGridEmitterNanoVdbParams_NvFlowReflectDataType,        "FlowEmitterNanoVdb"});

        grid->snapshot.refCount.store(0);

        return cast(grid);
    }

    void forceResetParams(GridParams* grid)
    {
        grid->committedSnapshots.deletePointers();
        grid->inUseSnapshots.deletePointers();
        grid->freeSnapshots.deletePointers();

        NvFlowGridParamsDesc nullParamsDesc = {};
        grid->snapshot.paramsDesc = nullParamsDesc;
    }

    NvFlowBool32 resetParams(NvFlowGridParams* gridIn)
    {
        auto grid = cast(gridIn);

        NvFlowBool32 resetSucceeded = NV_FLOW_FALSE;
        {
            std::lock_guard<std::mutex> mutex_lock(grid->mutex);

            int hangupValue = -0x20000000;
            int maxAttempts = 50;

            grid->snapshot.refCount.fetch_add(hangupValue);
            int latestRefCount = grid->snapshot.refCount.load();
            int attempts = 0;
            while (latestRefCount != hangupValue && attempts < maxAttempts)
            {
                latestRefCount = grid->snapshot.refCount.load();
                attempts++;
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            if (attempts < maxAttempts)
            {
                forceResetParams(grid);

                resetSucceeded = NV_FLOW_TRUE;
            }

            // restore ref count to allow future map to succeed.
            grid->snapshot.refCount.fetch_sub(hangupValue);
        }
        return resetSucceeded;
    }

    void destroyGridParams(NvFlowGridParams* gridParams)
    {
        auto grid = cast(gridParams);

        if (!resetParams(gridParams))
        {
            forceResetParams(grid);
        }

        delete grid;
    }

    void enumerateParamTypes(
        NvFlowGridParams* gridParams,
        const char** pTypenames,
        const char** pDisplayTypenames,
        const NvFlowReflectDataType** pDataTypes,
        NvFlowUint64* pCount
    )
    {
        auto grid = cast(gridParams);
        if (!pTypenames && !pDisplayTypenames)
        {
            (*pCount) = grid->paramTypes.size;
        }
        else
        {
            if ((*pCount) > grid->paramTypes.size)
            {
                (*pCount) = grid->paramTypes.size;
            }
            if (pTypenames)
            {
                for (NvFlowUint index = 0u; index < (*pCount); index++)
                {
                    pTypenames[index] = grid->paramTypes[index].dataType->structTypename;
                }
            }
            if (pDisplayTypenames)
            {
                for (NvFlowUint index = 0u; index < (*pCount); index++)
                {
                    pDisplayTypenames[index] = grid->paramTypes[index].displayTypename;
                }
            }
            if (pDataTypes)
            {
                for (NvFlowUint index = 0u; index < (*pCount); index++)
                {
                    pDataTypes[index] = grid->paramTypes[index].dataType;
                }
            }
        }
    }

    void getVersion(
        NvFlowGridParams* gridIn,
        NvFlowUint64* pStagingVersion,
        NvFlowUint64* pMinActiveVersion
        )
    {
        auto grid = cast(gridIn);
        if (pStagingVersion)
        {
            *pStagingVersion = grid->stagingVersion.fetch_add(1u);
        }
        if (pMinActiveVersion)
        {
            *pMinActiveVersion = grid->minActiveVersion.load();
        }
    }

    // Note, must be done under lock
    void updateMinActiveVersion(GridParams* grid)
    {
        // resolve min active version
        NvFlowUint64 minActiveVersion = ~0llu;
        for (NvFlowUint64 idx = 0u; idx < grid->inUseSnapshots.size; idx++)
        {
            if (grid->inUseSnapshots[idx]->snapshot.snapshot.version < minActiveVersion)
            {
                minActiveVersion = grid->inUseSnapshots[idx]->snapshot.snapshot.version;
            }
        }
        for (NvFlowUint64 idx = 0u; idx < grid->committedSnapshots.size; idx++)
        {
            if (grid->committedSnapshots[idx]->snapshot.snapshot.version < minActiveVersion)
            {
                minActiveVersion = grid->committedSnapshots[idx]->snapshot.snapshot.version;
            }
        }
        if (minActiveVersion != ~0llu)
        {
            grid->minActiveVersion.store(minActiveVersion);
        }
    }

    void commitParams(
        NvFlowGridParams* gridIn,
        const NvFlowGridParamsDescSnapshot* snapshot
    )
    {
        auto grid = cast(gridIn);

        // lock mutex to acquire copy instance
        {
            std::lock_guard<std::mutex> mutex_lock(grid->mutex);

            Snapshot* ptr = nullptr;
            if (grid->freeSnapshots.size > 0u)
            {
                ptr = grid->freeSnapshots[grid->freeSnapshots.size - 1u];
                grid->freeSnapshots[grid->freeSnapshots.size - 1u] = nullptr;
                grid->freeSnapshots.size--;
                grid->committedSnapshots.pushBackPointer(ptr);
            }
            else
            {
                ptr = grid->committedSnapshots.allocateBackPointer();
            }

            if (snapshot)
            {
                // copy snapshot
                ptr->snapshot = *snapshot;
                // make copy of userdata
                ptr->userdata.reserve(snapshot->userdataSizeInBytes);
                ptr->userdata.size = snapshot->userdataSizeInBytes;
                if (snapshot->userdataSizeInBytes > 0llu && snapshot->userdata)
                {
                    memcpy(ptr->userdata.data, snapshot->userdata, snapshot->userdataSizeInBytes);
                }
                // override in captured snapshot
                ptr->snapshot.userdata = ptr->userdata.data;

                // TEMPORARY override to add ray march parameters with levelCount > 0u until Kit is updated
                // check if levelCount > 0u
                NvFlowBool32 levelCountActive = NV_FLOW_FALSE;
                NvFlowUint64 simSnapshotIdx = ~0u;
                for (NvFlowUint64 typeSnapshotIdx = 0u; typeSnapshotIdx < snapshot->snapshot.typeSnapshotCount; typeSnapshotIdx++)
                {
                    const NvFlowDatabaseTypeSnapshot* typeSnapshot = &snapshot->snapshot.typeSnapshots[typeSnapshotIdx];
                    if (typeSnapshot->dataType->elementSize == sizeof(NvFlowGridSimulateLayerParams) &&
                        strcmp(typeSnapshot->dataType->structTypename, "NvFlowGridSimulateLayerParams") == 0)
                    {
                        simSnapshotIdx = typeSnapshotIdx;
                        for (NvFlowUint64 instanceIdx = 0u; instanceIdx < typeSnapshot->instanceCount; instanceIdx++)
                        {
                            NvFlowGridSimulateLayerParams* simParams = (NvFlowGridSimulateLayerParams*)typeSnapshot->instanceDatas[instanceIdx];
                            if (simParams->levelCount > 0u)
                            {
                                levelCountActive = NV_FLOW_TRUE;
                                break;
                            }
                        }
                        break;
                    }
                }
                if (levelCountActive)
                {
                    NvFlowUint64 renderSnapshotIdx = ~0u;

                    ptr->typeSnapshots_override.reserve(snapshot->snapshot.typeSnapshotCount);
                    ptr->typeSnapshots_override.size = snapshot->snapshot.typeSnapshotCount;
                    for (NvFlowUint64 typeSnapshotIdx = 0u; typeSnapshotIdx < ptr->typeSnapshots_override.size; typeSnapshotIdx++)
                    {
                        const NvFlowDatabaseTypeSnapshot* typeSnapshot = &snapshot->snapshot.typeSnapshots[typeSnapshotIdx];
                        ptr->typeSnapshots_override[typeSnapshotIdx] = *typeSnapshot;

                        if (typeSnapshot->dataType->elementSize == sizeof(NvFlowGridRenderLayerParams) &&
                            strcmp(typeSnapshot->dataType->structTypename, "NvFlowGridRenderLayerParams") == 0)
                        {
                            renderSnapshotIdx = typeSnapshotIdx;
                        }
                    }

                    if (simSnapshotIdx != ~0u && renderSnapshotIdx != ~0u)
                    {
                        const NvFlowDatabaseTypeSnapshot* simSnapshot = &snapshot->snapshot.typeSnapshots[simSnapshotIdx];
                        const NvFlowDatabaseTypeSnapshot* renderSnapshot = &snapshot->snapshot.typeSnapshots[renderSnapshotIdx];

                        ptr->instanceDatas_override.reserve(renderSnapshot->instanceCount);
                        ptr->instanceDatas_override.size = renderSnapshot->instanceCount;
                        ptr->instanceDatas_rootLevel.reserve(renderSnapshot->instanceCount);
                        ptr->instanceDatas_rootLevel.size = renderSnapshot->instanceCount;
                        for (NvFlowUint64 instanceIdx = 0u; instanceIdx < ptr->instanceDatas_override.size; instanceIdx++)
                        {
                            ptr->instanceDatas_override[instanceIdx] = (NvFlowGridRenderLayerParams*)renderSnapshot->instanceDatas[instanceIdx];
                            ptr->instanceDatas_rootLevel[instanceIdx] = ptr->instanceDatas_override[instanceIdx]->level;
                        }

                        ptr->renderLayerParams_override.size = 0u;
                        for (NvFlowUint64 simIdx = 0u; simIdx < simSnapshot->instanceCount; simIdx++)
                        {
                            NvFlowGridSimulateLayerParams* simParams = (NvFlowGridSimulateLayerParams*)simSnapshot->instanceDatas[simIdx];
                            for (int levelIdx = 1; levelIdx < simParams->levelCount; levelIdx++)
                            {
                                int simLayer = simParams->layer;
                                int simLevel = simParams->level + levelIdx;
                                // find best match in source
                                NvFlowGridRenderLayerParams* renderParamsOrig = nullptr;
                                NvFlowUint closestLevelDiff = ~0u;
                                for (NvFlowUint64 renderIdx = 0u; renderIdx < renderSnapshot->instanceCount; renderIdx++)
                                {
                                    NvFlowGridRenderLayerParams* renderParams = (NvFlowGridRenderLayerParams*)renderSnapshot->instanceDatas[renderIdx];
                                    if (simLayer == simParams->layer && renderParams->level >= simParams->level && renderParams->level <= simLevel)
                                    {
                                        NvFlowUint levelDiff = (NvFlowUint)(simLevel - renderParams->level);
                                        if (levelDiff <= closestLevelDiff)
                                        {
                                            closestLevelDiff = levelDiff;
                                            renderParamsOrig = renderParams;
                                        }
                                    }
                                }
                                // make override instance
                                NvFlowGridRenderLayerParams* renderParams_override = ptr->renderLayerParams_override.allocateBackPointer();
                                *renderParams_override = *renderParamsOrig;
                                renderParams_override->level += levelIdx;
                                renderParams_override->layer |= (levelIdx << 24); // Workaround until Kit/RTX updates
                                // add instance, but check for existing
                                NvFlowUint64 instanceIdx = 0u;
                                for (; instanceIdx < ptr->instanceDatas_override.size; instanceIdx++)
                                {
                                    if (NvFlow_packLayerAndLevel(simLayer, simLevel) ==
                                        NvFlow_packLayerAndLevel(ptr->instanceDatas_override[instanceIdx]->layer, ptr->instanceDatas_override[instanceIdx]->level))
                                    {
                                        if (renderParamsOrig->level >= ptr->instanceDatas_rootLevel[instanceIdx])
                                        {
                                            ptr->instanceDatas_rootLevel[instanceIdx] = renderParamsOrig->level;
                                            ptr->instanceDatas_override[instanceIdx] =  renderParams_override;
                                        }
                                        break;
                                    }
                                }
                                if (instanceIdx == ptr->instanceDatas_override.size)
                                {
                                    ptr->instanceDatas_rootLevel.pushBack(renderParamsOrig->level);
                                    ptr->instanceDatas_override.pushBack(renderParams_override);
                                }
                            }
                        }

                        ptr->typeSnapshots_override[renderSnapshotIdx].instanceDatas = (NvFlowUint8**)ptr->instanceDatas_override.data;
                        ptr->typeSnapshots_override[renderSnapshotIdx].instanceCount = ptr->instanceDatas_override.size;
                    }
                    ptr->snapshot.snapshot.typeSnapshots = ptr->typeSnapshots_override.data;
                }
                // END TEMPORARY override to add ray march parameters with levelCount > 0u until Kit is updated
            }
            else
            {
                NvFlowUint64 stagingVersion = 0llu;
                NvFlowUint64 minActiveVersion = 0llu;
                getVersion(gridIn, &stagingVersion, &minActiveVersion);

                NvFlowGridParamsDescSnapshot nullSnapshot = {};
                ptr->snapshot = nullSnapshot;

                // set version
                ptr->snapshot.snapshot.version = stagingVersion;
            }
        }
    }

    NvFlowGridParamsSnapshot* getParamsSnapshot(
        NvFlowGridParams* gridIn,
        double absoluteSimTime,
        NvFlowUint64 pullId
    )
    {
        auto grid = cast(gridIn);

        std::lock_guard<std::mutex> mutex_lock(grid->mutex);

        if (pullId == 0llu || pullId != grid->pullId)
        {
            grid->pullId = pullId;

            // compute eligible committed count
            NvFlowUint64 eligibleCommitted = 0llu;
            for (NvFlowUint64 committedIdx = 0u; committedIdx < grid->committedSnapshots.size; committedIdx++)
            {
                if (grid->committedSnapshots[committedIdx]->snapshot.absoluteSimTime <= absoluteSimTime)
                {
                    eligibleCommitted++;
                }
            }

            // if nothing committed retain latest inUse but set deltaTime to zero
            if (eligibleCommitted == 0llu)
            {
                // free all except last
                if (grid->inUseSnapshots.size > 1u)
                {
                    for (NvFlowUint64 inUseIdx = 0u; inUseIdx < grid->inUseSnapshots.size - 1u; inUseIdx++)
                    {
                        grid->freeSnapshots.pushBackPointer(grid->inUseSnapshots[inUseIdx]);
                        grid->inUseSnapshots[inUseIdx] = nullptr;
                    }
                    // move last to front
                    grid->inUseSnapshots[0u] = grid->inUseSnapshots[grid->inUseSnapshots.size - 1u];
                    grid->inUseSnapshots[grid->inUseSnapshots.size - 1u] = nullptr;
                    grid->inUseSnapshots.size = 1u;
                }
            }
            else // move all elligible to inUse, free previous in use
            {
                // move in use to free
                for (NvFlowUint64 inUseIdx = 0u; inUseIdx < grid->inUseSnapshots.size; inUseIdx++)
                {
                    grid->freeSnapshots.pushBackPointer(grid->inUseSnapshots[inUseIdx]);
                    grid->inUseSnapshots[inUseIdx] = nullptr;
                }
                grid->inUseSnapshots.size = 0u;

                // move committed values less than or equal to absoluteSimTime to in use
                for (NvFlowUint64 committedIdx = 0u; committedIdx < grid->committedSnapshots.size; committedIdx++)
                {
                    if (grid->committedSnapshots[committedIdx]->snapshot.absoluteSimTime <= absoluteSimTime)
                    {
                        grid->inUseSnapshots.pushBackPointer(grid->committedSnapshots[committedIdx]);
                        grid->committedSnapshots[committedIdx] = nullptr;
                    }
                }

                // compact committed array
                NvFlowUint64 keepCount = 0llu;
                for (NvFlowUint64 committedIdx = 0u; committedIdx < grid->committedSnapshots.size; committedIdx++)
                {
                    if (grid->committedSnapshots[committedIdx])
                    {
                        grid->committedSnapshots.swapPointers(keepCount, committedIdx);
                        keepCount++;
                    }
                }
                grid->committedSnapshots.size = keepCount;
            }

            // build NvFlowGridParamsDesc
            grid->packedSnapshots.size = 0u;
            grid->packedSnapshots.reserve(grid->inUseSnapshots.size);
            grid->packedSnapshots.size = grid->inUseSnapshots.size;
            for (NvFlowUint64 inUseIdx = 0u; inUseIdx < grid->inUseSnapshots.size; inUseIdx++)
            {
                grid->packedSnapshots[inUseIdx] = grid->inUseSnapshots[inUseIdx]->snapshot;
                // override to set deltaTime to 0.0 in nothing committed state
                if (eligibleCommitted == 0llu)
                {
                    grid->packedSnapshots[inUseIdx].deltaTime = 0.f;
                }
            }

            NvFlowGridParamsDesc paramsDesc = {};
            paramsDesc.snapshots = grid->packedSnapshots.data;
            paramsDesc.snapshotCount = grid->packedSnapshots.size;

            grid->snapshot.paramsDesc = paramsDesc;

            // While locked, update min active
            updateMinActiveVersion(grid);
        }

        return cast(&grid->snapshot);
    }

    NvFlowBool32 mapParamsDesc(
        NvFlowGridParams* gridIn,
        NvFlowGridParamsSnapshot* snapshotIn,
        NvFlowGridParamsDesc* pParamsDesc
    )
    {
        auto grid = cast(gridIn);
        auto snapshot = cast(snapshotIn);

        int oldRefCount = snapshot->refCount.fetch_add(1);
        if (oldRefCount < 0)
        {
            snapshot->refCount.fetch_sub(1);
            return NV_FLOW_FALSE;
        }

        if (pParamsDesc)
        {
            *pParamsDesc = snapshot->paramsDesc;
        }
        return NV_FLOW_TRUE;
    }

    void unmapParamsDesc(
        NvFlowGridParams* gridIn,
        NvFlowGridParamsSnapshot* snapshotIn
    )
    {
        auto grid = cast(gridIn);
        auto snapshot = cast(snapshotIn);

        snapshot->refCount.fetch_sub(1);
    }

    //----------------------- Named -------------------------------

    struct GridParamsNamed
    {
        int refCount = 0;
        NvFlowArray<char> name_data;
        NvFlowGridParams* gridParams = nullptr;
        ~GridParamsNamed()
        {
            if (gridParams)
            {
                destroyGridParams(gridParams);
                gridParams = nullptr;
            }
        }
    };

    NV_FLOW_CAST_PAIR(GridParamsNamed, NvFlowGridParamsNamed)

    std::mutex gGridParamsNamedMutex;
    NvFlowArrayPointer<GridParamsNamed*> gGridParamsNamed;

    NvFlowGridParamsNamed* createGridParamsNamed(const char* name)
    {
        std::lock_guard<std::mutex> lock(gGridParamsNamedMutex);

        // make nullptr and empty string the same
        if (name == nullptr)
        {
            name = "";
        }

        GridParamsNamed* gridParamsNamed = nullptr;
        for (NvFlowUint namedIdx = 0u; namedIdx < gGridParamsNamed.size; namedIdx++)
        {
            if (strcmp(name, gGridParamsNamed[namedIdx]->name_data.data) == 0)
            {
                gridParamsNamed = gGridParamsNamed[namedIdx];
                break;
            }
        }
        if (!gridParamsNamed)
        {
            gridParamsNamed = gGridParamsNamed.allocateBackPointer();

            gridParamsNamed->name_data.size = 0u;
            for (NvFlowUint charIdx = 0u; name[charIdx]; charIdx++)
            {
                gridParamsNamed->name_data.pushBack(name[charIdx]);
            }
            gridParamsNamed->name_data.pushBack(0);
        }

        // Create gridParams as needed
        if (!gridParamsNamed->gridParams)
        {
            gridParamsNamed->gridParams = createGridParams();
        }

        // Increment ref count
        gridParamsNamed->refCount++;

        return cast(gridParamsNamed);
    }

    int destroyGridParamsNamed(NvFlowGridParamsNamed* gridParamsNamedIn)
    {
        std::lock_guard<std::mutex> lock(gGridParamsNamedMutex);
        auto gridParamsNamed = cast(gridParamsNamedIn);

        gridParamsNamed->refCount--;
        if (gridParamsNamed->refCount == 0)
        {
            if (gridParamsNamed->gridParams)
            {
                destroyGridParams(gridParamsNamed->gridParams);
                gridParamsNamed->gridParams = nullptr;
            }
        }
        return gridParamsNamed->refCount;
    }

    NvFlowGridParams* mapGridParamsNamed(NvFlowGridParamsNamed* gridParamsNamedIn)
    {
        auto gridParamsNamed = cast(gridParamsNamedIn);

        return gridParamsNamed->gridParams;
    }

    //----------------------- GridParamsLayer -------------------------------

    struct GridParamsLayerSnapshot
    {
        NvFlowGridParamsDescSnapshot snapshotDesc = {};

        NvFlowArray<NvFlowDatabaseTypeSnapshot> typeSnapshots;

        NvFlowArray<NvFlowGridSimulateLayerParams*> simulate_instanceDatas;
        NvFlowArray<NvFlowGridRenderLayerParams*> render_instanceDatas;
        NvFlowArray<NvFlowGridEmitterPointParams*> emitterPoint_instanceDatas;
        NvFlowArray<NvFlowGridEmitterNanoVdbParams*> emitterNanoVdb_instanceDatas;

        NvFlowArrayPointer<NvFlowGridSimulateLayerParams*> simulate_pool;
        NvFlowArrayPointer<NvFlowGridRenderLayerParams*> render_pool;
        NvFlowArrayPointer<NvFlowGridEmitterPointParams*> emitterPoint_pool;
        NvFlowArrayPointer<NvFlowGridEmitterNanoVdbParams*> emitterNanoVdb_pool;
    };

    struct GridParamsLayer
    {
        NvFlowRingBufferPointer<GridParamsLayerSnapshot*> snapshots;

        NvFlowLuidTable luidTable;
    };

    NV_FLOW_CAST_PAIR(NvFlowGridParamsLayer, GridParamsLayer)

    NvFlowGridParamsLayer* createGridParamsLayerStandard()
    {
        auto ptr = new GridParamsLayer();

        return cast(ptr);
    }

    void destroyGridParamsLayer(NvFlowGridParamsLayer* layer)
    {
        auto ptr = cast(layer);

        delete ptr;
    }

    NvFlowGridParamsDescSnapshot applyGridParamsLayer(
        NvFlowGridParamsLayer* layer,
        const NvFlowGridParamsDescSnapshot* snapshot,
        NvFlowUint64 stagingVersion,
        NvFlowUint64 minActiveVersion
    )
    {
        auto ptr = cast(layer);

        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot->snapshot, NvFlowPointCloudParams)

        // free old snapshots
        while (ptr->snapshots.activeCount() > 1u && ptr->snapshots.front()->snapshotDesc.snapshot.version < minActiveVersion)
        {
            ptr->snapshots.popFront();
        }

        if (NvFlowPointCloudParams_elementCount > 0u)
        {
            GridParamsLayerSnapshot* newSnapshot = ptr->snapshots.allocateBackPointer();

            newSnapshot->simulate_pool.size = 0u;
            newSnapshot->render_pool.size = 0u;
            newSnapshot->emitterPoint_pool.size = 0u;
            newSnapshot->emitterNanoVdb_pool.size = 0u;

            NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot->snapshot, NvFlowGridSimulateLayerParams)
            NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot->snapshot, NvFlowGridRenderLayerParams)
            NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot->snapshot, NvFlowGridEmitterPointParams)
            NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot->snapshot, NvFlowGridEmitterNanoVdbParams)

            newSnapshot->simulate_instanceDatas.size = 0u;
            newSnapshot->render_instanceDatas.size = 0u;
            newSnapshot->emitterPoint_instanceDatas.size = 0u;
            newSnapshot->emitterNanoVdb_instanceDatas.size = 0u;

            newSnapshot->simulate_instanceDatas.pushBackN(NvFlowGridSimulateLayerParams_elements, NvFlowGridSimulateLayerParams_elementCount);
            newSnapshot->render_instanceDatas.pushBackN(NvFlowGridRenderLayerParams_elements, NvFlowGridRenderLayerParams_elementCount);
            newSnapshot->emitterPoint_instanceDatas.pushBackN(NvFlowGridEmitterPointParams_elements, NvFlowGridEmitterPointParams_elementCount);
            newSnapshot->emitterNanoVdb_instanceDatas.pushBackN(NvFlowGridEmitterNanoVdbParams_elements, NvFlowGridEmitterNanoVdbParams_elementCount);

            NvFlowUint64 tableCount =
                NvFlowGridSimulateLayerParams_elementCount +
                NvFlowGridRenderLayerParams_elementCount +
                NvFlowGridEmitterPointParams_elementCount +
                NvFlowGridEmitterNanoVdbParams_elementCount;

            static const NvFlowUint64 kTypeIdx_simulate = 0u;
            static const NvFlowUint64 kTypeIdx_render = 1u;
            static const NvFlowUint64 kTypeIdx_emitterPoint = 2u;
            static const NvFlowUint64 kTypeIdx_emitterNanoVdb = 3u;

            NvFlowLuidTable_reset(&ptr->luidTable, tableCount);
            for (NvFlowUint64 idx = 0u; idx < NvFlowGridSimulateLayerParams_elementCount; idx++)
            {
                NvFlowLuidTable_insert(&ptr->luidTable, NvFlowGridSimulateLayerParams_elements[idx]->luid, (kTypeIdx_simulate << 60llu) | idx);
            }
            for (NvFlowUint64 idx = 0u; idx < NvFlowGridRenderLayerParams_elementCount; idx++)
            {
                NvFlowLuidTable_insert(&ptr->luidTable, NvFlowGridRenderLayerParams_elements[idx]->luid, (kTypeIdx_render << 60llu) | idx);
            }
            for (NvFlowUint64 idx = 0u; idx < NvFlowGridEmitterPointParams_elementCount; idx++)
            {
                NvFlowLuidTable_insert(&ptr->luidTable, NvFlowGridEmitterPointParams_elements[idx]->luid, (kTypeIdx_emitterPoint << 60llu) | idx);
            }
            for (NvFlowUint64 idx = 0u; idx < NvFlowGridEmitterNanoVdbParams_elementCount; idx++)
            {
                NvFlowLuidTable_insert(&ptr->luidTable, NvFlowGridEmitterNanoVdbParams_elements[idx]->luid, (kTypeIdx_emitterNanoVdb << 60llu) | idx);
            }

            for (NvFlowUint64 pointCloudIdx = 0u; pointCloudIdx < NvFlowPointCloudParams_elementCount; pointCloudIdx++)
            {
                const NvFlowPointCloudParams* pointCloudParams = NvFlowPointCloudParams_elements[pointCloudIdx];

                // support disable of override
                if (!pointCloudParams->attributeCopyEnabled)
                {
                    continue;
                }

                for (NvFlowUint64 childIdx = 0u; childIdx < pointCloudParams->childLuidCount; childIdx++)
                {
                    NvFlowUint64 childLuid = pointCloudParams->childLuids[childIdx];
                    NvFlowUint64 findIdx = NvFlowLuidTable_find(&ptr->luidTable, childLuid);
                    if (findIdx == ~0llu)
                    {
                        continue;
                    }
                    NvFlowUint64 typeIdx = findIdx >> 60llu;
                    NvFlowUint64 instanceIdx = findIdx & ((1llu << 60llu) - 1u);
                    if (typeIdx == kTypeIdx_simulate)
                    {
                        auto params = newSnapshot->simulate_pool.allocateBackPointer();
                        *params = *newSnapshot->simulate_instanceDatas[instanceIdx];

                        params->forceClear = pointCloudParams->enabled ? NV_FLOW_FALSE : NV_FLOW_TRUE;
                        params->densityCellSize = pointCloudParams->cellSize;
                        params->autoCellSize = pointCloudParams->autoCellSize;
                        params->levelCount = pointCloudParams->levelCount;
                        params->enableLowPrecisionDensity = pointCloudParams->enableLowPrecision;
                        params->enableLowPrecisionVelocity = pointCloudParams->enableLowPrecision;
                        // forcesimulate?
                        params->enableSmallBlocks = pointCloudParams->enableSmallBlocks;
                        params->advection.forceFadeEnabled = pointCloudParams->fadeEnabled;
                        params->advection.temperature.fade = pointCloudParams->fadeRate;
                        params->advection.fuel.fade = pointCloudParams->fadeRate;
                        params->advection.burn.fade = pointCloudParams->fadeRate;
                        params->advection.smoke.fade = pointCloudParams->fadeRate;

                        newSnapshot->simulate_instanceDatas[instanceIdx] = params;
                    }
                    else if (typeIdx == kTypeIdx_render)
                    {
                        auto params = newSnapshot->render_pool.allocateBackPointer();
                        *params = *newSnapshot->render_instanceDatas[instanceIdx];

                        params->rayMarch.attenuation = pointCloudParams->rayMarchAttenuation;
                        params->rayMarch.colorScale = pointCloudParams->rayMarchColorScale;

                        newSnapshot->render_instanceDatas[instanceIdx] = params;
                    }
                    else if (typeIdx == kTypeIdx_emitterPoint)
                    {
                        auto params = newSnapshot->emitterPoint_pool.allocateBackPointer();
                        *params = *newSnapshot->emitterPoint_instanceDatas[instanceIdx];

                        params->enabled = pointCloudParams->enabled;
                        params->followVisibility = pointCloudParams->followVisibility;
                        params->enableStreaming = pointCloudParams->enableStreaming;
                        params->streamOnce = pointCloudParams->streamOnce;
                        params->streamClearAtStart = pointCloudParams->streamClearAtStart;
                        params->streamingBatchSize = pointCloudParams->streamingBatchSize;
                        params->levelCount = pointCloudParams->levelCount;
                        params->colorIsSrgb = pointCloudParams->colorIsSrgb;
                        params->widthScale = pointCloudParams->widthScale;
                        params->coupleRateTemperature = pointCloudParams->coupleRate;
                        params->coupleRateFuel = pointCloudParams->coupleRate;
                        params->coupleRateBurn = pointCloudParams->coupleRate;
                        params->coupleRateSmoke = pointCloudParams->coupleRate;

                        newSnapshot->emitterPoint_instanceDatas[instanceIdx] = params;
                    }
                    else if (typeIdx == kTypeIdx_emitterNanoVdb)
                    {
                        auto params = newSnapshot->emitterNanoVdb_pool.allocateBackPointer();
                        *params = *newSnapshot->emitterNanoVdb_instanceDatas[instanceIdx];

                        params->enabled = pointCloudParams->enabled;
                        params->followVisibility = pointCloudParams->followVisibility;
                        params->enableStreaming = pointCloudParams->enableStreaming;
                        params->streamOnce = pointCloudParams->streamOnce;
                        params->streamClearAtStart = pointCloudParams->streamClearAtStart;
                        params->streamingBatchSize = pointCloudParams->streamingBatchSize;
                        params->levelCount = pointCloudParams->levelCount;
                        params->colorIsSrgb = pointCloudParams->colorIsSrgb;
                        params->coupleRateTemperature = pointCloudParams->coupleRate;
                        params->coupleRateFuel = pointCloudParams->coupleRate;
                        params->coupleRateBurn = pointCloudParams->coupleRate;
                        params->coupleRateSmoke = pointCloudParams->coupleRate;

                        newSnapshot->emitterNanoVdb_instanceDatas[instanceIdx] = params;
                    }
                }
            }

            newSnapshot->typeSnapshots.size = 0u;
            newSnapshot->typeSnapshots.pushBackN(snapshot->snapshot.typeSnapshots, snapshot->snapshot.typeSnapshotCount);

            for (NvFlowUint64 typeIdx = 0u; typeIdx < newSnapshot->typeSnapshots.size; typeIdx++)
            {
                if (newSnapshot->typeSnapshots[typeIdx].dataType->elementSize == sizeof(NvFlowGridSimulateLayerParams) &&
                    strcmp(newSnapshot->typeSnapshots[typeIdx].dataType->structTypename, "NvFlowGridSimulateLayerParams") == 0)
                {
                    newSnapshot->typeSnapshots[typeIdx].instanceDatas = (NvFlowUint8**)newSnapshot->simulate_instanceDatas.data;
                    newSnapshot->typeSnapshots[typeIdx].instanceCount = newSnapshot->simulate_instanceDatas.size;
                }
                else if (newSnapshot->typeSnapshots[typeIdx].dataType->elementSize == sizeof(NvFlowGridRenderLayerParams) &&
                    strcmp(newSnapshot->typeSnapshots[typeIdx].dataType->structTypename, "NvFlowGridRenderLayerParams") == 0)
                {
                    newSnapshot->typeSnapshots[typeIdx].instanceDatas = (NvFlowUint8**)newSnapshot->render_instanceDatas.data;
                    newSnapshot->typeSnapshots[typeIdx].instanceCount = newSnapshot->render_instanceDatas.size;
                }
                else if (newSnapshot->typeSnapshots[typeIdx].dataType->elementSize == sizeof(NvFlowGridEmitterPointParams) &&
                    strcmp(newSnapshot->typeSnapshots[typeIdx].dataType->structTypename, "NvFlowGridEmitterPointParams") == 0)
                {
                    newSnapshot->typeSnapshots[typeIdx].instanceDatas = (NvFlowUint8**)newSnapshot->emitterPoint_instanceDatas.data;
                    newSnapshot->typeSnapshots[typeIdx].instanceCount = newSnapshot->emitterPoint_instanceDatas.size;
                }
                else if (newSnapshot->typeSnapshots[typeIdx].dataType->elementSize == sizeof(NvFlowGridEmitterNanoVdbParams) &&
                    strcmp(newSnapshot->typeSnapshots[typeIdx].dataType->structTypename, "NvFlowGridEmitterNanoVdbParams") == 0)
                {
                    newSnapshot->typeSnapshots[typeIdx].instanceDatas = (NvFlowUint8**)newSnapshot->emitterNanoVdb_instanceDatas.data;
                    newSnapshot->typeSnapshots[typeIdx].instanceCount = newSnapshot->emitterNanoVdb_instanceDatas.size;
                }
            }

            newSnapshot->snapshotDesc = *snapshot;
            newSnapshot->snapshotDesc.snapshot.typeSnapshots = newSnapshot->typeSnapshots.data;
            newSnapshot->snapshotDesc.snapshot.typeSnapshotCount = newSnapshot->typeSnapshots.size;
            return newSnapshot->snapshotDesc;
        }
        else
        {
            return *snapshot;
        }
    }
}

NvFlowGridParamsInterface* NvFlowGetGridParamsInterface()
{
    using namespace NvFlowGridParamsSimple;
    static NvFlowGridParamsInterface iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowGridParamsInterface) };
    iface.createGridParams = createGridParams;
    iface.destroyGridParams = destroyGridParams;
    iface.enumerateParamTypes = enumerateParamTypes;
    iface.getVersion = getVersion;
    iface.commitParams = commitParams;
    iface.resetParams = resetParams;
    iface.getParamsSnapshot = getParamsSnapshot;
    iface.mapParamsDesc = mapParamsDesc;
    iface.unmapParamsDesc = unmapParamsDesc;
    iface.createGridParamsNamed = createGridParamsNamed;
    iface.destroyGridParamsNamed = destroyGridParamsNamed;
    iface.mapGridParamsNamed = mapGridParamsNamed;
    iface.createGridParamsLayerStandard = createGridParamsLayerStandard;
    iface.destroyGridParamsLayer = destroyGridParamsLayer;
    iface.applyGridParamsLayer = applyGridParamsLayer;
    return &iface;
}

/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "EditorCommon.h"

struct NvFlowDatabasePrim
{
    NvFlowDatabasePrim* parent;
    const char* path;
    const char* name;
};

NV_FLOW_INLINE NvFlowDatabasePrim* createPrim(
    NvFlowDatabaseContext* context,
    NvFlowUint64 version,
    NvFlowDatabasePrim* parent,
    const char* displayTypename,
    const char* path,
    const char* name)
{
    auto prim = new NvFlowDatabasePrim();

    prim->parent = parent;
    prim->path = path;
    prim->name = name;

    printf("Create prim: displayTypename(%s), path(%s) name(%s)\n", displayTypename, path, name);

    return prim;
}

NV_FLOW_INLINE void updatePrim(
    NvFlowDatabaseContext* context,
    NvFlowUint64 version,
    NvFlowUint64 minActiveVersion,
    NvFlowDatabasePrim* prim)
{

}

NV_FLOW_INLINE void markDestroyedPrim(NvFlowDatabaseContext* context, NvFlowDatabasePrim* prim)
{
    printf("MarkDestroyed prim: path(%s) name(%s)\n", prim->path, prim->name);
}

NV_FLOW_INLINE void destroyPrim(NvFlowDatabaseContext* context, NvFlowDatabasePrim* prim)
{
    printf("Destroy prim: path(%s) name(%s)\n", prim->path, prim->name);

    delete prim;
}

struct NvFlowDatabaseValue
{
    NvFlowArray<NvFlowUint8> data;
    NvFlowUint64 version;
    NvFlowUint64 lastUsed;
};

struct NvFlowDatabaseAttr
{
    NvFlowDatabasePrim* prim;
    NvFlowRingBufferPointer<NvFlowDatabaseValue*> values;
};

NV_FLOW_INLINE NvFlowDatabaseValue* copyArray(
    NvFlowUint64 version,
    NvFlowUint64 minActiveVersion,
    NvFlowDatabaseAttr* attr,
    const NvFlowReflectData* reflectData,
    NvFlowUint8* mappedData,
    const void* srcData,
    NvFlowUint64 srcDataSizeInBytes
)
{
    auto value = attr->values.allocateBackPointer();

    value->version = version;
    value->lastUsed = version;
    value->data.size = 0u;

    NvFlowUint8** pData = (NvFlowUint8**)(mappedData + reflectData->dataOffset);
    NvFlowUint64* pArraySize = (NvFlowUint64*)(mappedData + reflectData->arraySizeOffset);

    value->data.reserve(srcDataSizeInBytes);
    value->data.size = srcDataSizeInBytes;
    if (srcData)
    {
        memcpy(value->data.data, srcData, srcDataSizeInBytes);
    }
    else
    {
        memset(value->data.data, 0, srcDataSizeInBytes);
    }

    // override to owned copy
    *pData = value->data.data;
    *pArraySize = srcDataSizeInBytes / reflectData->dataType->elementSize;
    if (reflectData->reflectMode == eNvFlowReflectMode_arrayVersioned)
    {
        NvFlowUint64* pVersion = (NvFlowUint64*)(mappedData + reflectData->versionOffset);

        // aligning array version to commit version, convenient by not required
        *pVersion = version;
    }

    return value;
}

NV_FLOW_INLINE NvFlowDatabaseAttr* createAttr(
    NvFlowDatabaseContext* context,
    NvFlowUint64 version,
    NvFlowDatabasePrim* prim,
    const NvFlowReflectData* reflectData,
    NvFlowUint8* mappedData)
{
    auto attr = new NvFlowDatabaseAttr();

    attr->prim = prim;

    // make copy of any read only arrays to allow in place edit
    if (reflectData->reflectMode == eNvFlowReflectMode_array ||
        reflectData->reflectMode == eNvFlowReflectMode_arrayVersioned)
    {
        NvFlowUint8** pData = (NvFlowUint8**)(mappedData + reflectData->dataOffset);
        NvFlowUint64* pArraySize = (NvFlowUint64*)(mappedData + reflectData->arraySizeOffset);
        NvFlowUint8* data = *pData;
        NvFlowUint64 arraySizeInBytes = (*pArraySize) * reflectData->dataType->elementSize;
        copyArray(version, version, attr, reflectData, mappedData, data, arraySizeInBytes);
    }
    return attr;
}

NV_FLOW_INLINE void updateAttr(
    NvFlowDatabaseContext* context,
    NvFlowUint64 version,
    NvFlowUint64 minActiveVersion,
    NvFlowDatabaseAttr* attr,
    const NvFlowReflectData* reflectData,
    NvFlowUint8* mappedData)
{
    EditorFlow* ptr = (EditorFlow*)context;

    // recycle before update to maximum chance of reuse
    if (reflectData->reflectMode == eNvFlowReflectMode_array ||
        reflectData->reflectMode == eNvFlowReflectMode_arrayVersioned)
    {
        // leave 1 to allow copy/migrate
        while (attr->values.activeCount() > 1u && attr->values.front()->lastUsed < minActiveVersion)
        {
            //printf("Popping %s version %llu lastUsed %llu\n", reflectData->name, attr->values.front()->version, attr->values.front()->lastUsed);
            attr->values.popFront();
        }
    }

    for (NvFlowUint64 idx = 0u; idx < ptr->commands.size; idx++)
    {
        EditorFlowCommand* cmd = &ptr->commands[idx];
        NvFlowUint* cmdState = &ptr->commandStates[idx];
        if ((*cmdState) == 0u &&
            strcmp(cmd->cmd, "setAttribute") == 0 &&
            strcmp(cmd->name, reflectData->name) == 0 &&
            strcmp(cmd->path, attr->prim->path) == 0)
        {
            if (reflectData->reflectMode == eNvFlowReflectMode_value ||
                reflectData->reflectMode == eNvFlowReflectMode_valueVersioned)
            {
                if (reflectData->dataType->elementSize == cmd->dataSize)
                {
                    memcpy(mappedData + reflectData->dataOffset, cmd->data, cmd->dataSize);
                    (*cmdState) = 1u; // mark completed
                }
            }
            else if (reflectData->reflectMode == eNvFlowReflectMode_array ||
                     reflectData->reflectMode == eNvFlowReflectMode_arrayVersioned)
            {
                copyArray(version, minActiveVersion, attr, reflectData, mappedData, cmd->data, cmd->dataSize);
                (*cmdState) = 1u; // mark completed
            }
        }
    }

    // free at end, in case new array allows old to free
    if (reflectData->reflectMode == eNvFlowReflectMode_array ||
        reflectData->reflectMode == eNvFlowReflectMode_arrayVersioned)
    {
        if (attr->values.activeCount() > 0u)
        {
            attr->values.back()->lastUsed = version;
        }

        // leave 1 to allow copy/migrate
        while (attr->values.activeCount() > 1u && attr->values.front()->lastUsed < minActiveVersion)
        {
            //printf("Popping %s version %llu lastUsed %llu\n", reflectData->name, attr->values.front()->version, attr->values.front()->lastUsed);
            attr->values.popFront();
        }
    }
}

NV_FLOW_INLINE void markDestroyedAttr(NvFlowDatabaseContext* context, NvFlowDatabaseAttr* attr)
{

}

NV_FLOW_INLINE void destroyAttr(NvFlowDatabaseContext* context, NvFlowDatabaseAttr* attr)
{
    delete attr;
}

static const NvFlowDatabaseInterface iface = {
    createPrim, updatePrim, markDestroyedPrim, destroyPrim,
    createAttr, updateAttr, markDestroyedAttr, destroyAttr
};

void editorFlow_init(EditorCompute* ctx, EditorFlow* ptr)
{
    NvFlowContext* context = ctx->loader.deviceInterface.getContext(ctx->deviceQueue);

    ptr->commandStringPool = NvFlowStringPoolCreate();

    NvFlowGridDesc gridDesc = NvFlowGridDesc_default;

    ptr->maxLocations = ptr->targetMaxLocations;
    gridDesc.maxLocations = ptr->maxLocations;

    ptr->grid = ctx->loader.gridInterface.createGrid(&ctx->contextInterface, context, &ctx->loader.opList, &ctx->loader.extOpList, &gridDesc);
    ptr->gridParamsServer = ctx->loader.gridParamsInterface.createGridParamsNamed(nullptr);
    ptr->gridParamsClient = ctx->loader.gridParamsInterface.createGridParamsNamed(nullptr);

    ptr->gridParams = ctx->loader.gridParamsInterface.mapGridParamsNamed(ptr->gridParamsServer);

    //ptr->loader.gridInterface.setResourceMinLifetime(context, ptr->grid, 0u);

    editorCompute_logPrint(eNvFlowLogLevel_info, "Initialized Flow Grid");

    NvFlowUint64 typeCount = 0u;
    ctx->loader.gridParamsInterface.enumerateParamTypes(ptr->gridParams, nullptr, nullptr, nullptr, &typeCount);

    ptr->typenames.reserve(typeCount);
    ptr->typenames.size = typeCount;
    ptr->displayTypenames.reserve(typeCount);
    ptr->displayTypenames.size = typeCount;
    ptr->dataTypes.reserve(typeCount);
    ptr->dataTypes.size = typeCount;
    ctx->loader.gridParamsInterface.enumerateParamTypes(ptr->gridParams, ptr->typenames.data, ptr->displayTypenames.data, ptr->dataTypes.data, &typeCount);

    // register types
    ptr->types.size = 0u;
    for (NvFlowUint64 typeIdx = 0u; typeIdx < ptr->dataTypes.size; typeIdx++)
    {
        ptr->types.pushBack(ptr->gridParamsSet.createType(ptr->dataTypes[typeIdx], ptr->displayTypenames[typeIdx]));
    }

    const EditorFlowStage** builtinStages = nullptr;
    NvFlowUint64 builtinStageCount = 0u;
    editorFlowStage_getBuiltinStages(&builtinStages, &builtinStageCount);
    for (NvFlowUint idx = 0u; idx < builtinStageCount; idx++)
    {
        ptr->stages.pushBack(builtinStages[idx]);
    }

    // command line stage selection
    if (ptr->cmdStage)
    {
        for (NvFlowUint64 idx = 0u; idx < ptr->stages.size; idx++)
        {
            if (strcmp(ptr->stages[idx]->stageName, ptr->cmdStage) == 0)
            {
                ptr->targetStageIdx = idx;
            }
        }
    }
    if (ptr->stages.size > 0u)
    {
        ptr->targetStageIdx = ptr->targetStageIdx % ptr->stages.size;
        const EditorFlowStage* targetStage = ptr->stages[ptr->targetStageIdx];

        ptr->currentStage = targetStage;
        ptr->stageUserdata = ptr->currentStage->init(ptr);
        editorFlowStage_applyOverrides(ptr, ptr->cellsizeOverride, ptr->smallBlocksOverride);
    }
}

void editorFlow_presimulate(EditorCompute* ctx, EditorFlow* ptr, float deltaTime, NvFlowBool32 isPaused)
{
    NvFlowGridParamsDesc nullGridParamsDesc = {};
    ptr->gridParamsDesc = nullGridParamsDesc;

    ptr->absoluteSimTime += deltaTime;

    float simDeltaTime = isPaused ? 0.f : deltaTime;
    ptr->animationTime += simDeltaTime;

    NvFlowBool32 globalForceClear = NV_FLOW_FALSE;

    if (ptr->stages.size > 0u)
    {
        ptr->targetStageIdx = ptr->targetStageIdx % ptr->stages.size;
        const EditorFlowStage* targetStage = ptr->stages[ptr->targetStageIdx];

        if (ptr->currentStage != targetStage)
        {
            if (ptr->currentStage)
            {
                if (ptr->currentStage->destroy)
                {
                    ptr->currentStage->destroy(ptr, ptr->stageUserdata);
                    ptr->stageUserdata = nullptr;
                }
            }
            editorFlow_clearStage(ptr);
            globalForceClear = NV_FLOW_TRUE;

            ptr->currentStage = targetStage;
            ptr->stageUserdata = ptr->currentStage->init(ptr);
            editorFlowStage_applyOverrides(ptr, ptr->cellsizeOverride, ptr->smallBlocksOverride);
        }
    }
    if (ptr->currentStage)
    {
        if (ptr->currentStage->update)
        {
            ptr->currentStage->update(ptr, ptr->stageUserdata, ptr->animationTime, simDeltaTime);
        }
    }

    //auto testParams = ptr->loader.gridParamsInterface.createAbstractParams(ptr->gridParams, 0u, "test");

    NvFlowUint64 stagingVersion = 0llu;
    NvFlowUint64 minActiveVersion = 0llu;
    ctx->loader.gridParamsInterface.getVersion(ptr->gridParams, &stagingVersion, &minActiveVersion);

    // match command state to commands size and clear
    ptr->commandStates.reserve(ptr->commands.size);
    ptr->commandStates.size = ptr->commands.size;
    for (NvFlowUint64 idx = 0u; idx < ptr->commandStates.size; idx++)
    {
        ptr->commandStates[idx] = 0u;
    }

    // process commands that work outside database update
    for (NvFlowUint64 idx = 0u; idx < ptr->commands.size; idx++)
    {
        EditorFlowCommand* cmd = &ptr->commands[idx];
        NvFlowUint* cmdState = &ptr->commandStates[idx];
        if (strcmp(cmd->cmd, "clearStage") == 0)
        {
            ptr->gridParamsSet.markAllInstancesForDestroy<&iface>((NvFlowDatabaseContext*)ptr);
            // mark this command and all previous commands complete
            // previous setAttribute() commands should no-op
            for (NvFlowUint64 idxLocal = 0u; idxLocal < idx; idxLocal++)
            {
                ptr->commandStates[idxLocal] = 1u;
            }
            (*cmdState) = 1u; // mark completed
        }
        if (strcmp(cmd->cmd, "definePrim") == 0)
        {
            NvFlowUint64 typenameIdx = 0u;
            for (; typenameIdx < ptr->typenames.size; typenameIdx++)
            {
                if (NvFlowReflectStringCompare(ptr->displayTypenames[typenameIdx], cmd->type) == 0 ||
                    NvFlowReflectStringCompare(ptr->typenames[typenameIdx], cmd->type) == 0)
                {
                    break;
                }
            }
            if (typenameIdx < ptr->typenames.size)
            {
                ptr->gridParamsSet.createInstance<&iface>(nullptr, stagingVersion, ptr->types[typenameIdx], cmd->path, cmd->name);
                (*cmdState) = 1u; // mark completed
            }
            else
            {
                editorCompute_logPrint(eNvFlowLogLevel_warning, "definePrim(%s, %s) failed, type not recognized", cmd->type, cmd->path);
            }
        }
    }

    ptr->gridParamsSet.update<&iface>((NvFlowDatabaseContext*)ptr, stagingVersion, minActiveVersion);

    for (NvFlowUint64 idx = 0u; idx < ptr->commands.size; idx++)
    {
        EditorFlowCommand* cmd = &ptr->commands[idx];
        NvFlowUint state = ptr->commandStates[idx];
        if (state == 0u)
        {
            editorCompute_logPrint(eNvFlowLogLevel_warning, "cmd %s path(%s) name(%s) type(%s) failed", cmd->cmd, cmd->path, cmd->name, cmd->type);
        }
    }

    // reset command queue
    ptr->commands.size = 0u;
    NvFlowStringPoolReset(ptr->commandStringPool);

    NvFlowGridParamsDescSnapshot snapshot = {};
    ptr->gridParamsSet.getSnapshot(&snapshot.snapshot, stagingVersion);
    snapshot.absoluteSimTime = ptr->absoluteSimTime;
    snapshot.deltaTime = simDeltaTime;
    snapshot.globalForceClear = globalForceClear;

    ctx->loader.gridParamsInterface.commitParams(ptr->gridParams, &snapshot);

    ptr->clientGridParams = ctx->loader.gridParamsInterface.mapGridParamsNamed(ptr->gridParamsClient);

    ptr->paramsSnapshot = ctx->loader.gridParamsInterface.getParamsSnapshot(ptr->clientGridParams, ptr->absoluteSimTime, 0llu);
    if (!ctx->loader.gridParamsInterface.mapParamsDesc(ptr->clientGridParams, ptr->paramsSnapshot, &ptr->gridParamsDesc))
    {
        printf("GridParams map failed!!!!!!!!!\n");
    }
}

void editorFlow_simulate(EditorCompute* ctx, EditorFlow* ptr, float deltaTime, NvFlowBool32 isPaused)
{
    NvFlowContext* context = ctx->loader.deviceInterface.getContext(ctx->deviceQueue);

    {
        if (ptr->maxLocations != ptr->targetMaxLocations)
        {
            ptr->maxLocations = ptr->targetMaxLocations;

            NvFlowGridDesc gridDesc = NvFlowGridDesc_default;
            gridDesc.maxLocations = ptr->maxLocations;
            ctx->loader.gridInterface.resetGrid(
                context,
                ptr->grid,
                &gridDesc
            );
        }

        ctx->loader.gridInterface.simulate(
            context,
            ptr->grid,
            &ptr->gridParamsDesc,
            NV_FLOW_FALSE
        );

        NvFlowDatabaseSnapshot databaseSnapshot = {};
        if (ptr->gridParamsDesc.snapshotCount > 0u)
        {
            databaseSnapshot = ptr->gridParamsDesc.snapshots[ptr->gridParamsDesc.snapshotCount - 1u].snapshot;
        }
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&databaseSnapshot, NvFlowGridSimulateLayerParams)

        ptr->activeBlockCount = ctx->loader.gridInterface.getActiveBlockCount(ptr->grid);
        ctx->benchmarkActiveBlockCount = ptr->activeBlockCount;
        ptr->activeBlockDim = { 32u, 16u, 16u };
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < NvFlowGridSimulateLayerParams_elementCount; layerParamIdx++)
        {
            if (NvFlowGridSimulateLayerParams_elements[layerParamIdx]->enableSmallBlocks)
            {
                ptr->activeBlockDim = { 16u, 8u, 8u };
            }
        }

        ctx->loader.gridInterface.updateIsosurface(
            context,
            ptr->grid,
            &ptr->gridParamsDesc
        );

        ptr->activeBlockCountIsosurface = ctx->loader.gridInterface.getActiveBlockCountIsosurface(ptr->grid);
    }

    // test grid export
    {
        NvFlowGridRenderData renderData = {};

        ctx->loader.gridInterface.getRenderData(context, ptr->grid, &renderData);

        static int writeFrame = 0;
        writeFrame++;
        if (writeFrame % 1000 == 999)
        {
            if (renderData.nanoVdb.readbackCount > 0u)
            {
                NvFlowUint64 lastGlobalFrameCompleted = ctx->contextInterface.getLastGlobalFrameCompleted(context);
                NvFlowSparseNanoVdbExportReadback* readback = &renderData.nanoVdb.readbacks[0u];
                if (readback->globalFrameCompleted <= lastGlobalFrameCompleted && readback->smokeNanoVdbReadback)
                {
                    const char* path = "../../../data/capture0.nvdb.raw";
                    FILE* file = nullptr;
                    fopen_s(&file, path, "wb");
                    if (file)
                    {
                        printf("Writing out capture0.nvdb.raw...\n");

                        fwrite(readback->smokeNanoVdbReadback, 1u, readback->smokeNanoVdbReadbackSize, file);

                        fclose(file);
                    }
                }
            }
        }
    }
}

void editorFlow_offscreen(EditorCompute* ctx, EditorFlow* ptr)
{
    NvFlowContext* context = ctx->loader.deviceInterface.getContext(ctx->deviceQueue);

    ctx->loader.gridInterface.offscreen(
        context,
        ptr->grid,
        &ptr->gridParamsDesc
    );
}

void editorFlow_render(
    EditorCompute* ctx,
    EditorFlow* ptr,
    NvFlowTextureTransient** colorFrontTransient,
    NvFlowTextureTransient* offscreenDepthTransient,
    NvFlowUint windowWidth,
    NvFlowUint windowHeight,
    const NvFlowFloat4x4* view,
    const NvFlowFloat4x4* projection
)
{
    NvFlowContext* context = ctx->loader.deviceInterface.getContext(ctx->deviceQueue);

    ctx->loader.gridInterface.render(
        context,
        ptr->grid,
        &ptr->gridParamsDesc,
        view,
        projection,
        projection,
        windowWidth,
        windowHeight,
        windowWidth,
        windowHeight,
        1.f,
        offscreenDepthTransient,
        eNvFlowFormat_r16g16b16a16_float,
        *colorFrontTransient,
        colorFrontTransient
    );

    ctx->loader.gridInterface.renderIsosurface(
        context,
        ptr->grid,
        &ptr->gridParamsDesc,
        view,
        projection,
        projection,
        windowWidth,
        windowHeight,
        windowWidth,
        windowHeight,
        1.f,
        offscreenDepthTransient,
        eNvFlowFormat_r16g16b16a16_float,
        *colorFrontTransient,
        colorFrontTransient
    );
}

void editorFlow_unmap(EditorCompute* ctx, EditorFlow* ptr)
{
    ctx->loader.gridParamsInterface.unmapParamsDesc(ptr->clientGridParams, ptr->paramsSnapshot);

    // invalidate mapped gridParamsDesc
    NvFlowGridParamsDesc nullGridParamsDesc = {};
    ptr->gridParamsDesc = nullGridParamsDesc;
}

void editorFlow_destroy(EditorCompute* ctx, EditorFlow* ptr)
{
    NvFlowContext* context = ctx->loader.deviceInterface.getContext(ctx->deviceQueue);

    if (ptr->currentStage)
    {
        if (ptr->currentStage->destroy)
        {
            ptr->currentStage->destroy(ptr, ptr->stageUserdata);
            ptr->stageUserdata = nullptr;
        }
    }

    ctx->loader.gridInterface.destroyGrid(context, ptr->grid);
    ctx->loader.gridParamsInterface.destroyGridParamsNamed(ptr->gridParamsServer);
    ctx->loader.gridParamsInterface.destroyGridParamsNamed(ptr->gridParamsClient);

    ptr->gridParamsSet.destroy<&iface>(nullptr);

    NvFlowStringPoolDestroy(ptr->commandStringPool);

    editorCompute_logPrint(eNvFlowLogLevel_info, "Destroyed Grid");
}

void editorFlow_clearStage(EditorFlow* ptr)
{
    EditorFlowCommand command = {};
    command.cmd = "clearStage";

    ptr->commands.pushBack(command);
}

void editorFlow_definePrim(EditorFlow* ptr, const char* type, const char* path, const char* name)
{
    EditorFlowCommand command = {};
    command.cmd = "definePrim";
    command.path = NvFlowStringDup(ptr->commandStringPool, path);
    command.name = NvFlowStringDup(ptr->commandStringPool, name);
    command.type = NvFlowStringDup(ptr->commandStringPool, type);

    ptr->commands.pushBack(command);
}

void editorFlow_setAttribute(EditorFlow* ptr, const char* primPath, const char* name, const void* data, NvFlowUint64 sizeInBytes)
{
    char* commandData = NvFlowStringPoolAllocate(ptr->commandStringPool, sizeInBytes);
    memcpy(commandData, data, sizeInBytes);

    EditorFlowCommand command = {};
    command.cmd = "setAttribute";
    command.path = NvFlowStringDup(ptr->commandStringPool, primPath);
    command.name = NvFlowStringDup(ptr->commandStringPool, name);
    command.data = (NvFlowUint8*)commandData;
    command.dataSize = sizeInBytes;

    ptr->commands.pushBack(command);
}

void editorFlow_setAttributeFloat(EditorFlow* ptr, const char* primPath, const char* name, float value)
{
    editorFlow_setAttribute(ptr, primPath, name, &value, sizeof(float));
}

void editorFlow_setAttributeInt(EditorFlow* ptr, const char* primPath, const char* name, int value)
{
    editorFlow_setAttribute(ptr, primPath, name, &value, sizeof(int));
}

void editorFlow_setAttributeUint(EditorFlow* ptr, const char* primPath, const char* name, NvFlowUint value)
{
    editorFlow_setAttribute(ptr, primPath, name, &value, sizeof(unsigned int));
}

void editorFlow_setAttributeBool(EditorFlow* ptr, const char* primPath, const char* name, NvFlowBool32 value)
{
    editorFlow_setAttribute(ptr, primPath, name, &value, sizeof(NvFlowBool32));
}

void editorFlow_setAttributeFloat3(EditorFlow* ptr, const char* primPath, const char* name, NvFlowFloat3 value)
{
    editorFlow_setAttribute(ptr, primPath, name, &value, sizeof(NvFlowFloat3));
}

void editorFlow_setAttributeFloat3Array(EditorFlow* ptr, const char* primPath, const char* name, const NvFlowFloat3* values, NvFlowUint64 elementCount)
{
    editorFlow_setAttribute(ptr, primPath, name, values, elementCount * sizeof(NvFlowFloat3));
}

void editorFlow_setAttributeFloat4Array(EditorFlow* ptr, const char* primPath, const char* name, const NvFlowFloat4* values, NvFlowUint64 elementCount)
{
    editorFlow_setAttribute(ptr, primPath, name, values, elementCount * sizeof(NvFlowFloat4));
}

void editorFlow_setAttributeIntArray(EditorFlow* ptr, const char* primPath, const char* name, const int* values, NvFlowUint64 elementCount)
{
    editorFlow_setAttribute(ptr, primPath, name, values, elementCount * sizeof(int));
}

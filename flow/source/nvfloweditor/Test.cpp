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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include <math.h>

#include "imgui.h"
#include "Loader.h"
#include "ImguiRenderer.h"
#include "ShapeRenderer.h"
#include "FrameCapture.h"

#include "NvFlowArray.h"

#include "NvFlowUploadBuffer.h"

#include "Camera.h"
#include "Timer.h"

#include "NvFlowDatabase.h"

#include "NvFlowLoader.h"

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
    NvFlowRingBufferPointer<NvFlowDatabaseValue*> values;
};

NV_FLOW_INLINE NvFlowDatabaseValue* copyArray(
    NvFlowUint64 version,
    NvFlowUint64 minActiveVersion,
    NvFlowDatabaseAttr* attr,
    const NvFlowReflectData* reflectData,
    NvFlowUint8* mappedData)
{
    auto value = attr->values.allocateBackPointer();
    //printf("Creating %s %p!!! version(%llu) minActiveVersion(%llu) active(%llu) free(%llu)\n",
    //    reflectData->name, value, version, minActiveVersion,
    //    attr->values.activeCount(), attr->values.freeCount());

    value->version = version;
    value->lastUsed = version;
    value->data.size = 0u;

    NvFlowUint8** pData = (NvFlowUint8**)(mappedData + reflectData->dataOffset);
    NvFlowUint64* pArraySize = (NvFlowUint64*)(mappedData + reflectData->arraySizeOffset);

    const NvFlowUint8* srcData = (*pData);

    NvFlowUint64 srcSizeInBytes = reflectData->dataType->elementSize * (*pArraySize);

    value->data.reserve(srcSizeInBytes);
    value->data.size = srcSizeInBytes;
    if (srcData)
    {
        //printf("Memcpy %p to %p of %llu bytes\n", srcData, value->data.data, srcSizeInBytes);
        memcpy(value->data.data, srcData, srcSizeInBytes);
    }
    else
    {
        memset(value->data.data, 0, srcSizeInBytes);
    }

    // override to owned copy
    *pData = value->data.data;

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

    if (reflectData->reflectMode & eNvFlowReflectMode_array)
    {
        copyArray(version, version, attr, reflectData, mappedData);
    }

    if (reflectData->dataType->dataType == eNvFlowType_float &&
        strcmp(reflectData->name, "colorScale") == 0)
    {
        float* pColorScale = (float*)(mappedData + reflectData->dataOffset);
        *pColorScale = 1.f;
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

    if (reflectData->reflectMode & eNvFlowReflectMode_array)
    {
        while (attr->values.activeCount() > 1u && attr->values.front()->lastUsed < minActiveVersion)
        {
            //printf("Popping %s version %llu lastUsed %llu\n", reflectData->name, attr->values.front()->version, attr->values.front()->lastUsed);
            attr->values.popFront();
        }

#if 0
        for (NvFlowUint64 idx = 0u; idx < attr->values.activeCount() + attr->values.freeCount(); idx++)
        {
            auto ptr = attr->values[idx - attr->values.freeCount()];
            if (idx < attr->values.freeCount())
            {
                printf("free element [%llu] version %llu lastUsed %llu\n", idx, ptr->version, ptr->lastUsed);
            }
            else
            {
                printf("element [%llu] version %llu lastUsed %llu\n", idx, ptr->version, ptr->lastUsed);
            }
        }
#endif

        // Copy each frame to test recycling
        if (attr->values.activeCount() > 0u && attr->values.back()->version != version)
        {
            copyArray(version, minActiveVersion, attr, reflectData, mappedData);
        }

        if (attr->values.activeCount() > 0u)
        {
            attr->values.back()->lastUsed = version;
        }

        while (attr->values.activeCount() > 0u && attr->values.front()->lastUsed < minActiveVersion)
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

void testDatabase()
{
    NvFlowDatabase versioned = {};

    auto type = versioned.createType(&NvFlowGridSimulateLayerParams_NvFlowReflectDataType, "FlowSimulate");

    NvFlowUint64 version = 64u;

    auto v0 = versioned.createInstance<&iface>(nullptr, version, type, "root/", "test0");
    auto v1 = versioned.createInstance<&iface>(nullptr, version, type, "root/", "test1");
    auto v2 = versioned.createInstance<&iface>(nullptr, version, type, "root/", "test2");

    for (NvFlowUint64 updateIdx = 0u; updateIdx < 4096u; updateIdx++)
    {
        versioned.update<&iface>(nullptr, version, version - 6u);

        if (updateIdx == 64u)
        {
            versioned.markInstanceForDestroy<&iface>(nullptr, v2);
        }

        NvFlowDatabaseSnapshot snapshot = {};
        versioned.getSnapshot(&snapshot, version);

        auto ptr = (NvFlowGridSimulateLayerParams*)snapshot.typeSnapshots[0].instanceDatas[0];

        version++;
    }

    versioned.update<&iface>(nullptr, version, ~0llu);

    versioned.destroy<&iface>(nullptr);
}

void testArray()
{
    NvFlowArrayPointer<int*> testArray;
    testArray.allocateBackPointer();
    testArray.allocateBackPointer();
    testArray.allocateBackPointer();
    testArray.size = 0u;
    testArray.reserve(64u);
    testArray.allocateBackPointer();
    testArray.allocateBackPointer();
    testArray.allocateBackPointer();
    testArray.allocateBackPointer();
    testArray.allocateBackPointer();
    testArray.allocateBackPointer();
}

static void flowLoaderError(const char* str, void* userdata)
{
    printf("omni.flow.usd failed to load Flow library!!!\n%s\n", str);
}

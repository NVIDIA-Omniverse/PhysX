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

#include "NvFlow.h"

#include "NvFlowArray.h"
#include "NvFlowString.h"
#include "NvFlowPreprocessor.h"
#include <string.h>
#if defined(_WIN32)
#include <malloc.h>
#else
#include <alloca.h>
#endif

struct NvFlowOpGraph;
typedef struct NvFlowOpGraph NvFlowOpGraph;

struct NvFlowOpRuntime;
typedef struct NvFlowOpRuntime NvFlowOpRuntime;

struct NvFlowOpRuntimeOp;
typedef struct NvFlowOpRuntimeOp NvFlowOpRuntimeOp;

typedef struct NvFlowOpGraphDesc
{
    const char* opTypename;
    const NvFlowReflectDataType* nativePinsIn;
    const NvFlowReflectDataType* nativePinsOut;
    NvFlowLogPrint_t logPrint;
}NvFlowOpGraphDesc;

typedef struct NvFlowOpGraphInterface
{
    NV_FLOW_REFLECT_INTERFACE();

    NvFlowOpGraph*(NV_FLOW_ABI* create)(const NvFlowOpGraphDesc* desc);

    void(NV_FLOW_ABI* destroy)(NvFlowOpGraph* graph);

    void(NV_FLOW_ABI* addOpInterfaces)(NvFlowOpGraph* graph, NvFlowOpInterface** opInterfaces, NvFlowUint opInterfaceCount);

    void(NV_FLOW_ABI* addPin)(NvFlowOpGraph* graph, NvFlowPinDir pinDir, NvFlowReflectHintFlags reflectHints, NvFlowReflectModeFlags reflectMode, const char* dataTypename, const char* name);

    void(NV_FLOW_ABI* addOp)(NvFlowOpGraph* graph, const char* opTypename, const char* name);

    void(NV_FLOW_ABI* addConnection)(NvFlowOpGraph* graph, const char* pinDst, const char* pinSrc);

    NvFlowBool32(NV_FLOW_ABI* build)(NvFlowOpGraph* graph);

    NvFlowBool32(NV_FLOW_ABI* getOpInterface)(NvFlowOpGraph* graph, NvFlowOpInterface** pOpInterface);
}NvFlowOpGraphInterface;

#define NV_FLOW_REFLECT_TYPE NvFlowOpGraphInterface
NV_FLOW_REFLECT_BEGIN()
NV_FLOW_REFLECT_FUNCTION_POINTER(create, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(destroy, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(addOpInterfaces, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(addPin, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(addOp, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(addConnection, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(build, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(getOpInterface, 0, 0)
NV_FLOW_REFLECT_END(0)
NV_FLOW_REFLECT_INTERFACE_IMPL()
#undef NV_FLOW_REFLECT_TYPE

typedef struct NvFlowOpRuntimeDesc
{
    NvFlowOpGraphInterface* opGraphInterface;
    NvFlowLogPrint_t logPrint;
    const NvFlowReflectDataType* nativePinsIn;
    const NvFlowReflectDataType* nativePinsOut;
}NvFlowOpRuntimeDesc;

typedef struct NvFlowOpRuntimeInterface
{
    NV_FLOW_REFLECT_INTERFACE();

    NvFlowOpRuntime*(NV_FLOW_ABI* create)(const NvFlowOpRuntimeDesc* desc);

    void(NV_FLOW_ABI* destroy)(NvFlowOpRuntime* runtime);

    NvFlowOpRuntimeOp*(NV_FLOW_ABI* addOp)(NvFlowOpRuntime* runtime, NvFlowOpInterface* opInterface, const char* instanceName);

    void(NV_FLOW_ABI* removeOp)(NvFlowOpRuntime* runtime, NvFlowOpRuntimeOp* op);

    void*(NV_FLOW_ABI* mapPinData)(NvFlowOpRuntime* runtime);

    void(NV_FLOW_ABI* unmapPinData)(NvFlowOpRuntime* runtime);

    const NvFlowReflectDataType* (NV_FLOW_ABI* getPinDataType)(NvFlowOpRuntime* runtime, NvFlowPinDir dir);

    void(NV_FLOW_ABI* execute)(NvFlowOpRuntime* runtime);

    void(NV_FLOW_ABI* enumerateExecuteGroups)(NvFlowOpRuntime* runtime, NvFlowOpExecuteGroupDesc* pExecuteGroups, NvFlowUint64* pExecuteGroupCount);

    void(NV_FLOW_ABI* executeGroup)(NvFlowOpRuntime* runtime, NvFlowOpExecuteGroup* group);

    NvFlowBool32(NV_FLOW_ABI* setPin)(NvFlowOpRuntime* runtime, NvFlowPinDir dir, const char* name, const void* srcData, NvFlowUint64 srcSizeInBytes);

    NvFlowBool32(NV_FLOW_ABI* getPin)(NvFlowOpRuntime* runtime, NvFlowPinDir dir, const char* name, void* dstData, NvFlowUint64 dstSizeInBytes);

    NvFlowBool32(NV_FLOW_ABI* setPinPointer)(NvFlowOpRuntime* runtime, NvFlowPinDir dir, const char* name, const void* srcPointer);

    NvFlowBool32(NV_FLOW_ABI* getPinPointer)(NvFlowOpRuntime* runtime, NvFlowPinDir dir, const char* name, void** pDstPointer);
}NvFlowOpRuntimeInterface;

#define NV_FLOW_REFLECT_TYPE NvFlowOpRuntimeInterface
NV_FLOW_REFLECT_BEGIN()
NV_FLOW_REFLECT_FUNCTION_POINTER(create, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(destroy, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(addOp, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(removeOp, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(mapPinData, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(unmapPinData, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(getPinDataType, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(execute, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(enumerateExecuteGroups, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(executeGroup, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(setPin, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(getPin, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(setPinPointer, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(getPinPointer, 0, 0)
NV_FLOW_REFLECT_END(0)
NV_FLOW_REFLECT_INTERFACE_IMPL()
#undef NV_FLOW_REFLECT_TYPE

#if 0

namespace
{
    struct OpReflectContext
    {
        const char* structTypename = nullptr;
        NvFlowArray<const NvFlowReflectDataType*> srcReflectDataTypes;
        NvFlowArray<NvFlowUint64> srcOffsets;

        NvFlowUint64 totalSize = 0llu;
        NvFlowArray<NvFlowReflectData> reflectData;
        NvFlowArray<NvFlowUint> childReflectContextIdx;
        NvFlowArrayPointer<OpReflectContext*> reflectContexts;

        NvFlowArray<NvFlowReflectDataType> reflectDataType;

        NvFlowArray<NvFlowArray<char>> stringPool;
        NvFlowArray<const char*> tmp_stack;
    };

    NV_FLOW_CAST_PAIR_NAMED(OpReflectContext, NvFlowReflectContext, OpReflectContext)

    void OpReflectContext_processGeneric(NvFlowReflectData* reflectDatas, NvFlowUint reflectCount, void* userdata)
    {
        OpReflectContext* ptr = (OpReflectContext*)userdata;

        for (NvFlowUint reflectDataIdx = 0u; reflectDataIdx < reflectCount; reflectDataIdx++)
        {
            const NvFlowReflectData* reflectData = &reflectDatas[reflectDataIdx];
            if (reflectData->dataType->dataType == eNvFlowType_struct)
            {
                ptr->reflectData.pushBack(*reflectData);
                ptr->childReflectContextIdx.pushBack(ptr->reflectContexts.size);

                OpReflectContext* childPtr = NvFlowArray_allocateBackPointer(ptr->reflectContexts);
                reflectData->dataType->traverse(
                    reflectData->dataType->reflectContext,
                    reflectData->data,
                    OpReflectContext_processGeneric,
                    childPtr
                );
            }
            else
            {
                ptr->reflectData.pushBack(*reflectData);
                ptr->childReflectContextIdx.pushBack(~0u);
            }
        }
    }

    void OpReflectContext_setStructTypename(OpReflectContext* ptr, const char* structTypename)
    {
        ptr->structTypename = structTypename;
    }

    void OpReflectContext_addToRoot(OpReflectContext* ptr, const NvFlowReflectDataType* reflectDataType)
    {
        if (!reflectDataType)
        {
            return;
        }
        // merges struct members at root scope
        if (reflectDataType->dataType == eNvFlowType_struct)
        {
            NvFlowUint64 baseOffset = sizeof(void*) * ((ptr->totalSize + sizeof(void*) - 1u) / sizeof(void*));
            ptr->totalSize = baseOffset + reflectDataType->elementSize;

            // capture needed data for default support
            ptr->srcReflectDataTypes.pushBack(reflectDataType);
            ptr->srcOffsets.pushBack(baseOffset);

            reflectDataType->traverse(reflectDataType->reflectContext, (void*)baseOffset, OpReflectContext_processGeneric, ptr);
        }
    }

    void OpReflectContext_add(OpReflectContext* ptr, NvFlowReflectHintFlags reflectHints, NvFlowReflectModeFlags reflectMode, const NvFlowReflectDataType* reflectDataType, const char* instanceName)
    {
        if (!reflectDataType)
        {
            return;
        }
        NvFlowUint64 baseOffset = sizeof(void*) * ((ptr->totalSize + sizeof(void*) - 1u) / sizeof(void*));
        NvFlowUint64 arrayBaseOffset = 0llu;
        if (reflectMode == eNvFlowReflectMode_default)
        {
            ptr->totalSize = baseOffset + reflectDataType->elementSize;
        }
        else if (reflectMode & eNvFlowReflectMode_array)
        {
            arrayBaseOffset = baseOffset + sizeof(void*);
            ptr->totalSize = baseOffset + sizeof(void*) + sizeof(NvFlowUint64);
        }
        else if (reflectMode == eNvFlowReflectMode_pointer)
        {
            ptr->totalSize = baseOffset + sizeof(void*);
        }

        // capture needed data for default support, only if inlined
        if (reflectMode == eNvFlowReflectMode_default)
        {
            ptr->srcReflectDataTypes.pushBack(reflectDataType);
            ptr->srcOffsets.pushBack(baseOffset);
        }

        NvFlowReflectData reflectData = { };
        reflectData.reflectHints = reflectHints;
        reflectData.reflectMode = reflectMode;
        reflectData.dataType = reflectDataType;
        reflectData.name = instanceName;
        reflectData.data = (void*)baseOffset;
        reflectData.pArraySize = (NvFlowUint64*)arrayBaseOffset;

        OpReflectContext_processGeneric(&reflectData, 1u, ptr);
    }

    const NvFlowReflectDataType* OpReflectContext_getReflectDataType(OpReflectContext* ptr);

    void OpReflectContext_addPath(OpReflectContext* ptr, NvFlowReflectHintFlags reflectHints, NvFlowReflectModeFlags reflectMode, const NvFlowReflectDataType* reflectDataType, const char* path)
    {
        NvFlowArray<char>& stackPool = ptr->stringPool[ptr->stringPool.allocateBack()];
        NvFlowArray<const char*>& stack = ptr->tmp_stack;
        stackPool.size = 0u;
        stack.size = 0u;

        for (NvFlowUint strIdx = 0u; path[strIdx]; strIdx++)
        {
            char c = path[strIdx];
            stackPool.pushBack(c == '.' ? '\0' : c);
        }
        stackPool.pushBack('\0');

        NvFlowUint baseStrIdx = 0u;
        for (NvFlowUint strIdx = 0u; strIdx < stackPool.size; strIdx++)
        {
            if (stackPool[strIdx] == '\0')
            {
                stack.pushBack(&stackPool[baseStrIdx]);
                baseStrIdx = strIdx + 1u;
            }
        }

        NvFlowUint stackIdx = 0u;
        OpReflectContext* current = ptr;
        // creates child context as needed
        if (stack.size > 1u)
        {
            for (; stackIdx < stack.size - 1u; stackIdx++)
            {
                const char* instanceName = stack[stackIdx];
                NvFlowUint reflectIdx = 0u;
                OpReflectContext* childPtr = nullptr;
                for (; reflectIdx < current->reflectData.size; reflectIdx++)
                {
                    if (NvFlowReflectStringCompare(instanceName, current->reflectData[reflectIdx].name) == 0)
                    {
                        NvFlowUint childReflectContextIdx = current->childReflectContextIdx[reflectIdx];
                        if (childReflectContextIdx < current->reflectContexts.size)
                        {
                            childPtr = current->reflectContexts[childReflectContextIdx];
                            break;
                        }
                    }
                }
                // update totalSize of current
                NvFlowUint64 baseOffset = sizeof(void*) * ((current->totalSize + sizeof(void*) - 1u) / sizeof(void*));
                if (reflectMode == eNvFlowReflectMode_default)
                {
                    current->totalSize = baseOffset + reflectDataType->elementSize;
                }
                else if (reflectMode & eNvFlowReflectMode_array)
                {
                    current->totalSize = baseOffset + sizeof(void*) + sizeof(NvFlowUint64);
                }
                else if (reflectMode == eNvFlowReflectMode_pointer)
                {
                    current->totalSize = baseOffset + sizeof(void*);
                }
                // if missing, create new OpReflectContext
                if (reflectIdx == current->reflectData.size)
                {
                    NvFlowUint childReflectContextIdx = current->reflectContexts.size;

                    childPtr = NvFlowArray_allocateBackPointer(current->reflectContexts);
                    NvFlowReflectData reflectData = {};
                    reflectData.reflectHints = eNvFlowReflectHint_pinGroup;
                    reflectData.reflectMode = eNvFlowReflectMode_default;
                    reflectData.dataType = OpReflectContext_getReflectDataType(childPtr);
                    reflectData.name = instanceName;
                    reflectData.data = (void*)baseOffset;
                    reflectData.pArraySize = 0llu;

                    current->reflectData.pushBack(reflectData);
                    current->childReflectContextIdx.pushBack(childReflectContextIdx);
                }
                // switch to new context as current
                current = childPtr;
            }
        }
        // add leaf
        {
            OpReflectContext_add(current, reflectHints, reflectMode, reflectDataType, stack[stackIdx]);
        }
    }

    void OpReflectContext_traverse(NvFlowReflectContext* context, void* data, NvFlowReflectProcess_t processReflect, void* userdata)
    {
        auto ptr = OpReflectContext_cast(context);

        NvFlowUint reflectCount = ptr->reflectData.size;
        NvFlowReflectData* reflectDatas = (NvFlowReflectData*)alloca(reflectCount * sizeof(NvFlowReflectData));
        for (NvFlowUint reflectIdx = 0u; reflectIdx < reflectCount; reflectIdx++)
        {
            reflectDatas[reflectIdx] = ptr->reflectData[reflectIdx];
            reflectDatas[reflectIdx].data = ((NvFlowUint8*)data) + ((NvFlowUint64)reflectDatas[reflectIdx].data);
            reflectDatas[reflectIdx].pArraySize = (NvFlowUint64*)(((NvFlowUint8*)data) + ((NvFlowUint64)reflectDatas[reflectIdx].pArraySize));
        }
        processReflect(reflectDatas, reflectCount, userdata);
    }

    void OpReflectContext_default(NvFlowReflectContext* context, void* data)
    {
        auto ptr = OpReflectContext_cast(context);

        NvFlowReflectClear(data, ptr->totalSize);
        for (NvFlowUint srcIdx = 0u; srcIdx < ptr->srcReflectDataTypes.size; srcIdx++)
        {
            const NvFlowReflectDataType* dataType = ptr->srcReflectDataTypes[srcIdx];
            if (dataType->defaultValue)
            {
                dataType->defaultValue(dataType->reflectContext, ((NvFlowUint8*)data) + ptr->srcOffsets[srcIdx]);
            }
        }
    }

    const NvFlowReflectDataType* OpReflectContext_getReflectDataType(OpReflectContext* ptr)
    {
        if (ptr->reflectDataType.size == 0u)
        {
            ptr->reflectDataType.allocateBack();
        }
        NvFlowReflectDataType* reflectDataType = ptr->reflectDataType.data;
        reflectDataType->dataType = eNvFlowType_struct;
        reflectDataType->elementSize = ptr->totalSize;
        reflectDataType->structTypename = ptr->structTypename;
        reflectDataType->traverse = OpReflectContext_traverse;
        reflectDataType->reflectContext = OpReflectContext_cast(ptr);
        reflectDataType->defaultValue = OpReflectContext_default;
        return ptr->reflectDataType.data;
    }

    struct Link
    {
        NvFlowReflectData reflectData = {};
        const char* path = nullptr;
        NvFlowUint copyCount = 0u;
    };

    struct Segment
    {
        OpReflectContext reflectContext;
        const NvFlowReflectDataType* reflectDataType = nullptr;
        NvFlowArray<Link> links;
    };

    struct SegmentCopy
    {
        NvFlowUint srcSegmentIdx;
        NvFlowUint dstSegmentIdx;
        NvFlowUint64 srcOffset;
        NvFlowUint64 dstOffset;
        NvFlowUint64 numBytes;
    };

    struct Pin
    {
        NvFlowPinDir pinDir;
        NvFlowReflectHintFlags reflectHints;
        NvFlowReflectModeFlags reflectMode;
        const char* dataTypename;
        const char* name;

        // resolved in build()
        const NvFlowReflectDataType* reflectDataType = nullptr;
    };

    struct Op
    {
        const char* opTypename = nullptr;
        const char* name = nullptr;

        // resolved in build()
        NvFlowUint opInterfaceIdx = ~0u;
        NvFlowUint segmentInIdx = ~0u;
        NvFlowUint segmentOutIdx = ~0u;
        NvFlowArray<SegmentCopy> preCopies;
        NvFlowArray<SegmentCopy> postCopies;
        NvFlowUint executeIdx = ~0u;
    };

    struct Connection
    {
        const char* pinSrc = nullptr;
        const char* pinDst = nullptr;
    };

    struct OpGraph
    {
        NvFlowOpGraphDesc desc = {};

        NvFlowStringPool* stringPool = nullptr;
        NvFlowPreprocessor* preprocessor = nullptr;

        NvFlowArray<NvFlowOpInterface> opInterfaces;

        NvFlowArray<const NvFlowReflectDataType*> reflectDataTypes;

        NvFlowArray<Pin> pins;
        NvFlowArray<Connection> connections;
        NvFlowArray<Op> ops;
        NvFlowArray<Segment> segments;
        NvFlowArray<NvFlowUint> executeList;

        NvFlowArray<const char*> tmp_stringStack;

        NvFlowOpInterface opInterface = {};
    };

    NV_FLOW_CAST_PAIR(NvFlowOpGraph, OpGraph)

    NvFlowOpGraph* OpGraph_create(const NvFlowOpGraphDesc* desc)
    {
        auto ptr = new OpGraph();

        ptr->stringPool = NvFlowStringPoolCreate();
        ptr->preprocessor = NvFlowPreprocessorCreate(ptr->stringPool);

        ptr->desc = *desc;
        ptr->desc.opTypename = NvFlowStringDup(ptr->stringPool, desc->opTypename);

        return cast(ptr);
    }

    void OpGraph_destroy(NvFlowOpGraph* graph)
    {
        auto ptr = cast(graph);

        NvFlowPreprocessorDestroy(ptr->preprocessor);
        NvFlowStringPoolDestroy(ptr->stringPool);

        delete ptr;
    }

    void OpGraph_addOpInterfaces(NvFlowOpGraph* graph, NvFlowOpInterface** opInterfaces, NvFlowUint opInterfaceCount)
    {
        auto ptr = cast(graph);
        for (NvFlowUint idx = 0u; idx < opInterfaceCount; idx++)
        {
            NvFlowOpInterface* iface = &ptr->opInterfaces[ptr->opInterfaces.allocateBack()];
            NvFlowOpInterface_duplicate(iface, opInterfaces[idx]);
        }
    }

    void OpGraph_addPin(NvFlowOpGraph* graph, NvFlowPinDir pinDir, NvFlowReflectHintFlags reflectHints, NvFlowReflectModeFlags reflectMode, const char* dataTypename, const char* name)
    {
        auto ptr = cast(graph);
        Pin* pin = &ptr->pins[ptr->pins.allocateBack()];
        pin->pinDir = pinDir;
        pin->reflectHints = reflectHints;
        pin->reflectMode = reflectMode;
        pin->dataTypename = NvFlowStringDup(ptr->stringPool, dataTypename);
        pin->name = NvFlowStringDup(ptr->stringPool, name);
    }

    void OpGraph_addOp(NvFlowOpGraph* graph, const char* opTypename, const char* name)
    {
        auto ptr = cast(graph);
        Op* op = &ptr->ops[ptr->ops.allocateBack()];
        op->opTypename = opTypename;
        op->name = name;
    }

    void OpGraph_addConnection(NvFlowOpGraph* graph, const char* pinDst, const char* pinSrc)
    {
        auto ptr = cast(graph);
        Connection* connection = &ptr->connections[ptr->connections.allocateBack()];
        connection->pinSrc = pinSrc;
        connection->pinDst = pinDst;
    }

    NvFlowBool32 findReflectDataType(OpGraph* ptr, NvFlowType dataType, const char* structTypename, NvFlowUint* pReflectDataTypeIdx)
    {
        NvFlowUint reflectDataTypeIdx = 0u;
        for (; reflectDataTypeIdx < ptr->reflectDataTypes.size; reflectDataTypeIdx++)
        {
            const NvFlowReflectDataType* reflectDataType = ptr->reflectDataTypes[reflectDataTypeIdx];
            if (reflectDataType->dataType == dataType)
            {
                if (reflectDataType->dataType == eNvFlowType_struct)
                {
                    if (NvFlowStringCompare(reflectDataType->structTypename, structTypename) == 0)
                    {
                        // found type, break
                        break;
                    }
                }
                else
                {
                    // builtin type, break
                    break;
                }
            }
        }
        if (reflectDataTypeIdx == ptr->reflectDataTypes.size)
        {
            return NV_FLOW_FALSE;
        }
        if (pReflectDataTypeIdx)
        {
            *pReflectDataTypeIdx = reflectDataTypeIdx;
        }
        return NV_FLOW_TRUE;
    }

    void registerReflectDataType_process(NvFlowReflectData* reflectDatas, NvFlowUint reflectCount, void* userdata);

    void registerReflectDataType(OpGraph* ptr, const NvFlowReflectDataType* reflectDataType)
    {
        if (!reflectDataType)
        {
            return;
        }
        NvFlowUint reflectDataTypeIdx = ~0u;
        NvFlowBool32 didFind = findReflectDataType(ptr, reflectDataType->dataType, reflectDataType->structTypename, &reflectDataTypeIdx);
        // if found, check for consistent size
        if (didFind)
        {
            const NvFlowReflectDataType* reflectDataTypeCmp = ptr->reflectDataTypes[reflectDataTypeIdx];
            if (reflectDataType->elementSize != reflectDataTypeCmp->elementSize)
            {
                ptr->desc.logPrint(eNvFlowLogLevel_error, "ReflectDataType %s elementSize mismatch %d vs %d.",
                    NvFlowTypeToString(reflectDataType->dataType),
                    reflectDataType->elementSize,
                    reflectDataTypeCmp->elementSize
                );
            }
        }
        else
        {
            ptr->reflectDataTypes.pushBack(reflectDataType);
            if (reflectDataType->dataType == eNvFlowType_struct)
            {
                reflectDataType->traverse(
                    reflectDataType->reflectContext,
                    nullptr,
                    registerReflectDataType_process,
                    ptr
                );
            }
        }
    }

    void registerReflectDataType_process(NvFlowReflectData* reflectDatas, NvFlowUint reflectCount, void* userdata)
    {
        OpGraph* ptr = (OpGraph*)userdata;
        for (NvFlowUint reflectDataIdx = 0u; reflectDataIdx < reflectCount; reflectDataIdx++)
        {
            const NvFlowReflectData* reflectData = &reflectDatas[reflectDataIdx];
            registerReflectDataType(ptr, reflectData->dataType);
            if (reflectData->dataType->dataType == eNvFlowType_struct)
            {
                reflectData->dataType->traverse(
                    reflectData->dataType->reflectContext,
                    reflectData->data,
                    registerReflectDataType_process,
                    userdata
                );
            }
        }
    }

    struct UpdateLinks_Userdata
    {
        OpGraph* opGraph;
        NvFlowArray<Link>& links;
        NvFlowUint linkCount;
    };

    void updateLinks_process(NvFlowReflectData* reflectDatas, NvFlowUint reflectCount, void* userdata)
    {
        UpdateLinks_Userdata* ptr = (UpdateLinks_Userdata*)userdata;
        auto& stringStack = ptr->opGraph->tmp_stringStack;
        for (NvFlowUint reflectDataIdx = 0u; reflectDataIdx < reflectCount; reflectDataIdx++)
        {
            const NvFlowReflectData* reflectData = &reflectDatas[reflectDataIdx];
            if (stringStack.size > 0u)
            {
                stringStack.pushBack(".");
            }
            stringStack.pushBack(reflectData->name);
            if (reflectData->reflectFlags & eNvFlowReflectHint_pinEnabled)
            {
                if (ptr->linkCount >= ptr->links.size)
                {
                    Link* link = &ptr->links[ptr->links.allocateBack()];
                    link->reflectData = *reflectData;
                    link->path = NvFlowStringConcatN(ptr->opGraph->stringPool, stringStack.data, stringStack.size);
                    link->copyCount = 0u;
                }
                ptr->linkCount++;
            }
            if (reflectData->reflectFlags & eNvFlowReflectHint_pinGroup &&
                reflectData->dataType->dataType == eNvFlowType_struct)
            {
                reflectData->dataType->traverse(
                    reflectData->dataType->reflectContext,
                    reflectData->data,
                    updateLinks_process,
                    userdata
                );
            }
            stringStack.size--;
            if (stringStack.size > 0u)
            {
                stringStack.size--;
            }
        }
    }

    void updateLinks(OpGraph* ptr, NvFlowArray<Link>& links, const NvFlowReflectDataType* reflectDataType)
    {
        if (!reflectDataType)
        {
            return;
        }
        ptr->tmp_stringStack.size = 0u;
        UpdateLinks_Userdata userdata = { ptr, links, 0u };
        reflectDataType->traverse(reflectDataType->reflectContext, nullptr, updateLinks_process, &userdata);
        ptr->tmp_stringStack.size = 0u;
    }

    void pushCopy(OpGraph* ptr, NvFlowUint opIdx, NvFlowBool32 isPull, NvFlowUint dstSegmentIdx, Link* dstLink, NvFlowUint srcSegmentIdx, Link* srcLink)
    {
        Op* op = &ptr->ops[opIdx];
        SegmentCopy copy = {};
        copy.srcSegmentIdx = srcSegmentIdx;
        copy.dstSegmentIdx = dstSegmentIdx;
        copy.srcOffset = (NvFlowUint64)srcLink->reflectData.data;
        copy.dstOffset = (NvFlowUint64)dstLink->reflectData.data;
        if (dstLink->reflectData.reflectMode == eNvFlowReflectMode_default)
        {
            copy.numBytes = dstLink->reflectData.dataType->elementSize;
        }
        else if (dstLink->reflectData.reflectMode & eNvFlowReflectMode_array)
        {
            copy.numBytes = sizeof(void*);
        }
        else if (dstLink->reflectData.reflectMode == eNvFlowReflectMode_pointer)
        {
            copy.numBytes = sizeof(void*);
        }
        if (isPull)
        {
            op->preCopies.pushBack(copy);
        }
        else
        {
            op->postCopies.pushBack(copy);
        }
        // push extra copy if array
        if (dstLink->reflectData.reflectMode & eNvFlowReflectMode_array)
        {
            copy.srcOffset = (NvFlowUint64)srcLink->reflectData.pArraySize;
            copy.dstOffset = (NvFlowUint64)dstLink->reflectData.pArraySize;
            copy.numBytes = sizeof(NvFlowUint64);
            if (isPull)
            {
                op->preCopies.pushBack(copy);
            }
            else
            {
                op->postCopies.pushBack(copy);
            }
        }
    }

    NvFlowBool32 OpGraph_build(NvFlowOpGraph* graph)
    {
        auto ptr = cast(graph);

        // capture native pin layout
        registerReflectDataType(ptr, ptr->desc.nativePinsIn);
        registerReflectDataType(ptr, ptr->desc.nativePinsOut);
        // capture pin layouts for each op type
        for (NvFlowUint opInterfaceIdx = 0u; opInterfaceIdx < ptr->opInterfaces.size; opInterfaceIdx++)
        {
            NvFlowOpInterface* opInterface = &ptr->opInterfaces[opInterfaceIdx];
            registerReflectDataType(ptr, opInterface->pinsIn);
            registerReflectDataType(ptr, opInterface->pinsOut);
        }

        // resolve pin data types
        for (NvFlowUint pinIdx = 0u; pinIdx < ptr->pins.size; pinIdx++)
        {
            Pin* pin = &ptr->pins[pinIdx];

            NvFlowType dataType = NvFlowTypeFromString(pin->dataTypename);
            const char* structTypename = nullptr;
            if (dataType == eNvFlowType_unknown)
            {
                dataType = eNvFlowType_struct;
                structTypename = pin->dataTypename;
            }

            NvFlowUint reflectDataTypeIdx = 0u;
            if (!findReflectDataType(ptr, dataType, structTypename, &reflectDataTypeIdx))
            {
                ptr->desc.logPrint(eNvFlowLogLevel_error, "Failed to resolve reflectDataTypename %s.", pin->dataTypename);
                return NV_FLOW_FALSE;
            }

            pin->reflectDataType = ptr->reflectDataTypes[reflectDataTypeIdx];
        }

        // resolve op instances
        for (NvFlowUint opIdx = 0u; opIdx < ptr->ops.size; opIdx++)
        {
            Op* op = &ptr->ops[opIdx];
            NvFlowUint opInterfaceIdx = 0u;
            for (; opInterfaceIdx < ptr->opInterfaces.size; opInterfaceIdx++)
            {
                NvFlowOpInterface* opInterface = &ptr->opInterfaces[opInterfaceIdx];
                if (NvFlowStringCompare(op->opTypename, opInterface->opTypename) == 0)
                {
                    op->opInterfaceIdx = opInterfaceIdx;
                    break;
                }
            }
            if (opInterfaceIdx == ptr->opInterfaces.size)
            {
                ptr->desc.logPrint(eNvFlowLogLevel_error, "Failed to resolve Op typename.");
                return NV_FLOW_FALSE;
            }
            op->opInterfaceIdx = opInterfaceIdx;
        }

        // segment 0, external input, link src
        {
            NvFlowUint segmentIdx = ptr->segments.allocateBack();
            Segment* segment = &ptr->segments[segmentIdx];

            const char* segmentName = NvFlowStringConcat(ptr->stringPool, ptr->desc.opTypename, "PinsIn");
            OpReflectContext_setStructTypename(&segment->reflectContext, segmentName);

            OpReflectContext_addToRoot(&segment->reflectContext, ptr->desc.nativePinsIn);
            for (NvFlowUint pinIdx = 0u; pinIdx < ptr->pins.size; pinIdx++)
            {
                Pin* pin = &ptr->pins[pinIdx];
                if (pin->pinDir == eNvFlowPinDir_in)
                {
                    OpReflectContext_add(&segment->reflectContext, pin->reflectFlags, pin->reflectMode, pin->reflectDataType, pin->name);
                }
            }
        }
        // segment 1, external output, link dst
        {
            NvFlowUint segmentIdx = ptr->segments.allocateBack();
            Segment* segment = &ptr->segments[segmentIdx];

            const char* segmentName = NvFlowStringConcat(ptr->stringPool, ptr->desc.opTypename, "PinsOut");
            OpReflectContext_setStructTypename(&segment->reflectContext, segmentName);

            OpReflectContext_addToRoot(&segment->reflectContext, ptr->desc.nativePinsOut);
            for (NvFlowUint pinIdx = 0u; pinIdx < ptr->pins.size; pinIdx++)
            {
                Pin* pin = &ptr->pins[pinIdx];
                if (pin->pinDir == eNvFlowPinDir_out)
                {
                    OpReflectContext_add(&segment->reflectContext, pin->reflectFlags, pin->reflectMode, pin->reflectDataType, pin->name);
                }
            }
        }
        // allocate op segments
        for (NvFlowUint opIdx = 0u; opIdx < ptr->ops.size; opIdx++)
        {
            Op* op = &ptr->ops[opIdx];
            NvFlowOpInterface* opIface = &ptr->opInterfaces[op->opInterfaceIdx];
            // output, link src
            {
                op->segmentOutIdx = ptr->segments.allocateBack();
                Segment* segment = &ptr->segments[op->segmentOutIdx];
                OpReflectContext_setStructTypename(
                    &segment->reflectContext,
                    opIface->pinsOut->structTypename
                );
                OpReflectContext_add(
                    &segment->reflectContext,
                    eNvFlowReflectHint_pinGroup,
                    eNvFlowReflectMode_default,
                    opIface->pinsOut,
                    op->name
                );
            }
            // input, link dst
            {
                op->segmentInIdx = ptr->segments.allocateBack();
                Segment* segment = &ptr->segments[op->segmentInIdx];
                OpReflectContext_setStructTypename(
                    &segment->reflectContext,
                    opIface->pinsIn->structTypename
                );
                OpReflectContext_add(
                    &segment->reflectContext,
                    eNvFlowReflectHint_pinGroup,
                    eNvFlowReflectMode_default,
                    opIface->pinsIn,
                    op->name
                );
            }
        }

        // update links for segments
        for (NvFlowUint segmentIdx = 0u; segmentIdx < ptr->segments.size; segmentIdx++)
        {
            Segment* segment = &ptr->segments[segmentIdx];
            updateLinks(ptr, segment->links, OpReflectContext_getReflectDataType(&segment->reflectContext));
        }

        // connections
        for (NvFlowUint connectionIdx = 0u; connectionIdx < ptr->connections.size; connectionIdx++)
        {
            Connection* connection = &ptr->connections[connectionIdx];
            NvFlowUint srcSegmentIdx = ~0u;
            NvFlowUint srcLinkIdx = ~0u;
            // src segments are even indices
            for (NvFlowUint segmentIdx = 0u; segmentIdx < ptr->segments.size; segmentIdx += 2u)
            {
                Segment* segment = &ptr->segments[segmentIdx];
                for (NvFlowUint linkIdx = 0u; linkIdx < segment->links.size; linkIdx++)
                {
                    Link* link = &segment->links[linkIdx];
                    if (NvFlowReflectStringCompare(link->path, connection->pinSrc) == 0)
                    {
                        srcSegmentIdx = segmentIdx;
                        srcLinkIdx = linkIdx;
                        break;
                    }
                }
            }
            NvFlowUint dstSegmentIdx = ~0u;
            NvFlowUint dstLinkIdx = ~0u;
            // dst segments are odd indices
            for (NvFlowUint segmentIdx = 1u; segmentIdx < ptr->segments.size; segmentIdx += 2u)
            {
                Segment* segment = &ptr->segments[segmentIdx];
                for (NvFlowUint linkIdx = 0u; linkIdx < segment->links.size; linkIdx++)
                {
                    Link* link = &segment->links[linkIdx];
                    if (NvFlowReflectStringCompare(link->path, connection->pinDst) == 0)
                    {
                        dstSegmentIdx = segmentIdx;
                        dstLinkIdx = linkIdx;
                        break;
                    }
                }
            }
            if (srcLinkIdx == ~0u)
            {
                ptr->desc.logPrint(eNvFlowLogLevel_error, "Failed to resolve connection src %s.", connection->pinSrc);
                return NV_FLOW_FALSE;
            }
            if (dstLinkIdx == ~0u)
            {
                ptr->desc.logPrint(eNvFlowLogLevel_error, "Failed to resolve connection dst %s.", connection->pinDst);
                return NV_FLOW_FALSE;
            }
            Link* srcLink = &ptr->segments[srcSegmentIdx].links[srcLinkIdx];
            Link* dstLink = &ptr->segments[dstSegmentIdx].links[dstLinkIdx];
            // validate type match
            if (srcLink->reflectData.reflectMode != dstLink->reflectData.reflectMode)
            {
                ptr->desc.logPrint(eNvFlowLogLevel_error, "Connection %s = %s reflectMode mismatch.", connection->pinDst, connection->pinSrc);
                return NV_FLOW_FALSE;
            }
            if (srcLink->reflectData.dataType->dataType != dstLink->reflectData.dataType->dataType)
            {
                ptr->desc.logPrint(eNvFlowLogLevel_error, "Connection %s = %s dataType mismatch.", connection->pinDst, connection->pinSrc);
                return NV_FLOW_FALSE;
            }
            if (srcLink->reflectData.dataType->dataType == eNvFlowType_struct)
            {
                if (NvFlowStringCompare(srcLink->reflectData.dataType->structTypename, dstLink->reflectData.dataType->structTypename) != 0)
                {
                    ptr->desc.logPrint(eNvFlowLogLevel_error, "Connection %s = %s dataType mismatch.", connection->pinDst, connection->pinSrc);
                    return NV_FLOW_FALSE;
                }
            }
            // count connection
            srcLink->copyCount++;
            dstLink->copyCount++;
            // produce copy command
            NvFlowUint srcOpIdx = srcSegmentIdx < 2 ? ~0u : (srcSegmentIdx - 2u) / 2;
            NvFlowUint dstOpIdx = dstSegmentIdx < 2 ? ~0u : (dstSegmentIdx - 2u) / 2;
            // favor pull, fallback to push to external output
            if (dstOpIdx < ptr->ops.size)
            {
                pushCopy(ptr, dstOpIdx, NV_FLOW_TRUE, dstSegmentIdx, dstLink, srcSegmentIdx, srcLink);
            }
            else if (srcOpIdx < ptr->ops.size)
            {
                pushCopy(ptr, srcOpIdx, NV_FLOW_FALSE, dstSegmentIdx, dstLink, srcSegmentIdx, srcLink);
            }
            else
            {
                ptr->desc.logPrint(eNvFlowLogLevel_error, "Connection %s = %s failed. Connections must connect to at least one op.", connection->pinDst, connection->pinSrc);
                return NV_FLOW_FALSE;
            }
        }

        // for unlinked op pins, automatically generate external pins, and connect
        // first pass does input pins, second pass does output pins
        for (NvFlowUint passIdx = 0u; passIdx < 2u; passIdx++)
        {
            for (NvFlowUint opIdx = 0u; opIdx < ptr->ops.size; opIdx++)
            {
                Op* op = &ptr->ops[opIdx];
                // pass 0, op is dst
                // pass 1, op is src
                NvFlowUint opSegmentIdx = passIdx == 0u ? op->segmentInIdx : op->segmentOutIdx;
                NvFlowUint extSegmentIdx = passIdx == 0u ? 0 : 1;
                Segment* opSegment = &ptr->segments[opSegmentIdx];
                for (NvFlowUint opLinkIdx = 0u; opLinkIdx < opSegment->links.size; opLinkIdx++)
                {
                    Link* opLink = &opSegment->links[opLinkIdx];
                    // skip if pin already connected
                    if (opLink->copyCount > 0u)
                    {
                        continue;
                    }
                    // src path depends on global flag
                    const char* targetPath = opLink->path;
                    if (opLink->reflectData.reflectFlags & eNvFlowReflectHint_pinGlobal)
                    {
                        targetPath = opLink->reflectData.name;
                    }
                    // add external input/output pin if missing
                    Segment* extSegment = &ptr->segments[extSegmentIdx];
                    Link* extLink = nullptr;
                    for (NvFlowUint extLinkIdx = 0u; extLinkIdx < extSegment->links.size; extLinkIdx++)
                    {
                        Link* link = &extSegment->links[extLinkIdx];
                        if (NvFlowStringCompare(targetPath, link->path) == 0)
                        {
                            extLink = link;
                            break;
                        }
                    }
                    if (!extLink)
                    {
                        OpReflectContext_addPath(
                            &extSegment->reflectContext,
                            opLink->reflectData.reflectFlags,
                            opLink->reflectData.reflectMode,
                            opLink->reflectData.dataType,
                            targetPath
                        );
                        updateLinks(ptr, extSegment->links, OpReflectContext_getReflectDataType(&extSegment->reflectContext));
                        // for now, assume last link is new
                        extLink = &extSegment->links[extSegment->links.size - 1u];
                    }
                    if (!extLink)
                    {
                        ptr->desc.logPrint(eNvFlowLogLevel_error, "Pin %s auto externalize failed.", opLink->path);
                        return NV_FLOW_FALSE;
                    }
                    // verify link has expected path
                    if (NvFlowStringCompare(extLink->path, targetPath) != 0)
                    {
                        ptr->desc.logPrint(eNvFlowLogLevel_error, "Pin %s auto externalize failed.", opLink->path);
                        return NV_FLOW_FALSE;
                    }
                    // add copy
                    {
                        // count connection
                        extLink->copyCount++;
                        opLink->copyCount++;
                        if (passIdx == 0u)
                        {
                            // pull from new input pin
                            pushCopy(ptr, opIdx, NV_FLOW_TRUE, opSegmentIdx, opLink, extSegmentIdx, extLink);
                        }
                        else
                        {
                            // push to new output pin
                            pushCopy(ptr, opIdx, NV_FLOW_FALSE, extSegmentIdx, extLink, opSegmentIdx, opLink);
                        }
                    }
                }
            }
        }

        // schedule
        while (ptr->executeList.size < ptr->ops.size)
        {
            NvFlowUint oldCount = ptr->executeList.size;
            for (NvFlowUint opIdx = 0u; opIdx < ptr->ops.size; opIdx++)
            {
                Op* op = &ptr->ops[opIdx];
                NvFlowBool32 canExecute = NV_FLOW_TRUE;
                for (NvFlowUint preCopyIdx = 0u; preCopyIdx < op->preCopies.size; preCopyIdx++)
                {
                    NvFlowUint srcSegmentIdx = op->preCopies[preCopyIdx].srcSegmentIdx;
                    // if segment is from op, check if op has executed
                    if (srcSegmentIdx >= 2u)
                    {
                        NvFlowUint srcOpIdx = (srcSegmentIdx - 2u) / 2u;
                        if (ptr->ops[srcOpIdx].executeIdx == ~0u)
                        {
                            canExecute = NV_FLOW_FALSE;
                            break;
                        }
                    }
                }
                if (canExecute && op->executeIdx == ~0u)
                {
                    op->executeIdx = ptr->executeList.size;
                    ptr->executeList.pushBack(opIdx);
                }
            }
            if (oldCount == ptr->executeList.size)
            {
                ptr->desc.logPrint(eNvFlowLogLevel_error, "Scheduling failed. Circular dependency.");
                return NV_FLOW_FALSE;
            }
        }

        // cache segment data types for OpInstance
        for (NvFlowUint segmentIdx = 0u; segmentIdx < ptr->segments.size; segmentIdx++)
        {
            Segment* segment = &ptr->segments[segmentIdx];
            segment->reflectDataType = OpReflectContext_getReflectDataType(&segment->reflectContext);
        }

        return NV_FLOW_TRUE;
    }

    struct OpInstanceSegment
    {
        const NvFlowOpGenericPinsIn* pinsIn = nullptr;
        NvFlowOpGenericPinsOut* pinsOut = nullptr;
        NvFlowArray<unsigned char> ownedData;
    };

    struct OpInstance
    {
        const OpGraph* opGraph = nullptr;
        NvFlowArray<NvFlowOp*> ops;
        NvFlowArray<OpInstanceSegment> segments;
    };

    NV_FLOW_CAST_PAIR(NvFlowOp, OpInstance)

    void op_pass(OpInstance* ptr, const NvFlowOpGenericPinsIn* in, NvFlowOpGenericPinsOut* out, NvFlowBool32 doCreate, NvFlowBool32 doExecute, NvFlowBool32 doDestroy, NvFlowUint executeGroupIdx);

    NvFlowOp* op_create(const NvFlowOpInterface* opInterface, const NvFlowOpGenericPinsIn* in, NvFlowOpGenericPinsOut* out)
    {
        auto ptr = new OpInstance();

        ptr->opGraph = cast(opInterface->opGraph);

        // create op instances and allocate pin data
        ptr->ops.reserve(ptr->opGraph->ops.size);
        ptr->ops.size = ptr->opGraph->ops.size;

        ptr->segments.reserve(ptr->opGraph->segments.size);
        ptr->segments.size = ptr->opGraph->segments.size;
        if (0u < ptr->segments.size)
        {
            ptr->segments[0u].pinsIn = in;
            ptr->segments[0u].pinsOut = nullptr;
        }
        if (1u < ptr->segments.size)
        {
            ptr->segments[1u].pinsIn = nullptr;
            ptr->segments[1u].pinsOut = out;
        }
        for (NvFlowUint segmentIdx = 2u; segmentIdx < ptr->segments.size; segmentIdx++)
        {
            const Segment* graphSegment = &ptr->opGraph->segments[segmentIdx];
            OpInstanceSegment* segment = &ptr->segments[segmentIdx];
            segment->ownedData.reserve(graphSegment->reflectDataType->elementSize);
            segment->ownedData.size = graphSegment->reflectDataType->elementSize;
            segment->pinsIn = (const NvFlowOpGenericPinsIn*)segment->ownedData.data;
            segment->pinsOut = (NvFlowOpGenericPinsOut*)segment->ownedData.data;
        }

        op_pass(ptr, in, out, NV_FLOW_TRUE, NV_FLOW_FALSE, NV_FLOW_FALSE, ~0u);

        return cast(ptr);
    }

    void op_destroy(NvFlowOp* op, const NvFlowOpGenericPinsIn* in, NvFlowOpGenericPinsOut* out)
    {
        auto ptr = cast(op);

        op_pass(ptr, in, out, NV_FLOW_FALSE, NV_FLOW_FALSE, NV_FLOW_TRUE, ~0u);

        delete ptr;
    }

    void op_execute(NvFlowOp* op, const NvFlowOpGenericPinsIn* in, NvFlowOpGenericPinsOut* out)
    {
        auto ptr = cast(op);

        op_pass(ptr, in, out, NV_FLOW_FALSE, NV_FLOW_TRUE, NV_FLOW_FALSE, ~0u);
    }

    void op_executeGroup(NvFlowOp* op, NvFlowOpExecuteGroup* group, const NvFlowOpGenericPinsIn* in, NvFlowOpGenericPinsOut* out)
    {
        auto ptr = cast(op);

        NvFlowUint executeGroupIdx = (NvFlowUint)(((NvFlowUint64)group) - 1u);

        op_pass(ptr, in, out, NV_FLOW_FALSE, NV_FLOW_TRUE, NV_FLOW_FALSE, executeGroupIdx);
    }

    void op_pass(OpInstance* ptr, const NvFlowOpGenericPinsIn* in, NvFlowOpGenericPinsOut* out, NvFlowBool32 doCreate, NvFlowBool32 doExecute, NvFlowBool32 doDestroy, NvFlowUint executeGroupIdx)
    {
        // update external in/out segments
        if (0u < ptr->segments.size)
        {
            ptr->segments[0u].pinsIn = in;
            ptr->segments[0u].pinsOut = nullptr;
        }
        if (1u < ptr->segments.size)
        {
            ptr->segments[1u].pinsIn = nullptr;
            ptr->segments[1u].pinsOut = out;
        }
        // run ops
        for (NvFlowUint listIdx = 0u; listIdx < ptr->opGraph->executeList.size; listIdx++)
        {
            NvFlowUint opIdx = ptr->opGraph->executeList[listIdx];
            const Op* op = &ptr->opGraph->ops[opIdx];
            const NvFlowOpInterface* opIface = &ptr->opGraph->opInterfaces[op->opInterfaceIdx];

            // pre copy
            for (NvFlowUint preCopyIdx = 0u; preCopyIdx < op->preCopies.size; preCopyIdx++)
            {
                const SegmentCopy* preCopy = &op->preCopies[preCopyIdx];
                const OpInstanceSegment* srcSegment = &ptr->segments[preCopy->srcSegmentIdx];
                const OpInstanceSegment* dstSegment = &ptr->segments[preCopy->dstSegmentIdx];
                NvFlowReflectMemcpy(
                    ((NvFlowUint8*)dstSegment->pinsOut) + preCopy->dstOffset,
                    ((const NvFlowUint8*)srcSegment->pinsIn) + preCopy->srcOffset,
                    preCopy->numBytes
                );
            }
            const NvFlowOpGenericPinsIn* pinsIn = ptr->segments[op->segmentInIdx].pinsIn;
            NvFlowOpGenericPinsOut* pinsOut = ptr->segments[op->segmentOutIdx].pinsOut;
            if (doCreate)
            {
                ptr->ops[opIdx] = opIface->create(opIface, pinsIn, pinsOut);
            }
            if (doExecute)
            {
                opIface->execute(ptr->ops[opIdx], pinsIn, pinsOut);
            }
            if (doDestroy)
            {
                opIface->destroy(ptr->ops[opIdx], pinsIn, pinsOut);
                ptr->ops[opIdx] = nullptr;
            }
            // post copy
            for (NvFlowUint postCopyIdx = 0u; postCopyIdx < op->postCopies.size; postCopyIdx++)
            {
                const SegmentCopy* postCopy = &op->postCopies[postCopyIdx];
                const OpInstanceSegment* srcSegment = &ptr->segments[postCopy->srcSegmentIdx];
                const OpInstanceSegment* dstSegment = &ptr->segments[postCopy->dstSegmentIdx];
                NvFlowReflectMemcpy(
                    ((NvFlowUint8*)dstSegment->pinsOut) + postCopy->dstOffset,
                    ((const NvFlowUint8*)srcSegment->pinsIn) + postCopy->srcOffset,
                    postCopy->numBytes
                );
            }
        }
    }

    NvFlowBool32 OpGraph_getOpInterface(NvFlowOpGraph* graph, NvFlowOpInterface** pOpInterface)
    {
        auto ptr = cast(graph);
        static NvFlowOpInterface iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowOpInterface) };
        iface.opTypename = ptr->desc.opTypename;
        iface.opGraph = graph;
        iface.pinsIn = OpReflectContext_getReflectDataType(&ptr->segments[0].reflectContext);
        iface.pinsOut = OpReflectContext_getReflectDataType(&ptr->segments[1].reflectContext);
        iface.executeGroupDescs = nullptr;
        iface.executeGroupCount = 0u;
        iface.create = op_create;
        iface.destroy = op_destroy;
        iface.execute = op_execute;
        iface.executeGroup = op_executeGroup;
        *pOpInterface =  &iface;
        return NV_FLOW_TRUE;
    }
}

NvFlowOpGraphInterface* NvFlowGetOpGraphInterface()
{
    static NvFlowOpGraphInterface iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowOpGraphInterface) };
    iface.create = OpGraph_create;
    iface.destroy = OpGraph_destroy;
    iface.addOpInterfaces = OpGraph_addOpInterfaces;
    iface.addPin = OpGraph_addPin;
    iface.addOp = OpGraph_addOp;
    iface.addConnection = OpGraph_addConnection;
    iface.build = OpGraph_build;
    iface.getOpInterface = OpGraph_getOpInterface;
    return &iface;
}

#endif

NvFlowOpGraphInterface* NvFlowGetOpGraphInterface()
{
    return nullptr;
}

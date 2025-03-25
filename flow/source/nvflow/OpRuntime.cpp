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

#include <string.h>

#if 0

namespace
{
    struct OpRuntimeOp
    {
        NvFlowOpInterface opInterface = {};
        const char* instanceName;
    };

    NV_FLOW_CAST_PAIR(NvFlowOpRuntimeOp, OpRuntimeOp)

    struct OpRuntime
    {
        NvFlowOpRuntimeDesc desc = { };

        NvFlowOpGraphInterface opGraphInterface = {};

        NvFlowBool32 isDirty = NV_FLOW_TRUE;

        NvFlowOpGraph* opGraph = nullptr;
        NvFlowOpInterface groupOpInterface = {};
        NvFlowOp* groupOp = nullptr;

        NvFlowArray<OpRuntimeOp*> runtimeOps;
        NvFlowArray<NvFlowArray<unsigned char>> pinDataPool;
    };

    NV_FLOW_CAST_PAIR(NvFlowOpRuntime, OpRuntime)

    NvFlowOpRuntime* OpRuntime_create(const NvFlowOpRuntimeDesc* desc)
    {
        auto ptr = new OpRuntime();

        ptr->desc = *desc;

        NvFlowOpGraphInterface_duplicate(&ptr->opGraphInterface, ptr->desc.opGraphInterface);

        return cast(ptr);
    }

    void OpRuntime_opGraphDestroy(NvFlowOpRuntime* runtime)
    {
        auto ptr = cast(runtime);

        if (ptr->groupOp)
        {
            ptr->groupOpInterface.destroy(ptr->groupOp, (NvFlowOpGenericPinsIn*)ptr->pinDataPool[0].data, (NvFlowOpGenericPinsOut*)ptr->pinDataPool[1].data);
            ptr->groupOp = nullptr;
        }
        if (ptr->opGraph)
        {
            ptr->opGraphInterface.destroy(ptr->opGraph);
            ptr->opGraph = nullptr;
        }
    }

    void OpRuntime_destroy(NvFlowOpRuntime* runtime)
    {
        auto ptr = cast(runtime);

        OpRuntime_opGraphDestroy(runtime);

        NvFlowArray_deletePointers(ptr->runtimeOps);

        delete ptr;
    }

    NvFlowOpRuntimeOp* OpRuntime_addOp(NvFlowOpRuntime* runtime, NvFlowOpInterface* opInterface, const char* instanceName)
    {
        auto ptr = cast(runtime);

        OpRuntimeOp* opPtr = NvFlowArray_allocateBackPointer(ptr->runtimeOps);

        NvFlowOpInterface_duplicate(&opPtr->opInterface, opInterface);
        opPtr->instanceName = instanceName;

        ptr->isDirty = NV_FLOW_TRUE;

        return cast(opPtr);
    }

    void OpRuntime_removeOp(NvFlowOpRuntime* runtime, NvFlowOpRuntimeOp* op)
    {
        auto ptr = cast(runtime);
        auto opPtr = cast(op);

        NvFlowArray_removeSwapPointer(ptr->runtimeOps, opPtr);

        ptr->isDirty = NV_FLOW_TRUE;
    }

    void setPinDefaults(NvFlowReflectData* pinDatas, NvFlowUint pinCount, void* userdata)
    {
        OpRuntime* ptr = (OpRuntime*)userdata;

        for (NvFlowUint idx = 0; idx < pinCount; idx++)
        {
            if (pinDatas[idx].reflectFlags & eNvFlowReflectHint_pinGroup)
            {
                pinDatas[idx].dataType->traverse(pinDatas[idx].dataType->reflectContext, pinDatas[idx].data, setPinDefaults, userdata);
            }
            else
            {
                if (pinDatas[idx].reflectMode == eNvFlowReflectMode_pointer)
                {
                    NvFlowUint allocIdx = ptr->pinDataPool.allocateBack();
                    ptr->pinDataPool[allocIdx].reserve(pinDatas[idx].dataType->elementSize);
                    ptr->pinDataPool[allocIdx].size = pinDatas[idx].dataType->elementSize;

                    memset(ptr->pinDataPool[allocIdx].data, 0, ptr->pinDataPool[allocIdx].size);
                    if (pinDatas[idx].dataType->defaultValue)
                    {
                        pinDatas[idx].dataType->defaultValue(nullptr, ptr->pinDataPool[allocIdx].data);
                    }

                    *(void**)pinDatas[idx].data = ptr->pinDataPool[allocIdx].data;
                }
                else if (pinDatas[idx].reflectMode & eNvFlowReflectMode_array)
                {
                    *(void**)pinDatas[idx].data = nullptr;
                    *pinDatas[idx].pArraySize = 0llu;
                }
                else
                {
                    if (pinDatas[idx].dataType->defaultValue)
                    {
                        pinDatas[idx].dataType->defaultValue(nullptr, pinDatas[idx].data);
                    }
                }
            }
        }
    }

    void update(NvFlowOpRuntime* runtime)
    {
        auto ptr = cast(runtime);
        if (!ptr->isDirty)
        {
            return;
        }

        OpRuntime_opGraphDestroy(runtime);

        // for each group, create an opGraph
        {
            // create root op for group
            NvFlowOpGraphDesc opGraphDesc = {};
            opGraphDesc.opTypename = "root";
            opGraphDesc.nativePinsIn = ptr->desc.nativePinsIn;
            opGraphDesc.nativePinsOut = ptr->desc.nativePinsOut;
            opGraphDesc.logPrint = ptr->desc.logPrint;

            ptr->opGraph = ptr->opGraphInterface.create(&opGraphDesc);

            // add op interfaces
            for (NvFlowUint opIdx = 0u; opIdx < ptr->runtimeOps.size; opIdx++)
            {
                OpRuntimeOp* op = ptr->runtimeOps[opIdx];
                NvFlowOpInterface* opInterfaces[1] = { &op->opInterface };
                ptr->opGraphInterface.addOpInterfaces(ptr->opGraph, opInterfaces, 1u);
            }

            // add op instances
            for (NvFlowUint opIdx = 0u; opIdx < ptr->runtimeOps.size; opIdx++)
            {
                OpRuntimeOp* op = ptr->runtimeOps[opIdx];

                ptr->opGraphInterface.addOp(ptr->opGraph, op->opInterface.opTypename, op->instanceName);
            }

            NvFlowBool32 succeeded = ptr->opGraphInterface.build(ptr->opGraph);
            // generate blank opGraph if compilation fails
            if (!succeeded)
            {
                ptr->desc.logPrint(eNvFlowLogLevel_error, "OpRuntime build failed, falling back to null behavior");

                ptr->opGraphInterface.destroy(ptr->opGraph);
                ptr->opGraph = nullptr;

                ptr->opGraph = ptr->opGraphInterface.create(&opGraphDesc);

                ptr->opGraphInterface.build(ptr->opGraph);
            }

            // get interface
            NvFlowOpInterface* groupOpInterface = nullptr;
            ptr->opGraphInterface.getOpInterface(ptr->opGraph, &groupOpInterface);
            NvFlowOpInterface_duplicate(&ptr->groupOpInterface, groupOpInterface);

            ptr->pinDataPool.size = 0u;
            ptr->pinDataPool.allocateBack();
            ptr->pinDataPool.allocateBack();

            ptr->pinDataPool[0].reserve(ptr->groupOpInterface.pinsIn->elementSize);
            ptr->pinDataPool[0].size = ptr->groupOpInterface.pinsIn->elementSize;
            ptr->pinDataPool[1].reserve(ptr->groupOpInterface.pinsOut->elementSize);
            ptr->pinDataPool[1].size = ptr->groupOpInterface.pinsOut->elementSize;

            // set defaults
            if (ptr->groupOpInterface.pinsIn->defaultValue)
            {
                ptr->groupOpInterface.pinsIn->defaultValue(ptr->groupOpInterface.pinsIn->reflectContext, ptr->pinDataPool[0].data);
            }
            if (ptr->groupOpInterface.pinsOut->defaultValue)
            {
                ptr->groupOpInterface.pinsOut->defaultValue(ptr->groupOpInterface.pinsOut->reflectContext, ptr->pinDataPool[1].data);
            }

            // write defaults
            ptr->groupOpInterface.pinsIn->traverse(ptr->groupOpInterface.pinsIn->reflectContext, ptr->pinDataPool[0].data, setPinDefaults, ptr);
            ptr->groupOpInterface.pinsOut->traverse(ptr->groupOpInterface.pinsOut->reflectContext, ptr->pinDataPool[1].data, setPinDefaults, ptr);

            // create op instance
            ptr->groupOp = ptr->groupOpInterface.create(&ptr->groupOpInterface, (NvFlowOpGenericPinsIn*)ptr->pinDataPool[0].data, (NvFlowOpGenericPinsOut*)ptr->pinDataPool[1].data);
        }

        ptr->isDirty = NV_FLOW_FALSE;
    }

    void* OpRuntime_mapPinData(NvFlowOpRuntime* runtime)
    {
        auto ptr = cast(runtime);

        update(runtime);

        return ptr->pinDataPool[0].data;
    }

    void OpRuntime_unmapPinData(NvFlowOpRuntime* runtime)
    {
        auto ptr = cast(runtime);

        update(runtime);
    }

    void OpRuntime_processPins(NvFlowOpRuntime* runtime, NvFlowPinDir dir, NvFlowReflectProcess_t process, void* userdata)
    {
        auto ptr = cast(runtime);

        update(runtime);

        if (dir == eNvFlowPinDir_in)
        {
            ptr->groupOpInterface.pinsIn->traverse(ptr->groupOpInterface.pinsIn->reflectContext, ptr->pinDataPool[0].data, process, userdata);
        }
        else
        {
            ptr->groupOpInterface.pinsOut->traverse(ptr->groupOpInterface.pinsOut->reflectContext, ptr->pinDataPool[1].data, process, userdata);
        }
    }

    void OpRuntime_execute(NvFlowOpRuntime* runtime)
    {
        auto ptr = cast(runtime);

        update(runtime);

        ptr->groupOpInterface.execute(ptr->groupOp, (NvFlowOpGenericPinsIn*)ptr->pinDataPool[0].data, (NvFlowOpGenericPinsOut*)ptr->pinDataPool[1].data);
    }

    void OpRuntime_enumerateExecuteGroups(NvFlowOpRuntime* runtime, NvFlowOpExecuteGroupDesc* pExecuteGroups, NvFlowUint64* pExecuteGroupCount)
    {
        auto ptr = cast(runtime);

        update(runtime);

        if (!pExecuteGroups)
        {
            *pExecuteGroupCount = ptr->groupOpInterface.executeGroupCount;
        }
        else
        {
            if ((*pExecuteGroupCount) > ptr->groupOpInterface.executeGroupCount)
            {
                (*pExecuteGroupCount) = ptr->groupOpInterface.executeGroupCount;
            }
            for (NvFlowUint idx = 0u; idx < (*pExecuteGroupCount); idx++)
            {
                pExecuteGroups[idx] = ptr->groupOpInterface.executeGroupDescs[idx];
            }
        }
    }

    void OpRuntime_executeGroup(NvFlowOpRuntime* runtime, NvFlowOpExecuteGroup* group)
    {
        auto ptr = cast(runtime);

        update(runtime);

        ptr->groupOpInterface.executeGroup(ptr->groupOp, group, (NvFlowOpGenericPinsIn*)ptr->pinDataPool[0].data, (NvFlowOpGenericPinsOut*)ptr->pinDataPool[1].data);
    }

    struct PinSet_Userdata
    {
        OpRuntime* ptr;
        NvFlowBool32 didWrite;
        const char* path;
        const void* srcData;
        void* dstData;
        NvFlowUint64 sizeInBytes;
        NvFlowUint stackIdx;
        NvFlowArray<const char*, 16u> stack;
        NvFlowArray<char, 256u> stackPool;
    };

    NV_FLOW_INLINE void PinSet_processPins_copy(PinSet_Userdata* params, NvFlowReflectData* pinDatas, NvFlowUint idx, NvFlowUint64 cmpSizeInBytes)
    {
        if (cmpSizeInBytes == params->sizeInBytes)
        {
            if (params->srcData)
            {
                NvFlowReflectMemcpy(pinDatas[idx].data, params->srcData, params->sizeInBytes);
            }
            if (params->dstData)
            {
                NvFlowReflectMemcpy(params->dstData, pinDatas[idx].data, params->sizeInBytes);
            }
            params->didWrite = NV_FLOW_TRUE;
        }
    }

    NV_FLOW_INLINE void PinSet_processPins(NvFlowReflectData* pinDatas, NvFlowUint pinCount, void* userdata)
    {
        PinSet_Userdata* params = (PinSet_Userdata*)userdata;

        // for root, support full paths
        if (params->stackIdx == 0u)
        {
            for (NvFlowUint idx = 0; idx < pinCount; idx++)
            {
                if (NvFlowReflectStringCompare(params->path, pinDatas[idx].name) == 0)
                {
                    if (pinDatas[idx].reflectMode == eNvFlowReflectMode_pointer)
                    {
                        PinSet_processPins_copy(params, pinDatas, idx, sizeof(void*));
                    }
                    else if (pinDatas[idx].reflectMode == eNvFlowReflectMode_array)
                    {
                        PinSet_processPins_copy(params, pinDatas, idx, pinDatas[idx].dataType->elementSize * (*pinDatas[idx].pArraySize));
                    }
                    else if (pinDatas[idx].reflectMode == eNvFlowReflectMode_pointerArray)
                    {
                        PinSet_processPins_copy(params, pinDatas, idx, sizeof(void*) * (*pinDatas[idx].pArraySize));
                    }
                    else
                    {
                        PinSet_processPins_copy(params, pinDatas, idx, pinDatas[idx].dataType->elementSize);
                    }
                }
            }
        }
        // proper recursive
        for (NvFlowUint idx = 0; idx < pinCount; idx++)
        {
            if (NvFlowReflectStringCompare(params->stack[params->stackIdx], pinDatas[idx].name) == 0)
            {
                // if end of stack, copy
                if (params->stackIdx + 1u == params->stack.size)
                {
                    if (pinDatas[idx].reflectMode == eNvFlowReflectMode_pointer)
                    {
                        PinSet_processPins_copy(params, pinDatas, idx, sizeof(void*));
                    }
                    else if (pinDatas[idx].reflectMode == eNvFlowReflectMode_array)
                    {
                        PinSet_processPins_copy(params, pinDatas, idx, pinDatas[idx].dataType->elementSize * (*pinDatas[idx].pArraySize));
                    }
                    else if (pinDatas[idx].reflectMode == eNvFlowReflectMode_pointerArray)
                    {
                        PinSet_processPins_copy(params, pinDatas, idx, sizeof(void*) * (*pinDatas[idx].pArraySize));
                    }
                    else
                    {
                        PinSet_processPins_copy(params, pinDatas, idx, pinDatas[idx].dataType->elementSize);
                    }
                }
                else if (pinDatas[idx].dataType->dataType == eNvFlowType_struct)
                {
                    params->stackIdx++;
                    void* structData = nullptr;
                    if (pinDatas[idx].reflectMode == eNvFlowReflectMode_pointer)
                    {
                        structData = *(void**)pinDatas[idx].data;
                    }
                    else if (pinDatas[idx].reflectMode == eNvFlowReflectMode_array)
                    {
                        structData = *(void**)pinDatas[idx].data;
                    }
                    else if (pinDatas[idx].reflectMode == eNvFlowReflectMode_pointerArray)
                    {
                        structData = *(void**)pinDatas[idx].data;
                    }
                    else
                    {
                        structData = pinDatas[idx].data;
                    }
                    pinDatas[idx].dataType->traverse(pinDatas[idx].dataType->reflectContext, structData, PinSet_processPins, userdata);
                    params->stackIdx--;
                }
            }
        }
    }

    void OpRuntime_setPin_initStack(PinSet_Userdata* ptr)
    {
        if (ptr->path)
        {
            for (NvFlowUint strIdx = 0u; ptr->path[strIdx]; strIdx++)
            {
                char c = ptr->path[strIdx];
                ptr->stackPool.pushBack(c == '.' ? '\0' : c);
            }
            ptr->stackPool.pushBack('\0');

            NvFlowUint baseStrIdx = 0u;
            for (NvFlowUint strIdx = 0u; strIdx < ptr->stackPool.size; strIdx++)
            {
                if (ptr->stackPool[strIdx] == '\0')
                {
                    ptr->stack.pushBack(&ptr->stackPool[baseStrIdx]);
                    baseStrIdx = strIdx + 1u;
                }
            }
        }
    }

    NvFlowBool32 OpRuntime_setPin(NvFlowOpRuntime* runtime, NvFlowPinDir dir, const char* name, const void* srcData, NvFlowUint64 srcSizeInBytes)
    {
        PinSet_Userdata pinSetParams = { cast(runtime), NV_FLOW_FALSE, name, srcData, 0, srcSizeInBytes, 0u };
        OpRuntime_setPin_initStack(&pinSetParams);
        OpRuntime_processPins(runtime, dir, PinSet_processPins, &pinSetParams);
        return pinSetParams.didWrite;
    }

    NvFlowBool32 OpRuntime_getPin(NvFlowOpRuntime* runtime, NvFlowPinDir dir, const char* name, void* dstData, NvFlowUint64 dstSizeInBytes)
    {
        PinSet_Userdata pinSetParams = { cast(runtime), NV_FLOW_FALSE, name, 0, dstData, dstSizeInBytes, 0u };
        OpRuntime_setPin_initStack(&pinSetParams);
        OpRuntime_processPins(runtime, dir, PinSet_processPins, &pinSetParams);
        return pinSetParams.didWrite;
    }

    NvFlowBool32 OpRuntime_setPinPointer(NvFlowOpRuntime* runtime, NvFlowPinDir dir, const char* name, const void* srcPointer)
    {
        return OpRuntime_setPin(runtime, dir, name, &srcPointer, sizeof(srcPointer));
    }

    NvFlowBool32 OpRuntime_getPinPointer(NvFlowOpRuntime* runtime, NvFlowPinDir dir, const char* name, void** pDstPointer)
    {
        void* ptr = nullptr;
        NvFlowBool32 ret = OpRuntime_getPin(runtime, dir, name, &ptr, sizeof(ptr));
        *pDstPointer = ptr;
        return ret;
    }
}

NvFlowOpRuntimeInterface* NvFlowGetOpRuntimeInterface()
{
    static NvFlowOpRuntimeInterface iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowOpRuntimeInterface) };
    iface.create = OpRuntime_create;
    iface.destroy = OpRuntime_destroy;
    iface.addOp = OpRuntime_addOp;
    iface.removeOp = OpRuntime_removeOp;
    iface.mapPinData = OpRuntime_mapPinData;
    iface.unmapPinData = OpRuntime_unmapPinData;
    iface.processPins = OpRuntime_processPins;
    iface.execute = OpRuntime_execute;
    iface.enumerateExecuteGroups = OpRuntime_enumerateExecuteGroups;
    iface.executeGroup = OpRuntime_executeGroup;
    iface.setPin = OpRuntime_setPin;
    iface.getPin = OpRuntime_getPin;
    iface.setPinPointer = OpRuntime_setPinPointer;
    iface.getPinPointer = OpRuntime_getPinPointer;
    return &iface;
}

#endif

NvFlowOpRuntimeInterface* NvFlowGetOpRuntimeInterface()
{
    return nullptr;
}

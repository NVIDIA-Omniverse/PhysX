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

#include "ShaderPreprocessorInternal.h"

char* NvFlowShaderPreprocessor_headerGen(NvFlowShaderPreprocessor* ptr, const char* readPath, const char* shaderName)
{
    clearTempStr(ptr);

    // include shader bytecode
    for (NvFlowUint64 perConfigIdx = 0u; perConfigIdx < ptr->perConfigs.size; perConfigIdx++)
    {
        auto perConfig = &ptr->perConfigs[perConfigIdx];

        if (NvFlowStringLength(perConfig->bytecodeHeaderFilename) > 0)
        {
            printTempStr(ptr, "#include \"%s\"\n", perConfig->bytecodeHeaderFilename);
        }
    }

    if (ptr->resources.size > 0u)
    {
        // compute max binding descs
        NvFlowUint64 maxBindingDescs = 0u;
        for (NvFlowUint64 configIdx = 0u; configIdx < ptr->perConfigs.size; configIdx++)
        {
            auto perConfig = &ptr->perConfigs[configIdx];
            if (perConfig->bindingDescs.size > maxBindingDescs)
            {
                maxBindingDescs = perConfig->bindingDescs.size;
            }
        }

        // pipeline struct
        printTempStr(ptr,
                     "\nstruct %s_Pipeline\n{\n"
                     "\tNvFlowContextInterface* contextInterface;\n"
                     "\tNvFlowComputePipeline* pipeline;\n"
                     "};\n",
                     shaderName);

        // pipeline create
        printTempStr(
            ptr,
            "\ninline void %s_init(NvFlowContextInterface* contextInterface, NvFlowContext* context, %s_Pipeline* ptr)\n{\n"
            "\tptr->contextInterface = contextInterface;\n"
            "\tNvFlowContextConfig config = {};\n"
            "\tptr->contextInterface->getContextConfig(context, &config);\n"
            "\tNvFlowBindingDesc bindingDescs[%d] = {};\n"
            "\tNvFlowComputePipelineDesc desc = {};\n",
            shaderName, shaderName, maxBindingDescs);
        for (NvFlowUint64 configIdx = 0u; configIdx < ptr->perConfigs.size; configIdx++)
        {
            auto perConfig = &ptr->perConfigs[configIdx];
            if (perConfig->bindingDescs.size > 0u)
            {
                printTempStr(ptr, "\tif (config.api == %s)\n\t{\n", ptr->contextApi_to_stringView[perConfig->config.api]);
                for (NvFlowUint64 bindingIdx = 0u; bindingIdx < perConfig->bindingDescs.size; bindingIdx++)
                {
                    NvFlowShaderPreprocessorBindingDesc* bindingDesc = &perConfig->bindingDescs[bindingIdx];
                    printTempStr(ptr, "\t\tbindingDescs[%d].type = e%s;\n", bindingIdx,
                                 ptr->descriptorType_to_stringView[bindingDesc->desc.type]);
                    if (perConfig->config.api == eNvFlowContextApi_vulkan)
                    {
                        printTempStr(ptr,
                                     "\t\tbindingDescs[%d].bindingDesc.vulkan.binding = %d;\n"
                                     "\t\tbindingDescs[%d].bindingDesc.vulkan.descriptorCount = %d;\n"
                                     "\t\tbindingDescs[%d].bindingDesc.vulkan.set = %d;\n",
                                     bindingIdx, bindingDesc->desc.bindingDesc.vulkan.binding, bindingIdx,
                                     bindingDesc->desc.bindingDesc.vulkan.descriptorCount, bindingIdx,
                                     bindingDesc->desc.bindingDesc.vulkan.set);
                    }
                    else if (perConfig->config.api == eNvFlowContextApi_d3d12)
                    {
                        printTempStr(ptr,
                                     "\t\tbindingDescs[%d].bindingDesc.d3d12.registerHlsl = %s;\n"
                                     "\t\tbindingDescs[%d].bindingDesc.d3d12.registerBegin = %d;\n"
                                     "\t\tbindingDescs[%d].bindingDesc.d3d12.numRegisters = %d;\n"
                                     "\t\tbindingDescs[%d].bindingDesc.d3d12.space = %d;\n",
                                     bindingIdx,
                                     ptr->registerHlsl_to_stringView[bindingDesc->desc.bindingDesc.d3d12.registerHlsl],
                                     bindingIdx, bindingDesc->desc.bindingDesc.d3d12.registerBegin, bindingIdx,
                                     bindingDesc->desc.bindingDesc.d3d12.numRegisters, bindingIdx,
                                     bindingDesc->desc.bindingDesc.d3d12.space);
                    }
                    printTempStr(ptr, "\n");
                }
                const char* entryName = perConfig->bytecodeVariableName;
                printTempStr(ptr,
                             "\t\tdesc.numBindingDescs = %d;\n"
                             "\t\tdesc.bindingDescs = bindingDescs;\n"
                             "\t\tdesc.bytecode.data = %s;\n"
                             "\t\tdesc.bytecode.sizeInBytes = sizeof(%s);\n",
                             perConfig->bindingDescs.size, entryName, entryName);
                printTempStr(ptr, "\t}\n");
            }
        }
        printTempStr(ptr,
                     "\tptr->pipeline = ptr->contextInterface->createComputePipeline(context, &desc);\n"
                     "};\n");

        // pipeline destroy
        printTempStr(ptr,
                     "\ninline void %s_destroy(NvFlowContext* context, %s_Pipeline* ptr)\n{\n"
                     "\tptr->contextInterface->destroyComputePipeline(context, ptr->pipeline);\n"
                     "}\n",
                     shaderName, shaderName);

        // abstract binding struct
        printTempStr(ptr, "\nstruct %s_PassParams\n{\n", shaderName);
        for (NvFlowUint64 idx = 0u; idx < ptr->resources.size; idx++)
        {
            auto resource = &ptr->resources[idx];

            const char* resourceTypeStr = nullptr;
            switch (resource->descriptorType)
            {
            case eNvFlowDescriptorType_constantBuffer:
            case eNvFlowDescriptorType_structuredBuffer:
            case eNvFlowDescriptorType_buffer:
            case eNvFlowDescriptorType_rwBuffer:
            case eNvFlowDescriptorType_rwStructuredBuffer:
                resourceTypeStr = "NvFlowBufferTransient";
                break;
            case eNvFlowDescriptorType_texture:
            case eNvFlowDescriptorType_rwTexture:
                resourceTypeStr = "NvFlowTextureTransient";
                break;
            case eNvFlowDescriptorType_sampler:
                resourceTypeStr = "NvFlowSampler";
                break;
            default:
                resourceTypeStr = "Invalid";
            }

            printTempStr(ptr, "\t%s* %s;\n", resourceTypeStr, resource->instanceName);
        }
        printTempStr(ptr, "};\n");

        // addPassCompute
        printTempStr(
            ptr,
            "\ninline void %s_addPassCompute(NvFlowContext* context, %s_Pipeline* ptr, NvFlowUint3 gridDim, const %s_PassParams* params)\n{\n"
            "\tNvFlowContextConfig config = {};\n"
            "\tptr->contextInterface->getContextConfig(context, &config);\n"
            "\tNvFlowDescriptorWrite descriptorWrites[%d] = {};\n"
            "\tNvFlowResource resources[%d] = {};\n"
            "\tNvFlowPassComputeParams passComputeParams = {};\n"
            "\tpassComputeParams.pipeline = ptr->pipeline;\n"
            "\tpassComputeParams.gridDim = gridDim;\n",
            shaderName, shaderName, shaderName, maxBindingDescs, maxBindingDescs);
        for (NvFlowUint64 configIdx = 0u; configIdx < ptr->perConfigs.size; configIdx++)
        {
            auto perConfig = &ptr->perConfigs[configIdx];
            if (perConfig->bindingDescs.size > 0u)
            {
                printTempStr(ptr, "\tif (config.api == %s)\n\t{\n", ptr->contextApi_to_stringView[perConfig->config.api]);
                for (NvFlowUint64 bindingIdx = 0u; bindingIdx < perConfig->bindingDescs.size; bindingIdx++)
                {
                    NvFlowShaderPreprocessorBindingDesc* bindingDesc = &perConfig->bindingDescs[bindingIdx];
                    NvFlowShaderPreprocessorResource* resource = bindingDesc->resourceIndex < ptr->resources.size ?
                                                                     &ptr->resources[bindingDesc->resourceIndex] :
                                                                     nullptr;
                    const char* resourceInstanceName = resource ? resource->instanceName : "InvalidInstanceName";

                    printTempStr(ptr, "\t\tdescriptorWrites[%d].type = e%s;\n", bindingIdx,
                                 ptr->descriptorType_to_stringView[bindingDesc->desc.type]);
                    if (perConfig->config.api == eNvFlowContextApi_d3d12)
                    {
                        printTempStr(ptr,
                                     "\t\tdescriptorWrites[%d].write.d3d12.registerHlsl = %s;\n"
                                     "\t\tdescriptorWrites[%d].write.d3d12.registerIndex = %d;\n"
                                     "\t\tdescriptorWrites[%d].write.d3d12.space = %d;\n",
                                     bindingIdx,
                                     ptr->registerHlsl_to_stringView[bindingDesc->desc.bindingDesc.d3d12.registerHlsl],
                                     bindingIdx,
                                     bindingDesc->desc.bindingDesc.d3d12.registerBegin, // TODO : array support proper
                                     bindingIdx, bindingDesc->desc.bindingDesc.d3d12.space);
                    }
                    else if (perConfig->config.api == eNvFlowContextApi_vulkan)
                    {
                        printTempStr(ptr,
                                     "\t\tdescriptorWrites[%d].write.vulkan.binding = %d;\n"
                                     "\t\tdescriptorWrites[%d].write.vulkan.arrayIndex = %d;\n"
                                     "\t\tdescriptorWrites[%d].write.vulkan.set = %d;\n",
                                     bindingIdx, bindingDesc->desc.bindingDesc.vulkan.binding, bindingIdx,
                                     0u, // TODO : array support proper
                                     bindingIdx, bindingDesc->desc.bindingDesc.vulkan.set);
                    }
                    if (bindingDesc->desc.type == eNvFlowDescriptorType_textureSampler)
                    {
                        // first, attempt to find just the texture name, if found, use default sampler
                        if (NvFlowStringCompare(resourceInstanceName, bindingDesc->bindingName) == 0)
                        {
                            printTempStr(ptr,
                                         "\t\tresources[%d].textureTransient = params->%s;\n"
                                         "\t\tresources[%d].sampler = defaultSampler;\n",
                                         bindingIdx, resourceInstanceName, bindingIdx);
                        }
                    }
                    else if (bindingDesc->desc.type == eNvFlowDescriptorType_texture ||
                             bindingDesc->desc.type == eNvFlowDescriptorType_rwTexture)
                    {
                        printTempStr(ptr, "\t\tresources[%d].textureTransient = params->%s;\n", bindingIdx,
                                     resourceInstanceName);
                    }
                    else if (bindingDesc->desc.type == eNvFlowDescriptorType_sampler)
                    {
                        printTempStr(ptr, "\t\tresources[%d].sampler = params->%s;\n", bindingIdx, resourceInstanceName);
                    }
                    else
                    {
                        printTempStr(
                            ptr, "\t\tresources[%d].bufferTransient = params->%s;\n", bindingIdx, resourceInstanceName);
                    }
                    printTempStr(ptr, "\n");
                }
                const char* entryName = perConfig->bytecodeVariableName;
                printTempStr(ptr,
                             "\t\tpassComputeParams.numDescriptorWrites = %d;\n"
                             "\t\tpassComputeParams.descriptorWrites = descriptorWrites;\n"
                             "\t\tpassComputeParams.resources = resources;\n"
                             "\t\tpassComputeParams.debugLabel = \"%s\";\n",
                             perConfig->bindingDescs.size, shaderName);
                printTempStr(ptr, "\t}\n");
            }
        }
        printTempStr(ptr,
                     "\tptr->contextInterface->addPassCompute(context, &passComputeParams);\n"
                     "};\n");
    }

    char* headerStr = concatTempStr(ptr);
    clearTempStr(ptr);

    return headerStr;
}

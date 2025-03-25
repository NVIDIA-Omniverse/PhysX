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
#include "SlangCompiler.h"

NvFlowShaderPreprocessor* NvFlowShaderPreprocessorCreate()
{
    auto ptr = new NvFlowShaderPreprocessor();

    init_descriptorType_to_registerHlsl(ptr->descriptorType_to_registerHlsl);

    init_descriptorType_to_stringView(ptr->descriptorType_to_stringView);
    init_contextApi_to_stringView(ptr->contextApi_to_stringView);
    init_registerHlsl_to_stringView(ptr->registerHlsl_to_stringView);

    ptr->stringPool = NvFlowStringPoolCreate();

    return ptr;
}

void NvFlowShaderPreprocessor_reset(NvFlowShaderPreprocessor* ptr)
{
    NvFlowStringPoolReset(ptr->stringPool);

    ptr->resources.size = 0u;
    ptr->perConfigs.size = 0u;
}

void NvFlowShaderPreprocessorDestroy(NvFlowShaderPreprocessor* ptr)
{
    NvFlowStringPoolDestroy(ptr->stringPool);

    delete ptr;
}

/// Local namespace begin
namespace
{

void bindingAllocatePerConfig(NvFlowShaderPreprocessor* ptr,
                              NvFlowDescriptorType type,
                              NvFlowUint64 resourceIndex,
                              const char* bindingName,
                              NvFlowUint64 configIdx)
{
    NvFlowShaderPreprocessorBindingDesc bindingDesc = {};
    bindingDesc.desc.type = type;
    bindingDesc.bindingName = bindingName;
    bindingDesc.resourceIndex = resourceIndex;

    if (ptr->perConfigs[configIdx].config.api == eNvFlowContextApi_d3d12)
    {
        bindingDesc.desc.bindingDesc.d3d12.registerHlsl = ptr->descriptorType_to_registerHlsl[type];
        bindingDesc.desc.bindingDesc.d3d12.registerBegin =
            ptr->hlslRegisterCount[bindingDesc.desc.bindingDesc.d3d12.registerHlsl]++;
        bindingDesc.desc.bindingDesc.d3d12.numRegisters = 1u; // TODO : array support proper
        bindingDesc.desc.bindingDesc.d3d12.space = 0u;
    }
    else if (ptr->perConfigs[configIdx].config.api == eNvFlowContextApi_vulkan)
    {
        bindingDesc.desc.bindingDesc.vulkan.binding = ptr->vkBindingCount++;
        bindingDesc.desc.bindingDesc.vulkan.descriptorCount = 1u; // TODO : array support proper
        bindingDesc.desc.bindingDesc.vulkan.set = 0u;
    }

    ptr->perConfigs[configIdx].bindingDescs.pushBack(bindingDesc);
}

void pushResource(NvFlowShaderPreprocessor* ptr, NvFlowShaderPreprocessorResource resource)
{
    for (NvFlowUint64 idx = 0u; idx < ptr->resources.size; idx++)
    {
        if (NvFlowStringCompare(ptr->resources[idx].instanceName, resource.instanceName) == 0)
        {
            return;
        }
    }
    ptr->resources.pushBack(resource);
}

NvFlowUint64 getResourceIndexByInstanceName(NvFlowShaderPreprocessor* ptr, const char* instanceName)
{
    for (NvFlowUint64 resourceIdx = 0u; resourceIdx < ptr->resources.size; resourceIdx++)
    {
        if (NvFlowStringCompare(ptr->resources[resourceIdx].instanceName, instanceName) == 0)
        {
            return resourceIdx;
        }
    }
    return ~0u;
}

NvFlowShaderPreprocessorResource getResourceByIndex(NvFlowShaderPreprocessor* ptr, NvFlowUint64 resourceIndex)
{
    if (resourceIndex < ptr->resources.size)
    {
        return ptr->resources[resourceIndex];
    }
    return NvFlowShaderPreprocessorResource();
}

static const NvFlowUint64 resourceTypeDescCount = 13u;

struct ResourceTypeDesc
{
    const char* typeName;
    NvFlowDescriptorType descriptorType;
    NvFlowUint64 dimension;
    NvFlowBool32 isWrite;
};

static const ResourceTypeDesc resourceTypeDescs[resourceTypeDescCount] = {
    { "InvalidResource", eNvFlowDescriptorType_unknown, 0u, NV_FLOW_FALSE },
    { "ConstantBuffer", eNvFlowDescriptorType_constantBuffer, 0u, NV_FLOW_FALSE },
    { "StructuredBuffer", eNvFlowDescriptorType_structuredBuffer, 0u, NV_FLOW_FALSE },
    { "RWStructuredBuffer", eNvFlowDescriptorType_rwStructuredBuffer, 0u, NV_FLOW_FALSE },
    { "Buffer", eNvFlowDescriptorType_buffer, 0u, NV_FLOW_FALSE },
    { "Texture1D", eNvFlowDescriptorType_texture, 1u, NV_FLOW_FALSE },
    { "Texture2D", eNvFlowDescriptorType_texture, 2u, NV_FLOW_FALSE },
    { "Texture3D", eNvFlowDescriptorType_texture, 3u, NV_FLOW_FALSE },
    { "SamplerState", eNvFlowDescriptorType_sampler, 0u, NV_FLOW_FALSE },
    { "RWBuffer", eNvFlowDescriptorType_rwBuffer, 0u, NV_FLOW_TRUE },
    { "RWTexture1D", eNvFlowDescriptorType_rwTexture, 1u, NV_FLOW_TRUE },
    { "RWTexture2D", eNvFlowDescriptorType_rwTexture, 2u, NV_FLOW_TRUE },
    { "RWTexture3D", eNvFlowDescriptorType_rwTexture, 3u, NV_FLOW_TRUE }
};

ResourceTypeDesc resourceTypeDescLookup(const char* resourceTypeName)
{
    for (NvFlowUint64 idx = 0u; idx < resourceTypeDescCount; idx++)
    {
        if (NvFlowStringCompare(resourceTypeDescs[idx].typeName, resourceTypeName) == 0)
        {
            return resourceTypeDescs[idx];
        }
    }
    return resourceTypeDescs[0u];
}

struct ResourceTypeSlangDesc
{
    const char* typeName;
    SlangTypeKind typeKind;
    SlangResourceShape resourceShape;
    SlangResourceAccess access;
};

// clang-format off
static ResourceTypeSlangDesc resourceTypeFromSlang[resourceTypeDescCount] = {
    {"InvalidResource", SLANG_TYPE_KIND_NONE, SLANG_RESOURCE_NONE, SLANG_RESOURCE_ACCESS_NONE},
    {"ConstantBuffer", SLANG_TYPE_KIND_CONSTANT_BUFFER, SLANG_RESOURCE_NONE, SLANG_RESOURCE_ACCESS_NONE},
    {"StructuredBuffer", SLANG_TYPE_KIND_RESOURCE, SLANG_STRUCTURED_BUFFER, SLANG_RESOURCE_ACCESS_READ},
    {"RWStructuredBuffer", SLANG_TYPE_KIND_RESOURCE, SLANG_STRUCTURED_BUFFER, SLANG_RESOURCE_ACCESS_READ_WRITE},
    {"Buffer", SLANG_TYPE_KIND_RESOURCE, SLANG_TEXTURE_BUFFER, SLANG_RESOURCE_ACCESS_READ},
    {"Texture1D", SLANG_TYPE_KIND_RESOURCE, SLANG_TEXTURE_1D, SLANG_RESOURCE_ACCESS_READ},
    {"Texture2D", SLANG_TYPE_KIND_RESOURCE, SLANG_TEXTURE_2D, SLANG_RESOURCE_ACCESS_READ},
    {"Texture3D", SLANG_TYPE_KIND_RESOURCE, SLANG_TEXTURE_3D, SLANG_RESOURCE_ACCESS_READ},
    {"SamplerState", SLANG_TYPE_KIND_SAMPLER_STATE, SLANG_RESOURCE_NONE, SLANG_RESOURCE_ACCESS_NONE},
    {"RWBuffer", SLANG_TYPE_KIND_RESOURCE, SLANG_RESOURCE_NONE, SLANG_RESOURCE_ACCESS_READ_WRITE},
    {"RWTexture1D", SLANG_TYPE_KIND_RESOURCE, SLANG_TEXTURE_1D, SLANG_RESOURCE_ACCESS_READ_WRITE},
    {"RWTexture2D", SLANG_TYPE_KIND_RESOURCE, SLANG_TEXTURE_2D, SLANG_RESOURCE_ACCESS_READ_WRITE},
    {"RWTexture3D", SLANG_TYPE_KIND_RESOURCE, SLANG_TEXTURE_3D, SLANG_RESOURCE_ACCESS_READ_WRITE},
};
// clang-format on

const char* resourceTypeFromSlangLookup(SlangTypeKind typeKind, SlangResourceShape resourceShape, SlangResourceAccess access)
{
    for (NvFlowUint64 idx = 0u; idx < resourceTypeDescCount; idx++)
    {
        if (typeKind == resourceTypeFromSlang[idx].typeKind &&
            resourceShape == resourceTypeFromSlang[idx].resourceShape && access == resourceTypeFromSlang[idx].access)
        {
            return resourceTypeFromSlang[idx].typeName;
        }
    }
    return resourceTypeFromSlang[0u].typeName;
}

void extractResourceFromSlang(NvFlowShaderPreprocessor* ptr,
                              NvFlowShaderPreprocessorResource* pResource,
                              const nvflow::ShaderParameter& shaderParam)
{
    pResource->resourceType =
        resourceTypeFromSlangLookup(shaderParam.typeKind, shaderParam.resourceShape, shaderParam.access);
    pResource->format = nullptr;
    ptr->tempStrViews.pushBack(shaderParam.name.c_str());
    pResource->instanceName = NvFlowStringConcatN(ptr->stringPool, ptr->tempStrViews.data, ptr->tempStrViews.size);
    ptr->tempStrViews.size = 0u;

    ResourceTypeDesc typeDesc = resourceTypeDescLookup(pResource->resourceType);
    pResource->descriptorType = typeDesc.descriptorType;
    pResource->dimension = typeDesc.dimension;
    pResource->isWrite = typeDesc.isWrite;
}

} /// Local namespace end

/// ******************************* Configure **********************************

void NvFlowShaderPreprocessor_configure(NvFlowShaderPreprocessor* ptr, NvFlowContextConfig config)
{
    ptr->vkBindingCount = 0u;
    for (NvFlowUint64 idx = 0u; idx < eNvFlowRegisterHlsl_count; idx++)
    {
        ptr->hlslRegisterCount[idx] = 0u;
    }

    NvFlowUint64 perConfigIdx = ptr->perConfigs.allocateBack();
    ptr->perConfig = &ptr->perConfigs[perConfigIdx];
    ptr->perConfig->config = config;
    ptr->perConfig->bytecodeVariableName = nullptr;
    ptr->perConfig->bytecodeHeaderFilename = nullptr;
    ptr->perConfig->bindingDescs.size = 0u;

    ptr->tempStrViews.size = 0u;
}

void NvFlowShaderPreprocessorBuildShader(NvFlowShaderPreprocessor* ptr, const NvFlowShaderPreprocessorBuildParams* params)
{
    NvFlowShaderPreprocessor_reset(ptr);

    char* readDir = {};
    char* readFile = {};
    NvFlowStringSplitDelimLast(ptr->stringPool, &readDir, &readFile, params->readPath, '/');

    char* writeDir = {};
    char* writeFile = {};
    NvFlowStringSplitDelimLast(ptr->stringPool, &writeDir, &writeFile, params->writePath, '/');

    char* writeFileLessExtension = {};
    char* writeFileExtension = {};
    NvFlowStringSplitDelimLast(ptr->stringPool, &writeFileLessExtension, &writeFileExtension, writeFile, '.');

    writeFileLessExtension = NvFlowStringTrimEnd(ptr->stringPool, writeFileLessExtension, '.');

    const char* shaderName = writeFileLessExtension;
    const char* readPath = params->readPath;

    char* writePath_h = NvFlowStringConcat3(ptr->stringPool, writeDir, writeFile, ".h");

    nvflow::SlangCompiler slangCompiler;

#if defined(_WIN32)
    NvFlowUint64 slangPasses = 2u;
#else
    NvFlowUint64 slangPasses = 1u;
#endif

    bool runHeaderPass = false;

#ifdef SLANG_DEBUG_OUTPUT
    {
        NvFlowUint64 passID = 0u;

        char* writePath_hlsl_h = NvFlowStringConcat3(ptr->stringPool, writeDir, writeFileLessExtension, ".slang");

        char* variableName = nullptr;
#else
    for (NvFlowUint64 passID = 0u; passID < slangPasses; passID++)
    {
        NvFlowContextApi contextApi = passID == 0u ? eNvFlowContextApi_vulkan : eNvFlowContextApi_d3d12;

        NvFlowShaderPreprocessor_configure(ptr, NvFlowContextConfig{ contextApi });

        const char* writeFileLessExtensionVariant =
            NvFlowStringConcat(ptr->stringPool, writeFileLessExtension, passID == 0u ? "_vulkan" : "_d3d12");

        // Output path with '_vulkan' or '_d3d12' config
        char* writePath_hlsl_h = NvFlowStringConcat3(ptr->stringPool, writeDir, writeFileLessExtensionVariant, ".hlsl.h");

        ptr->perConfig->bytecodeHeaderFilename =
            NvFlowStringConcat(ptr->stringPool, writeFileLessExtensionVariant, ".hlsl.h");

        char* variableName = NvFlowStringConcat(ptr->stringPool, writeFileLessExtensionVariant, "_hlsl");
        ptr->perConfig->bytecodeVariableName = variableName;

#endif
        if (!NvFlowTextFileTestOpen(writePath_hlsl_h) || !NvFlowTextFileTestOpen(writePath_h))
        {
            // Invalidate old bytecode, to throw C++ compile error if shader compile fails
            NvFlowTextFileRemove(writePath_hlsl_h);

            // Slang compilation
#if defined(_WIN32) && !defined(SLANG_DEBUG_OUTPUT)
            bool useVulkan = ptr->perConfig->config.api == eNvFlowContextApi_vulkan;
#else
            bool useVulkan = true;
#endif
            runHeaderPass |= slangCompiler.compileFile(readPath, writePath_hlsl_h, variableName,
                                                       size_t(params->numIncludePaths), params->includePaths, useVulkan);
        }
    }

#ifdef SLANG_DEBUG_OUTPUT
    return;
#endif

    // Create resources (slang reflection was copied after successful compilation)
    const auto& shaderParameters = slangCompiler.getShaderParameters();
    for (const auto& shaderParam : shaderParameters)
    {
        NvFlowShaderPreprocessorResource resource = {};
        extractResourceFromSlang(ptr, &resource, shaderParam);
        pushResource(ptr, resource);

        auto resourceIndex = getResourceIndexByInstanceName(ptr, resource.instanceName);
        ResourceTypeDesc typeDesc = resourceTypeDescLookup(resource.resourceType);

        for (NvFlowUint64 configIdx = 0u; configIdx < ptr->perConfigs.size; configIdx++)
        {
            bindingAllocatePerConfig(ptr, typeDesc.descriptorType, resourceIndex, resource.instanceName, configIdx);
        }
    }

    if (runHeaderPass)
    {
        char* writePath_h_tmp = NvFlowStringConcat3(ptr->stringPool, writeDir, writeFile, ".tmp.h");

        char* headerStr = NvFlowShaderPreprocessor_headerGen(ptr, params->readPath, shaderName);

        NvFlowTextFileStore(headerStr, writePath_h_tmp);

        NvFlowTextFileDiffAndWriteIfModified(writePath_h, writePath_h_tmp);
    }

    // reset to invalidate external string views
    NvFlowShaderPreprocessor_reset(ptr);
}

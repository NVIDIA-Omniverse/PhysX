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

#pragma once

#include "NvFlowArray.h"
#include "NvFlowContext.h"
#include "ShaderPreprocessor.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

struct NvFlowShaderPreprocessorResource
{
    const char* resourceType = nullptr;
    const char* format = nullptr;
    const char* instanceName = nullptr;
    NvFlowDescriptorType descriptorType = eNvFlowDescriptorType_unknown;
    NvFlowUint64 dimension = 0u;
    NvFlowBool32 isWrite = NV_FLOW_FALSE;
};

struct NvFlowShaderPreprocessorBindingDesc
{
    NvFlowBindingDesc desc;
    const char* bindingName;
    NvFlowUint64 resourceIndex;
};

struct NvFlowShaderPreprocessorPerConfig
{
    NvFlowContextConfig config;
    const char* bytecodeVariableName;
    const char* bytecodeHeaderFilename;
    NvFlowArray<NvFlowShaderPreprocessorBindingDesc> bindingDescs;
};

struct NvFlowShaderPreprocessor
{
    NvFlowStringPool* stringPool = nullptr;

    NvFlowArray<NvFlowShaderPreprocessorResource> resources;
    NvFlowArray<NvFlowShaderPreprocessorPerConfig, 8u> perConfigs;

    NvFlowUint vkBindingCount = 0u;
    NvFlowUint hlslRegisterCount[eNvFlowRegisterHlsl_count] = { 0 };

    NvFlowShaderPreprocessorPerConfig* perConfig = nullptr;
    NvFlowArray<const char*, 8u> tempStrViews;

    NvFlowRegisterHlsl descriptorType_to_registerHlsl[eNvFlowDescriptorType_count] = {};
    const char* descriptorType_to_stringView[eNvFlowDescriptorType_count] = {};
    const char* contextApi_to_stringView[eNvFlowContextApi_count] = {};
    const char* registerHlsl_to_stringView[eNvFlowRegisterHlsl_count] = {};
};

char* NvFlowShaderPreprocessor_headerGen(NvFlowShaderPreprocessor* ptr,
                                         const char* readPath,
                                         const char* writeFileLessExtension);

/// Local namespace begin
namespace
{

/// ******************************* printTemp **********************************

void clearTempStr(NvFlowShaderPreprocessor* ptr)
{
    ptr->tempStrViews.size = 0u;
}

void printTempStr(NvFlowShaderPreprocessor* ptr, const char* format, ...)
{
    va_list args;
    va_start(args, format);

    char* str = NvFlowStringPrintV(ptr->stringPool, format, args);

    va_end(args);

    ptr->tempStrViews.pushBack(str);
}

char* concatTempStr(NvFlowShaderPreprocessor* ptr)
{
    return NvFlowStringConcatN(ptr->stringPool, ptr->tempStrViews.data, ptr->tempStrViews.size);
}

/// ******************************* utils **********************************

void init_descriptorType_to_registerHlsl(NvFlowRegisterHlsl* table)
{
    table[eNvFlowDescriptorType_unknown] = eNvFlowRegisterHlsl_unknown;
    table[eNvFlowDescriptorType_constantBuffer] = eNvFlowRegisterHlsl_b;
    table[eNvFlowDescriptorType_structuredBuffer] = eNvFlowRegisterHlsl_t;
    table[eNvFlowDescriptorType_buffer] = eNvFlowRegisterHlsl_t;
    table[eNvFlowDescriptorType_texture] = eNvFlowRegisterHlsl_t;
    table[eNvFlowDescriptorType_sampler] = eNvFlowRegisterHlsl_s;
    table[eNvFlowDescriptorType_rwStructuredBuffer] = eNvFlowRegisterHlsl_u;
    table[eNvFlowDescriptorType_rwBuffer] = eNvFlowRegisterHlsl_u;
    table[eNvFlowDescriptorType_rwTexture] = eNvFlowRegisterHlsl_u;

    table[eNvFlowDescriptorType_textureSampler] = eNvFlowRegisterHlsl_unknown;

    table[eNvFlowDescriptorType_indirectBuffer] = eNvFlowRegisterHlsl_unknown;
    table[eNvFlowDescriptorType_bufferCopySrc] = eNvFlowRegisterHlsl_unknown;
    table[eNvFlowDescriptorType_bufferCopyDst] = eNvFlowRegisterHlsl_unknown;
    table[eNvFlowDescriptorType_textureCopySrc] = eNvFlowRegisterHlsl_unknown;
    table[eNvFlowDescriptorType_textureCopyDst] = eNvFlowRegisterHlsl_unknown;
}

void init_descriptorType_to_stringView(const char** table)
{
    table[eNvFlowDescriptorType_unknown] = "NvFlowDescriptorType_unknown";
    table[eNvFlowDescriptorType_constantBuffer] = "NvFlowDescriptorType_constantBuffer";
    table[eNvFlowDescriptorType_structuredBuffer] = "NvFlowDescriptorType_structuredBuffer";
    table[eNvFlowDescriptorType_buffer] = "NvFlowDescriptorType_buffer";
    table[eNvFlowDescriptorType_texture] = "NvFlowDescriptorType_texture";
    table[eNvFlowDescriptorType_sampler] = "NvFlowDescriptorType_sampler";
    table[eNvFlowDescriptorType_rwStructuredBuffer] = "NvFlowDescriptorType_rwStructuredBuffer";
    table[eNvFlowDescriptorType_rwBuffer] = "NvFlowDescriptorType_rwBuffer";
    table[eNvFlowDescriptorType_rwTexture] = "NvFlowDescriptorType_rwTexture";

    table[eNvFlowDescriptorType_textureSampler] = "NvFlowDescriptorType_textureSampler";

    table[eNvFlowDescriptorType_indirectBuffer] = "NvFlowDescriptorType_indirectBuffer";
    table[eNvFlowDescriptorType_bufferCopySrc] = "NvFlowDescriptorType_bufferCopySrc";
    table[eNvFlowDescriptorType_bufferCopyDst] = "NvFlowDescriptorType_bufferCopyDst";
    table[eNvFlowDescriptorType_textureCopySrc] = "NvFlowDescriptorType_textureCopySrc";
    table[eNvFlowDescriptorType_textureCopyDst] = "NvFlowDescriptorType_textureCopyDst";
}

void init_contextApi_to_stringView(const char** table)
{
    table[eNvFlowContextApi_abstract] = "eNvFlowContextApi_abstract";
    table[eNvFlowContextApi_vulkan] = "eNvFlowContextApi_vulkan";
    table[eNvFlowContextApi_d3d12] = "eNvFlowContextApi_d3d12";
    table[eNvFlowContextApi_cpu] = "eNvFlowContextApi_cpu";
}

void init_registerHlsl_to_stringView(const char** table)
{
    table[eNvFlowRegisterHlsl_unknown] = "eNvFlowRegisterHlsl_unknown";
    table[eNvFlowRegisterHlsl_b] = "eNvFlowRegisterHlsl_b";
    table[eNvFlowRegisterHlsl_t] = "eNvFlowRegisterHlsl_t";
    table[eNvFlowRegisterHlsl_s] = "eNvFlowRegisterHlsl_s";
    table[eNvFlowRegisterHlsl_u] = "eNvFlowRegisterHlsl_u";
}

} /// Local namespace end

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

#include "SlangCompiler.h"

#include <fstream>
#include <iomanip>
#include <sstream>

//#define ASM_DEBUG_OUTPUT          // Compiles and saves the assembly code instead of binary
//#define SLANG_DEBUG_OUTPUT        // Turns off obfuscation and saves slang files instead of binaries

#if !defined(SLANG_DEBUG_OUTPUT) && !defined(ASM_DEBUG_OUTPUT)
#   define BINARY_OUTPUT
// #   define SLANG_OBFUSCATE
#endif

namespace nvflow
{
const std::string SlangCompiler::kDxcPath = "../../../external/dxc/bin/dxcompiler";

SlangCompiler::~SlangCompiler()
{
    if (shader_)
    {
        delete shader_;
    }
}

bool SlangCompiler::compileFile(const char* sourceFile,
                                const char* destinationFile,
                                const char* variableName,
                                size_t numIncludePaths,
                                const char** includePaths,
                                bool useVulkan)
{
    std::ifstream inFile(sourceFile);
    if (!inFile)
    {
        return false;
    }

    std::stringstream buffer;
    buffer << inFile.rdbuf();
    std::string code = buffer.str();

    std::vector<std::string> includePathsString;
    for (size_t i = 0; i < numIncludePaths; ++i)
    {
        includePathsString.push_back(includePaths[i]);
    }

    if (!compile(code.c_str(), includePathsString, useVulkan) && !shader_)
    {
        printf("Slang shader compilation of '%s' failed\n", variableName);
        return false;
    }

#ifdef BINARY_OUTPUT
    std::ostringstream oss;
    oss << "const unsigned char " + std::string(variableName) + "[] = {\n  ";

    const unsigned char* bytes = static_cast<const unsigned char*>(shader_->computeShader.byteCode);

    for (size_t i = 0; i < shader_->computeShader.byteCodeSize; ++i)
    {
        oss << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(bytes[i]);
        if (i != shader_->computeShader.byteCodeSize - 1)
        {
            oss << ", ";
        }
        if ((i + 1) % 12 == 0)
        {
            oss << "\n  ";
        }
    }
    oss << "\n};";
#endif

    std::ofstream outFile(destinationFile);
    if (outFile.is_open())
    {
#ifndef BINARY_OUTPUT
        outFile.write(static_cast<const char*>(shader_->computeShader.byteCode), shader_->computeShader.byteCodeSize);
#else
        outFile << oss.str();
#endif
        outFile.close();
    }
    else
    {
        printf("Saving bytecode for '%s' failed\n", variableName);
        return false;
    }

    return true;
}

bool SlangCompiler::compile(const char* codeString, std::vector<std::string> includePaths, bool useVulkan)
{
    if (shader_)
    {
        delete shader_;
    }

    shader_ = new Shader();

    Slang::ComPtr<slang::IGlobalSession> slangSession(spCreateSession(NULL));

    if (!useVulkan)
    {
        slangSession->setDownstreamCompilerPath(SLANG_PASS_THROUGH_DXC, kDxcPath.c_str());
    }

    SlangCompileRequest* slangRequest = spCreateCompileRequest(slangSession);
    if (!slangRequest)
    {
        return false;
    }

    for (const auto& includePath : includePaths)
    {
        spAddSearchPath(slangRequest, includePath.c_str());
    }

    // Pass additional command line arguments to the slang compiler
    {
        std::vector<const char*> additionalArgs;

#if defined SLANG_OBFUSCATE
        additionalArgs.push_back("-obfuscate");
#endif

        if (additionalArgs.size() > 0)
        {
            spProcessCommandLineArguments(slangRequest, additionalArgs.data(), (int)additionalArgs.size());
        }
    }

    spSetMatrixLayoutMode(slangRequest, SLANG_MATRIX_LAYOUT_COLUMN_MAJOR);

    if (useVulkan)
    {
#ifdef ASM_DEBUG_OUTPUT
        int targetIndex = spAddCodeGenTarget(slangRequest, SLANG_SPIRV_ASM);
#elif defined(SLANG_DEBUG_OUTPUT)
        int targetIndex = spAddCodeGenTarget(slangRequest, SLANG_HLSL);
#else
        int targetIndex = spAddCodeGenTarget(slangRequest, SLANG_SPIRV);
#endif
        spSetTargetProfile(slangRequest, targetIndex, spFindProfile(slangSession, "sm_5_1"));
    }
    else
    {
#ifdef ASM_DEBUG_OUTPUT
        int targetIndex = spAddCodeGenTarget(slangRequest, SLANG_DXIL_ASM);
#elif defined(SLANG_DEBUG_OUTPUT)
        int targetIndex = spAddCodeGenTarget(slangRequest, SLANG_HLSL);
#else
        int targetIndex = spAddCodeGenTarget(slangRequest, SLANG_DXIL);
#endif
        spSetTargetProfile(slangRequest, targetIndex, spFindProfile(slangSession, "sm_6_0"));
    }

    int translationUnitIndex = spAddTranslationUnit(slangRequest, SLANG_SOURCE_LANGUAGE_HLSL, nullptr);
    spAddTranslationUnitSourceString(slangRequest, translationUnitIndex, "shader.hlsl", codeString);

    const char entryPointName[] = "main";
    int computeIndex = spAddEntryPoint(slangRequest, translationUnitIndex, entryPointName, SLANG_STAGE_COMPUTE);

    spSetDebugInfoLevel(slangRequest, SLANG_DEBUG_INFO_LEVEL_NONE);
    spSetLineDirectiveMode(slangRequest, SLANG_LINE_DIRECTIVE_MODE_NONE);

#ifdef _DEBUG
    spSetOptimizationLevel(slangRequest, SLANG_OPTIMIZATION_LEVEL_NONE);
    //spSetDebugInfoLevel(slangRequest, SLANG_DEBUG_INFO_LEVEL_MAXIMAL);
    //spSetDumpIntermediates(slangRequest, true);
#endif

    const SlangResult compileRes = spCompile(slangRequest);

    if (auto diagnostics = spGetDiagnosticOutput(slangRequest))
    {
        printf("%s", diagnostics);
    }

    if (SLANG_FAILED(compileRes))
    {
        spDestroyCompileRequest(slangRequest);
        return false;
    }

    // Use reflection to get shader parameters
    auto reflection = (slang::ShaderReflection*)spGetReflection(slangRequest);
    uint32_t parameterCount = reflection->getParameterCount();
    shader_->parameters.clear();
    for (uint32_t i = 0; i != parameterCount; i++)
    {
        slang::VariableLayoutReflection* parameter = reflection->getParameterByIndex(i);
        const char* name = parameter->getName();
        const auto typeKind = (SlangTypeKind)parameter->getType()->getKind();
        const auto shape = parameter->getType()->getResourceShape();
        const auto access = parameter->getType()->getResourceAccess();

        shader_->parameters.push_back({ name, typeKind, shape, access });
    }

    // Get shader
    ISlangBlob* computeShaderBlob = nullptr;
    spGetEntryPointCodeBlob(slangRequest, computeIndex, 0, &computeShaderBlob);
    shader_->computeShader = { computeShaderBlob->getBufferPointer(), computeShaderBlob->getBufferSize() };

    spDestroyCompileRequest(slangRequest);

    return true;
}

}

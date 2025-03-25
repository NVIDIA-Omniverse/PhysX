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

#include "NvFlowArray.h"
#include "NvFlowPreprocessor.h"
#include "NvFlowProcess.h"
#include "NvFlowString.h"

#include <stdio.h>
#include <stdlib.h>

namespace
{

struct Context
{
    NvFlowStringPool* pool;
    NvFlowProcessList* processList = nullptr;
    const char* projectBase;
    const char* generatedBase;
    NvFlowArray<const char*> includeDirs;
};

char* includeDir(NvFlowPreprocessor* ptr, void* userdata, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens)
{
    Context* context = (Context*)userdata;

    NvFlowUint64 extractTokenIdx = 0u;
    NvFlowPreprocessorExtractDelimited(
        ptr, &extractTokenIdx, numTokens, tokens, eNvFlowPreprocessorTokenType_leftParenthesis);
    const char* args = NvFlowPreprocessorExtractDelimited(
        ptr, &extractTokenIdx, numTokens, tokens, eNvFlowPreprocessorTokenType_rightParenthesis);
    args = NvFlowStringTrimBeginAndEnd(context->pool, args, '\"');

    // auto append a / if missing
    NvFlowUint64 args_size = NvFlowStringLength(args);
    if (args_size > 0 && args[args_size - 1] != '/')
    {
        args = NvFlowStringConcat(context->pool, args, "/");
    }

    context->includeDirs.pushBack(args);

    return nullptr;
}

char* computeShader(NvFlowPreprocessor* ptr, void* userdata, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens)
{
    Context* context = (Context*)userdata;

    NvFlowUint64 extractTokenIdx = 0u;
    NvFlowPreprocessorExtractDelimited(
        ptr, &extractTokenIdx, numTokens, tokens, eNvFlowPreprocessorTokenType_leftParenthesis);
    const char* args = NvFlowPreprocessorExtractDelimited(
        ptr, &extractTokenIdx, numTokens, tokens, eNvFlowPreprocessorTokenType_rightParenthesis);
    args = NvFlowStringTrimBeginAndEnd(context->pool, args, '\"');

    const char* readFilePath = NvFlowStringConcat3(context->pool, context->projectBase, "/", args);
    const char* writeFilePath = NvFlowStringConcat3(context->pool, context->generatedBase, "/", args);

    char* writeDir = {};
    char* writeFile = {};
    NvFlowStringSplitDelimLast(context->pool, &writeDir, &writeFile, writeFilePath, '/');

    char* writeFileLessExtension = {};
    char* writeFileExtension = {};
    NvFlowStringSplitDelimLast(context->pool, &writeFileLessExtension, &writeFileExtension, writeFile, '.');

    writeFileLessExtension = NvFlowStringTrimEnd(context->pool, writeFileLessExtension, '.');

    const char* readText = NvFlowTextFileLoad(context->pool, readFilePath);

    // Save currently compiled version to be able to diff with tmp next time
    char* writePath_hlsl = NvFlowStringConcat3(context->pool, writeDir, writeFileLessExtension, ".hlsl");
    char* writePath_hlsl_tmp = NvFlowStringConcat3(context->pool, writeDir, writeFileLessExtension, ".tmp.hlsl");

    char* writePath_h = NvFlowStringConcat3(context->pool, writeDir, writeFile, ".h");

    NvFlowTextFileStore(readText, writePath_hlsl_tmp);

    if (NvFlowTextFileDiffAndWriteIfModified(writePath_hlsl, writePath_hlsl_tmp))
    {
        // Invalidate the old header file
        NvFlowTextFileRemove(writePath_h);

#if defined(_WIN32)
#    if defined(_DEBUG)
        const char* compiler = "../../../_build/windows-x86_64/debug/nvflowshadercompiler.exe";
#    else
        const char* compiler = "../../../_build/windows-x86_64/release/nvflowshadercompiler.exe";
#    endif
#elif defined(__aarch64__)
#    if defined(_DEBUG)
        const char* compiler = "../../../_build/linux-aarch64/debug/nvflowshadercompiler";
#    else
        const char* compiler = "../../../_build/linux-aarch64/release/nvflowshadercompiler";
#    endif
#else
#    if defined(_DEBUG)
        const char* compiler = "../../../_build/linux-x86_64/debug/nvflowshadercompiler";
#    else
        const char* compiler = "../../../_build/linux-x86_64/release/nvflowshadercompiler";
#    endif
#endif
        NvFlowArray<const char*> processArgs;
        processArgs.pushBack(compiler);
        processArgs.pushBack(readFilePath);
        processArgs.pushBack(writeFilePath);

        for (NvFlowUint i = 0; i < context->includeDirs.size; ++i)
        {
            processArgs.pushBack(context->includeDirs[i]);
        }

        NvFlowProcessListAdd(context->processList, (int)processArgs.size, processArgs.data);
    }

    return nullptr;
};

} // end local namespace

int main(int argc, char** argv)
{
    NvFlowStringPool* pool = NvFlowStringPoolCreate();

    const char* generatedPath = argc > 1 ? argv[1] : nullptr;

    char* projectPath = argc > 2 ? NvFlowStringDup(pool, argv[2]) : nullptr;

    char* projectDir = nullptr;
    char* projectFile = nullptr;
    NvFlowStringSplitDelimLast(pool, &projectDir, &projectFile, projectPath, '/');
    projectDir = NvFlowStringTrimEnd(pool, projectDir, '/');

    char* sourcePath = nullptr;
    char* projectName = nullptr;
    NvFlowStringSplitDelimLast(pool, &sourcePath, &projectName, projectDir, '/');
    sourcePath = NvFlowStringTrimEnd(pool, sourcePath, '/');

    char* generatedPathBase = NvFlowStringConcat3(pool, generatedPath, "/", projectName);
    char* generatedPathBaseShaders = NvFlowStringConcat(pool, generatedPathBase, "/shaders");
    NvFlowProcessMkdirRecursive(generatedPathBaseShaders);

    const char* projectText = NvFlowTextFileLoad(pool, projectPath);
    NvFlowUint64 projectText_size = NvFlowStringLength(projectText);
    if (projectText_size > 0)
    {
        NvFlowPreprocessor* preproc = NvFlowPreprocessorCreate(pool);

        // extract base directory from project path
        char* projectBase = nullptr;
        char* projectFilename = nullptr;
        NvFlowStringSplitDelimLast(pool, &projectBase, &projectFilename, projectPath, '/');

        projectBase = NvFlowStringTrimEnd(pool, projectBase, '/');

        Context context = {};
        context.pool = pool;
        context.processList = NvFlowProcessListCreate();
        context.projectBase = projectBase;
        context.generatedBase = generatedPathBase;

        NvFlowPreprocessorFunction functions[2] = {
            { "includeDir", eNvFlowPreprocessorType_function, &context, includeDir, NV_FLOW_FALSE },
            { "computeShader", eNvFlowPreprocessorType_function, &context, computeShader, NV_FLOW_FALSE }
        };

        NvFlowPreprocessorAddFunctions(preproc, 2u, functions);

        NvFlowPreprocessorExecute(preproc, projectText);

        NvFlowProcessListFlush(context.processList);

        NvFlowProcessListDestroy(context.processList);
        NvFlowPreprocessorDestroy(preproc);
    }

    NvFlowStringPoolDestroy(pool);

    return 0;
}

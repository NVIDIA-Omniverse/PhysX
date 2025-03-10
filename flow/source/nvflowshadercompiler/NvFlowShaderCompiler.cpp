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
#include "NvFlowString.h"
#include "ShaderPreprocessor.h"

#include <stdio.h>

int main(int argc, char** argv)
{
    NvFlowStringPool* pool = NvFlowStringPoolCreate();

    const char* readFilePath = argc > 1 ? argv[1] : nullptr;

    char* writeFilePath = argc > 2 ? NvFlowStringDup(pool, argv[2]) : nullptr;

    NvFlowArray<const char*> includeDirs;
    if (argc > 3)
    {
        for (int i = 3; i < argc; ++i)
        {
            includeDirs.pushBack(argv[i]);
        }
    }

    NvFlowPreprocessor* preproc = NvFlowPreprocessorCreate(pool);
    NvFlowShaderPreprocessor* shaderPreproc = NvFlowShaderPreprocessorCreate();

    NvFlowShaderPreprocessorBuildParams buildParams = {};
    buildParams.readPath = readFilePath;
    buildParams.writePath = writeFilePath;
    buildParams.numIncludePaths = includeDirs.size;
    buildParams.includePaths = includeDirs.data;

    const char* message = NvFlowStringPrint(pool, "Building %s\n", buildParams.readPath);
    printf("%s", message);

    NvFlowShaderPreprocessorBuildShader(shaderPreproc, &buildParams);

    NvFlowShaderPreprocessorDestroy(shaderPreproc);

    NvFlowStringPoolDestroy(pool);

    return 0;
}

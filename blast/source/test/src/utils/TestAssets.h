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
//
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.


#ifndef TESTASSETS_H
#define TESTASSETS_H

#include "NvBlast.h"
#include "AssetGenerator.h"

struct ExpectedAssetValues
{
    uint32_t    totalChunkCount;
    uint32_t    graphNodeCount;
    uint32_t    leafChunkCount;
    uint32_t    bondCount;
    uint32_t    subsupportChunkCount;
};


// Indexable asset descriptors and expected values
extern const NvBlastAssetDesc g_assetDescs[6];
extern const ExpectedAssetValues g_assetExpectedValues[6];

// Indexable asset descriptors for assets missing coverage and expected values
extern const NvBlastAssetDesc g_assetDescsMissingCoverage[6];
extern const ExpectedAssetValues g_assetsFromMissingCoverageExpectedValues[6];


inline uint32_t getAssetDescCount()
{
    return sizeof(g_assetDescs) / sizeof(g_assetDescs[0]);
}

inline uint32_t getAssetDescMissingCoverageCount()
{
    return sizeof(g_assetDescsMissingCoverage) / sizeof(g_assetDescsMissingCoverage[0]);
}


void generateCube(GeneratorAsset& cubeAsset, NvBlastAssetDesc& assetDesc, size_t maxDepth, size_t width, 
    int32_t supportDepth = -1, CubeAssetGenerator::BondFlags bondFlags = CubeAssetGenerator::ALL_INTERNAL_BONDS);

void generateRandomCube(GeneratorAsset& cubeAsset, NvBlastAssetDesc& assetDesc, uint32_t minChunkCount, uint32_t maxChunkCount);

#endif // #ifdef TESTASSETS_H

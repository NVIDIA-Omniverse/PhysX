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


#include "NvBlastAsset.h"
#include "NvBlastMath.h"

#include "BlastBaseTest.h"

#include "NvBlastTkFramework.h"

#include <random>
#include <algorithm>


// all supported platform now provide serialization
// keep the define for future platforms that won't
#define ENABLE_SERIALIZATION_TESTS 1

#pragma warning( push )
#pragma warning( disable : 4267 )
// NOTE: Instead of excluding serialization and the tests when on VC12, should break the tests out into a separate C++ file.

#if ENABLE_SERIALIZATION_TESTS
#include "NvBlastExtSerialization.h"
#include "NvBlastExtLlSerialization.h"
#include "NvBlastExtSerializationInternal.h"
#endif

#include "NvBlastExtAssetUtils.h"

#pragma warning( pop )

#include <fstream>
#include <iosfwd>

#ifdef WIN32
#include <windows.h>
#endif

template<int FailLevel, int Verbosity>
class AssetTest : public BlastBaseTest<FailLevel, Verbosity>
{
public:

    AssetTest()
    {
        NvBlastTkFrameworkCreate();
    }

    ~AssetTest()
    {
        NvBlastTkFrameworkGet()->release();
    }

    static void messageLog(int type, const char* msg, const char* file, int line)
    {
        BlastBaseTest<FailLevel, Verbosity>::messageLog(type, msg, file, line);
    }

    static void* alloc(size_t size)
    {
        return BlastBaseTest<FailLevel, Verbosity>::alignedZeroedAlloc(size);
    }

    static void free(void* mem)
    {
        BlastBaseTest<FailLevel, Verbosity>::alignedFree(mem);
    }

    void testSubtreeLeafChunkCounts(const Nv::Blast::Asset& a)
    {
        const NvBlastChunk* chunks = a.getChunks();
        const uint32_t* subtreeLeafChunkCounts = a.getSubtreeLeafChunkCounts();
        uint32_t totalLeafChunkCount = 0;
        for (uint32_t chunkIndex = 0; chunkIndex < a.m_chunkCount; ++chunkIndex)
        {
            const NvBlastChunk& chunk = chunks[chunkIndex];
            if (Nv::Blast::isInvalidIndex(chunk.parentChunkIndex))
            {
                totalLeafChunkCount += subtreeLeafChunkCounts[chunkIndex];
            }
            const bool isLeafChunk = chunk.firstChildIndex >= chunk.childIndexStop;
            uint32_t subtreeLeafChunkCount = isLeafChunk ? 1 : 0;
            for (uint32_t childIndex = chunk.firstChildIndex; childIndex < chunk.childIndexStop; ++childIndex)
            {
                subtreeLeafChunkCount += subtreeLeafChunkCounts[childIndex];
            }
            EXPECT_EQ(subtreeLeafChunkCount, subtreeLeafChunkCounts[chunkIndex]);
        }
        EXPECT_EQ(totalLeafChunkCount, a.m_leafChunkCount);
    }

    void testChunkToNodeMap(const Nv::Blast::Asset& a)
    {
        for (uint32_t chunkIndex = 0; chunkIndex < a.m_chunkCount; ++chunkIndex)
        {
            const uint32_t nodeIndex = a.getChunkToGraphNodeMap()[chunkIndex];
            if (!Nv::Blast::isInvalidIndex(nodeIndex))
            {
                EXPECT_LT(nodeIndex, a.m_graph.m_nodeCount);
                EXPECT_EQ(chunkIndex, a.m_graph.getChunkIndices()[nodeIndex]);
            }
            else
            {
                const uint32_t* chunkIndexStop = a.m_graph.getChunkIndices() + a.m_graph.m_nodeCount;
                const uint32_t* it = std::find<const uint32_t*, uint32_t>(a.m_graph.getChunkIndices(), chunkIndexStop, chunkIndex);
                EXPECT_EQ(chunkIndexStop, it);
            }
        }
    }

    NvBlastAsset* buildAsset(const ExpectedAssetValues& expected, const NvBlastAssetDesc* desc)
    {
        std::vector<char> scratch;
        scratch.resize((size_t)NvBlastGetRequiredScratchForCreateAsset(desc, messageLog));
        void* mem = alloc(NvBlastGetAssetMemorySize(desc, messageLog));
        NvBlastAsset* asset = NvBlastCreateAsset(mem, desc, scratch.data(), messageLog);
        EXPECT_TRUE(asset != nullptr);
        if (asset == nullptr)
        {
            free(mem);
            return nullptr;
        }
        Nv::Blast::Asset& a = *(Nv::Blast::Asset*)asset;
        EXPECT_EQ(expected.totalChunkCount, a.m_chunkCount);
        EXPECT_EQ(expected.graphNodeCount, a.m_graph.m_nodeCount);
        EXPECT_EQ(expected.bondCount, a.m_graph.getAdjacencyPartition()[a.m_graph.m_nodeCount] / 2);
        EXPECT_EQ(expected.leafChunkCount, a.m_leafChunkCount);
        EXPECT_EQ(expected.subsupportChunkCount, a.m_chunkCount - a.m_firstSubsupportChunkIndex);
        testSubtreeLeafChunkCounts(a);
        testChunkToNodeMap(a);
        return asset;
    }

    void checkAssetsExpected(Nv::Blast::Asset& asset, const ExpectedAssetValues& expected)
    {
        EXPECT_EQ(expected.totalChunkCount, asset.m_chunkCount);
        EXPECT_EQ(expected.graphNodeCount, asset.m_graph.m_nodeCount);
        EXPECT_EQ(expected.bondCount, asset.m_graph.getAdjacencyPartition()[asset.m_graph.m_nodeCount] / 2);
        EXPECT_EQ(expected.leafChunkCount, asset.m_leafChunkCount);
        EXPECT_EQ(expected.subsupportChunkCount, asset.m_chunkCount - asset.m_firstSubsupportChunkIndex);
        testSubtreeLeafChunkCounts(asset);
        testChunkToNodeMap(asset);
    }

    // expects that the bond normal points from the lower indexed chunk to higher index chunk
    // uses chunk.centroid
    // convention, requirement from findClosestNode
    void checkNormalDir(NvBlastChunkDesc* chunkDescs, size_t chunkDescCount, NvBlastBondDesc* bondDescs, size_t bondDescCount)
    {
        for (size_t bondIndex = 0; bondIndex < bondDescCount; ++bondIndex)
        {
            NvBlastBondDesc& bond = bondDescs[bondIndex];
            uint32_t chunkIndex0 = bond.chunkIndices[0];
            uint32_t chunkIndex1 = bond.chunkIndices[1];

            bool swap = chunkIndex0 > chunkIndex1;
            uint32_t testIndex0 = swap ? chunkIndex1 : chunkIndex0;
            uint32_t testIndex1 = swap ? chunkIndex0 : chunkIndex1;

            EXPECT_TRUE(testIndex0 < testIndex1);

            // no convention for world chunks
            if (!Nv::Blast::isInvalidIndex(testIndex0) && !Nv::Blast::isInvalidIndex(testIndex1))
            {
                NvBlastChunkDesc& chunk0 = chunkDescs[testIndex0];
                NvBlastChunkDesc& chunk1 = chunkDescs[testIndex1];

                float dir[3];
                Nv::Blast::VecMath::sub(chunk1.centroid, chunk0.centroid, dir);
                bool meetsConvention = Nv::Blast::VecMath::dot(bond.bond.normal, dir) > 0;
                EXPECT_TRUE(meetsConvention);
                if (!meetsConvention)
                {
                    printf("bond %zd chunks(%d,%d):   %.2f %.2f %.2f   %.2f %.2f %.2f   %d\n",
                        bondIndex, chunkIndex0, chunkIndex1,
                        bond.bond.normal[0], bond.bond.normal[1], bond.bond.normal[2],
                        dir[0], dir[1], dir[2],
                        Nv::Blast::VecMath::dot(bond.bond.normal, dir) > 0);
                }
            }
        }
    }

    // expects that the bond normal points from the lower indexed node to higher index node
    // uses chunk.centroid
    // convention, requirement from findClosestNode
    void checkNormalDir(const NvBlastSupportGraph graph, const NvBlastChunk* assetChunks, const NvBlastBond* assetBonds)
    {
        for (uint32_t nodeIndex = 0; nodeIndex < graph.nodeCount; nodeIndex++)
        {
            uint32_t adjStart = graph.adjacencyPartition[nodeIndex];
            uint32_t adjStop = graph.adjacencyPartition[nodeIndex + 1];
            for (uint32_t adj = adjStart; adj < adjStop; ++adj)
            {
                uint32_t adjNodeIndex = graph.adjacentNodeIndices[adj];

                bool swap = nodeIndex > adjNodeIndex;
                uint32_t testIndex0 = swap ? adjNodeIndex : nodeIndex;
                uint32_t testIndex1 = swap ? nodeIndex : adjNodeIndex;

                // no convention for world chunks
                if (!Nv::Blast::isInvalidIndex(graph.chunkIndices[testIndex0]) && !Nv::Blast::isInvalidIndex(graph.chunkIndices[testIndex1]))
                {
                    const NvBlastChunk& chunk0 = assetChunks[graph.chunkIndices[testIndex0]];
                    const NvBlastChunk& chunk1 = assetChunks[graph.chunkIndices[testIndex1]];

                    uint32_t bondIndex = graph.adjacentBondIndices[adj];
                    const NvBlastBond& bond = assetBonds[bondIndex];

                    float dir[3];
                    Nv::Blast::VecMath::sub(chunk1.centroid, chunk0.centroid, dir);
                    bool meetsConvention = Nv::Blast::VecMath::dot(bond.normal, dir) > 0;
                    EXPECT_TRUE(meetsConvention);
                    if (!meetsConvention)
                    {
                        printf("bond %d nodes(%d,%d):   %.2f %.2f %.2f   %.2f %.2f %.2f   %d\n",
                            bondIndex, nodeIndex, adjNodeIndex,
                            bond.normal[0], bond.normal[1], bond.normal[2],
                            dir[0], dir[1], dir[2],
                            Nv::Blast::VecMath::dot(bond.normal, dir) > 0);
                    }
                }
            }
        }
    }

    void checkNormalDir(const NvBlastAsset* asset)
    {
        const NvBlastChunk* assetChunks = NvBlastAssetGetChunks(asset, nullptr);
        const NvBlastBond* assetBonds = NvBlastAssetGetBonds(asset, nullptr);
        const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(asset, nullptr);
        checkNormalDir(graph, assetChunks, assetBonds);
    }

    void buildAssetShufflingDescriptors(const NvBlastAssetDesc* desc, const ExpectedAssetValues& expected, uint32_t shuffleCount, bool useTk)
    {
        NvBlastAssetDesc shuffledDesc = *desc;
        std::vector<NvBlastChunkDesc> chunkDescs(desc->chunkDescs, desc->chunkDescs + desc->chunkCount);
        shuffledDesc.chunkDescs = chunkDescs.data();
        std::vector<NvBlastBondDesc> bondDescs(desc->bondDescs, desc->bondDescs + desc->bondCount);
        shuffledDesc.bondDescs = bondDescs.data();
        if (!useTk)
        {
            std::vector<char> scratch(desc->chunkCount);
            NvBlastEnsureAssetExactSupportCoverage(chunkDescs.data(), desc->chunkCount, scratch.data(), messageLog);
        }
        else
        {
            NvBlastTkFrameworkGet()->ensureAssetExactSupportCoverage(chunkDescs.data(), desc->chunkCount);
        }
        for (uint32_t i = 0; i < shuffleCount; ++i)
        {
            checkNormalDir(chunkDescs.data(), chunkDescs.size(), bondDescs.data(), bondDescs.size());
            shuffleAndFixChunkDescs(chunkDescs.data(), desc->chunkCount, bondDescs.data(), desc->bondCount, useTk);
            checkNormalDir(chunkDescs.data(), chunkDescs.size(), bondDescs.data(), bondDescs.size());

            NvBlastAsset* asset = buildAsset(expected, &shuffledDesc);
            EXPECT_TRUE(asset != nullptr);
            checkNormalDir(asset);
            if (asset)
            {
                free(asset);
            }
        }
    }

    void shuffleAndFixChunkDescs(NvBlastChunkDesc* chunkDescs, uint32_t chunkDescCount, NvBlastBondDesc* bondDescs, uint32_t bondDescCount, bool useTk)
    {
        // Create reorder array and fill with identity map
        std::vector<uint32_t> shuffledOrder(chunkDescCount);
        for (uint32_t i = 0; i < chunkDescCount; ++i)
        {
            shuffledOrder[i] = i;
        }

        // An array into which to copy the reordered descs
        std::vector<NvBlastChunkDesc> shuffledChunkDescs(chunkDescCount);

        std::random_device rd;
        std::mt19937 g(rd());

        std::vector<char> scratch;
        const uint32_t trials = 30;
        uint32_t attempt = 0;
        while(1)
        {
            // Shuffle the reorder array
            std::shuffle(shuffledOrder.begin(), shuffledOrder.end(), g);

            // Save initial bonds
            std::vector<NvBlastBondDesc> savedBondDescs(bondDescs, bondDescs + bondDescCount);

            // Shuffle chunks and bonds
            NvBlastApplyAssetDescChunkReorderMap(shuffledChunkDescs.data(), chunkDescs, chunkDescCount, bondDescs, bondDescCount, shuffledOrder.data(), true, nullptr);

            // All the normals are pointing in the expected direction (they have been swapped)
            checkNormalDir(shuffledChunkDescs.data(), chunkDescCount, bondDescs, bondDescCount);
            checkNormalDir(chunkDescs, chunkDescCount, savedBondDescs.data(), bondDescCount);

            // Check the results
            for (uint32_t i = 0; i < chunkDescCount; ++i)
            {
                EXPECT_EQ(chunkDescs[i].userData, shuffledChunkDescs[shuffledOrder[i]].userData);
                EXPECT_TRUE(chunkDescs[i].parentChunkDescIndex > chunkDescCount || shuffledChunkDescs[shuffledOrder[i]].parentChunkDescIndex == shuffledOrder[chunkDescs[i].parentChunkDescIndex]);
            }
            for (uint32_t i = 0; i < bondDescCount; ++i)
            {
                for (uint32_t k = 0; k < 2; ++k)
                {
                    if (!Nv::Blast::isInvalidIndex(savedBondDescs[i].chunkIndices[k]))
                    {
                        EXPECT_EQ(shuffledOrder[savedBondDescs[i].chunkIndices[k]], bondDescs[i].chunkIndices[k]);
                    }
                }
            }

            // Try creating asset, usually it should fail (otherwise make another attempt)
            NvBlastAssetDesc desc = { chunkDescCount, shuffledChunkDescs.data(), bondDescCount, bondDescs };
            scratch.resize((size_t)NvBlastGetRequiredScratchForCreateAsset(&desc, nullptr));
            void* mem = alloc(NvBlastGetAssetMemorySize(&desc, nullptr));
            NvBlastAsset* asset = NvBlastCreateAsset(mem, &desc, scratch.data(), nullptr);
            if (asset == nullptr)
            {
                free(mem);
                break;
            }
            else
            {
                free(asset);
                memcpy(bondDescs, savedBondDescs.data(), sizeof(NvBlastBondDesc) * bondDescCount);
                attempt++;
                if (attempt >= trials)
                {
                    GTEST_NONFATAL_FAILURE_("Shuffled chunk descs should fail asset creation (most of the time).");
                    break;
                }
            }
        }

        // Now we want to fix that order
        if (!useTk)
        {
            std::vector<uint32_t> chunkReorderMap(chunkDescCount);
            std::vector<char> scratch2(3 * chunkDescCount * sizeof(uint32_t));
            const bool isIdentity = NvBlastBuildAssetDescChunkReorderMap(chunkReorderMap.data(), shuffledChunkDescs.data(), chunkDescCount, scratch2.data(), messageLog);
            EXPECT_FALSE(isIdentity);
            NvBlastApplyAssetDescChunkReorderMap(chunkDescs, shuffledChunkDescs.data(), chunkDescCount, bondDescs, bondDescCount, chunkReorderMap.data(), true, messageLog);
        }
        else
        {
            memcpy(chunkDescs, shuffledChunkDescs.data(), chunkDescCount * sizeof(NvBlastChunkDesc));
            const bool isIdentity = NvBlastTkFrameworkGet()->reorderAssetDescChunks(chunkDescs, chunkDescCount, bondDescs, bondDescCount, nullptr, true);
            EXPECT_FALSE(isIdentity);
        }
    }

    void mergeAssetTest(const NvBlastAssetDesc& desc, bool fail)
    {
        std::vector<char> scratch;

        scratch.resize((size_t)NvBlastGetRequiredScratchForCreateAsset(&desc, messageLog));
        void* mem = NVBLAST_ALLOC(NvBlastGetAssetMemorySize(&desc, messageLog));
        NvBlastAsset* asset = NvBlastCreateAsset(mem, &desc, scratch.data(), messageLog);
        EXPECT_TRUE(asset != nullptr);
        if (asset == nullptr)
        {
            free(mem);
            return;
        }

        // Merge two copies of this asset together
        const NvBlastAsset* components[2] = { asset, asset };
        const NvcVec3 translations[2] = { { 0, 0, 0 },{ 2, 0, 0 } };

        const NvBlastBond bond = { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 };

        NvBlastExtAssetUtilsBondDesc newBondDescs[4];
        for (int i = 0; i < 4; ++i)
        {
            newBondDescs[i].bond = bond;
            newBondDescs[i].chunkIndices[0] = 2 * (i + 1);
            newBondDescs[i].chunkIndices[1] = 2 * i + 1;
            newBondDescs[i].componentIndices[0] = 0;
            newBondDescs[i].componentIndices[1] = 1;
        }

        // Create a merged descriptor
        std::vector<uint32_t> chunkIndexOffsets(2);
        std::vector<uint32_t> chunkReorderMap(2 * desc.chunkCount);

        NvBlastAssetDesc mergedDesc = NvBlastExtAssetUtilsMergeAssets(components, nullptr, nullptr, translations, 2, newBondDescs, 4, chunkIndexOffsets.data(), chunkReorderMap.data(), 2 * desc.chunkCount);
        EXPECT_EQ(2 * desc.bondCount + 4, mergedDesc.bondCount);
        EXPECT_EQ(2 * desc.chunkCount, mergedDesc.chunkCount);
        for (uint32_t i = 0; i < 2 * desc.chunkCount; ++i)
        {
            EXPECT_LT(chunkReorderMap[i], 2 * desc.chunkCount);
        }
        EXPECT_EQ(0, chunkIndexOffsets[0]);
        EXPECT_EQ(desc.chunkCount, chunkIndexOffsets[1]);

        scratch.resize((size_t)NvBlastGetRequiredScratchForCreateAsset(&mergedDesc, messageLog));
        mem = NVBLAST_ALLOC(NvBlastGetAssetMemorySize(&mergedDesc, messageLog));
        NvBlastAsset* mergedAsset = NvBlastCreateAsset(mem, &mergedDesc, scratch.data(), messageLog);
        EXPECT_TRUE(mergedAsset != nullptr);
        if (mergedAsset == nullptr)
        {
            free(mem);
            return;
        }

        NVBLAST_FREE(const_cast<NvBlastBondDesc*>(mergedDesc.bondDescs));
        NVBLAST_FREE(const_cast<NvBlastChunkDesc*>(mergedDesc.chunkDescs));
        NVBLAST_FREE(mergedAsset);

        if (!fail)
        {
            mergedDesc = NvBlastExtAssetUtilsMergeAssets(components, nullptr, nullptr, translations, 2, newBondDescs, 4, nullptr, chunkReorderMap.data(), 2 * desc.chunkCount);
            EXPECT_EQ(2 * desc.bondCount + 4, mergedDesc.bondCount);
            EXPECT_EQ(2 * desc.chunkCount, mergedDesc.chunkCount);
            for (uint32_t i = 0; i < 2 * desc.chunkCount; ++i)
            {
                EXPECT_LT(chunkReorderMap[i], 2 * desc.chunkCount);
            }
            scratch.resize((size_t)NvBlastGetRequiredScratchForCreateAsset(&mergedDesc, messageLog));
            mem = NVBLAST_ALLOC(NvBlastGetAssetMemorySize(&mergedDesc, messageLog));
            mergedAsset = NvBlastCreateAsset(mem, &mergedDesc, scratch.data(), messageLog);
            EXPECT_TRUE(mergedAsset != nullptr);
            free(mem);
            NVBLAST_FREE(const_cast<NvBlastBondDesc*>(mergedDesc.bondDescs));
            NVBLAST_FREE(const_cast<NvBlastChunkDesc*>(mergedDesc.chunkDescs));
        }
        else
        {
            // We don't pass in a valid chunkReorderMap so asset creation should fail
            mergedDesc = NvBlastExtAssetUtilsMergeAssets(components, nullptr, nullptr, translations, 2, newBondDescs, 4, chunkIndexOffsets.data(), nullptr, 0);
            EXPECT_EQ(2 * desc.bondCount + 4, mergedDesc.bondCount);
            EXPECT_EQ(2 * desc.chunkCount, mergedDesc.chunkCount);
            EXPECT_EQ(0, chunkIndexOffsets[0]);
            EXPECT_EQ(desc.chunkCount, chunkIndexOffsets[1]);
            scratch.resize((size_t)NvBlastGetRequiredScratchForCreateAsset(&mergedDesc, messageLog));
            mem = NVBLAST_ALLOC(NvBlastGetAssetMemorySize(&mergedDesc, messageLog));
            mergedAsset = NvBlastCreateAsset(mem, &mergedDesc, scratch.data(), messageLog);
            EXPECT_TRUE(mergedAsset == nullptr);
            free(mem);
            NVBLAST_FREE(const_cast<NvBlastBondDesc*>(mergedDesc.bondDescs));
            NVBLAST_FREE(const_cast<NvBlastChunkDesc*>(mergedDesc.chunkDescs));

            mergedDesc = NvBlastExtAssetUtilsMergeAssets(components, nullptr, nullptr, translations, 2, newBondDescs, 4, nullptr, nullptr, 0);
            EXPECT_EQ(2 * desc.bondCount + 4, mergedDesc.bondCount);
            EXPECT_EQ(2 * desc.chunkCount, mergedDesc.chunkCount);
            scratch.resize((size_t)NvBlastGetRequiredScratchForCreateAsset(&mergedDesc, messageLog));
            mem = NVBLAST_ALLOC(NvBlastGetAssetMemorySize(&mergedDesc, messageLog));
            mergedAsset = NvBlastCreateAsset(mem, &mergedDesc, scratch.data(), messageLog);
            EXPECT_TRUE(mergedAsset == nullptr);
            free(mem);
            NVBLAST_FREE(const_cast<NvBlastBondDesc*>(mergedDesc.bondDescs));
            NVBLAST_FREE(const_cast<NvBlastChunkDesc*>(mergedDesc.chunkDescs));

            // We lie and say the chunkReorderMap is not large enough.  It should be filled with 0xFFFFFFFF up to the size we gave
            mergedDesc = NvBlastExtAssetUtilsMergeAssets(components, nullptr, nullptr, translations, 2, newBondDescs, 4, nullptr, chunkReorderMap.data(), desc.chunkCount);
            EXPECT_EQ(2 * desc.bondCount + 4, mergedDesc.bondCount);
            EXPECT_EQ(2 * desc.chunkCount, mergedDesc.chunkCount);
            for (uint32_t i = 0; i < desc.chunkCount; ++i)
            {
                EXPECT_TRUE(Nv::Blast::isInvalidIndex(chunkReorderMap[i]));
            }
            scratch.resize((size_t)NvBlastGetRequiredScratchForCreateAsset(&mergedDesc, messageLog));
            mem = NVBLAST_ALLOC(NvBlastGetAssetMemorySize(&mergedDesc, messageLog));
            mergedAsset = NvBlastCreateAsset(mem, &mergedDesc, scratch.data(), messageLog);
            EXPECT_TRUE(mergedAsset == nullptr);
            free(mem);
            NVBLAST_FREE(const_cast<NvBlastBondDesc*>(mergedDesc.bondDescs));
            NVBLAST_FREE(const_cast<NvBlastChunkDesc*>(mergedDesc.chunkDescs));

            mergedDesc = NvBlastExtAssetUtilsMergeAssets(components, nullptr, nullptr, translations, 2, newBondDescs, 4, chunkIndexOffsets.data(), chunkReorderMap.data(), desc.chunkCount);
            EXPECT_EQ(2 * desc.bondCount + 4, mergedDesc.bondCount);
            EXPECT_EQ(2 * desc.chunkCount, mergedDesc.chunkCount);
            for (uint32_t i = 0; i < desc.chunkCount; ++i)
            {
                EXPECT_TRUE(Nv::Blast::isInvalidIndex(chunkReorderMap[i]));
            }
            EXPECT_EQ(0, chunkIndexOffsets[0]);
            EXPECT_EQ(desc.chunkCount, chunkIndexOffsets[1]);
            scratch.resize((size_t)NvBlastGetRequiredScratchForCreateAsset(&mergedDesc, messageLog));
            mem = NVBLAST_ALLOC(NvBlastGetAssetMemorySize(&mergedDesc, messageLog));
            mergedAsset = NvBlastCreateAsset(mem, &mergedDesc, scratch.data(), messageLog);
            EXPECT_TRUE(mergedAsset == nullptr);
            free(mem);
            NVBLAST_FREE(const_cast<NvBlastBondDesc*>(mergedDesc.bondDescs));
            NVBLAST_FREE(const_cast<NvBlastChunkDesc*>(mergedDesc.chunkDescs));
        }

        // Finally free the original asset
        NVBLAST_FREE(asset);
    }
};

typedef AssetTest<-1, 0> AssetTestAllowErrorsSilently;
typedef AssetTest<NvBlastMessage::Error, 0> AssetTestAllowWarningsSilently;
typedef AssetTest<NvBlastMessage::Error, 1> AssetTestAllowWarnings;
typedef AssetTest<NvBlastMessage::Warning, 1> AssetTestStrict;


TEST_F(AssetTestStrict, BuildAssets)
{
    const uint32_t assetDescCount = sizeof(g_assetDescs) / sizeof(g_assetDescs[0]);

    std::vector<NvBlastAsset*> assets(assetDescCount);

    // Build
    for (uint32_t i = 0; i < assetDescCount; ++i)
    {
        assets[i] = buildAsset(g_assetExpectedValues[i], &g_assetDescs[i]);
    }

    // Destroy
    for (uint32_t i = 0; i < assetDescCount; ++i)
    {
        if (assets[i])
        {
            free(assets[i]);
        }
    }
}

#if ENABLE_SERIALIZATION_TESTS
TEST_F(AssetTestStrict, SerializeAssets)
{
    Nv::Blast::ExtSerialization* ser = NvBlastExtSerializationCreate();
    EXPECT_TRUE(ser != nullptr);

    const uint32_t assetDescCount = sizeof(g_assetDescs) / sizeof(g_assetDescs[0]);

    std::vector<Nv::Blast::Asset*> assets(assetDescCount);

    // Build
    for (uint32_t i = 0; i < assetDescCount; ++i)
    {
        assets[i] = reinterpret_cast<Nv::Blast::Asset*>(buildAsset(g_assetExpectedValues[i], &g_assetDescs[i]));
    }

    // Serialize them
    for (Nv::Blast::Asset* asset : assets)
    {
        void* buffer;
        const uint64_t size = NvBlastExtSerializationSerializeAssetIntoBuffer(buffer, *ser, asset);
        EXPECT_TRUE(size != 0);

        uint32_t objectTypeID;
        uint32_t encodingID;
        uint64_t dataSize = 0;
        EXPECT_TRUE(ser->peekHeader(&objectTypeID, &encodingID, &dataSize, buffer, size));
        EXPECT_EQ(objectTypeID, Nv::Blast::LlObjectTypeID::Asset);
        EXPECT_EQ(encodingID, ser->getSerializationEncoding());
        EXPECT_EQ(dataSize + Nv::Blast::ExtSerializationInternal::HeaderSize, size);
    }

    // Destroy
    for (uint32_t i = 0; i < assetDescCount; ++i)
    {
        if (assets[i])
        {
            free(assets[i]);
        }
    }

    ser->release();
}

TEST_F(AssetTestStrict, SerializeAssetsRoundTrip)
{
    Nv::Blast::ExtSerialization* ser = NvBlastExtSerializationCreate();
    EXPECT_TRUE(ser != nullptr);

    const uint32_t assetDescCount = sizeof(g_assetDescs) / sizeof(g_assetDescs[0]);

    std::vector<Nv::Blast::Asset*> assets(assetDescCount);

    // Build
    for (uint32_t i = 0; i < assetDescCount; ++i)
    {
        assets[i] = reinterpret_cast<Nv::Blast::Asset*>(buildAsset(g_assetExpectedValues[i], &g_assetDescs[i]));
    }

    const uint32_t encodings[] =
    {
        Nv::Blast::ExtSerialization::EncodingID::CapnProtoBinary,
        Nv::Blast::ExtSerialization::EncodingID::RawBinary
    };

    for (auto encoding : encodings)
    {
        ser->setSerializationEncoding(encoding);

        // Serialize them
        for (uint32_t i = 0; i < assetDescCount; ++i)
        {
            Nv::Blast::Asset* asset = assets[i];

            void* buffer;
            const uint64_t size = NvBlastExtSerializationSerializeAssetIntoBuffer(buffer, *ser, asset);
            EXPECT_TRUE(size != 0);

            Nv::Blast::Asset* rtAsset = reinterpret_cast<Nv::Blast::Asset*>(ser->deserializeFromBuffer(buffer, size));

            //TODO: Compare assets
            checkAssetsExpected(*rtAsset, g_assetExpectedValues[i]);

            free(static_cast<void*>(rtAsset));
        }
    }

    // Destroy
    for (uint32_t i = 0; i < assetDescCount; ++i)
    {
        if (assets[i])
        {
            free(assets[i]);
        }
    }

    ser->release();
}

TEST_F(AssetTestStrict, SerializeAssetsRoundTripWithSkipping)
{
    Nv::Blast::ExtSerialization* ser = NvBlastExtSerializationCreate();
    EXPECT_TRUE(ser != nullptr);

    std::vector<char> stream;

    class StreamBufferProvider : public Nv::Blast::ExtSerialization::BufferProvider
    {
    public:
        StreamBufferProvider(std::vector<char>& stream) : m_stream(stream), m_cursor(0) {}

        virtual void*   requestBuffer(size_t size) override
        {
            m_stream.resize(m_cursor + size);
            void* data = m_stream.data() + m_cursor;
            m_cursor += size;
            return data;
        }

    private:
        std::vector<char>&  m_stream;
        size_t              m_cursor;
    } myStreamProvider(stream);

    ser->setBufferProvider(&myStreamProvider);
    
    const uint32_t assetDescCount = sizeof(g_assetDescs) / sizeof(g_assetDescs[0]);

    std::vector<Nv::Blast::Asset*> assets(assetDescCount);

    // Build
    for (uint32_t i = 0; i < assetDescCount; ++i)
    {
        assets[i] = reinterpret_cast<Nv::Blast::Asset*>(buildAsset(g_assetExpectedValues[i], &g_assetDescs[i]));
    }

    const uint32_t encodings[] =
    {
        Nv::Blast::ExtSerialization::EncodingID::CapnProtoBinary,
        Nv::Blast::ExtSerialization::EncodingID::RawBinary
    };

    for (auto encoding : encodings)
    {
        ser->setSerializationEncoding(encoding);

        // Serialize them
        for (uint32_t i = 0; i < assetDescCount; ++i)
        {
            void* buffer;
            const uint64_t size = NvBlastExtSerializationSerializeAssetIntoBuffer(buffer, *ser, assets[i]);
            EXPECT_TRUE(size != 0);
        }
    }

    // Deserialize from stream
    const void* buffer = stream.data();
    uint64_t bufferSize = stream.size();
    for (uint32_t assetCount = 0; bufferSize; ++assetCount)
    {
        uint32_t objectTypeID;
        uint32_t encodingID;
        const bool peekSuccess = ser->peekHeader(&objectTypeID, &encodingID, nullptr, buffer, bufferSize);
        EXPECT_TRUE(peekSuccess);
        if (!peekSuccess)
        {
            break;
        }

        EXPECT_EQ(Nv::Blast::LlObjectTypeID::Asset, objectTypeID);
        if (assetCount < assetDescCount)
        {
            EXPECT_EQ(Nv::Blast::ExtSerialization::EncodingID::CapnProtoBinary, encodingID);
        }
        else
        {
            EXPECT_EQ(Nv::Blast::ExtSerialization::EncodingID::RawBinary, encodingID);
        }

        const bool skip = (assetCount & 1) != 0;

        if (!skip)
        {
            const uint32_t assetnum = assetCount % assetDescCount;
            Nv::Blast::Asset* rtAsset = reinterpret_cast<Nv::Blast::Asset*>(ser->deserializeFromBuffer(buffer, bufferSize));
            EXPECT_TRUE(rtAsset != nullptr);
            if (rtAsset == nullptr)
            {
                break;
            }

            //TODO: Compare assets
            checkAssetsExpected(*rtAsset, g_assetExpectedValues[assetnum]);

            free(static_cast<void*>(rtAsset));
        }

        buffer = ser->skipObject(bufferSize, buffer);
    }

    // Destroy
    for (uint32_t i = 0; i < assetDescCount; ++i)
    {
        if (assets[i])
        {
            free(assets[i]);
        }
    }

    ser->release();
}
#endif  // ENABLE_SERIALIZATION_TESTS

TEST_F(AssetTestAllowWarnings, BuildAssetsMissingCoverage)
{
    const uint32_t assetDescCount = sizeof(g_assetDescsMissingCoverage) / sizeof(g_assetDescsMissingCoverage[0]);

    std::vector<NvBlastAsset*> assets(assetDescCount);

    // Build
    for (uint32_t i = 0; i < assetDescCount; ++i)
    {
        const NvBlastAssetDesc* desc = &g_assetDescsMissingCoverage[i];
        NvBlastAssetDesc fixedDesc = *desc;
        std::vector<NvBlastChunkDesc> chunkDescs(desc->chunkDescs, desc->chunkDescs + desc->chunkCount);
        std::vector<NvBlastBondDesc> bondDescs(desc->bondDescs, desc->bondDescs + desc->bondCount);
        std::vector<uint32_t> chunkReorderMap(desc->chunkCount);
        std::vector<char> scratch(desc->chunkCount * sizeof(NvBlastChunkDesc));
        const bool changedCoverage = !NvBlastEnsureAssetExactSupportCoverage(chunkDescs.data(), fixedDesc.chunkCount, scratch.data(), messageLog);
        EXPECT_TRUE(changedCoverage);
        NvBlastReorderAssetDescChunks(chunkDescs.data(), fixedDesc.chunkCount, bondDescs.data(), fixedDesc.bondCount, chunkReorderMap.data(), true, scratch.data(), messageLog);
        fixedDesc.chunkDescs = chunkDescs.data();
        fixedDesc.bondDescs = bondDescs.data();
        assets[i] = buildAsset(g_assetsFromMissingCoverageExpectedValues[i], &fixedDesc);
    }

    // Destroy
    for (uint32_t i = 0; i < assetDescCount; ++i)
    {
        if (assets[i])
        {
            free(assets[i]);
        }
    }
}

TEST_F(AssetTestAllowWarningsSilently, BuildAssetsShufflingChunkDescriptors)
{
    for (uint32_t i = 0; i < sizeof(g_assetDescs) / sizeof(g_assetDescs[0]); ++i)
    {
        buildAssetShufflingDescriptors(&g_assetDescs[i], g_assetExpectedValues[i], 10, false);
    }

    for (uint32_t i = 0; i < sizeof(g_assetDescsMissingCoverage) / sizeof(g_assetDescsMissingCoverage[0]); ++i)
    {
        buildAssetShufflingDescriptors(&g_assetDescsMissingCoverage[i], g_assetsFromMissingCoverageExpectedValues[i], 10, false);
    }
}

TEST_F(AssetTestAllowWarningsSilently, BuildAssetsShufflingChunkDescriptorsUsingTk)
{
    for (uint32_t i = 0; i < sizeof(g_assetDescs) / sizeof(g_assetDescs[0]); ++i)
    {
        buildAssetShufflingDescriptors(&g_assetDescs[i], g_assetExpectedValues[i], 10, true);
    }

    for (uint32_t i = 0; i < sizeof(g_assetDescsMissingCoverage) / sizeof(g_assetDescsMissingCoverage[0]); ++i)
    {
        buildAssetShufflingDescriptors(&g_assetDescsMissingCoverage[i], g_assetsFromMissingCoverageExpectedValues[i], 10, true);
    }
}

TEST_F(AssetTestStrict, MergeAssetsUpperSupportOnly)
{
    mergeAssetTest(g_assetDescs[0], false);
}

TEST_F(AssetTestStrict, MergeAssetsWithSubsupport)
{
    mergeAssetTest(g_assetDescs[1], false);
}

TEST_F(AssetTestStrict, MergeAssetsWithWorldBondsUpperSupportOnly)
{
    mergeAssetTest(g_assetDescs[3], false);
}

TEST_F(AssetTestStrict, MergeAssetsWithWorldBondsWithSubsupport)
{
    mergeAssetTest(g_assetDescs[4], false);
}

TEST_F(AssetTestAllowErrorsSilently, MergeAssetsUpperSupportOnlyExpectFail)
{
    mergeAssetTest(g_assetDescs[0], true);
}

TEST_F(AssetTestAllowErrorsSilently, MergeAssetsWithSubsupportExpectFail)
{
    mergeAssetTest(g_assetDescs[1], true);
}

TEST_F(AssetTestAllowErrorsSilently, MergeAssetsWithWorldBondsUpperSupportOnlyExpectFail)
{
    mergeAssetTest(g_assetDescs[3], true);
}

TEST_F(AssetTestAllowErrorsSilently, MergeAssetsWithWorldBondsWithSubsupportExpectFail)
{
    mergeAssetTest(g_assetDescs[4], true);
}

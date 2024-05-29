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


#include <algorithm>
#include "gtest/gtest.h"

//#include "NvBlast.h"
#include "NvBlastActor.h"
#include "NvBlastIndexFns.h"

#include "NvBlastGlobals.h"

#include "TestAssets.h"
#include "NvBlastActor.h"

static void messageLog(int type, const char* msg, const char* file, int line)
{
    {
        switch (type)
        {
        case NvBlastMessage::Error: std::cout << "NvBlast Error message in " << file << "(" << line << "): " << msg << "\n";    break;
        case NvBlastMessage::Warning:   std::cout << "NvBlast Warning message in " << file << "(" << line << "): " << msg << "\n";  break;
        case NvBlastMessage::Info:  std::cout << "NvBlast Info message in " << file << "(" << line << "): " << msg << "\n";     break;
        case NvBlastMessage::Debug: std::cout << "NvBlast Debug message in " << file << "(" << line << "): " << msg << "\n";    break;
        }
    }
}

TEST(CoreTests, IndexStartLookup)
{
    uint32_t lookup[32];
    uint32_t indices[] = {1,1,2,2,4,4,4};

    Nv::Blast::createIndexStartLookup<uint32_t>(lookup, 0, 30, indices, 7, 4);

    EXPECT_EQ(lookup[0], 0);
    EXPECT_EQ(lookup[1], 0);
    EXPECT_EQ(lookup[2], 2);
    EXPECT_EQ(lookup[3], 4);
    EXPECT_EQ(lookup[4], 4);
    EXPECT_EQ(lookup[5], 7);
    EXPECT_EQ(lookup[31], 7);
}

#include "NvBlastGeometry.h"

int findClosestNodeByBonds(const float point[4], const NvBlastActor* actor)
{
    const Nv::Blast::Actor* a = static_cast<const Nv::Blast::Actor*>(actor);
    const NvBlastFamily* family = NvBlastActorGetFamily(actor, messageLog);
    const NvBlastAsset* asset = NvBlastFamilyGetAsset(family, messageLog);
    const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(asset, messageLog);
    return Nv::Blast::findClosestNode(
        point,
        a->getFirstGraphNodeIndex(),
        a->getFamilyHeader()->getGraphNodeIndexLinks(),
        graph.adjacencyPartition,
        graph.adjacentNodeIndices,
        graph.adjacentBondIndices,
        NvBlastAssetGetBonds(asset, messageLog),
        NvBlastActorGetBondHealths(actor, messageLog),
        graph.chunkIndices
        );
}

int findClosestNodeByChunks(const float point[4], const NvBlastActor* actor)
{
    const Nv::Blast::Actor* a = static_cast<const Nv::Blast::Actor*>(actor);
    return Nv::Blast::findClosestNode(
        point,
        a->getFirstGraphNodeIndex(),
        a->getFamilyHeader()->getGraphNodeIndexLinks(),
        a->getAsset()->m_graph.getAdjacencyPartition(),
        a->getAsset()->m_graph.getAdjacentNodeIndices(),
        a->getAsset()->m_graph.getAdjacentBondIndices(),
        a->getAsset()->getBonds(),
        a->getFamilyHeader()->getBondHealths(),
        a->getAsset()->getChunks(),
        a->getFamilyHeader()->getLowerSupportChunkHealths(),
        a->getAsset()->m_graph.getChunkIndices()
        );
}

TEST(CoreTests, FindChunkByPosition)
{
    std::vector<char> scratch;
    const NvBlastAssetDesc& desc = g_assetDescs[0]; // 1-cube
    scratch.resize((size_t)NvBlastGetRequiredScratchForCreateAsset(&desc, nullptr));
    void* amem = NVBLAST_ALLOC(NvBlastGetAssetMemorySize(&desc, nullptr));
    NvBlastAsset* asset = NvBlastCreateAsset(amem, &desc, scratch.data(), nullptr);
    ASSERT_TRUE(asset != nullptr);

    uint32_t expectedNode[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
    const float positions[] = {
        -2.0f, -2.0f, -2.0f,
        +2.0f, -2.0f, -2.0f,
        -2.0f, +2.0f, -2.0f,
        +2.0f, +2.0f, -2.0f,
        -2.0f, -2.0f, +2.0f,
        +2.0f, -2.0f, +2.0f,
        -2.0f, +2.0f, +2.0f,
        +2.0f, +2.0f, +2.0f,
    };
    const float* pos = &positions[0];

    NvBlastActorDesc actorDesc;
    actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
    actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
    void* fmem = NVBLAST_ALLOC(NvBlastAssetGetFamilyMemorySize(asset, nullptr));
    NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, nullptr);
    scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, nullptr));
    NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), nullptr);
    ASSERT_TRUE(actor != nullptr);

    for (int i = 0; i < 8; ++i, pos += 3)
    {
        EXPECT_EQ(expectedNode[i], findClosestNodeByBonds(pos, actor));
        EXPECT_EQ(expectedNode[i], findClosestNodeByChunks(pos, actor));    
    }

    EXPECT_TRUE(NvBlastActorDeactivate(actor, nullptr));
    NVBLAST_FREE(family);
    NVBLAST_FREE(asset);
}

TEST(CoreTests, FindChunkByPositionUShape)
{
    /*
    considering this graph

    4->5->6
    ^
    |
    1->2->3

    and trying to find chunks by some position
    */
    const NvBlastChunkDesc uchunks[7] =
    {
        // centroid           volume parent idx     flags                         ID
        { {3.0f, 2.0f, 0.0f}, 0.0f, UINT32_MAX, NvBlastChunkDesc::NoFlags, 0 },
        { {1.0f, 1.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 1 },
        { {3.0f, 1.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 2 },
        { {5.0f, 1.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 3 },
        { {1.0f, 3.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 4 },
        { {3.0f, 3.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 5 },
        { {5.0f, 3.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 6 }
    };

    const NvBlastBondDesc ubonds[5] =
    {
//          normal                area  centroid              userData chunks      
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 2.0f, 1.0f, 0.0f }, 0 }, { 2, 1 } }, // index swap should not matter
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 4.0f, 1.0f, 0.0f }, 0 }, { 2, 3 } },
        { { { 0.0f, 1.0f, 0.0f }, 1.0f, { 1.0f, 2.0f, 0.0f }, 0 }, { 1, 4 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 2.0f, 3.0f, 0.0f }, 0 }, { 4, 5 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 4.0f, 3.0f, 0.0f }, 0 }, { 5, 6 } },
    };

    const NvBlastAssetDesc desc = { 7, uchunks, 5, ubonds };
    std::vector<char> scratch;
    scratch.resize((size_t)NvBlastGetRequiredScratchForCreateAsset(&desc, messageLog));
    void* amem = NVBLAST_ALLOC(NvBlastGetAssetMemorySize(&desc, messageLog));
    NvBlastAsset* asset = NvBlastCreateAsset(amem, &desc, scratch.data(), messageLog);
    ASSERT_TRUE(asset != nullptr);

    NvBlastActorDesc actorDesc;
    actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
    actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
    void* fmem = NVBLAST_ALLOC(NvBlastAssetGetFamilyMemorySize(asset, messageLog));
    NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, nullptr);
    scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, messageLog));
    NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), nullptr);
    ASSERT_TRUE(actor != nullptr);

    srand(100);
    for (uint32_t i = 0; i < 100000; i++)
    {
        float rx = 20 * (float)(rand() - 1) / RAND_MAX - 10;
        float ry = 20 * (float)(rand() - 1) / RAND_MAX - 10;
        float rz = 0.0f;
        float rpos[] = { rx, ry, rz };

        // open boundaries
        uint32_t col = std::max(0, std::min(2, int(rx / 2)));
        uint32_t row = std::max(0, std::min(1, int(ry / 2)));
        uint32_t expectedNode = col + row * 3;

        //printf("iteration %i: %.1f %.1f %.1f expected: %d\n", i, rpos[0], rpos[1], rpos[2], expectedNode);
        {
            uint32_t returnedNode = findClosestNodeByBonds(rpos, actor);
            if (expectedNode != returnedNode)
                findClosestNodeByBonds(rpos, actor);
            EXPECT_EQ(expectedNode, returnedNode);
        }
        {
            uint32_t returnedNode = findClosestNodeByChunks(rpos, actor);
            if (expectedNode != returnedNode)
                findClosestNodeByChunks(rpos, actor);
            EXPECT_EQ(expectedNode, returnedNode);
        }

    }

    EXPECT_TRUE(NvBlastActorDeactivate(actor, messageLog));

    NVBLAST_FREE(family);
    NVBLAST_FREE(asset);
}

TEST(CoreTests, FindChunkByPositionLandlocked)
{
    // 7 > 8 > 9
    // ^   ^   ^
    // 4 > 5 > 6
    // ^   ^   ^
    // 1 > 2 > 3

    // chunk 5 (node 4) is broken out (landlocked)
    // find closest chunk/node on the two new actors

    const NvBlastChunkDesc chunks[10] =
    {
        // centroid           volume parent idx     flags                         ID
        { {0.0f, 0.0f, 0.0f}, 0.0f, UINT32_MAX, NvBlastChunkDesc::NoFlags, 0 },
        { {1.0f, 1.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 1 },
        { {3.0f, 1.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 2 },
        { {5.0f, 1.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 3 },
        { {1.0f, 3.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 4 },
        { {3.0f, 3.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 5 },
        { {5.0f, 3.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 6 },
        { {1.0f, 5.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 7 },
        { {3.0f, 5.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 8 },
        { {5.0f, 5.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 9 },
    };

    const NvBlastBondDesc bonds[12] =
    {
//          normal                area  centroid              userData chunks
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 2.0f, 1.0f, 0.0f }, 0 }, { 1, 2 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 4.0f, 1.0f, 0.0f }, 0 }, { 2, 3 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 2.0f, 3.0f, 0.0f }, 0 }, { 4, 5 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 4.0f, 3.0f, 0.0f }, 0 }, { 5, 6 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 2.0f, 5.0f, 0.0f }, 0 }, { 7, 8 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 4.0f, 5.0f, 0.0f }, 0 }, { 8, 9 } },
        { { { 0.0f, 1.0f, 0.0f }, 1.0f, { 1.0f, 2.0f, 0.0f }, 0 }, { 1, 4 } },
        { { { 0.0f, 1.0f, 0.0f }, 1.0f, { 3.0f, 2.0f, 0.0f }, 0 }, { 2, 5 } },
        { { { 0.0f, 1.0f, 0.0f }, 1.0f, { 5.0f, 2.0f, 0.0f }, 0 }, { 3, 6 } },
        { { { 0.0f, 1.0f, 0.0f }, 1.0f, { 1.0f, 4.0f, 0.0f }, 0 }, { 4, 7 } },
        { { { 0.0f, 1.0f, 0.0f }, 1.0f, { 3.0f, 4.0f, 0.0f }, 0 }, { 5, 8 } },
        { { { 0.0f, 1.0f, 0.0f }, 1.0f, { 5.0f, 4.0f, 0.0f }, 0 }, { 6, 9 } },
    };

    const NvBlastAssetDesc desc = { 10, chunks, 12, bonds };
    std::vector<char> scratch;
    scratch.resize((size_t)NvBlastGetRequiredScratchForCreateAsset(&desc, messageLog));
    void* amem = NVBLAST_ALLOC(NvBlastGetAssetMemorySize(&desc, messageLog));
    NvBlastAsset* asset = NvBlastCreateAsset(amem, &desc, scratch.data(), messageLog);
    ASSERT_TRUE(asset != nullptr);

    NvBlastActorDesc actorDesc;
    actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
    actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
    void* fmem = NVBLAST_ALLOC(NvBlastAssetGetFamilyMemorySize(asset, nullptr));
    NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, nullptr);
    scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, nullptr));
    NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), nullptr);
    ASSERT_TRUE(actor != nullptr);

    float point[4] = { 3.0f, 3.0f, 0.0f };
    EXPECT_EQ(4, findClosestNodeByChunks(point, actor));
    EXPECT_EQ(4, findClosestNodeByBonds(point, actor));

    NvBlastChunkFractureData chunkBuffer[1];
    NvBlastFractureBuffers events = { 0, 1, nullptr, chunkBuffer };

    NvBlastChunkFractureData chunkFracture = { 0, 5, 1.0f };
    NvBlastFractureBuffers commands = { 0, 1, nullptr, &chunkFracture };

    NvBlastActorApplyFracture(&events, actor, &commands, messageLog, nullptr);
    EXPECT_EQ(1, events.chunkFractureCount);

    NvBlastActor* newActors[5];
    NvBlastActorSplitEvent splitEvent = { nullptr, newActors };
    scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, messageLog));
    size_t newActorsCount = NvBlastActorSplit(&splitEvent, actor, 5, scratch.data(), messageLog, nullptr);

    ASSERT_EQ(actor, newActors[1]);

    EXPECT_NE(4, findClosestNodeByChunks(point, actor));
    EXPECT_NE(4, findClosestNodeByBonds(point, actor));

    float point2[4] = { 80.0f, 80.0f, 80.0f };
    EXPECT_EQ(4, findClosestNodeByChunks(point2, newActors[0]));
    EXPECT_EQ(4, findClosestNodeByBonds(point, newActors[0]));

    for (uint32_t i = 0; i < newActorsCount; ++i)
    {
        EXPECT_TRUE(NvBlastActorDeactivate(newActors[i], nullptr));
    }

    NVBLAST_FREE(family);
    NVBLAST_FREE(asset);
}

TEST(CoreTests, FindClosestByChunkAccuracy)
{
    //  (0,0) +---+-------+
    //        |   |   1   |
    //        | 2 +---+---+
    //        |   | 5 |   |
    //        +---+---+ 4 |
    //        |   3   |   |
    //        +-------+---+ (6,6)

    // random point lookup over the actor's space
    // tests would fail if findClosestNodeByChunks didn't improve accuracy with the help of bonds

    const NvBlastChunkDesc chunks[6] =
    {
        // centroid           volume parent idx     flags                         ID
        { { 0.0f, 0.0f, 0.0f }, 0.0f, UINT32_MAX,   NvBlastChunkDesc::NoFlags, 0 },
        { { 4.0f, 1.0f, 0.0f }, 0.0f, 0,            NvBlastChunkDesc::SupportFlag, 1 },
        { { 1.0f, 2.0f, 0.0f }, 0.0f, 0,            NvBlastChunkDesc::SupportFlag, 2 },
        { { 2.0f, 5.0f, 0.0f }, 0.0f, 0,            NvBlastChunkDesc::SupportFlag, 3 },
        { { 5.0f, 4.0f, 0.0f }, 0.0f, 0,            NvBlastChunkDesc::SupportFlag, 4 },
        { { 3.0f, 3.0f, 0.0f }, 0.0f, 0,            NvBlastChunkDesc::SupportFlag, 5 },
    };

    const NvBlastBondDesc bonds[8] =
    {
        //          normal        area  centroid          userData chunks
        { { { -1.0f,  0.0f, 0.0f }, 1.0f,{ 2.0f, 1.0f, 0.0f }, 0 },{ 1, 2 } },
        { { {  0.0f,  1.0f, 0.0f }, 1.0f,{ 5.0f, 2.0f, 0.0f }, 0 },{ 1, 4 } },
        { { {  0.0f,  1.0f, 0.0f }, 1.0f,{ 3.0f, 2.0f, 0.0f }, 0 },{ 5, 1 } },

        { { {  0.0f,  1.0f, 0.0f }, 1.0f,{ 1.0f, 4.0f, 0.0f }, 0 },{ 2, 3 } },
        { { {  1.0f,  0.0f, 0.0f }, 1.0f,{ 2.0f, 3.0f, 0.0f }, 0 },{ 2, 5 } },

        { { {  1.0f,  0.0f, 0.0f }, 1.0f,{ 4.0f, 5.0f, 0.0f }, 0 },{ 3, 4 } },
        { { {  0.0f, -1.0f, 0.0f }, 1.0f,{ 3.0f, 4.0f, 0.0f }, 0 },{ 3, 5 } },

        { { { -1.0f,  0.0f, 0.0f }, 1.0f,{ 4.0f, 3.0f, 0.0f }, 0 },{ 4, 5 } },
    };

    const NvBlastAssetDesc desc = { 6, chunks, 8, bonds };
    std::vector<char> scratch;
    scratch.resize((size_t)NvBlastGetRequiredScratchForCreateAsset(&desc, messageLog));
    void* amem = NVBLAST_ALLOC(NvBlastGetAssetMemorySize(&desc, messageLog));
    NvBlastAsset* asset = NvBlastCreateAsset(amem, &desc, scratch.data(), messageLog);
    ASSERT_TRUE(asset != nullptr);

    NvBlastActorDesc actorDesc;
    actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
    actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
    void* fmem = NVBLAST_ALLOC(NvBlastAssetGetFamilyMemorySize(asset, nullptr));
    NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, nullptr);
    scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, nullptr));
    NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), nullptr);
    ASSERT_TRUE(actor != nullptr);

    srand(0xb007);
    for (uint32_t i = 0; i < 100000; i++)
    {
        float rx = 8 * (float)(rand()) / RAND_MAX - 1;
        float ry = 8 * (float)(rand()) / RAND_MAX - 1;
        float rz = 0.0f;
        float rpos[] = { rx, ry, rz };

        EXPECT_LE(-1.0f, rx);   EXPECT_GE(7.0f, rx);
        EXPECT_LE(-1.0f, ry);   EXPECT_GE(7.0f, ry);

        uint32_t expectedNode = 0xdefec7;

        if (rx < 2.0f) {
            if (ry < 4.0f) { expectedNode = 1; }
            else { expectedNode = 2; }
        }
        else if (rx < 4.0f) {
            if (ry < 2.0f) { expectedNode = 0; }
            else if (ry < 4.0f) { expectedNode = 4; }
            else { expectedNode = 2; }
        }
        else {
            if (ry < 2.0f) { expectedNode = 0; }
            else { expectedNode = 3; }
        }

        uint32_t nodeByBonds = findClosestNodeByBonds(rpos, actor);
        if (nodeByBonds != expectedNode)
        {
            printf("%.1f %.1f %.1f\n", rx, ry, rz);
        }
        EXPECT_EQ(expectedNode, nodeByBonds);

        uint32_t nodeByChunks = findClosestNodeByChunks(rpos, actor);
        if (nodeByChunks != expectedNode)
        {
            printf("%.1f %.1f %.1f\n", rx, ry, rz);
        }
        EXPECT_EQ(expectedNode, nodeByChunks);
    }

    EXPECT_TRUE(NvBlastActorDeactivate(actor, messageLog));

    NVBLAST_FREE(family);
    NVBLAST_FREE(asset);
}

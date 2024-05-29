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


#include "BlastBaseTest.h"
#include "NvBlastIndexFns.h"
#include "NvBlastExtDamageShaders.h"

#include <algorithm>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  Utils / Tests Common
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

using namespace Nv::Blast;

class APITest : public BlastBaseTest < NvBlastMessage::Error, 1 >
{
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                      Tests
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(APITest, Basic)
{
    // create asset
    const NvBlastAssetDesc& assetDesc = g_assetDescs[0];

    std::vector<char> scratch((size_t)NvBlastGetRequiredScratchForCreateAsset(&assetDesc, messageLog));
    void* amem = alignedZeroedAlloc(NvBlastGetAssetMemorySize(&assetDesc, messageLog));
    NvBlastAsset* asset = NvBlastCreateAsset(amem, &assetDesc, scratch.data(), messageLog);
    EXPECT_TRUE(asset != nullptr);

    // create actor
    NvBlastActorDesc actorDesc;
    actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
    actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
    void* fmem = alignedZeroedAlloc(NvBlastAssetGetFamilyMemorySize(asset, messageLog));
    NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, messageLog);
    scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, messageLog));
    NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), messageLog);
    EXPECT_TRUE(actor != nullptr);

    NvBlastExtRadialDamageDesc damage = {
        10.0f,                  // compressive
        { 0.0f, 0.0f, 0.0f },   // position
        4.0f,                   // min radius - maximum damage
        6.0f                    // max radius - zero damage
    };

    NvBlastBondFractureData outFracture[12]; /*num lower-support chunks + bonds?*/

    NvBlastFractureBuffers events;
    events.bondFractureCount = 12;
    events.bondFractures = outFracture;
    events.chunkFractureCount = 0;
    events.chunkFractures = nullptr;

    NvBlastExtProgramParams programParams = { &damage, nullptr };

    NvBlastDamageProgram program = {
        NvBlastExtFalloffGraphShader,
        nullptr
    };

    NvBlastActorGenerateFracture(&events, actor, program, &programParams, messageLog, nullptr);
    NvBlastActorApplyFracture(&events, actor, &events, messageLog, nullptr);
    EXPECT_TRUE(NvBlastActorIsSplitRequired(actor, messageLog));
    EXPECT_EQ(12, events.bondFractureCount);

    NvBlastActor* newActors[8]; /* num lower-support chunks? plus space for deletedActor */
    NvBlastActorSplitEvent result;
    result.deletedActor = nullptr;
    result.newActors = newActors;
    scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, messageLog));
    size_t newActorsCount = NvBlastActorSplit(&result, actor, 8, scratch.data(), messageLog, nullptr);
    EXPECT_EQ(8, newActorsCount);
    EXPECT_EQ(true, result.deletedActor == actor);

    for (uint32_t i = 0; i < newActorsCount; ++i)
    {
        const bool actorReleaseResult = NvBlastActorDeactivate(result.newActors[i], messageLog);
        EXPECT_TRUE(actorReleaseResult);
    }
    alignedFree(family);
    alignedFree(asset);
}

TEST_F(APITest, DamageBondsCompressive)
{
    const size_t bondsCount = 6;

    const NvBlastChunkDesc c_chunks[8] =
    {
        // centroid           volume parent idx     flags                         ID
        { {0.0f, 0.0f, 0.0f}, 0.0f, UINT32_MAX, NvBlastChunkDesc::NoFlags, 0 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 1 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 2 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 3 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 4 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 5 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 6 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 7 }
    };

    const NvBlastBondDesc c_bonds[bondsCount] =
    {
        { { {-1.0f, 0.0f, 0.0f }, 1.0f, { 1.0f, 2.0f, 0.0f }, 0 }, { 1, 2 } },
        { { {-1.0f, 0.0f, 0.0f }, 1.0f, {-1.0f, 2.0f, 0.0f }, 0 }, { 2, 3 } },
        { { { 0.0f,-1.0f, 0.0f }, 1.0f, {-2.0f, 1.0f, 0.0f }, 0 }, { 3, 4 } },
        { { { 0.0f,-1.0f, 0.0f }, 1.0f, {-2.0f,-1.0f, 0.0f }, 0 }, { 4, 5 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, {-1.0f,-2.0f, 0.0f }, 0 }, { 5, 6 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 1.0f,-2.0f, 0.0f }, 0 }, { 6, 7 } }
    };

    // create asset
    const NvBlastAssetDesc assetDesc = { 8, c_chunks, bondsCount, c_bonds };

    std::vector<char> scratch((size_t)NvBlastGetRequiredScratchForCreateAsset(&assetDesc, messageLog));
    void* amem = alignedZeroedAlloc(NvBlastGetAssetMemorySize(&assetDesc, messageLog));
    NvBlastAsset* asset = NvBlastCreateAsset(amem, &assetDesc, scratch.data(), messageLog);
    EXPECT_TRUE(asset != nullptr);

    // create actor
    NvBlastActorDesc actorDesc;
    actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
    actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
    void* fmem = alignedZeroedAlloc(NvBlastAssetGetFamilyMemorySize(asset, messageLog));
    NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, messageLog);
    scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, messageLog));
    NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), messageLog);
    EXPECT_TRUE(actor != nullptr);

    // get graph nodes check
    std::vector<uint32_t> graphNodeIndices;
    graphNodeIndices.resize(NvBlastActorGetGraphNodeCount(actor, nullptr));
    uint32_t graphNodesCount = NvBlastActorGetGraphNodeIndices(graphNodeIndices.data(), (uint32_t)graphNodeIndices.size(), actor, nullptr);
    EXPECT_EQ(graphNodesCount, 7);

    NvBlastExtRadialDamageDesc damage = {
        1.0f,                   // compressive
        { 4.0f, 2.0f, 0.0f },   // position
        4.0f,                   // min radius - maximum damage
        6.0f                    // max radius - zero damage
    };                          //              linear falloff

    NvBlastBondFractureData outCommands[bondsCount] = { 
        { UINT32_MAX, UINT32_MAX, UINT32_MAX, 0 },
        { UINT32_MAX, UINT32_MAX, UINT32_MAX, 0 },
        { UINT32_MAX, UINT32_MAX, UINT32_MAX, 0 },
        { UINT32_MAX, UINT32_MAX, UINT32_MAX, 0 },
        { UINT32_MAX, UINT32_MAX, UINT32_MAX, 0 },
        { UINT32_MAX, UINT32_MAX, UINT32_MAX, 0 },
    };

    NvBlastFractureBuffers commands = {
        6, 0, outCommands, nullptr
    };

    NvBlastExtProgramParams programParams = { &damage, nullptr };

    NvBlastDamageProgram program = {
        NvBlastExtFalloffGraphShader,
        nullptr
    };

    NvBlastActorGenerateFracture(&commands, actor, program, &programParams, messageLog, nullptr);

    ASSERT_EQ(3, commands.bondFractureCount);
    ASSERT_EQ(0, commands.chunkFractureCount);

    // node indices in _graph_ chunks
    NvBlastBondFractureData expectedCommand[] = {
        { 0, 0, 1, 1.0f },
        { 0, 1, 2, 0.5f },
        { 0, 5, 6, 0.5f }
    };

    for (int i = 0; i < 3; i++)
    {
        EXPECT_EQ(expectedCommand[i].nodeIndex0, outCommands[i].nodeIndex0);
        EXPECT_EQ(expectedCommand[i].nodeIndex1, outCommands[i].nodeIndex1);
        EXPECT_EQ(expectedCommand[i].health, outCommands[i].health);
    }

    const bool actorReleaseResult = NvBlastActorDeactivate(actor, messageLog);
    EXPECT_TRUE(actorReleaseResult);
    alignedFree(family);
    alignedFree(asset);
}

TEST_F(APITest, DirectFractureKillsChunk)
{
    // 1--2
    // |  |
    // 3--4 <-- kill 4

    const NvBlastChunkDesc c_chunks[9] =
    {
        // centroid           volume parent idx     flags                         ID
        { {0.0f, 0.0f, 0.0f}, 0.0f, UINT32_MAX, NvBlastChunkDesc::NoFlags, 0 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 1 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 2 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 3 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 4 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 4, NvBlastChunkDesc::NoFlags, 5 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 4, NvBlastChunkDesc::NoFlags, 6 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 4, NvBlastChunkDesc::NoFlags, 7 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 4, NvBlastChunkDesc::NoFlags, 8 },
    };

    const NvBlastBondDesc c_bonds[4] =
    {
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 0.0f, 1.0f, 0.0f }, 0 }, { 1, 2 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 0.0f,-1.0f, 0.0f }, 0 }, { 3, 4 } },
        { { { 0.0f,-1.0f, 0.0f }, 1.0f, {-1.0f, 0.0f, 0.0f }, 0 }, { 1, 3 } },
        { { { 0.0f,-1.0f, 0.0f }, 1.0f, { 1.0f, 0.0f, 0.0f }, 0 }, { 2, 4 } },
    };

    NvBlastAssetDesc assetDesc;
    assetDesc.chunkCount = 9;
    assetDesc.chunkDescs = c_chunks;
    assetDesc.bondCount = 4;
    assetDesc.bondDescs = c_bonds;

    // create asset
    std::vector<char> scratch((size_t)NvBlastGetRequiredScratchForCreateAsset(&assetDesc, messageLog));
    void* amem = alignedZeroedAlloc(NvBlastGetAssetMemorySize(&assetDesc, messageLog));
    NvBlastAsset* asset = NvBlastCreateAsset(amem, &assetDesc, scratch.data(), messageLog);
    EXPECT_TRUE(asset != nullptr);

    // create actor
    NvBlastActorDesc actorDesc;
    actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
    actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
    void* fmem = alignedZeroedAlloc(NvBlastAssetGetFamilyMemorySize(asset, messageLog));
    NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, messageLog);
    scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, messageLog));
    NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), messageLog);
    EXPECT_TRUE(actor != nullptr);

    NvBlastChunkFractureData fractureCmd;
    fractureCmd.chunkIndex = 4;
    fractureCmd.health = 1.0f;

    NvBlastFractureBuffers commands = { 0, 1, nullptr, &fractureCmd };

    NvBlastChunkFractureData fractureEvt;
    NvBlastFractureBuffers events = { 0, 1, nullptr, &fractureEvt };

    NvBlastActorApplyFracture(&events, actor, &commands, messageLog, nullptr);
    EXPECT_TRUE(NvBlastActorIsSplitRequired(actor, messageLog));
    EXPECT_EQ(1, events.chunkFractureCount);

    scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, messageLog));
    std::vector<NvBlastActor*> newActors(NvBlastActorGetMaxActorCountForSplit(actor, messageLog));
    NvBlastActorSplitEvent result;
    result.deletedActor = nullptr;
    result.newActors = newActors.data();
    size_t newActorsCount = NvBlastActorSplit(&result, actor, static_cast<uint32_t>(newActors.size()), scratch.data(), messageLog, nullptr);
    newActors.resize(newActorsCount);

    EXPECT_EQ(5, newActorsCount);
    EXPECT_EQ(actor, result.deletedActor);
    // check newActors contain original actor
    EXPECT_TRUE(std::any_of(newActors.begin(), newActors.end(), [&](const NvBlastActor* a) { return actor == a; }));

    for (uint32_t i = 0; i < newActorsCount; ++i)
    {
        const bool actorReleaseResult = NvBlastActorDeactivate(result.newActors[i], messageLog);
        EXPECT_TRUE(actorReleaseResult);
    }
    alignedFree(family);
    alignedFree(asset);
}

TEST_F(APITest, DirectFractureKillsIslandRootChunk)
{
    // 1--2 <-- kill 1
    // |  |
    // 3--4

    const NvBlastChunkDesc c_chunks[9] =
    {
        // centroid           volume parent idx     flags                         ID
        { {0.0f, 0.0f, 0.0f}, 0.0f, UINT32_MAX, NvBlastChunkDesc::NoFlags, 0 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 1 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 2 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 3 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 4 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 1, NvBlastChunkDesc::NoFlags, 5 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 1, NvBlastChunkDesc::NoFlags, 6 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 1, NvBlastChunkDesc::NoFlags, 7 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 1, NvBlastChunkDesc::NoFlags, 8 },
    };

    const NvBlastBondDesc c_bonds[4] =
    {
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 0.0f, 1.0f, 0.0f }, 0 }, { 1, 2 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 0.0f,-1.0f, 0.0f }, 0 }, { 3, 4 } },
        { { { 0.0f,-1.0f, 0.0f }, 1.0f, {-1.0f, 0.0f, 0.0f }, 0 }, { 1, 3 } },
        { { { 0.0f,-1.0f, 0.0f }, 1.0f, { 1.0f, 0.0f, 0.0f }, 0 }, { 2, 4 } },
    };

    NvBlastAssetDesc assetDesc;
    assetDesc.chunkCount = 9;
    assetDesc.chunkDescs = c_chunks;
    assetDesc.bondCount = 4;
    assetDesc.bondDescs = c_bonds;

    // create asset
    std::vector<char> scratch((size_t)NvBlastGetRequiredScratchForCreateAsset(&assetDesc, messageLog));
    void* amem = alignedZeroedAlloc(NvBlastGetAssetMemorySize(&assetDesc, messageLog));
    NvBlastAsset* asset = NvBlastCreateAsset(amem, &assetDesc, scratch.data(), messageLog);
    EXPECT_TRUE(asset != nullptr);

    // create actor
    NvBlastActorDesc actorDesc;
    actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
    actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
    void* fmem = alignedZeroedAlloc(NvBlastAssetGetFamilyMemorySize(asset, messageLog));
    NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, messageLog);
    scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, messageLog));
    NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), messageLog);
    EXPECT_TRUE(actor != nullptr);

    NvBlastChunkFractureData fractureCmd;
    fractureCmd.chunkIndex = 1;
    fractureCmd.health = 1.0f;

    NvBlastFractureBuffers commands = { 0, 1, nullptr, &fractureCmd };

    NvBlastChunkFractureData fractureEvt;
    NvBlastFractureBuffers events = { 0, 1, nullptr, &fractureEvt };

    NvBlastActorApplyFracture(&events, actor, &commands, messageLog, nullptr);
    EXPECT_TRUE(NvBlastActorIsSplitRequired(actor, messageLog));
    EXPECT_EQ(1, events.chunkFractureCount);

    scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, messageLog));
    std::vector<NvBlastActor*> newActors(NvBlastActorGetMaxActorCountForSplit(actor, messageLog));
    NvBlastActorSplitEvent result;
    result.deletedActor = nullptr;
    result.newActors = newActors.data();
    size_t newActorsCount = NvBlastActorSplit(&result, actor, static_cast<uint32_t>(newActors.size()), scratch.data(), messageLog, nullptr);
    newActors.resize(newActorsCount);

    EXPECT_EQ(5, newActorsCount);
    EXPECT_EQ(actor, result.deletedActor);
    // check if newActors don't contain original actor
    EXPECT_TRUE(!std::any_of(newActors.begin(), newActors.end(), [&](const NvBlastActor* a) { return actor == a; }));

    for (uint32_t i = 0; i < newActorsCount; ++i)
    {
        const bool actorReleaseResult = NvBlastActorDeactivate(result.newActors[i], messageLog);
        EXPECT_TRUE(actorReleaseResult);
    }
    alignedFree(family);
    alignedFree(asset);
}

TEST_F(APITest, SubsupportFracture)
{
    const NvBlastAssetDesc& assetDesc = g_assetDescs[1]; // cube with subsupport

    // create asset with chunk map
    std::vector<char> scratch((size_t)NvBlastGetRequiredScratchForCreateAsset(&assetDesc, messageLog));
    void* amem = alignedZeroedAlloc(NvBlastGetAssetMemorySize(&assetDesc, messageLog));
    NvBlastAsset* asset = NvBlastCreateAsset(amem, &assetDesc, scratch.data(), messageLog);
    EXPECT_TRUE(asset != nullptr);

    // create actor
    NvBlastActorDesc actorDesc;
    actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
    actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
    void* fmem = alignedZeroedAlloc(NvBlastAssetGetFamilyMemorySize(asset, messageLog));
    NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, messageLog);
    scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, messageLog));
    NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), messageLog);
    EXPECT_TRUE(actor != nullptr);

    // first set of fracture commands
    NvBlastChunkFractureData f1 = { 0, 1, 2.0f };
    NvBlastChunkFractureData f3 = { 0, 3, 0.5f };
    NvBlastChunkFractureData f5 = { 0, 5, 1.0f };
    NvBlastChunkFractureData f7 = { 0, 7, 1.0f };

    std::vector<NvBlastChunkFractureData> chunkFractureData;
    chunkFractureData.reserve(assetDesc.chunkCount);
    chunkFractureData.push_back(f1);
    chunkFractureData.push_back(f3);
    chunkFractureData.push_back(f5);
    chunkFractureData.push_back(f7);
    ASSERT_EQ(assetDesc.chunkCount, chunkFractureData.capacity());
    ASSERT_EQ(4, chunkFractureData.size());

    NvBlastFractureBuffers target = { 0, static_cast<uint32_t>(chunkFractureData.capacity()), nullptr, chunkFractureData.data() };
    {
        NvBlastFractureBuffers events = target;
        NvBlastFractureBuffers commands = { 0, static_cast<uint32_t>(chunkFractureData.size()), nullptr, chunkFractureData.data() };
        NvBlastActorApplyFracture(&events, actor, &commands, messageLog, nullptr);
        EXPECT_TRUE(NvBlastActorIsSplitRequired(actor, messageLog));
        ASSERT_EQ(4 + 8, events.chunkFractureCount); // all requested chunks take damage, and the children of one of them
    }

    // re-apply same set of commands
    chunkFractureData.clear();
    chunkFractureData.reserve(assetDesc.chunkCount);
    chunkFractureData.push_back(f1);
    chunkFractureData.push_back(f3);
    chunkFractureData.push_back(f5);
    chunkFractureData.push_back(f7);
    ASSERT_EQ(assetDesc.chunkCount, chunkFractureData.capacity());
    ASSERT_EQ(4, chunkFractureData.size());

    {
        NvBlastFractureBuffers events = target;
        NvBlastFractureBuffers commands = { 0, static_cast<uint32_t>(chunkFractureData.size()), nullptr, chunkFractureData.data() };
        NvBlastActorApplyFracture(&events, actor, &commands, messageLog, nullptr);
        EXPECT_TRUE(NvBlastActorIsSplitRequired(actor, messageLog));
        ASSERT_EQ(1, events.chunkFractureCount); // f3 has broken the chunk
    }

    // fracture all support chunks
    // the chunks from the previous fractures must not be reported again (since they are all broken already)
    NvBlastChunkFractureData f2 = { 0, 2, 2.0f }; // will damage chunk and children
    NvBlastChunkFractureData f4 = { 0, 4, 0.5f }; // will damage chunk without creating children on split
    NvBlastChunkFractureData f6 = { 0, 6, 2.0f }; // will damage chunk and children
    NvBlastChunkFractureData f8 = { 0, 8, 1.0f }; // will damage chunk 

    chunkFractureData.clear();
    chunkFractureData.reserve(assetDesc.chunkCount);
    chunkFractureData.push_back(f1);
    chunkFractureData.push_back(f2);
    chunkFractureData.push_back(f3);
    chunkFractureData.push_back(f4);
    chunkFractureData.push_back(f5);
    chunkFractureData.push_back(f6);
    chunkFractureData.push_back(f7);
    chunkFractureData.push_back(f8);
    ASSERT_EQ(assetDesc.chunkCount, chunkFractureData.capacity());
    ASSERT_EQ(8, chunkFractureData.size());

    NvBlastFractureBuffers events = target;
    {
        NvBlastFractureBuffers commands = { 0, static_cast<uint32_t>(chunkFractureData.size()), nullptr, chunkFractureData.data() };
        NvBlastActorApplyFracture(&events, actor, &commands, messageLog, nullptr);
        EXPECT_TRUE(NvBlastActorIsSplitRequired(actor, messageLog));
        ASSERT_EQ(4 + 8 + 8, events.chunkFractureCount); // the new fracture commands all apply, plus two of them damage their children too
    }

    for (size_t i = 0; i < events.chunkFractureCount; i++)
    {
        const uint32_t chunkIndex = events.chunkFractures[i].chunkIndex;

        ASSERT_TRUE(chunkIndex != 1);
        ASSERT_TRUE(chunkIndex != 3);
        ASSERT_TRUE(chunkIndex != 5);
        ASSERT_TRUE(chunkIndex != 7);

        // literal values come from g_cube2ChunkDescs
        bool isInSupportRange = chunkIndex <= 8 && chunkIndex >= 1;
        bool isChildOfTwo = chunkIndex <= 24 && chunkIndex >= 17;
        bool isChildOfSix = chunkIndex <= 56 && chunkIndex >= 49;

        ASSERT_TRUE(isInSupportRange || isChildOfTwo || isChildOfSix);
    }

    scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, messageLog));
    std::vector<NvBlastActor*> newActors(NvBlastActorGetMaxActorCountForSplit(actor, messageLog));
    NvBlastActorSplitEvent result;
    result.deletedActor = nullptr;
    result.newActors = newActors.data();
    size_t newActorsCount = NvBlastActorSplit(&result, actor, static_cast<uint32_t>(newActors.size()), scratch.data(), messageLog, nullptr);
    newActors.resize(newActorsCount);

    EXPECT_EQ(64 - 8 + 1, newActorsCount);
    for (uint32_t i = 0; i < newActorsCount; ++i)
    {
        const bool actorReleaseResult = NvBlastActorDeactivate(result.newActors[i], messageLog);
        EXPECT_TRUE(actorReleaseResult);
    }
    alignedFree(family);
    alignedFree(asset);
}

static bool hasWarned = false;
static void myLog(int type, const char* msg, const char* file, int line)
{
    BlastBaseTest<-1, 0>::messageLog(type, msg, file, line);
    hasWarned = true;
}
#define EXPECT_WARNING EXPECT_TRUE(hasWarned); hasWarned=false;
#define EXPECT_NO_WARNING EXPECT_FALSE(hasWarned); hasWarned=false;

TEST_F(APITest, FractureNoEvents)
{
    static const uint32_t GUARD = 0xb1a57;

    const uint32_t chunksCount = 17;
    const NvBlastChunkDesc c_chunks[chunksCount] =
    {
        // centroid           volume parent idx     flags                         ID
        { {0.0f, 0.0f, 0.0f}, 0.0f, UINT32_MAX, NvBlastChunkDesc::NoFlags, 0 },

        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 1 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 2 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 3 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 4 },

        { {0.0f, 0.0f, 0.0f}, 0.0f, 1, NvBlastChunkDesc::NoFlags, 5 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 1, NvBlastChunkDesc::NoFlags, 6 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 2, NvBlastChunkDesc::NoFlags, 7 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 2, NvBlastChunkDesc::NoFlags, 8 },

        { {0.0f, 0.0f, 0.0f}, 0.0f, 5, NvBlastChunkDesc::NoFlags, 9 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 5, NvBlastChunkDesc::NoFlags, 10 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 6, NvBlastChunkDesc::NoFlags, 11 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 6, NvBlastChunkDesc::NoFlags, 12 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 7, NvBlastChunkDesc::NoFlags, 13 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 7, NvBlastChunkDesc::NoFlags, 14 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 8, NvBlastChunkDesc::NoFlags, 15 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 8, NvBlastChunkDesc::NoFlags, 16 },
    };

    const NvBlastBondDesc c_bonds[3] =
    {
        // normal, area, centroid, userdata, chunks
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 1.0f, 0.0f, 0.0f }, 0 }, { 1, 2 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 2.0f, 0.0f, 0.0f }, 0 }, { 2, 3 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 3.0f, 0.0f, 0.0f }, 0 }, { 3, 4 } },
    };

    NvBlastAssetDesc assetDesc = { chunksCount, c_chunks, 3, c_bonds };

    // create asset with chunk map
    std::vector<char> scratch((size_t)NvBlastGetRequiredScratchForCreateAsset(&assetDesc, myLog));
    void* amem = alignedZeroedAlloc(NvBlastGetAssetMemorySize(&assetDesc, myLog));
    NvBlastAsset* asset = NvBlastCreateAsset(amem, &assetDesc, scratch.data(), myLog);
    EXPECT_TRUE(asset != nullptr);

    // create actor
    NvBlastActorDesc actorDesc;
    actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
    actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
    void* fmem = alignedZeroedAlloc(NvBlastAssetGetFamilyMemorySize(asset, myLog));
    NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, myLog);
    scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, myLog));
    NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), myLog);
    EXPECT_TRUE(actor != nullptr);

    std::vector<NvBlastChunkFractureData> cfData;
    cfData.resize(0 + 1);
    cfData[cfData.size() - 1].userdata = GUARD;
    std::vector<NvBlastBondFractureData> bfData;

    NvBlastChunkFractureData command[] =
    {
        { 0, 1, 10.0f },
        { 0, 2, 10.0f },
    };

    NvBlastFractureBuffers commands = { 0, 2, nullptr, command };
    NvBlastActorApplyFracture(nullptr, actor, &commands, myLog, nullptr);
    EXPECT_TRUE(NvBlastActorIsSplitRequired(actor, messageLog));

    EXPECT_NO_WARNING; // events can be null
    EXPECT_EQ(GUARD, cfData[cfData.size() - 1].userdata);

    scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, myLog));
    std::vector<NvBlastActor*> newActors(NvBlastActorGetMaxActorCountForSplit(actor, myLog));
    NvBlastActorSplitEvent result;
    result.deletedActor = nullptr;
    result.newActors = newActors.data();
    size_t newActorsCount = NvBlastActorSplit(&result, actor, static_cast<uint32_t>(newActors.size()), scratch.data(), myLog, nullptr);
    newActors.resize(newActorsCount);

    EXPECT_EQ(9, newActorsCount);
    for (uint32_t i = 0; i < newActorsCount; ++i)
    {
        const bool actorReleaseResult = NvBlastActorDeactivate(result.newActors[i], myLog);
        EXPECT_TRUE(actorReleaseResult);
    }
    
    alignedFree(family);
    alignedFree(asset);

    EXPECT_NO_WARNING;
}

TEST_F(APITest, FractureBufferLimits)
{
    static const uint32_t GUARD = 0xb1a57;

    const uint32_t chunksCount = 17;
    const NvBlastChunkDesc c_chunks[chunksCount] =
    {
        // centroid           volume parent idx     flags                         ID
        { {0.0f, 0.0f, 0.0f}, 0.0f, UINT32_MAX, NvBlastChunkDesc::NoFlags, 0 },

        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 1 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 2 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 3 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 4 },

        { {0.0f, 0.0f, 0.0f}, 0.0f, 1, NvBlastChunkDesc::NoFlags, 5 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 1, NvBlastChunkDesc::NoFlags, 6 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 2, NvBlastChunkDesc::NoFlags, 7 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 2, NvBlastChunkDesc::NoFlags, 8 },

        { {0.0f, 0.0f, 0.0f}, 0.0f, 5, NvBlastChunkDesc::NoFlags, 9 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 5, NvBlastChunkDesc::NoFlags, 10 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 6, NvBlastChunkDesc::NoFlags, 11 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 6, NvBlastChunkDesc::NoFlags, 12 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 7, NvBlastChunkDesc::NoFlags, 13 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 7, NvBlastChunkDesc::NoFlags, 14 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 8, NvBlastChunkDesc::NoFlags, 15 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 8, NvBlastChunkDesc::NoFlags, 16 },
    };

    const NvBlastBondDesc c_bonds[3] =
    {
        // normal, area, centroid, userdata, chunks
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 1.0f, 0.0f, 0.0f }, 0 }, { 1, 2 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 2.0f, 0.0f, 0.0f }, 0 }, { 2, 3 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 3.0f, 0.0f, 0.0f }, 0 }, { 3, 4 } },
    };

    NvBlastAssetDesc assetDesc = { chunksCount, c_chunks, 3, c_bonds };
    {
        // create asset with chunk map
        std::vector<char> scratch((size_t)NvBlastGetRequiredScratchForCreateAsset(&assetDesc, myLog));
        void* amem = alignedZeroedAlloc(NvBlastGetAssetMemorySize(&assetDesc, myLog));
        NvBlastAsset* asset = NvBlastCreateAsset(amem, &assetDesc, scratch.data(), myLog);
        EXPECT_TRUE(asset != nullptr);

        for (uint32_t i = 0; i < 14; i++)
        {
            // create actor
            NvBlastActorDesc actorDesc;
            actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
            actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
            void* fmem = alignedZeroedAlloc(NvBlastAssetGetFamilyMemorySize(asset, myLog));
            NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, myLog);
            scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, myLog));
            NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), myLog);
            EXPECT_TRUE(actor != nullptr);

            std::vector<NvBlastChunkFractureData> cfData;
            cfData.resize(i + 1);
            cfData[cfData.size() - 1].userdata = GUARD;
            std::vector<NvBlastBondFractureData> bfData;

            NvBlastChunkFractureData command[] =
            {
                { 0, 1, 10.0f },
                { 0, 2, 10.0f },
            };

            NvBlastFractureBuffers commands = { 0, 2, nullptr, command };
            NvBlastFractureBuffers events = { static_cast<uint32_t>(bfData.size()), static_cast<uint32_t>(cfData.size()) - 1, bfData.data(), cfData.data() };

            NvBlastActorApplyFracture(&events, actor, &commands, myLog, nullptr);

            EXPECT_WARNING;
            EXPECT_EQ(GUARD, cfData[cfData.size() - 1].userdata);
            EXPECT_EQ(i, events.chunkFractureCount);
            for (uint32_t i = 0; i < events.chunkFractureCount; i++)
            {
                EXPECT_EQ(events.chunkFractures[i].chunkIndex, events.chunkFractures[i].userdata);
            }

            EXPECT_TRUE(NvBlastActorDeactivate(actor, myLog));
            alignedFree(family);
        }

        {
            // create actor
            NvBlastActorDesc actorDesc;
            actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
            actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
            void* fmem = alignedZeroedAlloc(NvBlastAssetGetFamilyMemorySize(asset, myLog));
            NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, myLog);
            scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, myLog));
            NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), myLog);
            EXPECT_TRUE(actor != nullptr);

            std::vector<NvBlastChunkFractureData> cfData;
            cfData.resize(14 + 1);
            cfData[cfData.size() - 1].userdata = GUARD;
            std::vector<NvBlastBondFractureData> bfData;

            NvBlastChunkFractureData command[] =
            {
                { 0, 1, 10.0f },
                { 0, 2, 10.0f },
            };

            NvBlastFractureBuffers commands = { 0, 2, nullptr, command };
            NvBlastFractureBuffers events = { static_cast<uint32_t>(bfData.size()), static_cast<uint32_t>(cfData.size()) - 1, bfData.data(), cfData.data() };

            NvBlastActorApplyFracture(&events, actor, &commands, myLog, nullptr);
            EXPECT_TRUE(NvBlastActorIsSplitRequired(actor, messageLog));

            EXPECT_NO_WARNING;
            EXPECT_EQ(14, events.chunkFractureCount);
            for (uint32_t i = 0; i < events.chunkFractureCount; i++)
            {
                EXPECT_EQ(events.chunkFractures[i].chunkIndex, events.chunkFractures[i].userdata);
            }
            ASSERT_EQ(GUARD, cfData[cfData.size() - 1].userdata);

            scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, myLog));
            std::vector<NvBlastActor*> newActors(NvBlastActorGetMaxActorCountForSplit(actor, myLog));
            NvBlastActorSplitEvent result;
            result.deletedActor = nullptr;
            result.newActors = newActors.data();
            size_t newActorsCount = NvBlastActorSplit(&result, actor, static_cast<uint32_t>(newActors.size()), scratch.data(), myLog, nullptr);
            newActors.resize(newActorsCount);

            EXPECT_EQ(9, newActorsCount);
            for (uint32_t i = 0; i < newActorsCount; ++i)
            {
                const bool actorReleaseResult = NvBlastActorDeactivate(result.newActors[i], myLog);
                EXPECT_TRUE(actorReleaseResult);
            }

            alignedFree(family);
        }

        alignedFree(asset);
    }
    EXPECT_NO_WARNING;
}

TEST_F(APITest, FractureBufferLimitsInSitu)
{
    static const uint32_t GUARD = 0xb1a57;

    const uint32_t chunksCount = 17;
    const NvBlastChunkDesc c_chunks[chunksCount] =
    {
        // cenroid            volume parent idx     flags                         ID
        { {0.0f, 0.0f, 0.0f}, 0.0f, UINT32_MAX, NvBlastChunkDesc::NoFlags, 0 },

        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 1 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 2 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 3 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 4 },

        { {0.0f, 0.0f, 0.0f}, 0.0f, 1, NvBlastChunkDesc::NoFlags, 5 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 1, NvBlastChunkDesc::NoFlags, 6 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 2, NvBlastChunkDesc::NoFlags, 7 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 2, NvBlastChunkDesc::NoFlags, 8 },

        { {0.0f, 0.0f, 0.0f}, 0.0f, 5, NvBlastChunkDesc::NoFlags, 9 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 5, NvBlastChunkDesc::NoFlags, 10 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 6, NvBlastChunkDesc::NoFlags, 11 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 6, NvBlastChunkDesc::NoFlags, 12 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 7, NvBlastChunkDesc::NoFlags, 13 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 7, NvBlastChunkDesc::NoFlags, 14 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 8, NvBlastChunkDesc::NoFlags, 15 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 8, NvBlastChunkDesc::NoFlags, 16 },
    };

    const NvBlastBondDesc c_bonds[3] =
    {
        // normal, area, centroid, userdata, chunks
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 1.0f, 0.0f, 0.0f }, 0 }, { 1, 2 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 2.0f, 0.0f, 0.0f }, 0 }, { 2, 3 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 3.0f, 0.0f, 0.0f }, 0 }, { 3, 4 } },
    };

    NvBlastAssetDesc assetDesc = { chunksCount, c_chunks, 3, c_bonds };
    {
        // create asset with chunk map
        std::vector<char> scratch((size_t)NvBlastGetRequiredScratchForCreateAsset(&assetDesc, myLog));
        void* amem = alignedZeroedAlloc(NvBlastGetAssetMemorySize(&assetDesc, myLog));
        NvBlastAsset* asset = NvBlastCreateAsset(amem, &assetDesc, scratch.data(), myLog);
        EXPECT_TRUE(asset != nullptr);

        for (uint32_t i = 0; i < 14 - 2; i++)
        {
            // create actor
            NvBlastActorDesc actorDesc;
            actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
            actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
            void* fmem = alignedZeroedAlloc(NvBlastAssetGetFamilyMemorySize(asset, myLog));
            NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, myLog);
            scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, myLog));
            NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), myLog);
            EXPECT_TRUE(actor != nullptr);

            std::vector<NvBlastChunkFractureData> cfData;
            cfData.resize(2 + i + 1);

            std::vector<NvBlastBondFractureData> bfData;

            cfData[0].userdata = 0;
            cfData[0].chunkIndex = 1;
            cfData[0].health = 10.0f;

            cfData[1].userdata = 0;
            cfData[1].chunkIndex = 2;
            cfData[1].health = 10.0f;

            cfData[2 + i].userdata = GUARD;

            NvBlastFractureBuffers commands = { 0, 2, nullptr, cfData.data() };
            NvBlastFractureBuffers events = { static_cast<uint32_t>(bfData.size()), static_cast<uint32_t>(cfData.size()) - 1, bfData.data(), cfData.data() };

            NvBlastActorApplyFracture(&events, actor, &commands, myLog, nullptr);
            EXPECT_TRUE(NvBlastActorIsSplitRequired(actor, messageLog));

            EXPECT_WARNING;
            EXPECT_EQ(GUARD, cfData[cfData.size() - 1].userdata);
            EXPECT_EQ(2 + i, events.chunkFractureCount);
            for (uint32_t i = 0; i < events.chunkFractureCount; i++)
            {
                EXPECT_EQ(events.chunkFractures[i].chunkIndex, events.chunkFractures[i].userdata);
            }

            EXPECT_TRUE(NvBlastActorDeactivate(actor, myLog));

            alignedFree(family);
        }

        {
            // create actor
            NvBlastActorDesc actorDesc;
            actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
            actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
            void* fmem = alignedZeroedAlloc(NvBlastAssetGetFamilyMemorySize(asset, myLog));
            NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, myLog);
            scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, myLog));
            NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), myLog);
            EXPECT_TRUE(actor != nullptr);

            std::vector<NvBlastChunkFractureData> cfData;
            cfData.resize(14 + 1);
            cfData[cfData.size() - 1].userdata = GUARD;
            std::vector<NvBlastBondFractureData> bfData;

            cfData[0].userdata = 0;
            cfData[0].chunkIndex = 1;
            cfData[0].health = 10.0f;

            cfData[1].userdata = 0;
            cfData[1].chunkIndex = 2;
            cfData[1].health = 10.0f;

            cfData[14].userdata = GUARD;

            NvBlastFractureBuffers commands = { 0, 2, nullptr, cfData.data() };
            NvBlastFractureBuffers events = { static_cast<uint32_t>(bfData.size()), static_cast<uint32_t>(cfData.size()) - 1, bfData.data(), cfData.data() };

            NvBlastActorApplyFracture(&events, actor, &commands, myLog, nullptr);
            EXPECT_TRUE(NvBlastActorIsSplitRequired(actor, messageLog));

            EXPECT_NO_WARNING;
            EXPECT_EQ(14, events.chunkFractureCount);
            for (uint32_t i = 0; i < events.chunkFractureCount; i++)
            {
                EXPECT_EQ(events.chunkFractures[i].chunkIndex, events.chunkFractures[i].userdata);
            }
            ASSERT_EQ(GUARD, cfData[cfData.size() - 1].userdata);

            scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, myLog));
            std::vector<NvBlastActor*> newActors(NvBlastActorGetMaxActorCountForSplit(actor, myLog));
            NvBlastActorSplitEvent result;
            result.deletedActor = nullptr;
            result.newActors = newActors.data();
            size_t newActorsCount = NvBlastActorSplit(&result, actor, static_cast<uint32_t>(newActors.size()), scratch.data(), myLog, nullptr);
            newActors.resize(newActorsCount);

            EXPECT_EQ(9, newActorsCount);
            for (uint32_t i = 0; i < newActorsCount; ++i)
            {
                const bool actorReleaseResult = NvBlastActorDeactivate(result.newActors[i], myLog);
                EXPECT_TRUE(actorReleaseResult);
            }
            alignedFree(family);
        }

        alignedFree(asset);
    }
    EXPECT_NO_WARNING;
}

/*
This test checks if bond or chunk fracture commands passed to NvBlastActorApplyFracture do not correspond to 
the actor passed in they (commands) will be ignored and warning message will be fired.
*/
TEST_F(APITest, FractureWarnAndFilterOtherActorCommands)
{
    const uint32_t chunksCount = 17;
    const NvBlastChunkDesc c_chunks[chunksCount] =
    {
        // centroid           volume parent idx     flags                         ID
        { { 0.0f, 0.0f, 0.0f }, 0.0f, UINT32_MAX, NvBlastChunkDesc::NoFlags, 0 },

        { { 0.0f, 0.0f, 0.0f }, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 1 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 2 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 3 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 4 },

        { { 0.0f, 0.0f, 0.0f }, 0.0f, 1, NvBlastChunkDesc::NoFlags, 5 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 1, NvBlastChunkDesc::NoFlags, 6 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 2, NvBlastChunkDesc::NoFlags, 7 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 2, NvBlastChunkDesc::NoFlags, 8 },

        { { 0.0f, 0.0f, 0.0f }, 0.0f, 5, NvBlastChunkDesc::NoFlags, 9 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 5, NvBlastChunkDesc::NoFlags, 10 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 6, NvBlastChunkDesc::NoFlags, 11 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 6, NvBlastChunkDesc::NoFlags, 12 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 7, NvBlastChunkDesc::NoFlags, 13 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 7, NvBlastChunkDesc::NoFlags, 14 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 8, NvBlastChunkDesc::NoFlags, 15 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 8, NvBlastChunkDesc::NoFlags, 16 },
    };

    const NvBlastBondDesc c_bonds[4] =
    {
        // normal, area, centroid, userdata, chunks
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 }, { 1, 2 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 }, { 2, 3 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 }, { 3, 4 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 }, { 1, 3 } }
    };

    NvBlastAssetDesc assetDesc = { chunksCount, c_chunks, 4, c_bonds };

    // create asset with chunk map
    std::vector<char> scratch((size_t)NvBlastGetRequiredScratchForCreateAsset(&assetDesc, myLog));
    void* amem = alignedZeroedAlloc(NvBlastGetAssetMemorySize(&assetDesc, myLog));
    NvBlastAsset* asset = NvBlastCreateAsset(amem, &assetDesc, scratch.data(), myLog);
    EXPECT_TRUE(asset != nullptr);

    // create actor
    NvBlastActorDesc actorDesc;
    actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
    actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
    void* fmem = alignedZeroedAlloc(NvBlastAssetGetFamilyMemorySize(asset, myLog));
    NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, myLog);
    scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, myLog));
    NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), myLog);
    EXPECT_TRUE(actor != nullptr);

    // split in 2
    std::vector<NvBlastActor*> actors;
    {
        NvBlastBondFractureData command[] =
        {
            { 0, 0, 2, 10.0f },
            { 0, 1, 2, 10.0f }
        };

        NvBlastFractureBuffers commands = { 2, 0, command, nullptr };
        NvBlastActorApplyFracture(nullptr, actor, &commands, myLog, nullptr);
        EXPECT_TRUE(NvBlastActorIsSplitRequired(actor, messageLog));

        scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, myLog));
        std::vector<NvBlastActor*> newActors(NvBlastActorGetMaxActorCountForSplit(actor, myLog));
        NvBlastActorSplitEvent result;
        result.deletedActor = nullptr;
        result.newActors = newActors.data();
        size_t newActorsCount = NvBlastActorSplit(&result, actor, static_cast<uint32_t>(newActors.size()), scratch.data(), myLog, nullptr);

        EXPECT_EQ(2, newActorsCount);
        EXPECT_EQ(actor, result.deletedActor);

        actors.insert(actors.begin(), result.newActors, result.newActors + newActorsCount);
    }

    // damage bonds belonging to other actors, nothing expected to be broken
    {
        for (uint32_t i = 0; i < actors.size(); ++i)
        {
            NvBlastActor* actor = actors[i];
            NvBlastActor* otherActor = actors[(i + 1) % 2];

            // get graph nodes check
            std::vector<uint32_t> graphNodeIndices;
            graphNodeIndices.resize(NvBlastActorGetGraphNodeCount(otherActor, nullptr));
            uint32_t graphNodesCount = NvBlastActorGetGraphNodeIndices(graphNodeIndices.data(), (uint32_t)graphNodeIndices.size(), otherActor, nullptr);
            EXPECT_EQ(graphNodesCount, 2);

            NvBlastBondFractureData command[] =
            {
                { 0, graphNodeIndices[0], graphNodeIndices[1], 10.0f }
            };

            NvBlastFractureBuffers commands = { 1, 0, command, nullptr };
            NvBlastActorApplyFracture(nullptr, actor, &commands, myLog, nullptr);
            EXPECT_WARNING;
            EXPECT_FALSE(NvBlastActorIsSplitRequired(actor, messageLog));

            scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, myLog));
            std::vector<NvBlastActor*> newActors(NvBlastActorGetMaxActorCountForSplit(actor, myLog));
            NvBlastActorSplitEvent result;
            result.deletedActor = nullptr;
            result.newActors = newActors.data();
            size_t newActorsCount = NvBlastActorSplit(&result, actor, static_cast<uint32_t>(newActors.size()), scratch.data(), myLog, nullptr);

            EXPECT_EQ(0, newActorsCount);
            EXPECT_EQ(nullptr, result.deletedActor);
        }
    }

    // damage bonds, split actors in 2 each
    std::vector<NvBlastActor*> actors2;
    {
        for (uint32_t i = 0; i < 2; ++i)
        {
            NvBlastActor* actor = actors[i];

            // get graph nodes check
            std::vector<uint32_t> graphNodeIndices;
            graphNodeIndices.resize(NvBlastActorGetGraphNodeCount(actor, nullptr));
            uint32_t graphNodesCount = NvBlastActorGetGraphNodeIndices(graphNodeIndices.data(), (uint32_t)graphNodeIndices.size(), actor, nullptr);
            EXPECT_EQ(graphNodesCount, 2);

            NvBlastBondFractureData command[] =
            {
                { 0, graphNodeIndices[0], graphNodeIndices[1], 10.0f }
            };

            NvBlastFractureBuffers commands = { 1, 0, command, nullptr };
            NvBlastActorApplyFracture(nullptr, actor, &commands, myLog, nullptr);
            EXPECT_TRUE(NvBlastActorIsSplitRequired(actor, messageLog));

            scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, myLog));
            std::vector<NvBlastActor*> newActors(NvBlastActorGetMaxActorCountForSplit(actor, myLog));
            NvBlastActorSplitEvent result;
            result.deletedActor = nullptr;
            result.newActors = newActors.data();
            size_t newActorsCount = NvBlastActorSplit(&result, actor, static_cast<uint32_t>(newActors.size()), scratch.data(), myLog, nullptr);

            EXPECT_EQ(2, newActorsCount);
            EXPECT_EQ(actor, result.deletedActor);

            actors2.insert(actors2.begin(), result.newActors, result.newActors + newActorsCount);
        }
    }

    // damage chunk belonging to other actor (expect no split or damage taken)
    {
        for (uint32_t i = 0; i < actors.size(); ++i)
        {
            NvBlastActor* actor = actors[i];
            NvBlastActor* otherActor = actors[(i + 1) % 2];

            uint32_t chunkToDamage;
            NvBlastActorGetVisibleChunkIndices(&chunkToDamage, 1, otherActor, myLog);

            NvBlastChunkFractureData command[] =
            {
                { 0, chunkToDamage, 0.9f },
            };

            NvBlastFractureBuffers commands = { 0, 1, nullptr, command };
            NvBlastActorApplyFracture(nullptr, actor, &commands, myLog, nullptr);
            EXPECT_WARNING;
            EXPECT_FALSE(NvBlastActorIsSplitRequired(actor, messageLog));

            scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, myLog));
            std::vector<NvBlastActor*> newActors(NvBlastActorGetMaxActorCountForSplit(actor, myLog));
            NvBlastActorSplitEvent result;
            result.deletedActor = nullptr;
            result.newActors = newActors.data();
            size_t newActorsCount = NvBlastActorSplit(&result, actor, static_cast<uint32_t>(newActors.size()), scratch.data(), myLog, nullptr);

            EXPECT_EQ(0, newActorsCount);
            EXPECT_EQ(nullptr, result.deletedActor);

            EXPECT_EQ(1, NvBlastActorGetVisibleChunkCount(actor, myLog));
            uint32_t chunkIndex;
            NvBlastActorGetVisibleChunkIndices(&chunkIndex, 1, actor, myLog);
            EXPECT_NE(chunkToDamage, chunkIndex);
        }
    }

    for (NvBlastActor* actor : actors2)
    {
        NvBlastActorDeactivate(actor, myLog);
    }

    alignedFree(family);
    alignedFree(asset);

    EXPECT_NO_WARNING;
}

/**
If duplicate bonds are passed asset create routine will ignore them (but fire warning)
We pass duplicated bonds to world chunk and fully fracture actor once.
*/
TEST_F(APITest, FractureWithBondDuplicates)
{
    const uint32_t chunksCount = 17;
    const NvBlastChunkDesc c_chunks[chunksCount] =
    {
        // centroid           volume parent idx     flags                         ID
        { { 0.0f, 0.0f, 0.0f }, 0.0f, UINT32_MAX, NvBlastChunkDesc::NoFlags, 0 },

        { { 0.0f, 0.0f, 0.0f }, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 1 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 2 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 3 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 4 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 5 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 6 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 7 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 8 },

        { { 0.0f, 0.0f, 0.0f }, 0.0f, 5, NvBlastChunkDesc::NoFlags, 9 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 5, NvBlastChunkDesc::NoFlags, 10 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 6, NvBlastChunkDesc::NoFlags, 11 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 6, NvBlastChunkDesc::NoFlags, 12 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 7, NvBlastChunkDesc::NoFlags, 13 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 7, NvBlastChunkDesc::NoFlags, 14 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 8, NvBlastChunkDesc::NoFlags, 15 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 8, NvBlastChunkDesc::NoFlags, 16 },
    };

    const uint32_t bondCount = 20;
    const uint32_t world = ~(uint32_t)0; // world chunk => invalid index
    const NvBlastBondDesc c_bonds[bondCount] =
    {
        // normal, area, centroid, userdata, chunks
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 1, world } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 2, 1 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 2, world } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 2, world } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 3, 1 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 4, 3 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 4, 2 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 4, world } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 5, 1 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 5, world } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 6, 5 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 6, 2 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 6, world } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 6, world } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 7, 5 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 7, 3 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 8, 7 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 8, 6 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 8, 4 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 1.0f, 0.0f, 0.0f }, 0 },{ 8, world } }
    };

    NvBlastAssetDesc assetDesc = { chunksCount, c_chunks, bondCount, c_bonds };

    // create asset 
    std::vector<char> scratch((size_t)NvBlastGetRequiredScratchForCreateAsset(&assetDesc, myLog));
    void* amem = alignedZeroedAlloc(NvBlastGetAssetMemorySize(&assetDesc, myLog));
    NvBlastAsset* asset = NvBlastCreateAsset(amem, &assetDesc, scratch.data(), myLog);
    EXPECT_WARNING;
    EXPECT_TRUE(asset != nullptr);

    // create actor
    NvBlastActorDesc actorDesc;
    actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
    actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
    void* fmem = alignedZeroedAlloc(NvBlastAssetGetFamilyMemorySize(asset, myLog));
    NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, myLog);
    scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, myLog));
    NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), myLog);
    EXPECT_TRUE(actor != nullptr);

    // split in 2
    std::vector<NvBlastActor*> actors;
    {
        NvBlastExtRadialDamageDesc damage = {
            10.0f,                  // compressive
            { 0.0f, 0.0f, 0.0f },   // position
            100.0f,                 // min radius - maximum damage
            100.0f                  // max radius - zero damage
        };

        NvBlastBondFractureData outBondFracture[bondCount];
        NvBlastChunkFractureData outChunkFracture[chunksCount];

        NvBlastFractureBuffers events;
        events.bondFractureCount = 2;
        events.bondFractures = outBondFracture;
        events.chunkFractureCount = 2;
        events.chunkFractures = outChunkFracture;

        NvBlastExtProgramParams programParams = { &damage, nullptr };

        NvBlastDamageProgram program = {
            NvBlastExtFalloffGraphShader,
            NvBlastExtFalloffSubgraphShader
        };

        NvBlastActorGenerateFracture(&events, actor, program, &programParams, myLog, nullptr);
        NvBlastActorApplyFracture(nullptr, actor, &events, myLog, nullptr);
        EXPECT_TRUE(NvBlastActorIsSplitRequired(actor, messageLog));

        scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, myLog));
        std::vector<NvBlastActor*> newActors(NvBlastActorGetMaxActorCountForSplit(actor, myLog));
        NvBlastActorSplitEvent result;
        result.deletedActor = nullptr;
        result.newActors = newActors.data();
        size_t newActorsCount = NvBlastActorSplit(&result, actor, static_cast<uint32_t>(newActors.size()), scratch.data(), myLog, nullptr);

        EXPECT_EQ(8, newActorsCount);
        EXPECT_EQ(actor, result.deletedActor);

        actors.insert(actors.begin(), result.newActors, result.newActors + newActorsCount);
    }

    for (NvBlastActor* actor : actors)
    {
        NvBlastActorDeactivate(actor, myLog);
    }

    alignedFree(family);
    alignedFree(asset);

    EXPECT_NO_WARNING;
}

#if 0
TEST(APITest, UserChunkMap)
{
    for (int i = 0; i < 2; ++i)
    {
        // Choose descriptor list
        const NvBlastAssetDesc* descs = nullptr;
        size_t size = 0;
        switch (i)
        {
        case 0: 
            descs = g_assetDescs; 
            size = sizeof(g_assetDescs) / sizeof(g_assetDescs[0]);
            break;
        case 1: 
            descs = g_assetDescsMissingCoverage;    
            size = sizeof(g_assetDescsMissingCoverage) / sizeof(g_assetDescsMissingCoverage[0]);
            break;
        default: 
            continue;
        }

        // Iterate over list
        for (size_t j = 0; j < size; ++j)
        {
            // Create asset
            const NvBlastAssetDesc* desc = descs + j;
            std::vector<char> scratch((size_t)NvBlastGetRequiredScratchForCreateAsset(desc));
            std::vector<uint32_t> chunkMap(desc->chunkCount);
            NvBlastAsset* asset = NvBlastCreateAsset(&chunkMap[0], desc, alignedAlloc<malloc>, scratch.data(), nullptr);
            EXPECT_TRUE(asset);

            // Test map
            Nv::Blast::Asset& a = static_cast<Nv::Blast::Asset&>(asset);
            uint32_t supportChunkCount = 0;
            uint32_t subsupportChunkCount = 0;
            for (uint32_t i = 0; i < desc->chunkCount; ++i)
            {
                const uint32_t map = chunkMap[i];
                if (Nv::Blast::isInvalidIndex(map))
                {
                    continue;
                }
                else if (map < a.m_firstSubsupportChunkIndex)
                {
                    EXPECT_LT(map, asset.m_graph.m_nodeCount);
                    ++supportChunkCount;
                }
                else
                {
                    EXPECT_LT(map, asset.m_chunkCount);
                    EXPECT_GE(map, asset.m_graph.m_nodeCount);
                    ++subsupportChunkCount;
                }
            }
            EXPECT_EQ(supportChunkCount, asset.m_graph.m_nodeCount);
            EXPECT_EQ(subsupportChunkCount, a.getLowerSupportChunkCount() - asset.m_graph.m_nodeCount);

            // Release asset
            NvBlastAssetRelease(asset, free, nullptr);
        }
    }
}
#endif

TEST_F(APITest, NoBondsSausage)
{
    // create asset
    const NvBlastChunkDesc c_chunks[4] =
    {
        // centroid           volume parent idx     flags                         ID
        { { 0.0f, 0.0f, 0.0f }, 0.0f, UINT32_MAX, NvBlastChunkDesc::NoFlags, 0 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 1 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 1, NvBlastChunkDesc::NoFlags, 2 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 2, NvBlastChunkDesc::NoFlags, 3 }
    };

    NvBlastAssetDesc assetDesc;
    assetDesc.chunkCount = 4;
    assetDesc.chunkDescs = c_chunks;
    assetDesc.bondCount = 0;
    assetDesc.bondDescs = nullptr;

    std::vector<char> scratch((size_t)NvBlastGetRequiredScratchForCreateAsset(&assetDesc, messageLog));
    void* amem = alignedZeroedAlloc(NvBlastGetAssetMemorySize(&assetDesc, messageLog));
    NvBlastAsset* asset = NvBlastCreateAsset(amem, &assetDesc, scratch.data(), messageLog);
    const NvBlastChunk* chunks = NvBlastAssetGetChunks(asset, messageLog);
    EXPECT_TRUE(asset != nullptr);

    // create actor
    NvBlastActorDesc actorDesc;
    actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
    actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
    void* fmem = alignedZeroedAlloc(NvBlastAssetGetFamilyMemorySize(asset, messageLog));
    NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, messageLog);
    scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, messageLog));
    NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), messageLog);
    EXPECT_TRUE(actor != nullptr);

    // check visible chunk
    {
        EXPECT_EQ(NvBlastActorGetVisibleChunkCount(actor, messageLog), 1);
        uint32_t chunkIndex;
        NvBlastActorGetVisibleChunkIndices(&chunkIndex, 1, actor, messageLog);
        EXPECT_EQ(chunks[chunkIndex].userData, 0);
    }

    // damage
    NvBlastExtRadialDamageDesc damage = {
        10.0f,                  // compressive
        { 0.0f, 0.0f, 0.0f },   // position
        4.0f,                   // min radius - maximum damage
        6.0f                    // max radius - zero damage
    };

    NvBlastBondFractureData outBondFracture[2];
    NvBlastChunkFractureData outChunkFracture[2];

    NvBlastFractureBuffers events;
    events.bondFractureCount = 2;
    events.bondFractures = outBondFracture;
    events.chunkFractureCount = 2;
    events.chunkFractures = outChunkFracture;

    NvBlastExtProgramParams programParams = { &damage, nullptr };

    NvBlastDamageProgram program = {
        NvBlastExtFalloffGraphShader,
        NvBlastExtFalloffSubgraphShader
    };

    NvBlastActorGenerateFracture(&events, actor, program, &programParams, messageLog, nullptr);
    NvBlastActorApplyFracture(&events, actor, &events, messageLog, nullptr);
    EXPECT_TRUE(NvBlastActorIsSplitRequired(actor, messageLog));
    EXPECT_EQ(0, events.bondFractureCount);
    EXPECT_EQ(1, events.chunkFractureCount);

    // split
    NvBlastActor* newActors[8]; /* num lower-support chunks? plus space for deletedActor */
    NvBlastActorSplitEvent result;
    result.deletedActor = nullptr;
    result.newActors = newActors;
    scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, messageLog));
    size_t newActorsCount = NvBlastActorSplit(&result, actor, 8, scratch.data(), messageLog, nullptr);
    EXPECT_EQ(1, newActorsCount);
    EXPECT_EQ(true, result.deletedActor == actor);

    // check visible chunk
    {
        EXPECT_EQ(NvBlastActorGetVisibleChunkCount(result.newActors[0], messageLog), 1);
        uint32_t chunkIndex;
        NvBlastActorGetVisibleChunkIndices(&chunkIndex, 1, result.newActors[0], messageLog);
        EXPECT_EQ(chunks[chunkIndex].userData, 3);
    }

    // release all
    for (uint32_t i = 0; i < newActorsCount; ++i)
    {
        const bool actorReleaseResult = NvBlastActorDeactivate(result.newActors[i], messageLog);
        EXPECT_TRUE(actorReleaseResult);
    }

    alignedFree(family);
    alignedFree(asset);
}

TEST_F(APITest, SplitOnlyWhenNecessary)
{
    static const uint32_t GUARD = 0xb1a57;

    const uint32_t chunksCount = 17;
    const NvBlastChunkDesc c_chunks[chunksCount] =
    {
        // centroid           volume parent idx     flags                         ID
        { { 0.0f, 0.0f, 0.0f }, 0.0f, UINT32_MAX, NvBlastChunkDesc::NoFlags, 0 },

        { { 0.0f, 0.0f, 0.0f }, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 1 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 2 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 3 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 4 },

        { { 0.0f, 0.0f, 0.0f }, 0.0f, 1, NvBlastChunkDesc::NoFlags, 5 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 1, NvBlastChunkDesc::NoFlags, 6 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 2, NvBlastChunkDesc::NoFlags, 7 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 2, NvBlastChunkDesc::NoFlags, 8 },

        { { 0.0f, 0.0f, 0.0f }, 0.0f, 5, NvBlastChunkDesc::NoFlags, 9 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 5, NvBlastChunkDesc::NoFlags, 10 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 6, NvBlastChunkDesc::NoFlags, 11 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 6, NvBlastChunkDesc::NoFlags, 12 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 7, NvBlastChunkDesc::NoFlags, 13 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 7, NvBlastChunkDesc::NoFlags, 14 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 8, NvBlastChunkDesc::NoFlags, 15 },
        { { 0.0f, 0.0f, 0.0f }, 0.0f, 8, NvBlastChunkDesc::NoFlags, 16 },
    };

    const NvBlastBondDesc c_bonds[4] =
    {
        // normal, area, centroid, userdata, chunks
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 1.0f, 0.0f, 0.0f }, 0 }, { 1, 2 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 1.0f, 0.0f, 0.0f }, 0 }, { 2, 3 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 1.0f, 0.0f, 0.0f }, 0 }, { 3, 4 } },
        { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 1.0f, 0.0f, 0.0f }, 0 }, { 1, 3 } }
    };

    NvBlastAssetDesc assetDesc = { chunksCount, c_chunks, 4, c_bonds };

    // create asset with chunk map
    std::vector<char> scratch((size_t)NvBlastGetRequiredScratchForCreateAsset(&assetDesc, myLog));
    void* amem = alignedZeroedAlloc(NvBlastGetAssetMemorySize(&assetDesc, myLog));
    NvBlastAsset* asset = NvBlastCreateAsset(amem, &assetDesc, scratch.data(), myLog);
    EXPECT_TRUE(asset != nullptr);

    // create actor
    NvBlastActorDesc actorDesc;
    actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
    actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
    void* fmem = alignedZeroedAlloc(NvBlastAssetGetFamilyMemorySize(asset, myLog));
    NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, myLog);
    scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, myLog));
    NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), myLog);
    EXPECT_TRUE(actor != nullptr);


    // damage health only (expect no split)
    {
        NvBlastBondFractureData command[] =
        {
            { 0, 0, 1, 0.99f },
            { 0, 1, 2, 0.50f },
            { 0, 2, 3, 0.01f }
        };

        NvBlastFractureBuffers commands = { 3, 0, command, nullptr };
        NvBlastActorApplyFracture(nullptr, actor, &commands, myLog, nullptr);
        EXPECT_FALSE(NvBlastActorIsSplitRequired(actor, messageLog));

        scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, myLog));
        std::vector<NvBlastActor*> newActors(NvBlastActorGetMaxActorCountForSplit(actor, myLog));
        NvBlastActorSplitEvent result;
        result.deletedActor = nullptr;
        result.newActors = newActors.data();
        size_t newActorsCount = NvBlastActorSplit(&result, actor, static_cast<uint32_t>(newActors.size()), scratch.data(), myLog, nullptr);

        EXPECT_EQ(0, newActorsCount);
        EXPECT_EQ(nullptr, result.deletedActor);

        EXPECT_EQ(1, NvBlastActorGetVisibleChunkCount(actor, myLog));
        uint32_t chunkIndex;
        NvBlastActorGetVisibleChunkIndices(&chunkIndex, 1, actor, myLog);
        EXPECT_EQ(0, chunkIndex);
    }

    // break 1 bond (expect no split)
    {
        NvBlastBondFractureData command[] =
        {
            { 0, 0, 2, 10.0f },
        };

        NvBlastFractureBuffers commands = { 1, 0, command, nullptr };
        NvBlastActorApplyFracture(nullptr, actor, &commands, myLog, nullptr);
        EXPECT_TRUE(NvBlastActorIsSplitRequired(actor, messageLog));

        scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, myLog));
        std::vector<NvBlastActor*> newActors(NvBlastActorGetMaxActorCountForSplit(actor, myLog));
        NvBlastActorSplitEvent result;
        result.deletedActor = nullptr;
        result.newActors = newActors.data();
        size_t newActorsCount = NvBlastActorSplit(&result, actor, static_cast<uint32_t>(newActors.size()), scratch.data(), myLog, nullptr);

        EXPECT_EQ(0, newActorsCount);
        EXPECT_EQ(nullptr, result.deletedActor);

        EXPECT_EQ(1, NvBlastActorGetVisibleChunkCount(actor, myLog));
        uint32_t chunkIndex;
        NvBlastActorGetVisibleChunkIndices(&chunkIndex, 1, actor, myLog);
        EXPECT_EQ(0, chunkIndex);
    }

    // split in 4
    std::vector<NvBlastActor*> actors;
    {
        NvBlastBondFractureData command[] =
        {
            { 0, 0, 1, 10.0f },
            { 0, 1, 2, 10.0f },
            { 0, 2, 3, 10.0f }
        };

        NvBlastFractureBuffers commands = { 3, 0, command, nullptr };
        NvBlastActorApplyFracture(nullptr, actor, &commands, myLog, nullptr);
        EXPECT_TRUE(NvBlastActorIsSplitRequired(actor, messageLog));

        scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, myLog));
        std::vector<NvBlastActor*> newActors(NvBlastActorGetMaxActorCountForSplit(actor, myLog));
        NvBlastActorSplitEvent result;
        result.deletedActor = nullptr;
        result.newActors = newActors.data();
        size_t newActorsCount = NvBlastActorSplit(&result, actor, static_cast<uint32_t>(newActors.size()), scratch.data(), myLog, nullptr);

        EXPECT_EQ(4, newActorsCount);
        EXPECT_EQ(actor, result.deletedActor);

        actors.insert(actors.begin(), result.newActors, result.newActors + newActorsCount);
    }

    // damage chunk's health only (expect no split)
    {
        for (NvBlastActor* actor : actors)
        {
            uint32_t chunkToDamage;
            NvBlastActorGetVisibleChunkIndices(&chunkToDamage, 1, actor, myLog);

            NvBlastChunkFractureData command[] =
            {
                { 0, chunkToDamage, 0.9f },
            };

            NvBlastFractureBuffers commands = { 0, 1, nullptr, command };
            NvBlastActorApplyFracture(nullptr, actor, &commands, myLog, nullptr);
            EXPECT_FALSE(NvBlastActorIsSplitRequired(actor, messageLog));

            scratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, myLog));
            std::vector<NvBlastActor*> newActors(NvBlastActorGetMaxActorCountForSplit(actor, myLog));
            NvBlastActorSplitEvent result;
            result.deletedActor = nullptr;
            result.newActors = newActors.data();
            size_t newActorsCount = NvBlastActorSplit(&result, actor, static_cast<uint32_t>(newActors.size()), scratch.data(), myLog, nullptr);

            EXPECT_EQ(0, newActorsCount);
            EXPECT_EQ(nullptr, result.deletedActor);

            EXPECT_EQ(1, NvBlastActorGetVisibleChunkCount(actor, myLog));
            uint32_t chunkIndex;
            NvBlastActorGetVisibleChunkIndices(&chunkIndex, 1, actor, myLog);
            EXPECT_EQ(chunkToDamage, chunkIndex);
        }
    }

    for (NvBlastActor* actor : actors)
    {
        NvBlastActorDeactivate(actor, myLog);
    }

    alignedFree(family);
    alignedFree(asset);

    EXPECT_NO_WARNING;
}

#if NV_WINDOWS_FAMILY 
#include <windows.h>
TEST_F(APITest,CExportsNoNameMangling)
{
    
    //
    // tests the lib-link-free approach using unmangled names (extern "C")
    //

    const char* dllName = "NvBlast.dll";

    HMODULE dllHandle = LoadLibrary(TEXT(dllName));
    DWORD error = GetLastError();
    ASSERT_TRUE(dllHandle != nullptr);


    // Asset functions
    typedef size_t(*NvBlastGetRequiredScratchForCreateAsset)(const NvBlastAssetDesc* desc);
    typedef size_t(*NvBlastGetAssetMemorySize)(const NvBlastAssetDesc* desc);
    typedef NvBlastAsset*(*NvBlastCreateAsset)(void* mem, const NvBlastAssetDesc* desc, void* scratch, NvBlastLog logFn);

    NvBlastGetRequiredScratchForCreateAsset assetCreateRequiredScratch = (NvBlastGetRequiredScratchForCreateAsset)GetProcAddress(dllHandle, TEXT("NvBlastGetRequiredScratchForCreateAsset"));
    ASSERT_TRUE(assetCreateRequiredScratch != nullptr);

    NvBlastGetAssetMemorySize assetGetMemorySize = (NvBlastGetAssetMemorySize)GetProcAddress(dllHandle, TEXT("NvBlastGetAssetMemorySize"));
    ASSERT_TRUE(assetGetMemorySize != nullptr);

    NvBlastCreateAsset assetCreate = (NvBlastCreateAsset)GetProcAddress(dllHandle, TEXT("NvBlastCreateAsset"));
    ASSERT_TRUE(assetCreate != nullptr);

    // Family functions
    typedef NvBlastFamily* (*NvBlastAssetCreateFamily)(void* mem, const NvBlastAsset* asset, NvBlastLog logFn);
    typedef size_t(*NVBLASTASSETGETFAMILYMEMORYSIZE)(const NvBlastAsset* asset);

    NVBLASTASSETGETFAMILYMEMORYSIZE familyGetMemorySize = (NVBLASTASSETGETFAMILYMEMORYSIZE)GetProcAddress(dllHandle, TEXT("NvBlastAssetGetFamilyMemorySize"));
    ASSERT_TRUE(familyGetMemorySize != nullptr);

    NvBlastAssetCreateFamily familyCreate = (NvBlastAssetCreateFamily)GetProcAddress(dllHandle, TEXT("NvBlastAssetCreateFamily"));
    ASSERT_TRUE(familyCreate != nullptr);


    // Actor functions
    typedef size_t(*NvBlastFamilyGetRequiredScratchForCreateFirstActor)(const NvBlastFamily* family);
    typedef NvBlastActor* (*NvBlastFamilyCreateFirstActor)(NvBlastFamily* family, const NvBlastActorDesc* desc, void* scratch, NvBlastLog logFn);
    typedef bool(*NVBLASTACTORDEACTIVATE)(NvBlastActor* actor);

    NvBlastFamilyGetRequiredScratchForCreateFirstActor actorcreaterequiredscratch = (NvBlastFamilyGetRequiredScratchForCreateFirstActor)GetProcAddress(dllHandle, TEXT("NvBlastFamilyGetRequiredScratchForCreateFirstActor"));
    ASSERT_TRUE(actorcreaterequiredscratch != nullptr);

    NvBlastFamilyCreateFirstActor actorCreate = (NvBlastFamilyCreateFirstActor)GetProcAddress(dllHandle, TEXT("NvBlastFamilyCreateFirstActor"));
    ASSERT_TRUE(actorCreate != nullptr);

    NVBLASTACTORDEACTIVATE actorRelease = (NVBLASTACTORDEACTIVATE)GetProcAddress(dllHandle, TEXT("NvBlastActorDeactivate"));
    ASSERT_TRUE(actorRelease != nullptr);


    const NvBlastChunkDesc c_chunks[] =
    {
        // centroid           volume parent idx     flags                         ID
        { {0.0f, 0.0f, 0.0f}, 0.0f, UINT32_MAX, NvBlastChunkDesc::NoFlags, 0 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 0 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 0 },
        { {0.0f, 0.0f, 0.0f}, 0.0f, 0, NvBlastChunkDesc::SupportFlag, 0 },
    };

    NvBlastAssetDesc assetDesc;
    assetDesc.bondCount = 0;
    assetDesc.bondDescs = nullptr;
    assetDesc.chunkCount = 4;
    assetDesc.chunkDescs = c_chunks;

    NvBlastAsset* asset;
    {
        size_t requiredsize = assetCreateRequiredScratch(&assetDesc);
        std::vector<char>scratch(requiredsize);
        void* mem = alignedZeroedAlloc(assetGetMemorySize(&assetDesc));
        asset = assetCreate(mem, &assetDesc, scratch.data(), myLog);
        ASSERT_TRUE(asset != nullptr);
    }

    void* fmem = alignedZeroedAlloc(familyGetMemorySize(asset));
    NvBlastFamily* family = familyCreate(fmem, asset, myLog);

    {
        NvBlastActorDesc actorD;
        actorD.initialBondHealths = actorD.initialSupportChunkHealths = nullptr;
        actorD.uniformInitialBondHealth = actorD.uniformInitialLowerSupportChunkHealth = 1.0f;

        size_t requiredsize = actorcreaterequiredscratch(family);
        std::vector<char>scratch(requiredsize);
        NvBlastActor* actor = actorCreate(family, &actorD, scratch.data(), myLog);
        ASSERT_TRUE(actor != nullptr);

        ASSERT_TRUE(actorRelease(actor));
    }

    alignedFree(family);
    alignedFree(asset);

    EXPECT_NO_WARNING;
}
#endif

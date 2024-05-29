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


#include "BlastBasePerfTest.h"
#include "TestAssets.h"
#include "NvBlastExtDamageShaders.h"
#include <memory>


static void blast
(
    std::set<NvBlastActor*>& actorsToDamage,
    GeneratorAsset* testAsset,
    GeneratorAsset::Vec3 localPos,
    float minRadius, float maxRadius,
    float compressiveDamage,
    NvBlastTimers& timers
)
{
    std::vector<NvBlastChunkFractureData> chunkEvents; /* num lower-support chunks + bonds */
    std::vector<NvBlastBondFractureData> bondEvents; /* num lower-support chunks + bonds */
    chunkEvents.resize(testAsset->solverChunks.size());
    bondEvents.resize(testAsset->solverBonds.size());

    NvBlastExtRadialDamageDesc damage[] = {
        compressiveDamage,
        { localPos.x, localPos.y, localPos.z },
        minRadius,
        maxRadius
    };

    NvBlastExtProgramParams programParams =
    {
        damage,
        nullptr
    };

    NvBlastDamageProgram program = {
        NvBlastExtFalloffGraphShader,
        nullptr
    };

    std::vector<char> splitScratch;
    std::vector<NvBlastActor*> newActors(testAsset->solverChunks.size());

    size_t totalNewActorsCount = 0;
    for (std::set<NvBlastActor*>::iterator k = actorsToDamage.begin(); k != actorsToDamage.end();)
    {
        NvBlastActor* actor = *k;

        NvBlastFractureBuffers events = { (uint32_t)bondEvents.size(), (uint32_t)chunkEvents.size(), bondEvents.data(), chunkEvents.data() };

        NvBlastActorGenerateFracture(&events, actor, program, &programParams, nullptr, &timers);
        NvBlastActorApplyFracture(&events, actor, &events, nullptr, &timers);

        bool removeActor = false;

        if (events.bondFractureCount + events.chunkFractureCount > 0)
        {
            splitScratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, nullptr));
            NvBlastActorSplitEvent result;
            result.deletedActor = nullptr;
            result.newActors = &newActors[totalNewActorsCount];
            const size_t bufferSize = newActors.size() - totalNewActorsCount;
            const size_t newActorsCount = NvBlastActorSplit(&result, actor, (uint32_t)bufferSize, splitScratch.data(), nullptr, &timers);
            totalNewActorsCount += newActorsCount;
            removeActor = newActorsCount > 0;
        }

        if (removeActor)
        {
            k = actorsToDamage.erase(k);
        }
        else
        {
            ++k;
        }
    }

    for (size_t i = 0; i < totalNewActorsCount; ++i)
    {
        actorsToDamage.insert(newActors[i]);
    }
}

typedef BlastBasePerfTest<NvBlastMessage::Warning, 1> BlastBasePerfTestStrict;

class PerfTest : public BlastBasePerfTestStrict
{
public:
    void damageLeafSupportActors(const char* testName, uint32_t assetCount, uint32_t familyCount,   uint32_t damageCount)
    {
        const float relativeDamageRadius = 0.2f;
        const float compressiveDamage = 1.0f;
        const uint32_t minChunkCount = 100;
        const uint32_t maxChunkCount = 10000;

        srand(0);

        for (uint32_t assetNum = 0; assetNum < assetCount; ++assetNum)
        {
            GeneratorAsset cube;
            NvBlastAssetDesc desc;
            generateRandomCube(cube, desc, minChunkCount, maxChunkCount);

            {
                std::vector<char> scratch;
                scratch.resize((size_t)NvBlastGetRequiredScratchForCreateAsset(&desc, messageLog));
                void* mem = alignedZeroedAlloc(NvBlastGetAssetMemorySize(&desc, messageLog));
                NvBlastAsset* asset = NvBlastCreateAsset(mem, &desc, scratch.data(), messageLog);
                EXPECT_TRUE(asset != nullptr);

                // Generate familes
                for (uint32_t familyNum = 0; familyNum < familyCount; ++familyNum)
                {
                    // create actor
                    NvBlastActorDesc actorDesc;
                    actorDesc.initialBondHealths = nullptr;
                    actorDesc.uniformInitialBondHealth = 1.0f;
                    actorDesc.initialSupportChunkHealths = nullptr;
                    actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
                    void* mem = alignedZeroedAlloc(NvBlastAssetGetFamilyMemorySize(asset, messageLog));
                    NvBlastFamily* family = NvBlastAssetCreateFamily(mem, asset, messageLog);
                    scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, messageLog));
                    EXPECT_TRUE(family != nullptr);
                    NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), messageLog);
                    EXPECT_TRUE(actor != nullptr);

                    // Generate damage
                    std::set<NvBlastActor*> actors;
                    actors.insert(actor);
                    for (uint32_t damageNum = 0; damageNum < damageCount; ++damageNum)
                    {
                        GeneratorAsset::Vec3 localPos = cube.extents*GeneratorAsset::Vec3((float)rand() / RAND_MAX - 0.5f, (float)rand() / RAND_MAX - 0.5f, (float)rand() / RAND_MAX - 0.5f);

                        NvBlastTimers timers;
                        NvBlastTimersReset(&timers);
                        blast(actors, &cube, localPos, relativeDamageRadius, relativeDamageRadius*1.2f, compressiveDamage, timers);
                        const std::string timingName = std::string(testName) + " asset " + std::to_string(assetNum) + " family " + std::to_string(familyNum) + " damage " + std::to_string(damageNum);
                        BlastBasePerfTestStrict::reportData(timingName + " material", timers.material);
                        BlastBasePerfTestStrict::reportData(timingName + " fracture", timers.fracture);
                        BlastBasePerfTestStrict::reportData(timingName + " island", timers.island);
                        BlastBasePerfTestStrict::reportData(timingName + " partition", timers.partition);
                        BlastBasePerfTestStrict::reportData(timingName + " visibility", timers.visibility);
                    }

                    // Release remaining actors
                    std::for_each(actors.begin(), actors.end(), [](NvBlastActor* a){ NvBlastActorDeactivate(a, messageLog); });
                    actors.clear();

                    alignedFree(family);
                }

                // Release asset data
                alignedFree(asset);
            }
        }
    }
};

#if 0
// Tests
TEST_F(PerfTest, DamageLeafSupportActorsTestVisibility)
{
    const int trialCount = 1000;
    std::cout << "Trial (of " << trialCount << "): ";
    for (int trial = 1; trial <= trialCount; ++trial)
    {
        if (trial % 100 == 0)
        {
            std::cout << trial << ".. ";
            std::cout.flush();
        }
        damageLeafSupportActors(test_info_->name(), 4, 4, 5);
    }
    std::cout << "done." << std::endl;
}
#endif
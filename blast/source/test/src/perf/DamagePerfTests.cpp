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
#include "NvBlastExtDamageShaders.h"
#include "NvBlastExtSerialization.h"
#include "NvBlastTime.h"
#include "NvVec3.h"
#include "NvBounds3.h"
#include <memory>
#include <random>
#include <cstdio>

using namespace Nv::Blast;
using namespace nvidia;

static void blast
(
    std::set<NvBlastActor*>& actorsToDamage,
    GeneratorAsset* testAsset,
    NvBlastExtDamageAccelerator* accelerator,
    GeneratorAsset::Vec3 localPos,
    float minRadius, float maxRadius,
    float compressiveDamage,
    std::vector<uint32_t>& history,
    NvBlastTimers& timers)
{
    std::vector<NvBlastChunkFractureData> chunkEvents; /* num lower-support chunks + bonds */
    std::vector<NvBlastBondFractureData> bondEvents; /* num lower-support chunks + bonds */
    chunkEvents.resize(testAsset->solverChunks.size());
    bondEvents.resize(testAsset->solverBonds.size());

    NvBlastExtRadialDamageDesc damage = {
        compressiveDamage,
        { localPos.x, localPos.y, localPos.z },
        minRadius,
        maxRadius
    };

    NvBlastExtMaterial material;

    NvBlastExtProgramParams programParams =
    {
        &damage,
        &material,
        accelerator
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
        NvBlastActorApplyFracture(nullptr, actor, &events, nullptr, &timers);

        bool removeActor = false;

        if (events.bondFractureCount + events.chunkFractureCount > 0)
        {
            history.push_back(events.bondFractureCount + events.chunkFractureCount);

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


struct PerfResults
{
    int64_t totalTime;
    int64_t createTime;
};

class PerfTest : public BlastBasePerfTestStrict
{
public:

  NvBlastAsset* loadAsset(const char* path, ExtSerialization* ser)
  {
    std::ifstream infileStream(path, std::ios::binary);
    if (!infileStream.is_open())
    {
      return nullptr;
    }
    const std::vector<char> inBuffer((std::istreambuf_iterator<char>(infileStream)), std::istreambuf_iterator<char>());
    infileStream.close();

    NvBlastAsset* asset = static_cast<NvBlastAsset*>(ser->deserializeFromBuffer(inBuffer.data(), inBuffer.size()));

    return asset;
  }

    PerfResults damageLeafSupportActors(const char* testName, uint32_t assetCount, uint32_t familyCount, uint32_t damageCount, int accelType, std::vector<uint32_t>& history)
    {
        PerfResults results;
        results.totalTime = 0;
        results.createTime = 0;

        const float compressiveDamage = 1.0f;
        const uint32_t minChunkCount = 1000;
        const uint32_t maxChunkCount = 100000;

        srand(0);

        for (uint32_t assetNum = 0; assetNum < assetCount; ++assetNum)
        {
            GeneratorAsset cube;
            NvBlastAssetDesc desc;
            generateRandomCube(cube, desc, minChunkCount, maxChunkCount);

            {
                std::vector<char> scratch;
                nvidia::NvBounds3 bounds = nvidia::NvBounds3::empty();
#if 1
                scratch.resize((size_t)NvBlastGetRequiredScratchForCreateAsset(&desc, messageLog));
                void* mem = alignedZeroedAlloc(NvBlastGetAssetMemorySize(&desc, messageLog));
                NvBlastAsset* asset = NvBlastCreateAsset(mem, &desc, scratch.data(), messageLog);
                EXPECT_TRUE(asset != nullptr);
                bounds = nvidia::NvBounds3::centerExtents(nvidia::NvVec3(0, 0, 0), nvidia::NvVec3(cube.extents.x, cube.extents.y, cube.extents.z));
#else
                // load asset
                NvBlastAsset* asset = nullptr;
                ExtSerialization* ser = NvBlastExtSerializationCreate();
                for (int s = 0; s < 5 && !asset; s++)
                {
                    asset = loadAsset(&"../../../../../test/assets/table.blast"[s * 3], ser);
                }
                EXPECT_TRUE(asset != nullptr);
                ser->release();
                uint32_t bc = NvBlastAssetGetBondCount(asset, messageLog);
                const NvBlastBond* bonds = NvBlastAssetGetBonds(asset, messageLog);
                for (uint32_t i = 0; i < bc; i++)
                {
                    bounds.include(reinterpret_cast<const nvidia::NvVec3&>(bonds[i].centroid));
                }
#endif

                Nv::Blast::Time t;
                NvBlastExtDamageAccelerator* accelerator = NvBlastExtDamageAcceleratorCreate(asset, accelType);
                results.createTime += t.getElapsedTicks();

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
                        GeneratorAsset::Vec3 localPos = GeneratorAsset::Vec3((float)rand() / RAND_MAX - 0.5f, (float)rand() / RAND_MAX - 0.5f, (float)rand() / RAND_MAX - 0.5f) * 2;
                        localPos.x *= bounds.getExtents().x;
                        localPos.y *= bounds.getExtents().y;
                        localPos.z *= bounds.getExtents().z;
                        const float relativeDamageRadius = (float)rand() / RAND_MAX * bounds.getExtents().maxElement();

                        NvBlastTimers timers;
                        NvBlastTimersReset(&timers);
                        blast(actors, &cube, accelerator, localPos, relativeDamageRadius, relativeDamageRadius*1.2f, compressiveDamage, history, timers);
                        const std::string timingName = std::string(testName) + " asset " + std::to_string(assetNum) + " family " + std::to_string(familyNum) + " damage " + std::to_string(damageNum) + " accel " + std::to_string(accelType);
                        BlastBasePerfTestStrict::reportData(timingName + " material", timers.material);
                        history.push_back((uint32_t)actors.size());
                        results.totalTime += timers.material;
                        history.push_back(0); // separator
                    }

                    // Release remaining actors
                    std::for_each(actors.begin(), actors.end(), [](NvBlastActor* a) { NvBlastActorDeactivate(a, messageLog); });
                    actors.clear();

                    alignedFree(family);
                }

                if (accelerator)
                    accelerator->release();

                // Release asset data
                alignedFree(asset);
            }
        }

        return results;
    }
};


// Tests
TEST_F(PerfTest, DISABLED_DamageRadialSimple)
{
    const int trialCount = 10;
    std::cout << "Trial (of " << trialCount << "): ";
    for (int trial = 1; trial <= trialCount; ++trial)
    {
        if (trial % 100 == 0)
        {
            std::cout << trial << ".. ";
            std::cout.flush();
        }
        std::vector<uint32_t> history1, history2;

        uint32_t assetCount = 4;
        uint32_t familyCount = 4;
        uint32_t damageCount = 4;

        PerfResults results0 = damageLeafSupportActors(test_info_->name(), assetCount, familyCount, damageCount, 0, history1);
        BlastBasePerfTestStrict::reportData("DamageRadialSimple total0 " , results0.totalTime);
        BlastBasePerfTestStrict::reportData("DamageRadialSimple create0 ", results0.createTime);
        PerfResults results1 = damageLeafSupportActors(test_info_->name(), assetCount, familyCount, damageCount, 1, history2);
        BlastBasePerfTestStrict::reportData("DamageRadialSimple total1 ", results1.totalTime);
        BlastBasePerfTestStrict::reportData("DamageRadialSimple create1 ", results1.createTime);

        EXPECT_TRUE(history1 == history2);
    }
    std::cout << "done." << std::endl;
}

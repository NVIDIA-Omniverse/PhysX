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
#include "AssetGenerator.h"

#include <iostream>
#include <memory>
#include "TaskDispatcher.h"

#include "NvBlastActor.h"
#include "NvBlastExtDamageShaders.h"


typedef std::function<void(const Nv::Blast::Actor&, NvBlastLog)> ActorTestFunction;
typedef std::function<void(std::vector<NvBlastActor*>&, NvBlastLog)> PostDamageTestFunction;


static void blast(std::set<NvBlastActor*>& actorsToDamage, GeneratorAsset* testAsset, GeneratorAsset::Vec3 localPos, float minRadius, float maxRadius, float compressiveDamage)
{
    std::vector<NvBlastChunkFractureData> chunkEvents; /* num lower-support chunks + bonds */
    std::vector<NvBlastBondFractureData> bondEvents; /* num lower-support chunks + bonds */
    chunkEvents.resize(testAsset->solverChunks.size());
    bondEvents.resize(testAsset->solverBonds.size());

    NvBlastFractureBuffers events = { static_cast<uint32_t>(bondEvents.size()), static_cast<uint32_t>(chunkEvents.size()), bondEvents.data(), chunkEvents.data() };

    std::vector<float> scratch(chunkEvents.size() + bondEvents.size(), 0.0f);

    std::vector<char> splitScratch;
    std::vector<NvBlastActor*> newActorsBuffer(testAsset->solverChunks.size());

    NvBlastExtRadialDamageDesc damage = {
        compressiveDamage,
        { localPos.x, localPos.y, localPos.z },
        minRadius,
        maxRadius
    };

    NvBlastExtProgramParams programParams =
    {
        &damage,
        nullptr
    };

    NvBlastDamageProgram program = {
        NvBlastExtFalloffGraphShader,
        nullptr
    };

    size_t totalNewActorsCount = 0;
    for (std::set<NvBlastActor*>::iterator k = actorsToDamage.begin(); k != actorsToDamage.end();)
    {
        NvBlastActor* actor = *k;

        NvBlastActorGenerateFracture(&events, actor, program, &programParams, nullptr, nullptr);
        NvBlastActorApplyFracture(&events, actor, &events, nullptr, nullptr);

        bool removeActor = false;

        if (events.bondFractureCount + events.chunkFractureCount > 0)
        {
            NvBlastActorSplitEvent splitEvent;
            splitEvent.newActors = &newActorsBuffer.data()[totalNewActorsCount];
            uint32_t newActorSize = (uint32_t)(newActorsBuffer.size() - totalNewActorsCount);

            splitScratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, nullptr));
            const size_t newActorsCount = NvBlastActorSplit(&splitEvent, actor, newActorSize, splitScratch.data(), nullptr, nullptr);
            totalNewActorsCount += newActorsCount;
            removeActor = splitEvent.deletedActor != NULL;
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
        actorsToDamage.insert(newActorsBuffer[i]);
    }
}


template<int FailLevel, int Verbosity>
class MultithreadingTest : public BlastBaseTest<FailLevel, Verbosity>
{
public:
    MultithreadingTest()
    {

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

    static void testActorVisibleChunks(const Nv::Blast::Actor& actor, NvBlastLog)
    {
        const Nv::Blast::Asset& asset = *actor.getAsset();
        const NvBlastChunk* chunks = asset.getChunks();

        if (actor.isSubSupportChunk())
        {
            EXPECT_EQ(1, actor.getVisibleChunkCount());

            const uint32_t firstVisibleChunkIndex = (uint32_t)Nv::Blast::Actor::VisibleChunkIt(actor);

            EXPECT_EQ(actor.getIndex() - asset.m_graph.m_nodeCount, firstVisibleChunkIndex - asset.m_firstSubsupportChunkIndex);

            // Make sure the visible chunk is subsupport
            // Array of support flags
            std::vector<bool> isSupport(asset.m_chunkCount, false);
            for (uint32_t i = 0; i < asset.m_graph.m_nodeCount; ++i)
            {
                isSupport[asset.m_graph.getChunkIndices()[i]] = true;
            }

            // Climb hierarchy to find support chunk
            uint32_t chunkIndex = firstVisibleChunkIndex;
            while (chunkIndex != Nv::Blast::invalidIndex<uint32_t>())
            {
                if (isSupport[chunkIndex])
                {
                    break;
                }
                chunkIndex = chunks[chunkIndex].parentChunkIndex;
            }

            EXPECT_FALSE(Nv::Blast::isInvalidIndex(chunkIndex));
        }
        else
        {
            // Array of visibility flags
            std::vector<bool> isVisible(asset.m_chunkCount, false);
            for (Nv::Blast::Actor::VisibleChunkIt i = actor; (bool)i; ++i)
            {
                isVisible[(uint32_t)i] = true;
            }

            // Mark visible nodes representing graph chunks
            std::vector<bool> visibleChunkFound(asset.m_chunkCount, false);

            // Make sure every graph chunk is represented by a visible chunk
            for (Nv::Blast::Actor::GraphNodeIt i = actor; (bool)i; ++i)
            {
                const uint32_t graphNodeIndex = (uint32_t)i;
                uint32_t chunkIndex = asset.m_graph.getChunkIndices()[graphNodeIndex];
                // Climb hierarchy to find visible chunk
                while (chunkIndex != Nv::Blast::invalidIndex<uint32_t>())
                {
                    // Check that chunk owners are accurate
                    EXPECT_EQ(actor.getIndex(), actor.getFamilyHeader()->getChunkActorIndices()[chunkIndex]);
                    if (isVisible[chunkIndex])
                    {
                        visibleChunkFound[chunkIndex] = true;
                        break;
                    }
                    chunkIndex = chunks[chunkIndex].parentChunkIndex;
                }
                EXPECT_FALSE(Nv::Blast::isInvalidIndex(chunkIndex));
            }

            // Check that all visible chunks are accounted for
            for (uint32_t i = 0; i < asset.m_chunkCount; ++i)
            {
                EXPECT_EQ(visibleChunkFound[i], isVisible[i]);
            }

            // Make sure that, if all siblings are intact, they are invisible
            for (uint32_t i = 0; i < asset.m_chunkCount; ++i)
            {
                bool allIntact = true;
                bool noneVisible = true;
                if (chunks[i].firstChildIndex < asset.getUpperSupportChunkCount()) // Do not check subsupport
                {
                    for (uint32_t j = chunks[i].firstChildIndex; j < chunks[i].childIndexStop; ++j)
                    {
                        allIntact = allIntact && actor.getFamilyHeader()->getChunkActorIndices()[j] == actor.getIndex();
                        noneVisible = noneVisible && !isVisible[j];
                    }
                    EXPECT_TRUE(!allIntact || noneVisible);
                }
            }
        }
    }

    class DamageActorTask : public TaskDispatcher::Task
    {
    public:
        DamageActorTask(NvBlastActor* actor, GeneratorAsset* asset, GeneratorAsset::Vec3 localPos, float minRadius, float maxRadius, float compressiveDamage, ActorTestFunction testFunction)
            : m_asset(asset)
            , m_localPos(localPos)
            , m_minRadius(minRadius)
            , m_maxRadius(maxRadius)
            , m_compressiveDamage(compressiveDamage)
            , m_testFunction(testFunction)
        {
            m_actors.insert(actor);
        }

        virtual void process()
        {
            blast(m_actors, m_asset, m_localPos, m_minRadius, m_maxRadius, m_compressiveDamage);

            // Test individual actors
            if (m_testFunction != nullptr)
            {
                for (std::set<NvBlastActor*>::iterator k = m_actors.begin(); k != m_actors.end(); ++k)
                {
                    m_testFunction(*static_cast<Nv::Blast::Actor*>(*k), messageLog);
                }
            }
        }

        const std::set<NvBlastActor*>& getResult() const { return m_actors; }

    private:
        std::set<NvBlastActor*>          m_actors;
        GeneratorAsset*                  m_asset;
        GeneratorAsset::Vec3             m_localPos;
        float                            m_minRadius;
        float                            m_maxRadius;
        float                            m_compressiveDamage;
        ActorTestFunction                m_testFunction;

        std::vector<NvBlastActor*> m_resultActors;
    };

    void damageLeafSupportActorsParallelized
    (
        uint32_t assetCount,
        uint32_t minChunkCount,
        uint32_t damageCount,
        uint32_t threadCount,
        ActorTestFunction actorTestFunction,
        PostDamageTestFunction postDamageTestFunction
    )
    {
        const float relativeDamageRadius = 0.05f;
        const float compressiveDamage = 1.0f;

        srand(0);

        std::cout << "Asset # (out of " << assetCount << "): ";
        for (uint32_t assetNum = 0; assetNum < assetCount; ++assetNum)
        {
            std::cout << assetNum + 1 << ".. ";
            CubeAssetGenerator::Settings settings;
            settings.extents = GeneratorAsset::Vec3(1, 1, 1);
            CubeAssetGenerator::DepthInfo depthInfo;
            depthInfo.slicesPerAxis = GeneratorAsset::Vec3(1, 1, 1);
            depthInfo.flag = NvBlastChunkDesc::Flags::NoFlags;
            settings.depths.push_back(depthInfo);
            uint32_t chunkCount = 1;
            while (chunkCount < minChunkCount)
            {
                uint32_t chunkMul;
                do
                {
                    depthInfo.slicesPerAxis = GeneratorAsset::Vec3((float)(1 + rand() % 4), (float)(1 + rand() % 4), (float)(1 + rand() % 4));
                    chunkMul = (uint32_t)(depthInfo.slicesPerAxis.x * depthInfo.slicesPerAxis.y * depthInfo.slicesPerAxis.z);
                } while (chunkMul == 1);
                chunkCount *= chunkMul;
                settings.depths.push_back(depthInfo);
                settings.extents = settings.extents * depthInfo.slicesPerAxis;
            }
            settings.depths.back().flag = NvBlastChunkDesc::SupportFlag;    // Leaves are support

            // Make largest direction unit size
            settings.extents = settings.extents * (1.0f / std::max(settings.extents.x, std::max(settings.extents.y, settings.extents.z)));

            // Create asset
            GeneratorAsset testAsset;
            CubeAssetGenerator::generate(testAsset, settings);

            NvBlastAssetDesc desc;
            desc.chunkDescs = &testAsset.solverChunks[0];
            desc.chunkCount = (uint32_t)testAsset.solverChunks.size();
            desc.bondDescs = testAsset.solverBonds.data();
            desc.bondCount = (uint32_t)testAsset.solverBonds.size();

            std::vector<char> scratch;
            scratch.resize((size_t)NvBlastGetRequiredScratchForCreateAsset(&desc, messageLog));
            void* mem = alloc(NvBlastGetAssetMemorySize(&desc, messageLog));
            NvBlastAsset* asset = NvBlastCreateAsset(mem, &desc, scratch.data(), messageLog);
            EXPECT_TRUE(asset != nullptr);

            NvBlastActorDesc actorDesc;
            actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
            actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
            void* fmem = alloc(NvBlastAssetGetFamilyMemorySize(asset, messageLog));
            NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, nullptr); // Using zeroingAlloc in case actorTest compares memory blocks
            scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, messageLog));
            NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), messageLog);
            EXPECT_TRUE(actor != nullptr);

            // Run parallelized damage through TaskDispatcher
            std::set<NvBlastActor*> resultActors;
            {
                uint32_t damageNum = 0;

                // create DamageActorTask and it to dispatcher helper function
                auto addDamageTaskFunction = [&](TaskDispatcher& dispatcher, NvBlastActor* actor)
                {
                    GeneratorAsset::Vec3 localPos = settings.extents*GeneratorAsset::Vec3((float)rand() / RAND_MAX - 0.5f, (float)rand() / RAND_MAX - 0.5f, (float)rand() / RAND_MAX - 0.5f);
                    auto newTask = std::unique_ptr<DamageActorTask>(new DamageActorTask(actor, &testAsset, localPos, relativeDamageRadius, relativeDamageRadius*1.2f, compressiveDamage, actorTestFunction));
                    dispatcher.addTask(std::move(newTask));
                };

                // on task finished function for dispatcher (main thread)
                TaskDispatcher::OnTaskFinishedFunction onTaskFinishedFunction = [&](TaskDispatcher& dispatcher, std::unique_ptr<TaskDispatcher::Task> task) {
                    const DamageActorTask* damageTask = static_cast<const DamageActorTask*>(task.get());
                    const std::set<NvBlastActor*>& actors = damageTask->getResult();
                    for (NvBlastActor* actor : actors)
                    {
                        if (damageNum >= damageCount)
                        {
                            resultActors.insert(actor);
                        }
                        else
                        {
                            damageNum++;
                            addDamageTaskFunction(dispatcher, actor);
                        }
                    }
                };

                // create dispatcher, add first task and run 
                TaskDispatcher dispatcher(threadCount, onTaskFinishedFunction);
                addDamageTaskFunction(dispatcher, actor);
                dispatcher.process();
            }

            // Test fractured actor set
            if (postDamageTestFunction)
            {
                std::vector<NvBlastActor*> actorArray(resultActors.begin(), resultActors.end());
                postDamageTestFunction(actorArray, messageLog);
            }

            // Release remaining actors
            for (std::set<NvBlastActor*>::iterator k = resultActors.begin(); k != resultActors.end(); ++k)
            {
                NvBlastActorDeactivate(*k, messageLog);
            }
            resultActors.clear();

            const uint32_t actorCount = NvBlastFamilyGetActorCount(family, messageLog);
            EXPECT_TRUE(actorCount == 0);

            free(family);

            // Release asset data
            free(asset);
        }
        std::cout << "done.\n";
    }
};


// Specializations
typedef MultithreadingTest<NvBlastMessage::Error, 1> MultithreadingTestAllowWarnings;
typedef MultithreadingTest<NvBlastMessage::Error, 1> MultithreadingTestStrict;


TEST_F(MultithreadingTestStrict, MultithreadingTestDamageLeafSupportActorsTestVisibility)
{
    damageLeafSupportActorsParallelized(1, 1000, 50, 4, testActorVisibleChunks, nullptr);
}

TEST_F(MultithreadingTestStrict, MultithreadingTestDamageLeafSupportActors)
{
    damageLeafSupportActorsParallelized(1, 3000, 1000, 4, nullptr, nullptr);
}

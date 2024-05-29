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

#include <map>
#include <random>
#include <algorithm>

#include "NvBlastActor.h"
#include "NvBlastExtDamageShaders.h"
#include "NvBlastExtLlSerialization.h"
#include "NvBlastExtSerialization.h"


static bool chooseRandomGraphNodes(uint32_t* g, uint32_t count, const Nv::Blast::Actor& actor)
{
    const uint32_t graphNodeCount = actor.getGraphNodeCount();

    if (graphNodeCount < count)
    {
        return false;
    }

    std::vector<uint32_t> graphNodeIndices(graphNodeCount);
    uint32_t* index = graphNodeIndices.data();
    for (Nv::Blast::Actor::GraphNodeIt i = actor; (bool)i ; ++i)
    {
        *index++ = (uint32_t)i;
    }
    struct UserDataSorter
    {
        UserDataSorter(const Nv::Blast::Actor& actor) : m_asset(*actor.getAsset()) {}

        bool    operator () (uint32_t i0, uint32_t i1) const
        {
            const uint32_t c0 = m_asset.m_graph.getChunkIndices()[i0];
            const uint32_t c1 = m_asset.m_graph.getChunkIndices()[i1];
            if (Nv::Blast::isInvalidIndex(c0) || Nv::Blast::isInvalidIndex(c1))
            {
                return c0 < c1;
            }
            return m_asset.getChunks()[c0].userData < m_asset.getChunks()[c1].userData;
        }

        const Nv::Blast::Asset& m_asset;
    } userDataSorter(actor);
    std::sort(graphNodeIndices.data(), graphNodeIndices.data() + graphNodeCount, userDataSorter);

#if 0
    std::vector<uint32_t> descUserData(graphNodeCount);
    for (uint32_t i = 0; i < graphNodeCount; ++i)
    {
        descUserData[i] = actor.getAsset()->m_chunks[actor.getAsset()->m_graph.m_chunkIndices[graphNodeIndices[i]]].userData;
    }
#endif

    uint32_t t = 0;
    uint32_t m = 0;
    for (uint32_t i = 0; i < graphNodeCount && m < count; ++i, ++t)
    {
        NVBLAST_ASSERT(t < graphNodeCount);
        if (t >= graphNodeCount)
        {
            break;
        }
        const float U = (float)rand()/RAND_MAX;    // U is uniform random number in [0,1)
        if ((graphNodeCount - t)*U < count - m)
        {
            g[m++] = graphNodeIndices[i];
        }
    }

    return m == count;
}


static void blast(std::set<NvBlastActor*>& actorsToDamage, GeneratorAsset* testAsset, GeneratorAsset::Vec3 localPos, float minRadius, float maxRadius, float compressiveDamage)
{
    std::vector<NvBlastChunkFractureData> chunkEvents; /* num lower-support chunks + bonds */
    std::vector<NvBlastBondFractureData> bondEvents; /* num lower-support chunks + bonds */
    chunkEvents.resize(testAsset->solverChunks.size());
    bondEvents.resize(testAsset->solverBonds.size());


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
        NvBlastFractureBuffers events = { static_cast<uint32_t>(bondEvents.size()), static_cast<uint32_t>(chunkEvents.size()), bondEvents.data(), chunkEvents.data() };

        NvBlastActorGenerateFracture(&events, actor, program, &programParams, nullptr, nullptr);
        NvBlastActorApplyFracture(&events, actor, &events, nullptr, nullptr);
        const bool isDamaged = NvBlastActorIsSplitRequired(actor, nullptr);
        bool removeActor = false;

        if (events.bondFractureCount + events.chunkFractureCount > 0)
        {
            NvBlastActorSplitEvent splitEvent;
            splitEvent.newActors = &newActorsBuffer.data()[totalNewActorsCount];
            uint32_t newActorSize = (uint32_t)(newActorsBuffer.size() - totalNewActorsCount);

            splitScratch.resize((size_t)NvBlastActorGetRequiredScratchForSplit(actor, nullptr));
            const size_t newActorsCount = NvBlastActorSplit(&splitEvent, actor, newActorSize, splitScratch.data(), nullptr, nullptr);
            EXPECT_TRUE(isDamaged || newActorsCount == 0);
            totalNewActorsCount += newActorsCount;
            removeActor = splitEvent.deletedActor != NULL;
        }
        else
        {
            EXPECT_FALSE(isDamaged);
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
class ActorTest : public BlastBaseTest<FailLevel, Verbosity>
{
public:
    ActorTest()
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

    NvBlastAsset* buildAsset(const NvBlastAssetDesc& desc)
    {
        // fix desc if wrong order or missing coverage first
        NvBlastAssetDesc fixedDesc = desc;
        std::vector<NvBlastChunkDesc> chunkDescs(desc.chunkDescs, desc.chunkDescs + desc.chunkCount);
        std::vector<NvBlastBondDesc> bondDescs(desc.bondDescs, desc.bondDescs + desc.bondCount);
        std::vector<uint32_t> chunkReorderMap(desc.chunkCount);
        std::vector<char> scratch(desc.chunkCount * sizeof(NvBlastChunkDesc));
        NvBlastEnsureAssetExactSupportCoverage(chunkDescs.data(), fixedDesc.chunkCount, scratch.data(), messageLog);
        NvBlastReorderAssetDescChunks(chunkDescs.data(), fixedDesc.chunkCount, bondDescs.data(), fixedDesc.bondCount, chunkReorderMap.data(), true, scratch.data(), messageLog);
        fixedDesc.chunkDescs = chunkDescs.data();
        fixedDesc.bondDescs = bondDescs.empty() ? nullptr : bondDescs.data();

        // create asset
        m_scratch.resize((size_t)NvBlastGetRequiredScratchForCreateAsset(&fixedDesc, messageLog));
        void* mem = alloc(NvBlastGetAssetMemorySize(&fixedDesc, messageLog));
        NvBlastAsset* asset = NvBlastCreateAsset(mem, &fixedDesc, m_scratch.data(), messageLog);
        EXPECT_TRUE(asset != nullptr);
        return asset;
    }

    void buildAssets()
    {
        m_assets.resize(getAssetDescCount());
        for (uint32_t i = 0; i < m_assets.size(); ++i)
        {
            m_assets[i] = buildAsset(g_assetDescs[i]);
        }
    }

    NvBlastActor* instanceActor(const NvBlastAsset& asset)
    {
        NvBlastActorDesc actorDesc;
        actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
        actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
        void* fmem = alloc(NvBlastAssetGetFamilyMemorySize(&asset, nullptr));
        NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, &asset, nullptr);
        std::vector<char> scratch((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, messageLog));
        EXPECT_TRUE(scratch.capacity() > 0);
        NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), messageLog);
        EXPECT_TRUE(actor != nullptr);
        return actor;
    }

    void instanceActors()
    {
        m_actors.resize(m_assets.size());
        for (uint32_t i = 0; i < m_actors.size(); ++i)
        {
            m_actors[i] = instanceActor(*m_assets[i]);
        }
    }

    void releaseActors()
    {
        for (uint32_t i = 0; i < m_actors.size(); ++i)
        {
            NvBlastFamily* family = NvBlastActorGetFamily(m_actors[i], messageLog);

            const bool actorReleaseResult = NvBlastActorDeactivate(m_actors[i], messageLog);
            EXPECT_TRUE(actorReleaseResult);

            free(family);
        }
    }

    void destroyAssets()
    {
        for (uint32_t i = 0; i < m_assets.size(); ++i)
        {
            free(m_assets[i]);
        }
    }

    void instanceAndPartitionRecursively
    (
        const NvBlastAsset& asset,
        bool partitionToSubsupport,
        void (*preSplitTest)(const Nv::Blast::Actor&, NvBlastLog),
        void (*postSplitTest)(const std::vector<Nv::Blast::Actor*>&, uint32_t, uint32_t, bool)
    )
    {
        const Nv::Blast::Asset& solverAsset = *static_cast<const Nv::Blast::Asset*>(&asset);

        std::vector<Nv::Blast::Actor*> actors;
        std::vector<Nv::Blast::Actor*> buffer(NvBlastAssetGetChunkCount(&asset, messageLog));

        // Instance the first actor from the asset
        actors.push_back(static_cast<Nv::Blast::Actor*>(instanceActor(asset)));

        NvBlastFamily* family = NvBlastActorGetFamily(actors[0], messageLog);

        const uint32_t supportChunkCount = NvBlastAssetGetSupportChunkCount(&asset, messageLog);
        const uint32_t leafChunkCount = actors[0]->getAsset()->m_leafChunkCount;

        // Now randomly partition the actors in the array, and keep going until we're down to single support chunks
        bool canFracture = true;

        while (canFracture)
        {
            canFracture = false;

            for (uint32_t actorToPartition = 0; actorToPartition < actors.size(); ++actorToPartition)
            {
                Nv::Blast::Actor* a = (Nv::Blast::Actor*)actors[actorToPartition];
                if (a == nullptr)
                {
                    continue;
                }

                m_scratch.reserve((size_t)NvBlastActorGetRequiredScratchForSplit(a, messageLog));

                if (preSplitTest)
                {
                    preSplitTest(*a, nullptr);
                }

                const bool singleLowerSupportChunk = a->getGraphNodeCount() <= 1;
                uint32_t newActorCount = 0;

                for (int damageNum = 0; newActorCount < 2 && damageNum < 100; ++damageNum)    // Avoid infinite loops
                {
                    if (!singleLowerSupportChunk)
                    {
                        uint32_t g[2];
                        chooseRandomGraphNodes(g, 2, *a);
                        const uint32_t bondIndex = solverAsset.m_graph.findBond(g[0], g[1]);
                        if (bondIndex != Nv::Blast::invalidIndex<uint32_t>())
                        {
                            a->damageBond(g[0], g[1], bondIndex, 100.0f);
                            a->findIslands(m_scratch.data());
                        }
                    }
                    else
                    if (!partitionToSubsupport)
                    {
                        continue;
                    }

                    // Split actor
                    newActorCount = a->partition((Nv::Blast::Actor**)&buffer[0], (uint32_t)buffer.size(), messageLog);

                    if (newActorCount >= 2)
                    {
                        actors[actorToPartition] = nullptr;
                    }
                }

                if (newActorCount > 1)
                {
                    canFracture = true;
                }

                for (uint32_t i = 0; i < newActorCount; ++i)
                {
                    actors.push_back(buffer[i]);
                    buffer[i]->updateVisibleChunksFromGraphNodes();
                }
            }
        }
        
        if (postSplitTest)
        {
            postSplitTest(actors, leafChunkCount, supportChunkCount, partitionToSubsupport);
        }

        for (auto actor : actors)
        {
            if (actor)
                actor->release();
        }

        free(family);
    }

    static void recursivePartitionPostSplitTestCounts(const std::vector<Nv::Blast::Actor*>& actors, uint32_t leafChunkCount, uint32_t supportChunkCount, bool partitionToSubsupport)
    {
        // Test to see that all actors are split down to single support chunks
        uint32_t remainingActorCount = 0;
        for (uint32_t i = 0; i < actors.size(); ++i)
        {
            Nv::Blast::Actor* a = (Nv::Blast::Actor*)actors[i];
            if (a == nullptr)
            {
                continue;
            }

            ++remainingActorCount;

            NVBLAST_ASSERT(1 == a->getVisibleChunkCount() || a->hasExternalBonds());
            EXPECT_TRUE(1 == a->getVisibleChunkCount() || a->hasExternalBonds());
            if (!partitionToSubsupport)
            {
                EXPECT_EQ(1, a->getGraphNodeCount());
            }

            if (0 == a->getVisibleChunkCount())
            {
                EXPECT_TRUE(a->hasExternalBonds());
                EXPECT_EQ(1, a->getGraphNodeCount());
                EXPECT_EQ(a->getFamilyHeader()->m_asset->m_graph.m_nodeCount - 1, a->getFirstGraphNodeIndex());
                --remainingActorCount;    // Do not count this as a remaining actor, to be compared with leaf or support chunk counts later
            }

            const bool actorReleaseResult = NvBlastActorDeactivate(actors[i], nullptr);
            EXPECT_TRUE(actorReleaseResult);
        }

        if (partitionToSubsupport)
        {
            EXPECT_EQ(leafChunkCount, remainingActorCount);
        }
        else
        {
            EXPECT_EQ(supportChunkCount, remainingActorCount);
        }
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
                const uint32_t chunkIndex = asset.m_graph.getChunkIndices()[i];
                if (!Nv::Blast::isInvalidIndex(chunkIndex))
                {
                    isSupport[chunkIndex] = true;
                }
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

            // Make sure every graph chunk is represented by a visible chunk, or represents the world
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
                EXPECT_TRUE(!Nv::Blast::isInvalidIndex(chunkIndex) || (graphNodeIndex == asset.m_graph.m_nodeCount-1 && actor.hasExternalBonds()));
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

    static void recursivePartitionPostSplitTestVisibleChunks(const std::vector<Nv::Blast::Actor*>& actors, uint32_t leafChunkCount, uint32_t supportChunkCount, bool partitionToSubsupport)
    {
        for (uint32_t i = 0; i < actors.size(); ++i)
        {
            Nv::Blast::Actor* a = (Nv::Blast::Actor*)actors[i];
            if (a == nullptr)
            {
                continue;
            }

            testActorVisibleChunks(*a, nullptr);
        }
    }

    void partitionActorsToSupportChunks
    (
        uint32_t assetDescCount,
        const NvBlastAssetDesc* assetDescs,
        void(*preSplitTest)(const Nv::Blast::Actor&, NvBlastLog),
        void(*postSplitTest)(const std::vector<Nv::Blast::Actor*>&, uint32_t, uint32_t, bool),
        bool partitionToSubsupport
    )
    {
        srand(0);

        for (uint32_t i = 0; i < assetDescCount; ++i)
        {
            // Create an asset
            NvBlastAsset* asset = buildAsset(assetDescs[i]);

            // Perform repeated partitioning
            instanceAndPartitionRecursively(*asset, partitionToSubsupport, preSplitTest, postSplitTest);

            // Free the asset
            free(asset);
        }
    }

    static void compareFamilies(const NvBlastFamily* family1, const NvBlastFamily* family2, NvBlastLog logFn)
    {
        // first check that the family sizes are the same
        // still do the byte comparison even if they aren't equal to make it easier to spot where things went wrong
        const uint32_t size1 = NvBlastFamilyGetSize(family1, logFn);
        const uint32_t size2 = NvBlastFamilyGetSize(family2, logFn);
        const uint32_t size = std::min(size1, size2);
        if (size1 != size2)
        {
            std::ostringstream msg;
            msg << "Family deserialization sizes don't match [" << size1 << ", " << size2 << "].";
            logFn(NvBlastMessage::Error, msg.str().c_str(), __FILE__, __LINE__);
        }

        const char* block1 = reinterpret_cast<const char*>(family1);
        const char* block2 = reinterpret_cast<const char*>(family2);
#if 0
        EXPECT_EQ(0, memcmp(block1, block2, size));
#else
        bool diffFound = false;
        size_t startDiff = 0;
        for (size_t i = 0; i < size; ++i)
        {
            if (block1[i] != block2[i])
            {
                diffFound = true;
                startDiff = i;
                break;
            }
        }
        if (!diffFound)
        {
            return;
        }
        size_t endDiff = startDiff;
        for (size_t i = size; i--;)
        {
            if (block1[i] != block2[i])
            {
                endDiff = i;
                break;
            }
        }
        std::ostringstream msg;
        msg << "Family deserialization does not match in range [" << startDiff << ", " << endDiff << "].";
        logFn(NvBlastMessage::Error, msg.str().c_str(), __FILE__, __LINE__);
#endif
    }

    static void testActorBlockSerialize(std::vector<NvBlastActor*>& actors, NvBlastLog logFn)
    {
        if (actors.size())
        {
            const NvBlastFamily* family = NvBlastActorGetFamily(actors[0], logFn);
            const uint32_t size = NvBlastFamilyGetSize(family, logFn);
            s_storage.insert(s_storage.end(), (char*)family, (char*)family + size);
        }
    }

    static void testActorCapnSerialize(std::vector<NvBlastActor*>& actors, NvBlastLog logFn)
    {
        if (actors.size())
        {
            const NvBlastFamily* family = NvBlastActorGetFamily(actors[0], logFn);

            Nv::Blast::ExtSerialization* ser = NvBlastExtSerializationCreate();
            EXPECT_TRUE(ser != nullptr);
            EXPECT_TRUE(ser->getSerializationEncoding() == Nv::Blast::ExtSerialization::EncodingID::CapnProtoBinary);

            void* serializedFamilyBuffer = nullptr;
            const uint64_t serialFamilySize =
                ser->serializeIntoBuffer(serializedFamilyBuffer, family, Nv::Blast::LlObjectTypeID::Family);
            EXPECT_TRUE(serialFamilySize != 0);

            s_storage.insert(s_storage.end(), (char*)&serialFamilySize, (char*)&serialFamilySize + sizeof(uint64_t));
            s_storage.insert(s_storage.end(), (char*)serializedFamilyBuffer, (char*)serializedFamilyBuffer + serialFamilySize);
        }
    }

    static void testActorDeserializeCommon(const NvBlastFamily* family, std::vector<NvBlastActor*>& actors, uint32_t size, NvBlastLog logFn)
    {
        EXPECT_LT(s_curr, s_storage.size());
        EXPECT_TRUE(size > 0);
        EXPECT_LE(s_curr + size, s_storage.size());
        s_curr += size;

        const NvBlastFamily* actorFamily = NvBlastActorGetFamily(actors[0], logFn);
        // Family may contain different assets pointers, copy into new family block and set the same asset before comparing
        Nv::Blast::Actor& a = *static_cast<Nv::Blast::Actor*>(actors[0]);
        const Nv::Blast::Asset* solverAsset = a.getAsset();
        const uint32_t familySize = NvBlastFamilyGetSize(family, logFn);
        std::vector<char> storageFamilyCopy((char*)family, (char*)family + familySize);
        NvBlastFamily* storageFamily = reinterpret_cast<NvBlastFamily*>(storageFamilyCopy.data());
        NvBlastFamilySetAsset(storageFamily, solverAsset, logFn);
        {
            const uint32_t actorCountExpected = NvBlastFamilyGetActorCount(storageFamily, logFn);
            std::vector<NvBlastActor*> blockActors(actorCountExpected);
            const uint32_t actorCountReturned = NvBlastFamilyGetActors(blockActors.data(), actorCountExpected, storageFamily, logFn);
            EXPECT_EQ(actorCountExpected, actorCountReturned);
        }
        compareFamilies(storageFamily, actorFamily, logFn);
    }

    static void testActorBlockDeserialize(std::vector<NvBlastActor*>& actors, NvBlastLog logFn)
    {
        if (actors.size())
        {
            const NvBlastFamily* family = reinterpret_cast<NvBlastFamily*>(&s_storage[s_curr]);
            const uint32_t size = NvBlastFamilyGetSize(family, logFn);
            testActorDeserializeCommon(family, actors, size, logFn);
        }
    }

    static void testActorCapnDeserialize(std::vector<NvBlastActor*>& actors, NvBlastLog logFn)
    {
        if (actors.size())
        {
            Nv::Blast::ExtSerialization* ser = NvBlastExtSerializationCreate();
            EXPECT_TRUE(ser->getSerializationEncoding() == Nv::Blast::ExtSerialization::EncodingID::CapnProtoBinary);

            // the serialized size is stored in the stream right before the data itself, pull it out first
            uint32_t objTypeId;
            const uint64_t& size = *reinterpret_cast<const uint64_t*>(&s_storage[s_curr]);
            s_curr += sizeof(uint64_t);
            EXPECT_LE(size, UINT32_MAX);

            // now read the buffer itself
            void* object = ser->deserializeFromBuffer(&s_storage[s_curr], size, &objTypeId);
            EXPECT_TRUE(object != nullptr);
            EXPECT_TRUE(objTypeId == Nv::Blast::LlObjectTypeID::Family);

            // finally compare it with the original family
            const NvBlastFamily* family = reinterpret_cast<NvBlastFamily*>(object);
            testActorDeserializeCommon(family, actors, (uint32_t)size, logFn);
        }
    }

    // Serialize all actors and then deserialize back into a new family in a random order, and compare with the original family
    static void testActorSerializationNewFamily(std::vector<NvBlastActor*>& actors, NvBlastLog logFn)
    {
        if (actors.size() == 0)
        {
            return;
        }

        Nv::Blast::Actor& a = *static_cast<Nv::Blast::Actor*>(actors[0]);
        const Nv::Blast::Asset* solverAsset = a.getAsset();

        const uint32_t serSizeBound = NvBlastAssetGetActorSerializationSizeUpperBound(solverAsset, logFn);
            
        std::vector< std::vector<char> > streams(actors.size());
        for (size_t i = 0; i < actors.size(); ++i)
        {
            const uint32_t serSize = NvBlastActorGetSerializationSize(actors[i], logFn);
            EXPECT_GE(serSizeBound, serSize);
            std::vector<char>& stream = streams[i];
            stream.resize(serSize);
            const uint32_t bytesWritten = NvBlastActorSerialize(stream.data(), serSize, actors[i], logFn);
            EXPECT_EQ(serSize, bytesWritten);
        }

        void* fmem = alloc(NvBlastAssetGetFamilyMemorySize(solverAsset, logFn));
        NvBlastFamily* newFamily = NvBlastAssetCreateFamily(fmem, solverAsset, logFn);

        std::vector<size_t> order(actors.size());
        for (size_t i = 0; i < order.size(); ++i)
        {
            order[i] = i;
        }

        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(order.begin(), order.end(), g);

        for (size_t i = 0; i < actors.size(); ++i)
        {
            NvBlastActor* newActor = NvBlastFamilyDeserializeActor(newFamily, streams[order[i]].data(), logFn);
            EXPECT_TRUE(newActor != nullptr);
        }

        const NvBlastFamily* oldFamily = NvBlastActorGetFamily(&a, logFn);

        // Allow there to be differences with invalid actors
        const Nv::Blast::FamilyHeader* f1 = reinterpret_cast<const Nv::Blast::FamilyHeader*>(oldFamily);
        const Nv::Blast::FamilyHeader* f2 = reinterpret_cast<const Nv::Blast::FamilyHeader*>(newFamily);
        for (uint32_t actorN = 0; actorN < f1->getActorsArraySize(); ++actorN)
        {
            const Nv::Blast::Actor* a1 = f1->getActors() + actorN;
            Nv::Blast::Actor* a2 = const_cast<Nv::Blast::Actor*>(f2->getActors() + actorN);
            EXPECT_EQ(a1->isActive(), a2->isActive());
            if (!a1->isActive())
            {
                *a2 = *a1;    // Actual data does not matter, setting equal to pass comparison
            }
        }

        compareFamilies(oldFamily, newFamily, logFn);

        free(newFamily);
    }

    // Copy the family and then serialize some subset of actors, deleting them afterwards.
    // Then, deserialize back into the block and compare the original and new families.
    static void testActorSerializationPartialBlock(std::vector<NvBlastActor*>& actors, NvBlastLog logFn)
    {
        if (actors.size() <= 1)
        {
            return;
        }

        Nv::Blast::Actor& a = *static_cast<Nv::Blast::Actor*>(actors[0]);
        const Nv::Blast::Asset* solverAsset = a.getAsset();

        const NvBlastFamily* oldFamily = NvBlastActorGetFamily(&a, logFn);
        const uint32_t size = NvBlastFamilyGetSize(oldFamily, logFn);
        std::vector<char> buffer((char*)oldFamily, (char*)oldFamily + size);
        NvBlastFamily* familyCopy = reinterpret_cast<NvBlastFamily*>(buffer.data());

        const uint32_t serCount = 1 + (rand() % actors.size() - 1);

        const uint32_t actorCount = NvBlastFamilyGetActorCount(familyCopy, logFn);
        std::vector<NvBlastActor*> actorsRemaining(actorCount);
        const uint32_t actorsInFamily = NvBlastFamilyGetActors(&actorsRemaining[0], actorCount, familyCopy, logFn);
        EXPECT_EQ(actorCount, actorsInFamily);

        const uint32_t serSizeBound = NvBlastAssetGetActorSerializationSizeUpperBound(solverAsset, logFn);

        std::vector< std::vector<char> > streams(serCount);
        for (uint32_t i = 0; i < serCount; ++i)
        {
            std::vector<char>& stream = streams[i];
            const uint32_t indexToStream = rand() % actorsRemaining.size();
            NvBlastActor* actorToStream = actorsRemaining[indexToStream];
            std::swap(actorsRemaining[indexToStream], actorsRemaining[actorsRemaining.size() - 1]);
            actorsRemaining.pop_back();
            const uint32_t serSize = NvBlastActorGetSerializationSize(actorToStream, logFn);
            EXPECT_GE(serSizeBound, serSize);
            stream.resize(serSize);
            const uint32_t bytesWritten = NvBlastActorSerialize(&stream[0], serSize, actorToStream, logFn);
            EXPECT_EQ(serSize, bytesWritten);
            NvBlastActorDeactivate(actorToStream, logFn);
        }

        for (uint32_t i = 0; i < serCount; ++i)
        {
            NvBlastActor* newActor = NvBlastFamilyDeserializeActor(familyCopy, streams[i].data(), logFn);
            EXPECT_TRUE(newActor != nullptr);
        }

        compareFamilies(oldFamily, familyCopy, logFn);
    }

    void damageLeafSupportActors
    (
    uint32_t assetCount,
    uint32_t familyCount,
    uint32_t damageCount,
    bool simple,
    void (*actorTest)(const Nv::Blast::Actor&, NvBlastLog),
    void (*postDamageTest)(std::vector<NvBlastActor*>&, NvBlastLog),
    CubeAssetGenerator::BondFlags bondFlags = CubeAssetGenerator::BondFlags::ALL_INTERNAL_BONDS
    )
    {
        const float relativeDamageRadius = simple ? 0.75f : 0.2f;
        const float compressiveDamage = 1.0f;
        const uint32_t minChunkCount = simple ? 9 : 100;
        const uint32_t maxChunkCount = simple ? 9 : 10000;
        const bool printActorCount = false;

        srand(0);

        std::cout << "Asset # (out of " << assetCount << "): ";
        for (uint32_t assetNum = 0; assetNum < assetCount; ++assetNum)
        {
            std::cout << assetNum + 1 << ".. ";
            CubeAssetGenerator::Settings settings;
            settings.extents = GeneratorAsset::Vec3(1, 1, 1);
            settings.bondFlags = bondFlags;
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
                    depthInfo.slicesPerAxis = simple ? GeneratorAsset::Vec3(2, 2, 2) : GeneratorAsset::Vec3((float)(1 + rand() % 4), (float)(1 + rand() % 4), (float)(1 + rand() % 4));
                    chunkMul = (uint32_t)(depthInfo.slicesPerAxis.x * depthInfo.slicesPerAxis.y * depthInfo.slicesPerAxis.z);
                } while (chunkMul == 1);
                if (chunkCount*chunkMul > maxChunkCount)
                {
                    break;
                }
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
            desc.chunkDescs = testAsset.solverChunks.data();
            desc.chunkCount = (uint32_t)testAsset.solverChunks.size();
            desc.bondDescs = testAsset.solverBonds.data();
            desc.bondCount = (uint32_t)testAsset.solverBonds.size();
            NvBlastAsset* asset = buildAsset(desc);
            NvBlastID assetID = NvBlastAssetGetID(asset, messageLog);

            // copy asset (for setAsset testing)
            const char* data = (const char*)asset;
            const uint32_t dataSize = NvBlastAssetGetSize(asset, messageLog);
            char* duplicateData = (char*)alloc(dataSize);
            memcpy(duplicateData, data, dataSize);
            NvBlastAsset* assetDuplicate = (NvBlastAsset*)duplicateData;

            // Generate families
            for (uint32_t familyNum = 0; familyNum < familyCount; ++familyNum)
            {
                // family
                void* fmem = alloc(NvBlastAssetGetFamilyMemorySize(asset, messageLog));
                NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, messageLog);    // Using zeroingAlloc in case actorTest compares memory blocks
                NvBlastID id = NvBlastFamilyGetAssetID(family, messageLog);
                EXPECT_TRUE(!memcmp(&assetID, &id, sizeof(NvBlastID)));
                if (rand() % 2 == 0)
                {
                    // replace asset with duplicate in half of cases to test setAsset
                    NvBlastFamilySetAsset(family, assetDuplicate, messageLog);
                    NvBlastID id2 = NvBlastFamilyGetAssetID(family, messageLog);
                    EXPECT_TRUE(!memcmp(&assetID, &id2, sizeof(NvBlastID)));
                }

                // actor
                NvBlastActorDesc actorDesc;
                actorDesc.initialBondHealths = actorDesc.initialSupportChunkHealths = nullptr;
                actorDesc.uniformInitialBondHealth = actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
                m_scratch.resize((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, messageLog));
                NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, m_scratch.data(), messageLog);
                EXPECT_TRUE(actor != nullptr);

                // Generate damage
                std::set<NvBlastActor*> actors;
                actors.insert(actor);
                if (printActorCount) std::cout << "Actors: 1.. ";
                for (uint32_t damageNum = 0; damageNum < damageCount; ++damageNum)
                {
                    GeneratorAsset::Vec3 localPos = settings.extents*GeneratorAsset::Vec3((float)rand() / RAND_MAX - 0.5f, (float)rand() / RAND_MAX - 0.5f, (float)rand() / RAND_MAX - 0.5f);
                    blast(actors, &testAsset, localPos, relativeDamageRadius, relativeDamageRadius*1.2f, compressiveDamage);
                    if (printActorCount) std::cout << actors.size() << ".. ";
                    if (actors.size() > 0)
                    {
                        const NvBlastFamily* family = NvBlastActorGetFamily(*actors.begin(), messageLog);
                        const uint32_t actorCount = NvBlastFamilyGetActorCount(family, messageLog);
                        EXPECT_EQ((uint32_t)actors.size(), actorCount);
                        if ((uint32_t)actors.size() == actorCount)
                        {
                            std::vector<NvBlastActor*> buffer1(actorCount);
                            const uint32_t actorsWritten = NvBlastFamilyGetActors(&buffer1[0], actorCount, family, messageLog);
                            EXPECT_EQ(actorsWritten, actorCount);
                            std::vector<NvBlastActor*> buffer2(actors.begin(), actors.end());
                            EXPECT_EQ(0, memcmp(&buffer1[0], buffer2.data(), actorCount*sizeof(NvBlastActor*)));
                        }
                    }
                    // Test individual actors
                    if (actorTest != nullptr)
                    {
                        for (std::set<NvBlastActor*>::iterator k = actors.begin(); k != actors.end(); ++k)
                        {
                            actorTest(*static_cast<Nv::Blast::Actor*>(*k), messageLog);
                        }
                    }
                }
                if (printActorCount) std::cout << "\n";

                // Test fractured actor set
                if (postDamageTest)
                {
                    std::vector<NvBlastActor*> actorArray(actors.begin(), actors.end());
                    postDamageTest(actorArray, messageLog);
                }

                // Release remaining actors
                for (std::set<NvBlastActor*>::iterator k = actors.begin(); k != actors.end(); ++k)
                {
                    NvBlastActorDeactivate(*k, messageLog);
                }
                actors.clear();

                free(family);
            }

            // Release asset data
            free(asset);
            free(assetDuplicate);
        }
        std::cout << "done.\n";
    }

    std::vector<NvBlastAsset*>        m_assets;
    std::vector<NvBlastActor*>        m_actors;
    std::vector<char>                m_scratch;
    static std::vector<char>        s_storage;

    static size_t                    s_curr;
};

// Static values
template<int FailLevel, int Verbosity>
std::vector<char>    ActorTest<FailLevel, Verbosity>::s_storage;

template<int FailLevel, int Verbosity>
size_t                    ActorTest<FailLevel, Verbosity>::s_curr;

// Specializations
typedef ActorTest<NvBlastMessage::Error, 1> ActorTestAllowWarnings;
typedef ActorTest<NvBlastMessage::Warning, 1> ActorTestStrict;

// Tests
TEST_F(ActorTestStrict, InstanceActors)
{
    // Build assets and instance actors
    buildAssets();
    instanceActors();

    // Release actors and destroy assets
    releaseActors();
    destroyAssets();
}

TEST_F(ActorTestAllowWarnings, ActorHealthInitialization)
{
    // Test all assets
    std::vector<NvBlastAssetDesc> assetDescs;
    assetDescs.insert(assetDescs.end(), g_assetDescs, g_assetDescs + getAssetDescCount());
    assetDescs.insert(assetDescs.end(), g_assetDescsMissingCoverage, g_assetDescsMissingCoverage + getAssetDescMissingCoverageCount());

    struct TestMode
    {
        enum Enum
        {
            Uniform,
            Nonuniform,

            Count
        };
    };

    for (auto assetDesc : assetDescs)
    {
        NvBlastAsset* asset = buildAsset(assetDesc);
        EXPECT_TRUE(asset != nullptr);

        Nv::Blast::Asset& assetInt = static_cast<Nv::Blast::Asset&>(*asset);

        NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(asset, nullptr);

        std::vector<float> supportChunkHealths(graph.nodeCount);
        for (size_t i = 0; i < supportChunkHealths.size(); ++i)
        {
            supportChunkHealths[i] = 1.0f + (float)i;
        }

        std::vector<float> bondHealths(assetInt.getBondCount());
        for (size_t i = 0; i < bondHealths.size(); ++i)
        {
            bondHealths[i] = 1.5f + (float)i;
        }

        for (int chunkTestMode = 0; chunkTestMode < TestMode::Count; ++chunkTestMode)
        {
            for (int bondTestMode = 0; bondTestMode < TestMode::Count; ++bondTestMode)
            {
                NvBlastActorDesc actorDesc;

                switch (chunkTestMode)
                {
                default:
                case TestMode::Uniform:
                    actorDesc.initialSupportChunkHealths = nullptr;
                    actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
                    break;
                case TestMode::Nonuniform:
                    actorDesc.initialSupportChunkHealths = supportChunkHealths.data();
                    break;
                }

                switch (bondTestMode)
                {
                default:
                case TestMode::Uniform:
                    actorDesc.initialBondHealths = nullptr;
                    actorDesc.uniformInitialBondHealth = 2.0f;
                    break;
                case TestMode::Nonuniform:
                    actorDesc.initialBondHealths = bondHealths.data();
                    break;
                }

                void* fmem = alloc(NvBlastAssetGetFamilyMemorySize(asset, messageLog));
                NvBlastFamily* family = NvBlastAssetCreateFamily(fmem, asset, nullptr);
                std::vector<char> scratch((size_t)NvBlastFamilyGetRequiredScratchForCreateFirstActor(family, messageLog));
                NvBlastActor* actor = NvBlastFamilyCreateFirstActor(family, &actorDesc, scratch.data(), messageLog);
                EXPECT_TRUE(actor != nullptr);

                Nv::Blast::Actor& actorInt = static_cast<Nv::Blast::Actor&>(*actor);
                Nv::Blast::FamilyHeader* header = actorInt.getFamilyHeader();


                for (uint32_t i = 0; i < graph.nodeCount; ++i)
                {
                    const uint32_t supportChunkIndex = graph.chunkIndices[i];
                    for (Nv::Blast::Asset::DepthFirstIt it(assetInt, supportChunkIndex); (bool)it; ++it)
                    {
                        const uint32_t chunkIndex = (uint32_t)it;
                        const uint32_t lowerSupportIndex = assetInt.getContiguousLowerSupportIndex(chunkIndex);
                        NVBLAST_ASSERT(lowerSupportIndex < assetInt.getLowerSupportChunkCount());
                        const float health = header->getLowerSupportChunkHealths()[lowerSupportIndex];
                        switch (chunkTestMode)
                        {
                        default:
                        case TestMode::Uniform:
                            EXPECT_EQ(1.0f, health);
                            break;
                        case TestMode::Nonuniform:
                            EXPECT_EQ(supportChunkHealths[i], health);
                            break;
                        }
                    }
                }

                for (uint32_t i = 0; i < assetInt.getBondCount(); ++i)
                {
                    switch (bondTestMode)
                    {
                    default:
                    case TestMode::Uniform:
                        EXPECT_EQ(2.0f, header->getBondHealths()[i]);
                        break;
                    case TestMode::Nonuniform:
                        EXPECT_EQ(bondHealths[i], header->getBondHealths()[i]);
                        break;
                    }
                }

                NvBlastActorDeactivate(actor, messageLog);
                free(family);
            }
        }

        free(asset);
    }
}

TEST_F(ActorTestStrict, PartitionActorsToSupportChunksTestCounts)
{
    partitionActorsToSupportChunks(getAssetDescCount(), g_assetDescs, nullptr, recursivePartitionPostSplitTestCounts, false);
}

TEST_F(ActorTestAllowWarnings, PartitionActorsFromBadDescriptorsToSupportChunksTestCounts)
{
    partitionActorsToSupportChunks(getAssetDescMissingCoverageCount(), g_assetDescsMissingCoverage, nullptr, recursivePartitionPostSplitTestCounts, false);
}

TEST_F(ActorTestStrict, PartitionActorsToLeafChunksTestCounts)
{
    partitionActorsToSupportChunks(getAssetDescCount(), g_assetDescs, nullptr, recursivePartitionPostSplitTestCounts, true);
}

TEST_F(ActorTestAllowWarnings, PartitionActorsFromBadDescriptorsToLeafChunksTestCounts)
{
    partitionActorsToSupportChunks(getAssetDescMissingCoverageCount(), g_assetDescsMissingCoverage, nullptr, recursivePartitionPostSplitTestCounts, true);
}

TEST_F(ActorTestStrict, PartitionActorsToSupportChunksTestVisibility)
{
    partitionActorsToSupportChunks(getAssetDescCount(), g_assetDescs, testActorVisibleChunks, recursivePartitionPostSplitTestVisibleChunks, false);
}

TEST_F(ActorTestAllowWarnings, PartitionActorsFromBadDescriptorsToSupportChunksTestVisibility)
{
    partitionActorsToSupportChunks(getAssetDescMissingCoverageCount(), g_assetDescsMissingCoverage, testActorVisibleChunks, recursivePartitionPostSplitTestVisibleChunks, false);
}

TEST_F(ActorTestStrict, PartitionActorsToLeafChunksTestVisibility)
{
    partitionActorsToSupportChunks(getAssetDescCount(), g_assetDescs, testActorVisibleChunks, recursivePartitionPostSplitTestVisibleChunks, true);
}

TEST_F(ActorTestAllowWarnings, PartitionActorsFromBadDescriptorsToLeafChunksTestVisibility)
{
    partitionActorsToSupportChunks(getAssetDescMissingCoverageCount(), g_assetDescsMissingCoverage, testActorVisibleChunks, recursivePartitionPostSplitTestVisibleChunks, true);
}

TEST_F(ActorTestStrict, DamageLeafSupportActorsTestVisibility)
{
    damageLeafSupportActors(4, 4, 5, false, testActorVisibleChunks, nullptr);
}

TEST_F(ActorTestStrict, DamageLeafSupportActorTestBlockSerialization)
{
    typedef CubeAssetGenerator::BondFlags BF;
    s_storage.resize(0);
    damageLeafSupportActors(4, 4, 5, false, nullptr, testActorBlockSerialize);
    s_curr = 0;
    damageLeafSupportActors(4, 4, 5, false, nullptr, testActorBlockDeserialize);
    s_storage.resize(0);
    damageLeafSupportActors(4, 4, 5, false, nullptr, testActorBlockSerialize, BF::ALL_INTERNAL_BONDS | BF::Z_MINUS_WORLD_BONDS);
    s_curr = 0;
    damageLeafSupportActors(4, 4, 5, false, nullptr, testActorBlockDeserialize, BF::ALL_INTERNAL_BONDS | BF::Z_MINUS_WORLD_BONDS);
    s_storage.resize(0);

    s_storage.resize(0);
    damageLeafSupportActors(4, 4, 5, false, nullptr, testActorCapnSerialize);
    s_curr = 0;
    damageLeafSupportActors(4, 4, 5, false, nullptr, testActorCapnDeserialize);
    s_storage.resize(0);
    damageLeafSupportActors(4, 4, 5, false, nullptr, testActorCapnSerialize, BF::ALL_INTERNAL_BONDS | BF::Z_MINUS_WORLD_BONDS);
    s_curr = 0;
    damageLeafSupportActors(4, 4, 5, false, nullptr, testActorCapnDeserialize, BF::ALL_INTERNAL_BONDS | BF::Z_MINUS_WORLD_BONDS);
    s_storage.resize(0);
}

TEST_F(ActorTestStrict, DISABLED_DamageSimpleLeafSupportActorTestActorSerializationNewFamily)
{
    typedef CubeAssetGenerator::BondFlags BF;
    damageLeafSupportActors(1, 1, 4, true, nullptr, testActorSerializationNewFamily);
    damageLeafSupportActors(1, 1, 4, true, nullptr, testActorSerializationNewFamily, BF::ALL_INTERNAL_BONDS | BF::Z_MINUS_WORLD_BONDS);
}

TEST_F(ActorTestStrict, DamageSimpleLeafSupportActorTestActorSerializationPartialBlock)
{
    typedef CubeAssetGenerator::BondFlags BF;
    damageLeafSupportActors(1, 1, 4, true, nullptr, testActorSerializationPartialBlock);
    damageLeafSupportActors(1, 1, 4, true, nullptr, testActorSerializationPartialBlock, BF::ALL_INTERNAL_BONDS | BF::Z_MINUS_WORLD_BONDS);
}

TEST_F(ActorTestStrict, DISABLED_DamageLeafSupportActorTestActorSerializationNewFamily)
{
    typedef CubeAssetGenerator::BondFlags BF;
    damageLeafSupportActors(4, 4, 4, false, nullptr, testActorSerializationNewFamily);
    damageLeafSupportActors(4, 4, 4, false, nullptr, testActorSerializationNewFamily, BF::ALL_INTERNAL_BONDS | BF::Z_MINUS_WORLD_BONDS);
}

TEST_F(ActorTestStrict, DamageLeafSupportActorTestActorSerializationPartialBlock)
{
    typedef CubeAssetGenerator::BondFlags BF;
    damageLeafSupportActors(4, 4, 4, false, nullptr, testActorSerializationPartialBlock);
    damageLeafSupportActors(4, 4, 4, false, nullptr, testActorSerializationPartialBlock, BF::ALL_INTERNAL_BONDS | BF::Z_MINUS_WORLD_BONDS);
}

TEST_F(ActorTestStrict, DamageMultipleIslandLeafSupportActorsTestVisibility)
{
    typedef CubeAssetGenerator::BondFlags BF;
    damageLeafSupportActors(4, 4, 5, false, testActorVisibleChunks, nullptr, BF::Y_BONDS | BF::Z_BONDS);    // Only connect y-z plane islands
    damageLeafSupportActors(4, 4, 5, false, testActorVisibleChunks, nullptr, BF::Z_BONDS);    // Only connect z-direction islands
    damageLeafSupportActors(4, 4, 5, false, testActorVisibleChunks, nullptr, BF::NO_BONDS);    // All support chunks disconnected (single-chunk islands)
}

TEST_F(ActorTestStrict, DamageBoundToWorldLeafSupportActorsTestVisibility)
{
    typedef CubeAssetGenerator::BondFlags BF;
    damageLeafSupportActors(4, 4, 5, false, testActorVisibleChunks, nullptr, BF::ALL_INTERNAL_BONDS | BF::X_MINUS_WORLD_BONDS);
    damageLeafSupportActors(4, 4, 5, false, testActorVisibleChunks, nullptr, BF::ALL_INTERNAL_BONDS | BF::Y_PLUS_WORLD_BONDS);
    damageLeafSupportActors(4, 4, 5, false, testActorVisibleChunks, nullptr, BF::ALL_INTERNAL_BONDS | BF::Z_MINUS_WORLD_BONDS);
     damageLeafSupportActors(4, 4, 5, false, testActorVisibleChunks, nullptr, BF::ALL_INTERNAL_BONDS | BF::X_PLUS_WORLD_BONDS | BF::Y_MINUS_WORLD_BONDS);
    damageLeafSupportActors(4, 4, 5, false, testActorVisibleChunks, nullptr, BF::ALL_INTERNAL_BONDS | BF::X_PLUS_WORLD_BONDS | BF::X_MINUS_WORLD_BONDS
                                                                                                    | BF::Y_PLUS_WORLD_BONDS | BF::Y_MINUS_WORLD_BONDS
                                                                                                    | BF::Z_PLUS_WORLD_BONDS | BF::Z_MINUS_WORLD_BONDS);
}

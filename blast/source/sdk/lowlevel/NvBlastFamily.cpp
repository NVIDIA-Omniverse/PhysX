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


#include "NvBlastTypes.h"
#include "NvBlastFamily.h"
#include "NvBlastFamilyGraph.h"
#include "NvBlastIndexFns.h"
#include "NvBlastTime.h"

#include <new>

namespace Nv
{
namespace Blast
{

//////// Global functions ////////

struct FamilyDataOffsets
{
    size_t m_actors;
    size_t m_visibleChunkIndexLinks;
    size_t m_chunkActorIndices;
    size_t m_graphNodeIndexLinks;
    size_t m_lowerSupportChunkHealths;
    size_t m_graphBondHealths;
    size_t m_graphCachedBondHealths;
    size_t m_familyGraph;
};


static size_t createFamilyDataOffsets(FamilyDataOffsets& offsets, const NvBlastAssetMemSizeData& sizeData)
{
    NvBlastCreateOffsetStart(sizeof(FamilyHeader));
    NvBlastCreateOffsetAlign16(offsets.m_actors, sizeData.lowerSupportChunkCount * sizeof(Actor));
    NvBlastCreateOffsetAlign16(offsets.m_visibleChunkIndexLinks, sizeData.chunkCount * sizeof(IndexDLink<uint32_t>));
    NvBlastCreateOffsetAlign16(offsets.m_chunkActorIndices, sizeData.upperSupportChunkCount * sizeof(uint32_t));
    NvBlastCreateOffsetAlign16(offsets.m_graphNodeIndexLinks, sizeData.nodeCount * sizeof(uint32_t));
    NvBlastCreateOffsetAlign16(offsets.m_lowerSupportChunkHealths, sizeData.lowerSupportChunkCount * sizeof(float));
    NvBlastCreateOffsetAlign16(offsets.m_graphBondHealths, sizeData.bondCount * sizeof(float));
    NvBlastCreateOffsetAlign16(offsets.m_graphCachedBondHealths, sizeData.bondCount * sizeof(float));
    NvBlastCreateOffsetAlign16(offsets.m_familyGraph, static_cast<size_t>(FamilyGraph::requiredMemorySize(sizeData.nodeCount, sizeData.bondCount)));
    return NvBlastCreateOffsetEndAlign16();
}


size_t getFamilyMemorySize(const Asset* asset)
{
#if NVBLASTLL_CHECK_PARAMS
    if (asset == nullptr)
    {
        NVBLAST_ALWAYS_ASSERT();
        return 0;
    }
#endif

    const NvBlastAssetMemSizeData sizeData = NvBlastAssetMemSizeDataFromAsset(asset);
    return getFamilyMemorySize(sizeData);
}


size_t getFamilyMemorySize(const NvBlastAssetMemSizeData& sizeData)
{
    FamilyDataOffsets offsets;
    return createFamilyDataOffsets(offsets, sizeData);
}


// this path is used by the serialization code
// buffers are set up, but some parts (like asset ID) are left to the serialization code to fill in
static NvBlastFamily* createFamily(void* mem, const NvBlastAssetMemSizeData& sizeData, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(mem != nullptr, logFn, "createFamily: NULL mem pointer input.", return nullptr);
    NVBLASTLL_CHECK((reinterpret_cast<uintptr_t>(mem) & 0xF) == 0, logFn, "createFamily: mem pointer not 16-byte aligned.", return nullptr);

    if (sizeData.chunkCount == 0)
    {
        NVBLASTLL_LOG_ERROR(logFn, "createFamily: Asset has no chunks.  Family not created.\n");
        return nullptr;
    }

    const uint32_t bondCount = sizeData.bondCount;

    // We need to keep this many actor representations around for our island indexing scheme.
    const uint32_t lowerSupportChunkCount = sizeData.lowerSupportChunkCount;

    // We need this many chunk actor indices.
    const uint32_t upperSupportChunkCount = sizeData.upperSupportChunkCount;

    // Family offsets
    FamilyDataOffsets offsets;
    const size_t dataSize = createFamilyDataOffsets(offsets, sizeData);

    // Restricting our data size to < 4GB so that we may use uint32_t offsets
    if (dataSize > (size_t)UINT32_MAX)
    {
        NVBLASTLL_LOG_ERROR(logFn, "Nv::Blast::Actor::instanceAllocate: Instance data block size will exceed 4GB.  Instance not created.\n");
        return nullptr;
    }

    // Allocate family
    NvBlastFamily* family = reinterpret_cast<NvBlastFamily*>(memset(mem, 0, dataSize));

    // Fill in family header
    FamilyHeader* header = (FamilyHeader*)family;
    header->dataType = NvBlastDataBlock::FamilyDataBlock;
    header->formatVersion = 0;    // Not currently using this field
    header->size = (uint32_t)dataSize;
    header->m_actorCount = 0;
    header->m_actorsOffset = (uint32_t)offsets.m_actors;
    header->m_visibleChunkIndexLinksOffset = (uint32_t)offsets.m_visibleChunkIndexLinks;
    header->m_chunkActorIndicesOffset = (uint32_t)offsets.m_chunkActorIndices;
    header->m_graphNodeIndexLinksOffset = (uint32_t)offsets.m_graphNodeIndexLinks;
    header->m_lowerSupportChunkHealthsOffset = (uint32_t)offsets.m_lowerSupportChunkHealths;
    header->m_graphBondHealthsOffset = (uint32_t)offsets.m_graphBondHealths;
    header->m_graphCachedBondHealthsOffset = (uint32_t)offsets.m_graphCachedBondHealths;
    header->m_familyGraphOffset = (uint32_t)offsets.m_familyGraph;

    // Initialize family header data:

    // Actors - initialize to defaults, with zero offset value (indicating inactive state)
    Actor* actors = header->getActors();    // This will get the subsupport actors too
    for (uint32_t i = 0; i < lowerSupportChunkCount; ++i)
    {
        new (actors + i) Actor();
    }

    // Visible chunk index links - initialize to solitary links (0xFFFFFFFF fields)
    memset(header->getVisibleChunkIndexLinks(), 0xFF, sizeData.chunkCount*sizeof(IndexDLink<uint32_t>));

    // Chunk actor IDs - initialize to invalid (0xFFFFFFFF)
    memset(header->getChunkActorIndices(), 0xFF, upperSupportChunkCount*sizeof(uint32_t));

    // Graph node index links - initialize to solitary links
    memset(header->getGraphNodeIndexLinks(), 0xFF, sizeData.nodeCount*sizeof(uint32_t));

    // Healths are initialized to 0 - the entire memory block is already set to 0 above
    // memset(header->getLowerSupportChunkHealths(), 0, lowerSupportChunkCount*sizeof(float));
    // memset(header->getBondHealths(), 0, bondCount*sizeof(float));

    // FamilyGraph ctor
    new (header->getFamilyGraph()) FamilyGraph(sizeData.nodeCount, sizeData.bondCount);

    return family;
}

// this path is taken when an asset already exists and a family is to be created from it directly
static NvBlastFamily* createFamily(void* mem, const NvBlastAsset* asset, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(asset != nullptr, logFn, "createFamily: NULL asset pointer input.", return nullptr);
    const Asset* solverAsset = static_cast<const Asset*>(asset);

    // pull count info from the asset and use that to initialize the family buffers
    NvBlastAssetMemSizeData sizeData = NvBlastAssetMemSizeDataFromAsset(solverAsset);
    NvBlastFamily* family = createFamily(mem, sizeData, logFn);
    if (family != nullptr)
    {
        // set the asset ID and pointer since we have them available
        FamilyHeader* header = reinterpret_cast<FamilyHeader*>(family);
        header->m_assetID = solverAsset->m_ID;
        header->m_asset = solverAsset;
    }

    return family;
}


//////// Family member methods ////////

void FamilyHeader::fractureSubSupportNoEvents(uint32_t chunkIndex, uint32_t suboffset, float healthDamage, float* chunkHealths, const NvBlastChunk* chunks)
{
    const NvBlastChunk& chunk = chunks[chunkIndex];
    uint32_t numChildren = chunk.childIndexStop - chunk.firstChildIndex;

    if (numChildren > 0)
    {
        healthDamage /= numChildren;
        for (uint32_t childIndex = chunk.firstChildIndex; childIndex < chunk.childIndexStop; childIndex++)
        {
            float& health = chunkHealths[childIndex - suboffset];
            if (canTakeDamage(health))
            {
                float remainingDamage = healthDamage - health;
                health -= healthDamage;

                NVBLAST_ASSERT(chunks[childIndex].parentChunkIndex == chunkIndex);

                if (health <= 0.0f && remainingDamage > 0.0f)
                {
                    fractureSubSupportNoEvents(childIndex, suboffset, remainingDamage, chunkHealths, chunks);
                }
            }
        }
    }
}


void FamilyHeader::fractureSubSupport(uint32_t chunkIndex, uint32_t suboffset, float healthDamage, float* chunkHealths, const NvBlastChunk* chunks, NvBlastChunkFractureData* outBuffer, uint32_t* currentIndex, const uint32_t maxCount)
{
    const NvBlastChunk& chunk = chunks[chunkIndex];
    uint32_t numChildren = chunk.childIndexStop - chunk.firstChildIndex;

    if (numChildren > 0)
    {
        healthDamage /= numChildren;
        for (uint32_t childIndex = chunk.firstChildIndex; childIndex < chunk.childIndexStop; childIndex++)
        {
            float& health = chunkHealths[childIndex - suboffset];
            if (canTakeDamage(health))
            {
                float remainingDamage = healthDamage - health;
                health -= healthDamage;

                NVBLAST_ASSERT(chunks[childIndex].parentChunkIndex == chunkIndex);

                if (*currentIndex < maxCount)
                {
                    NvBlastChunkFractureData& event = outBuffer[*currentIndex];
                    event.userdata = chunks[childIndex].userData;
                    event.chunkIndex = childIndex;
                    event.health = health;
                }
                (*currentIndex)++;

                if (health <= 0.0f && remainingDamage > 0.0f)
                {
                    fractureSubSupport(childIndex, suboffset, remainingDamage, chunkHealths, chunks, outBuffer, currentIndex, maxCount);
                }
            }
        }
    }

}


void FamilyHeader::fractureNoEvents(uint32_t chunkFractureCount, const NvBlastChunkFractureData* chunkFractures, Actor* filterActor, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(m_asset != nullptr, logFn, "FamilyHeader::fractureNoEvents: m_asset is NULL.", return);

    const SupportGraph& graph = m_asset->m_graph;
    const uint32_t* graphAdjacencyPartition = graph.getAdjacencyPartition();
    const uint32_t* adjacentBondIndices = graph.getAdjacentBondIndices();
    float* bondHealths = getBondHealths();
    float* chunkHealths = getLowerSupportChunkHealths();
    float* subChunkHealths = getSubsupportChunkHealths();
    const NvBlastChunk* chunks = m_asset->getChunks();

    for (uint32_t i = 0; i < chunkFractureCount; ++i)
    {
        const NvBlastChunkFractureData& command = chunkFractures[i];
        const uint32_t chunkIndex = command.chunkIndex;
        const uint32_t chunkHealthIndex = m_asset->getContiguousLowerSupportIndex(chunkIndex);
        NVBLAST_ASSERT(!isInvalidIndex(chunkHealthIndex));
        if (isInvalidIndex(chunkHealthIndex))
        {
            continue;
        }
        float& health = chunkHealths[chunkHealthIndex];
        if (canTakeDamage(health) && command.health > 0.0f)
        {
            Actor* actor = getChunkActor(chunkIndex);
            if (filterActor && filterActor != actor)
            {
                NVBLASTLL_LOG_WARNING(logFn, "NvBlastActorApplyFracture: chunk fracture command corresponds to other actor, command is ignored.");
            }
            else if (actor)
            {
                const uint32_t nodeIndex = m_asset->getChunkToGraphNodeMap()[chunkIndex];
                if (actor->getGraphNodeCount() > 1 && !isInvalidIndex(nodeIndex))
                {
                    for (uint32_t adjacentIndex = graphAdjacencyPartition[nodeIndex]; adjacentIndex < graphAdjacencyPartition[nodeIndex + 1]; adjacentIndex++)
                    {
                        const uint32_t bondIndex = adjacentBondIndices[adjacentIndex];
                        NVBLAST_ASSERT(!isInvalidIndex(bondIndex));
                        if (bondHealths[bondIndex] > 0.0f)
                        {
                            bondHealths[bondIndex] = 0.0f;
                        }
                    }
                    getFamilyGraph()->notifyNodeRemoved(actor->getIndex(), nodeIndex, &graph);
                }

                health -= command.health;

                const float remainingDamage = -health;

                if (remainingDamage > 0.0f) // node chunk has been damaged beyond its health
                {
                    fractureSubSupportNoEvents(chunkIndex, m_asset->m_firstSubsupportChunkIndex, remainingDamage, subChunkHealths, chunks);
                }
            }
        }
    }
}


void FamilyHeader::fractureWithEvents(uint32_t chunkFractureCount, const NvBlastChunkFractureData* commands, NvBlastChunkFractureData* events, uint32_t eventsSize, uint32_t* count, Actor* filterActor, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(m_asset != nullptr, logFn, "FamilyHeader::fractureWithEvents: m_asset is NULL.", return);

    const SupportGraph& graph = m_asset->m_graph;
    const uint32_t* graphAdjacencyPartition = graph.getAdjacencyPartition();
    const uint32_t* adjacentBondIndices = graph.getAdjacentBondIndices();
    float* bondHealths = getBondHealths();
    float* chunkHealths = getLowerSupportChunkHealths();
    float* subChunkHealths = getSubsupportChunkHealths();
    const NvBlastChunk* chunks = m_asset->getChunks();

    for (uint32_t i = 0; i < chunkFractureCount; ++i)
    {
        const NvBlastChunkFractureData& command = commands[i];
        const uint32_t chunkIndex = command.chunkIndex;
        const uint32_t chunkHealthIndex = m_asset->getContiguousLowerSupportIndex(chunkIndex);
        NVBLAST_ASSERT(!isInvalidIndex(chunkHealthIndex));
        if (isInvalidIndex(chunkHealthIndex))
        {
            continue;
        }
        float& health = chunkHealths[chunkHealthIndex];
        if (canTakeDamage(health) && command.health > 0.0f)
        {
            Actor* actor = getChunkActor(chunkIndex);
            if (filterActor && filterActor != actor)
            {
                NVBLASTLL_LOG_WARNING(logFn, "NvBlastActorApplyFracture: chunk fracture command corresponds to other actor, command is ignored.");
            }
            else if (actor)
            {
                const uint32_t nodeIndex = m_asset->getChunkToGraphNodeMap()[chunkIndex];
                if (actor->getGraphNodeCount() > 1 && !isInvalidIndex(nodeIndex))
                {
                    for (uint32_t adjacentIndex = graphAdjacencyPartition[nodeIndex]; adjacentIndex < graphAdjacencyPartition[nodeIndex + 1]; adjacentIndex++)
                    {
                        const uint32_t bondIndex = adjacentBondIndices[adjacentIndex];
                        NVBLAST_ASSERT(!isInvalidIndex(bondIndex));
                        if (bondHealths[bondIndex] > 0.0f)
                        {
                            bondHealths[bondIndex] = 0.0f;
                        }
                    }
                    getFamilyGraph()->notifyNodeRemoved(actor->getIndex(), nodeIndex, &graph);
                }

                health -= command.health;

                if (*count < eventsSize)
                {
                    NvBlastChunkFractureData& outEvent = events[*count];
                    outEvent.userdata = chunks[chunkIndex].userData;
                    outEvent.chunkIndex = chunkIndex;
                    outEvent.health = health;
                }
                (*count)++;

                const float remainingDamage = -health;

                if (remainingDamage > 0.0f) // node chunk has been damaged beyond its health
                {
                    fractureSubSupport(chunkIndex, m_asset->m_firstSubsupportChunkIndex, remainingDamage, subChunkHealths, chunks, events, count, eventsSize);
                }
            }
        }
    }
}


void FamilyHeader::fractureInPlaceEvents(uint32_t chunkFractureCount, NvBlastChunkFractureData* inoutbuffer, uint32_t eventsSize, uint32_t* count, Actor* filterActor, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(m_asset != nullptr, logFn, "FamilyHeader::fractureInPlaceEvents: m_asset is NULL.", return);

    const SupportGraph& graph = m_asset->m_graph;
    const uint32_t* graphAdjacencyPartition = graph.getAdjacencyPartition();
    const uint32_t* adjacentBondIndices = graph.getAdjacentBondIndices();
    float* bondHealths = getBondHealths();
    float* chunkHealths = getLowerSupportChunkHealths();
    float* subChunkHealths = getSubsupportChunkHealths();
    const NvBlastChunk* chunks = m_asset->getChunks();

    //
    // First level Chunk Fractures
    //

    for (uint32_t i = 0; i < chunkFractureCount; ++i)
    {
        const NvBlastChunkFractureData& command = inoutbuffer[i];
        const uint32_t chunkIndex = command.chunkIndex;
        const uint32_t chunkHealthIndex = m_asset->getContiguousLowerSupportIndex(chunkIndex);
        NVBLAST_ASSERT(!isInvalidIndex(chunkHealthIndex));
        if (isInvalidIndex(chunkHealthIndex))
        {
            continue;
        }
        float& health = chunkHealths[chunkHealthIndex];
        if (canTakeDamage(health) && command.health > 0.0f)
        {
            Actor* actor = getChunkActor(chunkIndex);
            if (filterActor && filterActor != actor)
            {
                NVBLASTLL_LOG_WARNING(logFn, "NvBlastActorApplyFracture: chunk fracture command corresponds to other actor, command is ignored.");
            }
            else if (actor)
            {
                const uint32_t nodeIndex = m_asset->getChunkToGraphNodeMap()[chunkIndex];
                if (actor->getGraphNodeCount() > 1 && !isInvalidIndex(nodeIndex))
                {
                    for (uint32_t adjacentIndex = graphAdjacencyPartition[nodeIndex]; adjacentIndex < graphAdjacencyPartition[nodeIndex + 1]; adjacentIndex++)
                    {
                        const uint32_t bondIndex = adjacentBondIndices[adjacentIndex];
                        NVBLAST_ASSERT(!isInvalidIndex(bondIndex));
                        if (bondHealths[bondIndex] > 0.0f)
                        {
                            bondHealths[bondIndex] = 0.0f;
                        }
                    }
                    getFamilyGraph()->notifyNodeRemoved(actor->getIndex(), nodeIndex, &graph);
                }

                health -= command.health;

                NvBlastChunkFractureData& outEvent = inoutbuffer[(*count)++];
                outEvent.userdata = chunks[chunkIndex].userData;
                outEvent.chunkIndex = chunkIndex;
                outEvent.health = health;
            }
        }
    }

    //
    // Hierarchical Chunk Fractures
    //

    uint32_t commandedChunkFractures = *count;

    for (uint32_t i = 0; i < commandedChunkFractures; ++i)
    {
        NvBlastChunkFractureData& event = inoutbuffer[i];
        const uint32_t chunkIndex = event.chunkIndex;

        const float remainingDamage = -event.health;
        if (remainingDamage > 0.0f) // node chunk has been damaged beyond its health
        {
            fractureSubSupport(chunkIndex, m_asset->m_firstSubsupportChunkIndex, remainingDamage, subChunkHealths, chunks, inoutbuffer, count, eventsSize);
        }
    }
}


void FamilyHeader::applyFracture(NvBlastFractureBuffers* eventBuffers, const NvBlastFractureBuffers* commands, Actor* filterActor, NvBlastLog logFn, NvBlastTimers* timers)
{
    NVBLASTLL_CHECK(m_asset != nullptr, logFn, "FamilyHeader::applyFracture: m_asset is NULL.", return);

    NVBLASTLL_CHECK(commands != nullptr, logFn, "NvBlastActorApplyFracture: NULL commands pointer input.", return);
    NVBLASTLL_CHECK(isValid(commands), logFn, "NvBlastActorApplyFracture: commands memory is NULL but size is > 0.", return);
    NVBLASTLL_CHECK(eventBuffers == nullptr || isValid(eventBuffers), logFn, "NvBlastActorApplyFracture: eventBuffers memory is NULL but size is > 0.",
        eventBuffers->bondFractureCount = 0; eventBuffers->chunkFractureCount = 0; return);

#if NVBLASTLL_CHECK_PARAMS
    if (eventBuffers != nullptr && eventBuffers->bondFractureCount == 0 && eventBuffers->chunkFractureCount == 0)
    {
        NVBLASTLL_LOG_WARNING(logFn, "NvBlastActorApplyFracture: eventBuffers do not provide any space.");
        return;
    }
#endif

#if NV_PROFILE
    Time time;
#else
    NV_UNUSED(timers);
#endif

    //
    // Chunk Fracture
    //

    if (eventBuffers == nullptr || eventBuffers->chunkFractures == nullptr)
    {
        // immediate hierarchical fracture
        fractureNoEvents(commands->chunkFractureCount, commands->chunkFractures, filterActor, logFn);
    }
    else if (eventBuffers->chunkFractures != commands->chunkFractures)
    {
        // immediate hierarchical fracture
        uint32_t count = 0;
        fractureWithEvents(commands->chunkFractureCount, commands->chunkFractures, eventBuffers->chunkFractures, eventBuffers->chunkFractureCount, &count, filterActor, logFn);

        if (count > eventBuffers->chunkFractureCount)
        {
            NVBLASTLL_LOG_WARNING(logFn, "NvBlastActorApplyFracture: eventBuffers too small. Chunk events were lost.");
        }
        else
        {
            eventBuffers->chunkFractureCount = count;
        }
    }
    else if (eventBuffers->chunkFractures == commands->chunkFractures)
    {
        // compacting first
        uint32_t count = 0;
        fractureInPlaceEvents(commands->chunkFractureCount, commands->chunkFractures, eventBuffers->chunkFractureCount, &count, filterActor, logFn);

        if (count > eventBuffers->chunkFractureCount)
        {
            NVBLASTLL_LOG_WARNING(logFn, "NvBlastActorApplyFracture: eventBuffers too small. Chunk events were lost.");
        }
        else
        {
            eventBuffers->chunkFractureCount = count;
        }
    }

    //
    // Bond Fracture
    //

    uint32_t outCount = 0;
    const uint32_t eventBufferSize = eventBuffers ? eventBuffers->bondFractureCount : 0;

    NvBlastBond* bonds = m_asset->getBonds();
    float* bondHealths = getBondHealths();
    const uint32_t* graphChunkIndices = m_asset->m_graph.getChunkIndices();
    for (uint32_t i = 0; i < commands->bondFractureCount; ++i)
    {
        const NvBlastBondFractureData& frac = commands->bondFractures[i];
        NVBLAST_ASSERT(frac.nodeIndex0 < m_asset->m_graph.m_nodeCount);
        NVBLAST_ASSERT(frac.nodeIndex1 < m_asset->m_graph.m_nodeCount);
        uint32_t chunkIndex0 = graphChunkIndices[frac.nodeIndex0];
        uint32_t chunkIndex1 = graphChunkIndices[frac.nodeIndex1];
        NVBLAST_ASSERT(!isInvalidIndex(chunkIndex0) || !isInvalidIndex(chunkIndex1));
        Actor* actor0 = !isInvalidIndex(chunkIndex0) ? getChunkActor(chunkIndex0) : nullptr;
        Actor* actor1 = !isInvalidIndex(chunkIndex1) ? getChunkActor(chunkIndex1) : nullptr;
        NVBLAST_ASSERT(actor0 != nullptr || actor1 != nullptr);
        // If actors are not nullptr and different then bond is already broken
        // One of actor can be nullptr which probably means it's 'world' node.
        if (actor0 == actor1 || actor0 == nullptr || actor1 == nullptr)
        {
            Actor* actor = actor0 ? actor0 : actor1;
            NVBLAST_ASSERT_WITH_MESSAGE(actor, "NvBlastActorApplyFracture: all actors in bond fracture command are nullptr, command will be safely ignored, but investigation is recommended.");
            if (filterActor && filterActor != actor)
            {
                NVBLASTLL_LOG_WARNING(logFn, "NvBlastActorApplyFracture: bond fracture command corresponds to other actor, command is ignored.");
            }
            else if (actor)
            {
                const uint32_t bondIndex = actor->damageBond(frac.nodeIndex0, frac.nodeIndex1, frac.health);
                if (!isInvalidIndex(bondIndex))
                {
                    if (eventBuffers && eventBuffers->bondFractures)
                    {
                        if (outCount < eventBufferSize)
                        {
                            NvBlastBondFractureData& outEvent = eventBuffers->bondFractures[outCount];
                            outEvent.userdata = bonds[bondIndex].userData;
                            outEvent.nodeIndex0 = frac.nodeIndex0;
                            outEvent.nodeIndex1 = frac.nodeIndex1;
                            outEvent.health = bondHealths[bondIndex];
                        }
                    }
                    outCount++;
                }
            }
        }
    }

    if (eventBuffers && eventBuffers->bondFractures)
    {
        if (outCount > eventBufferSize)
        {
            NVBLASTLL_LOG_WARNING(logFn, "NvBlastActorApplyFracture: eventBuffers too small. Bond events were lost.");
        }
        else
        {
            eventBuffers->bondFractureCount = outCount;
        }
    }

#if NV_PROFILE
    if (timers != nullptr)
    {
        timers->fracture += time.getElapsedTicks();
    }
#endif

}


} // namespace Blast
} // namespace Nv


// API implementation

extern "C"
{

NvBlastAssetMemSizeData NvBlastAssetMemSizeDataFromAsset(const NvBlastAsset* asset)
{
    const Nv::Blast::Asset* solverAsset = reinterpret_cast<const Nv::Blast::Asset*>(asset);

    NvBlastAssetMemSizeData sizeData;
    if (solverAsset)
    {
        sizeData.bondCount = solverAsset->getBondCount();
        sizeData.chunkCount = solverAsset->m_chunkCount;
        sizeData.nodeCount = solverAsset->m_graph.m_nodeCount;

        sizeData.lowerSupportChunkCount = solverAsset->getLowerSupportChunkCount();
        sizeData.upperSupportChunkCount = solverAsset->getUpperSupportChunkCount();
    }
    else
    {
        memset(&sizeData, 0, sizeof(NvBlastAssetMemSizeData));
    }

    return sizeData;
}

NvBlastFamily* NvBlastAssetCreateFamily(void* mem, const NvBlastAsset* asset, NvBlastLog logFn)
{
    return Nv::Blast::createFamily(mem, asset, logFn);
}


NvBlastFamily* NvBlastAssetCreateFamilyFromSizeData(void* mem, const NvBlastAssetMemSizeData& sizeData, NvBlastLog logFn)
{
    return Nv::Blast::createFamily(mem, sizeData, logFn);
}


uint32_t NvBlastFamilyGetFormatVersion(const NvBlastFamily* family, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(family != nullptr, logFn, "NvBlastFamilyGetFormatVersion: NULL family pointer input.", return UINT32_MAX);
    return reinterpret_cast<const Nv::Blast::FamilyHeader*>(family)->formatVersion;
}


const NvBlastAsset* NvBlastFamilyGetAsset(const NvBlastFamily* family, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(family != nullptr, logFn, "NvBlastFamilyGetAsset: NULL family pointer input.", return nullptr);
    return reinterpret_cast<const Nv::Blast::FamilyHeader*>(family)->m_asset;
}


void NvBlastFamilySetAsset(NvBlastFamily* family, const NvBlastAsset* asset, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(family != nullptr, logFn, "NvBlastFamilySetAsset: NULL family pointer input.", return);
    NVBLASTLL_CHECK(asset != nullptr, logFn, "NvBlastFamilySetAsset: NULL asset pointer input.", return);

    Nv::Blast::FamilyHeader* header = reinterpret_cast<Nv::Blast::FamilyHeader*>(family);
    const Nv::Blast::Asset* solverAsset = reinterpret_cast<const Nv::Blast::Asset*>(asset);

    if (memcmp(&header->m_assetID, &solverAsset->m_ID, sizeof(NvBlastID)))
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastFamilySetAsset: wrong asset.  Passed asset ID doesn't match family asset ID.");
        return;
    }

    header->m_asset = solverAsset;
}


uint32_t NvBlastFamilyGetSize(const NvBlastFamily* family, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(family != nullptr, logFn, "NvBlastFamilyGetSize: NULL family pointer input.", return 0);
    return reinterpret_cast<const Nv::Blast::FamilyHeader*>(family)->size;
}


NvBlastID NvBlastFamilyGetAssetID(const NvBlastFamily* family, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(family != nullptr, logFn, "NvBlastFamilyGetAssetID: NULL family pointer input.", return NvBlastID());
    return reinterpret_cast<const Nv::Blast::FamilyHeader*>(family)->m_assetID;
}


uint32_t NvBlastFamilyGetActorCount(const NvBlastFamily* family, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(family != nullptr, logFn, "NvBlastFamilyGetActorCount: NULL family pointer input.", return 0);

    const Nv::Blast::FamilyHeader* header = reinterpret_cast<const Nv::Blast::FamilyHeader*>(family);

    return header->m_actorCount;
}


uint32_t NvBlastFamilyGetActors(NvBlastActor** actors, uint32_t actorsSize, const NvBlastFamily* family, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(actors != nullptr, logFn, "NvBlastFamilyGetActors: NULL actors pointer input.", return 0);
    NVBLASTLL_CHECK(family != nullptr, logFn, "NvBlastFamilyGetActors: NULL family pointer input.", return 0);

    const Nv::Blast::FamilyHeader* header = reinterpret_cast<const Nv::Blast::FamilyHeader*>(family);

    // Iterate through active actors and write to supplied array
    const uint32_t familyActorCount = header->getActorsArraySize();
    Nv::Blast::Actor* familyActor = header->getActors();
    uint32_t actorCount = 0;
    for (uint32_t i = 0; actorCount < actorsSize && i < familyActorCount; ++i, ++familyActor)
    {
        if (familyActor->isActive())
        {
            actors[actorCount++] = familyActor;
        }
    }

    return actorCount;
}


NvBlastActor* NvBlastFamilyGetActorByIndex(const NvBlastFamily* family, uint32_t actorIndex, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(family != nullptr, logFn, "NvBlastFamilyGetActorByIndex: NULL family pointer input.", return nullptr);
    const Nv::Blast::FamilyHeader* header = reinterpret_cast<const Nv::Blast::FamilyHeader*>(family);
    return header->getActorByIndex(actorIndex);
}


NvBlastActor* NvBlastFamilyGetChunkActor(const NvBlastFamily* family, uint32_t chunkIndex, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(family != nullptr, logFn, "NvBlastFamilyGetChunkActor: NULL family pointer input.", return nullptr);

    const Nv::Blast::FamilyHeader* header = reinterpret_cast<const Nv::Blast::FamilyHeader*>(family);

    NVBLASTLL_CHECK(header->m_asset != nullptr, logFn, "NvBlastFamilyGetChunkActor: NvBlastFamily has null asset set.", return nullptr);
    NVBLASTLL_CHECK(chunkIndex < header->m_asset->m_chunkCount, logFn, "NvBlastFamilyGetChunkActor: bad value of chunkIndex for the given family's asset.", return nullptr);

    return header->getChunkActor(chunkIndex);
}


uint32_t* NvBlastFamilyGetChunkActorIndices(const NvBlastFamily* family, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(family != nullptr, logFn, "NvBlastFamilyGetChunkActorIndices: NULL family pointer input.", return nullptr);

    const Nv::Blast::FamilyHeader* header = reinterpret_cast<const Nv::Blast::FamilyHeader*>(family);

    NVBLASTLL_CHECK(header->m_asset != nullptr, logFn, "NvBlastFamilyGetChunkActorIndices: NvBlastFamily has null asset set.", return nullptr);

    return header->getChunkActorIndices();
}


uint32_t NvBlastFamilyGetMaxActorCount(const NvBlastFamily* family, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(family != nullptr, logFn, "NvBlastFamilyGetMaxActorCount: NULL family pointer input.", return 0);
    const Nv::Blast::FamilyHeader* header = reinterpret_cast<const Nv::Blast::FamilyHeader*>(family);
    return header->getActorsArraySize();
}

} // extern "C"

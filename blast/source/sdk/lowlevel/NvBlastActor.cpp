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


#include "NvBlastActor.h"
#include "NvBlastFamilyGraph.h"
#include "NvBlastChunkHierarchy.h"
#include "NvBlastIndexFns.h"
#include "NvBlastDLink.h"
#include "NvBlastGeometry.h"
#include "NvBlastTime.h"
#include <float.h>
#include <algorithm>


namespace Nv
{
namespace Blast
{

//////// Actor static methods ////////

size_t Actor::createRequiredScratch(const NvBlastFamily* family, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(family != nullptr && reinterpret_cast<const FamilyHeader*>(family)->m_asset != nullptr, logFn, "Actor::createRequiredScratch: NULL family input or asset.", return 0);

    const Asset& solverAsset = *reinterpret_cast<const FamilyHeader*>(family)->m_asset;
    return FamilyGraph::findIslandsRequiredScratch(solverAsset.m_graph.m_nodeCount);
}


Actor* Actor::create(NvBlastFamily* family, const NvBlastActorDesc* desc, void* scratch, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(family != nullptr, logFn, "Actor::create: NULL family pointer input.", return nullptr);
    NVBLASTLL_CHECK(reinterpret_cast<FamilyHeader*>(family)->m_asset != nullptr, logFn, "Actor::create: family has NULL asset.", return nullptr);
    NVBLASTLL_CHECK(reinterpret_cast<FamilyHeader*>(family)->m_asset->m_graph.m_nodeCount != 0, logFn, "Actor::create: family's asset has no support chunks.", return nullptr);
    NVBLASTLL_CHECK(desc != nullptr, logFn, "Actor::create: NULL desc pointer input.", return nullptr);
    NVBLASTLL_CHECK(scratch != nullptr, logFn, "Actor::create: NULL scratch input.", return nullptr);

    FamilyHeader* header = reinterpret_cast<FamilyHeader*>(family);

    if (header->m_actorCount > 0)
    {
        NVBLASTLL_LOG_ERROR(logFn, "Actor::create: input family is not empty.");
        return nullptr;
    }

    const Asset& solverAsset = *static_cast<const Asset*>(header->m_asset);
    const SupportGraph& graph = solverAsset.m_graph;

    // Lower support chunk healths - initialize
    float* lowerSupportChunkHealths = header->getLowerSupportChunkHealths();
    if (desc->initialSupportChunkHealths != nullptr)    // Health array given
    {
        const uint32_t* supportChunkIndices = graph.getChunkIndices();
        for (uint32_t supportChunkNum = 0; supportChunkNum < graph.m_nodeCount; ++supportChunkNum)
        {
            const float initialHealth = desc->initialSupportChunkHealths[supportChunkNum];
            for (Asset::DepthFirstIt i(solverAsset, supportChunkIndices[supportChunkNum]); (bool)i; ++i)
            {
                lowerSupportChunkHealths[solverAsset.getContiguousLowerSupportIndex((uint32_t)i)] = initialHealth;
            }
        }
    }
    else    // Use uniform initialization
    {
        const uint32_t lowerSupportChunkCount = solverAsset.getLowerSupportChunkCount();
        for (uint32_t i = 0; i < lowerSupportChunkCount; ++i)
        {
            lowerSupportChunkHealths[i] = desc->uniformInitialLowerSupportChunkHealth;
        }
    }

    // Bond healths - initialize
    const uint32_t bondCount = solverAsset.getBondCount();
    float* bondHealths = header->getBondHealths();
    if (desc->initialBondHealths != nullptr)    // Health array given
    {
        memcpy(bondHealths, desc->initialBondHealths, bondCount * sizeof(float));
    }
    else    // Use uniform initialization
    {
        for (uint32_t bondNum = 0; bondNum < bondCount; ++bondNum)
        {
            bondHealths[bondNum] = desc->uniformInitialBondHealth;
        }
    }

    // Get first actor - NOTE: we don't send an event for this!  May need to do so for consistency.
    Actor* actor = header->borrowActor(0);    // Using actor[0]

    // Fill in actor fields
    actor->m_firstGraphNodeIndex = 0;
    actor->m_graphNodeCount = graph.m_nodeCount;
    actor->m_leafChunkCount = solverAsset.m_leafChunkCount;

    // Graph node index links - initialize to chain
    uint32_t* graphNodeLinks = header->getGraphNodeIndexLinks();
    for (uint32_t i = 0; i < graph.m_nodeCount - 1; ++i)
    {
        graphNodeLinks[i] = i + 1;
    }
    graphNodeLinks[graph.m_nodeCount - 1] = invalidIndex<uint32_t>();

    // Update visible chunks (we assume that all chunks belong to one actor at the beginning)
    actor->updateVisibleChunksFromGraphNodes();

    // Initialize instance graph with this actor
    header->getFamilyGraph()->initialize(actor->getIndex(), &graph);

    // Call findIslands to set up the internal instance graph data
    header->getFamilyGraph()->findIslands(actor->getIndex(), scratch, &graph);

    return actor;
}


//////// Actor member methods ////////

uint32_t Actor::damageBond(uint32_t nodeIndex0, uint32_t nodeIndex1, float healthDamage)
{
    const uint32_t bondIndex = getGraph()->findBond(nodeIndex0, nodeIndex1);
    damageBond(nodeIndex0, nodeIndex1, bondIndex, healthDamage);
    return bondIndex;
}


void Actor::damageBond(uint32_t nodeIndex0, uint32_t nodeIndex1, uint32_t bondIndex, float healthDamage)
{
    if (bondIndex == invalidIndex<uint32_t>())
    {
        NVBLAST_ALWAYS_ASSERT();
        return;
    }

    float* bondHealths = getBondHealths();
    if (canTakeDamage(bondHealths[bondIndex]) && healthDamage > 0.0f)
    {
        // Subtract health
        bondHealths[bondIndex] -= healthDamage;

        // Was removed?
        if (bondHealths[bondIndex] <= 0.0f)
        {
            // Notify graph that bond was removed
            getFamilyGraph()->notifyEdgeRemoved(getIndex(), nodeIndex0, nodeIndex1, bondIndex, getGraph());
            bondHealths[bondIndex] = 0.0f;    // Doing this for single-actor serialization consistency; should not actually be necessary
        }
    }
}


uint32_t Actor::damageBond(const NvBlastBondFractureData& cmd)
{
    NVBLAST_ASSERT(!isInvalidIndex(cmd.nodeIndex1));
    return damageBond(cmd.nodeIndex0, cmd.nodeIndex1, cmd.health);
}


void Actor::generateFracture(NvBlastFractureBuffers* commandBuffers, const NvBlastDamageProgram& program, const void* programParams,
    NvBlastLog logFn, NvBlastTimers* timers) const
{
    NVBLASTLL_CHECK(commandBuffers != nullptr, logFn, "Actor::generateFracture: NULL commandBuffers pointer input.", return);
    NVBLASTLL_CHECK(isValid(commandBuffers), logFn, "NvBlastActorGenerateFracture: commandBuffers memory is NULL but size is > 0.",
        commandBuffers->bondFractureCount = 0; commandBuffers->chunkFractureCount = 0; return);

#if NVBLASTLL_CHECK_PARAMS
    if (commandBuffers->bondFractureCount == 0 && commandBuffers->chunkFractureCount == 0)
    {
        NVBLASTLL_LOG_WARNING(logFn, "NvBlastActorGenerateFracture: commandBuffers do not provide any space.");
        return;
    }
#endif

#if NV_PROFILE
    Time time;
#else
    NV_UNUSED(timers);
#endif

    const SupportGraph* graph = getGraph();

    const uint32_t graphNodeCount = getGraphNodeCount();
    if (graphNodeCount > 1 && program.graphShaderFunction != nullptr)
    {
        const NvBlastGraphShaderActor shaderActor = {
            getIndex(),
            getGraphNodeCount(),
            graph->m_nodeCount,
            getFirstGraphNodeIndex(),
            getGraphNodeIndexLinks(),
            graph->getChunkIndices(),
            graph->getAdjacencyPartition(),
            graph->getAdjacentNodeIndices(),
            graph->getAdjacentBondIndices(),
            getBonds(),
            getChunks(),
            getBondHealths(),
            getLowerSupportChunkHealths(),
            getFamilyHeader()->getFamilyGraph()->getIslandIds()
        };

        program.graphShaderFunction(commandBuffers, &shaderActor, programParams);
    }
    else if (graphNodeCount <= 1 && program.subgraphShaderFunction != nullptr)
    {
        const NvBlastSubgraphShaderActor shaderActor = {
            // The conditional (visible vs. support chunk) is needed because we allow single-child chunk chains
            // This makes it possible that an actor with a single support chunk will have a different visible chunk (ancestor of the support chunk)
            graphNodeCount == 1 ? graph->getChunkIndices()[getFirstGraphNodeIndex()] : getFirstVisibleChunkIndex(),
            getChunks()
        };

        program.subgraphShaderFunction(commandBuffers, &shaderActor, programParams);
    }
    else
    {
        commandBuffers->bondFractureCount = 0;
        commandBuffers->chunkFractureCount = 0;
    }

#if NV_PROFILE
    if (timers != nullptr)
    {
        timers->material += time.getElapsedTicks();
    }
#endif
}




size_t Actor::splitRequiredScratch() const
{
    // Scratch is reused, just need the max of these two values
    return std::max(m_graphNodeCount * sizeof(uint32_t), static_cast<size_t>(FamilyGraph::findIslandsRequiredScratch(getGraph()->m_nodeCount)));
}


uint32_t Actor::split(NvBlastActorSplitEvent* result, uint32_t newActorsMaxCount, void* scratch, NvBlastLog logFn, NvBlastTimers* timers)
{
    NVBLASTLL_CHECK(result != nullptr, logFn, "Actor::split: NULL result pointer input.", return 0);
    NVBLASTLL_CHECK(newActorsMaxCount > 0 && result->newActors != nullptr, logFn, "NvBlastActorSplit: no space for results provided.", return 0);
    NVBLASTLL_CHECK(scratch != nullptr, logFn, "Actor::split: NULL scratch pointer input.", return 0);

#if NV_PROFILE
    Time time;
#else
    NV_UNUSED(timers);
#endif

    Actor** newActors = reinterpret_cast<Actor**>(result->newActors);

    uint32_t actorsCount = 0;

    if (getGraphNodeCount() <= 1)
    {
        uint32_t chunkHealthIndex = isSingleSupportChunk() ? getIndex() : getFirstVisibleChunkIndex() - getFirstSubsupportChunkIndex() + getGraph()->m_nodeCount;

        float* chunkHealths = getLowerSupportChunkHealths();
        if (chunkHealths[chunkHealthIndex] <= 0.0f)
        {
            actorsCount = partitionSingleLowerSupportChunk(newActors, newActorsMaxCount, logFn);

            for (uint32_t i = 0; i < actorsCount; ++i)
            {
                Actor* newActor = newActors[i];
                uint32_t firstVisible = newActor->getFirstVisibleChunkIndex();
                uint32_t firstSub = newActor->getFirstSubsupportChunkIndex();
                uint32_t nodeCount = newActor->getGraph()->m_nodeCount;
                uint32_t newActorIndex = newActor->getIndex();
                uint32_t healthIndex = newActor->isSubSupportChunk() ? firstVisible - firstSub + nodeCount : newActorIndex;

                if (chunkHealths[healthIndex] <= 0.0f)
                {
                    uint32_t brittleActors = newActors[i]->partitionSingleLowerSupportChunk(&newActors[actorsCount], newActorsMaxCount - actorsCount, logFn);
                    actorsCount += brittleActors;

                    if (brittleActors > 0)
                    {
                        actorsCount--;
                        newActors[i] = newActors[actorsCount];
                        i--;
                    }
                }
            }
        }


#if NV_PROFILE
        if (timers != nullptr)
        {
            timers->partition += time.getElapsedTicks();
        }
#endif
    }
    else
    {
        findIslands(scratch);

#if NV_PROFILE
        if (timers != nullptr)
        {
            timers->island += time.getElapsedTicks();
        }
#endif

        // Reuse scratch for node list
        uint32_t* graphNodeIndexList = reinterpret_cast<uint32_t*>(scratch);

        // Get the family header
        FamilyHeader* header = getFamilyHeader();
        NVBLAST_ASSERT(header != nullptr);    // If m_actorEntryDataIndex is valid, this should be too

        // Record nodes in this actor before splitting
        const uint32_t* graphNodeIndexLinks = header->getGraphNodeIndexLinks(); // Get the links for the graph nodes
        uint32_t graphNodeIndexCount = 0;
        for (uint32_t graphNodeIndex = m_firstGraphNodeIndex; !isInvalidIndex(graphNodeIndex); graphNodeIndex = graphNodeIndexLinks[graphNodeIndex])
        {
            if (graphNodeIndexCount >= m_graphNodeCount)
            {
                // Safety, splitRequiredScratch() only guarantees m_graphNodeCount elements.  In any case, this condition shouldn't happen.
                NVBLAST_ASSERT(graphNodeIndexCount < m_graphNodeCount);
                break;
            }
            graphNodeIndexList[graphNodeIndexCount++] = graphNodeIndex;
        }

        actorsCount = partitionMultipleGraphNodes(newActors, newActorsMaxCount, logFn);

        if (actorsCount > 1)
        {
#if NV_PROFILE
            if (timers != nullptr)
            {
                timers->partition += time.getElapsedTicks();
            }
#endif

            // Get various pointers and values to iterate
            const Asset* asset = getAsset();
            Actor* actors = header->getActors();
            IndexDLink<uint32_t>* visibleChunkIndexLinks = header->getVisibleChunkIndexLinks();
            uint32_t* chunkActorIndices = header->getChunkActorIndices();
            const SupportGraph& graph = asset->m_graph;
            const uint32_t* graphChunkIndices = graph.getChunkIndices();
            const NvBlastChunk* chunks = asset->getChunks();
            const uint32_t upperSupportChunkCount = asset->getUpperSupportChunkCount();
            const uint32_t* familyGraphIslandIDs = header->getFamilyGraph()->getIslandIds();

            // Iterate over all graph nodes and update visible chunk lists
            for (uint32_t graphNodeNum = 0; graphNodeNum < graphNodeIndexCount; ++graphNodeNum)
            {
                const uint32_t graphNodeIndex = graphNodeIndexList[graphNodeNum];
                const uint32_t supportChunkIndex = graphChunkIndices[graphNodeIndex];
                if (!isInvalidIndex(supportChunkIndex))    // Invalid if this is the world chunk
                {
                    updateVisibleChunksFromSupportChunk<Actor>(actors, visibleChunkIndexLinks, chunkActorIndices, familyGraphIslandIDs[graphNodeIndex], graphChunkIndices[graphNodeIndex], chunks, upperSupportChunkCount);
                }
            }

            // Remove actors with no visible chunks - this can happen if we've split such that the world node is by itself
            uint32_t actualActorsCount = 0;
            for (uint32_t i = 0; i < actorsCount; ++i)
            {
                newActors[actualActorsCount] = newActors[i];
                if (newActors[actualActorsCount]->getVisibleChunkCount() > 0)
                {
                    ++actualActorsCount;
                }
                else
                {
                    header->returnActor(*newActors[actualActorsCount]);
                }
            }
            actorsCount = actualActorsCount;

#if NV_PROFILE
            if (timers != nullptr)
            {
                timers->visibility += time.getElapsedTicks();
            }
#endif

            // NOTE: we MUST use header->getLowerSupportChunkHealths() instead of just getLowerSupportChunkHealths() here,
            // since this actor has been made inactive at this point.  Therefore Actor::getLowerSupportChunkHealths() will return
            // garbage since it calls getFamilyHeader() which does not return a valid header if the actor is not active.
            const float* chunkHealths = header->getLowerSupportChunkHealths();
            for (uint32_t i = 0; i < actorsCount; ++i)
            {
                Actor* newActor = newActors[i];
                if (newActor->getGraphNodeCount() <= 1)
                {
                    const uint32_t firstVisible = newActor->getFirstVisibleChunkIndex();
                    const uint32_t firstSub = newActor->getFirstSubsupportChunkIndex();
                    const uint32_t assetNodeCount = newActor->getGraph()->m_nodeCount;
                    const uint32_t newActorIndex = newActor->getIndex();
                    const uint32_t healthIndex = newActor->isSubSupportChunk() ? firstVisible - firstSub + assetNodeCount : newActorIndex;

                    // this relies on visibility updated, subsupport actors only have m_firstVisibleChunkIndex to identify the chunk
                    if (chunkHealths[healthIndex] <= 0.0f)
                    {
                        const uint32_t brittleActors = newActor->partitionSingleLowerSupportChunk(&newActors[actorsCount], newActorsMaxCount - actorsCount, logFn);
                        actorsCount += brittleActors;

                        if (brittleActors > 0)
                        {
                            actorsCount--;
                            newActors[i] = newActors[actorsCount];
                            i--;
                        }
                    }
                }
            }

#if NV_PROFILE
            if (timers != nullptr)
            {
                timers->partition += time.getElapsedTicks();
            }
#endif
        }
        else
        {
            actorsCount = 0;
        }
    }

    result->deletedActor = actorsCount == 0 ? nullptr : this;

    return actorsCount;
}


uint32_t Actor::findIslands(void* scratch)
{
    return getFamilyHeader()->getFamilyGraph()->findIslands(getIndex(), scratch, &getAsset()->m_graph);
}


uint32_t Actor::partitionMultipleGraphNodes(Actor** newActors, uint32_t newActorsSize, NvBlastLog logFn)
{
    NVBLAST_ASSERT(newActorsSize == 0 || newActors != nullptr);

    // Check for single subsupport chunk, no partitioning
    if (m_graphNodeCount <= 1)
    {
        NVBLASTLL_LOG_WARNING(logFn, "Nv::Blast::Actor::partitionMultipleGraphNodes: actor is a single lower-support chunk, and cannot be partitioned by this function.");
        return 0;
    }

    FamilyHeader* header = getFamilyHeader();
    NVBLAST_ASSERT(header != nullptr);    // If m_actorEntryDataIndex is valid, this should be too

    // Get the links for the graph nodes
    uint32_t* graphNodeIndexLinks = header->getGraphNodeIndexLinks();

    // Get the graph chunk indices and leaf chunk counts
    const Asset* asset = getAsset();
    const uint32_t* graphChunkIndices = asset->m_graph.getChunkIndices();
    const uint32_t* subtreeLeafChunkCounts = asset->getSubtreeLeafChunkCounts();

    // Distribute graph nodes to new actors
    uint32_t newActorCount = 0;
    const uint32_t thisActorIndex = getIndex();
    m_leafChunkCount = 0;
    const uint32_t* islandIDs = header->getFamilyGraph()->getIslandIds();
    uint32_t lastGraphNodeIndex = invalidIndex<uint32_t>();
    uint32_t nextGraphNodeIndex = invalidIndex<uint32_t>();
    bool overflow = false;
    for (uint32_t graphNodeIndex = m_firstGraphNodeIndex; !isInvalidIndex(graphNodeIndex); graphNodeIndex = nextGraphNodeIndex)
    {
        nextGraphNodeIndex = graphNodeIndexLinks[graphNodeIndex];
        const uint32_t islandID = islandIDs[graphNodeIndex];

        if (islandID == thisActorIndex)
        {
            const uint32_t graphChunkIndex = graphChunkIndices[graphNodeIndex];
            if (!isInvalidIndex(graphChunkIndex))    // Invalid if this is the world chunk
            {
                m_leafChunkCount += subtreeLeafChunkCounts[graphChunkIndex];
            }
            lastGraphNodeIndex = graphNodeIndex;
            continue;    // Leave the chunk in this actor
        }

        // Remove link from this actor
        if (isInvalidIndex(lastGraphNodeIndex))
        {
            m_firstGraphNodeIndex = nextGraphNodeIndex;
        }
        else
        {
            graphNodeIndexLinks[lastGraphNodeIndex] = nextGraphNodeIndex;
        }
        graphNodeIndexLinks[graphNodeIndex] = invalidIndex<uint32_t>();
        --m_graphNodeCount;

        // See if the chunk had been removed
        if (islandID == invalidIndex<uint32_t>())
        {
            continue;
        }

        // Get new actor if the islandID is valid
        Actor* newActor = header->borrowActor(islandID);

        // Check new actor to see if we're adding the first chunk
        if (isInvalidIndex(newActor->m_firstGraphNodeIndex))
        {
            // See if we can fit it in the output list
            if (newActorCount < newActorsSize)
            {
                newActors[newActorCount++] = newActor;
            }
            else
            {
                overflow = true;
            }
        }

        // Put link in new actor
        graphNodeIndexLinks[graphNodeIndex] = newActor->m_firstGraphNodeIndex;
        newActor->m_firstGraphNodeIndex = graphNodeIndex;
        ++newActor->m_graphNodeCount;
        // Add to the actor's leaf chunk count
        const uint32_t graphChunkIndex = graphChunkIndices[graphNodeIndex];
        if (!isInvalidIndex(graphChunkIndex))    // Invalid if this is the world chunk
        {
            newActor->m_leafChunkCount += subtreeLeafChunkCounts[graphChunkIndex];
        }
    }

    if (m_graphNodeCount > 0)
    {
        // There are still chunks in this actor.  See if we can fit this in the output list.
        if (newActorCount < newActorsSize)
        {
            newActors[newActorCount++] = this;
        }
        else
        {
            overflow = true;
        }
    }
    else
    {
        // No more chunks; release this actor.
        release();
    }

    if (overflow)
    {
        NVBLASTLL_LOG_WARNING(logFn, "Nv::Blast::Actor::partitionMultipleGraphNodes: input newActors array could not hold all actors generated.");
    }

    return newActorCount;
}


uint32_t Actor::partitionSingleLowerSupportChunk(Actor** newActors, uint32_t newActorsSize, NvBlastLog logFn)
{
    NVBLAST_ASSERT(newActorsSize == 0 || newActors != nullptr);

    // Ensure this is a single subsupport chunk, no partitioning
    if (m_graphNodeCount > 1)
    {
        NVBLASTLL_LOG_WARNING(logFn, "Nv::Blast::Actor::partitionSingleLowerSupportChunk: actor is not a single lower-support chunk, and cannot be partitioned by this function.");
        return 0;
    }

    FamilyHeader* header = getFamilyHeader();

    // The conditional (visible vs. support chunk) is needed because we allow single-child chunk chains
    // This makes it possible that an actor with a single support chunk will have a different visible chunk (ancestor of the support chunk)
    const uint32_t chunkIndex = m_graphNodeCount == 0 ? m_firstVisibleChunkIndex : getGraph()->getChunkIndices()[m_firstGraphNodeIndex];

    if (isInvalidIndex(chunkIndex))
    {
        return 0;    // This actor has no chunks; only a graph node representing the world
    }

    NVBLAST_ASSERT(isInvalidIndex(header->getVisibleChunkIndexLinks()[chunkIndex].m_adj[1]));

    const NvBlastChunk& chunk = header->m_asset->getChunks()[chunkIndex];
    uint32_t childCount = chunk.childIndexStop - chunk.firstChildIndex;

    // Warn if we cannot fit all child chunks in the output list
    if (childCount > newActorsSize)
    {
        NVBLASTLL_LOG_WARNING(logFn, "Nv::Blast::Actor::partitionSingleLowerSupportChunk: input newActors array will not hold all actors generated.");
        childCount = newActorsSize;
    }

    // Return if no chunks will be created.
    if (childCount == 0)
    {
        return 0;
    }

    // Activate a new actor for every child chunk
    const Asset* asset = getAsset();
    const NvBlastChunk* chunks = asset->getChunks();
    const uint32_t firstChildIndex = chunks[chunkIndex].firstChildIndex;
    for (uint32_t i = 0; i < childCount; ++i)
    {
        const uint32_t childIndex = firstChildIndex + i;
        NVBLAST_ASSERT(childIndex >= asset->m_firstSubsupportChunkIndex);
        const uint32_t actorIndex = asset->m_graph.m_nodeCount + (childIndex - asset->m_firstSubsupportChunkIndex);
        NVBLAST_ASSERT(!header->isActorActive(actorIndex));
        newActors[i] = header->borrowActor(actorIndex);
        newActors[i]->m_firstVisibleChunkIndex = childIndex;
        newActors[i]->m_visibleChunkCount = 1;
        newActors[i]->m_leafChunkCount = asset->getSubtreeLeafChunkCounts()[childIndex];
    }

    // Release this actor
    release();

    return childCount;
}


void Actor::updateVisibleChunksFromGraphNodes()
{
    // Only apply this to upper-support chunk actors
    if (m_graphNodeCount == 0)
    {
        return;
    }

    const Asset* asset = getAsset();

    const uint32_t thisActorIndex = getIndex();

    // Get various arrays
    FamilyHeader* header = getFamilyHeader();
    Actor* actors = header->getActors();
    IndexDLink<uint32_t>* visibleChunkIndexLinks = header->getVisibleChunkIndexLinks();
    uint32_t* chunkActorIndices = header->getChunkActorIndices();
    const SupportGraph& graph = asset->m_graph;
    const uint32_t* graphChunkIndices = graph.getChunkIndices();
    const NvBlastChunk* chunks = asset->getChunks();
    const uint32_t upperSupportChunkCount = asset->getUpperSupportChunkCount();

    // Iterate over all graph nodes and update visible chunk list
    const uint32_t* graphNodeIndexLinks = header->getGraphNodeIndexLinks();
    for (uint32_t graphNodeIndex = m_firstGraphNodeIndex; !isInvalidIndex(graphNodeIndex); graphNodeIndex = graphNodeIndexLinks[graphNodeIndex])
    {
        const uint32_t supportChunkIndex = graphChunkIndices[graphNodeIndex];
        if (!isInvalidIndex(supportChunkIndex))    // Invalid if this is the world chunk
        {
            updateVisibleChunksFromSupportChunk<Actor>(actors, visibleChunkIndexLinks, chunkActorIndices, thisActorIndex, graphChunkIndices[graphNodeIndex], chunks, upperSupportChunkCount);
        }
    }
}

} // namespace Blast
} // namespace Nv


// API implementation

extern "C"
{

NvBlastActor* NvBlastFamilyCreateFirstActor(NvBlastFamily* family, const NvBlastActorDesc* desc, void* scratch, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(family != nullptr, logFn, "NvBlastFamilyCreateFirstActor: NULL family input.", return nullptr);
    NVBLASTLL_CHECK(desc != nullptr, logFn, "NvBlastFamilyCreateFirstActor: NULL desc input.", return nullptr);
    NVBLASTLL_CHECK(scratch != nullptr, logFn, "NvBlastFamilyCreateFirstActor: NULL scratch input.", return nullptr);

    return Nv::Blast::Actor::create(family, desc, scratch, logFn);
}


size_t NvBlastFamilyGetRequiredScratchForCreateFirstActor(const NvBlastFamily* family, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(family != nullptr, logFn, "NvBlastFamilyGetRequiredScratchForCreateFirstActor: NULL family input.", return 0);
    NVBLASTLL_CHECK(reinterpret_cast<const Nv::Blast::FamilyHeader*>(family)->m_asset != nullptr,
        logFn, "NvBlastFamilyGetRequiredScratchForCreateFirstActor: family has NULL asset.", return 0);

    return Nv::Blast::Actor::createRequiredScratch(family, logFn);
}


bool NvBlastActorDeactivate(NvBlastActor* actor, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorDeactivate: NULL actor input.", return false);

    Nv::Blast::Actor& a = *static_cast<Nv::Blast::Actor*>(actor);

    if (!a.isActive())
    {
        NVBLASTLL_LOG_WARNING(logFn, "NvBlastActorDeactivate: inactive actor input.");
    }

    return a.release();
}


uint32_t NvBlastActorGetVisibleChunkCount(const NvBlastActor* actor, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorGetVisibleChunkCount: NULL actor input.", return 0);

    const Nv::Blast::Actor& a = *static_cast<const Nv::Blast::Actor*>(actor);

    if (!a.isActive())
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastActorGetVisibleChunkCount: inactive actor input.");
        return 0;
    }

    return a.getVisibleChunkCount();
}


uint32_t NvBlastActorGetVisibleChunkIndices(uint32_t* visibleChunkIndices, uint32_t visibleChunkIndicesSize, const NvBlastActor* actor, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(visibleChunkIndices != nullptr, logFn, "NvBlastActorGetVisibleChunkIndices: NULL visibleChunkIndices pointer input.",    return 0);
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorGetVisibleChunkIndices: NULL actor pointer input.", return 0);

    const Nv::Blast::Actor& a = *static_cast<const Nv::Blast::Actor*>(actor);

    if (!a.isActive())
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastActorGetVisibleChunkIndices: inactive actor pointer input.");
        return 0;
    }

    // Iterate through visible chunk list and write to supplied array
    uint32_t indexCount = 0;
    for (Nv::Blast::Actor::VisibleChunkIt i = a; indexCount < visibleChunkIndicesSize && (bool)i; ++i)
    {
        visibleChunkIndices[indexCount++] = (uint32_t)i;
    }

    return indexCount;
}


uint32_t NvBlastActorGetGraphNodeCount(const NvBlastActor* actor, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorGetGraphNodeCount: NULL actor pointer input.", return 0);

    const Nv::Blast::Actor& a = *static_cast<const Nv::Blast::Actor*>(actor);

    if (!a.isActive())
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastActorGetGraphNodeCount: inactive actor pointer input.");
        return 0;
    }

    return a.getGraphNodeCount();
}


uint32_t NvBlastActorGetGraphNodeIndices(uint32_t* graphNodeIndices, uint32_t graphNodeIndicesSize, const NvBlastActor* actor, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(graphNodeIndices != nullptr, logFn, "NvBlastActorGetGraphNodeIndices: NULL graphNodeIndices pointer input.", return 0);
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorGetGraphNodeIndices: NULL actor pointer input.", return 0);

    const Nv::Blast::Actor& a = *static_cast<const Nv::Blast::Actor*>(actor);

    if (!a.isActive())
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastActorGetGraphNodeIndices: inactive actor pointer input.");
        return 0;
    }

    // Iterate through graph node list and write to supplied array
    const uint32_t* graphChunkIndices = a.getAsset()->m_graph.getChunkIndices();
    uint32_t indexCount = 0;
    for (Nv::Blast::Actor::GraphNodeIt i = a; indexCount < graphNodeIndicesSize && (bool)i; ++i)
    {
        const uint32_t graphNodeIndex = (uint32_t)i;
        if (!Nv::Blast::isInvalidIndex(graphChunkIndices[graphNodeIndex]))
        {
            graphNodeIndices[indexCount++] = graphNodeIndex;
        }
    }

    return indexCount;
}


const float* NvBlastActorGetBondHealths(const NvBlastActor* actor, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorGetBondHealths: NULL actor pointer input.", return nullptr);

    const Nv::Blast::Actor& a = *static_cast<const Nv::Blast::Actor*>(actor);

    if (!a.isActive())
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastActorGetBondHealths: inactive actor pointer input.");
        return nullptr;
    }

    return a.getFamilyHeader()->getBondHealths();
}


const float* NvBlastActorGetCachedBondHeaths(const NvBlastActor* actor, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorGetCachedBondHeaths: NULL actor pointer input.", return nullptr);

    const Nv::Blast::Actor& a = *static_cast<const Nv::Blast::Actor*>(actor);

    if (!a.isActive())
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastActorGetCachedBondHeaths: inactive actor pointer input.");
        return nullptr;
    }

    return a.getFamilyHeader()->getCachedBondHealths();
}


bool NvBlastActorCacheBondHeath(const NvBlastActor* actor, uint32_t bondIndex, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorCacheBondHeath: NULL actor pointer input.", return nullptr);

    const Nv::Blast::Actor& a = *static_cast<const Nv::Blast::Actor*>(actor);

    if (!a.isActive())
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastActorCacheBondHeath: inactive actor pointer input.");
        return false;
    }

    // copy the value over from the current bond health
    Nv::Blast::FamilyHeader* familyHeader = a.getFamilyHeader();
    const float curHealth = familyHeader->getBondHealths()[bondIndex];
    familyHeader->getCachedBondHealths()[bondIndex] = curHealth;

    return true;
}

NvBlastFamily* NvBlastActorGetFamily(const NvBlastActor* actor, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorGetFamily: NULL actor pointer input.", return nullptr);

    const Nv::Blast::Actor& a = *static_cast<const Nv::Blast::Actor*>(actor);

    if (!a.isActive())
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastActorGetFamily: inactive actor pointer input.");
        return nullptr;
    }

    return reinterpret_cast<NvBlastFamily*>(a.getFamilyHeader());
}


uint32_t NvBlastActorGetIndex(const NvBlastActor* actor, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorGetIndex: NULL actor pointer input.", return Nv::Blast::invalidIndex<uint32_t>());

    const Nv::Blast::Actor& a = *static_cast<const Nv::Blast::Actor*>(actor);

    if (!a.isActive())
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastActorGetIndex: actor is not active.");
        return Nv::Blast::invalidIndex<uint32_t>();
    }

    return a.getIndex();
}


void NvBlastActorGenerateFracture
(
    NvBlastFractureBuffers* commandBuffers,
    const NvBlastActor* actor,
    const NvBlastDamageProgram program,
    const void* programParams,
    NvBlastLog logFn,
    NvBlastTimers* timers
)
{
    NVBLASTLL_CHECK(commandBuffers != nullptr, logFn, "NvBlastActorGenerateFracture: NULL commandBuffers pointer input.", return);
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorGenerateFracture: NULL actor pointer input.", return);

    const Nv::Blast::Actor& a = *static_cast<const Nv::Blast::Actor*>(actor);

    if (!a.isActive())
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastActorGenerateFracture: actor is not active.");
        commandBuffers->bondFractureCount = 0;
        commandBuffers->chunkFractureCount = 0;
        return;
    }

    a.generateFracture(commandBuffers, program, programParams, logFn, timers);
}


void NvBlastActorApplyFracture
(
    NvBlastFractureBuffers* eventBuffers,
    NvBlastActor* actor,
    const NvBlastFractureBuffers* commands,
    NvBlastLog logFn,
    NvBlastTimers* timers
)
{
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorApplyFracture: NULL actor pointer input.", return);
    NVBLASTLL_CHECK(commands != nullptr, logFn, "NvBlastActorApplyFracture: NULL commands pointer input.", return);
    NVBLASTLL_CHECK(Nv::Blast::isValid(commands), logFn, "NvBlastActorApplyFracture: commands memory is NULL but size is > 0.", return);

    Nv::Blast::Actor& a = *static_cast<Nv::Blast::Actor*>(actor);

    if (!a.isActive())
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastActorApplyFracture: actor is not active.");
        if (eventBuffers != nullptr)
        {
            eventBuffers->bondFractureCount = 0;
            eventBuffers->chunkFractureCount = 0;
        }
        return;
    }

    a.getFamilyHeader()->applyFracture(eventBuffers, commands, &a, logFn, timers);
}


size_t NvBlastActorGetRequiredScratchForSplit(const NvBlastActor* actor, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorGetRequiredScratchForSplit: NULL actor input.", return 0);

    const Nv::Blast::Actor& a = *static_cast<const Nv::Blast::Actor*>(actor);

    if (!a.isActive())
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastActorGetRequiredScratchForSplit: actor is not active.");
        return 0;
    }

    return a.splitRequiredScratch();
}


uint32_t NvBlastActorGetMaxActorCountForSplit(const NvBlastActor* actor, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorGetMaxActorCountForSplit: NULL actor input.", return 0);

    const Nv::Blast::Actor& a = *static_cast<const Nv::Blast::Actor*>(actor);

    if (!a.isActive())
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastActorGetMaxActorCountForSplit: actor is not active.");
        return 0;
    }

    return a.getLeafChunkCount() + 1; // GWD-167 workaround (+1)
}


uint32_t NvBlastActorSplit
(
    NvBlastActorSplitEvent* result,
    NvBlastActor* actor,
    uint32_t newActorsMaxCount,
    void* scratch,
    NvBlastLog logFn,
    NvBlastTimers* timers
)
{
    NVBLASTLL_CHECK(result != nullptr, logFn, "NvBlastActorSplit: NULL result pointer input.", return 0);
    NVBLASTLL_CHECK(newActorsMaxCount > 0 && result->newActors != nullptr, logFn, "NvBlastActorSplit: no space for results provided.", return 0);
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorSplit: NULL actor pointer input.", return 0);
    NVBLASTLL_CHECK(scratch != nullptr, logFn, "NvBlastActorSplit: NULL scratch pointer input.", return 0);

    Nv::Blast::Actor& a = *static_cast<Nv::Blast::Actor*>(actor);

    if (!a.isActive())
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastActorGetIndex: actor is not active.");
        return 0;
    }

    return a.split(result, newActorsMaxCount, scratch, logFn, timers);
}


bool NvBlastActorCanFracture(const NvBlastActor* actor, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorCanFracture: NULL actor input.", return false);

    const Nv::Blast::Actor& a = *static_cast<const Nv::Blast::Actor*>(actor);

    if (!a.isActive())
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastActorCanFracture: actor is not active.");
        return false;
    }

    bool canFracture = true;

    uint32_t graphNodeCount = a.getGraphNodeCount();
    if (graphNodeCount < 2)
    {
        uint32_t chunkHealthIndex = graphNodeCount == 0 ?
            a.getFirstVisibleChunkIndex() - a.getFirstSubsupportChunkIndex() + a.getGraph()->m_nodeCount :
            a.getFirstGraphNodeIndex();
        canFracture = (a.getLowerSupportChunkHealths()[chunkHealthIndex] > 0.0f);
    }

    return canFracture;
}


bool NvBlastActorHasExternalBonds(const NvBlastActor* actor, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorHasExternalBonds: NULL actor input.", return false);

    return static_cast<const Nv::Blast::Actor*>(actor)->hasExternalBonds();
}


bool NvBlastActorIsSplitRequired(const NvBlastActor* actor, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(actor != nullptr, logFn, "NvBlastActorIsSplitRequired: NULL actor input.", return false);

    const Nv::Blast::Actor& a = *static_cast<const Nv::Blast::Actor*>(actor);
    if (!a.isActive())
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastActorIsSplitRequired: actor is not active.");
        return false;
    }
    return a.isSplitRequired();
}

} // extern "C"

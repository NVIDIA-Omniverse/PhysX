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


#ifndef NVBLASTCHUNKHIERARCHY_H
#define NVBLASTCHUNKHIERARCHY_H


#include "NvBlastIndexFns.h"
#include "NvBlastDLink.h"
#include "NvBlast.h"
#include "NvBlastAssert.h"
#include "NvBlastIteratorBase.h"


namespace Nv
{
namespace Blast
{

/**
Chunk hierarchy depth-first iterator.  Traverses subtree with root given by startChunkIndex.
Will not traverse chunks with index at or beyond chunkIndexLimit.
*/
class ChunkDepthFirstIt : public IteratorBase<uint32_t>
{
public:
    /** Constructed from a chunk array. */
    ChunkDepthFirstIt(const NvBlastChunk* chunks, uint32_t startChunkIndex, uint32_t chunkIndexLimit) :
        IteratorBase<uint32_t>(startChunkIndex), m_chunks(chunks), m_stop(startChunkIndex), m_limit(chunkIndexLimit)
    {
        if (m_curr >= m_limit)
        {
            m_curr = invalidIndex<uint32_t>();
        }
    }

    /** Pre-increment.  Only use if valid() == true. */
    uint32_t    operator ++ ()
    {
        NVBLAST_ASSERT(!isInvalidIndex(m_curr));
        const NvBlastChunk* chunk = m_chunks + m_curr;
        if (chunk->childIndexStop > chunk->firstChildIndex && chunk->firstChildIndex < m_limit)
        {
            m_curr = chunk->firstChildIndex;
        }
        else
        {
            for (;;)
            {
                if (m_curr == m_stop)
                {
                    m_curr = invalidIndex<uint32_t>();
                    break;
                }
                NVBLAST_ASSERT(!isInvalidIndex(chunk->parentChunkIndex));   // This should not be possible with this search
                const NvBlastChunk* parentChunk = m_chunks + chunk->parentChunkIndex;
                if (++m_curr < parentChunk->childIndexStop)
                {
                    break;  // Sibling chunk is valid, that's the next chunk
                }
                m_curr = chunk->parentChunkIndex;
                chunk = parentChunk;
            }
        }
        return m_curr;
    }

private:
    const NvBlastChunk* m_chunks;
    uint32_t            m_stop;
    uint32_t            m_limit;
};


/**
Enumerates chunk indices in a subtree with root given by chunkIndex, in breadth-first order.
Will not traverse chunks with index at or beyond chunkIndexLimit.
Returns the number of indices written to the chunkIndex array
*/
NV_INLINE uint32_t enumerateChunkHierarchyBreadthFirst
(
uint32_t* chunkIndices,
uint32_t chunkIndicesSize,
const NvBlastChunk* chunks,
uint32_t chunkIndex,
bool includeRoot = true,
uint32_t chunkIndexLimit = invalidIndex<uint32_t>()
)
{
    if (chunkIndicesSize == 0)
    {
        return 0;
    }
    uint32_t chunkIndexCount = 0;
    bool rootHandled = false;
    if (includeRoot)
    {
        chunkIndices[chunkIndexCount++] = chunkIndex;
        rootHandled = true;
    }
    for (uint32_t curr = 0; !rootHandled || curr < chunkIndexCount;)
    {
        const NvBlastChunk& chunk = chunks[rootHandled ? chunkIndices[curr] : chunkIndex];
        if (chunk.firstChildIndex < chunkIndexLimit)
        {
            const uint32_t childIndexStop = chunk.childIndexStop < chunkIndexLimit ? chunk.childIndexStop : chunkIndexLimit;
            const uint32_t childIndexBufferStop = chunk.firstChildIndex + (chunkIndicesSize - chunkIndexCount);
            const uint32_t stop = childIndexStop < childIndexBufferStop ? childIndexStop : childIndexBufferStop;
            for (uint32_t childIndex = chunk.firstChildIndex; childIndex < stop; ++childIndex)
            {
                chunkIndices[chunkIndexCount++] = childIndex;
            }
        }
        if (rootHandled)
        {
            ++curr;
        }
        rootHandled = true;
    }
    return chunkIndexCount;
}


/**
VisibilityRep must have m_firstVisibleChunkIndex and m_visibleChunkCount fields
*/
template<class VisibilityRep>
void updateVisibleChunksFromSupportChunk
(
VisibilityRep* actors,
IndexDLink<uint32_t>* visibleChunkIndexLinks,
uint32_t* chunkActorIndices,
uint32_t actorIndex,
uint32_t supportChunkIndex,
const NvBlastChunk* chunks,
uint32_t upperSupportChunkCount
)
{
    uint32_t chunkIndex = supportChunkIndex;
    uint32_t chunkActorIndex = chunkActorIndices[supportChunkIndex];
    uint32_t newChunkActorIndex = actorIndex;
    VisibilityRep& thisActor = actors[actorIndex];

    do
    {
        if (chunkActorIndex == newChunkActorIndex)
        {
            break;  // Nothing to do
        }

        const uint32_t parentChunkIndex = chunks[chunkIndex].parentChunkIndex;
        const uint32_t parentChunkActorIndex = parentChunkIndex != invalidIndex<uint32_t>() ? chunkActorIndices[parentChunkIndex] : invalidIndex<uint32_t>();
        const bool chunkVisible = chunkActorIndex != parentChunkActorIndex;

        // If the chunk is visible, it needs to be removed from its old actor's visibility list
        if (chunkVisible && !isInvalidIndex(chunkActorIndex))
        {
            VisibilityRep& chunkActor = actors[chunkActorIndex];
            IndexDList<uint32_t>().removeFromList(chunkActor.m_firstVisibleChunkIndex, visibleChunkIndexLinks, chunkIndex);
            --chunkActor.m_visibleChunkCount;
        }

        // Now update the chunk's actor index
        const uint32_t oldChunkActorIndex = chunkActorIndices[chunkIndex];
        chunkActorIndices[chunkIndex] = newChunkActorIndex;
        if (newChunkActorIndex != invalidIndex<uint32_t>() && parentChunkActorIndex != newChunkActorIndex)
        {
            // The chunk is now visible.  Add it to this actor's visibility list
            IndexDList<uint32_t>().insertListHead(thisActor.m_firstVisibleChunkIndex, visibleChunkIndexLinks, chunkIndex);
            ++thisActor.m_visibleChunkCount;
            // Remove its children from this actor's visibility list
            if (actorIndex != oldChunkActorIndex)
            {
                const NvBlastChunk& chunk = chunks[chunkIndex];
                if (chunk.firstChildIndex < upperSupportChunkCount) // Only need to deal with upper-support children
                {
                    for (uint32_t childChunkIndex = chunk.firstChildIndex; childChunkIndex < chunk.childIndexStop; ++childChunkIndex)
                    {
                        if (chunkActorIndices[childChunkIndex] == actorIndex)
                        {
                            IndexDList<uint32_t>().removeFromList(thisActor.m_firstVisibleChunkIndex, visibleChunkIndexLinks, childChunkIndex);
                            --thisActor.m_visibleChunkCount;
                        }
                    }
                }
            }
        }

        if (parentChunkIndex != invalidIndex<uint32_t>())
        {
            // If all of its siblings have the same index, then the parent will too.  Otherwise, the parent will have an invalid index and its children will be visible
            const NvBlastChunk& parentChunk = chunks[parentChunkIndex];
            bool uniform = true;
            for (uint32_t childChunkIndex = parentChunk.firstChildIndex; uniform && childChunkIndex < parentChunk.childIndexStop; ++childChunkIndex)
            {
                uniform = (newChunkActorIndex == chunkActorIndices[childChunkIndex]);
            }
            if (!uniform)
            {
                newChunkActorIndex = invalidIndex<uint32_t>();

                // no need to search if the parent index is invalid
                // the conditional in the loop could never be true in that case
                if (parentChunkActorIndex != invalidIndex<uint32_t>())
                {
                    for (uint32_t childChunkIndex = parentChunk.firstChildIndex; childChunkIndex < parentChunk.childIndexStop; ++childChunkIndex)
                    {
                        const uint32_t childChunkActorIndex = chunkActorIndices[childChunkIndex];
                        if (childChunkActorIndex != invalidIndex<uint32_t>() && childChunkActorIndex == parentChunkActorIndex)
                        {
                            // The child was invisible.  Add it to its actor's visibility list
                            VisibilityRep& childChunkActor = actors[childChunkActorIndex];
                            IndexDList<uint32_t>().insertListHead(childChunkActor.m_firstVisibleChunkIndex, visibleChunkIndexLinks, childChunkIndex);
                            ++childChunkActor.m_visibleChunkCount;
                        }
                    }
                }
            }
        }

        // Climb the hierarchy
        chunkIndex = parentChunkIndex;
        chunkActorIndex = parentChunkActorIndex;
    } while (chunkIndex != invalidIndex<uint32_t>());
}

} // namespace Blast
} // namespace Nv


#endif // ifndef NVBLASTCHUNKHIERARCHY_H

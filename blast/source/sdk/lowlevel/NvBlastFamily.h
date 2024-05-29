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


#ifndef NVBLASTFAMILY_H
#define NVBLASTFAMILY_H


#include "NvBlastAsset.h"
#include "NvPreprocessor.h"
#include "NvBlastDLink.h"
#include "NvBlastAtomic.h"
#include "NvBlastMemory.h"

#include <cstring>


struct NvBlastAsset;


namespace Nv
{
namespace Blast
{

// Forward declarations
class FamilyGraph;
class Actor;
class Asset;


/**
Data header at the beginning of every NvBlastActor family

The block address may be cast to a valid FamilyHeader pointer.
*/
struct FamilyHeader : public NvBlastDataBlock
{
    /**
    The ID for the asset.  This will be resolved into a pointer in the runtime data.
    */
    NvBlastID    m_assetID;

    /**
    Actors, of type Actor.

    Actors with support chunks will use this array in the range [0, m_asset->m_graphNodeCount),
    while subsupport actors will be placed in the range [m_asset->m_graphNodeCount, m_asset->getLowerSupportChunkCount()).
    */
    NvBlastBlockArrayData(Actor, m_actorsOffset, getActors, m_asset ? m_asset->getLowerSupportChunkCount() : 0);

    /**
    Visible chunk index links, of type IndexDLink<uint32_t>.

    getVisibleChunkIndexLinks returns an array of size m_asset->m_chunkCount of IndexDLink<uint32_t> (see IndexDLink).
    */
    NvBlastBlockArrayData(IndexDLink<uint32_t>, m_visibleChunkIndexLinksOffset, getVisibleChunkIndexLinks, m_asset ? m_asset->m_chunkCount : 0);

    /**
    Chunk actor IDs, of type uint32_t.  These correspond to the ID of the actor which owns each chunk.  A value of invalidIndex<uint32_t>() indicates no owner.

    getChunkActorIndices returns an array of size m_asset->m_firstSubsupportChunkIndex.
    */
    NvBlastBlockArrayData(uint32_t, m_chunkActorIndicesOffset, getChunkActorIndices, m_asset ? m_asset->m_firstSubsupportChunkIndex : 0);

    /**
    Graph node index links, of type uint32_t.  The successor to index[i] is m_graphNodeIndexLinksOffset[i].  A value of invalidIndex<uint32_t>() indicates no successor.

    getGraphNodeIndexLinks returns an array of size m_asset->m_graphNodeCount.
    */
    NvBlastBlockArrayData(uint32_t, m_graphNodeIndexLinksOffset, getGraphNodeIndexLinks, m_asset ? m_asset->m_graph.m_nodeCount : 0);

    /**
    Health for each support chunk and subsupport chunk, of type float.

    To access support chunks, use the corresponding graph node index in the array returned by getLowerSupportChunkHealths.

    To access subsupport chunk healths, use getSubsupportChunkHealths (see documentation for details).
    */
    NvBlastBlockArrayData(float, m_lowerSupportChunkHealthsOffset, getLowerSupportChunkHealths, m_asset ? m_asset->getLowerSupportChunkCount() : 0);

    /**
    Utility function to get the start of the subsupport chunk health array.

    To access a subsupport chunk health indexed by i, use getSubsupportChunkHealths()[i - m_asset->m_firstSubsupportChunkIndex]

    \return the array of health values associated with all descendants of support chunks.
    */
    float*    getSubsupportChunkHealths() const
    {
        NVBLAST_ASSERT(m_asset != nullptr);
        return (float*)((uintptr_t)this + m_lowerSupportChunkHealthsOffset) + (m_asset ? m_asset->m_graph.m_nodeCount : 0);
    }

    /**
    Bond health for the interfaces between two chunks, of type float.  Since the bond is shared by two chunks, the same bond health is used for chunk[i] -> chunk[j] as for chunk[j] -> chunk[i].

    getBondHealths returns the array of healths associated with all bonds in the support graph.
    */
    NvBlastBlockArrayData(float, m_graphBondHealthsOffset, getBondHealths, m_asset ? m_asset->getBondCount() : 0);

    /**
    Bond health for the interfaces between two chunks, of type float.  Since the bond is shared by two chunks, the same bond health is used for chunk[i] -> chunk[j] as for chunk[j] -> chunk[i].

    getCachedBondHealths returns the array of manually cached healths associated with all bonds in the support graph.
    */
    NvBlastBlockArrayData(float, m_graphCachedBondHealthsOffset, getCachedBondHealths, m_asset ? m_asset->getBondCount() : 0);

    /**
    The instance graph for islands searching, of type FamilyGraph.

    Return the dynamic data generated for the support graph.  (See FamilyGraph.)
    This is used to store current connectivity information based upon bond and chunk healths, as well as cached intermediate data for faster incremental updates.
    */
    NvBlastBlockData(FamilyGraph, m_familyGraphOffset, getFamilyGraph);


    //////// Runtime data ////////

    /**
    The number of actors using this block.
    */
    volatile uint32_t    m_actorCount;

    /**
    The asset corresponding to all actors in this family.
    This is runtime data and will be resolved from m_assetID.
    */
    union
    {
        const Asset*    m_asset;
        uint64_t        m_runtimePlaceholder;    // Make sure we reserve enough room for an 8-byte pointer
    };


    //////// Functions ////////

    /**
    Gets an actor from the actor array and validates it if it is not already valid.  This increments the actor reference count.

    \param[in] index    The index of the actor to borrow.  Must be in the range [0, getActorsArraySize()).

    \return    A pointer to the indexed Actor.
    */
    Actor*        borrowActor(uint32_t index);

    /**
    Invalidates the actor if it is not already invalid.  This decrements the actor reference count, but does not free this block when the count goes to zero.

    \param[in] actor    The actor to invalidate.
    */
    void        returnActor(Actor& actor);

    /**
    Returns a value to indicate whether or not the Actor with the given index is valid for use (active).

    \return    true iff the indexed actor is active.
    */
    bool        isActorActive(uint32_t index) const;

    /**
    Retrieve the actor from an index. If actor is inactive nullptr is returned.

    \param[in] index    The index of an actor.

    \return    A pointer to the indexed actor if the actor is active, nullptr otherwise.
    */
    Actor*        getActorByIndex(uint32_t index) const;

    /**
    Retrieve the index of an actor associated with the given chunk.

    \param[in] chunkIndex    The index of chunk.

    \return the index of associated actor in the FamilyHeader's getActors() array.
    */
    uint32_t    getChunkActorIndex(uint32_t chunkIndex) const;

    /**
    Retrieve the index of an actor associated with the given node.

    \param[in] nodeIndex    The index of node.

    \return the index of associated actor in the FamilyHeader's getActors() array.
    */
    uint32_t    getNodeActorIndex(uint32_t nodeIndex) const;

    /**
    Retrieve an actor associated with the given chunk.

    \param[in] chunkIndex    The index of chunk.

    \return    A pointer to the actor if the actor is active, nullptr otherwise.
    */
    Actor*        getChunkActor(uint32_t chunkIndex) const;

    /**
    Retrieve an actor associated with the given node.

    \param[in] nodeIndex    The index of node.

    \return    A pointer to the actor if the actor is active, nullptr otherwise.
    */
    Actor*        getNodeActor(uint32_t nodeIndex) const;


    //////// Fracturing methods ////////

    /**
    Hierarchically distribute damage to child chunks.

    \param chunkIndex        asset chunk index to hierarchically damage
    \param suboffset        index of the first sub-support health
    \param healthDamage        damage strength to apply
    \param chunkHealths        instance chunk healths
    \param chunks            asset chunk collection
    */
    void                fractureSubSupportNoEvents(uint32_t chunkIndex, uint32_t suboffset, float healthDamage, float* chunkHealths, const NvBlastChunk* chunks);

    /**
    Hierarchically distribute damage to child chunks, recording a fracture event for each health damage applied.

    If outBuffer is too small, events are dropped but the chunks are still damaged.

    \param chunkIndex        asset chunk index to hierarchically damage
    \param suboffset        index of the first sub-support health
    \param healthDamage        damage strength to apply
    \param chunkHealths        instance chunk healths
    \param chunks            asset chunk collection
    \param outBuffer        target buffer for fracture events
    \param currentIndex        current position in outBuffer - returns the number of damaged chunks
    \param maxCount            capacity of outBuffer
    \param[in] filterActor    pointer to the actor to filter commands that target other actors. May be NULL to apply all commands
    \param[in] logFn        User-supplied message function (see NvBlastLog definition).  May be NULL.
    */
    void                fractureSubSupport(uint32_t chunkIndex, uint32_t suboffset, float healthDamage, float* chunkHealths, const NvBlastChunk* chunks, NvBlastChunkFractureData* outBuffer, uint32_t* currentIndex, const uint32_t maxCount);

    /**
    Apply chunk fracture commands hierarchically.

    \param chunkFractureCount    number of chunk fracture commands to apply
    \param chunkFractures        array of chunk fracture commands
    \param filterActor            pointer to the actor to filter commands corresponding to other actors. May be NULL to apply all commands
    \param[in] logFn            User-supplied message function (see NvBlastLog definition).  May be NULL.
    */
    void                fractureNoEvents(uint32_t chunkFractureCount, const NvBlastChunkFractureData* chunkFractures, Actor* filterActor, NvBlastLog logFn);

    /**
    Apply chunk fracture commands hierarchically, recording a fracture event for each health damage applied.

    If events array is too small, events are dropped but the chunks are still damaged.

    \param chunkFractureCount    number of chunk fracture commands to apply
    \param commands                array of chunk fracture commands
    \param events                target buffer for fracture events
    \param eventsSize            number of available entries in 'events'
    \param count                returns the number of damaged chunks
    \param[in] filterActor        pointer to the actor to filter commands that target other actors. May be NULL to apply all commands
    \param[in] logFn            User-supplied message function (see NvBlastLog definition).  May be NULL.

    */
    void                fractureWithEvents(uint32_t chunkFractureCount, const NvBlastChunkFractureData* commands, NvBlastChunkFractureData* events, uint32_t eventsSize, uint32_t* count, Actor* filterActor, NvBlastLog logFn);

    /**
    Apply chunk fracture commands hierarchically, recording a fracture event for each health damage applied.

    In-Place version: fracture commands are replaced by fracture events.

    If inoutbuffer array is too small, events are dropped but the chunks are still damaged.

    \param chunkFractureCount    number of chunk fracture commands to apply
    \param inoutbuffer            array of chunk fracture commands to be replaced by events
    \param eventsSize            number of available entries in inoutbuffer
    \param count                returns the number of damaged chunks
    \param[in] filterActor        pointer to the actor to filter commands that target other actors. May be NULL to apply all commands
    \param[in] logFn            User-supplied message function (see NvBlastLog definition).  May be NULL.

    */
    void                fractureInPlaceEvents(uint32_t chunkFractureCount, NvBlastChunkFractureData* inoutbuffer, uint32_t eventsSize, uint32_t* count, Actor* filterActor, NvBlastLog logFn);

    /**
    See NvBlastActorApplyFracture

    \param[in,out]    eventBuffers        Target buffers to hold applied fracture events. May be NULL, in which case events are not reported.
    To avoid data loss, provide an entry for every lower-support chunk and every bond in the original actor.
    \param[in,out]    actor                The NvBlastActor to apply fracture to.
    \param[in]        commands            The fracture commands to process.
    \param[in]        filterActor            pointer to the actor to filter commands that target other actors. May be NULL to apply all commands
    \param[in]        logFn                User-supplied message function (see NvBlastLog definition).  May be NULL.
    \param[in,out]    timers                If non-NULL this struct will be filled out with profiling information for the step, in profile build configurations.
    */
    void                applyFracture(NvBlastFractureBuffers* eventBuffers, const NvBlastFractureBuffers* commands, Actor* filterActor, NvBlastLog logFn, NvBlastTimers* timers);
};

} // namespace Blast
} // namespace Nv


#include "NvBlastActor.h"


namespace Nv
{
namespace Blast
{

//////// FamilyHeader inline methods ////////

NV_INLINE Actor* FamilyHeader::borrowActor(uint32_t index)
{
    NVBLAST_ASSERT(index < getActorsArraySize());
    Actor& actor = getActors()[index];
    if (actor.m_familyOffset == 0)
    {
        const uintptr_t offset = (uintptr_t)&actor - (uintptr_t)this;
        NVBLAST_ASSERT(offset <= UINT32_MAX);
        actor.m_familyOffset = (uint32_t)offset;
        atomicIncrement(reinterpret_cast<volatile int32_t*>(&m_actorCount));
    }
    return &actor;
}


NV_INLINE void FamilyHeader::returnActor(Actor& actor)
{
    if (actor.m_familyOffset != 0)
    {
        actor.m_familyOffset = 0;
        // The actor count should be positive since this actor was valid.  Check to be safe.
        NVBLAST_ASSERT(m_actorCount > 0);
        atomicDecrement(reinterpret_cast<volatile int32_t*>(&m_actorCount));
    }
}


NV_INLINE bool FamilyHeader::isActorActive(uint32_t index) const
{
    NVBLAST_ASSERT(index < getActorsArraySize());
    return getActors()[index].m_familyOffset != 0;
}


NV_INLINE Actor* FamilyHeader::getActorByIndex(uint32_t index) const
{
    NVBLAST_ASSERT(index < getActorsArraySize());
    Actor& actor = getActors()[index];
    return actor.isActive() ? &actor : nullptr;
}


NV_INLINE uint32_t FamilyHeader::getChunkActorIndex(uint32_t chunkIndex) const
{
    if (m_asset == nullptr || chunkIndex >= m_asset->m_chunkCount)
    {
        NVBLAST_ASSERT(m_asset && chunkIndex < m_asset->m_chunkCount);
        return invalidIndex<uint32_t>();
    }

    if (chunkIndex < m_asset->getUpperSupportChunkCount())
    {
        return getChunkActorIndices()[chunkIndex];
    }
    else
    {
        return chunkIndex - (m_asset->getUpperSupportChunkCount() - m_asset->m_graph.m_nodeCount);
    }
}


NV_INLINE uint32_t FamilyHeader::getNodeActorIndex(uint32_t nodeIndex) const
{
    if (m_asset == nullptr || nodeIndex >= m_asset->m_graph.m_nodeCount)
    {
        NVBLAST_ASSERT(m_asset && nodeIndex < m_asset->m_graph.m_nodeCount);
        return invalidIndex<uint32_t>();
    }

    const uint32_t chunkIndex = m_asset->m_graph.getChunkIndices()[nodeIndex];
    return isInvalidIndex(chunkIndex) ? chunkIndex : getChunkActorIndices()[chunkIndex];
}


NV_INLINE Actor* FamilyHeader::getChunkActor(uint32_t chunkIndex) const
{
    uint32_t actorIndex = getChunkActorIndex(chunkIndex);
    return !isInvalidIndex(actorIndex) ? getActorByIndex(actorIndex) : nullptr;
}


NV_INLINE Actor* FamilyHeader::getNodeActor(uint32_t nodeIndex) const
{
    uint32_t actorIndex = getNodeActorIndex(nodeIndex);
    return !isInvalidIndex(actorIndex) ? getActorByIndex(actorIndex) : nullptr;
}


//////// Global functions ////////

/**
Returns the number of bytes of memory that a family created using the given asset will require.  A pointer
to a block of memory of at least this size must be passed in as the mem argument of createFamily.

\param[in] asset        The asset that will be passed into NvBlastAssetCreateFamily.
\param[in] sizeData     Alternate version where the counts are known but there is not an existing asset.
*/
size_t getFamilyMemorySize(const Asset* asset);
size_t getFamilyMemorySize(const NvBlastAssetMemSizeData& sizeData);

} // namespace Blast
} // namespace Nv


#endif // ifndef NVBLASTFAMILY_H

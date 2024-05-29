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


#ifndef NVBLASTACTOR_H
#define NVBLASTACTOR_H


#include "NvBlastAsset.h"
#include "NvBlastDLink.h"
#include "NvBlastIteratorBase.h"
#include "NvBlastSupportGraph.h"
#include "NvBlastFamilyGraph.h"
#include "NvBlastPreprocessorInternal.h"

#include <cstring>


namespace Nv
{
namespace Blast
{

// Forward declarations
class FamilyGraph;
struct FamilyHeader;

/**
Internal implementation of solver actor.

These objects are stored within the family in a single array.  A pointer to a Actor class will be given
to the user through the NvBlastActor opaque type.
*/
class Actor : public NvBlastActor
{
    friend struct FamilyHeader;

    friend void updateVisibleChunksFromSupportChunk<>(Actor*, IndexDLink<uint32_t>*, uint32_t*, uint32_t, uint32_t, const NvBlastChunk*, uint32_t);

public:
    Actor() : m_familyOffset(0), m_firstVisibleChunkIndex(UINT32_MAX), m_visibleChunkCount(0), m_firstGraphNodeIndex(UINT32_MAX), m_graphNodeCount(0), m_leafChunkCount(0) {}

    //////// Accessors ////////

    /**
    Find the family (see FamilyHeader) that this actor belongs to.

    \return    a pointer to the FamilyHeader for this actor.
    */
    FamilyHeader*        getFamilyHeader() const;

    /**
    Utility to get the asset this actor is associated with, through its family.

    \return    the asset associated with this actor.
    */
    const Asset*        getAsset() const;

    /**
    Since this object is not deleted (unless the family is deleted), we use m_familyOffset
    to determine if the actor is valid, or "active."  When no actors in an instance return isActive(),
    it should be safe to delete the family.

    \return true iff this actor is valid for use (active).
    */
    bool                isActive() const;

    /**
    Whether or not this actor represents a subsupport chunk.  If the actor contains a subsupport chunk, then it can have only that chunk.

    \return true iff this actor contains a chunk which is a descendant of a support chunk.
    */
    bool                isSubSupportChunk() const;

    /**
    Whether or not this actor represents a single support chunk.  If the actor contains a single support chunk, it can have no other
    chunks associated with it.

    \return true iff this actor contains exactly one support chunk.
    */
    bool                isSingleSupportChunk() const;

    /**
    Utility to calculate actor index.

    \return the index of this actor in the FamilyHeader's getActors() array.
    */
    uint32_t            getIndex() const;

    /**
    Offset to block of memory which holds the data associated with all actors in this actor's lineage

    \return the family offset.
    */
    uint32_t            getFamilyOffset() const;
    void                setFamilyOffset(uint32_t familyOffset);

    /**
    The number of visible chunks.  This is calculated from updateVisibleChunksFromGraphNodes().
    See also getFirstVisibleChunkIndex.

    \return the number of chunks in the actor's visible chunk index list.
    */
    uint32_t            getVisibleChunkCount() const;
    void                setVisibleChunkCount(uint32_t visibleChunkCount);

    /**
    Access to visible chunk linked list for this actor.  The index returned is that of a link in the FamilyHeader's getVisibleChunkIndexLinks().

    \return the index of the head of the visible chunk linked list.
    */
    uint32_t            getFirstVisibleChunkIndex() const;
    void                setFirstVisibleChunkIndex(uint32_t firstVisibleChunkIndex);

    /**
    The number of graph nodes, corresponding to support chunks, for this actor.
    See also getFirstGraphNodeIndex.

    \return the number of graph nodes in the actor's graph node index list.
    */
    uint32_t            getGraphNodeCount() const;
    void                setGraphNodeCount(uint32_t graphNodeCount);

    /**
    The number of leaf chunks for this actor.

    \return number of leaf chunks for this actor.
    */
    uint32_t            getLeafChunkCount() const;
    void                setLeafChunkCount(uint32_t leafChunkCount);

    /**
    Access to graph node linked list for this actor.  The index returned is that of a link in the FamilyHeader's getGraphNodeIndexLinks().

    \return the index of the head of the graph node linked list.
    */
    uint32_t            getFirstGraphNodeIndex() const;
    void                setFirstGraphNodeIndex(uint32_t firstGraphNodeIndex);

    /**
    Access to the index of the first subsupport chunk.

    \return the index of the first subsupport chunk.
    */
    uint32_t            getFirstSubsupportChunkIndex() const;

    /**
    Access to the support graph.

    \return the support graph associated with this actor.
    */
    const SupportGraph*    getGraph() const;

    /**
    Access the instance graph for islands searching.

    Return the dynamic data generated for the support graph.  (See FamilyGraph.)
    This is used to store current connectivity information based upon bond and chunk healths, as well as cached intermediate data for faster incremental updates.
    */
    FamilyGraph*        getFamilyGraph() const;

    /**
    Access to the chunks, of type NvBlastChunk.

    \return an array of size m_chunkCount.
    */
    NvBlastChunk*        getChunks() const;

    /**
    Access to the bonds, of type NvBlastBond.

    \return an array of size m_bondCount.
    */
    NvBlastBond*        getBonds() const;

    /**
    Access to the health for each support chunk and subsupport chunk, of type float.

    Use getAsset()->getContiguousLowerSupportIndex() to map lower-support chunk indices into the range of indices valid for this array.

    \return a float array of chunk healths.
    */
    float*                getLowerSupportChunkHealths() const;

    /**
    Access to the start of the subsupport chunk health array.

    \return the array of health values associated with all descendants of support chunks.
    */
    float*                getSubsupportChunkHealths() const;

    /**
    Bond health for the interfaces between two chunks, of type float.  Since the bond is shared by two chunks, the same bond health is used for chunk[i] -> chunk[j] as for chunk[j] -> chunk[i].

    \return the array of healths associated with all bonds in the support graph.
    */
    float*                getBondHealths() const;

    /**
    Graph node index links, of type uint32_t.  The successor to index[i] is m_graphNodeIndexLinksOffset[i].  A value of invalidIndex<uint32_t>() indicates no successor.

    getGraphNodeIndexLinks returns an array of size m_asset->m_graphNodeCount.
    */
    const uint32_t*        getGraphNodeIndexLinks() const;


    //////// Iterators ////////

    /**
    Visible chunk iterator.  Usage:

    Given a solver actor a,

    for (Actor::VisibleChunkIt i = a; (bool)i; ++i)
    {
        uint32_t visibleChunkIndex = (uint32_t)i;

        // visibleChunkIndex references the asset index list
    }

    */
    class VisibleChunkIt : public DListIt<uint32_t>
    {
    public:
        /** Constructed from an actor. */
        VisibleChunkIt(const Actor& actor);
    };

    /**
    Graph node iterator.  Usage:

    Given a solver actor a,

    for (Actor::GraphNodeIt i = a; (bool)i; ++i)
    {
    uint32_t graphNodeIndex = (uint32_t)i;

    // graphNodeIndex references the asset's graph node index list
    }

    */
    class GraphNodeIt : public LListIt<uint32_t>
    {
    public:
        /** Constructed from an actor. */
        GraphNodeIt(const Actor& actor);
    };


    //////// Operations ////////

    /**
    Create an actor from a descriptor (creates a family).  This actor will represent an unfractured instance of the asset.
    The asset must be in a valid state, for example each chunk hierarchy in it must contain at least one support chunk (a single
    support chunk in a hierarchy corresponds to the root chunk).  This will always be the case for assets created by NvBlastCreateAsset.

    \param[in] family    Family in which to create a new actor.  The family must be valid and have no other actors in it.  (See createFamily.)
    \param[in] desc        Actor initialization data, must be a valid pointer.
    \param[in] scratch    User-supplied scratch memory of size createRequiredScratch(desc) bytes.
    \param[in] logFn    User-supplied message function (see NvBlastLog definition).  May be NULL.

    \return the new actor if the input is valid (by the conditions described above), NULL otherwise.
    */
    static Actor*        create(NvBlastFamily* family, const NvBlastActorDesc* desc, void* scratch, NvBlastLog logFn);

    /**
    Returns the size of the scratch space (in bytes) required to be passed into the create function, based upon
    the family that will be passed to the create function.

    \param[in] family    The family being instanced.

    \return the number of bytes required.
    */
    static size_t        createRequiredScratch(const NvBlastFamily* family, NvBlastLog logFn);

    /**
    Deserialize a single Actor from a buffer.  An actor family must given, into which
    the actor will be inserted if it is compatible.  That is, it must not share any chunks or internal
    IDs with the actors already present in the block.

    \param[in] family    Family in which to deserialize the actor.
    \param[in] buffer    Buffer containing the serialized actor data.
    \param[in] logFn    User-supplied message function (see NvBlastLog definition).  May be NULL.

    \return the deserialized actor if successful, NULL otherwise.
    */
    static Actor*        deserialize(NvBlastFamily* family, const void* buffer, NvBlastLog logFn);

    /**
    Serialize actor into single-actor buffer.

    \param[out] buffer        User-supplied buffer, must be at least of size given by NvBlastActorGetSerializationSize(actor).
    \param[in] bufferSize    The size of the user-supplied buffer.  The buffer size must be less than 4GB.  If NvBlastActorGetSerializationSize(actor) >= 4GB, this actor cannot be serialized with this method.
    \param[in] logFn        User-supplied message function (see NvBlastLog definition).  May be NULL.

    \return the number of bytes written to the buffer, or 0 if there is an error (such as an under-sized buffer).
    */
    uint32_t            serialize(void* buffer, uint32_t bufferSize, NvBlastLog logFn) const;

    /**
    Calculate the space required to serialize this actor.

    \param[in] logFn    User-supplied message function (see NvBlastLog definition).  May be NULL.

    \return the required buffer size in bytes.
    */
    uint32_t            serializationRequiredStorage(NvBlastLog logFn) const;

    /**
    Release this actor's association with a family, if any.  This actor should be considered deleted
    after this function is called.

    \return true if release was successful (actor was active).
    */
    bool                release();


    //////// Damage and fracturing methods ////////

    /**
    See NvBlastActorGenerateFracture
    */
    void                generateFracture(NvBlastFractureBuffers* commandBuffers, const NvBlastDamageProgram& program, const void* programParams, NvBlastLog logFn, NvBlastTimers* timers) const;

    /**
    Damage bond between two chunks by health amount (instance graph also will be notified in case bond is broken after).
    */
    uint32_t            damageBond(uint32_t nodeIndex0, uint32_t nodeIndex1, float healthDamage);

    /**
    TODO: document
    */
    void                damageBond(uint32_t nodeIndex0, uint32_t nodeIndex1, uint32_t bondIndex, float healthDamage);

    /**
    TODO: document
    */
    uint32_t            damageBond(const NvBlastBondFractureData& cmd);

    /**
    See NvBlastActorApplyFracture
    */
    void                applyFracture(NvBlastFractureBuffers* eventBuffers, const NvBlastFractureBuffers* commands, NvBlastLog logFn, NvBlastTimers* timers);

    /**
    The scratch space required to call the findIslands function, or the split function, in bytes.
    
    \return the number of bytes required.
    */
    size_t                splitRequiredScratch() const;

    /**
    See NvBlastActorSplit
    */
    uint32_t            split(NvBlastActorSplitEvent* result, uint32_t newActorsMaxCount, void* scratch, NvBlastLog logFn, NvBlastTimers* timers);

    /**
    Perform islands search.  Bonds which are broken when their health values drop to zero (or below) may lead
    to new islands of chunks which need to be split into new actors.  This function labels all nodes in the instance
    graph (see FamilyGraph) with a unique index per island that may be used as actor indices for new islands.

    \param[in] scratch    User-supplied scratch memory of size splitRequiredScratch().

    \return    the number of new islands found.
    */
    uint32_t            findIslands(void* scratch);

    /**
    Partition this actor into smaller pieces.

    If this actor represents a single support or subsupport chunk, then after this operation
    this actor will released if child chunks are created (see Return value), and its pointer no longer valid for use (unless it appears in the newActors list).

    This function will not split a leaf chunk actor.  In that case, the actor is not destroyed and this function returns 0.

    \param[in] newActors        user-supplied array of actor pointers to hold the actors generated from this partitioning.
                                This array must be of size equal to the number of leaf chunks in the asset, to guarantee
                                that all actors are reported.  (See AssetDataHeader::m_leafChunkCount.)
    \param[in] newActorsSize    The size of the user-supplied newActors array.
    \param[in] logFn            User-supplied message function (see NvBlastLog definition).  May be NULL.

    \return    the number of new actors created.  If greater than newActorsSize, some actors are not reported in the newActors array.
    */
    uint32_t            partition(Actor** newActors, uint32_t newActorsSize, NvBlastLog logFn);

    /**
    Recalculate the visible chunk list for this actor based upon it graph node list (does not modify subsupport chunk actors)
    */
    void                updateVisibleChunksFromGraphNodes();

    /**
    Partition this actor into smaller pieces if it is a single lower-support chunk actor.  Use this function on single support or sub-support chunks.

    After this operation, if successful (child chunks created, see Return value), this actor will released, and its pointer no longer valid for use.

    This function will not split a leaf chunk actor.  In that case, the actor is not destroyed and this function returns 0.

    \param[in] newActors        User-supplied array of actor pointers to hold the actors generated from this partitioning.  Note: this actor will be released.
                                This array must be of size equal to the lower-support chunk's child count, to guarantee that all actors are reported.
    \param[in] newActorsSize    The size of the user-supplied newActors array.
    \param[in] logFn            User-supplied message function (see NvBlastLog definition).  May be NULL.

    \return the number of new actors created.
    */
    uint32_t            partitionSingleLowerSupportChunk(Actor** newActors, uint32_t newActorsSize, NvBlastLog logFn);

    /**
    Partition this actor into smaller pieces.  Use this function if this actor contains more than one support chunk. 

    After this operation, if successful, this actor will released, and its pointer no longer valid for use (unless it appears in the newActors list).

    \param[in] newActors        User-supplied array of actor pointers to hold the actors generated from this partitioning.  Note: this actor will not be released,
                                but will hold a subset of the graph nodes that it had before the function was called.
                                This array must be of size equal to the number of graph nodes in the asset, to guarantee
                                that all actors are reported.
    \param[in] newActorsSize    The size of the user-supplied newActors array.
    \param[in] logFn            User-supplied message function (see NvBlastLog definition).  May be NULL.

    \return the number of new actors created.
    */
    uint32_t            partitionMultipleGraphNodes(Actor** newActors, uint32_t newActorsSize, NvBlastLog logFn);

    /**
    \return true iff this actor contains the "external" support graph node, created when a bond contains the invalidIndex<uint32_t>() value for one of their chunkIndices.
    */
    bool                hasExternalBonds() const;

    /**
    \return true iff this actor was damaged and split() call is required.
    */
    bool                isSplitRequired() const;

private:

    //////// Data ////////

    /**
    Offset to block of memory which holds the data associated with all actors in this actor's lineage.
    This offset is positive.  The block address is this object's pointer _minus_ the m_familyOffset.
    This value is initialized to 0, which denotes an invalid actor.  Actors should be obtained through
    the FamilyHeader::borrowActor API, which will create a valid offset, and
    the FamilyHeader::returnActor API, which will zero the offset.
    */
    uint32_t    m_familyOffset;

    /**
    The index of the head of a doubly-linked list of visible chunk indices.  If m_firstVisibleChunkIndex == invalidIndex<uint32_t>(),
    then there are no visible chunks.
    */
    uint32_t    m_firstVisibleChunkIndex;

    /**
    The number of elements in the visible chunk list.
    */
    uint32_t    m_visibleChunkCount;

    /**
    The index of the head of a singly-linked list of graph node indices.  If m_firstGraphNodeIndex == invalidIndex<uint32_t>(),
    then there are no graph nodes.
    */
    uint32_t    m_firstGraphNodeIndex;

    /**
    The number of elements in the graph node list.
    */
    uint32_t    m_graphNodeCount;

    /**
    The number of leaf chunks in this actor.
    */
    uint32_t    m_leafChunkCount;
};

} // namespace Blast
} // namespace Nv


#include "NvBlastFamily.h"


namespace Nv
{
namespace Blast
{

//////// Actor inline methods ////////

NV_INLINE FamilyHeader* Actor::getFamilyHeader() const
{
    NVBLAST_ASSERT(isActive());
    return isActive() ? (FamilyHeader*)((uintptr_t)this - (uintptr_t)m_familyOffset) : nullptr;
}


NV_INLINE const Asset* Actor::getAsset() const
{
    return getFamilyHeader()->m_asset;
}


NV_INLINE bool Actor::isActive() const
{
    return m_familyOffset != 0;
}


NV_INLINE bool Actor::isSubSupportChunk() const
{
    return m_graphNodeCount == 0;
}


NV_INLINE bool Actor::isSingleSupportChunk() const
{
    return m_graphNodeCount == 1;
}


NV_INLINE uint32_t Actor::getIndex() const
{
    NVBLAST_ASSERT(isActive());
    const FamilyHeader* header = getFamilyHeader();
    NVBLAST_ASSERT(header != nullptr);
    const size_t index = this - header->getActors();
    NVBLAST_ASSERT(index <= UINT32_MAX);
    return (uint32_t)index;
}


NV_INLINE uint32_t Actor::getFamilyOffset() const
{
    return m_familyOffset;
}
NV_INLINE void Actor::setFamilyOffset(uint32_t familyOffset)
{
    m_familyOffset = familyOffset;
}


NV_INLINE uint32_t Actor::getVisibleChunkCount() const
{
    return m_visibleChunkCount;
}
NV_INLINE void Actor::setVisibleChunkCount(uint32_t visibleChunkCount)
{
    m_visibleChunkCount = visibleChunkCount;
}


NV_INLINE uint32_t Actor::getFirstVisibleChunkIndex() const
{
    return m_firstVisibleChunkIndex;
}
NV_INLINE void Actor::setFirstVisibleChunkIndex(uint32_t firstVisibleChunkIndex)
{
    m_firstVisibleChunkIndex = firstVisibleChunkIndex;
}


NV_INLINE uint32_t Actor::getGraphNodeCount() const
{
    return m_graphNodeCount;
}
NV_INLINE void Actor::setGraphNodeCount(uint32_t graphNodeCount)
{
    m_graphNodeCount = graphNodeCount;
}


NV_INLINE uint32_t Actor::getLeafChunkCount() const
{
    return m_leafChunkCount;
}
NV_INLINE void Actor::setLeafChunkCount(uint32_t leafChunkCount)
{
    m_leafChunkCount = leafChunkCount;
}


NV_INLINE uint32_t Actor::getFirstGraphNodeIndex() const
{
    return m_firstGraphNodeIndex;
}
NV_INLINE void Actor::setFirstGraphNodeIndex(uint32_t firstGraphNodeIndex)
{
    m_firstGraphNodeIndex = firstGraphNodeIndex;
}

NV_INLINE uint32_t Actor::getFirstSubsupportChunkIndex() const
{
    return getAsset()->m_firstSubsupportChunkIndex;
}

NV_INLINE const SupportGraph* Actor::getGraph() const
{
    return &getAsset()->m_graph;
}

NV_INLINE FamilyGraph* Actor::getFamilyGraph() const
{
    return getFamilyHeader()->getFamilyGraph();
}

NV_INLINE NvBlastChunk* Actor::getChunks() const
{
    return getAsset()->getChunks();
}

NV_INLINE NvBlastBond* Actor::getBonds() const
{
    return getAsset()->getBonds();
}

NV_INLINE float* Actor::getLowerSupportChunkHealths() const
{
    return getFamilyHeader()->getLowerSupportChunkHealths();
}

NV_INLINE float*    Actor::getSubsupportChunkHealths() const
{
    return getFamilyHeader()->getSubsupportChunkHealths();
}

NV_INLINE float* Actor::getBondHealths() const
{
    return getFamilyHeader()->getBondHealths();
}

NV_INLINE const uint32_t* Actor::getGraphNodeIndexLinks() const
{
    return getFamilyHeader()->getGraphNodeIndexLinks();
}


NV_INLINE bool Actor::release()
{
    // Do nothing if this actor is not currently active.
    if (!isActive())
    {
        return false;
    }

    FamilyHeader* header = getFamilyHeader();

    // Clear the graph node list
    uint32_t* graphNodeIndexLinks = getFamilyHeader()->getGraphNodeIndexLinks();
    while (!isInvalidIndex(m_firstGraphNodeIndex))
    {
        const uint32_t graphNodeIndex = m_firstGraphNodeIndex;
        m_firstGraphNodeIndex = graphNodeIndexLinks[m_firstGraphNodeIndex];
        graphNodeIndexLinks[graphNodeIndex] = invalidIndex<uint32_t>();
        --m_graphNodeCount;
    }
    NVBLAST_ASSERT(m_graphNodeCount == 0);

    const Asset* asset = getAsset();

    // Clear the visible chunk list
    IndexDLink<uint32_t>* visibleChunkIndexLinks = header->getVisibleChunkIndexLinks();
    uint32_t* chunkActorIndices = header->getChunkActorIndices();
    while (!isInvalidIndex(m_firstVisibleChunkIndex))
    {
        // Descendants of the visible actor may be accessed again if the actor is deserialized.  Clear subtree.
        for (Asset::DepthFirstIt i(*asset, m_firstVisibleChunkIndex, true); (bool)i; ++i)
        {
            chunkActorIndices[(uint32_t)i] = invalidIndex<uint32_t>();
        }
        IndexDList<uint32_t>().removeListHead(m_firstVisibleChunkIndex, visibleChunkIndexLinks);
        --m_visibleChunkCount;
    }
    NVBLAST_ASSERT(m_visibleChunkCount == 0);

    // Clear the leaf chunk count
    m_leafChunkCount = 0;

    // This invalidates the actor and decrements the reference count
    header->returnActor(*this);

    return true;
}


NV_INLINE uint32_t Actor::partition(Actor** newActors, uint32_t newActorsSize, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(newActorsSize == 0 || newActors != nullptr, logFn, "Nv::Blast::Actor::partition: NULL newActors pointer array input with non-zero newActorCount.", return 0);

    // Call one of two partition functions depending on the actor's support status
    return m_graphNodeCount <= 1 ?
        partitionSingleLowerSupportChunk(newActors, newActorsSize, logFn) :    // This actor will partition into subsupport chunks
        partitionMultipleGraphNodes(newActors, newActorsSize, logFn);        // This actor will partition into support chunks
}


NV_INLINE bool Actor::hasExternalBonds() const
{
    const SupportGraph& graph = *getGraph();

    if (graph.m_nodeCount == 0)
    {
        return false;    // This shouldn't happen
    }

    const uint32_t lastGraphChunkIndex = graph.getChunkIndices()[graph.m_nodeCount - 1];

    if (!isInvalidIndex(lastGraphChunkIndex))
    {
        return false;    // There is no external node
    }

    return getFamilyGraph()->getIslandIds()[graph.m_nodeCount - 1] == getIndex();
}


NV_INLINE bool Actor::isSplitRequired() const
{
    NVBLAST_ASSERT(isActive());

    if (getGraphNodeCount() <= 1)
    {
        uint32_t chunkHealthIndex = isSingleSupportChunk() ? getIndex() : getFirstVisibleChunkIndex() - getFirstSubsupportChunkIndex() + getGraph()->m_nodeCount;
        float* chunkHealths = getLowerSupportChunkHealths();
        if (chunkHealths[chunkHealthIndex] <= 0.0f)
        {
            const uint32_t chunkIndex = m_graphNodeCount == 0 ? m_firstVisibleChunkIndex : getGraph()->getChunkIndices()[m_firstGraphNodeIndex];
            if (!isInvalidIndex(chunkIndex))
            {
                const NvBlastChunk& chunk = getChunks()[chunkIndex];
                uint32_t childCount = chunk.childIndexStop - chunk.firstChildIndex;
                return childCount > 0;
            }
        }
    }
    else
    {
        uint32_t* firstDirtyNodeIndices = getFamilyGraph()->getFirstDirtyNodeIndices();
        if (!isInvalidIndex(firstDirtyNodeIndices[getIndex()]))
        {
            return true;
        }

    }
    return false;
}


//////// Actor::VisibleChunkIt inline methods ////////

NV_INLINE Actor::VisibleChunkIt::VisibleChunkIt(const Actor& actor) : DListIt<uint32_t>(actor.m_firstVisibleChunkIndex, actor.getFamilyHeader()->getVisibleChunkIndexLinks())
{
}


//////// Actor::GraphNodeIt inline methods ////////

NV_INLINE Actor::GraphNodeIt::GraphNodeIt(const Actor& actor) : LListIt<uint32_t>(actor.m_firstGraphNodeIndex, actor.getFamilyHeader()->getGraphNodeIndexLinks())
{
}


//////// Helper functions ////////

#if NVBLASTLL_CHECK_PARAMS
/**
Helper function to validate fracture buffer values being meaningful.
*/
static inline bool isValid(const NvBlastFractureBuffers* buffers)
{
    if (buffers->chunkFractureCount != 0 && buffers->chunkFractures == nullptr)
        return false;

    if (buffers->bondFractureCount != 0 && buffers->bondFractures == nullptr)
        return false;

    return true;
}
#endif


} // namespace Blast
} // namespace Nv


#endif // ifndef NVBLASTACTOR_H

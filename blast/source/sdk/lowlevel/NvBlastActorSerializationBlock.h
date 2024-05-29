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


#ifndef NVBLASTACTORSERIALIZATIONBLOCK_H
#define NVBLASTACTORSERIALIZATIONBLOCK_H


#include "NvBlastFixedBoolArray.h"


namespace Nv
{
namespace Blast
{

/**
Struct-enum which keeps track of the actor serialization format.
*/
struct ActorSerializationFormat
{
    enum Version
    {
        /** Initial version */
        Initial,

        //  New formats must come before Count.  They should be given descriptive names with more information in comments.

        /** The number of serialization formats. */
        Count,

        /** The current version.  This should always be Count-1 */
        Current = Count - 1
    };
};


/**
Data header at the beginning of a NvBlastActor serialization block

The block address may be cast to a valid ActorSerializationHeader pointer.

Serialization state is only valid if partition has been called since the last call to findIslands().
*/
struct ActorSerializationHeader
{
    /**
    A number which is incremented every time the data layout changes.
    */
    uint32_t    m_formatVersion;

    /**
    The size of the serialization block, including this header.

    Memory sizes are restricted to 32-bit representable values.
    */
    uint32_t    m_size;

    /**
    The index of the actor within its family.
    */
    uint32_t    m_index;

    /**
    The number of elements in the visible chunk indices list.
    */
    uint32_t    m_visibleChunkCount;

    /**
    The number of elements in the graph node indices list.
    */
    uint32_t    m_graphNodeCount;

    /**
    The number of leaf chunks in this actor.
    */
    uint32_t    m_leafChunkCount;

    /**
    Visible chunk indices, of type uint32_t.
    */
    NvBlastBlockArrayData(uint32_t, m_visibleChunkIndicesOffset, getVisibleChunkIndices, m_visibleChunkCount);

    /**
    Graph node indices, of type uint32_t.
    */
    NvBlastBlockArrayData(uint32_t, m_graphNodeIndicesOffset, getGraphNodeIndices, m_graphNodeCount);

    /**
    Healths for lower support chunks in this actor, in breadth-first order from the support chunks associated with the graph nodes.  Type float.
    */
    NvBlastBlockData(float, m_lowerSupportChunkHealthsOffset, getLowerSupportChunkHealths);

    /**
    Healths for bonds associated with support chunks in this actor, in order of graph adjacency from associated graph nodes, i < j only.  Type float.
    */
    NvBlastBlockData(float, m_bondHealthsOffset, getBondHealths);

    /**
    Fast route in instance graph calculated for each graph node in this actor, of type uint32_t.
    */
    NvBlastBlockArrayData(uint32_t, m_fastRouteOffset, getFastRoute, m_graphNodeCount);

    /**
    Hop counts in instance graph calculated for each graph node in this actor, of type uint32_t.
    */
    NvBlastBlockArrayData(uint32_t, m_hopCountsOffset, getHopCounts, m_graphNodeCount);

    /**
    "Edge removed" bits for bonds associated with support chunks in this actor, in order of graph adjacency from associated graph nodes, i < j only.  Type FixedBoolArray.
    */
    NvBlastBlockData(FixedBoolArray, m_edgeRemovedArrayOffset, getEdgeRemovedArray);
};


//////// Global functions ////////

/**
A buffer size sufficient to serialize an actor with a given visible chunk count, lower support chunk count, graph node count, and bond count.

\param[in] visibleChunkCount        The number of visible chunks
\param[in] lowerSupportChunkCount   The number of lower-support chunks in the asset.
\param[in] graphNodeCount           The number of graph nodes in the asset.
\param[in] bondCount                The number of graph bonds in the asset.

\return the required buffer size in bytes.
*/
NV_INLINE size_t getActorSerializationSize(uint32_t visibleChunkCount, uint32_t lowerSupportChunkCount, uint32_t graphNodeCount, uint32_t bondCount)
{
    // Family offsets
    const size_t visibleChunkIndicesOffset = align16(sizeof(ActorSerializationHeader));                             // size = visibleChunkCount*sizeof(uint32_t)
    const size_t graphNodeIndicesOffset = align16(visibleChunkIndicesOffset + visibleChunkCount*sizeof(uint32_t));  // size = graphNodeCount*sizeof(uint32_t)
    const size_t lowerSupportHealthsOffset = align16(graphNodeIndicesOffset + graphNodeCount*sizeof(uint32_t));     // size = lowerSupportChunkCount*sizeof(float)
    const size_t bondHealthsOffset = align16(lowerSupportHealthsOffset + lowerSupportChunkCount*sizeof(float));     // size = bondCount*sizeof(float)
    const size_t fastRouteOffset = align16(bondHealthsOffset + bondCount*sizeof(float));                            // size = graphNodeCount*sizeof(uint32_t)
    const size_t hopCountsOffset = align16(fastRouteOffset + graphNodeCount*sizeof(uint32_t));                      // size = graphNodeCount*sizeof(uint32_t)
    const size_t edgeRemovedArrayOffset = align16(hopCountsOffset + graphNodeCount*sizeof(uint32_t));               // size = 0 or FixedBoolArray::requiredMemorySize(bondCount)
    return align16(edgeRemovedArrayOffset + (bondCount == 0 ? 0 : FixedBoolArray::requiredMemorySize(bondCount)));
}

} // namespace Blast
} // namespace Nv


#endif // ifndef NVBLASTACTORSERIALIZATIONBLOCK_H

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


#ifndef NVBLASTSUPPORTGRAPH_H
#define NVBLASTSUPPORTGRAPH_H


#include "NvBlastIndexFns.h"
#include "NvBlastMemory.h"

namespace Nv
{
namespace Blast
{

/**
Describes the connectivity between support chunks via bonds.

Vertices in the support graph are termed "nodes," and represent particular chunks (NvBlastChunk) in an NvBlastAsset.
The indexing for nodes is not the same as that for chunks.  Only some chunks are represented by nodes in the graph,
and these chunks are called "support chunks."

Adjacent node indices and adjacent bond indices are stored for each node, and therefore each bond is represented twice in this graph,
going from node[i] -> node[j] and from node[j] -> node[i].  Therefore the size of the getAdjacentNodeIndices() and getAdjacentBondIndices()
arrays are twice the number of bonds stored in the corresponding NvBlastAsset.

The graph is used as follows.  Given a SupportGraph "graph" and node index i, (0 <= i < graph.nodeCount), one may find all
adjacent bonds and nodes using:

    const uint32_t* adjacencyPartition = graph.getAdjacencyPartition();
    const uint32_t* adjacentNodeIndices = graph.getAdjacentNodeIndices();
    const uint32_t* adjacentBondIndices = graph.getAdjacentBondIndices();

    // adj is the lookup value in adjacentNodeIndices and graph.getAdjacentBondIndices()
    for (uint32_t adj = adjacencyPartition[i]; adj < adjacencyPartition[i+1]; ++adj)
    {
        // An adjacent node:
        uint32_t adjacentNodeIndex = adjacentNodeIndices[adj];

        // The corresponding bond (that connects node index i with node indexed adjacentNodeIndex:
        uint32_t adjacentBondIndex = adjacentBondIndices[adj];
    }

For a graph node with index i, the corresponding asset chunk index is found using graph.getChunkIndices()[i].  The reverse mapping
(obtaining a graph node index from an asset chunk index) can be done using the

    NvBlastAssetGetChunkToGraphNodeMap(asset, logFn);

function.  See the documentation for its use.  The returned "node index" for a non-support chunk is the invalid value 0xFFFFFFFF.
*/
struct SupportGraph
{
    /**
    Total number of nodes in the support graph.
    */
    uint32_t    m_nodeCount;

    /**
    Indices of chunks represented by the nodes.

    getChunkIndices returns an array of size m_nodeCount.
    */
    NvBlastBlockArrayData(uint32_t, m_chunkIndicesOffset, getChunkIndices, m_nodeCount);

    /**
    Adjacency lookup table, of type uint32_t.

    Partitions both the getAdjacentNodeIndices() and the getAdjacentBondIndices() arrays into subsets corresponding to each node.
    The size of this array is nodeCount+1.
    For 0 <= i < nodeCount, getAdjacencyPartition()[i] is the index of the first element in getAdjacentNodeIndices() (or getAdjacentBondIndices()) for nodes adjacent to the node with index i.
    getAdjacencyPartition()[nodeCount] is the size of the getAdjacentNodeIndices() and getAdjacentBondIndices() arrays.
    This allows one to easily count the number of nodes adjacent to a node with index i, using getAdjacencyPartition()[i+1] - getAdjacencyPartition()[i].

    getAdjacencyPartition returns an array of size m_nodeCount + 1.
    */
    NvBlastBlockArrayData(uint32_t, m_adjacencyPartitionOffset, getAdjacencyPartition, m_nodeCount + 1);

    /**
    Array of uint32_t composed of subarrays holding the indices of nodes adjacent to a given node.  The subarrays may be accessed through the getAdjacencyPartition() array.

    getAdjacentNodeIndices returns an array of size getAdjacencyPartition()[m_nodeCount].
    */
    NvBlastBlockArrayData(uint32_t, m_adjacentNodeIndicesOffset, getAdjacentNodeIndices, getAdjacencyPartition()[m_nodeCount]);

    /**
    Array of uint32_t composed of subarrays holding the indices of bonds (NvBlastBond) for a given node.  The subarrays may be accessed through the getAdjacencyPartition() array.

    getAdjacentBondIndices returns an array of size getAdjacencyPartition()[m_nodeCount].
    */
    NvBlastBlockArrayData(uint32_t, m_adjacentBondIndicesOffset, getAdjacentBondIndices, getAdjacencyPartition()[m_nodeCount]);

    /**
    Finds the bond between two given graph nodes (if it exists) and returns the bond index.
    If no bond exists, returns invalidIndex<uint32_t>().

    \return the index of the bond between the given nodes.
    */
    uint32_t    findBond(uint32_t nodeIndex0, uint32_t nodeIndex1) const;
};


//////// SupportGraph inline member functions ////////

NV_INLINE uint32_t SupportGraph::findBond(uint32_t nodeIndex0, uint32_t nodeIndex1) const
{
    const uint32_t* adjacencyPartition = getAdjacencyPartition();
    const uint32_t* adjacentNodeIndices = getAdjacentNodeIndices();
    const uint32_t* adjacentBondIndices = getAdjacentBondIndices();

    // Iterate through all neighbors of nodeIndex0 chunk
    for (uint32_t i = adjacencyPartition[nodeIndex0]; i < adjacencyPartition[nodeIndex0 + 1]; i++)
    {
        if (adjacentNodeIndices[i] == nodeIndex1)
        {
            return adjacentBondIndices[i];
        }
    }

    return invalidIndex<uint32_t>();
}

} // namespace Blast
} // namespace Nv


#endif // ifndef NVBLASTSUPPORTGRAPH_H

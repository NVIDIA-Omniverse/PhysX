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


#ifndef NVBLASTGEOMETRY_H
#define NVBLASTGEOMETRY_H

#include "NvBlastTypes.h"
#include "NvBlastMath.h"
#include "NvBlastAssert.h"

#include <limits>


namespace Nv {
namespace Blast{


/**
Find the closest node to point in the graph. Uses primarily distance to chunk centroids.
Bond normals are expected to be directed from the lower to higher node index.
Cannot be used for graph actors with only the external chunk in the graph.

\param[in]  point                       the point to test against
\param[in]  firstGraphNodeIndex         the entry point for familyGraphNodeIndexLinks
\param[in]  familyGraphNodeIndexLinks   the list index links of the actor's graph
\param[in]  adjacencyPartition          the actor's SupportGraph adjacency partition
\param[in]  adjacentNodeIndices         the actor's SupportGraph adjacent node indices
\param[in]  adjacentBondIndices         the actor's SupportGraph adjacent bond indices
\param[in]  assetBonds                  the actor's asset bonds
\param[in]  bondHealths                 the actor's bond healths
\param[in]  assetChunks                 the actor's asset chunks
\param[in]  supportChunkHealths         the actor's graph chunks healths
\param[in]  chunkIndices                maps node index to chunk index in SupportGraph

\return     the index of the node closest to point
*/
NV_FORCE_INLINE uint32_t findClosestNode(const float point[4], 
    const uint32_t firstGraphNodeIndex, const uint32_t* familyGraphNodeIndexLinks,
    const uint32_t* adjacencyPartition, const uint32_t* adjacentNodeIndices, const uint32_t* adjacentBondIndices,
    const NvBlastBond* assetBonds, const float* bondHealths,
    const NvBlastChunk* assetChunks, const float* supportChunkHealths, const uint32_t* chunkIndices)
{
    // firstGraphNodeIndex could still be the external chunk, however
    // there should be no way a single-node actor that is just the external chunk exists.
    uint32_t nodeIndex = firstGraphNodeIndex;
    // Since there should always be a regular chunk in the graph, it is possible to initialize closestNode
    // as external chunk index but it would always evaluate to some meaningful node index eventually.
    uint32_t closestNode = nodeIndex;
    float minDist = std::numeric_limits<float>().max();

    // find the closest healthy chunk in the graph by its centroid to point distance
    while (!Nv::Blast::isInvalidIndex(nodeIndex))
    {
        if (supportChunkHealths[nodeIndex] > 0.0f)
        {
            uint32_t chunkIndex = chunkIndices[nodeIndex];
            if (!isInvalidIndex(chunkIndex)) // Invalid if this is the external chunk
            {
                const NvBlastChunk& chunk = assetChunks[chunkIndex];
                const float* centroid = chunk.centroid;

                float d[3]; VecMath::sub(point, centroid, d);
                float dist = VecMath::dot(d, d);

                if (dist < minDist)
                {
                    minDist = dist;
                    closestNode = nodeIndex;
                }
            }
        }
        nodeIndex = familyGraphNodeIndexLinks[nodeIndex];
    }

    // as long as the external chunk is not input as a single-node graph actor
    NVBLAST_ASSERT(!isInvalidIndex(chunkIndices[closestNode]));

    bool iterateOnBonds = true;
    if (iterateOnBonds)
    {
        // improve geometric accuracy by looking on which side of the closest bond the point lies
        // expects bond normals to point from the smaller to the larger node index

        nodeIndex = closestNode;
        minDist = std::numeric_limits<float>().max();

        const uint32_t startIndex = adjacencyPartition[nodeIndex];
        const uint32_t stopIndex = adjacencyPartition[nodeIndex + 1];

        for (uint32_t adjacentIndex = startIndex; adjacentIndex < stopIndex; adjacentIndex++)
        {
            const uint32_t neighbourIndex = adjacentNodeIndices[adjacentIndex];
            const uint32_t neighbourChunk = chunkIndices[neighbourIndex];
            if (!isInvalidIndex(neighbourChunk)) // Invalid if neighbor is the external chunk
            {
                const uint32_t bondIndex = adjacentBondIndices[adjacentIndex];
                // do not follow broken bonds, since it means that neighbor is not actually connected in the graph
                if (bondHealths[bondIndex] > 0.0f && supportChunkHealths[neighbourIndex] > 0.0f)
                {
                    const NvBlastBond& bond = assetBonds[bondIndex];

                    const float* centroid = bond.centroid;
                    float d[3]; VecMath::sub(point, centroid, d);
                    float dist = VecMath::dot(d, d);

                    if (dist < minDist)
                    {
                        minDist = dist;
                        float s = VecMath::dot(d, bond.normal);
                        if (nodeIndex < neighbourIndex)
                        {
                            closestNode = s < 0.0f ? nodeIndex : neighbourIndex;
                        }
                        else
                        {
                            closestNode = s < 0.0f ? neighbourIndex : nodeIndex;
                        }
                    }
                }
            }
        }
    }

    return closestNode;
}


/**
Find the closest node to point in the graph. Uses primarily distance to bond centroids.
Slower compared to chunk based lookup but may yield better accuracy in some cases.
Bond normals are expected to be directed from the lower to higher node index.
Cannot be used for graph actors with only the external chunk in the graph.

\param[in]  point                       the point to test against
\param[in]  firstGraphNodeIndex         the entry point for familyGraphNodeIndexLinks
\param[in]  familyGraphNodeIndexLinks   the list index links of the actor's graph
\param[in]  adjacencyPartition          the actor's SupportGraph adjacency partition
\param[in]  adjacentNodeIndices         the actor's SupportGraph adjacent node indices
\param[in]  adjacentBondIndices         the actor's SupportGraph adjacent bond indices
\param[in]  assetBonds                  the actor's asset bonds
\param[in]  bondHealths                 the actor's bond healths
\param[in]  chunkIndices                maps node index to chunk index in SupportGraph

\return     the index of the node closest to point
*/
NV_FORCE_INLINE uint32_t findClosestNode(const float point[4],
    const uint32_t firstGraphNodeIndex, const uint32_t* familyGraphNodeIndexLinks,
    const uint32_t* adjacencyPartition, const uint32_t* adjacentNodeIndices, const uint32_t* adjacentBondIndices,
    const NvBlastBond* bonds, const float* bondHealths, const uint32_t* chunkIndices)
{
    // firstGraphNodeIndex could still be the external chunk, however
    // there should be no way a single-node actor that is just the external chunk exists.
    uint32_t nodeIndex = firstGraphNodeIndex;
    // Since there should always be a regular chunk in the graph, it is possible to initialize closestNode
    // as external chunk index but it would always evaluate to some meaningful node index eventually.
    uint32_t closestNode = nodeIndex;
    float minDist = std::numeric_limits<float>().max();

    while (!Nv::Blast::isInvalidIndex(nodeIndex))
    {
        const uint32_t startIndex = adjacencyPartition[nodeIndex];
        const uint32_t stopIndex = adjacencyPartition[nodeIndex + 1];

        for (uint32_t adjacentIndex = startIndex; adjacentIndex < stopIndex; adjacentIndex++)
        {
            const uint32_t neighbourIndex = adjacentNodeIndices[adjacentIndex];
            if (nodeIndex < neighbourIndex)
            {
                const uint32_t bondIndex = adjacentBondIndices[adjacentIndex];
                if (bondHealths[bondIndex] > 0.0f)
                {
                    const NvBlastBond& bond = bonds[bondIndex];

                    const float* centroid = bond.centroid;
                    float d[3]; VecMath::sub(point, centroid, d);
                    float dist = VecMath::dot(d, d);

                    if (dist < minDist)
                    {
                        minDist = dist;
                        // if any of the nodes is the external chunk, use the valid one instead
                        if (isInvalidIndex(chunkIndices[neighbourIndex]))
                        {
                            closestNode = nodeIndex;
                        }
                        else if (isInvalidIndex(chunkIndices[nodeIndex]))
                        {
                            closestNode = neighbourIndex;
                        }
                        else
                        {
                            float s = VecMath::dot(d, bond.normal);
                            closestNode = s < 0 ? nodeIndex : neighbourIndex;
                        }
                    }
                }
            }
        }
        nodeIndex = familyGraphNodeIndexLinks[nodeIndex];
    }

    // as long as the external chunk is not input as a single-node graph actor
    NVBLAST_ASSERT(!isInvalidIndex(chunkIndices[closestNode]));
    return closestNode;
}


} // namespace Blast
} // namespace Nv


#endif // NVBLASTGEOMETRY_H

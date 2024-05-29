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
// Copyright (c) 2022-2024 NVIDIA Corporation. All rights reserved.


#include "AssetDTO.h"
#include "NvBlastGlobals.h"
#include "NvBlastIDDTO.h"
#include "NvBlastChunkDTO.h"
#include "NvBlastBondDTO.h"
#include "NvBlastAsset.h"


namespace Nv
{
namespace Blast
{

bool AssetDTO::serialize(Nv::Blast::Serialization::Asset::Builder builder, const Nv::Blast::Asset * poco)
{
    NvBlastIDDTO::serialize(builder.initID(), &poco->m_ID);

    builder.setLeafChunkCount(poco->m_leafChunkCount);

    builder.setFirstSubsupportChunkIndex(poco->m_firstSubsupportChunkIndex);

    capnp::List<Nv::Blast::Serialization::NvBlastChunk>::Builder chunks = builder.initChunks(poco->m_chunkCount);

    builder.setChunkCount(poco->m_chunkCount);

    NVBLAST_ASSERT_WITH_MESSAGE(builder.getChunkCount() == poco->m_chunkCount, "WTF");

    for (uint32_t i = 0; i < poco->m_chunkCount; i++)
    {
        NvBlastChunk& chunk = poco->getChunks()[i];

        NvBlastChunkDTO::serialize(chunks[i], &chunk);
    }

    NVBLAST_ASSERT_WITH_MESSAGE(builder.getChunkCount() == poco->m_chunkCount, "WTF");

    capnp::List<Nv::Blast::Serialization::NvBlastBond>::Builder bonds = builder.initBonds(poco->m_bondCount);

    builder.setBondCount(poco->m_bondCount);

    for (uint32_t i = 0; i < poco->m_bondCount; i++)
    {
        NvBlastBond& bond = poco->getBonds()[i];

        NvBlastBondDTO::serialize(bonds[i], &bond);
    }

    kj::ArrayPtr<uint32_t> stlcArray(poco->getSubtreeLeafChunkCounts(), poco->m_chunkCount);
    builder.initSubtreeLeafChunkCounts(poco->m_chunkCount);
    builder.setSubtreeLeafChunkCounts(stlcArray);

    kj::ArrayPtr<uint32_t> ctgnArray(poco->getChunkToGraphNodeMap(), poco->m_chunkCount);
    builder.setChunkToGraphNodeMap(ctgnArray);

    Nv::Blast::Serialization::NvBlastSupportGraph::Builder graphBulder = builder.initGraph();

    graphBulder.setNodeCount(poco->m_graph.m_nodeCount);

    uint32_t* ciPtr = poco->m_graph.getChunkIndices();

    kj::ArrayPtr<const uint32_t> ciArray(ciPtr, poco->m_graph.m_nodeCount);
    graphBulder.setChunkIndices(ciArray);

    kj::ArrayPtr<const uint32_t> adjPart(poco->m_graph.getAdjacencyPartition(), poco->m_graph.m_nodeCount + 1);
    graphBulder.setAdjacencyPartition(adjPart);

    NVBLAST_ASSERT(graphBulder.getAdjacencyPartition().size() == poco->m_graph.m_nodeCount + 1);

    kj::ArrayPtr<const uint32_t> nodeIndices(poco->m_graph.getAdjacentNodeIndices(), poco->m_bondCount * 2);
    graphBulder.setAdjacentNodeIndices(nodeIndices);

    NVBLAST_ASSERT(graphBulder.getAdjacentNodeIndices().size() == poco->m_bondCount * 2);

    kj::ArrayPtr<const uint32_t> bondIndices(poco->m_graph.getAdjacentBondIndices(), poco->m_bondCount * 2);
    graphBulder.setAdjacentBondIndices(bondIndices);

    return true;
}


Nv::Blast::Asset* AssetDTO::deserialize(Nv::Blast::Serialization::Asset::Reader reader)
{
    NvBlastAssetMemSizeData sizeData;
    sizeData.chunkCount = reader.getChunkCount();
    sizeData.nodeCount = reader.getGraph().getNodeCount();
    sizeData.bondCount = reader.getBondCount();
    const uint32_t leafChunkCount = reader.getLeafChunkCount();
    const uint32_t firstSubsupportChunkIndex = reader.getFirstSubsupportChunkIndex();
    const size_t assetSize = NvBlastGetAssetMemorySizeFromSizeData(sizeData, nullptr);
    void* mem = NVBLAST_ALLOC(assetSize);
    auto asset = Nv::Blast::initializeAsset(mem, sizeData.chunkCount, sizeData.nodeCount, leafChunkCount, firstSubsupportChunkIndex, sizeData.bondCount, logLL);

    if (deserializeInto(reader, asset))
        return asset;

    // free the memory so it doesn't leak
    NVBLAST_FREE(asset);
    return nullptr;
}


bool AssetDTO::deserializeInto(Nv::Blast::Serialization::Asset::Reader reader, Nv::Blast::Asset * poco)
{
    NvBlastIDDTO::deserializeInto(reader.getID(), &poco->m_ID);

    NvBlastBond* bonds = poco->getBonds();

    uint32_t bondCount = reader.getBondCount();
    auto readerBonds = reader.getBonds();
    for (uint32_t i = 0; i < bondCount; i++)
    {
        auto bondReader = readerBonds[i];

        NvBlastBondDTO::deserializeInto(bondReader, &bonds[i]);
    }

    NvBlastChunk* chunks = poco->getChunks();

    uint32_t chunkCount = reader.getChunkCount();
    auto readerChunks = reader.getChunks();
    for (uint32_t i = 0; i < chunkCount; i++)
    {
        auto chunkReader = readerChunks[i];

        NvBlastChunkDTO::deserializeInto(chunkReader, &chunks[i]);
    }

    poco->m_graph.m_nodeCount = reader.getGraph().getNodeCount();

    NVBLAST_ASSERT(reader.getSubtreeLeafChunkCounts().size() == poco->m_chunkCount);
    auto readerSubtreeLeafChunkCounts = reader.getSubtreeLeafChunkCounts();
    for (uint32_t i = 0; i < poco->m_chunkCount; i++)
    {
        poco->getSubtreeLeafChunkCounts()[i] = readerSubtreeLeafChunkCounts[i];
    }

    auto readerChunkToGraphNodeMap = reader.getChunkToGraphNodeMap();
    for (uint32_t i = 0; i < chunkCount; i++)
    {
        poco->getChunkToGraphNodeMap()[i] = readerChunkToGraphNodeMap[i];
    }

    uint32_t* ciPtr = poco->m_graph.getChunkIndices();

    NVBLAST_ASSERT(reader.getGraph().getChunkIndices().size() == poco->m_graph.m_nodeCount);
    auto readerGraphChunkIndices = reader.getGraph().getChunkIndices();
    for (uint32_t i = 0; i < poco->m_graph.m_nodeCount; i++)
    {
        ciPtr[i] = readerGraphChunkIndices[i];
    }

    uint32_t* adjPartition = poco->m_graph.getAdjacencyPartition();
    const uint32_t graphAdjacencyPartitionSize = reader.getGraph().getAdjacencyPartition().size();
    auto readerGraphAdjacencyPartition = reader.getGraph().getAdjacencyPartition();
    for (uint32_t i = 0; i < graphAdjacencyPartitionSize; ++i)
    {
        adjPartition[i] = readerGraphAdjacencyPartition[i];
    }

    uint32_t* adjNodes = poco->m_graph.getAdjacentNodeIndices();
    const uint32_t graphAdjacentNodeIndicesSize = reader.getGraph().getAdjacentNodeIndices().size();
    auto readerGraphAdjacentNodeIndices = reader.getGraph().getAdjacentNodeIndices();
    for (uint32_t i = 0; i < graphAdjacentNodeIndicesSize; ++i)
    {
        adjNodes[i] = readerGraphAdjacentNodeIndices[i];
    }

    uint32_t* adjBonds = poco->m_graph.getAdjacentBondIndices();
    const uint32_t graphAdjacentBondIndicesSize = reader.getGraph().getAdjacentBondIndices().size();
    auto readerGraphAdjacentBondIndices = reader.getGraph().getAdjacentBondIndices();
    for (uint32_t i = 0; i < graphAdjacentBondIndicesSize; ++i)
    {
        adjBonds[i] = readerGraphAdjacentBondIndices[i];
    }

    return true;
}

}   // namespace Blast
}   // namespace Nv

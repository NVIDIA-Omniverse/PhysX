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


#include "FamilyDTO.h"

#include "ActorDTO.h"
#include "AssetDTO.h"
#include "FamilyGraphDTO.h"
#include "NvBlastFamilyGraph.h"
#include "NvBlastGlobals.h"
#include "NvBlastIDDTO.h"
#include "NvBlastChunkDTO.h"
#include "NvBlastBondDTO.h"

#include <vector>

namespace Nv
{
namespace Blast
{

bool FamilyDTO::serialize(Nv::Blast::Serialization::Family::Builder builder, const Nv::Blast::FamilyHeader* poco)
{
    NvBlastIDDTO::serialize(builder.initAssetID(), &poco->m_assetID);

    // cache off the count data from the asset needed to re-create the family post serialization
    const NvBlastAssetMemSizeData sizeData = NvBlastAssetMemSizeDataFromAsset(poco->m_asset);
    builder.setBondCount(sizeData.bondCount);
    builder.setChunkCount(sizeData.chunkCount);
    builder.setNodeCount(sizeData.nodeCount);
    builder.setLowerSupportChunkCount(sizeData.lowerSupportChunkCount);
    builder.setUpperSupportChunkCount(sizeData.upperSupportChunkCount);

    // actorCount - these are active
    builder.setActorCount(poco->m_actorCount);

    // all possible actors
    const uint32_t actorCount = poco->getActorsArraySize();
    capnp::List<Nv::Blast::Serialization::Actor>::Builder actors = builder.initActors(actorCount);
    for (uint32_t i = 0; i < actorCount; i++)
    {
        Actor& actor = poco->getActors()[i];
        ActorDTO::serialize(actors[i], &actor);
    }

    // visibleChunkIndexLinks
    uint32_t* visibleChunkIndexLinks = reinterpret_cast<uint32_t *>(poco->getVisibleChunkIndexLinks());
    kj::ArrayPtr<uint32_t> visibleChunkIndexLinksArray(visibleChunkIndexLinks, sizeData.chunkCount * 2);
    builder.setVisibleChunkIndexLinks(visibleChunkIndexLinksArray);

    // chunkActorIndices
    kj::ArrayPtr<uint32_t> chunkActorIndicesArray(poco->getChunkActorIndices(), sizeData.chunkCount);
    builder.setChunkActorIndices(chunkActorIndicesArray);

    // graphNodeIndexLinks
    kj::ArrayPtr<uint32_t> graphNodeIndexLinksArray(poco->getGraphNodeIndexLinks(), sizeData.chunkCount);
    builder.setGraphNodeIndexLinks(graphNodeIndexLinksArray);

    // lowerSupportChunkHealths
    kj::ArrayPtr<float> lowerSupportChunkHealthsArray(poco->getLowerSupportChunkHealths(), sizeData.chunkCount);
    builder.setLowerSupportChunkHealths(lowerSupportChunkHealthsArray);

    // graphBondHealths
    kj::ArrayPtr<float> graphBondHealthsArray(poco->getBondHealths(), sizeData.bondCount);
    builder.setGraphBondHealths(graphBondHealthsArray);

    // familyGraph
    FamilyGraph *graph = poco->getFamilyGraph();
    auto builderGraph = builder.initFamilyGraph();
    builderGraph.setNodeCount(sizeData.nodeCount);
    FamilyGraphDTO::serialize(builderGraph, graph);

    return true;
}


Nv::Blast::FamilyHeader* FamilyDTO::deserialize(Nv::Blast::Serialization::Family::Reader reader)
{
    // fill in the count info from the reader
    NvBlastAssetMemSizeData sizeData;
    sizeData.bondCount = reader.getBondCount();
    sizeData.chunkCount = reader.getChunkCount();
    sizeData.nodeCount = reader.getNodeCount();
    sizeData.lowerSupportChunkCount = reader.getLowerSupportChunkCount();
    sizeData.upperSupportChunkCount = reader.getUpperSupportChunkCount();

    // allocate enough space to hold the family
    const size_t familySize = NvBlastAssetGetFamilyMemorySizeFromSizeData(sizeData, nullptr);
    void* mem = NVBLAST_ALLOC(familySize);

    // use the count info to initialize the family
    auto family = reinterpret_cast<Nv::Blast::FamilyHeader *>(NvBlastAssetCreateFamilyFromSizeData(mem, sizeData, Nv::Blast::logLL));

    // then fill in the data from the reader
    if (deserializeInto(reader, family))
        return family;

    // failed to deserialize, free the allocated memory so it doesn't leak
    NVBLAST_FREE(mem);
    return nullptr;
}


bool FamilyDTO::deserializeInto(Nv::Blast::Serialization::Family::Reader reader, Nv::Blast::FamilyHeader* poco)
{
    NvBlastIDDTO::deserializeInto(reader.getAssetID(), &poco->m_assetID);

    // active actor count
    poco->m_actorCount = reader.getActorCount();

    // all possible actors
    Actor* actors = poco->getActors();
    auto readerActors = reader.getActors();
    NVBLAST_ASSERT(poco->m_actorCount <= readerActors.size());
    for (uint32_t i = 0; i < readerActors.size(); i++)
    {
        auto actorReader = readerActors[i];
        ActorDTO::deserializeInto(actorReader, &actors[i]);
    }

    // visibleChunkIndexLinks
    // they are stored in the buffer as a flat list of uint32_t values,
    // but stored as pairs in the Family
    auto readerVisibleChunkIndexLinks = reader.getVisibleChunkIndexLinks();
    const uint32_t numVisibleChunkIndexLinks = readerVisibleChunkIndexLinks.size();
    for (uint32_t i = 0; i < numVisibleChunkIndexLinks; i += 2)
    {
        const uint32_t vcil = i / 2;
        poco->getVisibleChunkIndexLinks()[vcil].m_adj[0] = readerVisibleChunkIndexLinks[i];
        poco->getVisibleChunkIndexLinks()[vcil].m_adj[1] = readerVisibleChunkIndexLinks[i+1];
    }

    // chunkActorIndices
    auto readerChunkActorIndices = reader.getChunkActorIndices();
    const uint32_t numChunkActorIndices = readerChunkActorIndices.size();
    for (uint32_t i = 0; i < numChunkActorIndices; i++)
    {
        poco->getChunkActorIndices()[i] = readerChunkActorIndices[i];
    }

    // graphNodeIndexLinks
    auto readerGraphNodeIndexLinks = reader.getGraphNodeIndexLinks();
    const uint32_t numGraphNodeIndexLinks = readerGraphNodeIndexLinks.size();
    for (uint32_t i = 0; i < numGraphNodeIndexLinks; i++)
    {
        poco->getGraphNodeIndexLinks()[i] = readerGraphNodeIndexLinks[i];
    }

    // lowerSupportChunkHealths
    auto readerLowerSupportChunkHealths = reader.getLowerSupportChunkHealths();
    const uint32_t numLowerSupportChunkHealths = readerLowerSupportChunkHealths.size();
    for (uint32_t i = 0; i < numLowerSupportChunkHealths; i++)
    {
        poco->getLowerSupportChunkHealths()[i] = readerLowerSupportChunkHealths[i];
    }

    // graphBondHealths
    auto readerGraphBondHealths = reader.getGraphBondHealths();
    const uint32_t numGraphBondHealths = readerGraphBondHealths.size();
    for (uint32_t i = 0; i < numGraphBondHealths; i++)
    {
        poco->getBondHealths()[i] = readerGraphBondHealths[i];
    }

    // familyGraph
    FamilyGraphDTO::deserializeInto(reader.getFamilyGraph(), poco->getFamilyGraph());

    return true;
}

}   // namespace Blast
}   // namespace Nv

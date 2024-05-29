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


#include "FamilyGraphDTO.h"
#include "NvBlastGlobals.h"

namespace Nv
{
namespace Blast
{

bool FamilyGraphDTO::serialize(Nv::Blast::Serialization::FamilyGraph::Builder builder, const Nv::Blast::FamilyGraph * poco)
{
    // this needs to be set externally so we have access to it here
    const uint32_t nodeCount = builder.getNodeCount();

    kj::ArrayPtr<IslandId> islandIdsArray(poco->getIslandIds(), nodeCount);
    builder.setIslandIds(islandIdsArray);

    kj::ArrayPtr<NodeIndex> dirtyNodeLinksArray(poco->getDirtyNodeLinks(), nodeCount);
    builder.setDirtyNodeLinks(dirtyNodeLinksArray);

    kj::ArrayPtr<uint32_t> firstDirtyNodeIndicesArray(poco->getFirstDirtyNodeIndices(), nodeCount);
    builder.setFirstDirtyNodeIndices(firstDirtyNodeIndicesArray);

    kj::ArrayPtr<NodeIndex> fastRouteArray(poco->getFastRoute(), nodeCount);
    builder.setFastRoute(fastRouteArray);

    kj::ArrayPtr<uint32_t> hopCountsArray(poco->getHopCounts(), nodeCount);
    builder.setHopCounts(hopCountsArray);

    auto isEdgeRemoved = poco->getIsEdgeRemoved();
    uint8_t* isEdgeRemovedData = reinterpret_cast<uint8_t*>(const_cast<char*>(isEdgeRemoved->getData()));
    capnp::Data::Reader isEdgeRemovedReader(isEdgeRemovedData, isEdgeRemoved->getSize());
    builder.setIsEdgeRemoved(isEdgeRemovedReader);


    auto isNodeInDirtyList = poco->getIsNodeInDirtyList();
    uint8_t* isNodeInDirtyListData = reinterpret_cast<uint8_t*>(const_cast<char*>(isNodeInDirtyList->getData()));
    capnp::Data::Reader isNodeInDirtyListReader(isNodeInDirtyListData, isNodeInDirtyList->getSize());
    builder.setIsNodeInDirtyList(isNodeInDirtyListReader);

    return true;
}


Nv::Blast::FamilyGraph* FamilyGraphDTO::deserialize(Nv::Blast::Serialization::FamilyGraph::Reader reader)
{
    NV_UNUSED(reader);
    return nullptr;
}


bool FamilyGraphDTO::deserializeInto(Nv::Blast::Serialization::FamilyGraph::Reader reader, Nv::Blast::FamilyGraph * poco)
{
    auto readerIslandIds = reader.getIslandIds();
    const uint32_t numIslandIds = readerIslandIds.size();
    for (uint32_t i = 0; i < numIslandIds; i++)
    {
        poco->getIslandIds()[i] = readerIslandIds[i];
    }

    auto readerDirtyNodeLinks = reader.getDirtyNodeLinks();
    const uint32_t numDirtyNodeLinks = readerDirtyNodeLinks.size();
    for (uint32_t i = 0; i < numDirtyNodeLinks; i++)
    {
        poco->getDirtyNodeLinks()[i] = readerDirtyNodeLinks[i];
    }

    auto readerFirstDirtyNodeIndices = reader.getFirstDirtyNodeIndices();
    const uint32_t numFirstDirtyNodeIndices = readerFirstDirtyNodeIndices.size();
    for (uint32_t i = 0; i < numFirstDirtyNodeIndices; i++)
    {
        poco->getFirstDirtyNodeIndices()[i] = readerFirstDirtyNodeIndices[i];
    }

    auto readerFastRoute = reader.getFastRoute();
    const uint32_t numFastRoute = readerFastRoute.size();
    for (uint32_t i = 0; i < numFastRoute; i++)
    {
        poco->getFastRoute()[i] = readerFastRoute[i];
    }

    auto readerHopCounts = reader.getHopCounts();
    const uint32_t numHopCounts = readerHopCounts.size();
    for (uint32_t i = 0; i < numHopCounts; i++)
    {
        poco->getHopCounts()[i] = readerHopCounts[i];
    }

    auto readerIsEdgeRemoved = reader.getIsEdgeRemoved();
    const uint32_t numIsEdgeRemoved = readerIsEdgeRemoved.size();
    const char* isEdgeRemovedData = reinterpret_cast<const char*>(readerIsEdgeRemoved.begin());
    auto isEdgeRemoved = poco->getIsEdgeRemoved();
    isEdgeRemoved->setData(isEdgeRemovedData, numIsEdgeRemoved);

    auto readerIsNodeInDirtyList = reader.getIsNodeInDirtyList();
    const uint32_t numIsNodeInDirtyList = readerIsNodeInDirtyList.size();
    const char* readerIsNodeInDirtyListData = reinterpret_cast<const char*>(readerIsNodeInDirtyList.begin());
    auto isNodeInDirtyList = poco->getIsNodeInDirtyList();
    isNodeInDirtyList->setData(readerIsNodeInDirtyListData, numIsNodeInDirtyList);

    return true;
}

}   // namespace Blast
}   // namespace Nv

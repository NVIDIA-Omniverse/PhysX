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


#include "NvBlastTkFrameworkImpl.h"
#include "NvBlastTkAssetImpl.h"
#include "NvBlastTkFamilyImpl.h"

#include "NvBlast.h"
#include "NvBlastMemory.h"


namespace Nv
{
namespace Blast
{

//////// Static data ////////

NVBLASTTK_DEFINE_TYPE_IDENTIFIABLE(Asset);


//////// Member functions ////////

TkAssetImpl::TkAssetImpl()
    : m_assetLL(nullptr), m_ownsAsset(false)
{
}


TkAssetImpl::TkAssetImpl(const NvBlastID& id)
    : TkAssetType(id), m_assetLL(nullptr), m_ownsAsset(false)
{
}


TkAssetImpl::~TkAssetImpl()
{
    if (m_assetLL != nullptr && m_ownsAsset)
    {
        NVBLAST_FREE(m_assetLL);
    }
}


const NvBlastAsset* TkAssetImpl::getAssetLL() const
{
    return getAssetLLInternal();
}


uint32_t TkAssetImpl::getChunkCount() const
{
    return NvBlastAssetGetChunkCount(m_assetLL, logLL);
}


uint32_t TkAssetImpl::getLeafChunkCount() const
{
    return NvBlastAssetGetLeafChunkCount(m_assetLL, logLL);
}


uint32_t TkAssetImpl::getBondCount() const
{
    return NvBlastAssetGetBondCount(m_assetLL, logLL);
}


const NvBlastChunk* TkAssetImpl::getChunks() const
{
    return NvBlastAssetGetChunks(m_assetLL, logLL);
}


const NvBlastBond* TkAssetImpl::getBonds() const
{
    return NvBlastAssetGetBonds(m_assetLL, logLL);
}


const NvBlastSupportGraph TkAssetImpl::getGraph() const
{
    return NvBlastAssetGetSupportGraph(m_assetLL, logLL);
}


uint32_t TkAssetImpl::getDataSize() const
{
    return NvBlastAssetGetSize(m_assetLL, logLL);
}


uint32_t TkAssetImpl::getJointDescCount() const
{
    return getJointDescCountInternal();
}


const TkAssetJointDesc* TkAssetImpl::getJointDescs() const
{
    return getJointDescsInternal();
}


void TkAssetImpl::release()
{
    const TkType& tkType = TkFamilyImpl::s_type;
    const uint32_t num = TkFrameworkImpl::get()->getObjectCount(tkType);

    if (num)
    {
        Array<TkIdentifiable*>::type dependents(num);
        TkFrameworkImpl::get()->getObjects(dependents.begin(), dependents.size(), tkType);

        for (TkObject* o : dependents)
        {
            TkFamilyImpl* f = static_cast<TkFamilyImpl*>(o);
            if (f->getAssetImpl() == this)
            {
                f->release();
            }
        }
    }

    NVBLAST_DELETE(this, TkAssetImpl);
}


//////// Static functions ////////

TkAssetImpl* TkAssetImpl::create(const TkAssetDesc& desc)
{
    TkAssetImpl* asset = NVBLAST_NEW(TkAssetImpl);

    Array<char>::type scratch((uint32_t)NvBlastGetRequiredScratchForCreateAsset(&desc, logLL));
    void* mem = NVBLAST_ALLOC_NAMED(NvBlastGetAssetMemorySize(&desc, logLL), "TkAssetImpl::create");
    asset->m_assetLL = NvBlastCreateAsset(mem, &desc, scratch.begin(), logLL);
    if (asset->m_assetLL == nullptr)
    {
        NVBLAST_LOG_ERROR("TkAssetImpl::create: low-level asset could not be created.");
        asset->release();
        return nullptr;
    }

    if (desc.bondFlags != nullptr)
    {
        for (uint32_t bondN = 0; bondN < desc.bondCount; ++bondN)
        {
            if (0 != (desc.bondFlags[bondN] & TkAssetDesc::BondJointed))
            {
                const NvBlastBondDesc& bondDesc = desc.bondDescs[bondN];
                const uint32_t c0 = bondDesc.chunkIndices[0];
                const uint32_t c1 = bondDesc.chunkIndices[1];
                if (c0 >= desc.chunkCount || c1 >= desc.chunkCount)
                {
                    NVBLAST_LOG_WARNING("TkAssetImpl::create: joint flag set for badly described bond.  No joint descriptor created.");
                    continue;
                }

                if (!asset->addJointDesc(c0, c1))
                {
                    NVBLAST_LOG_WARNING("TkAssetImpl::create: no bond corresponds to the user-described bond indices.  No joint descriptor created.");
                }
            }
        }
    }

    asset->m_ownsAsset = true;
//  asset->setID(NvBlastAssetGetID(asset->m_assetLL, logLL));   // Keeping LL and Tk IDs distinct

    return asset;
}


TkAssetImpl* TkAssetImpl::create(const NvBlastAsset* assetLL, Nv::Blast::TkAssetJointDesc* jointDescs, uint32_t jointDescCount, bool ownsAsset)
{
    TkAssetImpl* asset = NVBLAST_NEW(TkAssetImpl);

    //NOTE: Why are we passing in a const NvBlastAsset* and then discarding the const?
    asset->m_assetLL = const_cast<NvBlastAsset*>(assetLL);
    if (asset->m_assetLL == nullptr)
    {
        NVBLAST_LOG_ERROR("TkAssetImpl::create: low-level asset could not be created.");
        asset->release();
        return nullptr;
    }

    asset->m_ownsAsset = ownsAsset;
    asset->setID(NvBlastAssetGetID(asset->m_assetLL, logLL));

    asset->m_jointDescs.resize(jointDescCount);
    for (uint32_t i = 0; i < asset->m_jointDescs.size(); ++i)
    {
        asset->m_jointDescs[i] = jointDescs[i];
    }

    return asset;
}

bool TkAssetImpl::addJointDesc(uint32_t chunkIndex0, uint32_t chunkIndex1)
{
    if (m_assetLL == nullptr)
    {
        return false;
    }

    const uint32_t upperSupportChunkCount = NvBlastAssetGetFirstSubsupportChunkIndex(m_assetLL, logLL);
    if (chunkIndex0 >= upperSupportChunkCount || chunkIndex1 >= upperSupportChunkCount)
    {
        return false;
    }

    const uint32_t* chunkToGraphNodeMap = NvBlastAssetGetChunkToGraphNodeMap(m_assetLL, logLL);
    const uint32_t node0 = chunkToGraphNodeMap[chunkIndex0];
    const uint32_t node1 = chunkToGraphNodeMap[chunkIndex1];
    const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(m_assetLL, logLL);
    if (node0 >= graph.nodeCount || node1 >= graph.nodeCount)
    {
        return false;
    }

    // Find bond index
    // Iterate through all neighbors of node0 chunk
    uint32_t bondIndex = 0xFFFFFFFF;
    for (uint32_t i = graph.adjacencyPartition[node0]; i < graph.adjacencyPartition[node0 + 1]; i++)
    {
        if (graph.adjacentNodeIndices[i] == node1)
        {
            bondIndex = graph.adjacentBondIndices[i];
            break;
        }
    }

    if (bondIndex >= NvBlastAssetGetBondCount(m_assetLL, logLL))
    {
        return false;
    }

    const NvBlastBond& bond = NvBlastAssetGetBonds(m_assetLL, logLL)[bondIndex];

    TkAssetJointDesc jointDesc;
    jointDesc.attachPositions[0] = jointDesc.attachPositions[1] = nvidia::NvVec3(bond.centroid[0], bond.centroid[1], bond.centroid[2]);
    jointDesc.nodeIndices[0] = node0;
    jointDesc.nodeIndices[1] = node1;
    m_jointDescs.pushBack(jointDesc);

    return true;
}

} // namespace Blast
} // namespace Nv

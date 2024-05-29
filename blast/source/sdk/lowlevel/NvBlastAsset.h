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


#ifndef NVBLASTASSET_H
#define NVBLASTASSET_H


#include "NvBlastSupportGraph.h"
#include "NvBlast.h"
#include "NvBlastAssert.h"
#include "NvBlastIndexFns.h"
#include "NvBlastChunkHierarchy.h"


namespace Nv
{
namespace Blast
{

class Asset : public NvBlastAsset
{
public:

    /**
    Struct-enum which is used to mark chunk descriptors when building an asset.
    */
    struct ChunkAnnotation
    {
        enum Enum
        {
            Parent = (1 << 0),
            Support = (1 << 1),
            SuperSupport = (1 << 2),

            // Combinations
            UpperSupport = Support | SuperSupport
        };
    };


    /**
    Create an asset from a descriptor.

    \param[in] mem      Pointer to block of memory of at least the size given by getMemorySize(desc).  Must be 16-byte aligned.
    \param[in] desc     Asset descriptor (see NvBlastAssetDesc).
    \param[in] scratch  User-supplied scratch memory of size createRequiredScratch(desc) bytes.
    \param[in] logFn    User-supplied message function (see NvBlastLog definition).  May be NULL.

    \return the pointer to the new asset, or nullptr if unsuccessful.
    */
    static Asset*   create(void* mem, const NvBlastAssetDesc* desc, void* scratch, NvBlastLog logFn);

    /**
    Returns the number of bytes of memory that an asset created using the given descriptor will require.  A pointer
    to a block of memory of at least this size must be passed in as the mem argument of create.

    \param[in] desc The asset descriptor that will be passed into NvBlastCreateAsset.
    */
    static size_t   getMemorySize(const NvBlastAssetDesc* desc);

    /**
    Returns the size of the scratch space (in bytes) required to be passed into the create function, based upon
    the input descriptor that will be passed to the create function.

    \param[in] desc The descriptor that will be passed to the create function.

    \return the number of bytes required.
    */
    static size_t   createRequiredScratch(const NvBlastAssetDesc* desc, NvBlastLog logFn);


    /**
    Returns the number of upper-support chunks in this asset..

    \return the number of upper-support chunks.
    */
    uint32_t        getUpperSupportChunkCount() const;

    /**
    Returns the number of lower-support chunks in this asset.  This is the required actor buffer size for a Actor family.

    \return the number of lower-support chunks.
    */
    uint32_t        getLowerSupportChunkCount() const;

    /**
    Returns the number of bonds in this asset's support graph.

    \return the number of bonds in this asset's support graph.
    */
    uint32_t        getBondCount() const;

    /**
    Returns the number of separate chunk hierarchies in the asset.  This will be the initial number of visible chunks in an actor instanced from this asset.

    \return the number of separate chunk hierarchies in the asset.
    */
    uint32_t        getHierarchyCount() const;

    /**
    Maps all lower-support chunk indices to a contiguous range [0, getLowerSupportChunkCount()).

    \param[in] chunkIndex   Asset chunk index.

    \return an index in the range [0, getLowerSupportChunkCount()) if it is a lower-support chunk, invalidIndex<uint32_t>() otherwise.
    */
    uint32_t        getContiguousLowerSupportIndex(uint32_t chunkIndex) const;


    // Static functions

    /**
    Function to ensure support coverage of chunks.

    Support chunks (marked in the NvBlastChunkDesc struct) must provide full coverage over the asset.
    This means that from any leaf chunk to the root node, exactly one chunk must be support.  If this condition
    is not met, the actual support chunks will be adjusted accordingly.

    Chunk order depends on support coverage, so this function should be called before chunk reordering.

    \param[out] supportChunkCount   The number of support chunks.  NOTE - this value is not meaninful if testOnly = true and the return value is false.
    \param[out] leafChunkCount      The number of leaf chunks.  NOTE - this value is not meaninful if testOnly = true and the return value is false.
    \param[out] chunkAnnotation     User-supplied char array of size chunkCount.  NOTE - these values are not meaninful if testOnly = true and the return value is false.
    \param[in]  chunkCount          The number of chunk descriptors.
    \param[in]  chunkDescs          Array of chunk descriptors of size chunkCount. It will be updated accordingly.
    \param[in]  testOnly            If true, this function early-outs if support coverage is not exact.  If false, exact coverage is ensured by possibly modifying chunkDescs' flags.
    \param[in]  logFn               User-supplied message function (see NvBlastLog definition).  May be NULL.

    \return true iff coverage was already exact.
    */
    static bool     ensureExactSupportCoverage(uint32_t& supportChunkCount, uint32_t& leafChunkCount, char* chunkAnnotation, uint32_t chunkCount, NvBlastChunkDesc* chunkDescs, bool testOnly, NvBlastLog logFn);

    /**
    Tests a set of chunk descriptors to see if chunks are in valid chunk order.

    Chunk order conditions checked:
    1. 'all chunks with same parent index should go in a row'.
    2. 'chunks should come after their parents'.
    3. 'root chunks should go first'.
    4. 'upper-support chunks should come before subsupport chunks'.

    \param[in]  chunkCount      The number of chunk descriptors.
    \param[in]  chunkDescs      An array of chunk descriptors of length chunkCount.
    \param[in]  chunkAnnotation Annotation generated from ensureExactSupportCoverage (see ensureExactSupportCoverage).
    \param[in]  scratch         User-supplied scratch memory of chunkCount bytes.

    \return true if the descriptors meet the ordering conditions, false otherwise.
    */
    static bool     testForValidChunkOrder(uint32_t chunkCount, const NvBlastChunkDesc* chunkDescs, const char* chunkAnnotation, void* scratch);


    //////// Data ////////

    /**
    Asset data block header.
    */
    NvBlastDataBlock    m_header;

    /**
    ID for this asset.
    */
    NvBlastID           m_ID;

    /**
    The total number of chunks in the asset, support and non-support.
    */
    uint32_t            m_chunkCount;

    /**
    The support graph.
    */
    SupportGraph        m_graph;

    /**
    The number of leaf chunks in the asset.
    */
    uint32_t            m_leafChunkCount;

    /**
    Chunks are sorted such that subsupport chunks come last.  This is the first subsupport chunk index.  Equals m_chunkCount if there are no subsupport chunks.
    */
    uint32_t            m_firstSubsupportChunkIndex;

    /**
    The number of bonds in the asset.
    */
    uint32_t            m_bondCount;

    /**
    Chunks, of type NvBlastChunk.

    getChunks returns an array of size m_chunkCount.
    */
    NvBlastBlockArrayData(NvBlastChunk, m_chunksOffset, getChunks, m_chunkCount);

    /**
    Array of bond data for the interfaces between two chunks.  Since the bond is shared by two chunks, the same
    bond data is used for chunk[i] -> chunk[j] as for chunk[j] -> chunk[i].
    The size of the array is m_graph.adjacencyPartition[m_graph.m_nodeCount]/2.
    See NvBlastBond.

    getBonds returns an array of size m_bondCount.
    */
    NvBlastBlockArrayData(NvBlastBond, m_bondsOffset, getBonds, m_bondCount);

    /**
    Caching the number of leaf chunks descended from each chunk (including the chunk itself).
    This data parallels the Chunks array, and is an array of the same size.

    getSubtreeLeafChunkCount returns a uint32_t array of size m_chunkCount.
    */
    NvBlastBlockArrayData(uint32_t, m_subtreeLeafChunkCountsOffset, getSubtreeLeafChunkCounts, m_chunkCount);

    /**
    Mapping from chunk index to graph node index (inverse of m_graph.getChunkIndices().

    getChunkToGraphNodeMap returns a uint32_t array of size m_chunkCount.
    */
    NvBlastBlockArrayData(uint32_t, m_chunkToGraphNodeMapOffset, getChunkToGraphNodeMap, m_chunkCount);


    //////// Iterators ////////

    /**
    Chunk hierarchy depth-first iterator.  Traverses subtree with root given by startChunkIndex.
    If upperSupportOnly == true, then the iterator will not traverse subsuppport chunks.
    */
    class DepthFirstIt : public ChunkDepthFirstIt
    {
    public:
        /** Constructed from an asset. */
        DepthFirstIt(const Asset& asset, uint32_t startChunkIndex, bool upperSupportOnly = false) :
            ChunkDepthFirstIt(asset.getChunks(), startChunkIndex, upperSupportOnly ? asset.getUpperSupportChunkCount() : asset.m_chunkCount) {}
    };
};


//////// Asset inline member functions ////////

NV_INLINE uint32_t Asset::getUpperSupportChunkCount() const
{
    return m_firstSubsupportChunkIndex;
}


NV_INLINE uint32_t Asset::getLowerSupportChunkCount() const
{
    return m_graph.m_nodeCount + (m_chunkCount - m_firstSubsupportChunkIndex);
}


NV_INLINE uint32_t Asset::getBondCount() const
{
    NVBLAST_ASSERT((m_graph.getAdjacencyPartition()[m_graph.m_nodeCount] & 1) == 0);    // The bidirectional graph data should have an even number of edges
    return m_graph.getAdjacencyPartition()[m_graph.m_nodeCount] / 2;    // Directional bonds, divide by two
}


NV_INLINE uint32_t Asset::getHierarchyCount() const
{
    const NvBlastChunk* chunks = getChunks();
    for (uint32_t i = 0; i < m_chunkCount; ++i)
    {
        if (!isInvalidIndex(chunks[i].parentChunkIndex))
        {
            return i;
        }
    }
    return m_chunkCount;
}


NV_INLINE uint32_t Asset::getContiguousLowerSupportIndex(uint32_t chunkIndex) const
{
    NVBLAST_ASSERT(chunkIndex < m_chunkCount);

    return chunkIndex < m_firstSubsupportChunkIndex ? getChunkToGraphNodeMap()[chunkIndex] : (chunkIndex - m_firstSubsupportChunkIndex + m_graph.m_nodeCount);
}


//JDM: Expose this so serialization layer can use it.
NV_C_API Asset* initializeAsset(void* mem, uint32_t chunkCount, uint32_t graphNodeCount, uint32_t leafChunkCount, uint32_t firstSubsupportChunkIndex, uint32_t bondCount, NvBlastLog logFn);

} // namespace Blast
} // namespace Nv


#endif // ifndef NVBLASTASSET_H

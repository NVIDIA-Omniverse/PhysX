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


#include "NvBlastAsset.h"
#include "NvBlastIndexFns.h"
#include "NvBlastAssert.h"
#include "NvBlastMemory.h"
#include "NvBlastMath.h"
#include "NvBlastPreprocessorInternal.h"

#include <algorithm>


namespace Nv
{
namespace Blast
{

/**
Class to hold chunk descriptor and annotation context for sorting a list of indices
*/
class ChunksOrdered
{
public:
    ChunksOrdered(const NvBlastChunkDesc* descs, const char* annotation)
        : m_descs(descs), m_annotation(annotation), m_chunkMap(nullptr), m_chunkInvMap(nullptr) {}

    // Map and inverse to apply to chunk descs
    bool    setMap(const uint32_t* map, const uint32_t* inv)
    {
        if ((map == nullptr) != (inv == nullptr))
        {
            return false;
        }
        m_chunkMap = map;
        m_chunkInvMap = inv;
        return true;
    }

    bool    operator () (uint32_t ii0, uint32_t ii1) const
    {
        const uint32_t i0 = m_chunkMap ? m_chunkMap[ii0] : ii0;
        const uint32_t i1 = m_chunkMap ? m_chunkMap[ii1] : ii1;

        const bool upperSupport0 = (m_annotation[i0] & Asset::ChunkAnnotation::UpperSupport) != 0;
        const bool upperSupport1 = (m_annotation[i1] & Asset::ChunkAnnotation::UpperSupport) != 0;

        if (upperSupport0 != upperSupport1)
        {
            return upperSupport0;   // If one is uppersupport and one is subsupport, uppersupport should come first
        }

        const uint32_t p0 = m_descs[i0].parentChunkDescIndex;
        const uint32_t p1 = m_descs[i1].parentChunkDescIndex;

        // Parent chunk index (+1 so that UINT32_MAX becomes the lowest value)
        const uint32_t pp0 = 1 + (m_chunkInvMap && !isInvalidIndex(p0) ? m_chunkInvMap[p0] : p0);
        const uint32_t pp1 = 1 + (m_chunkInvMap && !isInvalidIndex(p1) ? m_chunkInvMap[p1] : p1);

        return pp0 < pp1;   // With the same support relationship, order by parent index
    }

private:
    const NvBlastChunkDesc* m_descs;
    const char*             m_annotation;
    const uint32_t*         m_chunkMap;
    const uint32_t*         m_chunkInvMap;
};

} // namespace Blast
} // namespace Nv


using namespace Nv::Blast;

extern "C"
{

bool NvBlastBuildAssetDescChunkReorderMap(uint32_t* chunkReorderMap, const NvBlastChunkDesc* chunkDescs, uint32_t chunkCount, void* scratch, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(chunkCount == 0 || chunkDescs != nullptr, logFn, "NvBlastBuildAssetDescChunkReorderMap: NULL chunkDescs input with non-zero chunkCount", return false);
    NVBLASTLL_CHECK(chunkReorderMap == nullptr || chunkCount != 0, logFn, "NvBlastBuildAssetDescChunkReorderMap: NULL chunkReorderMap input with non-zero chunkCount", return false);
    NVBLASTLL_CHECK(chunkCount == 0 || scratch != nullptr, logFn, "NvBlastBuildAssetDescChunkReorderMap: NULL scratch input with non-zero chunkCount", return false);

    uint32_t* composedMap = static_cast<uint32_t*>(scratch);    scratch = pointerOffset(scratch, chunkCount * sizeof(uint32_t));
    uint32_t* chunkMap = static_cast<uint32_t*>(scratch);   scratch = pointerOffset(scratch, chunkCount * sizeof(uint32_t));
    char* chunkAnnotation = static_cast<char*>(scratch);    scratch = pointerOffset(scratch, chunkCount * sizeof(char));

    uint32_t supportChunkCount;
    uint32_t leafChunkCount;
    if (!Asset::ensureExactSupportCoverage(supportChunkCount, leafChunkCount, chunkAnnotation, chunkCount, const_cast<NvBlastChunkDesc*>(chunkDescs), true, logFn))
    {
        NVBLASTLL_LOG_ERROR(logFn, "NvBlastBuildAssetDescChunkReorderMap: chunk descriptors did not have exact coverage, map could not be built.  Use NvBlastEnsureAssetExactSupportCoverage to fix descriptors.");
        return false;
    }

    // Initialize composedMap and its inverse to identity
    for (uint32_t i = 0; i < chunkCount; ++i)
    {
        composedMap[i] = i;
        chunkReorderMap[i] = i;
    }

    // Create a chunk ordering operator using the composedMap
    ChunksOrdered chunksOrdered(chunkDescs, chunkAnnotation);
    chunksOrdered.setMap(composedMap, chunkReorderMap);

    // Check initial order
    bool ordered = true;
    if (chunkCount > 1)
    {
        for (uint32_t i = chunkCount - 1; ordered && i--;)
        {
            ordered = !chunksOrdered(i + 1, i);
        }
    }
    if (ordered)
    {
        return true;    // Initially ordered, return true
    }

    NVBLAST_ASSERT(chunkCount > 1);

    // Max depth is bounded by chunkCount, so that is the vound on the number of iterations
    uint32_t iter = chunkCount;
    do
    {
        // Reorder based on current composed map
        for (uint32_t i = 0; i < chunkCount; ++i)
        {
            chunkMap[i] = i;
        }
        std::stable_sort(chunkMap, chunkMap + chunkCount, chunksOrdered);

        // Fold chunkMap into composedMap
        for (uint32_t i = 0; i < chunkCount; ++i)
        {
            chunkMap[i] = composedMap[chunkMap[i]];
        }
        for (uint32_t i = 0; i < chunkCount; ++i)
        {
            composedMap[i] = chunkMap[i];
            chunkMap[i] = i;
        }
        invertMap(chunkReorderMap, composedMap, chunkCount);

        // Check order
        ordered = true;
        for (uint32_t i = chunkCount - 1; ordered && i--;)
        {
            ordered = !chunksOrdered(i + 1, i);
        }
    } while (!ordered && iter--);

    NVBLAST_ASSERT(ordered);

    return false;
}


void NvBlastApplyAssetDescChunkReorderMap
(
    NvBlastChunkDesc* reorderedChunkDescs,
    const NvBlastChunkDesc* chunkDescs,
    uint32_t chunkCount,
    NvBlastBondDesc* bondDescs,
    uint32_t bondCount,
    const uint32_t* chunkReorderMap,
    bool keepBondNormalChunkOrder, 
    NvBlastLog logFn
)
{
    NVBLASTLL_CHECK(chunkCount == 0 || chunkDescs != nullptr, logFn, "NvBlastApplyAssetDescChunkReorderMap: NULL chunkDescs input with non-zero chunkCount", return);
    NVBLASTLL_CHECK(reorderedChunkDescs == nullptr || chunkCount != 0, logFn, "NvBlastApplyAssetDescChunkReorderMap: NULL reorderedChunkDescs input with non-zero chunkCount", return);
    NVBLASTLL_CHECK(chunkReorderMap == nullptr || chunkCount != 0, logFn, "NvBlastApplyAssetDescChunkReorderMap: NULL chunkReorderMap input with non-zero chunkCount", return);
    NVBLASTLL_CHECK(bondCount == 0 || bondDescs != nullptr, logFn, "NvBlastApplyAssetDescChunkReorderMap: NULL bondDescs input with non-zero bondCount", return);
    NVBLASTLL_CHECK(bondDescs == nullptr || chunkReorderMap != nullptr, logFn, "NvBlastApplyAssetDescChunkReorderMap: NULL bondDescs input with NULL chunkReorderMap", return);

    // Copy chunk descs 
    if (reorderedChunkDescs)
    {
        for (uint32_t i = 0; i < chunkCount; ++i)
        {
            reorderedChunkDescs[chunkReorderMap[i]] = chunkDescs[i];
            uint32_t& parentIndex = reorderedChunkDescs[chunkReorderMap[i]].parentChunkDescIndex;
            if (parentIndex < chunkCount)
            {
                parentIndex = chunkReorderMap[parentIndex]; // If the parent index is valid, remap it too to reflect the new order
            }
        }
    }

    if (bondDescs)
    {
        for (uint32_t i = 0; i < bondCount; ++i)
        {
            NvBlastBondDesc& bondDesc = bondDescs[i];
            uint32_t& index0 = bondDesc.chunkIndices[0];
            uint32_t& index1 = bondDesc.chunkIndices[1];
            const uint32_t newIndex0 = index0 < chunkCount ? chunkReorderMap[index0] : index0;
            const uint32_t newIndex1 = index1 < chunkCount ? chunkReorderMap[index1] : index1;
            if (keepBondNormalChunkOrder && (index0 < index1) != (newIndex0 < newIndex1))
            {
                VecMath::mul(bondDesc.bond.normal, -1);
            }
            index0 = newIndex0;
            index1 = newIndex1;
        }
    }
}


void NvBlastApplyAssetDescChunkReorderMapInPlace
(
    NvBlastChunkDesc* chunkDescs,
    uint32_t chunkCount,
    NvBlastBondDesc* bondDescs,
    uint32_t bondCount,
    const uint32_t* chunkReorderMap,
    bool keepBondNormalChunkOrder,
    void* scratch,
    NvBlastLog logFn
)
{
    NVBLASTLL_CHECK(chunkCount == 0 || chunkDescs != nullptr, logFn, "NvBlastApplyAssetDescChunkReorderMapInPlace: NULL chunkDescs input with non-zero chunkCount", return);
    NVBLASTLL_CHECK(chunkCount == 0 || scratch != nullptr, logFn, "NvBlastApplyAssetDescChunkReorderMapInPlace: NULL scratch input with non-zero chunkCount", return);

    NvBlastChunkDesc* chunksTemp = static_cast<NvBlastChunkDesc*>(scratch);
    memcpy(chunksTemp, chunkDescs, sizeof(NvBlastChunkDesc) * chunkCount);
    NvBlastApplyAssetDescChunkReorderMap(chunkDescs, chunksTemp, chunkCount, bondDescs, bondCount, chunkReorderMap, keepBondNormalChunkOrder, logFn);
}


bool NvBlastReorderAssetDescChunks
(
    NvBlastChunkDesc* chunkDescs,
    uint32_t chunkCount,
    NvBlastBondDesc* bondDescs,
    uint32_t bondCount,
    uint32_t* chunkReorderMap,
    bool keepBondNormalChunkOrder,
    void* scratch,
    NvBlastLog logFn
)
{
    if (!NvBlastBuildAssetDescChunkReorderMap(chunkReorderMap, chunkDescs, chunkCount, scratch, logFn))
    {
        NvBlastApplyAssetDescChunkReorderMapInPlace(chunkDescs, chunkCount, bondDescs, bondCount, chunkReorderMap, keepBondNormalChunkOrder, scratch, logFn);
        return false;
    }
    return true;
}


bool NvBlastEnsureAssetExactSupportCoverage(NvBlastChunkDesc* chunkDescs, uint32_t chunkCount, void* scratch, NvBlastLog logFn)
{
    NVBLASTLL_CHECK(chunkCount == 0 || chunkDescs != nullptr, logFn, "NvBlastEnsureAssetExactSupportCoverage: NULL chunkDescs input with non-zero chunkCount", return false);
    NVBLASTLL_CHECK(chunkCount == 0 || scratch != nullptr, logFn, "NvBlastEnsureAssetExactSupportCoverage: NULL scratch input with non-zero chunkCount", return false);

    uint32_t supportChunkCount;
    uint32_t leafChunkCount;
    return Asset::ensureExactSupportCoverage(supportChunkCount, leafChunkCount, static_cast<char*>(scratch), chunkCount, chunkDescs, false, logFn);
}

} // extern "C"

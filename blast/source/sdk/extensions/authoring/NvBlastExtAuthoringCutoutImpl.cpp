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


#include "NvBlastGlobals.h"
#include <NvBlastAssert.h>
#include "NvBounds3.h"
#include "NvMath.h"
#include "NvAssert.h"
#include <NvBlastNvSharedHelpers.h>
#include "NvBlastExtAuthoringCutoutImpl.h"
#include <algorithm>
#include <set>
#include <map>
#include <stack>

#define CUTOUT_DISTANCE_THRESHOLD   (0.7f)

#define CUTOUT_DISTANCE_EPS         (0.01f)

using namespace Nv::Blast;

// Unsigned modulus
 uint32_t mod(int32_t n, uint32_t modulus)
{
    const int32_t d = n/(int32_t)modulus;
    const int32_t m = n - d*(int32_t)modulus;
    return m >= 0 ? (uint32_t)m : (uint32_t)m + modulus;
}

 float square(float x)
{
    return x * x;
}

// 2D cross product
 float dotXY(const nvidia::NvVec3& v, const nvidia::NvVec3& w)
{
    return v.x * w.x + v.y * w.y;
}

// Z-component of cross product
 float crossZ(const nvidia::NvVec3& v, const nvidia::NvVec3& w)
{
    return v.x * w.y - v.y * w.x;
}

// z coordinates may be used to store extra info - only deal with x and y
 float perpendicularDistanceSquared(const nvidia::NvVec3& v0, const nvidia::NvVec3& v1, const nvidia::NvVec3& v2)
{
    const nvidia::NvVec3 base = v2 - v0;
    const nvidia::NvVec3 leg = v1 - v0;

    const float baseLen2 = dotXY(base, base);

    return baseLen2 > NV_EPS_F32 * dotXY(leg, leg) ? square(crossZ(base, leg)) / baseLen2 : 0.0f;
}

// z coordinates may be used to store extra info - only deal with x and y
 float perpendicularDistanceSquared(const std::vector< nvidia::NvVec3 >& cutout, uint32_t index)
{
    const uint32_t size = cutout.size();
    return perpendicularDistanceSquared(cutout[(index + size - 1) % size], cutout[index], cutout[(index + 1) % size]);
}

////////////////////////////////////////////////
// ApexShareUtils - Begin
////////////////////////////////////////////////

struct BoundsRep
{
    BoundsRep() : type(0)
    {
        aabb.setEmpty();
    }

    nvidia::NvBounds3    aabb;
    uint32_t            type;   // By default only reports if subtypes are the same, configurable.  Valid range {0...7}
};

struct IntPair
{
    void    set(int32_t _i0, int32_t _i1)
    {
        i0 = _i0;
        i1 = _i1;
    }

    int32_t i0, i1;

    static  int compare(const void* a, const void* b)
    {
        const int32_t diff0 = ((IntPair*)a)->i0 - ((IntPair*)b)->i0;
        return diff0 ? diff0 : (((IntPair*)a)->i1 - ((IntPair*)b)->i1);
    }
};

struct BoundsInteractions
{
    BoundsInteractions() : bits(0x8040201008040201ULL) {}
    BoundsInteractions(bool setAll) : bits(setAll ? 0xFFFFFFFFFFFFFFFFULL : 0x0000000000000000ULL) {}

    bool    set(unsigned group1, unsigned group2, bool interacts)
    {
        if (group1 >= 8 || group2 >= 8)
        {
            return false;
        }
        const uint64_t mask = (uint64_t)1 << ((group1 << 3) + group2) | (uint64_t)1 << ((group2 << 3) + group1);
        if (interacts)
        {
            bits |= mask;
        }
        else
        {
            bits &= ~mask;
        }
        return true;
    }

    uint64_t bits;
};

enum Bounds3Axes
{
    Bounds3X = 1,
    Bounds3Y = 2,
    Bounds3Z = 4,

    Bounds3XY = Bounds3X | Bounds3Y,
    Bounds3YZ = Bounds3Y | Bounds3Z,
    Bounds3ZX = Bounds3Z | Bounds3X,

    Bounds3XYZ = Bounds3X | Bounds3Y | Bounds3Z
};

void boundsCalculateOverlaps(std::vector<IntPair>& overlaps, Bounds3Axes axesToUse, const BoundsRep* bounds, uint32_t boundsCount, uint32_t boundsByteStride,
    const BoundsInteractions& interactions = BoundsInteractions(), bool append = false);

void createIndexStartLookup(std::vector<uint32_t>& lookup, int32_t indexBase, uint32_t indexRange, int32_t* indexSource, uint32_t indexCount, uint32_t indexByteStride);

/*
Index bank - double-sided free list for O(1) borrow/return of unique IDs

Type IndexType should be an unsigned integer type or something that can be cast to and from
an integer
*/
template <class IndexType>
class IndexBank
{
public:
    IndexBank<IndexType>(uint32_t capacity = 0) : indexCount(0), capacityLocked(false)
    {
        maxCapacity = calculateMaxCapacity();
        reserve_internal(capacity);
    }

    // Copy constructor
    IndexBank<IndexType>(const IndexBank<IndexType>& other)
    {
        *this = other;
    }

    virtual             ~IndexBank<IndexType>() {}

    // Assignment operator
    IndexBank<IndexType>& operator = (const IndexBank<IndexType>& other)
    {
        indices = other.indices;
        ranks = other.ranks;
        maxCapacity = other.maxCapacity;
        indexCount = other.indexCount;
        capacityLocked = other.capacityLocked;
        return *this;
    }

    void                setIndicesAndRanks(uint16_t* indicesIn, uint16_t* ranksIn, uint32_t capacityIn, uint32_t usedCountIn)
    {
        indexCount = usedCountIn;
        reserve_internal(capacityIn);
        for (uint32_t i = 0; i < capacityIn; ++i)
        {
            indices[i] = indicesIn[i];
            ranks[i] = ranksIn[i];
        }
    }

    void                clear(uint32_t capacity = 0, bool used = false)
    {
        capacityLocked = false;
        indices.reset();
        ranks.reset();
        reserve_internal(capacity);
        if (used)
        {
            indexCount = capacity;
            indices.resize(capacity);
            for (IndexType i = (IndexType)0; i < (IndexType)capacity; ++i)
            {
                indices[i] = i;
            }
        }
        else
        {
            indexCount = 0;
        }
    }

    // Equivalent to calling freeLastUsed() until the used list is empty.
    void                clearFast()
    {
        indexCount = 0;
    }

    // This is the reserve size.  The bank can only grow, due to shuffling of indices
    virtual void        reserve(uint32_t capacity)
    {
        reserve_internal(capacity);
    }

    // If lock = true, keeps bank from automatically resizing
    void                lockCapacity(bool lock)
    {
        capacityLocked = lock;
    }

    bool                isCapacityLocked() const
    {
        return capacityLocked;
    }

    void                setMaxCapacity(uint32_t inMaxCapacity)
    {
        // Cannot drop below current capacity, nor above max set by data types
        maxCapacity = nvidia::NvClamp(inMaxCapacity, capacity(), calculateMaxCapacity());
    }

    uint32_t        capacity() const
    {
        return indices.size();
    }
    uint32_t        usedCount() const
    {
        return indexCount;
    }
    uint32_t        freeCount() const
    {
        return capacity() - usedCount();
    }

    // valid from [0] to [size()-1]
    const IndexType*    usedIndices() const
    {
        return indices.data();
    }

    // valid from [0] to [free()-1]
    const IndexType*    freeIndices() const
    {
        return indices.begin() + usedCount();
    }

    bool                isValid(IndexType index) const
    {
        return index < (IndexType)capacity();
    }
    bool                isUsed(IndexType index) const
    {
        return isValid(index) && (ranks[index] < (IndexType)usedCount());
    }
    bool                isFree(IndexType index) const
    {
        return isValid(index) && !isUsed();
    }

    IndexType           getRank(IndexType index) const
    {
        return ranks[index];
    }

    // Gets the next available index, if any
    bool                useNextFree(IndexType& index)
    {
        if (freeCount() == 0)
        {
            if (capacityLocked)
            {
                return false;
            }
            if (capacity() >= maxCapacity)
            {
                return false;
            }
            reserve(nvidia::NvClamp(capacity() * 2, (uint32_t)1, maxCapacity));
            NVBLAST_ASSERT(freeCount() > 0);
        }
        index = indices[indexCount++];
        return true;
    }

    // Frees the last used index, if any
    bool                freeLastUsed(IndexType& index)
    {
        if (usedCount() == 0)
        {
            return false;
        }
        index = indices[--indexCount];
        return true;
    }

    // Requests a particular index.  If that index is available, it is borrowed and the function
    // returns true.  Otherwise nothing happens and the function returns false.
    bool                use(IndexType index)
    {
        if (!indexIsValidForUse(index))
        {
            return false;
        }
        IndexType oldRank;
        placeIndexAtRank(index, (IndexType)indexCount++, oldRank);
        return true;
    }

    bool                free(IndexType index)
    {
        if (!indexIsValidForFreeing(index))
        {
            return false;
        }
        IndexType oldRank;
        placeIndexAtRank(index, (IndexType)--indexCount, oldRank);
        return true;
    }

    bool                useAndReturnRanks(IndexType index, IndexType& newRank, IndexType& oldRank)
    {
        if (!indexIsValidForUse(index))
        {
            return false;
        }
        newRank = (IndexType)indexCount++;
        placeIndexAtRank(index, newRank, oldRank);
        return true;
    }

    bool                freeAndReturnRanks(IndexType index, IndexType& newRank, IndexType& oldRank)
    {
        if (!indexIsValidForFreeing(index))
        {
            return false;
        }
        newRank = (IndexType)--indexCount;
        placeIndexAtRank(index, newRank, oldRank);
        return true;
    }

protected:

    bool                indexIsValidForUse(IndexType index)
    {
        if (!isValid(index))
        {
            if (capacityLocked)
            {
                return false;
            }
            if (capacity() >= maxCapacity)
            {
                return false;
            }
            reserve(nvidia::NvClamp(2 * (uint32_t)index, (uint32_t)1, maxCapacity));
            NVBLAST_ASSERT(isValid(index));
        }
        return !isUsed(index);
    }

    bool                indexIsValidForFreeing(IndexType index)
    {
        if (!isValid(index))
        {
            // Invalid index
            return false;
        }
        return isUsed(index);
    }

    // This is the reserve size.  The bank can only grow, due to shuffling of indices
    void                reserve_internal(uint32_t capacity)
    {
        capacity = std::min(capacity, maxCapacity);
        const uint32_t oldCapacity = indices.size();
        if (capacity > oldCapacity)
        {
            indices.resize(capacity);
            ranks.resize(capacity);
            for (IndexType i = (IndexType)oldCapacity; i < (IndexType)capacity; ++i)
            {
                indices[i] = i;
                ranks[i] = i;
            }
        }
    }

private:

    void                placeIndexAtRank(IndexType index, IndexType newRank, IndexType& oldRank)    // returns old rank
    {
        const IndexType replacementIndex = indices[newRank];
        oldRank = ranks[index];
        indices[oldRank] = replacementIndex;
        indices[newRank] = index;
        ranks[replacementIndex] = oldRank;
        ranks[index] = newRank;
    }

    uint32_t                calculateMaxCapacity()
    {
#pragma warning(push)
#pragma warning(disable: 4127) // conditional expression is constant
        if (sizeof(IndexType) >= sizeof(uint32_t))
        {
            return 0xFFFFFFFF;  // Limited by data type we use to report capacity
        }
        else
        {
            return (1u << (8 * std::min((uint32_t)sizeof(IndexType), 3u))) - 1; // Limited by data type we use for indices
        }
#pragma warning(pop)
    }

protected:

    std::vector<IndexType>      indices;
    std::vector<IndexType>      ranks;
    uint32_t                    maxCapacity;
    uint32_t                    indexCount;
    bool                        capacityLocked;
};

struct Marker
{
    float   pos;
    uint32_t    id; // lsb = type (0 = max, 1 = min), other bits used for object index

    void    set(float _pos, int32_t _id)
    {
        pos = _pos;
        id = (uint32_t)_id;
    }
};

static int compareMarkers(const void* A, const void* B)
{
    // Sorts by value.  If values equal, sorts min types greater than max types, to reduce the # of overlaps
    const float delta = ((Marker*)A)->pos - ((Marker*)B)->pos;
    return delta != 0 ? (delta < 0 ? -1 : 1) : ((int)(((Marker*)A)->id & 1) - (int)(((Marker*)B)->id & 1));
}

void boundsCalculateOverlaps(std::vector<IntPair>& overlaps, Bounds3Axes axesToUse, const BoundsRep* bounds, uint32_t boundsCount, uint32_t boundsByteStride,
    const BoundsInteractions& interactions, bool append)
{
    if (!append)
    {
        overlaps.clear();
    }

    uint32_t D = 0;
    uint32_t axisNums[3];
    for (unsigned i = 0; i < 3; ++i)
    {
        if ((axesToUse >> i) & 1)
        {
            axisNums[D++] = i;
        }
    }

    if (D == 0 || D > 3)
    {
        return;
    }

    std::vector< std::vector<Marker> > axes;
    axes.resize(D);
    uint32_t overlapCount[3];

    for (uint32_t n = 0; n < D; ++n)
    {
        const uint32_t axisNum = axisNums[n];
        std::vector<Marker>& axis = axes[n];
        overlapCount[n] = 0;
        axis.resize(2 * boundsCount);
        uint8_t* boundsPtr = (uint8_t*)bounds;
        for (uint32_t i = 0; i < boundsCount; ++i, boundsPtr += boundsByteStride)
        {
            const BoundsRep& boundsRep = *(const BoundsRep*)boundsPtr;
            const nvidia::NvBounds3& box = boundsRep.aabb;
            float min = box.minimum[axisNum];
            float max = box.maximum[axisNum];
            if (min >= max)
            {
                const float mid = 0.5f * (min + max);
                float pad = 0.000001f * fabsf(mid);
                min = mid - pad;
                max = mid + pad;
            }
            axis[i << 1].set(min, (int32_t)i << 1 | 1);
            axis[i << 1 | 1].set(max, (int32_t)i << 1);
        }
        qsort(axis.data(), axis.size(), sizeof(Marker), compareMarkers);
        uint32_t localOverlapCount = 0;
        for (uint32_t i = 0; i < axis.size(); ++i)
        {
            Marker& marker = axis[i];
            if (marker.id & 1)
            {
                overlapCount[n] += localOverlapCount;
                ++localOverlapCount;
            }
            else
            {
                --localOverlapCount;
            }
        }
    }

    unsigned int axis0;
    unsigned int axis1;
    unsigned int axis2;
    unsigned int maxBin;
    if (D == 1)
    {
        maxBin = 0;
        axis0 = axisNums[0];
        axis1 = axis0;
        axis2 = axis0;
    }
    else if (D == 2)
    {
        if (overlapCount[0] < overlapCount[1])
        {
            maxBin = 0;
            axis0 = axisNums[0];
            axis1 = axisNums[1];
            axis2 = axis0;
        }
        else
        {
            maxBin = 1;
            axis0 = axisNums[1];
            axis1 = axisNums[0];
            axis2 = axis0;
        }
    }
    else
    {
        maxBin = overlapCount[0] < overlapCount[1] ? (overlapCount[0] < overlapCount[2] ? 0U : 2U) : (overlapCount[1] < overlapCount[2] ? 1U : 2U);
        axis0 = axisNums[maxBin];
        axis1 = (axis0 + 1) % 3;
        axis2 = (axis0 + 2) % 3;
    }

    const uint64_t interactionBits = interactions.bits;

    IndexBank<uint32_t> localOverlaps(boundsCount);
    std::vector<Marker>& axis = axes[maxBin];
    float boxMin1 = 0.0f;
    float boxMax1 = 0.0f;
    float boxMin2 = 0.0f;
    float boxMax2 = 0.0f;

    for (uint32_t i = 0; i < axis.size(); ++i)
    {
        Marker& marker = axis[i];
        const uint32_t index = marker.id >> 1;
        if (marker.id & 1)
        {
            const BoundsRep& boundsRep = *(const BoundsRep*)((uint8_t*)bounds + index*boundsByteStride);
            const uint8_t interaction = (uint8_t)((interactionBits >> (boundsRep.type << 3)) & 0xFF);
            const nvidia::NvBounds3& box = boundsRep.aabb;
            // These conditionals compile out with optimization:
            if (D > 1)
            {
                boxMin1 = box.minimum[axis1];
                boxMax1 = box.maximum[axis1];
                if (D == 3)
                {
                    boxMin2 = box.minimum[axis2];
                    boxMax2 = box.maximum[axis2];
                }
            }
            const uint32_t localOverlapCount = localOverlaps.usedCount();
            const uint32_t* localOverlapIndices = localOverlaps.usedIndices();
            for (uint32_t j = 0; j < localOverlapCount; ++j)
            {
                const uint32_t overlapIndex = localOverlapIndices[j];
                const BoundsRep& overlapBoundsRep = *(const BoundsRep*)((uint8_t*)bounds + overlapIndex*boundsByteStride);
                if ((interaction >> overlapBoundsRep.type) & 1)
                {
                    const nvidia::NvBounds3& overlapBox = overlapBoundsRep.aabb;
                    // These conditionals compile out with optimization:
                    if (D > 1)
                    {
                        if (boxMin1 >= overlapBox.maximum[axis1] || boxMax1 <= overlapBox.minimum[axis1])
                        {
                            continue;
                        }
                        if (D == 3)
                        {
                            if (boxMin2 >= overlapBox.maximum[axis2] || boxMax2 <= overlapBox.minimum[axis2])
                            {
                                continue;
                            }
                        }
                    }
                    // Add overlap
                    IntPair pair;
                    pair.i0 = (int32_t)index;
                    pair.i1 = (int32_t)overlapIndex;
                    overlaps.push_back(pair);
                }
            }
            NVBLAST_ASSERT(localOverlaps.isValid(index));
            NVBLAST_ASSERT(!localOverlaps.isUsed(index));
            localOverlaps.use(index);
        }
        else
        {
            // Remove local overlap
            NVBLAST_ASSERT(localOverlaps.isValid(index));
            localOverlaps.free(index);
        }
    }
}

void createIndexStartLookup(std::vector<uint32_t>& lookup, int32_t indexBase, uint32_t indexRange, int32_t* indexSource, uint32_t indexCount, uint32_t indexByteStride)
{
    if (indexRange == 0)
    {
        lookup.resize(std::max(indexRange + 1, 2u));
        lookup[0] = 0;
        lookup[1] = indexCount;
    }
    else
    {
        lookup.resize(indexRange + 1);
        uint32_t indexPos = 0;
        for (uint32_t i = 0; i < indexRange; ++i)
        {
            for (; indexPos < indexCount; ++indexPos, indexSource = (int32_t*)((uintptr_t)indexSource + indexByteStride))
            {
                if (*indexSource >= (int32_t)i + indexBase)
                {
                    lookup[i] = indexPos;
                    break;
                }
            }
            if (indexPos == indexCount)
            {
                lookup[i] = indexPos;
            }
        }
        lookup[indexRange] = indexCount;
    }
}

////////////////////////////////////////////////
// ApexShareUtils - End
////////////////////////////////////////////////

struct CutoutVert
{
    int32_t cutoutIndex;
    int32_t vertIndex;

    void    set(int32_t _cutoutIndex, int32_t _vertIndex)
    {
        cutoutIndex = _cutoutIndex;
        vertIndex = _vertIndex;
    }
};

struct NewVertex
{
    CutoutVert  vertex;
    float       edgeProj;
};

static int compareNewVertices(const void* a, const void* b)
{
    const int32_t cutoutDiff = ((NewVertex*)a)->vertex.cutoutIndex - ((NewVertex*)b)->vertex.cutoutIndex;
    if (cutoutDiff)
    {
        return cutoutDiff;
    }
    const int32_t vertDiff = ((NewVertex*)a)->vertex.vertIndex - ((NewVertex*)b)->vertex.vertIndex;
    if (vertDiff)
    {
        return vertDiff;
    }
    const float projDiff = ((NewVertex*)a)->edgeProj - ((NewVertex*)b)->edgeProj;
    return projDiff ? (projDiff < 0.0f ? -1 : 1) : 0;
}

template<typename T>
class Map2d
{
public:
    Map2d(uint32_t width, uint32_t height)
    {
        create_internal(width, height, NULL);
    }
    Map2d(uint32_t width, uint32_t height, T fillValue)
    {
        create_internal(width, height, &fillValue);
    }
    Map2d(const Map2d& map)
    {
        *this = map;
    }

    Map2d&      operator = (const Map2d& map)
    {
        mMem.clear();
        create_internal(map.mWidth, map.mHeight, NULL);
        return *this;
    }

    void        create(uint32_t width, uint32_t height)
    {
        return create_internal(width, height, NULL);
    }
    void        create(uint32_t width, uint32_t height, T fillValue)
    {
        create_internal(width, height, &fillValue);
    }

    //void      clear(const T value)
    //{
    //  for (auto it = mMem.begin(); it != mMem.end(); it++)
    //  {
    //      for (auto it2 = it->begin(); it2 != it->end(); it2++)
    //      {
    //          *it2 = value;
    //      }
    //  }
    //}

    void        setOrigin(uint32_t x, uint32_t y)
    {
        mOriginX = x;
        mOriginY = y;
    }

    const T&    operator()(int32_t x, int32_t y) const
    {
        x = (int32_t)mod(x+(int32_t)mOriginX, mWidth);
        y = (int32_t)mod(y+(int32_t)mOriginY, mHeight);
        return mMem[y][x];
    }
    T&          operator()(int32_t x, int32_t y)
    {
        x = (int32_t)mod(x+(int32_t)mOriginX, mWidth);
        y = (int32_t)mod(y+(int32_t)mOriginY, mHeight);
        return mMem[y][x];
    }

private:

    void    create_internal(uint32_t width, uint32_t height, T* val)
    {
        mMem.clear();
        mWidth = width;
        mHeight = height;
        mMem.resize(mHeight);
        for (auto it = mMem.begin(); it != mMem.end(); it++)
        {
            it->resize(mWidth, val ? *val : 0);
        }
        mOriginX = 0;
        mOriginY = 0;
    }

    std::vector<std::vector<T>> mMem;
    uint32_t    mWidth;
    uint32_t    mHeight;
    uint32_t    mOriginX;
    uint32_t    mOriginY;
};

class BitMap
{
public:
    BitMap() : mMem(NULL) {}
    BitMap(uint32_t width, uint32_t height) : mMem(NULL)
    {
        create_internal(width, height, NULL);
    }
    BitMap(uint32_t width, uint32_t height, bool fillValue) : mMem(NULL)
    {
        create_internal(width, height, &fillValue);
    }
    BitMap(const BitMap& map)
    {
        *this = map;
    }
    ~BitMap()
    {
        delete [] mMem;
    }

    BitMap& operator = (const BitMap& map)
    {
        delete [] mMem;
        mMem = NULL;
        if (map.mMem)
        {
            create_internal(map.mWidth, map.mHeight, NULL);
            memcpy(mMem, map.mMem, mHeight * mRowBytes);
        }
        return *this;
    }

    void    create(uint32_t width, uint32_t height)
    {
        return create_internal(width, height, NULL);
    }
    void    create(uint32_t width, uint32_t height, bool fillValue)
    {
        create_internal(width, height, &fillValue);
    }

    void    clear(bool value)
    {
        memset(mMem, value ? 0xFF : 0x00, mRowBytes * mHeight);
    }

    void    setOrigin(uint32_t x, uint32_t y)
    {
        mOriginX = x;
        mOriginY = y;
    }

    bool    read(int32_t x, int32_t y) const
    {
        x = (int32_t)mod(x+(int32_t)mOriginX, mWidth);
        y = (int32_t)mod(y+(int32_t)mOriginY, mHeight);
        return ((mMem[(x >> 3) + y * mRowBytes] >> (x & 7)) & 1) != 0;
    }
    void    set(int32_t x, int32_t y)
    {
        x = (int32_t)mod(x+(int32_t)mOriginX, mWidth);
        y = (int32_t)mod(y+(int32_t)mOriginY, mHeight);
        mMem[(x >> 3) + y * mRowBytes] |= 1 << (x & 7);
    }
    void    reset(int32_t x, int32_t y)
    {
        x = (int32_t)mod(x+(int32_t)mOriginX, mWidth);
        y = (int32_t)mod(y+(int32_t)mOriginY, mHeight);
        mMem[(x >> 3) + y * mRowBytes] &= ~(1 << (x & 7));
    }

private:

    void    create_internal(uint32_t width, uint32_t height, bool* val)
    {
        delete [] mMem;
        mRowBytes = (width + 7) >> 3;
        const uint32_t bytes = mRowBytes * height;
        if (bytes == 0)
        {
            mWidth = mHeight = 0;
            mMem = NULL;
            return;
        }
        mWidth = width;
        mHeight = height;
        mMem = new uint8_t[bytes];
        mOriginX = 0;
        mOriginY = 0;
        if (val)
        {
            clear(*val);
        }
    }

    uint8_t*    mMem;
    uint32_t    mWidth;
    uint32_t    mHeight;
    uint32_t    mRowBytes;
    uint32_t    mOriginX;
    uint32_t    mOriginY;
};


 int32_t taxicabSine(int32_t i)
{
    // 0 1 1 1 0 -1 -1 -1
    return (int32_t)((0x01A9 >> ((i & 7) << 1)) & 3) - 1;
}

// Only looks at x and y components
 bool directionsXYOrderedCCW(const nvidia::NvVec3& d0, const nvidia::NvVec3& d1, const nvidia::NvVec3& d2)
{
    const bool ccw02 = crossZ(d0, d2) > 0.0f;
    const bool ccw01 = crossZ(d0, d1) > 0.0f;
    const bool ccw21 = crossZ(d2, d1) > 0.0f;
    return ccw02 ? ccw01 && ccw21 : ccw01 || ccw21;
}

 std::pair<float, float> compareTraceSegmentToLineSegment(const std::vector<POINT2D>& trace, int _start, int delta, float distThreshold, uint32_t width, uint32_t height, bool hasBorder)
{
    if (delta < 2)
    {
        return std::make_pair(0.0f, 0.0f);
    }

    const uint32_t size = trace.size();

    uint32_t start = (uint32_t)_start, end = (uint32_t)(_start + delta) % size;

    const bool startIsOnBorder = hasBorder && (trace[start].x == -1 || trace[start].x == (int)width || trace[start].y == -1 || trace[start].y == (int)height);
    const bool endIsOnBorder = hasBorder && (trace[end].x == -1 || trace[end].x == (int)width || trace[end].y == -1 || trace[end].y == (int)height);

    if (startIsOnBorder || endIsOnBorder)
    {
        if ((trace[start].x == -1 && trace[end].x == -1) ||
                (trace[start].y == -1 && trace[end].y == -1) ||
                (trace[start].x == (int)width && trace[end].x == (int)width) ||
                (trace[start].y == (int)height && trace[end].y == (int)height))
        {
            return std::make_pair(0.0f, 0.0f);
        }
        return std::make_pair(NV_MAX_F32, NV_MAX_F32);
    }

    nvidia::NvVec3 orig((float)trace[start].x, (float)trace[start].y, 0);
    nvidia::NvVec3 dest((float)trace[end].x, (float)trace[end].y, 0);
    nvidia::NvVec3 dir = dest - orig;

    dir.normalize();

    float aveError = 0.0f;
    float aveError2 = 0.0f;

    for (;;)
    {
        if (++start >= size)
        {
            start = 0;
        }
        if (start == end)
        {
            break;
        }
        nvidia::NvVec3 testDisp((float)trace[start].x, (float)trace[start].y, 0);
        testDisp -= orig;
        aveError += (float)(nvidia::NvAbs(testDisp.x * dir.y - testDisp.y * dir.x) >= distThreshold);
        aveError2 += nvidia::NvAbs(testDisp.x * dir.y - testDisp.y * dir.x);
    }

    aveError /= delta - 1;
    aveError2 /= delta - 1;

    return std::make_pair(aveError, aveError2);
}

// Segment i starts at vi and ends at vi+ei
// Tests for overlap in segments' projection onto xy plane
// Returns distance between line segments.  (Negative value indicates overlap.)
 float segmentsIntersectXY(const nvidia::NvVec3& v0, const nvidia::NvVec3& e0, const nvidia::NvVec3& v1, const nvidia::NvVec3& e1)
{
    const nvidia::NvVec3 dv = v1 - v0;

    nvidia::NvVec3 d0 = e0;
    d0.normalize();
    nvidia::NvVec3 d1 = e1;
    d1.normalize();

    const float c10 = crossZ(dv, d0);
    const float d10 = crossZ(e1, d0);

    float a1 = nvidia::NvAbs(c10);
    float b1 = nvidia::NvAbs(c10 + d10);

    if (c10 * (c10 + d10) < 0.0f)
    {
        if (a1 < b1)
        {
            a1 = -a1;
        }
        else
        {
            b1 = -b1;
        }
    }

    const float c01 = crossZ(d1, dv);
    const float d01 = crossZ(e0, d1);

    float a2 = nvidia::NvAbs(c01);
    float b2 = nvidia::NvAbs(c01 + d01);

    if (c01 * (c01 + d01) < 0.0f)
    {
        if (a2 < b2)
        {
            a2 = -a2;
        }
        else
        {
            b2 = -b2;
        }
    }

    return nvidia::NvMax(nvidia::NvMin(a1, b1), nvidia::NvMin(a2, b2));
}

// If point projects onto segment, returns true and proj is set to a
// value in the range [0,1], indicating where along the segment (from v0 to v1)
// the projection lies, and dist2 is set to the distance squared from point to
// the line segment.  Otherwise, returns false.
// Note, if v1 = v0, then the function returns true with proj = 0.
 bool projectOntoSegmentXY(float& proj, float& dist2, const nvidia::NvVec3& point, const nvidia::NvVec3& v0, const nvidia::NvVec3& v1, float margin)
{
    const nvidia::NvVec3 seg = v1 - v0;
    const nvidia::NvVec3 x = point - v0;
    const float seg2 = dotXY(seg, seg);
    const float d = dotXY(x, seg);

    if (d < 0.0f || d > seg2)
    {
        return false;
    }

    const float margin2 = margin * margin;

    const float p = seg2 > 0.0f ? d / seg2 : 0.0f;
    const float lineDist2 = d * p;

    if (lineDist2 < margin2)
    {
        return false;
    }

    const float pPrime = 1.0f - p;
    const float dPrime = seg2 - d;
    const float lineDistPrime2 = dPrime * pPrime;

    if (lineDistPrime2 < margin2)
    {
        return false;
    }

    proj = p;
    dist2 = dotXY(x, x) - lineDist2;
    return true;
}

 bool isOnBorder(const nvidia::NvVec3& v, uint32_t width, uint32_t height)
{
    return v.x < -0.5f || v.x >= width - 0.5f || v.y < -0.5f || v.y >= height - 0.5f;
}

static void createCutout(Nv::Blast::Cutout& cutout, const std::vector<POINT2D>& trace, float segmentationErrorThreshold, float snapThreshold, uint32_t width, uint32_t height, bool hasBorder)
{
    cutout.vertices.clear();
    cutout.smoothingGroups.clear();

    std::vector<int> smoothingGroups;

    const uint32_t traceSize = trace.size();

    if (traceSize == 0)
    {
        return; // Nothing to do
    }

    uint32_t size = traceSize;

    std::vector<int> vertexIndices;

    const float pixelCenterOffset = hasBorder ? 0.5f : 0.0f;

    // Find best segment
    uint32_t start = 0;
    uint32_t delta = 0;
    float err2 = 0.f;
    for (uint32_t iStart = 0; iStart < size; ++iStart)
    {
        uint32_t iDelta = (size >> 1) + (size & 1);
        for (; iDelta > 1; --iDelta)
        {
            auto fit = compareTraceSegmentToLineSegment(trace, (int32_t)iStart, (int32_t)iDelta, CUTOUT_DISTANCE_THRESHOLD, width, height, hasBorder);
            if (fit.first < segmentationErrorThreshold)
            {
                err2 = fit.second;
                break;
            }
        }
        if (iDelta > delta)
        {
            start = iStart;
            delta = iDelta;
        }
    }
    if (err2 < segmentationErrorThreshold)
    {
        smoothingGroups.push_back(cutout.vertices.size());
    }
    cutout.vertices.push_back(nvidia::NvVec3((float)trace[start].x + pixelCenterOffset, (float)trace[start].y + pixelCenterOffset, 0));

    // Now complete the loop
    while ((size -= delta) > 0)
    {
        start = (start + delta) % traceSize;
        cutout.vertices.push_back(nvidia::NvVec3((float)trace[start].x + pixelCenterOffset, (float)trace[start].y + pixelCenterOffset, 0));
        if (size == 1)
        {
            delta = 1;
            break;
        }
        bool sg = true;
        for (delta = size - 1; delta > 1; --delta)
        {
            auto fit = compareTraceSegmentToLineSegment(trace, (int32_t)start, (int32_t)delta, CUTOUT_DISTANCE_THRESHOLD, width, height, hasBorder);
            if (fit.first < segmentationErrorThreshold)
            {
                if (fit.second > segmentationErrorThreshold)
                {
                    sg = false;
                }
                break;
            }
        }
        if (sg)
        {
            smoothingGroups.push_back(cutout.vertices.size());
        }
    }

    const float snapThresh2 = square(snapThreshold);

    // Use the snapThreshold to clean up
    while ((size = cutout.vertices.size()) >= 4)
    {
        bool reduced = false;
        for (uint32_t i = 0; i < size; ++i)
        {
            const uint32_t i1 = (i + 1) % size;
            const uint32_t i2 = (i + 2) % size;
            const uint32_t i3 = (i + 3) % size;
            nvidia::NvVec3& v0 = cutout.vertices[i];
            nvidia::NvVec3& v1 = cutout.vertices[i1];
            nvidia::NvVec3& v2 = cutout.vertices[i2];
            nvidia::NvVec3& v3 = cutout.vertices[i3];
            const nvidia::NvVec3 d0 = v1 - v0;
            const nvidia::NvVec3 d1 = v2 - v1;
            const nvidia::NvVec3 d2 = v3 - v2;
            const float den = crossZ(d0, d2);
            if (den != 0)
            {
                const float recipDen = 1.0f / den;
                const float s0 = crossZ(d1, d2) * recipDen;
                const float s2 = crossZ(d0, d1) * recipDen;
                if (s0 >= 0 || s2 >= 0)
                {
                    if (d0.magnitudeSquared()*s0* s0 <= snapThresh2 && d2.magnitudeSquared()*s2* s2 <= snapThresh2)
                    {
                        v1 += d0 * s0;

                        //uint32_t index = (uint32_t)(&v2 - cutout.vertices.begin());
                        int dist = std::distance(cutout.vertices.data(), &v2);
                        cutout.vertices.erase(cutout.vertices.begin() + dist);

                        for (auto& idx : smoothingGroups)
                        {
                            if (idx > dist)
                            {
                                idx--;
                            }
                        }
                        reduced = true;
                        break;

                    }
                }
            }
        }
        if (!reduced)
        {
            break;
        }
    }

    for (size_t i = 0; i < smoothingGroups.size(); i++)
    {
        if (i > 0 && smoothingGroups[i] == smoothingGroups[i - 1])
        {
            continue;
        }
        if (smoothingGroups[i] < static_cast<int>(cutout.vertices.size()))
        {
            cutout.smoothingGroups.push_back(cutout.vertices[smoothingGroups[i]]);
        }
    }
}

static void splitTJunctions(Nv::Blast::CutoutSetImpl& cutoutSet, float threshold)
{
    // Set bounds reps
    std::vector<BoundsRep> bounds;
    std::vector<CutoutVert> cutoutMap;  // maps bounds # -> ( cutout #, vertex # ).
    std::vector<IntPair> overlaps;

    const float distThreshold2 = threshold * threshold;

    // Split T-junctions
    uint32_t edgeCount = 0;
    for (uint32_t i = 0; i < cutoutSet.cutoutLoops.size(); ++i)
    {
        edgeCount += cutoutSet.cutoutLoops[i].vertices.size();
    }

    bounds.resize(edgeCount);
    cutoutMap.resize(edgeCount);

    edgeCount = 0;
    for (uint32_t i = 0; i < cutoutSet.cutoutLoops.size(); ++i)
    {
        Nv::Blast::Cutout& cutout = cutoutSet.cutoutLoops[i];
        const uint32_t cutoutSize = cutout.vertices.size();
        for (uint32_t j = 0; j < cutoutSize; ++j)
        {
            bounds[edgeCount].aabb.include(cutout.vertices[j]);
            bounds[edgeCount].aabb.include(cutout.vertices[(j + 1) % cutoutSize]);
            NVBLAST_ASSERT(!bounds[edgeCount].aabb.isEmpty());
            bounds[edgeCount].aabb.fattenFast(threshold);
            cutoutMap[edgeCount].set((int32_t)i, (int32_t)j);
            ++edgeCount;
        }
    }

    // Find bounds overlaps
    if (bounds.size() > 0)
    {
        boundsCalculateOverlaps(overlaps, Bounds3XY, &bounds[0], bounds.size(), sizeof(bounds[0]));
    }

    std::vector<NewVertex> newVertices;
    for (uint32_t overlapIndex = 0; overlapIndex < overlaps.size(); ++overlapIndex)
    {
        const IntPair& mapPair = overlaps[overlapIndex];
        const CutoutVert& seg0Map = cutoutMap[(uint32_t)mapPair.i0];
        const CutoutVert& seg1Map = cutoutMap[(uint32_t)mapPair.i1];

        if (seg0Map.cutoutIndex == seg1Map.cutoutIndex)
        {
            // Only split based on vertex/segment junctions from different cutouts
            continue;
        }

        NewVertex newVertex;
        float dist2 = 0;

        const Nv::Blast::Cutout& cutout0 = cutoutSet.cutoutLoops[(uint32_t)seg0Map.cutoutIndex];
        const uint32_t cutoutSize0 = cutout0.vertices.size();
        const Nv::Blast::Cutout& cutout1 = cutoutSet.cutoutLoops[(uint32_t)seg1Map.cutoutIndex];
        const uint32_t cutoutSize1 = cutout1.vertices.size();

        if (projectOntoSegmentXY(newVertex.edgeProj, dist2, cutout0.vertices[(uint32_t)seg0Map.vertIndex], cutout1.vertices[(uint32_t)seg1Map.vertIndex], 
            cutout1.vertices[(uint32_t)(seg1Map.vertIndex + 1) % cutoutSize1], 0.25f))
        {
            if (dist2 <= distThreshold2)
            {
                newVertex.vertex = seg1Map;
                newVertices.push_back(newVertex);
            }
        }

        if (projectOntoSegmentXY(newVertex.edgeProj, dist2, cutout1.vertices[(uint32_t)seg1Map.vertIndex], cutout0.vertices[(uint32_t)seg0Map.vertIndex], 
            cutout0.vertices[(uint32_t)(seg0Map.vertIndex + 1) % cutoutSize0], 0.25f))
        {
            if (dist2 <= distThreshold2)
            {
                newVertex.vertex = seg0Map;
                newVertices.push_back(newVertex);
            }
        }
    }

    if (newVertices.size())
    {
        // Sort new vertices
        qsort(newVertices.data(), newVertices.size(), sizeof(NewVertex), compareNewVertices);

        // Insert new vertices
        uint32_t lastCutoutIndex = 0xFFFFFFFF;
        uint32_t lastVertexIndex = 0xFFFFFFFF;
        float lastProj = 1.0f;
        for (uint32_t newVertexIndex = newVertices.size(); newVertexIndex--;)
        {
            const NewVertex& newVertex = newVertices[newVertexIndex];
            if (newVertex.vertex.cutoutIndex != (int32_t)lastCutoutIndex)
            {
                lastCutoutIndex = (uint32_t)newVertex.vertex.cutoutIndex;
                lastVertexIndex = 0xFFFFFFFF;
            }
            if (newVertex.vertex.vertIndex != (int32_t)lastVertexIndex)
            {
                lastVertexIndex = (uint32_t)newVertex.vertex.vertIndex;
                lastProj = 1.0f;
            }
            Nv::Blast::Cutout& cutout = cutoutSet.cutoutLoops[(uint32_t)newVertex.vertex.cutoutIndex];
            const float proj = lastProj > 0.0f ? newVertex.edgeProj / lastProj : 0.0f;
            const nvidia::NvVec3 pos = (1.0f - proj) * cutout.vertices[(uint32_t)newVertex.vertex.vertIndex] 
                + proj * cutout.vertices[(uint32_t)(newVertex.vertex.vertIndex + 1) % cutout.vertices.size()];
            cutout.vertices.push_back(nvidia::NvVec3());
            for (uint32_t n = cutout.vertices.size(); --n > (uint32_t)newVertex.vertex.vertIndex + 1;)
            {
                cutout.vertices[n] = cutout.vertices[n - 1];
            }
            cutout.vertices[(uint32_t)newVertex.vertex.vertIndex + 1] = pos;
            lastProj = newVertex.edgeProj;
        }
    }
}


static void mergeVertices(Nv::Blast::CutoutSetImpl& cutoutSet, float threshold, uint32_t width, uint32_t height)
{
    // Set bounds reps
    uint32_t vertexCount = 0;
    for (uint32_t i = 0; i < cutoutSet.cutoutLoops.size(); ++i)
    {
        vertexCount += cutoutSet.cutoutLoops[i].vertices.size();
    }

    std::vector<BoundsRep> bounds;
    std::vector<CutoutVert> cutoutMap;  // maps bounds # -> ( cutout #, vertex # ).
    bounds.resize(vertexCount);
    cutoutMap.resize(vertexCount);

    vertexCount = 0;
    for (uint32_t i = 0; i < cutoutSet.cutoutLoops.size(); ++i)
    {
        Nv::Blast::Cutout& cutout = cutoutSet.cutoutLoops[i];
        for (uint32_t j = 0; j < cutout.vertices.size(); ++j)
        {
            nvidia::NvVec3& vertex = cutout.vertices[j];
            nvidia::NvVec3 min(vertex.x - threshold, vertex.y - threshold, 0.0f);
            nvidia::NvVec3 max(vertex.x + threshold, vertex.y + threshold, 0.0f);
            bounds[vertexCount].aabb = nvidia::NvBounds3(min, max);
            cutoutMap[vertexCount].set((int32_t)i, (int32_t)j);
            ++vertexCount;
        }
    }

    // Find bounds overlaps
    std::vector<IntPair> overlaps;
    if (bounds.size() > 0)
    {
        boundsCalculateOverlaps(overlaps, Bounds3XY, &bounds[0], bounds.size(), sizeof(bounds[0]));
    }
    uint32_t overlapCount = overlaps.size();

    if (overlapCount == 0)
    {
        return;
    }

    // Sort by first index
    qsort(overlaps.data(), overlapCount, sizeof(IntPair), IntPair::compare);

    const float threshold2 = threshold * threshold;

    std::vector<IntPair> pairs;

    // Group by first index
    std::vector<uint32_t> lookup;
    createIndexStartLookup(lookup, 0, vertexCount, &overlaps.begin()->i0, overlapCount, sizeof(IntPair));
    for (uint32_t i = 0; i < vertexCount; ++i)
    {
        const uint32_t start = lookup[i];
        const uint32_t stop = lookup[i + 1];
        if (start == stop)
        {
            continue;
        }
        const CutoutVert& cutoutVert0 = cutoutMap[(uint32_t)overlaps[start].i0];
        const nvidia::NvVec3& vert0 = cutoutSet.cutoutLoops[(uint32_t)cutoutVert0.cutoutIndex].vertices[(uint32_t)cutoutVert0.vertIndex];
        const bool isOnBorder0 = !cutoutSet.periodic && isOnBorder(vert0, width, height);
        for (uint32_t j = start; j < stop; ++j)
        {
            const CutoutVert& cutoutVert1 = cutoutMap[(uint32_t)overlaps[j].i1];
            if (cutoutVert0.cutoutIndex == cutoutVert1.cutoutIndex)
            {
                // No pairs from the same cutout
                continue;
            }
            const nvidia::NvVec3& vert1 = cutoutSet.cutoutLoops[(uint32_t)cutoutVert1.cutoutIndex].vertices[(uint32_t)cutoutVert1.vertIndex];
            const bool isOnBorder1 = !cutoutSet.periodic && isOnBorder(vert1, width, height);
            if (isOnBorder0 != isOnBorder1)
            {
                // No border/non-border pairs
                continue;
            }
            if ((vert0 - vert1).magnitudeSquared() > threshold2)
            {
                // Distance outside threshold
                continue;
            }
            // A keeper.  Keep a symmetric list
            IntPair overlap = overlaps[j];
            pairs.push_back(overlap);
            const int32_t i0 = overlap.i0;
            overlap.i0 = overlap.i1;
            overlap.i1 = i0;
            pairs.push_back(overlap);
        }
    }
    
    if (pairs.size() == 0)
    {
        return;
    }

    // Sort by first index
    qsort(pairs.data(), pairs.size(), sizeof(IntPair), IntPair::compare);

    // For every vertex, only keep closest neighbor from each cutout
    createIndexStartLookup(lookup, 0, vertexCount, &pairs.begin()->i0, pairs.size(), sizeof(IntPair));
    for (uint32_t i = 0; i < vertexCount; ++i)
    {
        const uint32_t start = lookup[i];
        const uint32_t stop = lookup[i + 1];
        if (start == stop)
        {
            continue;
        }
        const CutoutVert& cutoutVert0 = cutoutMap[(uint32_t)pairs[start].i0];
        const nvidia::NvVec3& vert0 = cutoutSet.cutoutLoops[(uint32_t)cutoutVert0.cutoutIndex].vertices[(uint32_t)cutoutVert0.vertIndex];
        uint32_t groupStart = start;
        while (groupStart < stop)
        {
            uint32_t next = groupStart;
            const CutoutVert& cutoutVert1 = cutoutMap[(uint32_t)pairs[next].i1];
            int32_t currentOtherCutoutIndex = cutoutVert1.cutoutIndex;
            const nvidia::NvVec3& vert1 = cutoutSet.cutoutLoops[(uint32_t)currentOtherCutoutIndex].vertices[(uint32_t)cutoutVert1.vertIndex];
            uint32_t keep = groupStart;
            float minDist2 = (vert0 - vert1).magnitudeSquared();
            while (++next < stop)
            {
                const CutoutVert& cutoutVertNext = cutoutMap[(uint32_t)pairs[next].i1];
                if (currentOtherCutoutIndex != cutoutVertNext.cutoutIndex)
                {
                    break;
                }
                const nvidia::NvVec3& vertNext = cutoutSet.cutoutLoops[(uint32_t)cutoutVertNext.cutoutIndex].vertices[(uint32_t)cutoutVertNext.vertIndex];
                const float dist2 = (vert0 - vertNext).magnitudeSquared();
                if (dist2 < minDist2)
                {
                    pairs[keep].set(-1, -1);    // Invalidate
                    keep = next;
                    minDist2 = dist2;
                }
                else
                {
                    pairs[next].set(-1, -1);    // Invalidate
                }
            }
            groupStart = next;
        }
    }

    // Eliminate invalid pairs (compactify)
    uint32_t pairCount = 0;
    for (uint32_t i = 0; i < pairs.size(); ++i)
    {
        if (pairs[i].i0 >= 0 && pairs[i].i1 >= 0)
        {
            pairs[pairCount++] = pairs[i];
        }
    }
    pairs.resize(pairCount);

    // Snap points together
    std::vector<bool> pinned(vertexCount, false);

    for (uint32_t i = 0; i < pairCount; ++i)
    {
        const uint32_t i0 = (uint32_t)pairs[i].i0;
        if (pinned[i0])
        {
            continue;
        }
        const CutoutVert& cutoutVert0 = cutoutMap[i0];
        nvidia::NvVec3& vert0 = cutoutSet.cutoutLoops[(uint32_t)cutoutVert0.cutoutIndex].vertices[(uint32_t)cutoutVert0.vertIndex];
        const uint32_t i1 = (uint32_t)pairs[i].i1;
        const CutoutVert& cutoutVert1 = cutoutMap[i1];
        nvidia::NvVec3& vert1 = cutoutSet.cutoutLoops[(uint32_t)cutoutVert1.cutoutIndex].vertices[(uint32_t)cutoutVert1.vertIndex];
        const nvidia::NvVec3 disp = vert1 - vert0;
        // Move and pin
        pinned[i0] = true;
        if (pinned[i1])
        {
            vert0 = vert1;
        }
        else
        {
            vert0 += 0.5f * disp;
            vert1 = vert0;
            pinned[i1] = true;
        }
    }
}

static void eliminateStraightAngles(Nv::Blast::CutoutSetImpl& cutoutSet)
{
    // Eliminate straight angles
    for (uint32_t i = 0; i < cutoutSet.cutoutLoops.size(); ++i)
    {
        Nv::Blast::Cutout& cutout = cutoutSet.cutoutLoops[i];
        uint32_t oldSize;
        do
        {
            oldSize = cutout.vertices.size();
            for (uint32_t j = 0; j < cutout.vertices.size();)
            {
//              if( isOnBorder( cutout.vertices[j], width, height ) )
//              {   // Don't eliminate border vertices
//                  ++j;
//                  continue;
//              }
                
                if (perpendicularDistanceSquared(cutout.vertices, j) < CUTOUT_DISTANCE_EPS * CUTOUT_DISTANCE_EPS)
                {
                    cutout.vertices.erase(cutout.vertices.begin() + j);
                }
                else
                {
                    ++j;
                }
            }
        }
        while (cutout.vertices.size() != oldSize);
    }
}

static void removeTheSamePoints(Nv::Blast::CutoutSetImpl& cutoutSet)
{
    for (uint32_t i = 0; i < cutoutSet.cutoutLoops.size(); ++i)
    {
        Nv::Blast::Cutout& cutout = cutoutSet.cutoutLoops[i];
        uint32_t oldSize;
        do
        {
            oldSize = cutout.vertices.size();
            for (uint32_t j = 0; j < cutout.vertices.size();)
            {
                if ((cutout.vertices[(j + cutout.vertices.size() - 1) % cutout.vertices.size()] - cutout.vertices[j]).magnitudeSquared() < CUTOUT_DISTANCE_EPS * CUTOUT_DISTANCE_EPS)
                {
                    cutout.vertices.erase(cutout.vertices.begin() + j);
                }
                else
                {
                    ++j;
                }
            }
        } while (cutout.vertices.size() != oldSize);
    }
}

static void simplifyCutoutSetImpl(Nv::Blast::CutoutSetImpl& cutoutSet, float threshold, uint32_t width, uint32_t height)
{
    splitTJunctions(cutoutSet, 1.0f);
    mergeVertices(cutoutSet, threshold, width, height);
    eliminateStraightAngles(cutoutSet);
    splitTJunctions(cutoutSet, 1.0f);
    removeTheSamePoints(cutoutSet);
}

//static void cleanCutout(Nv::Blast::Cutout& cutout, uint32_t loopIndex, float tolerance)
//{
//  Nv::Blast::ConvexLoop& loop = cutout.convexLoops[loopIndex];
//  const float tolerance2 = tolerance * tolerance;
//  uint32_t oldSize;
//  do
//  {
//      oldSize = loop.polyVerts.size();
//      uint32_t size = oldSize;
//      for (uint32_t i = 0; i < size; ++i)
//      {
//          Nv::Blast::PolyVert& v0 = loop.polyVerts[(i + size - 1) % size];
//          Nv::Blast::PolyVert& v1 = loop.polyVerts[i];
//          Nv::Blast::PolyVert& v2 = loop.polyVerts[(i + 1) % size];
//          if (perpendicularDistanceSquared(cutout.vertices[v0.index], cutout.vertices[v1.index], cutout.vertices[v2.index]) <= tolerance2)
//          {
//              loop.polyVerts.erase(loop.polyVerts.begin() + i);
//              --size;
//              --i;
//          }
//      }
//  }
//  while (loop.polyVerts.size() != oldSize);
//}

//static bool decomposeCutoutIntoConvexLoops(Nv::Blast::Cutout& cutout, float cleanupTolerance = 0.0f)
//{
//  const uint32_t size = cutout.vertices.size();
//
//  if (size < 3)
//  {
//      return false;
//  }
//
//  // Initialize to one loop, which may not be convex
//  cutout.convexLoops.resize(1);
//  cutout.convexLoops[0].polyVerts.resize(size);
//
//  // See if the winding is ccw:
//
//  // Scale to normalized size to avoid overflows
//  nvidia::NvBounds3 bounds;
//  bounds.setEmpty();
//  for (uint32_t i = 0; i < size; ++i)
//  {
//      bounds.include(cutout.vertices[i]);
//  }
//  nvidia::NvVec3 center = bounds.getCenter();
//  nvidia::NvVec3 extent = bounds.getExtents();
//  if (extent[0] < NV_EPS_F32 || extent[1] < NV_EPS_F32)
//  {
//      return false;
//  }
//  const nvidia::NvVec3 scale(1.0f / extent[0], 1.0f / extent[1], 0.0f);
//
//  // Find "area" (it will only be correct in sign!)
//  nvidia::NvVec3 prevV = (cutout.vertices[size - 1] - center).multiply(scale);
//  float area = 0.0f;
//  for (uint32_t i = 0; i < size; ++i)
//  {
//      const nvidia::NvVec3 v = (cutout.vertices[i] - center).multiply(scale);
//      area += crossZ(prevV, v);
//      prevV = v;
//  }
//
//  if (nvidia::NvAbs(area) < NV_EPS_F32 * NV_EPS_F32)
//  {
//      return false;
//  }
//
//  const bool ccw = area > 0.0f;
//
//  for (uint32_t i = 0; i < size; ++i)
//  {
//      Nv::Blast::PolyVert& vert = cutout.convexLoops[0].polyVerts[i];
//      vert.index = (uint16_t)(ccw ? i : size - i - 1);
//      vert.flags = 0;
//  }
//
//  const float cleanupTolerance2 = square(cleanupTolerance);
//
//  // Find reflex vertices
//  for (uint32_t i = 0; i < cutout.convexLoops.size();)
//  {
//      Nv::Blast::ConvexLoop& loop = cutout.convexLoops[i];
//      const uint32_t loopSize = loop.polyVerts.size();
//      if (loopSize <= 3)
//      {
//          ++i;
//          continue;
//      }
//      uint32_t j = 0;
//      for (; j < loopSize; ++j)
//      {
//          const nvidia::NvVec3& v0 = cutout.vertices[loop.polyVerts[(j + loopSize - 1) % loopSize].index];
//          const nvidia::NvVec3& v1 = cutout.vertices[loop.polyVerts[j].index];
//          const nvidia::NvVec3& v2 = cutout.vertices[loop.polyVerts[(j + 1) % loopSize].index];
//          const nvidia::NvVec3 e0 = v1 - v0;
//          if (crossZ(e0, v2 - v1) < 0.0f)
//          {
//              // reflex
//              break;
//          }
//      }
//      if (j < loopSize)
//      {
//          // Find a vertex
//          float minLen2 = NV_MAX_F32;
//          float maxMinDist = -NV_MAX_F32;
//          uint32_t kToUse = 0;
//          uint32_t mToUse = 2;
//          bool cleanSliceFound = false;   // A transversal is parallel with an edge
//          for (uint32_t k = 0; k < loopSize; ++k)
//          {
//              const nvidia::NvVec3& vkPrev = cutout.vertices[loop.polyVerts[(k + loopSize - 1) % loopSize].index];
//              const nvidia::NvVec3& vk = cutout.vertices[loop.polyVerts[k].index];
//              const nvidia::NvVec3& vkNext = cutout.vertices[loop.polyVerts[(k + 1) % loopSize].index];
//              const uint32_t mStop = k ? loopSize : loopSize - 1;
//              for (uint32_t m = k + 2; m < mStop; ++m)
//              {
//                  const nvidia::NvVec3& vmPrev = cutout.vertices[loop.polyVerts[(m + loopSize - 1) % loopSize].index];
//                  const nvidia::NvVec3& vm = cutout.vertices[loop.polyVerts[m].index];
//                  const nvidia::NvVec3& vmNext = cutout.vertices[loop.polyVerts[(m + 1) % loopSize].index];
//                  const nvidia::NvVec3 newEdge = vm - vk;
//                  if (!directionsXYOrderedCCW(vk - vkPrev, newEdge, vkNext - vk) ||
//                          !directionsXYOrderedCCW(vm - vmPrev, -newEdge, vmNext - vm))
//                  {
//                      continue;
//                  }
//                  const float len2 = newEdge.magnitudeSquared();
//                  float minDist = NV_MAX_F32;
//                  for (uint32_t l = 0; l < loopSize; ++l)
//                  {
//                      const uint32_t l1 = (l + 1) % loopSize;
//                      if (l == k || l1 == k || l == m || l1 == m)
//                      {
//                          continue;
//                      }
//                      const nvidia::NvVec3& vl = cutout.vertices[loop.polyVerts[l].index];
//                      const nvidia::NvVec3& vl1 = cutout.vertices[loop.polyVerts[l1].index];
//                      const float dist = segmentsIntersectXY(vl, vl1 - vl, vk, newEdge);
//                      if (dist < minDist)
//                      {
//                          minDist = dist;
//                      }
//                  }
//                  if (minDist <= 0.0f)
//                  {
//                      if (minDist > maxMinDist)
//                      {
//                          maxMinDist = minDist;
//                          kToUse = k;
//                          mToUse = m;
//                      }
//                  }
//                  else
//                  {
//                      if (perpendicularDistanceSquared(vkPrev, vk, vm) <= cleanupTolerance2 ||
//                              perpendicularDistanceSquared(vk, vm, vmNext) <= cleanupTolerance2)
//                      {
//                          if (!cleanSliceFound)
//                          {
//                              minLen2 = len2;
//                              kToUse = k;
//                              mToUse = m;
//                          }
//                          else
//                          {
//                              if (len2 < minLen2)
//                              {
//                                  minLen2 = len2;
//                                  kToUse = k;
//                                  mToUse = m;
//                              }
//                          }
//                          cleanSliceFound = true;
//                      }
//                      else if (!cleanSliceFound && len2 < minLen2)
//                      {
//                          minLen2 = len2;
//                          kToUse = k;
//                          mToUse = m;
//                      }
//                  }
//              }
//          }
//          cutout.convexLoops.push_back(Nv::Blast::ConvexLoop());
//          Nv::Blast::ConvexLoop& newLoop = cutout.convexLoops.back();
//          Nv::Blast::ConvexLoop& oldLoop = cutout.convexLoops[i];
//          newLoop.polyVerts.resize(mToUse - kToUse + 1);
//          for (uint32_t n = 0; n <= mToUse - kToUse; ++n)
//          {
//              newLoop.polyVerts[n] = oldLoop.polyVerts[kToUse + n];
//          }
//          newLoop.polyVerts[mToUse - kToUse].flags = 1;   // Mark this vertex (and edge that follows) as a split edge
//          oldLoop.polyVerts[kToUse].flags = 1;    // Mark this vertex (and edge that follows) as a split edge
//          oldLoop.polyVerts.erase(oldLoop.polyVerts.begin() + kToUse + 1, oldLoop.polyVerts.begin() + (mToUse - (kToUse + 1)));
//          if (cleanupTolerance > 0.0f)
//          {
//              cleanCutout(cutout, i, cleanupTolerance);
//              cleanCutout(cutout, cutout.convexLoops.size() - 1, cleanupTolerance);
//          }
//      }
//      else
//      {
//          if (cleanupTolerance > 0.0f)
//          {
//              cleanCutout(cutout, i, cleanupTolerance);
//          }
//          ++i;
//      }
//  }
//
//  return true;
//}

static void traceRegion(std::vector<POINT2D>& trace, Map2d<uint32_t>& regions, Map2d<uint8_t>& pathCounts, uint32_t regionIndex, const POINT2D& startPoint)
{
    POINT2D t = startPoint;
    trace.clear();
    trace.push_back(t);
    ++pathCounts(t.x, t.y); // Increment path count
    // Find initial path direction
    int32_t dirN;

    uint32_t previousRegion = 0xFFFFFFFF;
    for (dirN = 0; dirN < 8; ++dirN) //TODO Should we start from dirN = 0?
    {
        const POINT2D t1 = POINT2D(t.x + taxicabSine(dirN + 2), t.y + taxicabSine(dirN));
        if (regions(t1.x, t1.y) != regionIndex && previousRegion == regionIndex)
        {
            break;
        }
        previousRegion = regions(t1.x, t1.y);
    }
    bool done = false;
    do
    {
        for (int32_t i = 1; i < 8; ++i) // Skip direction we just came from
        {
            --dirN;
            const POINT2D t1 = POINT2D(t.x + taxicabSine(dirN + 2), t.y + taxicabSine(dirN));
            if (regions(t1.x, t1.y) != regionIndex)
            {
                if (t1.x == trace[0].x && t1.y == trace[0].y)
                {
                    done = true;
                    break;
                }
                trace.push_back(t1);
                t = t1;
                ++pathCounts(t.x, t.y); // Increment path count
                dirN += 4;
                break;
            }
        }
    } while (!done && dirN >= 0);

    //NvBlast GWD-399: Try to fix bad corners
    int32_t sz = (int32_t)trace.size();
    if (sz > 4)
    {
        struct CornerPixel
        {
            int32_t id;
            POINT2D p;
            CornerPixel(int32_t id, int32_t x, int32_t y) : id(id), p(x, y) { }
        };
        std::vector <CornerPixel> cp;
        int32_t xb = 0, yb = 0; //bit buffer stores 1 if value do not changed from preview point and 0 otherwise (5 bits is used)
        for (int32_t i = -4; i < sz; i++) //fill buffer with 4 elements from the end of trace
        {
            //idx, idx - 1, idx - 2, idx - 3 values with correct indexing to trace
            int32_t idx = (sz + i) % sz, idx_ = (sz + i - 1) % sz, idx__ = (sz + i - 2) % sz, idx___ = (sz + i - 3) % sz;
            //update buffer
            xb <<= 1;
            yb <<= 1;
            xb += (trace[idx].x - trace[idx_].x) == 0;
            yb += (trace[idx].y - trace[idx_].y) == 0;
            //filter buffer for 11100-00111 or 00111-11100 corner patterns
            if (i >= 0 && ((xb & 0x1F) ^ (yb & 0x1F)) == 0x1B)
            {
                if ((xb & 3) == 3)
                {
                    if (((yb >> 3) & 3) == 3)
                    {
                        cp.push_back(CornerPixel(idx__, trace[idx].x, trace[idx___].y));
                    }
                }
                else if ((yb & 3) == 3)
                {
                    if (((xb >> 3) & 3) == 3)
                    {
                        cp.push_back(CornerPixel(idx__, trace[idx___].x, trace[idx].y));
                    }
                }
            }
        }
        std::sort(cp.begin(), cp.end(), [](const CornerPixel& cp1, const CornerPixel& cp2) -> bool
        {
            return cp1.id > cp2.id;
        });
        for (auto it = cp.begin(); it != cp.end(); it++)
        {
            trace.insert(trace.begin() + it->id, it->p);
            ++pathCounts(it->p.x, it->p.y);
        }
    }
}

void Nv::Blast::createCutoutSet(Nv::Blast::CutoutSetImpl& cutoutSet, const uint8_t* pixelBuffer, uint32_t bufferWidth, uint32_t bufferHeight, 
    float segmentationErrorThreshold, float snapThreshold, bool periodic, bool expandGaps)
{
    cutoutSet.cutouts.clear();
    cutoutSet.cutoutLoops.clear();
    cutoutSet.periodic = periodic;
    cutoutSet.dimensions = nvidia::NvVec2((float)bufferWidth, (float)bufferHeight);

    if (!periodic)
    {
        cutoutSet.dimensions[0] += 1.0f;
        cutoutSet.dimensions[1] += 1.0f;
    }

    if (pixelBuffer == NULL || bufferWidth == 0 || bufferHeight == 0)
    {
        return;
    }

    const int borderPad = periodic ? 0 : 2; // Padded for borders if not periodic
    const int originCoord = periodic ? 0 : 1;

    BitMap map(bufferWidth + borderPad, bufferHeight + borderPad, 0);
    map.setOrigin((uint32_t)originCoord, (uint32_t)originCoord);

    bool hasBorder = false;
    for (uint32_t y = 0; y < bufferHeight; ++y)
    {
        for (uint32_t x = 0; x < bufferWidth; ++x)
        {
            const uint32_t pix = 5033165 * (uint32_t)pixelBuffer[0] + 9898557 * (uint32_t)pixelBuffer[1] + 1845494 * (uint32_t)pixelBuffer[2];
            pixelBuffer += 3;
            if ((pix >> 28) != 0)
            {
                map.set((int32_t)x, (int32_t)y);
                hasBorder = true;
            }
        }
    }

    // Add borders if not tiling
    if (!periodic)
    {
        for (int32_t x = -1; x <= (int32_t)bufferWidth; ++x)
        {
            map.set(x, -1);
            map.set(x, (int32_t)bufferHeight);
        }
        for (int32_t y = -1; y <= (int32_t)bufferHeight; ++y)
        {
            map.set(-1, y);
            map.set((int32_t)bufferWidth, y);
        }
    }

    // Now search for regions

    // Create a region map
    Map2d<uint32_t> regions(bufferWidth + borderPad, bufferHeight + borderPad, 0xFFFFFFFF); // Initially an invalid value
    regions.setOrigin((uint32_t)originCoord, (uint32_t)originCoord);

    // Create a path counting map
    Map2d<uint8_t> pathCounts(bufferWidth + borderPad, bufferHeight + borderPad, 0);
    pathCounts.setOrigin((uint32_t)originCoord, (uint32_t)originCoord);

    // Bump path counts on borders
    if (!periodic)
    {
        for (int32_t x = -1; x <= (int32_t)bufferWidth; ++x)
        {
            pathCounts(x, -1) = 1;
            pathCounts(x, (int32_t)bufferHeight) = 1;
        }
        for (int32_t y = -1; y <= (int32_t)bufferHeight; ++y)
        {
            pathCounts(-1, y) = 1;
            pathCounts((int32_t)bufferWidth, y) = 1;
        }
    }

    std::vector<POINT2D> stack;
    std::vector<uint32_t> newCutout;
    std::vector<POINT2D> traceStarts;
    std::vector<std::vector<POINT2D>* > traces;

    std::set<uint64_t> regionBoundary;

    // Initial fill of region maps and path maps
    for (int32_t y = 0; y < (int32_t)bufferHeight; ++y)
    {
        for (int32_t x = 0; x < (int32_t)bufferWidth; ++x)
        {
            if (map.read(x - 1, y) && !map.read(x, y))
            {
                // Found an empty spot next to a filled spot
                POINT2D t(x - 1, y);
                const uint32_t regionIndex = traceStarts.size();
                newCutout.push_back(traces.size());
                traceStarts.push_back(t);   // Save off initial point
                traces.push_back(new std::vector<POINT2D>());
                NVBLAST_ASSERT(traces.size() == traceStarts.size()); // This must be the same size as traceStarts
                //traces.back() = (std::vector<POINT2D>*)NVBLAST_ALLOC(sizeof(std::vector<POINT2D>), NV_DEBUG_EXP("CutoutPoint2DSet"));
                //new(traces.back()) std::vector<POINT2D>;
                // Flood fill region map
                std::set<uint64_t> visited;
                stack.push_back(POINT2D(x, y));
#define COMPRESS(x, y) (((uint64_t)(x) << 32) + (y))
                visited.insert(COMPRESS(x, y));
                do
                {
                    const POINT2D s = stack.back();
                    stack.pop_back();
                    map.set(s.x, s.y);
                    regions(s.x, s.y) = regionIndex;
                    POINT2D n;
                    for (int32_t i = 0; i < 4; ++i)
                    {
                        const int32_t i0 = i & 1;
                        const int32_t i1 = (i >> 1) & 1;
                        n.x = s.x + i0 - i1;
                        n.y = s.y + i0 + i1 - 1;
                        if (visited.find(COMPRESS(n.x, n.y)) == visited.end())
                        {
                            if (!map.read(n.x, n.y))
                            {
                                stack.push_back(n);
                                visited.insert(COMPRESS(n.x, n.y));
                            }
                            else
                            {
                                regionBoundary.insert(COMPRESS(n.x, n.y));
                            }
                        }
                    }
                } while (stack.size());

                // Trace region
                NVBLAST_ASSERT(map.read(t.x, t.y));
                std::vector<POINT2D>* trace = traces.back();
                traceRegion(*trace, regions, pathCounts, regionIndex, t);

                //Find innner traces
                while(true)
                {
                    for (auto& point : *trace)
                    {
                        regionBoundary.erase(COMPRESS(point.x, point.y));
                    }
                    if (trace->size() < 4)
                    {
                        trace->~vector<POINT2D>();
                        delete trace;
                        traces.pop_back();
                        traceStarts.pop_back();
                    }
                    if (!regionBoundary.empty())
                    {
                        auto it = regionBoundary.begin();
                        t.x = *it >> 32;
                        t.y = *it & 0xFFFFFFFF;
                        traces.push_back(new std::vector<POINT2D>());
                        traceStarts.push_back(t);
                        trace = traces.back();
                        traceRegion(*trace, regions, pathCounts, regionIndex, t);
                        continue;
                    }
                    break;
                }
#undef COMPRESS
            }
        }
    }

    uint32_t cutoutCount = traces.size();

    //find internal traces

    // Now expand regions until the paths completely overlap
    if (expandGaps)
    {
        bool somePathChanged;
        int sanityCounter = 1000;
        bool abort = false;
        do
        {
            somePathChanged = false;
            for (uint32_t i = 0; i < cutoutCount; ++i)
            {
                if (traces[i] == nullptr)
                {
                    continue;
                }
                uint32_t regionIndex = 0;
                for (uint32_t c : newCutout)
                {
                    if (i >= c)
                    {
                        regionIndex = c;
                    }
                    else
                    {
                        break;
                    }
                }
                bool pathChanged = false;
                std::vector<POINT2D>& trace = *traces[i];
                for (size_t j = 0; j < trace.size(); ++j)
                {
                    const POINT2D& t = trace[j];
                    if (pathCounts(t.x, t.y) == 1)
                    {
                        if (regions(t.x, t.y) == 0xFFFFFFFF)
                        {
                            regions(t.x, t.y) = regionIndex;
                            pathChanged = true;
                        }
                        else
                        {
                            trace.erase(trace.begin() + j--);
                        }
                    }
                }
                if (pathChanged)
                {
                    // Recalculate cutout
                    // Decrement pathCounts
                    for (uint32_t j = 0; j < trace.size(); ++j)
                    {
                        const POINT2D& t = trace[j];
                        --pathCounts(t.x, t.y);
                    }
                    // Erase trace
                    // Calculate new start point
                    POINT2D& t = traceStarts[i];
                    POINT2D t1 = t;
                    abort = true;
                    for (int32_t dirN = 0; dirN < 8; ++dirN)
                    {
                        t1 = POINT2D(t.x + taxicabSine(dirN + 2), t.y + taxicabSine(dirN));
                        if (regions(t1.x, t1.y) != regionIndex)
                        {
                            t = t1;
                            abort = false;
                            break;
                        }
                    }
                    if (abort)
                    {
                        break;
                    }
                    traceRegion(trace, regions, pathCounts, regionIndex, t);
                    somePathChanged = true;
                }
            }
            if (--sanityCounter <= 0)
            {
                abort = true;
                break;
            }
        } while (somePathChanged);

        if (abort)
        {
            for (uint32_t i = 0; i < cutoutCount; ++i)
            {
                traces[i]->~vector<POINT2D>();
                delete traces[i];
            }
            cutoutCount = 0;
        }
    }

    // Create cutouts
    cutoutSet.cutouts = newCutout;
    cutoutSet.cutouts.push_back(cutoutCount);
    cutoutSet.cutoutLoops.resize(cutoutCount);
    for (uint32_t i = 0; i < cutoutCount; ++i)
    {
        createCutout(cutoutSet.cutoutLoops[i], *traces[i], segmentationErrorThreshold, snapThreshold, bufferWidth, bufferHeight, !cutoutSet.periodic);
    }

    if (expandGaps)
    {
        simplifyCutoutSetImpl(cutoutSet, snapThreshold, bufferWidth, bufferHeight);
    }

    // Release traces
    for (uint32_t i = 0; i < cutoutCount; ++i)
    {
        if (traces[i] != nullptr)
        {
            traces[i]->~vector<POINT2D>();
            delete traces[i];
        }
    }

    // Decompose each cutout in the set into convex loops
    //uint32_t cutoutSetSize = 0;
    //for (uint32_t i = 0; i < cutoutSet.cutoutLoops.size(); ++i)
    //{
    //  bool success = decomposeCutoutIntoConvexLoops(cutoutSet.cutoutLoops[i]);
    //  if (success)
    //  {
    //      if (cutoutSetSize != i)
    //      {
    //          cutoutSet.cutouts[cutoutSetSize] = cutoutSet.cutoutLoops[i];
    //      }
    //      ++cutoutSetSize;
    //  }
    //}
    //cutoutSet.cutoutLoops.resize(cutoutSetSize);

    //Check if single cutout spread to the whole area for non periodic (no need to cutout then)
    if (!periodic && cutoutSet.cutoutLoops.size() == 1 && (expandGaps || !hasBorder))
    {
        cutoutSet.cutoutLoops.clear();
    }
}

class Matrix22
{
public:
    //! Default constructor
    Matrix22()
    {}

    //! Construct from two base vectors
    Matrix22(const nvidia::NvVec2& col0, const nvidia::NvVec2& col1)
        : column0(col0), column1(col1)
    {}

    //! Construct from float[4]
    explicit Matrix22(float values[]):
        column0(values[0],values[1]),
        column1(values[2],values[3])
    {
    }

    //! Copy constructor
    Matrix22(const Matrix22& other)
        : column0(other.column0), column1(other.column1)
    {}

    //! Assignment operator
    Matrix22& operator=(const Matrix22& other)
    {
        column0 = other.column0;
        column1 = other.column1;
        return *this;
    }

    //! Set to identity matrix
    static Matrix22 createIdentity()
    {
        return Matrix22(nvidia::NvVec2(1,0), nvidia::NvVec2(0,1));
    }

    //! Set to zero matrix
    static Matrix22 createZero()
    {
        return Matrix22(nvidia::NvVec2(0.0f), nvidia::NvVec2(0.0f));
    }

    //! Construct from diagonal, off-diagonals are zero.
    static Matrix22 createDiagonal(const nvidia::NvVec2& d)
    {
        return Matrix22(nvidia::NvVec2(d.x,0.0f), nvidia::NvVec2(0.0f,d.y));
    }


    //! Get transposed matrix
    Matrix22 getTranspose() const
    {
        const nvidia::NvVec2 v0(column0.x, column1.x);
        const nvidia::NvVec2 v1(column0.y, column1.y);

        return Matrix22(v0,v1);   
    }

    //! Get the real inverse
    Matrix22 getInverse() const
    {
        const float det = getDeterminant();
        Matrix22 inverse;

        if(det != 0)
        {
            const float invDet = 1.0f/det;

            inverse.column0[0] = invDet * column1[1];                       
            inverse.column0[1] = invDet * (-column0[1]);

            inverse.column1[0] = invDet * (-column1[0]);
            inverse.column1[1] = invDet * column0[0];

            return inverse;
        }
        else
        {
            return createIdentity();
        }
    }

    //! Get determinant
    float getDeterminant() const
    {
        return column0[0] * column1[1] - column0[1] * column1[0];
    }

    //! Unary minus
    Matrix22 operator-() const
    {
        return Matrix22(-column0, -column1);
    }

    //! Add
    Matrix22 operator+(const Matrix22& other) const
    {
        return Matrix22( column0+other.column0,
                      column1+other.column1);
    }

    //! Subtract
    Matrix22 operator-(const Matrix22& other) const
    {
        return Matrix22( column0-other.column0,
                      column1-other.column1);
    }

    //! Scalar multiplication
    Matrix22 operator*(float scalar) const
    {
        return Matrix22(column0*scalar, column1*scalar);
    }
    
    //! Matrix vector multiplication (returns 'this->transform(vec)')
    nvidia::NvVec2 operator*(const nvidia::NvVec2& vec) const
    {
        return transform(vec);
    }

    //! Matrix multiplication
    Matrix22 operator*(const Matrix22& other) const
    {
        //Rows from this <dot> columns from other
        //column0 = transform(other.column0) etc
        return Matrix22(transform(other.column0), transform(other.column1));
    }

    // a <op>= b operators

    //! Equals-add
    Matrix22& operator+=(const Matrix22& other)
    {
        column0 += other.column0;
        column1 += other.column1;
        return *this;
    }

    //! Equals-sub
    Matrix22& operator-=(const Matrix22& other)
    {
        column0 -= other.column0;
        column1 -= other.column1;
        return *this;
    }

    //! Equals scalar multiplication
    Matrix22& operator*=(float scalar)
    {
        column0 *= scalar;
        column1 *= scalar;
        return *this;
    }

    //! Element access, mathematical way!
    float operator()(unsigned int row, unsigned int col) const
    {
        return (*this)[col][(int)row];
    }

    //! Element access, mathematical way!
    float& operator()(unsigned int row, unsigned int col)
    {
        return (*this)[col][(int)row];
    }

    // Transform etc
    
    //! Transform vector by matrix, equal to v' = M*v
    nvidia::NvVec2 transform(const nvidia::NvVec2& other) const
    {
        return column0*other.x + column1*other.y;
    }

    nvidia::NvVec2& operator[](unsigned int num)         {return (&column0)[num];}
    const   nvidia::NvVec2& operator[](unsigned int num) const   {return (&column0)[num];}

    //Data, see above for format!

    nvidia::NvVec2 column0, column1; //the two base vectors
};

 bool calculateUVMapping(const Nv::Blast::Triangle& triangle, nvidia::NvMat33& theResultMapping)
{
    nvidia::NvMat33 rMat;
    nvidia::NvMat33 uvMat;
    for (unsigned col = 0; col < 3; ++col)
    {
        auto v = (&triangle.a)[col];
        rMat[col] = toNvShared(v.p);
        uvMat[col] = nvidia::NvVec3(v.uv[0].x, v.uv[0].y, 1.0f);
    }

    if (uvMat.getDeterminant() == 0.0f)
    {
        return false;
    }

    theResultMapping = rMat*uvMat.getInverse();

    return true;
}

//static bool calculateUVMapping(ExplicitHierarchicalMesh& theHMesh, const nvidia::NvVec3& theDir, nvidia::NvMat33& theResultMapping)
//{ 
//  nvidia::NvVec3 cutoutDir( theDir );
//  cutoutDir.normalize( );
//
//  const float cosineThreshold = nvidia::NvCos(3.141593f / 180);    // 1 degree
//
//  ExplicitRenderTriangle* triangleToUse = NULL;
//  float greatestCosine = -NV_MAX_F32;
//  float greatestArea = 0.0f;  // for normals within the threshold
//  for ( uint32_t partIndex = 0; partIndex < theHMesh.partCount(); ++partIndex )
//  {
//      ExplicitRenderTriangle* theTriangles = theHMesh.meshTriangles( partIndex );
//      uint32_t triangleCount = theHMesh.meshTriangleCount( partIndex );
//      for ( uint32_t tIndex = 0; tIndex < triangleCount; ++tIndex )
//      {           
//          ExplicitRenderTriangle& theTriangle = theTriangles[tIndex];
//          nvidia::NvVec3 theEdge1 = theTriangle.vertices[1].position - theTriangle.vertices[0].position;
//          nvidia::NvVec3 theEdge2 = theTriangle.vertices[2].position - theTriangle.vertices[0].position;
//          nvidia::NvVec3 theNormal = theEdge1.cross( theEdge2 );
//          float theArea = theNormal.normalize();  // twice the area, but that's ok
//
//          if (theArea == 0.0f)
//          {
//              continue;
//          }
//
//          const float cosine = cutoutDir.dot(theNormal);
//
//          if (cosine < cosineThreshold)
//          {
//              if (cosine > greatestCosine && greatestArea == 0.0f)
//              {
//                  greatestCosine = cosine;
//                  triangleToUse = &theTriangle;
//              }
//          }
//          else
//          {
//              if (theArea > greatestArea)
//              {
//                  greatestArea = theArea;
//                  triangleToUse = &theTriangle;
//              }
//          }
//      }
//  }
//
//  if (triangleToUse == NULL)
//  {
//      return false;
//  }
//
//  return calculateUVMapping(*triangleToUse, theResultMapping);
//}



//bool calculateCutoutUVMapping(ExplicitHierarchicalMesh& hMesh, const nvidia::NvVec3& targetDirection, nvidia::NvMat33& theMapping)
//{
//  return ::calculateUVMapping(hMesh, targetDirection, theMapping);
//}

//bool calculateCutoutUVMapping(const Nv::Blast::Triangle& targetDirection, nvidia::NvMat33& theMapping)
//{
//  return ::calculateUVMapping(targetDirection, theMapping);
//}

const NvcVec3& CutoutSetImpl::getCutoutVertex(uint32_t cutoutIndex, uint32_t loopIndex, uint32_t vertexIndex) const
{
    return fromNvShared(cutoutLoops[cutouts[cutoutIndex] + loopIndex].vertices[vertexIndex]);
}

const NvcVec2& CutoutSetImpl::getDimensions() const
{
    return fromNvShared(dimensions);
}

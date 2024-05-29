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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.


#ifndef NVBLASTINDEXFNS_H
#define NVBLASTINDEXFNS_H


#include "NvBlastTypes.h"

#include <cstring>


namespace Nv
{
namespace Blast
{

/**
Set to invalid index.
*/
template<typename T>
NV_INLINE T invalidIndex()
{
    return ~(T)0;
}


/**
Test for invalid index (max representable integer).
*/
template<typename T>
NV_INLINE bool isInvalidIndex(T index)
{
    return index == invalidIndex<T>();
}


/**
    Create a lookup table for data sorted by a templated index type.

    Note: when using this function with unsigned integer index types invalidIndex<T>() is treated as a value less than zero.

    On input:

        The indices must lie in the interval [indexBase, indexBase+indexRange].
    
        indexSource must point to the first index in the data.

        indexCount must be set to the number of indices in the data.

        indexByteStride must be set to the distance, in bytes, between subequent indices.

        lookup must point to a T array of size indexRange+2.

    On return:

        lookup will be filled such that:

            lookup[i] = the position of first data element with index (i + indexBase)

            lookup[indexRange+1] = indexCount

        The last (indexRange+1) element is used so that one may always determine the number of data elements with the given index using:

            count = lookup[i+1] - lookup[i]

        Note, if an index (i + indexBase) is not present in the data then, lookup[i+1] = lookup[i], so the count (above) will correctly be zero.
        In this case, the actual value of lookup[i] is irrelevant.
*/
template<typename T>
void createIndexStartLookup(T* lookup, T indexBase, T indexRange, T* indexSource, T indexCount, T indexByteStride)
{
    ++indexBase;    // Ordering invalidIndex<T>() as lowest value
    T indexPos = 0;
    for (T i = 0; i <= indexRange; ++i)
    {
        for (; indexPos < indexCount; ++indexPos, indexSource = (T*)((uintptr_t)indexSource + indexByteStride))
        {
            if (*indexSource + 1 >= i + indexBase)  // +1 to order invalidIndex<T>() as lowest value
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
    lookup[indexRange + 1] = indexCount;
}


/**
Creates the inverse of a map, such that inverseMap[map[i]] = i.
Unmapped indices are set to invalidIndex<T>.

\param[out]     inverseMap      inverse map space of given size
\param[in]      map             original map of given size, unmapped entries must contain invalidIndex<T>
\param[in]      size            size of the involved maps
*/
template<typename T>
void invertMap(T* inverseMap, const T* map, const T size)
{
    memset(inverseMap, invalidIndex<T>(), size*sizeof(T));

    for (T i = 0; i < size; i++)
    {
        if (!isInvalidIndex(map[i]))
        {
            inverseMap[map[i]] = i;
        }
    }
}

} // end namespace Blast
} // end namespace Nv


#endif // #ifndef NVBLASTINDEXFNS_H

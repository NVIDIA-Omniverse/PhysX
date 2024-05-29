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


#ifndef NVBLASTITERATORBASE_H
#define NVBLASTITERATORBASE_H


#include "NvBlastIndexFns.h"

namespace Nv
{
namespace Blast
{

/**
Common functionality and implementation for iterators over an index, using invalidIndex<T>() to indicate termination.
Derived class needs to implement increment operators.
*/
template<typename T>
class IteratorBase
{
public:
    /** Constructor sets m_curr value */
    IteratorBase(T curr);

    /** Validity of current value. */
    operator    bool() const;

    /** Current value. */
    operator    T() const;

protected:
    T   m_curr;
};


//////// IteratorBase<T> inline methods ////////

template<typename T>
NV_INLINE IteratorBase<T>::IteratorBase(T curr) : m_curr(curr)
{
}


template<typename T>
NV_INLINE IteratorBase<T>::operator bool() const
{
    return !isInvalidIndex<T>(m_curr);
}


template<typename T>
NV_INLINE IteratorBase<T>::operator T() const
{
    return m_curr;
}


/**
Common functionality and implementation for an indexed linked list iterator
*/
template<typename IndexType>
class LListIt : public IteratorBase<IndexType>
{
public:
    LListIt(IndexType curr, IndexType* links);

    /** Pre-increment.  Only use if valid() == true. */
    uint32_t    operator ++ ();

protected:
    IndexType*  m_links;
};


//////// LListIt<IndexType> inline methods ////////

template<typename IndexType>
NV_INLINE LListIt<IndexType>::LListIt(IndexType curr, IndexType* links) : IteratorBase<IndexType>(curr), m_links(links)
{
}


template<typename IndexType>
NV_INLINE uint32_t LListIt<IndexType>::operator ++ ()
{
    NVBLAST_ASSERT((bool)(*this));
    return (this->m_curr = m_links[this->m_curr]);
}


/**
Common functionality and implementation for an IndexDList<IndexType> iterator
*/
template<typename IndexType>
class DListIt : public IteratorBase<IndexType>
{
public:
    DListIt(IndexType curr, IndexDLink<IndexType>* links);

    /** Pre-increment.  Only use if valid() == true. */
    uint32_t    operator ++ ();

protected:
    IndexDLink<IndexType>*  m_links;
};


//////// DListIt<IndexType> inline methods ////////

template<typename IndexType>
NV_INLINE DListIt<IndexType>::DListIt(IndexType curr, IndexDLink<IndexType>* links) : IteratorBase<IndexType>(curr), m_links(links)
{
}


template<typename IndexType>
NV_INLINE uint32_t DListIt<IndexType>::operator ++ ()
{
    NVBLAST_ASSERT((bool)(*this));
    return (this->m_curr = m_links[this->m_curr].m_adj[1]);
}

} // end namespace Blast
} // end namespace Nv


#endif // #ifndef NVBLASTITERATORBASE_H

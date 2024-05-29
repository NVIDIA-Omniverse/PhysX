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


#ifndef NVBLASTFIXEDPRIORITYQUEUE_H
#define NVBLASTFIXEDPRIORITYQUEUE_H

#include "NvBlastAssert.h"
#include "NvBlastMemory.h"

namespace Nv
{

namespace Blast
{

/*!
FixedPriorityQueue is a priority queue container which is intended to be used with placement new on chunk of memory.
It'll use following memory for data layout. As follows:

// some memory
char ​*buf = new char[64 *​ 1024];

// placement new on this memory
FixedPriorityQueue<SomeClass>* arr = new (buf) FixedPriorityQueue<SomeClass>();

// you can get max requiredMemorySize by an array of 'capacity' elements count to use memory left
buf = buf + FixedPriorityQueue<SomeClass>::requiredMemorySize(capacity);

buf:

+------------------------------------------------------------+
|  uint32_t  |  T[0]  |  T[1]  |  T[2]  |         ...        |
+------------------------------------------------------------+

*/

template <typename A>
struct Less
{
    bool operator()(const A& a, const A& b) const
    {
        return a < b;
    }
};


template<class Element, class Comparator = Less<Element> >
class FixedPriorityQueue : protected Comparator // inherit so that stateless comparators take no space
{
public:
    FixedPriorityQueue(const Comparator& less = Comparator()) : Comparator(less), mHeapSize(0)
    {
    }
        
    ~FixedPriorityQueue()
    {
    }

    static size_t requiredMemorySize(uint32_t capacity)
    {
        return align16(sizeof(FixedPriorityQueue<Element, Comparator>)) + align16(capacity * sizeof(Element));
    }
        
    //! Get the element with the highest priority
    const Element top() const
    {
        return data()[0];
    }

    //! Get the element with the highest priority
    Element top()
    {
        return data()[0];
    }
        
    //! Check to whether the priority queue is empty
    bool empty() const
    {
        return (mHeapSize == 0);
    }
        
    //! Empty the priority queue
    void clear()
    {
        mHeapSize = 0;
    }  

    //! Insert a new element into the priority queue. Only valid when size() is less than Capacity
    void push(const Element& value)
    {
        uint32_t newIndex;
        uint32_t parentIndex = parent(mHeapSize);

        for (newIndex = mHeapSize; newIndex > 0 && compare(value, data()[parentIndex]); newIndex = parentIndex, parentIndex= parent(newIndex)) 
        {
            data()[ newIndex ] = data()[parentIndex];
        }
        data()[newIndex] = value; 
        mHeapSize++;
        NVBLAST_ASSERT(valid());
    }

    //! Delete the highest priority element. Only valid when non-empty.
    Element pop()
    {
        NVBLAST_ASSERT(mHeapSize > 0);
        uint32_t i, child;
        //try to avoid LHS
        uint32_t tempHs = mHeapSize-1;
        mHeapSize = tempHs;
        Element min = data()[0];
        Element last = data()[tempHs];
            
        for (i = 0; (child = left(i)) < tempHs; i = child) 
        {
            /* Find highest priority child */
            const uint32_t rightChild = child + 1;
            
            child += ((rightChild < tempHs) & compare((data()[rightChild]), (data()[child]))) ? 1 : 0;

            if(compare(last, data()[child]))
                break;

            data()[i] = data()[child];
        }
        data()[ i ] = last;
            
        NVBLAST_ASSERT(valid());
        return min;
    } 

    //! Make sure the priority queue sort all elements correctly
    bool valid() const
    {
        const Element& min = data()[0];
        for(uint32_t i=1; i<mHeapSize; ++i)
        {
            if(compare(data()[i], min))
                return false;
        }

        return true;
    }

    //! Return number of elements in the priority queue
    uint32_t size() const
    {
        return mHeapSize;
    }

private:
    uint32_t mHeapSize;

    NV_FORCE_INLINE Element* data()
    {
        return (Element*)((char*)this + sizeof(FixedPriorityQueue<Element, Comparator>));
    }

    NV_FORCE_INLINE Element* data() const
    {
        return (Element*)((char*)this + sizeof(FixedPriorityQueue<Element, Comparator>));
    }

    bool compare(const Element& a, const Element& b) const
    {
        return Comparator::operator()(a,b);
    }

    static uint32_t left(uint32_t nodeIndex) 
    {
        return (nodeIndex << 1) + 1;
    }
        
    static uint32_t parent(uint32_t nodeIndex) 
    {
        return (nodeIndex - 1) >> 1;
    }

    FixedPriorityQueue<Element, Comparator>& operator = (const FixedPriorityQueue<Element, Comparator>);
};

} // namespace Blast
} // namespace Nv

#endif // ifndef NVBLASTFIXEDPRIORITYQUEUE_H

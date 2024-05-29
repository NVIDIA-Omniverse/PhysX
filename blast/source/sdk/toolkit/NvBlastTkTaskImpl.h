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


#ifndef NVBLASTTKTASKIMPL_H
#define NVBLASTTKTASKIMPL_H

#include "NvBlast.h"

#include "NvBlastTkFrameworkImpl.h"
#include "NvBlastTkEventQueue.h"
#include "NvBlastArray.h"

#include <atomic>
#include <mutex>
#include <condition_variable>

#include "NvBlastAssert.h"

#include "NvBlastTkGroup.h" // TkGroupStats


namespace Nv
{
namespace Blast
{

class TkGroupImpl;
class TkActorImpl;
class TkFamilyImpl;


/**
Transient structure describing a job and its results.
*/
struct TkWorkerJob
{
    TkActorImpl*    m_tkActor;          //!< the actor to process
    TkActorImpl**   m_newActors;        //!< list of child actors created by splitting
    uint32_t        m_newActorsCount;   //!< the number of child actors created
};





/**
A list of equally sized memory blocks sharable between tasks.
*/
template<typename T>
class SharedBlock
{
public:

    SharedBlock() : m_numElementsPerBlock(0), m_numBlocks(0), m_buffer(nullptr) {}

    /**
    Allocates one large memory block of elementsPerBlock*numBlocks elements.
    */
    void allocate(uint32_t elementsPerBlock, uint32_t numBlocks)
    {
        NVBLAST_ASSERT(elementsPerBlock > 0 && numBlocks > 0);

        m_buffer = reinterpret_cast<T*>(NVBLAST_ALLOC_NAMED(elementsPerBlock*numBlocks*sizeof(T), "SharedBlock"));
        m_numElementsPerBlock = elementsPerBlock;
        m_numBlocks = numBlocks;
    }

    /**
    Returns the pointer to the first element of a block of numElementsPerBlock() elements.
    */
    T* getBlock(uint32_t id)
    {
        NVBLAST_ASSERT(id < m_numBlocks || 0 == m_numElementsPerBlock);
        return &m_buffer[id*m_numElementsPerBlock];
    }

    /**
    The number of elements available per block.
    */
    uint32_t numElementsPerBlock() const 
    {
        return m_numElementsPerBlock; 
    }

    /**
    Frees the whole memory block.
    */
    void release()
    {
        m_numBlocks = 0;
        m_numElementsPerBlock = 0;
        NVBLAST_FREE(m_buffer);
        m_buffer = nullptr;
    }

private:
    uint32_t    m_numElementsPerBlock;  //!< elements available in one block
    uint32_t    m_numBlocks;            //!< number of virtual blocks available
    T*          m_buffer;               //!< contiguous memory for all blocks
};


/**
A preallocated, shared array from which can be allocated from in tasks.
Intended to be used when the maximum amount of data (e.g. for a family) 
is known in advance. No further allocations take place on exhaustion.
Exhaustion asserts in debug builds and overflows otherwise.
*/
template<typename T>
class SharedBuffer
{
public:
    SharedBuffer() : m_capacity(0), m_used(0), m_buffer(nullptr) {}

    /**
    Atomically gets a pointer to the first element of an array of n elements.
    */
    T* reserve(size_t n)
    {
        NVBLAST_ASSERT(m_used + n <= m_capacity);
        size_t start = m_used.fetch_add(n);
        return &m_buffer[start];
    }

    /**
    Preallocates memory for capacity elements.
    */
    void allocate(size_t capacity)
    {
        NVBLAST_ASSERT(m_buffer == nullptr);
        m_buffer = reinterpret_cast<T*>(NVBLAST_ALLOC_NAMED(capacity*sizeof(T), "SplitMemory"));
        m_capacity = capacity;
    }

    /**
    Preserves the memory allocated but resets to reserve from the beginning of the array.
    */
    void reset()
    {
        m_used = 0;
    }

    /**
    Frees the preallocated array.
    */
    void release()
    {
        NVBLAST_ASSERT(m_buffer != nullptr);
        NVBLAST_FREE(m_buffer);
        m_buffer = nullptr;
        m_capacity = m_used = 0;
    }

private:
    size_t              m_capacity; //!< available elements in the buffer
    std::atomic<size_t> m_used;     //!< used elements in the buffer
    T*                  m_buffer;   //!< the memory containing T's
};


/**
Allocates from a preallocated, externally owned memory block initialized with.
When blocks run out of space, new ones are allocated and owned by this class.
*/
template<typename T>
class LocalBuffer
{
public:
    /**
    Returns the pointer to the first element of an array of n elements.
    Allocates a new block of memory when exhausted, its size being the larger of n and capacity set with initialize().
    */
    T* allocate(size_t n)
    {
        if (m_used + n > m_capacity)
        {
            allocateNewBlock(n > m_capacity ? n : m_capacity);
        }

        size_t index = m_used;
        m_used += n;
        return &m_currentBlock[index];
    }

    /**
    Release the additionally allocated memory blocks.
    The externally owned memory block remains untouched.
    */
    void clear()
    {
        for (void* block : m_memoryBlocks)
        {
            NVBLAST_FREE(block);
        }
        m_memoryBlocks.clear();
    }

    /**
    Set the externally owned memory block to start allocating from,
    with a size of capacity elements.
    */
    void initialize(T* block, size_t capacity)
    {
        m_currentBlock = block;
        m_capacity = capacity;
        m_used = 0;
    }

private:
    /**
    Allocates space for capacity elements.
    */
    void allocateNewBlock(size_t capacity)
    {
        BLAST_PROFILE_SCOPE_L("Local Buffer allocation");
        m_capacity = capacity;
        m_currentBlock = static_cast<T*>(NVBLAST_ALLOC_NAMED(capacity*sizeof(T), "Blast LocalBuffer"));
        m_memoryBlocks.pushBack(m_currentBlock);
        m_used = 0;
    }

    InlineArray<void*, 4>::type     m_memoryBlocks; //!< storage for memory blocks
    T*                              m_currentBlock; //!< memory block used to allocate from
    size_t                          m_used;         //!< elements used in current block
    size_t                          m_capacity;     //!< elements available in current block
};


/**
Holds the memory used by TkWorker for each family in each group.
*/
class SharedMemory
{
public:
    SharedMemory() : m_eventsMemory(0), m_eventsCount(0), m_refCount(0) {}

    /**
    Reserves n entries from preallocated memory.
    */
    NvBlastActor** reserveNewActors(size_t n)
    {
        return m_newActorBuffers.reserve(n);
    }

    /**
    Reserves n entries from preallocated memory.
    */
    TkActor** reserveNewTkActors(size_t n)
    {
        return m_newTkActorBuffers.reserve(n);
    }

    /**
    Allocates buffers to hold 
    */
    void allocate(TkFamilyImpl&);

    /**
    Resets the internal buffers to reserve from their beginning.
    Preserves the allocated memory.
    */
    void reset()
    {
        m_newActorBuffers.reset();
        m_newTkActorBuffers.reset();
    }

    /**
    Increments the reference count.
    */
    void addReference()         { m_refCount++; }

    /**
    Increments the reference count by n.
    */
    void addReference(size_t n) { m_refCount += n; }

    /**
    Decrements the reference count.
    Returns true if the count reached zero.
    */
    bool removeReference()
    {
        m_refCount--;
        return !isUsed();
    }

    /**
    Checks if the reference count is not zero. 
    */
    bool isUsed()
    {
        return m_refCount > 0;
    }

    /**
    Release the internal buffers' memory.
    */
    void release()
    {
        m_newActorBuffers.release();
        m_newTkActorBuffers.release();
    }

    TkEventQueue                m_events;               //!< event queue shared across a group's actors of the same family
    uint32_t                    m_eventsMemory;         //!< expected memory size for event data
    uint32_t                    m_eventsCount;          //!< expected number of events

private:
    size_t                      m_refCount;             //!< helper for usage and releasing memory

    SharedBuffer<NvBlastActor*> m_newActorBuffers;      //!< memory for splitting
    SharedBuffer<TkActor*>      m_newTkActorBuffers;    //!< memory for split events
};


/**
Thread worker fracturing and splitting actors sequentially.
The list of actual jobs is provided by the group owning this worker.
*/
class TkWorker final : public TkGroupWorker
{
public:
    TkWorker() : m_id(~(uint32_t)0), m_group(nullptr), m_isBusy(false) {}

    void        process(uint32_t jobID);
    void        initialize();

    void        process(TkWorkerJob& job);

    uint32_t                                m_id;           //!< this worker's id
    TkGroupImpl*                            m_group;        //!< the group owning this worker

    LocalBuffer<NvBlastChunkFractureData>   m_chunkBuffer;  //!< memory manager for chunk event data
    LocalBuffer<NvBlastBondFractureData>    m_bondBuffer;   //!< memory manager for bonds event data

    void*                                   m_splitScratch;
    NvBlastFractureBuffers                  m_tempBuffer;
    bool                                    m_isBusy;

#if NV_PROFILE
    TkGroupStats    m_stats;
#endif
};
}
}

#endif // NVBLASTTKTASKIMPL_H

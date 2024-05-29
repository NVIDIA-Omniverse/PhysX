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


#ifndef NVBLASTTKEVENTQUEUE_H
#define NVBLASTTKEVENTQUEUE_H

#include <algorithm>
#include <vector>

#include <mutex>
#include <atomic>

#include "NvBlastTkFrameworkImpl.h"
#include "NvBlastAssert.h"


namespace Nv {
namespace Blast {

/**
A dispatcher queue providing preallocation and thread-safe insertions therein.

Typical usage:
- preallocate space for events and payload:                                         
 - reserveEvents, reserveData
- enable asserts to detect undersized storage (allocations are not thread safe):    
 - protect(true)
- get pointers to payload data and events to fill in, thread safe for preallocated memory:                                              
 - allocData, addEvent
- back on main thread, ensure consistency:                                          
 - protect(false)

- continue adding events and payload on main thread if necessary like above (allocations are safe here)
eventually dispatch, or reset if dispatched by proxy
*/
class TkEventQueue
{
public:
    TkEventQueue() : m_currentEvent(0), m_poolCapacity(0), m_pool(nullptr), m_allowAllocs(true) {}

    /**
    Peek events queue for dispatch.
    Do not use in protected state.
    */
    operator const Array<TkEvent>::type&() 
    {
        NVBLAST_ASSERT(m_allowAllocs);
        NVBLAST_ASSERT(m_currentEvent == m_events.size());
        return m_events; 
    }

    /** 
    Debug help to catch (unwanted) allocations during task work.
    Note that this will not actually avoid allocations, but assert in debug builds.

    Set true before using in distributed environment.
    Set false to return to single-thread mode.
    */
    void protect(bool enable)
    {
        // During parallel use, m_events.size() and m_currentEvent are allowed to diverge.
        // This is fine because resizeUninitialized does not alter the stored data.
        NVBLAST_ASSERT(m_currentEvent <= m_events.capacity());
        m_events.resizeUninitialized(m_currentEvent);
        m_allowAllocs = !enable;
    }

    /**
    Restores initial state.
    Data memory is currently not being reused. To be improved.
    */
    void reset()
    {
        m_events.clear();
        m_currentEvent = 0;
        for (void* mem : m_memory)
        {
            NVBLAST_FREE(mem);
        }
        m_memory.clear();
        m_currentData = 0;
        m_allowAllocs = true;
        m_poolCapacity = 0;
        m_pool = nullptr;
    }

    /**
    Queue an event with a payload.
    */
    template<class T>
    void addEvent(T* payload)
    {
        uint32_t index = m_currentEvent.fetch_add(1);

        // Should not allocate in protected state.
        NVBLAST_ASSERT(m_allowAllocs || m_currentEvent <= m_events.capacity());

        m_events.resizeUninitialized(m_currentEvent);

        // During parallel use, m_events.size() and m_currentEvent are allowed to diverge.
        // Consistency is restored in protect().
        NVBLAST_ASSERT(!m_allowAllocs || m_currentEvent == m_events.size());

        TkEvent& evt = m_events[index];
        evt.type = TkEvent::Type(T::EVENT_TYPE);
        evt.payload = payload;
    }

    /**
    Request storage for payload.
    */
    template<typename T>
    T* allocData()
    {
        uint32_t index = m_currentData.fetch_add(sizeof(T));
        if (m_currentData <= m_poolCapacity)
        {
            return reinterpret_cast<T*>(&m_pool[index]);
        }
        else
        {
            // Could do larger block allocation here.
            reserveData(sizeof(T));
            // Account for the requested size.
            m_currentData = sizeof(T);
            return reinterpret_cast<T*>(&m_pool[0]);
        }
    }

    /**
    Preallocate a memory block of size Bytes for payload data.
    Note that this will inevitably allocate a new memory block.
    Subsequent calls to allocData will use this memory piecewise.
    */
    void reserveData(size_t size)
    {
        NVBLAST_ASSERT(m_allowAllocs);
        m_pool = reinterpret_cast<uint8_t*>(allocDataBySize(size));
        m_poolCapacity = size;
        m_currentData = 0;
    }

    /**
    Preallocate space for events.
    */
    void reserveEvents(uint32_t n)
    {
        NVBLAST_ASSERT(m_allowAllocs);
        m_events.reserve(m_events.size() + n);
    }

    /**
    Add a listener to dispatch to.
    */
    void addListener(TkEventListener& l)
    {
        m_listeners.pushBack(&l);
    }

    /**
    Remove a listener from dispatch list.
    */
    void removeListener(TkEventListener& l)
    {
        m_listeners.findAndReplaceWithLast(&l);
    }

    /**
    Dispatch the stored events to the registered listeners.
    After dispatch, all data is invalidated.
    */
    void dispatch()
    {
        dispatch(*this);
        reset();
    }

    /**
    Proxy function to dispatch events to this queue's listeners.
    */
    void dispatch(const Array<TkEvent>::type& events) const
    {
        if (events.size())
        {
            for (TkEventListener* l : m_listeners)
            {
                BLAST_PROFILE_SCOPE_M("TkEventQueue::dispatch");
                l->receive(events.begin(), events.size());
            }
        }
    }

private:
    /**
    Allocates and stores a block of size Bytes of payload data.
    */
    void* allocDataBySize(size_t size)
    {
        void* memory = nullptr;
        if (size > 0)
        {
            memory = NVBLAST_ALLOC_NAMED(size, "TkEventQueue Data");
            m_memory.pushBack(memory);
        }
        return memory;
    }


    Array<TkEvent>::type                    m_events;       //!< holds events
    Array<void*>::type                      m_memory;       //!< holds allocated data memory blocks
    std::atomic<uint32_t>                   m_currentEvent; //!< reference index for event insertion
    std::atomic<uint32_t>                   m_currentData;  //!< reference index for data insertion
    size_t                                  m_poolCapacity; //!< size of the currently active memory block (m_pool)
    uint8_t*                                m_pool;         //!< the current memory block allocData() uses
    bool                                    m_allowAllocs;  //!< assert guard
    InlineArray<TkEventListener*,4>::type   m_listeners;    //!< objects to dispatch to
};

}   // namespace Blast
}   // namespace Nv


#endif  // ifndef NVBLASTTKEVENTQUEUE_H

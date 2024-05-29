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


#ifndef NVBLASTDLINK_H
#define NVBLASTDLINK_H


#include "NvBlastAssert.h"
#include "NvBlastIndexFns.h"


namespace Nv
{
namespace Blast
{

template<typename IndexType>
struct IndexDLink
{
    IndexType   m_adj[2];
};


template<typename IndexType>
class IndexDList
{
public:
    void        initLinksSolitary(IndexDLink<IndexType>* links, IndexType linkCount)
    {
        for (IndexType i = 0; i < linkCount; ++i)
        {
            links[i].m_adj[0] = invalidIndex<IndexType>();
            links[i].m_adj[1] = invalidIndex<IndexType>();
        }
    }

    void        initLinksChain(IndexDLink<IndexType>* links, IndexType linkCount)
    {
        if (linkCount > 0)
        {
            links[0].m_adj[0] = invalidIndex<IndexType>();
            for (IndexType i = 1; i < linkCount; ++i)
            {
                links[i - 1].m_adj[1] = i;
                links[i].m_adj[0] = i - 1;
            }
            links[linkCount - 1].m_adj[1] = invalidIndex<IndexType>();
        }
    }

    IndexType   getAdj(IndexDLink<IndexType>* links, IndexType linkIndex, int which)
    {
        return links[linkIndex].m_adj[which & 1];
    }

    void        remove(IndexDLink<IndexType>* links, IndexType linkIndex)
    {
        IndexDLink<IndexType>& link = links[linkIndex];
        const IndexType adj0 = link.m_adj[0];
        const IndexType adj1 = link.m_adj[1];
        if (!isInvalidIndex(adj1))
        {
            links[adj1].m_adj[0] = adj0;
            link.m_adj[1] = invalidIndex<IndexType>();
        }
        if (!isInvalidIndex(adj0))
        {
            links[adj0].m_adj[1] = adj1;
            link.m_adj[0] = invalidIndex<IndexType>();
        }
    }

    bool        isSolitary(IndexDLink<IndexType>* links, IndexType linkIndex)
    {
        const IndexDLink<IndexType>& link = links[linkIndex];
        return isInvalidIndex(link.m_adj[0]) && isInvalidIndex(link.m_adj[1]);
    }

    void        insertListHead(IndexType& listHead, IndexDLink<IndexType>* links, IndexType linkIndex)
    {
        NVBLAST_ASSERT(!isInvalidIndex(linkIndex));
        if (!isInvalidIndex(listHead))
        {
            links[listHead].m_adj[0] = linkIndex;
        }
        links[linkIndex].m_adj[1] = listHead;
        listHead = linkIndex;
    }

    IndexType   removeListHead(IndexType& listHead, IndexDLink<IndexType>* links)
    {
        const IndexType linkIndex = listHead;
        if (!isInvalidIndex(linkIndex))
        {
            listHead = links[linkIndex].m_adj[1];
            if (!isInvalidIndex(listHead))
            {
                links[listHead].m_adj[0] = invalidIndex<IndexType>();
            }
            links[linkIndex].m_adj[1] = invalidIndex<IndexType>();
        }
        return linkIndex;
    }

    void        removeFromList(IndexType& listHead, IndexDLink<IndexType>* links, IndexType linkIndex)
    {
        NVBLAST_ASSERT(!isInvalidIndex(linkIndex));
        if (listHead == linkIndex)
        {
            listHead = links[linkIndex].m_adj[1];
        }
        remove(links, linkIndex);
    }
};


struct DLink
{
    DLink() : m_prev(nullptr), m_next(nullptr) {}

    DLink*  getPrev() const
    {
        return m_prev;
    }

    DLink*  getNext() const
    {
        return m_next;
    }

private:
    DLink*  m_prev;
    DLink*  m_next;

    friend class DList;
};


class DList
{
public:
    DList() : m_head(nullptr), m_tail(nullptr) {}

    bool    isEmpty() const
    {
        NVBLAST_ASSERT((m_head == nullptr) == (m_tail == nullptr));
        return m_head == nullptr;
    }

    bool    isSolitary(const DLink& link) const
    {
        return link.m_prev == nullptr && link.m_next == nullptr && m_head != &link;
    }

    DLink*  getHead() const
    {
        return m_head;
    }

    DLink*  getTail() const
    {
        return m_tail;
    }

    bool    insertHead(DLink& link)
    {
        NVBLAST_ASSERT(isSolitary(link));
        if (!isSolitary(link))
        {
            return false;
        }

        link.m_next = m_head;
        if (m_head != nullptr)
        {
            m_head->m_prev = &link;
        }
        m_head = &link;
        if (m_tail == nullptr)
        {
            m_tail = &link;
        }

        return true;
    }

    bool    insertTail(DLink& link)
    {
        NVBLAST_ASSERT(isSolitary(link));
        if (!isSolitary(link))
        {
            return false;
        }

        link.m_prev = m_tail;
        if (m_tail != nullptr)
        {
            m_tail->m_next = &link;
        }
        m_tail = &link;
        if (m_head == nullptr)
        {
            m_head = &link;
        }

        return true;
    }

    void    remove(DLink& link)
    {
        if (link.m_prev != nullptr)
        {
            link.m_prev->m_next = link.m_next;
        }
        else
        if (m_head == &link)
        {
            m_head = link.m_next;
        }

        if (link.m_next != nullptr)
        {
            link.m_next->m_prev = link.m_prev;
        }
        else
        if (m_tail == &link)
        {
            m_tail = link.m_prev;
        }

        link.m_next = link.m_prev = nullptr;
    }

    class It
    {
    public:
        enum Direction { Reverse, Forward };

        It(const DList& list, Direction dir = Forward) : m_curr(dir == Forward ? list.getHead() : list.getTail()) {}

        /** Validity of current value. */
        operator bool() const
        {
            return m_curr != nullptr;
        }

        /** Current value. */
        operator const DLink*() const
        {
            return m_curr;
        }

        /** Pre-increment. */
        const DLink*    operator ++ ()
        {
            return m_curr = m_curr->getNext();
        }

        /** Pre-deccrement. */
        const DLink*    operator -- ()
        {
            return m_curr = m_curr->getPrev();
        }

    private:
        const DLink*    m_curr;
    };

private:
    DLink*  m_head;
    DLink*  m_tail;
};

} // end namespace Blast
} // end namespace Nv


#endif // #ifndef NVBLASTDLINK_H

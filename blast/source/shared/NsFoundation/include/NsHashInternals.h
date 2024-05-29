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

#ifndef NV_NSFOUNDATION_NSHASHINTERNALS_H
#define NV_NSFOUNDATION_NSHASHINTERNALS_H

#include "NsBasicTemplates.h"
#include "NsArray.h"
#include "NsBitUtils.h"
#include "NsHash.h"
#include "NvIntrinsics.h"

#if NV_VC
#pragma warning(push)
#pragma warning(disable : 4127) // conditional expression is constant
#endif
namespace nvidia
{
namespace shdfnd
{
namespace internal
{
template <class Entry, class Key, class HashFn, class GetKey, class Allocator, bool compacting>
class HashBase : private Allocator
{
    void init(uint32_t initialTableSize, float loadFactor)
    {
        mBuffer = NULL;
        mEntries = NULL;
        mEntriesNext = NULL;
        mHash = NULL;
        mEntriesCapacity = 0;
        mHashSize = 0;
        mLoadFactor = loadFactor;
        mFreeList = uint32_t(EOL);
        mTimestamp = 0;
        mEntriesCount = 0;

        if(initialTableSize)
            reserveInternal(initialTableSize);
    }

  public:
    typedef Entry EntryType;

    HashBase(uint32_t initialTableSize = 64, float loadFactor = 0.75f) : Allocator(NV_DEBUG_EXP("hashBase"))
    {
        init(initialTableSize, loadFactor);
    }

    HashBase(uint32_t initialTableSize, float loadFactor, const Allocator& alloc) : Allocator(alloc)
    {
        init(initialTableSize, loadFactor);
    }

    HashBase(const Allocator& alloc) : Allocator(alloc)
    {
        init(64, 0.75f);
    }

    ~HashBase()
    {
        destroy(); // No need to clear()

        if(mBuffer)
            Allocator::deallocate(mBuffer);
    }

    static const uint32_t EOL = 0xffffffff;

    NV_INLINE Entry* create(const Key& k, bool& exists)
    {
        uint32_t h = 0;
        if(mHashSize)
        {
            h = hash(k);
            uint32_t index = mHash[h];
            while(index != EOL && !HashFn().equal(GetKey()(mEntries[index]), k))
                index = mEntriesNext[index];
            exists = index != EOL;
            if(exists)
                return mEntries + index;
        }
        else
            exists = false;

        if(freeListEmpty())
        {
            grow();
            h = hash(k);
        }

        uint32_t entryIndex = freeListGetNext();

        mEntriesNext[entryIndex] = mHash[h];
        mHash[h] = entryIndex;

        mEntriesCount++;
        mTimestamp++;

        return mEntries + entryIndex;
    }

    NV_INLINE const Entry* find(const Key& k) const
    {
        if(!mHashSize)
            return NULL;

        const uint32_t h = hash(k);
        uint32_t index = mHash[h];
        while(index != EOL && !HashFn().equal(GetKey()(mEntries[index]), k))
            index = mEntriesNext[index];
        return index != EOL ? mEntries + index : NULL;
    }

    NV_INLINE bool erase(const Key& k)
    {
        if(!mHashSize)
            return false;

        const uint32_t h = hash(k);
        uint32_t* ptr = mHash + h;
        while(*ptr != EOL && !HashFn().equal(GetKey()(mEntries[*ptr]), k))
            ptr = mEntriesNext + *ptr;

        if(*ptr == EOL)
            return false;

        const uint32_t index = *ptr;
        *ptr = mEntriesNext[index];

        mEntries[index].~Entry();

        mEntriesCount--;
        mTimestamp++;

        if(compacting && index != mEntriesCount)
            replaceWithLast(index);

        freeListAdd(index);

        return true;
    }

    NV_INLINE uint32_t size() const
    {
        return mEntriesCount;
    }

    NV_INLINE uint32_t capacity() const
    {
        return mHashSize;
    }

    void clear()
    {
        if(!mHashSize || mEntriesCount == 0)
            return;

        destroy();

        intrinsics::memSet(mHash, EOL, mHashSize * sizeof(uint32_t));

        const uint32_t sizeMinus1 = mEntriesCapacity - 1;
        for(uint32_t i = 0; i < sizeMinus1; i++)
        {
            prefetchLine(mEntriesNext + i, 128);
            mEntriesNext[i] = i + 1;
        }
        mEntriesNext[mEntriesCapacity - 1] = uint32_t(EOL);
        mFreeList = 0;
        mEntriesCount = 0;
    }

    void reserve(uint32_t size)
    {
        if(size > mHashSize)
            reserveInternal(size);
    }

    NV_INLINE const Entry* getEntries() const
    {
        return mEntries;
    }

    NV_INLINE Entry* insertUnique(const Key& k)
    {
        NV_ASSERT(find(k) == NULL);
        uint32_t h = hash(k);

        uint32_t entryIndex = freeListGetNext();

        mEntriesNext[entryIndex] = mHash[h];
        mHash[h] = entryIndex;

        mEntriesCount++;
        mTimestamp++;

        return mEntries + entryIndex;
    }

  private:
    void destroy()
    {
        for(uint32_t i = 0; i < mHashSize; i++)
        {
            for(uint32_t j = mHash[i]; j != EOL; j = mEntriesNext[j])
                mEntries[j].~Entry();
        }
    }

    template <typename HK, typename GK, class A, bool comp>
    NV_NOINLINE void copy(const HashBase<Entry, Key, HK, GK, A, comp>& other);

    // free list management - if we're coalescing, then we use mFreeList to hold
    // the top of the free list and it should always be equal to size(). Otherwise,
    // we build a free list in the next() pointers.

    NV_INLINE void freeListAdd(uint32_t index)
    {
        if(compacting)
        {
            mFreeList--;
            NV_ASSERT(mFreeList == mEntriesCount);
        }
        else
        {
            mEntriesNext[index] = mFreeList;
            mFreeList = index;
        }
    }

    NV_INLINE void freeListAdd(uint32_t start, uint32_t end)
    {
        if(!compacting)
        {
            for(uint32_t i = start; i < end - 1; i++) // add the new entries to the free list
                mEntriesNext[i] = i + 1;

            // link in old free list
            mEntriesNext[end - 1] = mFreeList;
            NV_ASSERT(mFreeList != end - 1);
            mFreeList = start;
        }
        else if(mFreeList == EOL) // don't reset the free ptr for the compacting hash unless it's empty
            mFreeList = start;
    }

    NV_INLINE uint32_t freeListGetNext()
    {
        NV_ASSERT(!freeListEmpty());
        if(compacting)
        {
            NV_ASSERT(mFreeList == mEntriesCount);
            return mFreeList++;
        }
        else
        {
            uint32_t entryIndex = mFreeList;
            mFreeList = mEntriesNext[mFreeList];
            return entryIndex;
        }
    }

    NV_INLINE bool freeListEmpty() const
    {
        if(compacting)
            return mEntriesCount == mEntriesCapacity;
        else
            return mFreeList == EOL;
    }

    NV_INLINE void replaceWithLast(uint32_t index)
    {
        NV_PLACEMENT_NEW(mEntries + index, Entry)(mEntries[mEntriesCount]);
        mEntries[mEntriesCount].~Entry();
        mEntriesNext[index] = mEntriesNext[mEntriesCount];

        uint32_t h = hash(GetKey()(mEntries[index]));
        uint32_t* ptr;
        for(ptr = mHash + h; *ptr != mEntriesCount; ptr = mEntriesNext + *ptr)
            NV_ASSERT(*ptr != EOL);
        *ptr = index;
    }

    NV_INLINE uint32_t hash(const Key& k, uint32_t hashSize) const
    {
        return HashFn()(k) & (hashSize - 1);
    }

    NV_INLINE uint32_t hash(const Key& k) const
    {
        return hash(k, mHashSize);
    }

    void reserveInternal(uint32_t size)
    {
        if(!isPowerOfTwo(size))
            size = nextPowerOfTwo(size);

        NV_ASSERT(!(size & (size - 1)));

        // decide whether iteration can be done on the entries directly
        bool resizeCompact = compacting || freeListEmpty();

        // define new table sizes
        uint32_t oldEntriesCapacity = mEntriesCapacity;
        uint32_t newEntriesCapacity = uint32_t(float(size) * mLoadFactor);
        uint32_t newHashSize = size;

        // allocate new common buffer and setup pointers to new tables
        uint8_t* newBuffer;
        uint32_t* newHash;
        uint32_t* newEntriesNext;
        Entry* newEntries;
        {
            uint32_t newHashByteOffset = 0;
            uint32_t newEntriesNextBytesOffset = newHashByteOffset + newHashSize * sizeof(uint32_t);
            uint32_t newEntriesByteOffset = newEntriesNextBytesOffset + newEntriesCapacity * sizeof(uint32_t);
            newEntriesByteOffset += (16 - (newEntriesByteOffset & 15)) & 15;
            uint32_t newBufferByteSize = newEntriesByteOffset + newEntriesCapacity * sizeof(Entry);

            newBuffer = reinterpret_cast<uint8_t*>(Allocator::allocate(newBufferByteSize, __FILE__, __LINE__));
            NV_ASSERT(newBuffer);

            newHash = reinterpret_cast<uint32_t*>(newBuffer + newHashByteOffset);
            newEntriesNext = reinterpret_cast<uint32_t*>(newBuffer + newEntriesNextBytesOffset);
            newEntries = reinterpret_cast<Entry*>(newBuffer + newEntriesByteOffset);
        }

        // initialize new hash table
        intrinsics::memSet(newHash, uint32_t(EOL), newHashSize * sizeof(uint32_t));

        // iterate over old entries, re-hash and create new entries
        if(resizeCompact)
        {
            // check that old free list is empty - we don't need to copy the next entries
            NV_ASSERT(compacting || mFreeList == EOL);

            for(uint32_t index = 0; index < mEntriesCount; ++index)
            {
                uint32_t h = hash(GetKey()(mEntries[index]), newHashSize);
                newEntriesNext[index] = newHash[h];
                newHash[h] = index;

                NV_PLACEMENT_NEW(newEntries + index, Entry)(mEntries[index]);
                mEntries[index].~Entry();
            }
        }
        else
        {
            // copy old free list, only required for non compact resizing
            intrinsics::memCopy(newEntriesNext, mEntriesNext, mEntriesCapacity * sizeof(uint32_t));

            for(uint32_t bucket = 0; bucket < mHashSize; bucket++)
            {
                uint32_t index = mHash[bucket];
                while(index != EOL)
                {
                    uint32_t h = hash(GetKey()(mEntries[index]), newHashSize);
                    newEntriesNext[index] = newHash[h];
                    NV_ASSERT(index != newHash[h]);

                    newHash[h] = index;

                    NV_PLACEMENT_NEW(newEntries + index, Entry)(mEntries[index]);
                    mEntries[index].~Entry();

                    index = mEntriesNext[index];
                }
            }
        }

        // swap buffer and pointers
        Allocator::deallocate(mBuffer);
        mBuffer = newBuffer;
        mHash = newHash;
        mHashSize = newHashSize;
        mEntriesNext = newEntriesNext;
        mEntries = newEntries;
        mEntriesCapacity = newEntriesCapacity;

        freeListAdd(oldEntriesCapacity, newEntriesCapacity);
    }

    void grow()
    {
        NV_ASSERT((mFreeList == EOL) || (compacting && (mEntriesCount == mEntriesCapacity)));

        uint32_t size = mHashSize == 0 ? 16 : mHashSize * 2;
        reserve(size);
    }

    uint8_t* mBuffer;
    Entry* mEntries;
    uint32_t* mEntriesNext; // same size as mEntries
    uint32_t* mHash;
    uint32_t mEntriesCapacity;
    uint32_t mHashSize;
    float mLoadFactor;
    uint32_t mFreeList;
    uint32_t mTimestamp;
    uint32_t mEntriesCount; // number of entries

  public:
    class Iter
    {
      public:
        NV_INLINE Iter(HashBase& b) : mBucket(0), mEntry(uint32_t(b.EOL)), mTimestamp(b.mTimestamp), mBase(b)
        {
            if(mBase.mEntriesCapacity > 0)
            {
                mEntry = mBase.mHash[0];
                skip();
            }
        }

        NV_INLINE void check() const
        {
            NV_ASSERT(mTimestamp == mBase.mTimestamp);
        }
        NV_INLINE Entry operator*() const
        {
            check();
            return mBase.mEntries[mEntry];
        }
        NV_INLINE Entry* operator->() const
        {
            check();
            return mBase.mEntries + mEntry;
        }
        NV_INLINE Iter operator++()
        {
            check();
            advance();
            return *this;
        }
        NV_INLINE Iter operator++(int)
        {
            check();
            Iter i = *this;
            advance();
            return i;
        }
        NV_INLINE bool done() const
        {
            check();
            return mEntry == mBase.EOL;
        }

      private:
        NV_INLINE void advance()
        {
            mEntry = mBase.mEntriesNext[mEntry];
            skip();
        }
        NV_INLINE void skip()
        {
            while(mEntry == mBase.EOL)
            {
                if(++mBucket == mBase.mHashSize)
                    break;
                mEntry = mBase.mHash[mBucket];
            }
        }

        Iter& operator=(const Iter&);

        uint32_t mBucket;
        uint32_t mEntry;
        uint32_t mTimestamp;
        HashBase& mBase;
    };
};

template <class Entry, class Key, class HashFn, class GetKey, class Allocator, bool compacting>
template <typename HK, typename GK, class A, bool comp>
NV_NOINLINE void
HashBase<Entry, Key, HashFn, GetKey, Allocator, compacting>::copy(const HashBase<Entry, Key, HK, GK, A, comp>& other)
{
    reserve(other.mEntriesCount);

    for(uint32_t i = 0; i < other.mEntriesCount; i++)
    {
        for(uint32_t j = other.mHash[i]; j != EOL; j = other.mEntriesNext[j])
        {
            const Entry& otherEntry = other.mEntries[j];

            bool exists;
            Entry* newEntry = create(GK()(otherEntry), exists);
            NV_ASSERT(!exists);

            NV_PLACEMENT_NEW(newEntry, Entry)(otherEntry);
        }
    }
}

template <class Key, class HashFn, class Allocator = typename AllocatorTraits<Key>::Type, bool Coalesced = false>
class HashSetBase
{
    NV_NOCOPY(HashSetBase)
  public:
    struct GetKey
    {
        NV_INLINE const Key& operator()(const Key& e)
        {
            return e;
        }
    };

    typedef HashBase<Key, Key, HashFn, GetKey, Allocator, Coalesced> BaseMap;
    typedef typename BaseMap::Iter Iterator;

    HashSetBase(uint32_t initialTableSize, float loadFactor, const Allocator& alloc)
    : mBase(initialTableSize, loadFactor, alloc)
    {
    }

    HashSetBase(const Allocator& alloc) : mBase(64, 0.75f, alloc)
    {
    }

    HashSetBase(uint32_t initialTableSize = 64, float loadFactor = 0.75f) : mBase(initialTableSize, loadFactor)
    {
    }

    bool insert(const Key& k)
    {
        bool exists;
        Key* e = mBase.create(k, exists);
        if(!exists)
            NV_PLACEMENT_NEW(e, Key)(k);
        return !exists;
    }

    NV_INLINE bool contains(const Key& k) const
    {
        return mBase.find(k) != 0;
    }
    NV_INLINE bool erase(const Key& k)
    {
        return mBase.erase(k);
    }
    NV_INLINE uint32_t size() const
    {
        return mBase.size();
    }
    NV_INLINE uint32_t capacity() const
    {
        return mBase.capacity();
    }
    NV_INLINE void reserve(uint32_t size)
    {
        mBase.reserve(size);
    }
    NV_INLINE void clear()
    {
        mBase.clear();
    }

  protected:
    BaseMap mBase;
};

template <class Key, class Value, class HashFn, class Allocator = typename AllocatorTraits<Pair<const Key, Value> >::Type>
class HashMapBase
{
    NV_NOCOPY(HashMapBase)
  public:
    typedef Pair<const Key, Value> Entry;

    struct GetKey
    {
        NV_INLINE const Key& operator()(const Entry& e)
        {
            return e.first;
        }
    };

    typedef HashBase<Entry, Key, HashFn, GetKey, Allocator, true> BaseMap;
    typedef typename BaseMap::Iter Iterator;

    HashMapBase(uint32_t initialTableSize, float loadFactor, const Allocator& alloc)
    : mBase(initialTableSize, loadFactor, alloc)
    {
    }

    HashMapBase(const Allocator& alloc) : mBase(64, 0.75f, alloc)
    {
    }

    HashMapBase(uint32_t initialTableSize = 64, float loadFactor = 0.75f) : mBase(initialTableSize, loadFactor)
    {
    }

    bool insert(const Key /*&*/ k, const Value /*&*/ v)
    {
        bool exists;
        Entry* e = mBase.create(k, exists);
        if(!exists)
            NV_PLACEMENT_NEW(e, Entry)(k, v);
        return !exists;
    }

    Value& operator[](const Key& k)
    {
        bool exists;
        Entry* e = mBase.create(k, exists);
        if(!exists)
            NV_PLACEMENT_NEW(e, Entry)(k, Value());

        return e->second;
    }

    NV_INLINE const Entry* find(const Key& k) const
    {
        return mBase.find(k);
    }
    NV_INLINE bool erase(const Key& k)
    {
        return mBase.erase(k);
    }
    NV_INLINE uint32_t size() const
    {
        return mBase.size();
    }
    NV_INLINE uint32_t capacity() const
    {
        return mBase.capacity();
    }
    NV_INLINE Iterator getIterator()
    {
        return Iterator(mBase);
    }
    NV_INLINE void reserve(uint32_t size)
    {
        mBase.reserve(size);
    }
    NV_INLINE void clear()
    {
        mBase.clear();
    }

  protected:
    BaseMap mBase;
};
}

} // namespace shdfnd
} // namespace nvidia

#if NV_VC
#pragma warning(pop)
#endif
#endif // #ifndef NV_NSFOUNDATION_NSHASHINTERNALS_H

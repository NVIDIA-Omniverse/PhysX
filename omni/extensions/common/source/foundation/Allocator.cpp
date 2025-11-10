// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include <carb/logging/Log.h>

#include <cassert>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>

#ifdef _WIN32
#    include <crtdbg.h>
#endif

#include "Allocator.h"

#ifdef _MSC_VER
#    pragma warning(disable : 4996)
#    pragma warning(disable : 4302)
#    pragma warning(disable : 4311)
#    pragma warning(disable : 4312)
#    pragma warning(disable : 4477)
#endif

#define INVALID_ID 0xffffffff
#define ASSERT assert

//#define ZERO_OVERHEAD_RELEASE
#ifdef _WIN32
    #define SIMD_ALLOC
    #define FOUNDATION_FORCE_INLINE __forceinline
#else
    // On GCC we get warning: always_inline function might not be inlinable [-Wattributes] and this is only used on _DEBUG anyway
    #define FOUNDATION_FORCE_INLINE
#endif
#define LOCAL_MALLOC ::malloc
#define LOCAL_FREE ::free

// WARNING: this makes allocations a lot slower. Only use when tracking memory leaks.
//#define ALLOC_STRINGS

#define DEBUG_IDENTIFIER 0xBeefBabe
#define DEBUG_DEALLOCATED 0xDeadDead

FOUNDATION_FORCE_INLINE void AZeroMemory(void* addr, size_t size)
{
    memset(addr, 0, size);
}

FOUNDATION_FORCE_INLINE void ACopyMemory(void* dest, const void* src, size_t size)
{
    memcpy(dest, src, size);
}

#ifdef ALLOC_STRINGS
static const char* AllocString(const char* str)
{
    if (!str)
        return nullptr;
    char* mem = (char*)LOCAL_MALLOC(strlen(str) + 1);
    strcpy(mem, str);
    return mem;
}

static void FreeString(const char* str)
{
    if (str)
        LOCAL_FREE((void*)str);
}

#endif

class DefaultAllocator : public Allocator
{
public:
                    DefaultAllocator();
    virtual         ~DefaultAllocator() override;

            void    reset();

    virtual void*   malloc(size_t size) override;
    virtual void*   mallocDebug(size_t size, const char* filename, int line, const char* className, bool fromNew) override;
    virtual void    free(void* memory, bool fromNew) override;

            void    Release();

private:
    void**      mMemBlockList;
    uint32_t    mMemBlockListSize;
    uint32_t    mFirstFree;
    uint32_t    mMemBlockUsed;

    int32_t     mNbAllocatedBytes;
    int32_t     mHighWaterMark;
    int32_t     mTotalNbAllocs;
    int32_t     mNbAllocs;
    int32_t     mNbReallocs;
    int32_t     mPreviousTotalNbAllocs;
    int32_t     mPreviousNbAllocs;
    uint32_t    mFrameCount;
    bool        mNoLeak;
};

#define MEMBLOCKSTART 64

struct DebugBlock
{
    uint32_t    mCheckValue;
    uint32_t    mSize;
    const char* mFilename;
    uint32_t    mLine;
    uint32_t    mSlotIndex;
    const char* mClassName;
    int         mFromNew;
};

DefaultAllocator::DefaultAllocator() :
    mMemBlockList       (nullptr),
    mNbAllocatedBytes   (0),
    mHighWaterMark      (0),
    mTotalNbAllocs      (0),
    mNbAllocs           (0),
    mNbReallocs         (0)
{

#ifdef _DEBUG
    // Initialize the Memory blocks list (DEBUG mode only)
    mMemBlockList = (void**)LOCAL_MALLOC(MEMBLOCKSTART * sizeof(void*));
    AZeroMemory(mMemBlockList, MEMBLOCKSTART * sizeof(void*));
    mMemBlockListSize = MEMBLOCKSTART;
    mFirstFree = INVALID_ID;
    mMemBlockUsed = 0;
#endif

    mPreviousTotalNbAllocs = 0;
    mPreviousNbAllocs = 0;
    mFrameCount = 0;
    mNoLeak = true;
}

DefaultAllocator::~DefaultAllocator()
{
    Release();
}

void DefaultAllocator::Release()
{
#ifdef _DEBUG
//	if(mNbAllocatedBytes)
    {
        CARB_LOG_INFO("Memory leak detected: %d bytes non released\n", mNbAllocatedBytes);
    }
    if (mNbAllocs)
    {
        CARB_LOG_WARN("Remaining allocs: %d\n", mNbAllocs);
    }
    CARB_LOG_INFO("Total nb alloc: %d\n", mTotalNbAllocs);
    CARB_LOG_INFO("Nb realloc: %d\n", mNbReallocs);
    CARB_LOG_INFO("High water mark: %d Kb\n", mHighWaterMark / 1024);

    // Scanning for memory leaks
    if (mMemBlockList && mNbAllocs)
    {
        uint32_t NbLeaks = 0;
        CARB_LOG_WARN("\n\n  ICE Message Memory leaks detected :\n\n");

        for (uint32_t i = 0; i < mMemBlockUsed; i++)
        {
            if (size_t(mMemBlockList[i]) & 1)
                continue;

            const DebugBlock* DB = (const DebugBlock*)mMemBlockList[i];
//			IceTrace(_F(" Address 0x%.8X, %d bytes (%s), allocated in: %s(%d):\n\n", cur+6, cur[1], (const
            // char*)cur[5], (const char*)cur[2], cur[3])); 			IceTrace(_F(" Address 0x%.8X, %d bytes (%s),
            // allocated in: %s(%d):\n\n", DB+1, DB->mSize, DB->mClassName, DB->mFilename, DB->mLine));
            CARB_LOG_WARN(" Address 0x%.8" PRIXPTR ", %d bytes (%s), allocated in: %s(%d):\n\n", DB + 1, DB->mSize,
                          DB->mClassName, DB->mFilename, DB->mLine);

            NbLeaks++;
//			Free(cur+4);
        }

//		IceTrace(_F("\n  Dump complete (%d leaks)\n\n", NbLeaks));
        CARB_LOG_WARN("\n  Dump complete (%d leaks)\n\n", NbLeaks);
    }
    // Free the Memory Block list
    if (mMemBlockList)
        LOCAL_FREE(mMemBlockList);
    mMemBlockList = nullptr;
#endif
}

void DefaultAllocator::reset()
{
    mNbAllocatedBytes = 0;
    mHighWaterMark = 0;
    mNbAllocs = 0;
}

void* DefaultAllocator::malloc(size_t size)
{
//	return ::malloc(size);

#ifdef _DEBUG
    return mallocDebug(size, nullptr, 0, "Undefined", false);
#endif

    if (!size)
    {
#ifdef _DEBUG
        CARB_LOG_WARN("Warning: trying to allocate 0 bytes\n");
#endif
        return nullptr;
    }

    mTotalNbAllocs++;
    mNbAllocs++;

    mNbAllocatedBytes += int32_t(size);
    if (mNbAllocatedBytes > mHighWaterMark)
        mHighWaterMark = mNbAllocatedBytes;

#ifdef ZERO_OVERHEAD_RELEASE
    return LOCAL_MALLOC(size);
#else
    #ifdef SIMD_ALLOC
    void* ptr = _aligned_malloc(size + 16, 16);
    #else
    void* ptr = (void*)LOCAL_MALLOC(size + 8);
    #endif

    uint32_t* blockStart = (uint32_t*)ptr;
    blockStart[0] = DEBUG_IDENTIFIER;
    blockStart[1] = uint32_t(size);

    #ifdef SIMD_ALLOC
    return ((uint32_t*)ptr) + 4;
    #else
    return ((uint32_t*)ptr) + 2;
    #endif
#endif
}

void* DefaultAllocator::mallocDebug(size_t size, const char* filename, int line, const char* class_name, bool from_new)
{
//	printf("%s (%d)\n", filename, line);

#ifdef _DEBUG
    if (!size)
    {
        CARB_LOG_WARN("Warning: trying to allocate 0 bytes\n");
        return nullptr;
    }

    // Catch improper use of alloc macro...
    if (0 && class_name)
    {
        const char* c = class_name;
        while (*c)
        {
            if (*c == ']' || *c == '[')
            {
                int stop = 0;
                CARB_UNUSED(stop);
            }
            c++;
        }
    }

    // Make sure size is even
    if (size & 1)
        size++;

    // Allocate one debug block in front of each real allocation
    void* ptr = (void*)LOCAL_MALLOC(size + sizeof(DebugBlock));
    //	ASSERT(IS_ALIGNED_2(size_t(ptr)));

    // Fill debug block
    DebugBlock* DB = (DebugBlock*)ptr;
    DB->mCheckValue = DEBUG_IDENTIFIER;
    DB->mFromNew = from_new;
    DB->mSize = uint32_t(size);
    DB->mLine = line;
    DB->mSlotIndex = INVALID_ID;

    #ifdef ALLOC_STRINGS
    DB->mFilename = AllocString(filename);
    DB->mClassName = AllocString(class_name);
    #else
    DB->mFilename = filename;
    DB->mClassName = class_name;
    #endif

    // Update global stats
    mTotalNbAllocs++;
    mNbAllocs++;
    mNbAllocatedBytes += int32_t(size);
    if (mNbAllocatedBytes > mHighWaterMark)
        mHighWaterMark = mNbAllocatedBytes;

    // Insert the allocated block in the debug memory block list
    if (mMemBlockList)
    {
        if (mFirstFree != INVALID_ID)
        {
            // Recycle old location
    #ifdef _WIN32
            uint32_t NextFree = uint32_t(mMemBlockList[mFirstFree]);
    #else
            // HLL Fixme
            uintptr_t NextFree = uintptr_t(mMemBlockList[mFirstFree]);
    #endif
            if (NextFree != INVALID_ID)
                NextFree >>= 1;

            mMemBlockList[mFirstFree] = ptr;
            DB->mSlotIndex = mFirstFree;

            mFirstFree = NextFree;
        }
        else
        {
            if (mMemBlockUsed == mMemBlockListSize)
            {
                // Allocate a bigger block
                void** tps = (void**)LOCAL_MALLOC((mMemBlockListSize + MEMBLOCKSTART) * sizeof(void*));
                // Copy already used part
                ACopyMemory(tps, mMemBlockList, mMemBlockListSize * sizeof(void*));
                // Initialize remaining part
                void* Next = tps + mMemBlockListSize;
                AZeroMemory(Next, MEMBLOCKSTART * sizeof(void*));

                // Free previous memory, setup new pointer
                LOCAL_FREE(mMemBlockList);
                mMemBlockList = tps;
                // Setup new size
                mMemBlockListSize += MEMBLOCKSTART;
            }

            mMemBlockList[mMemBlockUsed] = ptr;
            DB->mSlotIndex = mMemBlockUsed++;
        }
    }

    return ((unsigned char*)ptr) + sizeof(DebugBlock);
#else
    //	Log("Error: mallocDebug has been called in release!\n");
    ASSERT(0); // Don't use debug malloc for release mode code!
    return 0;
#endif
}

void DefaultAllocator::free(void* memory, bool from_new)
{
    if (!memory)
    {
#ifdef _DEBUG
        CARB_LOG_WARN("Warning: trying to free null pointer\n");
#endif
        return;
    }

#ifdef _DEBUG
    DebugBlock* DB = ((DebugBlock*)memory) - 1;

    //	DebugBlock TmpDB = *DB;	// Keep a local copy to have readable data when ::free() fails!

    // Check we allocated it
    if (DB->mCheckValue != DEBUG_IDENTIFIER)
    {
        CARB_LOG_WARN("Error: free unknown memory!!\n");
        // ### should we really continue??
        return;
    }

    ASSERT(int(from_new) == DB->mFromNew);

    // Update global stats
    mNbAllocatedBytes -= DB->mSize;
    mNbAllocs--;

    // Remove the block from the Memory block list
    if (mMemBlockList)
    {
        uint32_t FreeSlot = DB->mSlotIndex;
        ASSERT(mMemBlockList[FreeSlot] == DB);

        uint32_t NextFree = mFirstFree;
        if (NextFree != INVALID_ID)
        {
            NextFree <<= 1;
            NextFree |= 1;
        }

        mMemBlockList[FreeSlot] = (void*)(uintptr_t)NextFree;
        mFirstFree = FreeSlot;
    }

    #ifdef ALLOC_STRINGS
    FreeString(DB->mClassName);
    FreeString(DB->mFilename);
    #endif

    // ### should be useless since we'll release the memory just afterwards
    DB->mCheckValue = DEBUG_DEALLOCATED;
    DB->mSize = 0;
    DB->mClassName = nullptr;
    DB->mFilename = nullptr;
    DB->mSlotIndex = INVALID_ID;
    DB->mLine = INVALID_ID;

    LOCAL_FREE(DB);
#else
// Release codepath
    #ifdef ZERO_OVERHEAD_RELEASE
    //	mNbAllocatedBytes -= ptr[1];	// ### use _msize() ?
    mNbAllocs--;
    LOCAL_FREE(memory);
    #else
        #ifdef SIMD_ALLOC
    uint32_t* ptr = ((uint32_t*)memory) - 4;
        #else
    uint32_t* ptr = ((uint32_t*)memory) - 2;
        #endif

    if (ptr[0] != DEBUG_IDENTIFIER)
    {
#ifdef _DEBUG
        CARB_LOG_WARN("Error: free unknown memory!!\n");
#endif
    }
    mNbAllocatedBytes -= ptr[1];
    if (mNbAllocatedBytes < 0)
    {
#ifdef _DEBUG
        CARB_LOG_WARN(_F("Oops (%d)\n", ptr[1]));
#endif
    }
    mNbAllocs--;
    ptr[0] = DEBUG_DEALLOCATED;
    ptr[1] = 0;

        #ifdef SIMD_ALLOC
    _aligned_free(ptr);
        #else
    LOCAL_FREE(ptr);
        #endif

    #endif
#endif
}


// anonymous namespace due to a bit of paranoia about Linux. Symbols should be hidden by default
// for the projects using this library but still. The anonymous namespace does create a separate
// instance in each library (not officially documented/confirmed but shown in a nice post).
namespace
{
    static DefaultAllocator gAllocator;
}

Allocator* GetAllocator()
{
    return &gAllocator;
}

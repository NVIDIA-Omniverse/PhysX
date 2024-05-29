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

#ifndef NV_NSFOUNDATION_NSALLOCATOR_H
#define NV_NSFOUNDATION_NSALLOCATOR_H

#include "NvAllocatorCallback.h"
#include "Ns.h"
#include "NsGlobals.h"

#if(NV_WINDOWS_FAMILY || NV_WINRT || NV_X360 || NV_XBOXONE)
#include <exception>
#include <typeinfo.h>
#endif
#if(NV_APPLE_FAMILY)
#include <typeinfo>
#endif

#if NV_WIIU
#pragma ghs nowarning 193 // warning #193-D: zero used for undefined preprocessing identifier
#endif

#include <new>

#if NV_WIIU
#pragma ghs endnowarning
#endif

// Allocation macros going through user allocator
#if NV_CHECKED
#define NV_ALLOC(n, name) nvidia::shdfnd::NamedAllocator(name).allocate(n, __FILE__, __LINE__)
#else
#define NV_ALLOC(n, name) nvidia::shdfnd::NonTrackingAllocator().allocate(n, __FILE__, __LINE__)
#endif
#define NV_ALLOC_TEMP(n, name) NV_ALLOC(n, name)
#define NV_FREE(x) nvidia::shdfnd::NonTrackingAllocator().deallocate(x)
#define NV_FREE_AND_RESET(x)                                                                                           \
    {                                                                                                                  \
        NV_FREE(x);                                                                                                    \
        x = 0;                                                                                                         \
    }

// The following macros support plain-old-types and classes derived from UserAllocated.
#define NV_NEW(T) new (nvidia::shdfnd::ReflectionAllocator<T>(), __FILE__, __LINE__) T
#define NV_NEW_TEMP(T) NV_NEW(T)
#define NV_DELETE(x) delete x
#define NV_DELETE_AND_RESET(x)                                                                                         \
    {                                                                                                                  \
        NV_DELETE(x);                                                                                                  \
        x = 0;                                                                                                         \
    }
#define NV_DELETE_POD(x)                                                                                               \
    {                                                                                                                  \
        NV_FREE(x);                                                                                                    \
        x = 0;                                                                                                         \
    }
#define NV_DELETE_ARRAY(x)                                                                                             \
    {                                                                                                                  \
        NV_DELETE([] x);                                                                                               \
        x = 0;                                                                                                         \
    }

// aligned allocation
#define NV_ALIGNED16_ALLOC(n) nvidia::shdfnd::AlignedAllocator<16>().allocate(n, __FILE__, __LINE__)
#define NV_ALIGNED16_FREE(x) nvidia::shdfnd::AlignedAllocator<16>().deallocate(x)

//! placement new macro to make it easy to spot bad use of 'new'
#define NV_PLACEMENT_NEW(p, T) new (p) T

#if NV_DEBUG || NV_CHECKED
#define NV_USE_NAMED_ALLOCATOR 1
#else
#define NV_USE_NAMED_ALLOCATOR 0
#endif

// Don't use inline for alloca !!!
#if NV_WINDOWS_FAMILY || NV_WINRT
#include <malloc.h>
#define NvAlloca(x) _alloca(x)
#elif NV_LINUX || NV_ANDROID
#include <malloc.h>
#define NvAlloca(x) alloca(x)
#elif NV_PSP2
#include <alloca.h>
#define NvAlloca(x) alloca(x)
#elif NV_APPLE_FAMILY
#include <alloca.h>
#define NvAlloca(x) alloca(x)
#elif NV_PS3
#include <alloca.h>
#define NvAlloca(x) alloca(x)
#elif NV_X360
#include <malloc.h>
#define NvAlloca(x) _alloca(x)
#elif NV_WIIU
#include <alloca.h>
#define NvAlloca(x) alloca(x)
#elif NV_PS4
#include <memory.h>
#define NvAlloca(x) alloca(x)
#elif NV_XBOXONE
#include <malloc.h>
#define NvAlloca(x) alloca(x)
#endif

#define NvAllocaAligned(x, alignment) ((size_t(NvAlloca(x + alignment)) + (alignment - 1)) & ~size_t(alignment - 1))

namespace nvidia
{
namespace shdfnd
{
/*
 * Bootstrap allocator using malloc/free.
 * Don't use unless your objects get allocated before foundation is initialized.
 */
class RawAllocator
{
  public:
    RawAllocator(const char* = 0)
    {
    }
    void* allocate(size_t size, const char*, int)
    {
        // malloc returns valid pointer for size==0, no need to check
        return ::malloc(size);
    }
    void deallocate(void* ptr)
    {
        // free(0) is guaranteed to have no side effect, no need to check
        ::free(ptr);
    }
};

/*
 * Allocator that simply calls straight back to the application without tracking.
 * This is used by the heap (Foundation::mNamedAllocMap) that tracks allocations
 * because it needs to be able to grow as a result of an allocation.
 * Making the hash table re-entrant to deal with this may not make sense.
 */
class NonTrackingAllocator
{
  public:
    NV_FORCE_INLINE NonTrackingAllocator(const char* = 0)
    {
    }
    NV_FORCE_INLINE void* allocate(size_t size, const char* file, int line)
    {
        return !size ? 0 : getAllocator().allocate(size, "NonTrackedAlloc", file, line);
    }
    NV_FORCE_INLINE void deallocate(void* ptr)
    {
        if(ptr)
            getAllocator().deallocate(ptr);
    }
};

/**
Allocator used to access the global NvAllocatorCallback instance using a dynamic name.
*/
void initializeNamedAllocatorGlobals();
void terminateNamedAllocatorGlobals();

#if NV_USE_NAMED_ALLOCATOR // can be slow, so only use in debug/checked
class NV_FOUNDATION_API NamedAllocator
{
  public:
    NamedAllocator(const NvEMPTY);
    NamedAllocator(const char* name = 0); // todo: should not have default argument!
    NamedAllocator(const NamedAllocator&);
    ~NamedAllocator();
    NamedAllocator& operator=(const NamedAllocator&);
    void* allocate(size_t size, const char* filename, int line);
    void deallocate(void* ptr);
};
#else
class NamedAllocator;
#endif // NV_DEBUG

/**
Allocator used to access the global NvAllocatorCallback instance using a static name derived from T.
*/

template <typename T>
class ReflectionAllocator
{
    static const char* getName()
    {
        if(!getReflectionAllocatorReportsNames())
            return "<allocation names disabled>";
#if NV_GCC_FAMILY
        return __PRETTY_FUNCTION__;
#else
        // name() calls malloc(), raw_name() wouldn't
        return typeid(T).name();
#endif
    }

  public:
    ReflectionAllocator(const NvEMPTY)
    {
    }
    ReflectionAllocator(const char* = 0)
    {
    }
    inline ReflectionAllocator(const ReflectionAllocator&)
    {
    }
    void* allocate(size_t size, const char* filename, int line)
    {
        return size ? getAllocator().allocate(size, getName(), filename, line) : 0;
    }
    void deallocate(void* ptr)
    {
        if(ptr)
            getAllocator().deallocate(ptr);
    }
};

template <typename T>
struct AllocatorTraits
{
#if NV_USE_NAMED_ALLOCATOR
    typedef NamedAllocator Type;
#else
    typedef ReflectionAllocator<T> Type;
#endif
};

// if you get a build error here, you are trying to NV_NEW a class
// that is neither plain-old-type nor derived from UserAllocated
template <typename T, typename X>
union EnableIfPod
{
    int i;
    T t;
    typedef X Type;
};

} // namespace shdfnd
} // namespace nvidia

// Global placement new for ReflectionAllocator templated by
// plain-old-type. Allows using NV_NEW for pointers and built-in-types.
//
// ATTENTION: You need to use NV_DELETE_POD or NV_FREE to deallocate
// memory, not NV_DELETE. NV_DELETE_POD redirects to NV_FREE.
//
// Rationale: NV_DELETE uses global operator delete(void*), which we dont' want to overload.
// Any other definition of NV_DELETE couldn't support array syntax 'NV_DELETE([]a);'.
// NV_DELETE_POD was preferred over NV_DELETE_ARRAY because it is used
// less often and applies to both single instances and arrays.
template <typename T>
NV_INLINE void* operator new(size_t size, nvidia::shdfnd::ReflectionAllocator<T> alloc, const char* fileName,
                             typename nvidia::shdfnd::EnableIfPod<T, int>::Type line)
{
    return alloc.allocate(size, fileName, line);
}

template <typename T>
NV_INLINE void* operator new [](size_t size, nvidia::shdfnd::ReflectionAllocator<T> alloc, const char* fileName,
                                typename nvidia::shdfnd::EnableIfPod<T, int>::Type line)
{ return alloc.allocate(size, fileName, line); }

// If construction after placement new throws, this placement delete is being called.
template <typename T>
NV_INLINE void operator delete(void* ptr, nvidia::shdfnd::ReflectionAllocator<T> alloc, const char* fileName,
                               typename nvidia::shdfnd::EnableIfPod<T, int>::Type line)
{
    NV_UNUSED(fileName);
    NV_UNUSED(line);

    alloc.deallocate(ptr);
}

// If construction after placement new throws, this placement delete is being called.
template <typename T>
NV_INLINE void operator delete [](void* ptr, nvidia::shdfnd::ReflectionAllocator<T> alloc, const char* fileName,
                                  typename nvidia::shdfnd::EnableIfPod<T, int>::Type line)
{
    NV_UNUSED(fileName);
    NV_UNUSED(line);

    alloc.deallocate(ptr);
}

#endif // #ifndef NV_NSFOUNDATION_NSALLOCATOR_H

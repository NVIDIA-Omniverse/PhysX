// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

// Copied from ICE
// PT: TODO: refactor all of this

#include <carb/Types.h>
#include "Preprocessor.h"

class Allocator
{
public:
                    Allocator()     {}
    virtual         ~Allocator()    {}

    virtual void*   malloc(size_t size) = 0;
    virtual void*   mallocDebug(size_t size, const char* filename, int line, const char* className, bool fromNew) = 0;
    virtual void    free(void* memory, bool fromNew) = 0;
};

// PT: TODO: hide this function because people aren't supposed to call it directly
Allocator* GetAllocator();

class Allocateable
{
public:
    __forceinline void* operator new(size_t size)
    {
        return GetAllocator()->malloc(size);
    }

    __forceinline void* operator new(size_t size, void* ptr)
    {
        return ptr;
    }

    __forceinline void* operator new(size_t size, const char* filename, int line, const char* class_name)
    {
        return GetAllocator()->mallocDebug(size, filename, line, class_name, true);
    }
    __forceinline void* operator new[](size_t size)
    {
        return GetAllocator()->malloc(size);
    }
    __forceinline void* operator new[](size_t size, const char* filename, int line, const char* class_name)
    {
        return GetAllocator()->mallocDebug(size, filename, line, class_name, true);
    }

    __forceinline void operator delete(void* p, void* p0)
    {
    }

    __forceinline void operator delete(void* p)
    {
        GetAllocator()->free(p, true);
    }
    __forceinline void operator delete(void* p, const char*, int, const char*)
    {
        GetAllocator()->free(p, true);
    }
    __forceinline void operator delete[](void* p)
    {        
        GetAllocator()->free(p, true);
    }
    __forceinline void operator delete[](void* p, const char*, int, const char*)
    {        
        GetAllocator()->free(p, true);
    }

    bool mOwnsMemory { true };
};

#ifdef _DEBUG
    #define ICE_ALLOC(x) GetAllocator()->mallocDebug(x, OV_FL, "(undefined)", false)
#else
    #define ICE_ALLOC(x) GetAllocator()->malloc(x)
#endif
#define ICE_FREE_BASIC(x) GetAllocator()->free(x, false);
#define ICE_FREE(x)         \
    if (x)                  \
    {                       \
        ICE_FREE_BASIC(x);  \
        x = nullptr;        \
    }

#ifdef _DEBUG
    #define ICE_NEW(x) new (OV_FL, #x) x
#else
    #define ICE_NEW(x) new x
#endif

#ifdef _DEBUG
    #define ICE_PLACEMENT_NEW(x) new (GetAllocator()->mallocDebug(sizeof(x), OV_FL, "(undefined)", false)) x
#else
    #define ICE_PLACEMENT_NEW(x) new (GetAllocator()->malloc(sizeof(x))) x
#endif

#define ICE_PLACEMENT_DELETE_UNCHECKED(x, y)    \
    (x)->~y();                                  \
    ICE_FREE_BASIC(x)

#define ICE_PLACEMENT_DELETE(x, y)              \
    if (x)                                      \
    {                                           \
        ICE_PLACEMENT_DELETE_UNCHECKED(x, y)    \
    }

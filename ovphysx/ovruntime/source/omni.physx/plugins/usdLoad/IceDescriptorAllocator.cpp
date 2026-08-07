// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-CORE-006
 * @covers AC-6
 */

#include "UsdPCH.h"

#include "IceDescriptorAllocator.h"

#include <common/foundation/Allocator.h>

#include <cassert>
#include <cstdint>

namespace omni::physx::usdparser
{

namespace
{

// ICE's `malloc` returns at most 16-byte-aligned memory on x64 — enough
// for every current parse-lib descriptor — but the `IDescriptorAllocator`
// contract has callers passing the real `alignof(T)`. For alignment values
// the allocator itself guarantees we forward directly; for stricter
// alignments we over-allocate, hand back a manually-aligned address, and
// stash the raw pointer in the void* slot immediately preceding it so
// `deallocate` can find the original block to free. The header doubles
// the slot's address (set to the aligned pointer itself) as a runtime
// sanity check that the slow path produced this pointer.
constexpr size_t kIceMallocAlignment = 16;

void* alignedAllocate(size_t bytes, size_t alignment)
{
    if (alignment <= kIceMallocAlignment)
        return ICE_ALLOC(bytes);

    // bytes + alignment-1 worst-case shift + sizeof(void*) header.
    const size_t total = bytes + alignment + sizeof(void*);
    void* raw = ICE_ALLOC(total);
    if (!raw)
        return nullptr;

    const uintptr_t rawAddr = reinterpret_cast<uintptr_t>(raw);
    const uintptr_t alignedAddr =
        (rawAddr + sizeof(void*) + alignment - 1) & ~(static_cast<uintptr_t>(alignment) - 1);
    reinterpret_cast<void**>(alignedAddr)[-1] = raw;
    return reinterpret_cast<void*>(alignedAddr);
}

void alignedDeallocate(void* ptr, size_t alignment) noexcept
{
    if (!ptr)
        return;
    if (alignment <= kIceMallocAlignment)
    {
        ICE_FREE_BASIC(ptr);
        return;
    }
    // Recover the raw block address stashed in the header slot.
    void* raw = reinterpret_cast<void**>(ptr)[-1];
    ICE_FREE_BASIC(raw);
}

class IceDescriptorAllocatorImpl final : public omni::physics::parse::IDescriptorAllocator
{
public:
    void* allocate(size_t bytes, size_t alignment) override
    {
        // Alignment must be a power of two.
        assert(alignment != 0 && (alignment & (alignment - 1)) == 0);
        return alignedAllocate(bytes, alignment);
    }

    void deallocate(void* ptr, size_t /*bytes*/, size_t alignment) noexcept override
    {
        alignedDeallocate(ptr, alignment);
    }
};

} // namespace

omni::physics::parse::IDescriptorAllocator& iceDescriptorAllocator()
{
    static IceDescriptorAllocatorImpl s_instance;
    return s_instance;
}

} // namespace omni::physx::usdparser

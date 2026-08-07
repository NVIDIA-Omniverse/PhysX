// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * Pluggable descriptor allocator for the parse library.
 *
 * @implements REQ-PARSE-CORE-001
 */

#pragma once

#include <cstddef>
#include <memory>
#include <utility>

namespace omni::physics::parse
{

// Floor for `allocateDesc<T>` alignment requests. Call sites needing
// stricter alignment (`alignof(T) > 16`) pass the value explicitly.
inline constexpr size_t kDefaultDescriptorAlignment = 16;

// Back-end allocator policy for parse-lib descriptors. Returns raw
// memory; parse-lib constructs and destroys via placement-new and an
// explicit destructor call. Subclasses must satisfy the requested
// power-of-two `alignment`; `deallocate` must be safe to call from
// any thread that owns the pointer.

class IDescriptorAllocator
{
public:
    virtual ~IDescriptorAllocator() = default;

    // Returns nullptr on failure.
    virtual void* allocate(size_t bytes,
                           size_t alignment = kDefaultDescriptorAlignment) = 0;

    // `bytes` and `alignment` must mirror the values used at allocation
    // (allows size-class-bucketed allocators to dispatch without a
    // pointer-to-metadata lookup).
    virtual void deallocate(void* ptr,
                            size_t bytes,
                            size_t alignment = kDefaultDescriptorAlignment) noexcept = 0;
};

// Type-erased deleter used by `DescPtr`. The destroy fn is minted by
// `allocateDesc<T>` capturing the most-derived `T`, so destruction
// runs the right destructor even when stored through a `DescPtr<Base>`.
// Avoids the need for virtual destructors on the descriptor hierarchy.

template <typename Base>
struct DescDeleter
{
    IDescriptorAllocator* alloc = nullptr;
    void (*destroy)(Base*, IDescriptorAllocator*) = nullptr;

    void operator()(Base* p) const noexcept
    {
        if (p && destroy && alloc)
            destroy(p, alloc);
    }
};

template <typename T>
using DescPtr = std::unique_ptr<T, DescDeleter<T>>;

// Allocate + construct a `T`, returning a `DescPtr<Base>` that owns it.
// `Base` defaults to `T`; pass an explicit `Base` for polymorphic storage,
// e.g. `allocateDesc<SpherePhysxShapeDesc, PhysxShapeDesc>(alloc, radius)`.

template <typename T, typename Base = T, typename... Args>
DescPtr<Base> allocateDesc(IDescriptorAllocator& alloc, Args&&... args)
{
    // `static` matters: both constants are read inside the captureless
    // `+[]` deleter below, which must stay captureless to convert to a plain
    // function pointer. GCC/Clang accept that (a constexpr local read as a
    // constant expression is not odr-used, so no capture is required), but
    // MSVC rejects it with C3493 "cannot be implicitly captured because no
    // default capture mode has been specified". Static storage duration takes
    // them out of the capture rules entirely and compiles everywhere.
    static constexpr size_t bytes = sizeof(T);
    static constexpr size_t alignment = alignof(T) > kDefaultDescriptorAlignment
                                            ? alignof(T)
                                            : kDefaultDescriptorAlignment;

    void* mem = alloc.allocate(bytes, alignment);
    if (!mem)
        return DescPtr<Base>(nullptr, DescDeleter<Base>{});

    T* obj = new (mem) T(std::forward<Args>(args)...);

    DescDeleter<Base> d;
    d.alloc = &alloc;
    d.destroy = +[](Base* p, IDescriptorAllocator* a) noexcept
    {
        T* derived = static_cast<T*>(p);
        derived->~T();
        a->deallocate(derived, bytes, alignment);
    };
    return DescPtr<Base>(obj, d);
}

// Rebind a `DescPtr<Derived>` into a `DescPtr<Base>` for polymorphic
// storage. The destroy fn is re-minted so the most-derived destructor
// still runs.

template <typename Base, typename Derived>
DescPtr<Base> descPtrCast(DescPtr<Derived>&& src) noexcept
{
    static_assert(std::is_base_of_v<Base, Derived>,
                  "descPtrCast: Base must be a base of Derived");

    if (!src)
        return DescPtr<Base>(nullptr, DescDeleter<Base>{});

    // `static` for the same reason as in allocateDesc above: these are read
    // inside the captureless `+[]` deleter, which MSVC otherwise rejects with
    // C3493.
    static constexpr size_t bytes = sizeof(Derived);
    static constexpr size_t alignment = alignof(Derived) > kDefaultDescriptorAlignment
                                            ? alignof(Derived)
                                            : kDefaultDescriptorAlignment;

    DescDeleter<Base> d;
    d.alloc = src.get_deleter().alloc;
    d.destroy = +[](Base* p, IDescriptorAllocator* a) noexcept
    {
        Derived* derived = static_cast<Derived*>(p);
        derived->~Derived();
        a->deallocate(derived, bytes, alignment);
    };
    return DescPtr<Base>(src.release(), d);
}

} // namespace omni::physics::parse

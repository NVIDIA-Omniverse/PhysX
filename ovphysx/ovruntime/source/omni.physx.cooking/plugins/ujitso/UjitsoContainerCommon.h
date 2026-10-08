// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <memory>
#include <new>
#include <utility>
#include <vector>

#include <omni/physx/IPhysxCookingService.h>

#include <carb/ujitso/Container.h>

#include "../service/CookingComputeService.h"
#include "../service/CookingHashing.h"

// TEMP: this can be removed once it is added to DataStoreUtils.inl on the rendering side
#include <type_traits>

#include <carb/Types.h>
#include <carb/Defines.h>
#include <carb/extras/Hash.h>
#include <carb/profiler/Profile.h>
#include <omni/utils/Serializer.h>

/*
    N.B.: this is pulled from kit rendering/include/carb/ujitso/ContainerUtils.inl

    We can't include that file without bringing in a dependency on xxhash.h, which we'd like to avoid.

    The macro has the comment "non - ABI stable helpers" above it.  If and when that macro is stabilized, it will
    hopefully be moved to a header file that doesn't require xxhash.  Then we can include that and remove this
    macro from our file.
 */
#define UJITSO_CONTAINER_DEFINITION(typeName)                                                                          \
private:                                                                                                               \
    ContainerContentHashPOD m_hash;                                                                                    \
                                                                                                                       \
public:                                                                                                                \
    const char* getName() const override                                                                               \
    {                                                                                                                  \
        return #typeName;                                                                                              \
    }                                                                                                                  \
    static constexpr ContainerType getTypeStatic()                                                                     \
    {                                                                                                                  \
        return CARB_HASH_STRING(#typeName);                                                                            \
    }                                                                                                                  \
    ContainerType getType() const override                                                                             \
    {                                                                                                                  \
        return getTypeStatic();                                                                                        \
    }                                                                                                                  \
    ContainerContentHash getContentHash() const override                                                               \
    {                                                                                                                  \
        return fromPod(m_hash);                                                                                        \
    }

namespace omni
{
namespace physx
{
// currently needed for UJITSO_CONTAINER_DEFINITION
// can remove once that code gets updated
using namespace carb::ujitso;

// Allocator whose construct() default-initializes (no parens: `::new (ptr) U`) instead of
// value-initializing (zero-filling), for trivially-default-constructible U. std::vector has no
// resize-with-tag overload like VtArray's `resize(count, DoNotInitialize<T>())` (the mechanism
// these Ujitso build-data buffers relied on before their VtArray -> std::vector retype, c8cad805ab
// -- lost there, not by design: this restores the equivalent skip-init optimization the only way
// std::vector supports it, by hooking the allocator instead of the resize call). Every buffer
// resized with this allocator is fully overwritten immediately after -- either via
// serializeBufferCopy() (POD path) or per-element serializer.serialize() (the loop below) -- so
// the zero-fill resize() would otherwise do first is wasted work on this hot cache-read path.
// Non-trivial element types still get their real constructor run (default-init calls a
// user-provided default constructor exactly like value-init does); only scalar/aggregate types
// with no user-declared constructor skip the zeroing.
template <typename T>
struct DefaultInitAllocator : std::allocator<T>
{
    using std::allocator<T>::allocator;

    template <typename U>
    struct rebind
    {
        using other = DefaultInitAllocator<U>;
    };

    template <typename U>
    void construct(U* ptr) noexcept(std::is_nothrow_default_constructible<U>::value)
    {
        ::new (static_cast<void*>(ptr)) U;
    }

    template <typename U, typename... Args>
    void construct(U* ptr, Args&&... args)
    {
        ::new (static_cast<void*>(ptr)) U(std::forward<Args>(args)...);
    }

    template <typename U>
    void destroy(U* ptr)
    {
        ptr->~U();
    }
};

template <typename T, typename U>
inline bool operator==(const DefaultInitAllocator<T>&, const DefaultInitAllocator<U>&)
{
    return true;
}

template <typename T, typename U>
inline bool operator!=(const DefaultInitAllocator<T>&, const DefaultInitAllocator<U>&)
{
    return false;
}

// Convenience alias for the Ujitso build-data buffers that want the skip-init resize above.
template <typename T>
using UninitVector = std::vector<T, DefaultInitAllocator<T>>;

template <bool readOnly, typename T, typename Alloc, typename SerializerT>
void serializeVector(std::vector<T, Alloc>& data, SerializerT& serializer)
{
    CARB_PROFILE_ZONE(0, "omni::physx::serializeVector");

    size_t arraySize = data.size();
    serializer.serialize(readOnly, arraySize);

    if (readOnly)
    {
        data.resize(arraySize);
    }

    // Skip buffer serialization if it is empty. Needs to be after the resize above this.
    if (data.empty())
    {
        return;
    }

    if (std::is_pod_v<T>)
    {
        size_t dataSize = sizeof(T) * arraySize;
        // Whether reading or writing, data() is the buffer to fill from / read from; const_cast
        // for serializeBufferCopy's void* interface.
        void* dataPtr = const_cast<T*>(data.data());

        serializer.serializeBufferCopy(readOnly, dataPtr, dataSize);
        CARB_CHECK(dataSize == sizeof(T) * arraySize);
    }
    else
    {
        for (T& value : data)
        {
            serializer.serialize(readOnly, value);
        }
    }
}

template <typename ToType, typename FromType, typename Alloc>
inline bool copyVectorData(std::vector<ToType, Alloc>& to, const omni::span<const FromType>& from)
{
    CARB_PROFILE_ZONE(0, "omni::physx::copyVectorData");

    if (from.size() > 0)
    {
        static_assert(sizeof(ToType) == sizeof(FromType), "Data size mismatch");
        to.assign(
            reinterpret_cast<const ToType*>(from.data()), reinterpret_cast<const ToType*>(from.data()) + from.size());
        return true;
    }
    return false;
}

// Base class for all Ujitso physics cooking input containers
class PhysicsInputContainerBase : public omni::core::Implements<carb::ujitso::IContainer>
{
public:
    PhysicsInputContainerBase(const PhysxCookingComputeResult& result, const PhysxCookingComputeRequest& request)
        : m_result(result), m_request(request)
    {
        // update the result pointer to the request to point at the local copy
        m_result.request = &m_request;
    }

    virtual ~PhysicsInputContainerBase() = default;

    void fill() override = 0;
    void read(carb::ujitso::IReader& reader) override = 0;
    void write(carb::ujitso::IWriter& writer) override = 0;

protected:
    // Whether the request's input views must be copied *now*, i.e. in the container constructor,
    // which still runs on the thread that submitted the cooking request.
    //
    // An eINPUT_MODE_FROM_PRIM_MESH_VIEW request carries omni::span views over memory the caller
    // owns; IPhysxCookingService.h only promises that memory "for entire duration of request*
    // calls". A synchronous cook honours that: MeshCookingContext::initiateBuild does
    // requestBuild() + waitRequest() inline, so fill() runs before the submitting call returns.
    // An asynchronous cook does not: initiateBuild only queues the context
    // (UjitsoProcessManager::schedule), and the build -- and with it fill() -- first runs from a
    // later pumpAsyncContext(), by which time the caller's buffers are gone. Copying the views
    // here is the only point at which they are still guaranteed valid.
    //
    // Every request is mesh-view mode now (eINPUT_MODE_FROM_PRIM_ID removed, REQ-COOK-SOURCE-001),
    // so this always applies for an async cook -- no mode check left to make.
    bool shouldSnapshotInputNow() const
    {
        return m_request.options.hasFlag(PhysxCookingComputeRequest::Options::kComputeAsynchronously);
    }

    PhysxCookingComputeResult m_result;
    PhysxCookingComputeRequest m_request;

    // Set once the constructor has copied the input views into m_buildData; fill() then has
    // nothing left to do and must not touch the (by then dangling) views again.
    bool m_inputSnapshotted = false;
};

} // namespace physx
} // namespace omni

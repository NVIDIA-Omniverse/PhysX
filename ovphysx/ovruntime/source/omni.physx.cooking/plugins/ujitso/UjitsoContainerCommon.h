// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

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
#include <common/utilities/UsdMaterialParsing.h>
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

// this is used to avoid initializing arrays on resize that are about to be read into
template <typename T>
struct DoNotInitialize
{
    inline void operator()(T*, T*) const
    {
    }
};

template <bool readOnly, typename T, typename SerializerT>
void serializeVtArray(PXR_NS::VtArray<T>& data, SerializerT& serializer)
{
    CARB_PROFILE_ZONE(0, "omni::physx::serializeVtArray");

    size_t arraySize = data.size();
    serializer.serialize(readOnly, arraySize);

    if (readOnly)
    {
        data.resize(arraySize, DoNotInitialize<T>());
    }

    // Skip buffer serialization if it is empty. Needs to be after the resize above this.
    if (data.empty())
    {
        return;
    }

    if (std::is_pod_v<T>)
    {
        size_t dataSize = sizeof(T) * arraySize;
        void* dataPtr = nullptr;
        if (readOnly)
        {
            dataPtr = data.data();
        }
        else
        {
            // Use cdata() here to avoid copy on write when writing; const_cast for serializeBufferCopy interface
            dataPtr = const_cast<T*>(data.cdata());
        }

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

template <typename ToType, typename FromType>
inline bool copyVtArrayData(PXR_NS::VtArray<ToType>& to, const omni::span<const FromType>& from)
{
    CARB_PROFILE_ZONE(0, "omni::physx::copyVtArrayData");

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
    PhysxCookingComputeResult m_result;
    PhysxCookingComputeRequest m_request;
};

} // namespace physx
} // namespace omni

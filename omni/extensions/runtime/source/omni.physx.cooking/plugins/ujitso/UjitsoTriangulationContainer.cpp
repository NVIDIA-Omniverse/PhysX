// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "UjitsoTriangulationContainer.h"
#include "../service/CookingComputeService.h"
#include "../service/CookingHashing.h"

// TEMP: this can be removed once it is added to DataStoreUtils.inl on the rendering side
#include <type_traits>

#include <carb/Defines.h>
#include <carb/extras/Hash.h>
#include <carb/profiler/Profile.h>

#include <common/utilities/UsdMaterialParsing.h>
#include <omni/utils/Serializer.h>

#define CONTAINER_DATA_FORMAT_VERSION (1)

namespace omni
{
namespace physx
{

namespace
{

// this is used to avoid initializing arrays on resize that are about to be read into
template <typename T>
struct DoNotInitialize
{
    inline void operator()(T*, T*) const
    {
    }
};

template <bool readOnly, typename T, typename SerializerT>
void serializeVtArray(pxr::VtArray<T>& data, SerializerT& serializer)
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

} // anonymous namespace

template<typename ToType, typename FromType>
inline bool copyVtArrayData(pxr::VtArray<ToType>& to, const omni::span<const FromType>& from)
{
    CARB_PROFILE_ZONE(0, "omni::physx::copyVtArrayData");

    if (from.size() > 0)
    {
        static_assert(sizeof(ToType) == sizeof(FromType), "Data size mismatch");
        to.assign(reinterpret_cast<const ToType*>(from.data()), reinterpret_cast<const ToType*>(from.data()) + from.size());
        return true;
    }
    return false;
}

PhysicsTriangulationInputContainer::PhysicsTriangulationInputContainer
(
    const omni::physx::PhysxCookingComputeResult& result,
    const omni::physx::PhysxCookingComputeRequest& request
) : m_result(result), m_request(request)
{
    CARB_PROFILE_ZONE(0, "PhysicsTriangulationInputContainer::PhysicsTriangulationInputContainer");

    // update the result pointer to the request to point at the local copy
    m_result.request = &m_request;

    // copy or compute the hash for the mesh data
    copyOrComputeHash();
}

// This is only called if ujitso decides that the derived data needs to be rebuilt
void PhysicsTriangulationInputContainer::fill()
{
    CARB_PROFILE_ZONE(0, "PhysicsTriangulationInputContainer::fill");

    // we may have to reload the prim data here based on the dataInputMode
    CookingStageAndPrim stageAndPrim;
    switch (m_request.dataInputMode)
    {
        case PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_MESH_VIEW:
            // the data is already loaded, nothing needs to be done in this case
            break;

        case PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_ID:
            // the data needs to be reloaded from the prim
            if (!(ICookingComputeService::getStageAndPrim(m_result, m_request, stageAndPrim) &&
                  ICookingComputeService::fillMeshView(m_result, m_request, stageAndPrim)))
            {
                return;
            }
            break;
        
        default:
            CARB_LOG_ERROR("Unexpected data input mode: %u", m_request.dataInputMode);
            return;
    }

    // data should be loaded at this point, copy it over to the buildData output
    copyVtArrayData(m_buildData.points, m_request.primMeshView.points);
    copyVtArrayData(m_buildData.indices, m_request.primMeshView.indices);
    copyVtArrayData(m_buildData.faceCounts, m_request.primMeshView.faces);
    copyVtArrayData(m_buildData.holeIndices, m_request.primMeshView.holeIndices);
    copyVtArrayData(m_buildData.faceMaterials, m_request.primMeshView.faceMaterials);

    // copy this across so build functions only need to deal with one struct
    m_buildData.rightHandedOrientation = m_request.primMeshView.rightHandedOrientation;
}

// For now, we simply serialize bytes over the wire, but there's potential for compression here
template <bool readOnly, typename SerializerT>
void PhysicsTriangulationInputContainer::serialize(SerializerT& serializer)
{
    CARB_PROFILE_ZONE(0, "PhysicsTriangulationInputContainer::serialize");

    // container version is used to enable adding new optional data without invalidating existing cache entries
    // it is not included in the container hash, only used to support branching in the serialize code
    uint32_t version = CONTAINER_DATA_FORMAT_VERSION;
    serializer.serialize(readOnly, version);

    // serialize the heavy array data
    serializeVtArray<readOnly>(m_buildData.points, serializer);
    serializeVtArray<readOnly>(m_buildData.indices, serializer);
    serializeVtArray<readOnly>(m_buildData.faceCounts, serializer);
    serializeVtArray<readOnly>(m_buildData.holeIndices, serializer);
    serializeVtArray<readOnly>(m_buildData.faceMaterials, serializer);

    // serialize the POD pieces
    serializer.serialize(readOnly, m_buildData.rightHandedOrientation);
}

void PhysicsTriangulationInputContainer::read(carb::ujitso::IReader& reader)
{
    CARB_PROFILE_ZONE(0, "PhysicsTriangulationInputContainer::read");

    reader.read(reader.bytesLeft(),
        [](void* context, const uint8_t* data, size_t size)
        {
            PhysicsTriangulationInputContainer* thisContainer =
                reinterpret_cast<PhysicsTriangulationInputContainer*>(context);

            constexpr bool kReadOnly = true;

            omni::ExternalSerializer<kReadOnly> serializer(data, size);
            thisContainer->serialize<kReadOnly>(serializer);
        },
        this);
}

void PhysicsTriangulationInputContainer::write(carb::ujitso::IWriter& writer)
{
    CARB_PROFILE_ZONE(0, "PhysicsTriangulationInputContainer::write");

    size_t inputSizeEstimate = 0;

    // We calculate a rough input size estimate for serialization to reduce memory allocations
    // This should match what is written out in serialize()
    inputSizeEstimate += sizeof(uint32_t); // version

    // array data
    inputSizeEstimate += m_buildData.points.size() * sizeof(carb::Float3) + sizeof(size_t);
    inputSizeEstimate += m_buildData.indices.size() * sizeof(uint32_t) + sizeof(size_t);
    inputSizeEstimate += m_buildData.faceCounts.size() * sizeof(uint32_t) + sizeof(size_t);
    inputSizeEstimate += m_buildData.holeIndices.size() * sizeof(uint32_t) + sizeof(size_t);
    inputSizeEstimate += m_buildData.faceMaterials.size() * sizeof(uint16_t) + sizeof(size_t);

    // POD pieces
    inputSizeEstimate += sizeof(m_buildData.rightHandedOrientation);

    // Reserve at the input size estimate, should prevent having to resize the buffer
    omni::VectorSerializer serializer{ inputSizeEstimate };
    constexpr bool kLoading = false;
    serialize<kLoading>(serializer);
    writer.write(serializer.start, serializer.getCurrentPos());
}

void PhysicsTriangulationInputContainer::copyOrComputeHash()
{
    CARB_PROFILE_ZONE(0, "PhysicsTriangulationInputContainer::copyOrComputeHash");

    omni::physx::usdparser::MeshKey meshKey = m_result.meshKey;
    if (meshKey == omni::physx::usdparser::MeshKey())
    {
        // ask omni.physics to build the hash for us if it isn't already valid
        // needs to match what would be passed in if it is pre-computed
        meshKey = omni::physx::MeshKeyComputation::computeMeshKey(m_request.primMeshView);
    }
    else
    {
        // in debug builds, check that the supplied hash matches what we would compute
        CARB_ASSERT(meshKey == omni::physx::MeshKeyComputation::computeMeshKey(m_request.primMeshView));
    }

    // roll the winding order in to the hash (it isn't included by default)
    meshKey.setRightHandedOrientation(m_request.primMeshView.rightHandedOrientation);

    // convert it to a 256 bit hash by duplicating it
    const carb::extras::hash128_t value = meshKey.getFullHash();
    m_hash = { value.d[0], value.d[1] };
}

} // namespace physx
} // namespace omni

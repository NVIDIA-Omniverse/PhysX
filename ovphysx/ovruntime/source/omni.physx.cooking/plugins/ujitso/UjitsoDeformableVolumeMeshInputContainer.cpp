// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#include "UjitsoDeformableVolumeMeshInputContainer.h"

#define DEFORMABLE_VOLUME_MESH_CONTAINER_DATA_FORMAT_VERSION (1)

namespace omni
{
namespace physx
{

PhysicsDeformableVolumeMeshInputContainer::PhysicsDeformableVolumeMeshInputContainer(
    const omni::physx::PhysxCookingComputeResult& result, const omni::physx::PhysxCookingComputeRequest& request)
    : PhysicsInputContainerBase(result, request)
{
    CARB_PROFILE_ZONE(0, "PhysicsDeformableVolumeMeshInputContainer::PhysicsDeformableVolumeMeshInputContainer");

    copyOrComputeHash();

    if (shouldSnapshotInputNow())
    {
        copyInputViews();
        m_inputSnapshotted = true;
    }
}

// This is only called if ujitso decides that the derived data needs to be rebuilt
void PhysicsDeformableVolumeMeshInputContainer::fill()
{
    CARB_PROFILE_ZONE(0, "PhysicsDeformableVolumeMeshInputContainer::fill");

    if (m_inputSnapshotted)
        return; // the constructor already copied the views, while the caller's memory was alive

    // Every request is mesh-view mode now (eINPUT_MODE_FROM_PRIM_ID removed, REQ-COOK-SOURCE-001).
    // This only runs for a synchronous cook (an async one already snapshotted above and returned
    // at the m_inputSnapshotted check), so the caller's buffers are still alive here -- the data
    // is already loaded, nothing to reload.

    // data should be loaded at this point, copy it over to the buildData output
    copyInputViews();
}

void PhysicsDeformableVolumeMeshInputContainer::copyInputViews()
{
    CARB_PROFILE_ZONE(0, "PhysicsDeformableVolumeMeshInputContainer::copyInputViews");

    copyVectorData(m_buildData.simPoints, m_request.volumeMeshView.simPoints);
    copyVectorData(m_buildData.simBindPoints, m_request.volumeMeshView.simBindPoints);
    copyVectorData(m_buildData.simIndices, m_request.volumeMeshView.simIndices);
    copyVectorData(m_buildData.collBindPointsInSim, m_request.volumeMeshView.collBindPointsInSim);
    copyVectorData(m_buildData.collIndices, m_request.volumeMeshView.collIndices);
    copyVectorData(m_buildData.collSurfaceIndices, m_request.volumeMeshView.collSurfaceIndices);
}

// For now, we simply serialize bytes over the wire, but there's potential for compression here
template <bool readOnly, typename SerializerT>
void PhysicsDeformableVolumeMeshInputContainer::serialize(SerializerT& serializer)
{
    CARB_PROFILE_ZONE(0, "PhysicsDeformableVolumeMeshInputContainer::serialize");

    // container version is used to enable adding new optional data without invalidating existing cache entries
    // it is not included in the container hash, only used to support branching in the serialize code
    uint32_t version = DEFORMABLE_VOLUME_MESH_CONTAINER_DATA_FORMAT_VERSION;
    serializer.serialize(readOnly, version);

    // serialize the heavy array data
    serializeVector<readOnly>(m_buildData.simPoints, serializer);
    serializeVector<readOnly>(m_buildData.simBindPoints, serializer);
    serializeVector<readOnly>(m_buildData.simIndices, serializer);
    serializeVector<readOnly>(m_buildData.collBindPointsInSim, serializer);
    serializeVector<readOnly>(m_buildData.collIndices, serializer);
    serializeVector<readOnly>(m_buildData.collSurfaceIndices, serializer);
}

void PhysicsDeformableVolumeMeshInputContainer::read(carb::ujitso::IReader& reader)
{
    CARB_PROFILE_ZONE(0, "PhysicsDeformableVolumeMeshInputContainer::read");

    reader.read(
        reader.bytesLeft(),
        [](void* context, const uint8_t* data, size_t size) {
            PhysicsDeformableVolumeMeshInputContainer* thisContainer =
                reinterpret_cast<PhysicsDeformableVolumeMeshInputContainer*>(context);

            constexpr bool kReadOnly = true;

            omni::ExternalSerializer<kReadOnly> serializer(data, size);
            thisContainer->serialize<kReadOnly>(serializer);
        },
        this);
}

void PhysicsDeformableVolumeMeshInputContainer::write(carb::ujitso::IWriter& writer)
{
    CARB_PROFILE_ZONE(0, "PhysicsDeformableVolumeMeshInputContainer::write");

    size_t inputSizeEstimate = 0;

    // We calculate a rough input size estimate for serialization to reduce memory allocations
    // This should match what is written out in serialize()
    inputSizeEstimate += sizeof(uint32_t); // version

    // array data
    inputSizeEstimate += m_buildData.simPoints.size() * sizeof(carb::Float3) + sizeof(size_t);
    inputSizeEstimate += m_buildData.simBindPoints.size() * sizeof(carb::Float3) + sizeof(size_t);
    inputSizeEstimate += m_buildData.simIndices.size() * sizeof(carb::Int4) + sizeof(size_t);
    inputSizeEstimate += m_buildData.collBindPointsInSim.size() * sizeof(carb::Float3) + sizeof(size_t);
    inputSizeEstimate += m_buildData.collIndices.size() * sizeof(carb::Int4) + sizeof(size_t);
    inputSizeEstimate += m_buildData.collSurfaceIndices.size() * sizeof(carb::Int3) + sizeof(size_t);

    // Reserve at the input size estimate, should prevent having to resize the buffer
    omni::VectorSerializer serializer{ inputSizeEstimate };
    constexpr bool kLoading = false;
    serialize<kLoading>(serializer);
    writer.write(serializer.start, serializer.getCurrentPos());
}

void PhysicsDeformableVolumeMeshInputContainer::copyOrComputeHash()
{
    CARB_PROFILE_ZONE(0, "PhysicsDeformableVolumeMeshInputContainer::copyOrComputeHash");

    omni::physx::usdparser::MeshKey meshKey = m_result.meshKey;
    if (meshKey == omni::physx::usdparser::MeshKey())
    {
        // ask omni.physics to build the hash for us if it isn't already valid
        // needs to match what would be passed in if it is pre-computed
        meshKey = omni::physx::MeshKeyComputation::computeMeshKey(m_request.volumeMeshView);
    }
    else
    {
        // in debug builds, check that the supplied hash matches what we would compute
        CARB_ASSERT(meshKey == omni::physx::MeshKeyComputation::computeMeshKey(m_request.volumeMeshView));
    }

    // convert it to a 256 bit hash by duplicating it
    const carb::extras::hash128_t value = meshKey.getFullHash();
    m_hash = { value.d[0], value.d[1] };
}

} // namespace physx
} // namespace omni

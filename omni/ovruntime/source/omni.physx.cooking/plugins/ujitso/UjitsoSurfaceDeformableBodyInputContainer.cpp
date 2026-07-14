// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include "UsdPCH.h"

#include "UjitsoSurfaceDeformableBodyInputContainer.h"

#define SURFACE_DEFORMABLE_BODY_CONTAINER_DATA_FORMAT_VERSION (1)

namespace omni
{
namespace physx
{

PhysicsSurfaceDeformableBodyInputContainer::PhysicsSurfaceDeformableBodyInputContainer(
    const omni::physx::PhysxCookingComputeResult& result, const omni::physx::PhysxCookingComputeRequest& request)
    : PhysicsInputContainerBase(result, request)
{
    CARB_PROFILE_ZONE(0, "PhysicsSurfaceDeformableBodyInputContainer::PhysicsSurfaceDeformableBodyInputContainer");

    // copy or compute the hash for the mesh data
    copyOrComputeHash();
}

// This is only called if ujitso decides that the derived data needs to be rebuilt
void PhysicsSurfaceDeformableBodyInputContainer::fill()
{
    CARB_PROFILE_ZONE(0, "PhysicsSurfaceDeformableBodyInputContainer::fill");

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
    copyVtArrayData(m_buildData.srcPointsInSim, m_request.surfaceDeformableBodyView.srcPointsInSim);
}

// For now, we simply serialize bytes over the wire, but there's potential for compression here
template <bool readOnly, typename SerializerT>
void PhysicsSurfaceDeformableBodyInputContainer::serialize(SerializerT& serializer)
{
    CARB_PROFILE_ZONE(0, "PhysicsSurfaceDeformableBodyInputContainer::serialize");

    // container version is used to enable adding new optional data without invalidating existing cache entries
    // it is not included in the container hash, only used to support branching in the serialize code
    uint32_t version = SURFACE_DEFORMABLE_BODY_CONTAINER_DATA_FORMAT_VERSION;
    serializer.serialize(readOnly, version);

    // serialize the heavy array data
    serializeVtArray<readOnly>(m_buildData.srcPointsInSim, serializer);
}

void PhysicsSurfaceDeformableBodyInputContainer::read(carb::ujitso::IReader& reader)
{
    CARB_PROFILE_ZONE(0, "PhysicsSurfaceDeformableBodyInputContainer::read");

    reader.read(
        reader.bytesLeft(),
        [](void* context, const uint8_t* data, size_t size) {
            PhysicsSurfaceDeformableBodyInputContainer* thisContainer =
                reinterpret_cast<PhysicsSurfaceDeformableBodyInputContainer*>(context);

            constexpr bool kReadOnly = true;

            omni::ExternalSerializer<kReadOnly> serializer(data, size);
            thisContainer->serialize<kReadOnly>(serializer);
        },
        this);
}

void PhysicsSurfaceDeformableBodyInputContainer::write(carb::ujitso::IWriter& writer)
{
    CARB_PROFILE_ZONE(0, "PhysicsSurfaceDeformableBodyInputContainer::write");

    size_t inputSizeEstimate = 0;

    // We calculate a rough input size estimate for serialization to reduce memory allocations
    // This should match what is written out in serialize()
    inputSizeEstimate += sizeof(uint32_t); // version

    // array data
    inputSizeEstimate += m_buildData.srcPointsInSim.size() * sizeof(carb::Float3) + sizeof(size_t);

    // Reserve at the input size estimate, should prevent having to resize the buffer
    omni::VectorSerializer serializer{ inputSizeEstimate };
    constexpr bool kLoading = false;
    serialize<kLoading>(serializer);
    writer.write(serializer.start, serializer.getCurrentPos());
}

void PhysicsSurfaceDeformableBodyInputContainer::copyOrComputeHash()
{
    CARB_PROFILE_ZONE(0, "PhysicsSurfaceDeformableBodyInputContainer::copyOrComputeHash");

    omni::physx::usdparser::MeshKey meshKey = m_result.meshKey;
    if (meshKey == omni::physx::usdparser::MeshKey())
    {
        // ask omni.physics to build the hash for us if it isn't already valid
        // needs to match what would be passed in if it is pre-computed
        meshKey = omni::physx::MeshKeyComputation::computeMeshKey(m_request.surfaceDeformableBodyView);
    }
    else
    {
        // in debug builds, check that the supplied hash matches what we would compute
        CARB_ASSERT(meshKey == omni::physx::MeshKeyComputation::computeMeshKey(m_request.surfaceDeformableBodyView));
    }

    // convert it to a 256 bit hash by duplicating it
    const carb::extras::hash128_t value = meshKey.getFullHash();
    m_hash = { value.d[0], value.d[1] };
}

} // namespace physx
} // namespace omni

// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <omni/physx/IPhysxCookingService.h>

#include <carb/ujitso/Container.h>

#include <carb/Types.h>
#include <carb/extras/Hash.h>

#define PHYSICS_TRIANGULATION_INPUT_CONTAINER_NAME "PhysicsTriangulationInputContainer"

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

// This data is used during the build step of the processor
// It is not guaranteed to be valid until fill() is called
struct PhysicsTriangulationBuildData
{
    pxr::VtArray<carb::Float3> points; // vert buffer
    pxr::VtArray<int32_t> indices; // index buffer
    pxr::VtArray<int32_t> faceCounts; // number of indices per face, lenth of array is number of faces total
    pxr::VtArray<int32_t> holeIndices; // face indices that should be treated as holes
    pxr::VtArray<uint16_t> faceMaterials; // mapping between faces and materials

    bool rightHandedOrientation; // winding of faces, duplicated from input data
};

// Container class for use with ujitso distribution
// If the processor is run locally the provided PhysicsTriangulationBuildData will be used directly (after a call to
// fill) Otherwise if run remotely, read/write will be invoked for network serialization
class PhysicsTriangulationInputContainer : public omni::core::Implements<carb::ujitso::IContainer>
{
    UJITSO_CONTAINER_DEFINITION(PhysicsTriangulationInputContainer);

public:
    PhysicsTriangulationInputContainer(const omni::physx::PhysxCookingComputeResult& result,
                                       const omni::physx::PhysxCookingComputeRequest& request);

    void fill() override;
    void read(carb::ujitso::IReader& reader) override;
    void write(carb::ujitso::IWriter& writer) override;

    const PhysicsTriangulationBuildData& getBuildData() const
    {
        return m_buildData;
    }

private:
    void copyOrComputeHash();

    template <bool readOnly, typename SerializerT>
    void serialize(SerializerT& serializer);

    omni::physx::PhysxCookingComputeResult m_result;
    omni::physx::PhysxCookingComputeRequest m_request;
    PhysicsTriangulationBuildData m_buildData;
};

} // namespace physx
} // namespace omni

// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "UjitsoContainerCommon.h"

#define PHYSICS_TRIANGULATION_INPUT_CONTAINER_NAME "PhysicsTriangulationInputContainer"

namespace omni
{
namespace physx
{
// This data is used during the build step of the processor
// It is not guaranteed to be valid until fill() is called
struct PhysicsTriangulationBuildData
{
    UninitVector<carb::Float3> points; // vert buffer
    UninitVector<int32_t> indices; // index buffer
    UninitVector<int32_t> faceCounts; // number of indices per face, lenth of array is number of faces total
    UninitVector<int32_t> holeIndices; // face indices that should be treated as holes
    UninitVector<uint16_t> faceMaterials; // mapping between faces and materials

    bool rightHandedOrientation; // winding of faces, duplicated from input data
};

// Container class for use with ujitso distribution
// If the processor is run locally the provided PhysicsTriangulationBuildData will be used directly (after a call to
// fill) Otherwise if run remotely, read/write will be invoked for network serialization
class PhysicsTriangulationInputContainer : public PhysicsInputContainerBase
{
    UJITSO_CONTAINER_DEFINITION(PhysicsTriangulationInputContainer);

public:
    PhysicsTriangulationInputContainer(const PhysxCookingComputeResult& result,
                                       const PhysxCookingComputeRequest& request);

    void fill() override;
    void read(carb::ujitso::IReader& reader) override;
    void write(carb::ujitso::IWriter& writer) override;

    const PhysicsTriangulationBuildData& getBuildData() const
    {
        return m_buildData;
    }

private:
    void copyOrComputeHash();

    // Copy the request's mesh view into m_buildData. Called either from fill() or, for an
    // asynchronous mesh-view request, from the constructor -- see shouldSnapshotInputNow().
    void copyInputViews();

    template <bool readOnly, typename SerializerT>
    void serialize(SerializerT& serializer);

    PhysicsTriangulationBuildData m_buildData;
};

} // namespace physx
} // namespace omni

// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

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
    PXR_NS::VtArray<carb::Float3> points; // vert buffer
    PXR_NS::VtArray<int32_t> indices; // index buffer
    PXR_NS::VtArray<int32_t> faceCounts; // number of indices per face, lenth of array is number of faces total
    PXR_NS::VtArray<int32_t> holeIndices; // face indices that should be treated as holes
    PXR_NS::VtArray<uint16_t> faceMaterials; // mapping between faces and materials

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

    template <bool readOnly, typename SerializerT>
    void serialize(SerializerT& serializer);

    PhysicsTriangulationBuildData m_buildData;
};

} // namespace physx
} // namespace omni

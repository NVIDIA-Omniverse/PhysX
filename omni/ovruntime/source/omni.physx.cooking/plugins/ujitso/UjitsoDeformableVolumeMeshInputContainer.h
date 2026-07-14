// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#pragma once

#include "UjitsoContainerCommon.h"

#define PHYSICS_DEFORMABLE_VOLUME_MESH_INPUT_CONTAINER_NAME "PhysicsDeformableVolumeMeshInputContainer"

namespace omni
{
namespace physx
{
// This data is used during the build step of the processor
// It is not guaranteed to be valid until fill() is called
struct PhysicsDeformableVolumeMeshBuildData
{
    PXR_NS::VtArray<carb::Float3> simPoints;
    PXR_NS::VtArray<carb::Float3> simBindPoints;
    PXR_NS::VtArray<carb::Int4> simIndices;
    PXR_NS::VtArray<carb::Float3> collBindPointsInSim;
    PXR_NS::VtArray<carb::Int4> collIndices;
    PXR_NS::VtArray<carb::Int3> collSurfaceIndices;
};

// Container class for use with ujitso distribution
// If the processor is run locally the provided PhysicsDeformableVolumeMeshBuildData will be used directly (after a call
// to fill) Otherwise if run remotely, read/write will be invoked for network serialization
class PhysicsDeformableVolumeMeshInputContainer : public PhysicsInputContainerBase
{
    UJITSO_CONTAINER_DEFINITION(PhysicsDeformableVolumeMeshInputContainer);

public:
    PhysicsDeformableVolumeMeshInputContainer(const PhysxCookingComputeResult& result,
                                              const PhysxCookingComputeRequest& request);

    void fill() override;
    void read(carb::ujitso::IReader& reader) override;
    void write(carb::ujitso::IWriter& writer) override;

    const PhysicsDeformableVolumeMeshBuildData& getBuildData() const
    {
        return m_buildData;
    }

private:
    void copyOrComputeHash();

    template <bool readOnly, typename SerializerT>
    void serialize(SerializerT& serializer);

    PhysicsDeformableVolumeMeshBuildData m_buildData;
};

} // namespace physx
} // namespace omni

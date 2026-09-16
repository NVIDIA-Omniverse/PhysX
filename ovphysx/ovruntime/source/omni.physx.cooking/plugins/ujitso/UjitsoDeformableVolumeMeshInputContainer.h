// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
    UninitVector<carb::Float3> simPoints;
    UninitVector<carb::Float3> simBindPoints;
    UninitVector<carb::Int4> simIndices;
    UninitVector<carb::Float3> collBindPointsInSim;
    UninitVector<carb::Int4> collIndices;
    UninitVector<carb::Int3> collSurfaceIndices;
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

    // Copy the request's volume mesh view into m_buildData. Called either from fill() or, for an
    // asynchronous mesh-view request, from the constructor -- see shouldSnapshotInputNow().
    void copyInputViews();

    template <bool readOnly, typename SerializerT>
    void serialize(SerializerT& serializer);

    PhysicsDeformableVolumeMeshBuildData m_buildData;
};

} // namespace physx
} // namespace omni

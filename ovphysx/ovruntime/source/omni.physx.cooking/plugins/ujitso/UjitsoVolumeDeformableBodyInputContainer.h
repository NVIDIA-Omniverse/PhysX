// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "UjitsoContainerCommon.h"

#define PHYSICS_VOLUME_DEFORMABLE_BODY_INPUT_CONTAINER_NAME "PhysicsVolumeDeformableBodyInputContainer"

namespace omni
{
namespace physx
{
// This data is used during the build step of the processor
// It is not guaranteed to be valid until fill() is called
struct PhysicsVolumeDeformableBodyBuildData
{
    PXR_NS::VtArray<carb::Float3> srcPointsInSim;
};

// Container class for use with ujitso distribution
// If the processor is run locally the provided PhysicsVolumeDeformableBodyBuildData will be used directly (after a call
// to fill) Otherwise if run remotely, read/write will be invoked for network serialization
class PhysicsVolumeDeformableBodyInputContainer : public PhysicsInputContainerBase
{
    UJITSO_CONTAINER_DEFINITION(PhysicsVolumeDeformableBodyInputContainer);

public:
    PhysicsVolumeDeformableBodyInputContainer(const PhysxCookingComputeResult& result,
                                              const PhysxCookingComputeRequest& request);

    void fill() override;
    void read(carb::ujitso::IReader& reader) override;
    void write(carb::ujitso::IWriter& writer) override;

    const PhysicsVolumeDeformableBodyBuildData& getBuildData() const
    {
        return m_buildData;
    }

private:
    void copyOrComputeHash();

    template <bool readOnly, typename SerializerT>
    void serialize(SerializerT& serializer);

    PhysicsVolumeDeformableBodyBuildData m_buildData;
};

} // namespace physx
} // namespace omni

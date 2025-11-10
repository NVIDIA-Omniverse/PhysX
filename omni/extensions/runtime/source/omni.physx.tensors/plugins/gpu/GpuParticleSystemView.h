// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../CommonTypes.h"
#include "../base/BaseParticleSystemView.h"
#include "GpuSimulationData.h"

#include <omni/physics/tensors/IParticleSystemView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class GpuSimulationView;

class GpuParticleSystemView : public BaseParticleSystemView
{
public:
    GpuParticleSystemView(GpuSimulationView* sim, const std::vector<ParticleSystemEntry>& entries);

    ~GpuParticleSystemView() override;
};

} // namespace tensors
} // namespace physx
} // namespace omni

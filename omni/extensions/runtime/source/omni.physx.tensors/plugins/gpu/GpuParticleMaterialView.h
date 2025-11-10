// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../CommonTypes.h"
#include "../base/BaseParticleMaterialView.h"
#include "GpuSimulationData.h"

#include <omni/physics/tensors/IParticleMaterialView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class GpuSimulationView;

class GpuParticleMaterialView : public BaseParticleMaterialView
{
public:
    GpuParticleMaterialView(GpuSimulationView* sim, const std::vector<ParticleMaterialEntry>& entries);

    ~GpuParticleMaterialView() override;
};

} // namespace tensors
} // namespace physx
} // namespace omni

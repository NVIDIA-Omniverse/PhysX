// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../CommonTypes.h"
#include "../base/BaseParticleMaterialView.h"
#include "CpuSimulationData.h"

#include <omni/physics/tensors/IParticleMaterialView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class CpuSimulationView;

class CpuParticleMaterialView : public BaseParticleMaterialView
{
public:
    CpuParticleMaterialView(CpuSimulationView* sim, const std::vector<ParticleMaterialEntry>& entries);

    ~CpuParticleMaterialView() override;
};

} // namespace tensors
} // namespace physx
} // namespace omni

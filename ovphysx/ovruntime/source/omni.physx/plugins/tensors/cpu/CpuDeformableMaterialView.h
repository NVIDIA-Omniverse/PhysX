// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "tensors/base/BaseDeformableMaterialView.h"
#include <omni/physics/tensors/IDeformableMaterialView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class CpuSimulationView;

class CpuDeformableMaterialView : public BaseDeformableMaterialView
{
public:
    CpuDeformableMaterialView(CpuSimulationView* sim, const std::vector<DeformableMaterialEntry>& entries);

    ~CpuDeformableMaterialView() override;
};

} // namespace tensors
} // namespace physx
} // namespace omni

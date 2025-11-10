// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "../base/BaseDeformableMaterialView.h"
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

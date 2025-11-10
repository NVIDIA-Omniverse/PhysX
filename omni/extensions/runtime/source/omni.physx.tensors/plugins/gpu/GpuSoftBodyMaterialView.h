// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../base/BaseSoftBodyMaterialView.h"
#include <omni/physics/tensors/ISoftBodyMaterialView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class GpuSimulationView;

class GpuSoftBodyMaterialView : public BaseSoftBodyMaterialView
{
public:
    GpuSoftBodyMaterialView(GpuSimulationView* sim, const std::vector<SoftBodyMaterialEntry>& entries);

    ~GpuSoftBodyMaterialView() override;
};

} // namespace tensors
} // namespace physx
} // namespace omni

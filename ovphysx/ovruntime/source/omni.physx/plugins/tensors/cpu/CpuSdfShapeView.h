// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "tensors/CommonTypes.h"
#include "tensors/base/BaseSdfShapeView.h"

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class CpuSimulationView;

class CpuSdfShapeView : public BaseSdfShapeView
{
public:
    CpuSdfShapeView(CpuSimulationView* sim, const std::vector<SdfShapeEntry>& entries);

    ~CpuSdfShapeView() override;
};

} // namespace tensors
} // namespace physx
} // namespace omni

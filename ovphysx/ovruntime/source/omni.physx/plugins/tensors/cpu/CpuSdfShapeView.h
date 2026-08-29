// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

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

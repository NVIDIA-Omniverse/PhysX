// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../CommonTypes.h"
#include "../base/BaseSdfShapeView.h"

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

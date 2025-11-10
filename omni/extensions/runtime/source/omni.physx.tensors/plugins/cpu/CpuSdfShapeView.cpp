// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "CpuSdfShapeView.h"
#include "CpuSimulationView.h"

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

CpuSdfShapeView::CpuSdfShapeView(CpuSimulationView* sim, const std::vector<SdfShapeEntry>& entries)
    : BaseSdfShapeView(sim, entries)
{
}

CpuSdfShapeView::~CpuSdfShapeView()
{
}

} // namespace tensors
} // namespace physx
} // namespace omni

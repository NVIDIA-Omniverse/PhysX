// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "CpuDeformableMaterialView.h"
#include "CpuSimulationView.h"
#include <PxPhysicsAPI.h>
#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{


CpuDeformableMaterialView::CpuDeformableMaterialView(CpuSimulationView* sim, const std::vector<DeformableMaterialEntry>& entries)
    : BaseDeformableMaterialView(sim, entries)
{
}

CpuDeformableMaterialView::~CpuDeformableMaterialView()
{
}

} // namespace tensors
} // namespace physx
} // namespace omni

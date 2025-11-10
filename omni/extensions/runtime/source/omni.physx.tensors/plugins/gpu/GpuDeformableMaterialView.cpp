// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "GpuDeformableMaterialView.h"
#include "GpuSimulationView.h"
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
GpuDeformableMaterialView::GpuDeformableMaterialView(GpuSimulationView* sim, const std::vector<DeformableMaterialEntry>& entries)
    : BaseDeformableMaterialView(sim, entries)
{
}

GpuDeformableMaterialView::~GpuDeformableMaterialView()
{
}

} // namespace tensors
} // namespace physx
} // namespace omni

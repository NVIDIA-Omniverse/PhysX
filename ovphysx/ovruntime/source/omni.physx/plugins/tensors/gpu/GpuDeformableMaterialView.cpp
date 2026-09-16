// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "tensors/gpu/GpuDeformableMaterialView.h"
#include "tensors/gpu/GpuSimulationView.h"
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

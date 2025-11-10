// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "CpuSoftBodyMaterialView.h"
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


CpuSoftBodyMaterialView::CpuSoftBodyMaterialView(CpuSimulationView* sim, const std::vector<SoftBodyMaterialEntry>& entries)
    : BaseSoftBodyMaterialView(sim, entries)
{
}

CpuSoftBodyMaterialView::~CpuSoftBodyMaterialView()
{
}

} // namespace tensors
} // namespace physx
} // namespace omni

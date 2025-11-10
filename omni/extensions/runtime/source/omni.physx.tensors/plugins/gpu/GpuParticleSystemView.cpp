// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "GpuParticleSystemView.h"
#include "GpuSimulationView.h"

#include "../GlobalsAreBad.h"
#include "../CommonTypes.h"
#include "../SimulationBackend.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

#include <omni/physics/tensors/TensorUtils.h>

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

GpuParticleSystemView::GpuParticleSystemView(GpuSimulationView* sim, const std::vector<ParticleSystemEntry>& entries)
    : BaseParticleSystemView(sim, entries) { }

GpuParticleSystemView::~GpuParticleSystemView() { }

}
}
}

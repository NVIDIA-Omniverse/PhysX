// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "CpuParticleMaterialView.h"
#include "CpuSimulationView.h"

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

CpuParticleMaterialView::CpuParticleMaterialView(CpuSimulationView* sim, const std::vector<ParticleMaterialEntry>& entries)
    : BaseParticleMaterialView(sim, entries) { }

CpuParticleMaterialView::~CpuParticleMaterialView() { }

}
}
}

// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

namespace omni
{
namespace physx
{
struct IPhysx;
struct IPhysxSimulation;
struct IPhysxPrivate;
struct IPhysxJoint;

namespace tensors
{
class SimulationBackend;

SimulationBackend& GetSimulationBackend();

extern omni::physx::IPhysx* g_physx;
extern omni::physx::IPhysxSimulation* g_physxSimulation;
extern omni::physx::IPhysxPrivate* g_physxPrivate;
extern omni::physx::IPhysxJoint* g_physxJoint;

} // namespace tensors
} // namespace physx
} // namespace omni

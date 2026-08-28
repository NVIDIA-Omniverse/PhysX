// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <omni/physx/IOptionalCuda.h>
#include <omni/physx/IPhysxFoundation.h>

namespace omni
{
namespace physx
{
namespace foundation
{

// Explicit runtime setup/teardown for the statically linked foundation library.
// These are not Carbonite plugin lifecycle callbacks: this library has no
// plugin descriptor and is not published or acquired through carb::Framework.
//
// Call initializeRuntime() after carb.settings is available and before PhysX
// creates CUDA context managers. Call shutdownRuntime() after those consumers
// are gone so the PhysX GPU runtime libraries unload in a deterministic place,
// not from static object teardown.
void initializeRuntime();
void shutdownRuntime();

IPhysxFoundation& getInterface();
IOptionalCuda& getOptionalCudaInterface();

} // namespace foundation
} // namespace physx
} // namespace omni

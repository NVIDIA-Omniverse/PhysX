// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>

namespace omni
{
namespace physx
{
namespace usdparser
{

// Character controller parsing
// fills in CCT desc from a given USD prim
CctDesc* parseCct(AttachedStage& attachedStage, const pxr::UsdPrim& usdPrim);
} // namespace usdparser
} // namespace physx
} // namespace omni

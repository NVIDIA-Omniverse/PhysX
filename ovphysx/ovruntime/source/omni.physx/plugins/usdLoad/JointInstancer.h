// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include "LoadTools.h"

namespace omni
{
namespace physx
{
namespace usdparser
{
void parseJointInstancer(AttachedStage& attachedStage, omni::physics::parse::ObjectKey instancerKey);
}
} // namespace physx
} // namespace omni

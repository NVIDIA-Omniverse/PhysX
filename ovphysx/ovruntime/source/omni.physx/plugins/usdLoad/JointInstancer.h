// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include "LoadTools.h"

namespace omni
{
namespace physx
{
namespace usdparser
{
void parseJointInstancer(AttachedStage& attachedStage, const PXR_NS::SdfPath& instancerPath);
}
} // namespace physx
} // namespace omni

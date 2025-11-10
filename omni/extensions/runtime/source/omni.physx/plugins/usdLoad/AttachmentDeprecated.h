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

// PhysX attachment parser
PhysxAttachmentDesc* parsePhysxAttachmentDeprecated(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim);

} // namespace usdparser
} // namespace physx
} // namespace omni

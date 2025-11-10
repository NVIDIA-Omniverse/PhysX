// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdLoad.h"

namespace omni
{
namespace physics
{
namespace schema
{
AttachmentDesc* parseAttachment(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& prim, uint64_t typeFlags);
ElementCollisionFilterDesc* parseCollisionFilter(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& prim);
} // namespace schema
} // namespace physics
} // namespace omni

// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
AttachmentDesc* parseAttachment(const PXR_NS::UsdStageWeakPtr stage, const PXR_NS::UsdPrim& prim, uint64_t typeFlags);
ElementCollisionFilterDesc* parseCollisionFilter(const PXR_NS::UsdStageWeakPtr stage, const PXR_NS::UsdPrim& prim);
} // namespace schema
} // namespace physics
} // namespace omni

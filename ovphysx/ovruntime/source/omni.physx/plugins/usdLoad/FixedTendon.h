// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <private/omni/physx/PhysxUsd.h>

using namespace PXR_NS;

namespace omni
{
namespace physx
{
namespace usdparser
{
// Fixed tendon parsing lives in the parse library and the native walker's
// `finalizeFixedTendons` pass; see `omni.physics.parse/ParseFixedTendon.cpp`
// and REQ-PARSE-TENDON-002. Only the engine-side create helper remains here.

void createFixedTendons(AttachedStage& attachedStage, TendonAxisMap& tendonAxes, FixedTendonVector& fixedTendons);
} // namespace usdparser
} // namespace physx
} // namespace omni

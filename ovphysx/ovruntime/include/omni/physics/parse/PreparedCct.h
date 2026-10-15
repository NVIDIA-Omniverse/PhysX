// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */
#pragma once
#include "ParseApi.h"
#include "PreparedAttribute.h"
namespace omni::physics::parse
{
struct PreparedCct
{
    bool prepared = false;
    ObjectKey key, owner;
    carb::Float3 pos{}, scale{};
    float radius = 0.0f, halfHeight = 0.0f;
    PreparedAttribute<float> slopeLimit;
};
PreparedCct prepareCct(ParseContext& context, ObjectKey key, const CctInfo& info);
bool fillCct(const PreparedCct& input, CapsuleCctDesc& output) noexcept;
}

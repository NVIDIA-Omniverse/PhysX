// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2 AC-3
 */
#pragma once
#include "ParseApi.h"
#include "PreparedAttribute.h"
#include <cmath>
namespace omni::physics::parse
{
inline void fillTendonFloat(const PreparedAttribute<float>& input, float min, float max, float& output) noexcept
{
    if (!input.valid) return;
    float value = input.value;
    if (std::isinf(value)) value = value < 0.0f ? min : max;
    if (value < min) value = min;
    else if (value > max) value = max;
    output = value;
}
struct PreparedTendonValues
{
    PreparedAttribute<float> gearing, forceCoefficient, stiffness, limitStiffness, damping, offset, restLength, lowerLimit, upperLimit;
    PreparedAttribute<bool> enabled;
};
struct PreparedFixedTendon
{
    ObjectKey key;
    TokenId instance;
    FixedTendonParseInfo info;
    bool prepared = false, root = false;
    PreparedTendonValues values;
};
struct PreparedSpatialTendon
{
    ObjectKey key, parent;
    TokenId instance, parentToken;
    ObjectType type = eUndefined;
    carb::Float3 scale{ 1.0f, 1.0f, 1.0f };
    PreparedAttribute<carb::Float3> localPos;
    PreparedTendonValues values;
};
void prepareFixedTendons(ParseContext&, ObjectKey, const FixedTendonParseInfo&, std::vector<PreparedFixedTendon>&);
bool fillFixedTendonAxis(const PreparedFixedTendon&, PhysxTendonAxisDesc&) noexcept;
bool fillFixedTendonRoot(const PreparedFixedTendon&, PhysxTendonFixedDesc&) noexcept;
void prepareSpatialTendons(ParseContext&, ObjectKey, const SpatialTendonParseInfo&, std::vector<PreparedSpatialTendon>&);
bool fillSpatialTendon(const PreparedSpatialTendon&, PhysxTendonAttachmentDesc&) noexcept;
}

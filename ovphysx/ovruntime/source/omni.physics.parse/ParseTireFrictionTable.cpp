// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-VEH-TIREFRICTION-001
 * @covers AC-1 AC-2 AC-3
 */

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

namespace omni::physics::parse
{

DescPtr<TireFrictionTableDesc> parseTireFrictionTable(ParseContext& ctx, ObjectKey key,
                                                      const TireFrictionTableInfo& info)
{
    IPhysicsSource& src = ctx.source();
    const std::string ownerPath(src.sourceKeyToString(key));

    if (info.defaultFrictionValue < 0.0f)
    {
        CARB_LOG_ERROR("Usd Physics: tire friction table \"%s\": defaultFrictionValue must be >= 0.",
                       ownerPath.c_str());
        return {};
    }

    // Validate each material rel target has PhysicsMaterialAPI applied.
    // The materialPaths are filtered here; invalid entries cause the
    // whole table to be dropped (return nullptr) with an error.
    const TokenId materialApi = src.internToken("PhysicsMaterialAPI");
    for (const ObjectKey matKey : info.materialPaths)
    {
        if (!matKey.valid())
        {
            CARB_LOG_ERROR(
                "Usd Physics: tire friction table \"%s\": \"groundMaterials\" relationship has nonexistent target.",
                ownerPath.c_str());
            return {};
        }
        if (!src.hasSchema(matKey, materialApi))
        {
            CARB_LOG_ERROR(
                "Usd Physics: tire friction table \"%s\": \"groundMaterials\" target \"%s\" must have MaterialAPI applied.",
                ownerPath.c_str(),
                std::string(src.sourceKeyToString(matKey)).c_str());
            return {};
        }
    }

    if (!info.materialPaths.empty() &&
        info.materialPaths.size() != info.frictionValues.size())
    {
        CARB_LOG_ERROR(
            "Usd Physics: tire friction table \"%s\": attribute \"frictionValues\" needs to have the same number of entries as relationships in \"groundMaterials\".",
            ownerPath.c_str());
        return {};
    }

    DescPtr<TireFrictionTableDesc> desc = allocateDesc<TireFrictionTableDesc>(ctx.descriptorAllocator());
    desc->key                 = key;
    desc->defaultFrictionValue = info.defaultFrictionValue;
    desc->materialPaths        = info.materialPaths;
    desc->frictionValues       = info.frictionValues;
    return desc;
}

} // namespace omni::physics::parse

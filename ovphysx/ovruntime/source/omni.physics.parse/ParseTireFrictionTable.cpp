// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-VEH-TIREFRICTION-001
 * @covers AC-1 AC-2 AC-3
 */

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedVehicle.h>
#include <stdexcept>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

namespace omni::physics::parse
{

PreparedTireFrictionTable prepareTireFrictionTable(ParseContext& ctx, ObjectKey key, const TireFrictionTableInfo& info)
{
    PreparedTireFrictionTable input;
    input.key = key;
    input.info = info;
    input.reads = prepareVehicleReads(ctx.source(), key, {

    });
    for (ObjectKey material : info.materialPaths)
        input.reads.references.push_back({ material, std::string(ctx.source().sourceKeyToString(material)),
            material.valid() && ctx.source().hasSchema(material, ctx.source().internToken("PhysicsMaterialAPI")) });
    return input;
}

void prepareTireFrictionTableStorage(const PreparedTireFrictionTable& input, TireFrictionTableDesc& output)
{
    output.materialPaths.resize(input.info.materialPaths.size());
    output.frictionValues.resize(input.info.frictionValues.size());
}

bool fillTireFrictionTable(const PreparedTireFrictionTable& input, TireFrictionTableDesc& output, VehicleFillStatus& status) noexcept
{
    PreparedVehicleReader src(input.reads, status);
    ObjectKey key = input.key;
    const TireFrictionTableInfo& info = input.info;
    VehicleOutputVector<typename decltype(output.materialPaths)::value_type> materialPathsOut(output.materialPaths, vehicle_output_slot::materialPaths, status);
    VehicleOutputVector<typename decltype(output.frictionValues)::value_type> frictionValuesOut(output.frictionValues, vehicle_output_slot::frictionValues, status);


    const std::string& ownerPath = src.sourceKeyToString(key);

    if (info.defaultFrictionValue < 0.0f)
    {
        VEHICLE_LOG_ERROR(status, "Usd Physics: tire friction table \"%s\": defaultFrictionValue must be >= 0.",
                       ownerPath.c_str());
        return false;
    }

    // Validate each material rel target has PhysicsMaterialAPI applied.
    // The materialPaths are filtered here; invalid entries cause the
    // whole table to be dropped (return nullptr) with an error.
    const std::string_view materialApi = src.attribute("PhysicsMaterialAPI");
    for (const ObjectKey matKey : info.materialPaths)
    {
        if (!matKey.valid())
        {
            VEHICLE_LOG_ERROR(status,
                "Usd Physics: tire friction table \"%s\": \"groundMaterials\" relationship has nonexistent target.",
                ownerPath.c_str());
            return false;
        }
        if (!src.hasSchema(matKey, materialApi))
        {
            VEHICLE_LOG_ERROR(status,
                "Usd Physics: tire friction table \"%s\": \"groundMaterials\" target \"%s\" must have MaterialAPI applied.",
                ownerPath.c_str(),
                src.sourceKeyToString(matKey).c_str());
            return false;
        }
    }

    if (!info.materialPaths.empty() &&
        info.materialPaths.size() != info.frictionValues.size())
    {
        VEHICLE_LOG_ERROR(status,
            "Usd Physics: tire friction table \"%s\": attribute \"frictionValues\" needs to have the same number of entries as relationships in \"groundMaterials\".",
            ownerPath.c_str());
        return false;
    }

    TireFrictionTableDesc* desc = &output;
    desc->key                 = key;
    desc->defaultFrictionValue = info.defaultFrictionValue;
    materialPathsOut        = info.materialPaths;
    frictionValuesOut       = info.frictionValues;
    return true;
}

void finishTireFrictionTableStorage(TireFrictionTableDesc& output, const VehicleFillStatus& status)
{
    output.materialPaths.resize(status.written[vehicle_output_slot::materialPaths]);
    output.frictionValues.resize(status.written[vehicle_output_slot::frictionValues]);
}

DescPtr<TireFrictionTableDesc> parseTireFrictionTable(ParseContext& ctx, ObjectKey key, const TireFrictionTableInfo& info)
{
    const PreparedTireFrictionTable input = prepareTireFrictionTable(ctx, key, info);
    DescPtr<TireFrictionTableDesc> output = allocateDesc<TireFrictionTableDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareTireFrictionTableStorage(input, *output);
    VehicleFillStatus status;
    const bool valid = fillTireFrictionTable(input, *output, status);
    status.report();
    if (status.incomplete) throw std::logic_error("Incomplete prepared vehicle input or storage");
    if (!valid) return {};
    finishTireFrictionTableStorage(*output, status);
    return output;
}


} // namespace omni::physics::parse

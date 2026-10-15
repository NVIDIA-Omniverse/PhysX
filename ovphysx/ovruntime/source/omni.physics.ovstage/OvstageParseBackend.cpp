// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-2 AC-6
 */

#include <omni/physics/ovstage/OvstageParseBackend.h>

#include <omni/physics/parse/IParseBackend.h>

#include "OvstageSource.h"

namespace omni
{
namespace physics
{
namespace ovstage
{

namespace
{

class OvstageParseBackend final : public parse::IParseBackend
{
public:
    std::string_view id() const override
    {
        return "ovstage";
    }

    parse::SourceBundle createSource(const parse::AttachTarget& target) override
    {
        parse::SourceBundle bundle;
        if (!target.nativeStage)
            return bundle;

        // nativeStage carries the instance and dictionary. An optional explicit
        // developer backing id is classified by the runtime before source creation;
        // zero never asks ovstage for its private USD stage.
        const OvstageAttach* attach = static_cast<const OvstageAttach*>(target.nativeStage);
        if (!attach->instance)
            return bundle;

        ovx_path_dictionary_t* dict = attach->dict ? attach->dict : ovstage_get_path_dictionary(attach->instance);
        const ovstage_ordinal_t readOrdinal =
            target.readOrdinal ? static_cast<ovstage_ordinal_t>(target.readOrdinal) : 1;
        std::unique_ptr<OvstageSource> source = std::make_unique<OvstageSource>(
            attach->instance, dict, readOrdinal, target.residentBackingStageId);
        // Change feed (ADR-0003 M3): pull-based, drained over an explicit ordinal
        // range via IPhysxSimulation::updateFromOvStage. Null for an unconfigured
        // source. The write sink stays null (no ovstage write path, by design).
        bundle.changeFeed = source->createChangeFeed();
        bundle.source = std::move(source);
        return bundle;
    }
};

} // namespace

std::unique_ptr<parse::IParseBackend> makeOvstageParseBackend()
{
    return std::make_unique<OvstageParseBackend>();
}

} // namespace ovstage
} // namespace physics
} // namespace omni

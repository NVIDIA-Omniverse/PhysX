// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-2
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

        // nativeStage is a const OvstageAttach* (instance + path dictionary +
        // raw backing candidate). Backing identity is classified once, before
        // session mutation, and carried on AttachTarget. Do not query the raw
        // payload here: an error or nonresident id must not become a local USD
        // fallback after attach has started.
        const OvstageAttach* attach = static_cast<const OvstageAttach*>(target.nativeStage);

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

ovstage_api_status_t queryBackingUsdStageId(const void* attachPayload, uint64_t& outStageId)
{
    outStageId = 0;
    if (!attachPayload)
        return OVSTAGE_ERROR_INVALID_ARGUMENT;
    const OvstageAttach* attach = static_cast<const OvstageAttach*>(attachPayload);
    if (!attach->instance)
        return OVSTAGE_ERROR_INVALID_ARGUMENT;

    // Preserve an explicit consumer-supplied candidate. Residency and zero-id
    // validation belong to the attach-time runtime classifier, not this query.
    if (attach->usdStageId != 0)
    {
        outStageId = attach->usdStageId;
        return OVSTAGE_OK;
    }

    return ovstage_get_usd_stage_id(attach->instance, &outStageId);
}

} // namespace ovstage
} // namespace physics
} // namespace omni

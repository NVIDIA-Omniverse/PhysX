// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OVSTAGE-OUTPUT-001
 * @covers AC-1 AC-3 AC-4 AC-6
 */

// [tutorial-start]
#include <ovphysx/experimental/OvStageOutput.hpp>
#include <ovstage/ovstage.h>

#include <cstdio>

// The caller has populated and attached this stage at sealed ordinal 1.
// This sample has no application edits to drain back into physics.
bool publishFrames(ovphysx_handle_t physics, ovstage_instance_t* stage)
{
    // Destroy the cache before its owning physics instance and stage.
    ovphysx::utils::OvStageOutputCache cache;
    for (ovstage_ordinal_t outputOrdinal = 2; outputOrdinal < 7; ++outputOrdinal)
    {
        if (ovphysx_step_sync(physics, 1.0f / 60.0f).status != OVPHYSX_API_SUCCESS)
            return false;

        const ovphysx::utils::OvStageOutputResult result =
            ovphysx::utils::writeWorldTransformsToOvstage(physics, stage, outputOrdinal, &cache);
        if (!result.ok())
        {
            std::fprintf(stderr, "World transform output failed: %s\n", result.message.c_str());
            return false;
        }
        if (result.matricesWritten == 0 || result.instancerGroupsSkipped != 0)
            return false;

        // Other producers can contribute to this ordinal before the caller seals it.
        // Never pass these physics output ordinals to ovphysx_update_from_ovstage.
        ovstage_write_floor_desc_t floor{};
        floor.ordinal = outputOrdinal;
        floor.scope = OVSTAGE_SCOPE_ALL;
        const ovstage_enqueue_result_t seal = ovstage_advance_write_floor(stage, &floor);
        if (seal.status != OVSTAGE_OK)
            return false;
        ovstage_op_wait_result_t wait{};
        const ovstage_api_status_t status = ovstage_wait_op(stage, seal.op_index, OVSTAGE_TIMEOUT_INFINITE, &wait);
        const bool ok = status == OVSTAGE_OK && wait.error_op_id_count == 0;
        const ovstage_api_status_t released = ovstage_release_op(stage, seal.op_index);
        if (!ok || released != OVSTAGE_OK)
            return false;

        std::printf("Published %zu world matrices at ordinal %llu\n", result.matricesWritten,
                    static_cast<unsigned long long>(outputOrdinal));
    }
    return true;
}
// [tutorial-end]

#include "ovstage_sample.h"

int main()
{
    if (ovphysx_initialize().status != OVPHYSX_API_SUCCESS)
        return 1;

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t physics = OVPHYSX_INVALID_HANDLE;
    if (ovphysx_create_instance(&args, &physics).status != OVPHYSX_API_SUCCESS)
    {
        ovphysx_shutdown();
        return 1;
    }

    ovphysx_sample_stage_attachment_t attachment{};
    bool ok = ovphysx_sample_attach_usd_with_ovstage(
                  physics, OVPHYSX_TEST_DATA "/simple_physics_scene_cpu.usda", &attachment) != 0;
    if (ok)
        ok = publishFrames(physics, attachment.stage);
    ok = ovphysx_sample_destroy_stage(physics, &attachment) != 0 && ok;
    ok = ovphysx_destroy_instance(physics).status == OVPHYSX_API_SUCCESS && ok;
    ovphysx_shutdown();
    return ok ? 0 : 1;
}

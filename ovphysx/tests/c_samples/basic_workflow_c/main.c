// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <ovphysx/ovphysx.h>
#include "ovstage_sample.h"
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_population.h>
#include <stdio.h>
#include <string.h>

static int complete_task(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    if (task.status != OVPHYSX_API_SUCCESS)
        return 0;
    ovphysx_wait_task_result_t waited = {0};
    ovphysx_result_t result =
        ovphysx_wait_task(handle, task, OVPHYSX_TIMEOUT_INFINITE, &waited);
    for (size_t i = 0; i < waited.num_errors; ++i) {
        ovphysx_string_t error = ovphysx_get_last_task_error(handle, waited.error_tasks[i]);
        fprintf(stderr, "%.*s\n", (int)error.length, error.ptr);
    }
    ovphysx_destroy_wait_task_result(&waited);
    return result.status == OVPHYSX_API_SUCCESS;
}

int main(void)
{
    if (ovphysx_initialize().status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "Failed to initialize ovphysx\n");
        return 1;
    }

    ovphysx_instance_handle_t handle = 0;
    if (ovphysx_create_instance(&handle).status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "Failed to create instance\n");
        ovphysx_shutdown();
        return 1;
    }

    if (!ovphysx_sample_register_physx_schemas()) {
        ovphysx_destroy_instance(handle);
        ovphysx_shutdown();
        return 1;
    }

    // Create an ovstage and populate it from USD (public ovstage API).
    ovstage_instance_desc_t stage_desc;
    memset(&stage_desc, 0, sizeof(stage_desc));
    stage_desc.name = "ovphysx-basic";
    ovstage_instance_t* stage = NULL;
    if (ovstage_create_instance(&stage_desc, &stage) != OVSTAGE_OK) {
        fprintf(stderr, "Failed to create ovstage\n");
        ovphysx_destroy_instance(handle);
        ovphysx_shutdown();
        return 1;
    }

    const char* usd_path = OVPHYSX_TEST_DATA "/simple_physics_scene.usda";
    ovx_string_t path;
    path.ptr = usd_path;
    path.length = strnlen(usd_path, 4096);

    const uint64_t ordinal = 1;
    ovstage_population_enqueue_result_t open = ovphysx_sample_open_usd_from_file(
        stage, path, ordinal, 0.0, /*with_rendering=*/true);
    ovstage_population_op_wait_result_t open_wait;
    memset(&open_wait, 0, sizeof(open_wait));
    if (open.status != OVSTAGE_OK ||
        ovstage_population_wait_op(stage, open.op_index, OVSTAGE_TIMEOUT_INFINITE, &open_wait) != OVSTAGE_OK) {
        fprintf(stderr, "Failed to populate ovstage\n");
        ovstage_destroy_instance(stage);
        ovphysx_destroy_instance(handle);
        ovphysx_shutdown();
        return 1;
    }

    ovstage_write_floor_desc_t floor_desc;
    memset(&floor_desc, 0, sizeof(floor_desc));
    floor_desc.ordinal = ordinal;
    floor_desc.scope = OVSTAGE_SCOPE_ALL;
    ovstage_enqueue_result_t floor = ovstage_advance_write_floor(stage, &floor_desc);
    ovstage_op_wait_result_t floor_wait;
    memset(&floor_wait, 0, sizeof(floor_wait));
    if (floor.status != OVSTAGE_OK ||
        ovstage_wait_op(stage, floor.op_index, OVSTAGE_TIMEOUT_INFINITE, &floor_wait) != OVSTAGE_OK ||
        floor_wait.error_op_id_count != 0) {
        fprintf(stderr, "Failed to seal ovstage ordinal\n");
        ovstage_destroy_instance(stage);
        ovphysx_destroy_instance(handle);
        ovphysx_shutdown();
        return 1;
    }
    if (ovstage_release_op(stage, floor.op_index) != OVSTAGE_OK) {
        fprintf(stderr, "Failed to release ovstage floor operation\n");
        ovstage_destroy_instance(stage);
        ovphysx_destroy_instance(handle);
        ovphysx_shutdown();
        return 1;
    }

    if (!complete_task(handle, ovphysx_attach_ovstage(handle, stage, ordinal))) {
        fprintf(stderr, "Failed to attach ovstage\n");
        // Failed attach can retain ownership if its rollback fails.
        if (ovphysx_destroy_instance(handle).status != OVPHYSX_API_SUCCESS)
            return 1;
        ovstage_destroy_instance(stage);
        ovphysx_shutdown();
        return 1;
    }

    // step_sync steps and waits; a nonzero status means the step failed.
    if (ovphysx_step_sync(handle, 1.0f / 60.0f).status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "Simulation step failed\n");
        if (ovphysx_destroy_instance(handle).status != OVPHYSX_API_SUCCESS)
            return 1;
        ovstage_destroy_instance(stage);
        ovphysx_shutdown();
        return 1;
    }

    int detached = complete_task(handle, ovphysx_detach_ovstage(handle));
    if (ovphysx_destroy_instance(handle).status != OVPHYSX_API_SUCCESS)
        return 1;
    ovstage_destroy_instance(stage);
    ovphysx_shutdown();
    return detached ? 0 : 1;
}

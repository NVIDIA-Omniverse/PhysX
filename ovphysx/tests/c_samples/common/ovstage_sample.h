// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OVSTAGE-SCHEMA-001
 * @covers AC-4
 */

// @implements REQ-CAPI-OVSTAGE-XFORM-001
// @covers AC-1

#ifndef OVPHYSX_SAMPLE_OVSTAGE_SAMPLE_H
#define OVPHYSX_SAMPLE_OVSTAGE_SAMPLE_H

#include <ovphysx/ovphysx.h>
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_population.h>

#include "schema_support.h"

#include <stdio.h>
#include <string.h>

/* Wait for this operation and report its diagnostic before releasing result storage. */
static inline ovphysx_result_t ovphysx_sample_complete_task(
    ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    ovphysx_result_t status = { task.status };
    ovphysx_wait_task_result_t result = { 0 };
    if (task.status == OVPHYSX_API_SUCCESS)
        status = ovphysx_wait_task(handle, task, OVPHYSX_TIMEOUT_INFINITE, &result);
    for (size_t index = 0; index < result.num_errors; ++index)
    {
        const ovphysx_string_t error = ovphysx_get_last_task_error(handle, result.error_tasks[index]);
        fprintf(stderr, "ovphysx task failed: %.*s\n", (int)error.length, error.ptr ? error.ptr : "");
    }
    if (status.status != OVPHYSX_API_SUCCESS && !result.num_errors)
    {
        const ovphysx_string_t error = ovphysx_get_last_error();
        fprintf(stderr, "ovphysx task rejected or wait failed (%d): %.*s\n", (int)status.status,
                (int)error.length, error.ptr ? error.ptr : "");
    }
    ovphysx_destroy_wait_task_result(&result);
    return status;
}

typedef struct ovphysx_sample_stage_attachment_t
{
    ovstage_instance_t* stage;
    uint64_t ordinal;
} ovphysx_sample_stage_attachment_t;

static inline int ovphysx_sample_destroy_stage(
    ovphysx_instance_handle_t handle,
    ovphysx_sample_stage_attachment_t* attachment)
{
    if (!attachment)
    {
        return 1;
    }
    if (handle)
    {
        const ovphysx_result_t detach = ovphysx_sample_complete_task(handle, ovphysx_detach_ovstage(handle));
        if (detach.status != OVPHYSX_API_SUCCESS)
        {
            fprintf(stderr, "ERROR: ovphysx_detach_ovstage failed: %d; retaining caller-owned Stage\n",
                    (int)detach.status);
            return 0;
        }
    }
    if (attachment->stage)
    {
        ovstage_destroy_instance(attachment->stage);
    }
    attachment->stage = NULL;
    attachment->ordinal = 0;
    return 1;
}

/* Register external schemas before population: USD builds its registry only once. */
static inline int ovphysx_sample_register_physx_schemas(void)
{
    static int s_registered = 0;
    if (s_registered)
    {
        return 1;
    }

    physx_sample_schema_paths_t paths;
    char error[PHYSX_SAMPLE_SCHEMA_PATH_SIZE];
    ovx_string_t schema_paths[2];
    size_t i;
    if (!physx_sample_schema_paths(getenv("PHYSX_USD_SCHEMAS_ROOT"), &paths, error, sizeof(error)))
    {
        fprintf(stderr, "ERROR: %s\n", error);
        return 0;
    }
    for (i = 0; i < 2; ++i)
    {
        schema_paths[i].ptr = paths.resources[i];
        schema_paths[i].length = paths.lengths[i];
    }
    const ovstage_api_status_t status = ovstage_population_register_usd_schemas(schema_paths, 2);
    if (status != OVSTAGE_OK)
    {
        const ovx_string_t err = ovstage_population_get_last_error();
        fprintf(stderr, "ERROR: ovstage_population_register_usd_schemas failed: %d %.*s\n",
                (int)status, (int)err.length, err.ptr ? err.ptr : "");
        return 0;
    }
    s_registered = 1;
    return 1;
}

static inline int ovphysx_sample_attach_usd_with_ovstage(
    ovphysx_instance_handle_t handle,
    const char* usd_path,
    ovphysx_sample_stage_attachment_t* attachment)
{
    if (!handle || !usd_path || !attachment)
    {
        fprintf(stderr, "ERROR: invalid ovstage attach arguments\n");
        return 0;
    }

    attachment->stage = NULL;
    attachment->ordinal = 1;

    if (!ovphysx_sample_register_physx_schemas())
    {
        return 0;
    }

    ovstage_instance_desc_t desc;
    memset(&desc, 0, sizeof(desc));
    desc.name = "ovphysx-sample-stage";

    ovstage_api_status_t create_status = ovstage_create_instance(&desc, &attachment->stage);
    if (create_status != OVSTAGE_OK || !attachment->stage)
    {
        fprintf(stderr, "ERROR: ovstage_create_instance failed: %d\n", (int)create_status);
        return 0;
    }

    ovx_string_t path;
    path.ptr = usd_path;
    /* usd_path is a filesystem path, so PATH_MAX bounds the scan. */
    path.length = strnlen(usd_path, 4096);

    ovstage_population_enqueue_result_t enqueue = ovstage_population_open_usd_from_file(
        attachment->stage,
        path,
        attachment->ordinal,
        0.0,
        OVSTAGE_POPULATION_DOMAIN_ALL);
    if (enqueue.status != OVSTAGE_OK)
    {
        ovx_string_t err = ovstage_population_get_last_error();
        fprintf(stderr, "ERROR: ovstage population enqueue failed: %d %.*s\n",
                (int)enqueue.status, (int)err.length, err.ptr ? err.ptr : "");
        ovphysx_sample_destroy_stage(handle, attachment);
        return 0;
    }

    ovstage_population_op_wait_result_t wait_result;
    memset(&wait_result, 0, sizeof(wait_result));
    ovstage_api_status_t wait_status = ovstage_population_wait_op(
        attachment->stage,
        enqueue.op_index,
        OVSTAGE_TIMEOUT_INFINITE,
        &wait_result);
    if (wait_status != OVSTAGE_OK)
    {
        ovx_string_t err = ovstage_population_get_last_error();
        fprintf(stderr, "ERROR: ovstage population wait failed: %d %.*s\n",
                (int)wait_status, (int)err.length, err.ptr ? err.ptr : "");
        ovphysx_sample_destroy_stage(handle, attachment);
        return 0;
    }

    ovstage_write_floor_desc_t floor_desc;
    memset(&floor_desc, 0, sizeof(floor_desc));
    floor_desc.ordinal = attachment->ordinal;
    floor_desc.scope = OVSTAGE_SCOPE_ALL;

    ovstage_enqueue_result_t floor = ovstage_advance_write_floor(attachment->stage, &floor_desc);
    ovstage_op_wait_result_t floor_wait;
    memset(&floor_wait, 0, sizeof(floor_wait));
    if (floor.status != OVSTAGE_OK ||
        ovstage_wait_op(attachment->stage, floor.op_index, OVSTAGE_TIMEOUT_INFINITE, &floor_wait) != OVSTAGE_OK ||
        floor_wait.error_op_id_count != 0)
    {
        fprintf(stderr, "ERROR: ovstage write-floor advance failed\n");
        ovphysx_sample_destroy_stage(handle, attachment);
        return 0;
    }
    if (ovstage_release_op(attachment->stage, floor.op_index) != OVSTAGE_OK)
    {
        fprintf(stderr, "ERROR: ovstage floor operation release failed\n");
        ovphysx_sample_destroy_stage(handle, attachment);
        return 0;
    }

    const ovphysx_result_t attach = ovphysx_sample_complete_task(
        handle, ovphysx_attach_ovstage(handle, attachment->stage, attachment->ordinal));
    if (attach.status != OVPHYSX_API_SUCCESS)
    {
        fprintf(stderr, "ERROR: ovphysx_attach_ovstage failed: %d\n", (int)attach.status);
        ovphysx_sample_destroy_stage(handle, attachment);
        return 0;
    }

    return 1;
}

#endif // OVPHYSX_SAMPLE_OVSTAGE_SAMPLE_H

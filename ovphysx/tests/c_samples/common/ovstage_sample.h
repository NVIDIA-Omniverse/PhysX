// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#ifndef OVPHYSX_SAMPLE_OVSTAGE_SAMPLE_H
#define OVPHYSX_SAMPLE_OVSTAGE_SAMPLE_H

#include <ovphysx/ovphysx.h>
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_population.h>

#include <stdio.h>
#include <string.h>

typedef struct ovphysx_sample_stage_attachment_t
{
    ovstage_instance_t* stage;
    uint64_t ordinal;
} ovphysx_sample_stage_attachment_t;

static inline int ovphysx_sample_destroy_stage(
    ovphysx_handle_t handle,
    ovphysx_sample_stage_attachment_t* attachment)
{
    if (!attachment)
    {
        return 1;
    }
    if (handle)
    {
        const ovphysx_result_t detach = ovphysx_detach_ovstage(handle);
        if (detach.status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t err = ovphysx_get_last_error();
            fprintf(stderr, "ERROR: ovphysx_detach_ovstage failed: %d %.*s; retaining caller-owned Stage\n",
                    (int)detach.status, (int)err.length, err.ptr ? err.ptr : "");
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

static inline int ovphysx_sample_attach_usd_with_ovstage(
    ovphysx_handle_t handle,
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
    /* bounded length: usd_path is a filesystem path (PATH_MAX ceiling) */
    path.length = strnlen(usd_path, 4096);

    ovstage_population_enqueue_result_t enqueue = ovstage_population_open_usd_from_file(
        attachment->stage,
        path,
        attachment->ordinal,
        0.0,
        OVSTAGE_POPULATION_DOMAIN_PHYSICS);
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

    ovphysx_result_t attach = ovphysx_attach_ovstage(handle, attachment->stage, attachment->ordinal);
    if (attach.status != OVPHYSX_API_SUCCESS)
    {
        ovphysx_string_t err = ovphysx_get_last_error();
        fprintf(stderr, "ERROR: ovphysx_attach_ovstage failed: %d %.*s\n",
                (int)attach.status, (int)err.length, err.ptr ? err.ptr : "");
        ovphysx_sample_destroy_stage(handle, attachment);
        return 0;
    }

    return 1;
}

#endif // OVPHYSX_SAMPLE_OVSTAGE_SAMPLE_H

// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @cond OVPHYSX_PLC_TRACE */
/**
 * @implements REQ-CAPI-OVD-NAMES-001
 * @covers AC-3
 * @implements REQ-CAPI-OMNIPVD-001
 * @covers AC-1
 * @implements REQ-CAPI-READPOOL-001
 * @covers AC-1
 * @implements REQ-CAPI-PROFILER-001
 * @covers AC-2 AC-8
 */
/** @endcond */

/**
 * @file ovphysx_legacy_config.h
 * @brief Deprecated typed-entry builders retained for source migration.
 */

#ifndef OVPHYSX_LEGACY_CONFIG_H
#define OVPHYSX_LEGACY_CONFIG_H

#include "ovphysx_legacy_types.h"
#include "ovphysx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline ovphysx_config_entry_t ovphysx_config_entry_bool(ovphysx_config_bool_t key, bool value)
{
    ovphysx_config_entry_t entry;
    entry.key_type = OVPHYSX_CONFIG_KEY_TYPE_BOOL;
    entry.key.bool_key = key;
    entry.value.bool_value = value;
    return entry;
}

static inline ovphysx_config_entry_t ovphysx_config_entry_int32(ovphysx_config_int32_t key, int32_t value)
{
    ovphysx_config_entry_t entry;
    entry.key_type = OVPHYSX_CONFIG_KEY_TYPE_INT32;
    entry.key.int32_key = key;
    entry.value.int32_value = value;
    return entry;
}

static inline ovphysx_config_entry_t ovphysx_config_entry_float(ovphysx_config_float_t key, float value)
{
    ovphysx_config_entry_t entry;
    entry.key_type = OVPHYSX_CONFIG_KEY_TYPE_FLOAT;
    entry.key.float_key = key;
    entry.value.float_value = value;
    return entry;
}

static inline ovphysx_config_entry_t ovphysx_config_entry_string(
    ovphysx_config_string_t key,
    ovphysx_string_t value)
{
    ovphysx_config_entry_t entry;
    entry.key_type = OVPHYSX_CONFIG_KEY_TYPE_STRING;
    entry.key.string_key = key;
    entry.value.string_value = value;
    return entry;
}

/**
 * Build a config entry for an arbitrary Carbonite setting.
 *
 * The value string is converted to bool, integer, float, or string by the
 * legacy consumer. Both borrowed strings must remain valid through the call.
 */
static inline ovphysx_config_entry_t ovphysx_config_entry_carbonite(
    ovphysx_string_t key,
    ovphysx_string_t value)
{
    ovphysx_config_entry_t entry;
    entry.key_type = OVPHYSX_CONFIG_KEY_TYPE_CARBONITE;
    entry.key.carbonite_key = key;
    entry.value.string_value = value;
    return entry;
}

static inline ovphysx_config_entry_t ovphysx_config_entry_disable_contact_processing(bool value)
{
    return ovphysx_config_entry_bool(OVPHYSX_CONFIG_DISABLE_CONTACT_PROCESSING, value);
}

static inline ovphysx_config_entry_t ovphysx_config_entry_collision_cone_custom_geometry(bool value)
{
    return ovphysx_config_entry_bool(OVPHYSX_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY, value);
}

static inline ovphysx_config_entry_t ovphysx_config_entry_collision_cylinder_custom_geometry(bool value)
{
    return ovphysx_config_entry_bool(OVPHYSX_CONFIG_COLLISION_CYLINDER_CUSTOM_GEOMETRY, value);
}

/** Set PhysX worker count (/persistent/physics/numThreads) before scene attachment.
 *  The default dispatcher caps it at the shared tasking pool size. 0 runs tasks inline.
 *  This setting does not resize supporting worker pools or an already attached dispatcher. */
static inline ovphysx_config_entry_t ovphysx_config_entry_num_threads(int32_t value)
{
    return ovphysx_config_entry_int32(OVPHYSX_CONFIG_NUM_THREADS, value);
}

static inline ovphysx_config_entry_t ovphysx_config_entry_scene_multi_gpu_mode(int32_t value)
{
    return ovphysx_config_entry_int32(OVPHYSX_CONFIG_SCENE_MULTI_GPU_MODE, value);
}

static inline ovphysx_config_entry_t ovphysx_config_entry_ovstage_read_pool_max_mb(int32_t value)
{
    return ovphysx_config_entry_int32(OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB, value);
}

static inline ovphysx_config_entry_t ovphysx_config_entry_profiler_backend(
    ovphysx_profiler_backend_t value)
{
    return ovphysx_config_entry_int32(
        OVPHYSX_CONFIG_PROFILER_BACKEND,
        (int32_t)value);
}

static inline ovphysx_config_entry_t ovphysx_config_entry_profiler_output_path(
    ovphysx_string_t value)
{
    return ovphysx_config_entry_string(OVPHYSX_CONFIG_PROFILER_OUTPUT_PATH, value);
}

#ifdef __cplusplus
}
#endif

#endif /* OVPHYSX_LEGACY_CONFIG_H */

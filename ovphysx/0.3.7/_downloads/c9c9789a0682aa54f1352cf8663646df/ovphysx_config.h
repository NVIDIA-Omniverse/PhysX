// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

/**
 * @file ovphysx_config.h
 * @brief Builder functions for typed config entries.
 *
 * Provides static inline helpers to construct ovphysx_config_entry_t values
 * for use with ovphysx_create_instance() and ovphysx_set_global_config().
 *
 * Pattern: generic type builders + named convenience functions per enum value.
 * Matches the ovrtx_config.h builder pattern for API consistency.
 */

#ifndef OVPHYSX_CONFIG_H
#define OVPHYSX_CONFIG_H

#include "ovphysx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*--------------------------------------------------*/
/* Generic type builders                            */
/*--------------------------------------------------*/

/** Build a config entry for a boolean setting. */
static inline ovphysx_config_entry_t ovphysx_config_entry_bool(ovphysx_config_bool_t key, bool value)
{
    ovphysx_config_entry_t entry;
    entry.key_type = OVPHYSX_CONFIG_KEY_TYPE_BOOL;
    entry.key.bool_key = key;
    entry.value.bool_value = value;
    return entry;
}

/** Build a config entry for an int32 setting. */
static inline ovphysx_config_entry_t ovphysx_config_entry_int32(ovphysx_config_int32_t key, int32_t value)
{
    ovphysx_config_entry_t entry;
    entry.key_type = OVPHYSX_CONFIG_KEY_TYPE_INT32;
    entry.key.int32_key = key;
    entry.value.int32_value = value;
    return entry;
}

/** Build a config entry for a float setting. */
static inline ovphysx_config_entry_t ovphysx_config_entry_float(ovphysx_config_float_t key, float value)
{
    ovphysx_config_entry_t entry;
    entry.key_type = OVPHYSX_CONFIG_KEY_TYPE_FLOAT;
    entry.key.float_key = key;
    entry.value.float_value = value;
    return entry;
}

/** Build a config entry for a string setting.
 *  value.ptr must remain valid until the API call that consumes the config returns. */
static inline ovphysx_config_entry_t ovphysx_config_entry_string(ovphysx_config_string_t key, ovphysx_string_t value)
{
    ovphysx_config_entry_t entry;
    entry.key_type = OVPHYSX_CONFIG_KEY_TYPE_STRING;
    entry.key.string_key = key;
    entry.value.string_value = value;
    return entry;
}

/**
 * Build a config entry for an arbitrary Carbonite setting (escape hatch).
 *
 * The key is a Carbonite settings path (e.g., "/physics/fabricUpdateVelocities")
 * and the value is a string whose type is auto-detected at runtime:
 * "true"/"false" → bool, integer string → int, float string → float, else string.
 *
 * Both key.ptr and value.ptr must remain valid until the API call returns.
 */
static inline ovphysx_config_entry_t ovphysx_config_entry_carbonite(ovphysx_string_t key, ovphysx_string_t value)
{
    ovphysx_config_entry_t entry;
    entry.key_type = OVPHYSX_CONFIG_KEY_TYPE_CARBONITE;
    entry.key.carbonite_key = key;
    entry.value.string_value = value;
    return entry;
}

/*--------------------------------------------------*/
/* Named convenience builders (one per enum value)  */
/*--------------------------------------------------*/

/** Enable/disable contact processing (/physics/disableContactProcessing). */
static inline ovphysx_config_entry_t ovphysx_config_entry_disable_contact_processing(bool value)
{
    return ovphysx_config_entry_bool(OVPHYSX_CONFIG_DISABLE_CONTACT_PROCESSING, value);
}

/** Enable/disable cone custom geometry for collisions (/physics/collisionConeCustomGeometry). */
static inline ovphysx_config_entry_t ovphysx_config_entry_collision_cone_custom_geometry(bool value)
{
    return ovphysx_config_entry_bool(OVPHYSX_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY, value);
}

/** Enable/disable cylinder custom geometry for collisions (/physics/collisionCylinderCustomGeometry). */
static inline ovphysx_config_entry_t ovphysx_config_entry_collision_cylinder_custom_geometry(bool value)
{
    return ovphysx_config_entry_bool(OVPHYSX_CONFIG_COLLISION_CYLINDER_CUSTOM_GEOMETRY, value);
}

/** Set number of worker threads (/physics/numThreads). 0 = auto. */
static inline ovphysx_config_entry_t ovphysx_config_entry_num_threads(int32_t value)
{
    return ovphysx_config_entry_int32(OVPHYSX_CONFIG_NUM_THREADS, value);
}

/** Set scene multi-GPU mode (/physics/sceneMultiGPUMode). 0=disabled, 1=all GPUs, 2=skip first GPU. */
static inline ovphysx_config_entry_t ovphysx_config_entry_scene_multi_gpu_mode(int32_t value)
{
    return ovphysx_config_entry_int32(OVPHYSX_CONFIG_SCENE_MULTI_GPU_MODE, value);
}

#ifdef __cplusplus
}
#endif

#endif /* OVPHYSX_CONFIG_H */

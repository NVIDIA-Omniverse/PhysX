// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @cond OVPHYSX_PLC_TRACE */
/**
 * @implements REQ-CAPI-OMNIPVD-001
 * @covers AC-1
 * @implements REQ-CAPI-OMNIPVD-LATE-001
 * @covers AC-2
 * @implements REQ-CAPI-READPOOL-001
 * @covers AC-1
 */
/** @endcond */

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

#define OVPHYSX_OMNIPVD_TRANSPORT_FILE_NAME OVPHYSX_LITERAL("file")
#define OVPHYSX_OMNIPVD_TRANSPORT_TCP_NAME OVPHYSX_LITERAL("tcp")

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
 * Build a config entry for an arbitrary Carbonite setting (direct override).
 *
 * The key is a Carbonite settings path (e.g., "/physics/updateToUsd")
 * and the value is a string whose type is auto-detected at runtime:
 * "true"/"false" becomes bool, an integer string int, a float string float, anything else string.
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

/** Set scene multi-GPU mode (/physics/sceneMultiGPUMode).
 *  0=disabled, 1=all GPUs, 2=skip first GPU. Used only when
 *  ovphysx_create_args.active_cuda_gpus is empty. */
static inline ovphysx_config_entry_t ovphysx_config_entry_scene_multi_gpu_mode(int32_t value)
{
    return ovphysx_config_entry_int32(OVPHYSX_CONFIG_SCENE_MULTI_GPU_MODE, value);
}

/** Set the ovstage read device-buffer pool retention budget in MiB
 *  (/physics/ovstageReadPoolMaxMB). Bounds the device and pinned-host memory the
 *  per-context output-read pool retains between reads. 0 or a negative value DISABLES the pool
 *  (nothing is retained, and every read allocates and frees as if the pool were absent). Default 256. */
static inline ovphysx_config_entry_t ovphysx_config_entry_ovstage_read_pool_max_mb(int32_t value)
{
    return ovphysx_config_entry_int32(OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB, value);
}

/** Select the OmniPVD startup transport: exact lowercase "file" or "tcp". */
static inline ovphysx_config_entry_t ovphysx_config_entry_omnipvd_transport(ovphysx_string_t value)
{
    return ovphysx_config_entry_string(OVPHYSX_CONFIG_OMNIPVD_TRANSPORT, value);
}

/** Set the OmniPVD TCP peer address. */
static inline ovphysx_config_entry_t ovphysx_config_entry_omnipvd_tcp_address(ovphysx_string_t value)
{
    return ovphysx_config_entry_string(OVPHYSX_CONFIG_OMNIPVD_TCP_ADDRESS, value);
}

/** Set the OmniPVD TCP peer port (1..65535). */
static inline ovphysx_config_entry_t ovphysx_config_entry_omnipvd_tcp_port(int32_t value)
{
    return ovphysx_config_entry_int32(OVPHYSX_CONFIG_OMNIPVD_TCP_PORT, value);
}

/** Set the blocked-send timeout in milliseconds (0 keeps the OS default and uses a 3000 ms connect window). */
static inline ovphysx_config_entry_t ovphysx_config_entry_omnipvd_tcp_timeout_ms(int32_t value)
{
    return ovphysx_config_entry_int32(OVPHYSX_CONFIG_OMNIPVD_TCP_TIMEOUT_MS, value);
}

/** Set OmniPVD OVD recording directory (/persistent/physics/omniPvdOvdRecordingDirectory).
 *  Both this and omnipvd_output_enabled must be set before instance creation.
 *  When passed together in config_entries, order within the array does not matter
 *  (both are applied before the physics engine reads them).
 *  value.ptr must remain valid until the API call returns. */
static inline ovphysx_config_entry_t ovphysx_config_entry_omnipvd_ovd_recording_directory(ovphysx_string_t value)
{
    return ovphysx_config_entry_string(OVPHYSX_CONFIG_OMNIPVD_OVD_RECORDING_DIRECTORY, value);
}

/** Enable/disable OmniPVD recording (/physics/omniPvdOutputEnabled).
 *  Both this and omnipvd_ovd_recording_directory must be set before instance creation.
 *  When passed together in config_entries, order within the array does not matter. */
static inline ovphysx_config_entry_t ovphysx_config_entry_omnipvd_output_enabled(bool value)
{
    return ovphysx_config_entry_bool(OVPHYSX_CONFIG_OMNIPVD_OUTPUT_ENABLED, value);
}

/** Enable/disable OmniPVD recording capability (/physics/omniPvdRecordingCapable).
 *  This is a creation-time, process-wide setting that defaults to false.
 *  Enabling startup OmniPVD output also enables this capability. */
static inline ovphysx_config_entry_t ovphysx_config_entry_omnipvd_recording_capable(bool value)
{
    return ovphysx_config_entry_bool(OVPHYSX_CONFIG_OMNIPVD_RECORDING_CAPABLE, value);
}

/** Enable/disable NVTX ranges for capture with Nsight Systems (/physics/nvtxEnabled).
 *  Must be set before instance creation. Setting OVPHYSX_NVTX=1 in the environment
 *  has the same effect and needs no code change. */
static inline ovphysx_config_entry_t ovphysx_config_entry_nvtx_enabled(bool value)
{
    return ovphysx_config_entry_bool(OVPHYSX_CONFIG_NVTX_ENABLED, value);
}

#ifdef __cplusplus
}
#endif

#endif /* OVPHYSX_CONFIG_H */

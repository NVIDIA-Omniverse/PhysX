// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @cond OVPHYSX_PLC_TRACE */
/**
 * @implements REQ-CAPI-CONFIG-001
 * @covers AC-2
 */
/** @endcond */

/**
 * @file ovphysx_config.h
 * @brief Builder functions for scoped configuration values.
 *
 * Deprecated entry builders are declared in ovphysx_legacy_config.h.
 */

#ifndef OVPHYSX_CONFIG_H
#define OVPHYSX_CONFIG_H

#include "ovphysx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Build a boolean scoped configuration value. */
static inline ovphysx_config_value_t ovphysx_config_value_bool(bool value)
{
    ovphysx_config_value_t result;
    result.type = OVPHYSX_CONFIG_VALUE_BOOL;
    result.value.bool_value = value;
    return result;
}

/** Build a signed 64-bit scoped configuration value. */
static inline ovphysx_config_value_t ovphysx_config_value_int64(int64_t value)
{
    ovphysx_config_value_t result;
    result.type = OVPHYSX_CONFIG_VALUE_INT64;
    result.value.int64_value = value;
    return result;
}

/** Build a double-precision scoped configuration value. */
static inline ovphysx_config_value_t ovphysx_config_value_double(double value)
{
    ovphysx_config_value_t result;
    result.type = OVPHYSX_CONFIG_VALUE_DOUBLE;
    result.value.double_value = value;
    return result;
}

/** Build a borrowed string scoped configuration value. */
static inline ovphysx_config_value_t ovphysx_config_value_string(ovphysx_string_t value)
{
    ovphysx_config_value_t result;
    result.type = OVPHYSX_CONFIG_VALUE_STRING;
    result.value.string_value = value;
    return result;
}

#ifdef __cplusplus
}
#endif

/* Source-compatibility umbrella. New code does not use this legacy header. */
#include "ovphysx_legacy_config.h"

#endif /* OVPHYSX_CONFIG_H */
